/*
 * xiongmai/xm_osd.c -- OSD on the XM530/XM550 region device (/dev/rgn).
 *
 * VI draws regions over the picture of each of its channels, so a region
 * on VI 0 is in the main stream and its snapshots, one on VI 1 in the sub
 * stream. Per channel, from openxiongmai's open_rgn (drivers/xm530_rgn,
 * whose README is the reference for every number here):
 *   - overlays: up to 8; 1 bit a pixel (transparent, colour 1) or 2
 *     (transparent, colour 1, colour 2), coloured through the channel's
 *     16-entry YUV palette; all of a channel's bitmaps share 8 KiB of OSD
 *     memory (65536 pixels at 1 bit, 32768 at 2);
 *   - covers: up to 32 rectangles of one YUV colour.
 * libmpi's region calls are unfinished (no create, no attributes), so
 * this file speaks the driver's ioctls itself.
 *
 * rvd hands every region a BGRA bitmap. An overlay is filled from it in
 * 2-bit form: transparent where alpha is low, otherwise one of two colour
 * classes split at the middle of the opaque pixels' luma -- the brighter
 * one is the text, the darker its outline, as rod draws them -- each class
 * taking two palette entries of the channel's own (overlay h uses 2h and
 * 2h + 1), loaded with the class's average colour. When a new overlay
 * does not fit, the largest overlays already on the channel drop to 1 bit
 * a pixel one by one (their text stays, their outline goes) until it does;
 * then the new one itself goes to 1 bit; only when nothing more can give
 * way is it refused. On a bench with no passwords, rod's "UNSECURED"
 * banner is the one that gives way, and the timestamp keeps its outline.
 *
 * rvd's groups are its encoder groups: group 0 (main) draws on VI 0 and
 * group 1 (sub) on VI 1, through the binding xm_encoder.c records.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE

#include "xm_state.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/ioctl.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* ---- open_rgn's ABI (records of 32-bit words, copied in only) ---- */
struct xrgn_osd_create {
    uint32_t chn, handle, w, h, format;
};
struct xrgn_osd_attr {
    uint32_t chn, handle, x, y, colormap, effect;
};
struct xrgn_bitmap {
    uint32_t chn, handle, data, size;
};
struct xrgn_show {
    uint32_t chn, handle, on;
};
struct xrgn_cover {
    uint32_t chn, handle, w, h, x, y, color, effect;
};
struct xrgn_palette {
    uint32_t chn, color[16];
};
#define XRGN_RESET _IO('R', 1)
#define XRGN_OSD_CREATE _IOW('R', 2, struct xrgn_osd_create)
#define XRGN_OSD_DESTROY _IOW('R', 3, struct xrgn_osd_create)
#define XRGN_OSD_ATTR _IOW('R', 4, struct xrgn_osd_attr)
#define XRGN_OSD_BITMAP _IOW('R', 5, struct xrgn_bitmap)
#define XRGN_OSD_SHOW _IOW('R', 6, struct xrgn_show)
#define XRGN_COVER_CREATE _IOW('R', 7, struct xrgn_cover)
#define XRGN_COVER_DESTROY _IOW('R', 8, struct xrgn_cover)
#define XRGN_COVER_ATTR _IOW('R', 9, struct xrgn_cover)
#define XRGN_COVER_SHOW _IOW('R', 10, struct xrgn_show)
#define XRGN_PALETTE _IOW('R', 11, struct xrgn_palette)

#define XOSD_CHNS 2       /* VI 0 and 1: the streams */
#define XOSD_OVERLAYS 8   /* a channel */
#define XOSD_COVERS 32    /* a channel */
#define XOSD_REGIONS 48   /* raptor handles */
#define XOSD_POS_MAX 4095 /* overlay x, y, w, h are 12-bit */

typedef struct {
    bool used;
    rss_osd_type_t type;
    int x, y, w, h;
    uint32_t cover_color; /* BGRA */
    int grp, vi;          /* registered to, -1: not yet */
    int hw;               /* overlay or cover handle on vi, -1: none */
    int format;           /* 1: 2 bits a pixel, 0: 1 bit */
    bool shown;
    uint8_t *packed;      /* the last bitmap, in the hardware's form */
    size_t packed_len;
    uint32_t col[2];      /* the two colour classes, 0x00YYUUVV */
} xosd_region_t;

static pthread_mutex_t xosd_lock = PTHREAD_MUTEX_INITIALIZER;
static xosd_region_t xosd[XOSD_REGIONS];
static int rgn_fd = -1;
static bool ov_used[XOSD_CHNS][XOSD_OVERLAYS];
static bool cv_used[XOSD_CHNS][XOSD_COVERS];
static uint32_t palette[XOSD_CHNS][16];
static bool warned_1bit[XOSD_REGIONS];

static int xosd_open(void)
{
    int c, i;

    if (rgn_fd >= 0)
        return RSS_OK;
    rgn_fd = open("/dev/rgn", O_RDWR | O_CLOEXEC);
    if (rgn_fd < 0) {
        HAL_LOG_ERR("osd: /dev/rgn: %s (is open_rgn loaded?)", strerror(errno));
        return RSS_ERR_IO;
    }
    /* a previous rvd that was killed leaves its regions on the picture */
    if (ioctl(rgn_fd, XRGN_RESET))
        HAL_LOG_WARN("osd: region reset: %s", strerror(errno));
    for (c = 0; c < XOSD_CHNS; c++)
        for (i = 0; i < 16; i++)
            palette[c][i] = (uint32_t)(i * 17) << 16 | 0x8080; /* the driver's greys */
    HAL_LOG_INFO("osd: /dev/rgn open, regions cleared");
    return RSS_OK;
}

static uint32_t bgra_to_yuv(uint8_t b, uint8_t g, uint8_t r)
{
    int y = (77 * r + 150 * g + 29 * b + 128) >> 8;
    int u = ((-43 * r - 85 * g + 128 * b + 128) >> 8) + 128;
    int v = ((128 * r - 107 * g - 21 * b + 128) >> 8) + 128;

    y = y < 0 ? 0 : y > 255 ? 255 : y;
    u = u < 0 ? 0 : u > 255 ? 255 : u;
    v = v < 0 ? 0 : v > 255 ? 255 : v;
    return (uint32_t)y << 16 | (uint32_t)u << 8 | (uint32_t)v;
}

static int grp_vi(xm_state_t *st, int grp)
{
    if (grp >= 0 && grp < XM_ENC_CHN_MAX && st->enc[grp].vi >= 0)
        return st->enc[grp].vi;
    return grp >= 0 && grp < XOSD_CHNS ? grp : -1;
}

/*
 * BGRA -> the overlay's form: pixels of 1 << format bits, MSB first, rows
 * unpadded. Two classes of opaque pixels split at the middle of their luma;
 * the brighter is value 1 (the text), the darker 2 (its outline; dropped at
 * 1 bit a pixel). The classes' average colours go to r->col.
 */
static int xosd_pack(xosd_region_t *r, const uint8_t *bgra)
{
    size_t n = (size_t)r->w * r->h, i, bytes;
    int bits = 1 << r->format, lo = 255, hi = 0, mid;
    uint64_t sum[2][3] = {{0}}, cnt[2] = {0};
    uint8_t *out;

    for (i = 0; i < n; i++) {
        const uint8_t *p = bgra + 4 * i;
        int l;

        if (p[3] < 96)
            continue;
        l = (77 * p[2] + 150 * p[1] + 29 * p[0]) >> 8;
        lo = l < lo ? l : lo;
        hi = l > hi ? l : hi;
    }
    mid = (lo + hi + 1) / 2;
    if (hi - lo < 24) /* one colour only: all of it is text */
        mid = lo;

    bytes = (n * bits + 7) / 8;
    if (r->packed_len != bytes) {
        out = realloc(r->packed, bytes);
        if (!out)
            return RSS_ERR_NOMEM;
        r->packed = out;
        r->packed_len = bytes;
    }
    out = r->packed;
    memset(out, 0, bytes);
    for (i = 0; i < n; i++) {
        const uint8_t *p = bgra + 4 * i;
        int l, cls, v;

        if (p[3] < 96)
            continue;
        l = (77 * p[2] + 150 * p[1] + 29 * p[0]) >> 8;
        cls = l >= mid ? 0 : 1;
        sum[cls][0] += p[0];
        sum[cls][1] += p[1];
        sum[cls][2] += p[2];
        cnt[cls]++;
        v = cls == 0 ? 1 : 2;
        if (bits == 1 && v == 2)
            continue;
        out[i * bits / 8] |= (uint8_t)(v << (8 - bits - (int)(i * bits % 8)));
    }
    for (i = 0; i < 2; i++)
        r->col[i] = cnt[i] ? bgra_to_yuv((uint8_t)(sum[i][0] / cnt[i]),
                                         (uint8_t)(sum[i][1] / cnt[i]),
                                         (uint8_t)(sum[i][2] / cnt[i]))
                           : (i == 0 ? 0xff8080u : 0x008080u);
    return RSS_OK;
}

/* the overlay's palette entries (2h text, 2h + 1 outline), loaded if new */
static void xosd_palette(xosd_region_t *r)
{
    struct xrgn_palette p;
    uint32_t *pal = palette[r->vi];

    if (pal[2 * r->hw] == r->col[0] && pal[2 * r->hw + 1] == r->col[1])
        return;
    pal[2 * r->hw] = r->col[0];
    pal[2 * r->hw + 1] = r->col[1];
    memset(&p, 0, sizeof(p));
    p.chn = (uint32_t)r->vi;
    memcpy(p.color, pal, sizeof(p.color));
    if (ioctl(rgn_fd, XRGN_PALETTE, &p))
        HAL_LOG_WARN("osd: palette of VI %d: %s", r->vi, strerror(errno));
}

static int xosd_place(xosd_region_t *r)
{
    if (r->type == RSS_OSD_COVER) {
        struct xrgn_cover c = {(uint32_t)r->vi, (uint32_t)r->hw, (uint32_t)r->w, (uint32_t)r->h,
                               (uint32_t)r->x, (uint32_t)r->y,
                               bgra_to_yuv((uint8_t)r->cover_color, (uint8_t)(r->cover_color >> 8),
                                           (uint8_t)(r->cover_color >> 16)),
                               1};

        return ioctl(rgn_fd, XRGN_COVER_ATTR, &c) ? -errno : 0;
    } else {
        /* colour map: bits 7-4 colour 1 (the text), 11-8 colour 2 (outline) */
        struct xrgn_osd_attr a = {(uint32_t)r->vi, (uint32_t)r->hw, (uint32_t)r->x & ~1u,
                                  (uint32_t)r->y & ~1u,
                                  (uint32_t)(2 * r->hw) << 4 | (uint32_t)(2 * r->hw + 1) << 8, 0};

        return ioctl(rgn_fd, XRGN_OSD_ATTR, &a) ? -errno : 0;
    }
}

static int xosd_load(xosd_region_t *r)
{
    struct xrgn_bitmap b = {(uint32_t)r->vi, (uint32_t)r->hw, (uint32_t)(uintptr_t)r->packed,
                            (uint32_t)r->packed_len};

    if (!r->packed)
        return 0;
    xosd_palette(r);
    return ioctl(rgn_fd, XRGN_OSD_BITMAP, &b) ? -errno : 0;
}

static int xosd_create_overlay(xosd_region_t *r, int h, int format)
{
    struct xrgn_osd_create c = {(uint32_t)r->vi, (uint32_t)h, (uint32_t)r->w, (uint32_t)r->h,
                                (uint32_t)format};

    return ioctl(rgn_fd, XRGN_OSD_CREATE, &c) ? errno : 0;
}

/*
 * Make room on VI vi: the largest overlay there still at 2 bits a pixel
 * goes to 1 bit -- recreated, its bitmap repacked from what it holds (1s
 * stay, 2s go), placed and shown again. False when none is left to give.
 */
static bool xosd_demote_one(int vi, const xosd_region_t *except)
{
    xosd_region_t *best = NULL;
    struct xrgn_osd_create c;
    struct xrgn_show s;
    size_t i, n;
    uint8_t *one;
    bool shown;
    int h;

    for (i = 0; i < XOSD_REGIONS; i++) {
        xosd_region_t *r = &xosd[i];

        if (r->used && r != except && r->type == RSS_OSD_PIC && r->vi == vi && r->hw >= 0 &&
            r->format == 1 && (!best || r->w * r->h > best->w * best->h))
            best = r;
    }
    if (!best)
        return false;

    n = (size_t)best->w * best->h;
    one = calloc(1, (n + 7) / 8);
    if (!one)
        return false;
    for (i = 0; best->packed && i < n; i++)
        if (((best->packed[i / 4] >> (6 - 2 * (i % 4))) & 3) == 1)
            one[i / 8] |= (uint8_t)(0x80 >> (i % 8));

    shown = best->shown;
    h = best->hw;
    s = (struct xrgn_show){(uint32_t)vi, (uint32_t)h, 0};
    ioctl(rgn_fd, XRGN_OSD_SHOW, &s);
    c = (struct xrgn_osd_create){(uint32_t)vi, (uint32_t)h, 0, 0, 0};
    ioctl(rgn_fd, XRGN_OSD_DESTROY, &c);
    if (xosd_create_overlay(best, h, 0)) {
        /* it fitted at 2 bits a moment ago, so 1 bit fits: not expected */
        HAL_LOG_ERR("osd: %dx%d on VI %d lost making room: %s", best->w, best->h, vi,
                    strerror(errno));
        ov_used[vi][h] = false;
        best->hw = -1;
        best->shown = false;
        free(one);
        return true;
    }
    best->format = 0;
    free(best->packed);
    best->packed = one;
    best->packed_len = (n + 7) / 8;
    xosd_place(best);
    xosd_load(best);
    if (shown) {
        s.on = 1;
        best->shown = !ioctl(rgn_fd, XRGN_OSD_SHOW, &s);
    }
    HAL_LOG_INFO("osd: %dx%d on VI %d goes to 1 bit a pixel (no outline) to make room", best->w,
                 best->h, vi);
    return true;
}

/* Make the region in the hardware of its channel, 2 bits a pixel if room */
static int xosd_hw_create(xosd_region_t *r, int idx)
{
    int h, e;

    if (r->type == RSS_OSD_COVER) {
        struct xrgn_cover c;

        for (h = 0; h < XOSD_COVERS && cv_used[r->vi][h]; h++)
            ;
        if (h == XOSD_COVERS)
            return RSS_ERR_NOMEM;
        memset(&c, 0, sizeof(c));
        c.chn = (uint32_t)r->vi;
        c.handle = (uint32_t)h;
        c.w = (uint32_t)r->w;
        c.h = (uint32_t)r->h;
        c.x = (uint32_t)r->x;
        c.y = (uint32_t)r->y;
        c.effect = 1;
        if (ioctl(rgn_fd, XRGN_COVER_CREATE, &c)) {
            HAL_LOG_ERR("osd: cover on VI %d: %s", r->vi, strerror(errno));
            return RSS_ERR_IO;
        }
        cv_used[r->vi][h] = true;
        r->hw = h;
        return xosd_place(r) ? RSS_ERR_IO : RSS_OK;
    }

    if (r->w < 2 || r->h < 2 || r->w > XOSD_POS_MAX || r->h > XOSD_POS_MAX)
        return RSS_ERR_INVAL;
    for (h = 0; h < XOSD_OVERLAYS && ov_used[r->vi][h]; h++)
        ;
    if (h == XOSD_OVERLAYS) {
        HAL_LOG_WARN("osd: VI %d has all 8 overlays in use", r->vi);
        return RSS_ERR_NOMEM;
    }
    /* 2 bits a pixel, making room from the others if need be; then 1 */
    e = xosd_create_overlay(r, h, 1);
    while (e == ENOSPC && xosd_demote_one(r->vi, r))
        e = xosd_create_overlay(r, h, 1);
    r->format = 1;
    if (e == ENOSPC) {
        r->format = 0;
        e = xosd_create_overlay(r, h, 0);
    }
    if (e) {
        HAL_LOG_ERR("osd: %dx%d overlay on VI %d: %s%s", r->w, r->h, r->vi, strerror(e),
                    e == ENOSPC ? " -- the channel's 8 KiB of OSD memory is full even at 1 bit "
                                  "a pixel; a smaller font or fewer elements on this stream "
                                  "will fit"
                                : "");
        return e == ENOSPC ? RSS_ERR_NOMEM : RSS_ERR_IO;
    }
    if (r->format == 0 && !warned_1bit[idx]) {
        warned_1bit[idx] = true;
        HAL_LOG_WARN("osd: %dx%d on VI %d fits only at 1 bit a pixel: text without its "
                     "outline",
                     r->w, r->h, r->vi);
    }
    ov_used[r->vi][h] = true;
    r->hw = h;
    if (xosd_place(r)) {
        HAL_LOG_WARN("osd: placing %dx%d at %d,%d on VI %d: %s", r->w, r->h, r->x, r->y, r->vi,
                     strerror(errno));
    }
    return RSS_OK;
}

static void xosd_hw_destroy(xosd_region_t *r)
{
    if (r->vi < 0 || r->hw < 0)
        return;
    if (r->type == RSS_OSD_COVER) {
        struct xrgn_cover c = {(uint32_t)r->vi, (uint32_t)r->hw, 0, 0, 0, 0, 0, 0};
        struct xrgn_show s = {(uint32_t)r->vi, (uint32_t)r->hw, 0};

        ioctl(rgn_fd, XRGN_COVER_SHOW, &s);
        ioctl(rgn_fd, XRGN_COVER_DESTROY, &c);
        cv_used[r->vi][r->hw] = false;
    } else {
        struct xrgn_osd_create c = {(uint32_t)r->vi, (uint32_t)r->hw, 0, 0, 0};
        struct xrgn_show s = {(uint32_t)r->vi, (uint32_t)r->hw, 0};

        ioctl(rgn_fd, XRGN_OSD_SHOW, &s);
        ioctl(rgn_fd, XRGN_OSD_DESTROY, &c);
        ov_used[r->vi][r->hw] = false;
    }
    r->hw = -1;
    r->shown = false;
}

static xosd_region_t *xosd_get(int handle)
{
    return handle >= 0 && handle < XOSD_REGIONS && xosd[handle].used ? &xosd[handle] : NULL;
}

/* ================================================================
 * HAL OPS
 * ================================================================ */

/* The bitmaps live in VI's OSD memory, not a pool of ours */
int hal_osd_set_pool_size(void *ctx, uint32_t bytes)
{
    (void)bytes;
    return ctx ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_create_group(void *ctx, int grp)
{
    xm_state_t *st = xm_state(ctx);

    (void)grp;
    if (!st)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&xosd_lock);
    int rc = xosd_open();
    pthread_mutex_unlock(&xosd_lock);
    return rc;
}

int hal_osd_destroy_group(void *ctx, int grp)
{
    (void)grp;
    return ctx ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr)
{
    int i, rc;

    if (!ctx || !handle || !attr)
        return RSS_ERR_INVAL;
    if (attr->type != RSS_OSD_PIC && attr->type != RSS_OSD_COVER)
        return RSS_ERR_NOTSUP;
    pthread_mutex_lock(&xosd_lock);
    rc = xosd_open();
    for (i = 0; !rc && i < XOSD_REGIONS && xosd[i].used; i++)
        ;
    if (!rc && i == XOSD_REGIONS)
        rc = RSS_ERR_NOMEM;
    if (!rc) {
        xosd_region_t *r = &xosd[i];

        free(r->packed);
        memset(r, 0, sizeof(*r));
        r->used = true;
        r->type = attr->type;
        r->x = attr->x;
        r->y = attr->y;
        r->w = attr->width;
        r->h = attr->height;
        r->cover_color = attr->cover_color;
        r->grp = r->vi = r->hw = -1;
        warned_1bit[i] = false;
        *handle = i;
    }
    pthread_mutex_unlock(&xosd_lock);
    return rc;
}

int hal_osd_destroy_region(void *ctx, int handle)
{
    xosd_region_t *r;

    if (!ctx)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&xosd_lock);
    r = xosd_get(handle);
    if (r) {
        xosd_hw_destroy(r);
        free(r->packed);
        memset(r, 0, sizeof(*r));
    }
    pthread_mutex_unlock(&xosd_lock);
    return r ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_register_region(void *ctx, int handle, int grp)
{
    xm_state_t *st = xm_state(ctx);
    xosd_region_t *r;
    int vi, rc;

    if (!st)
        return RSS_ERR_INVAL;
    vi = grp_vi(st, grp);
    if (vi < 0 || vi >= XOSD_CHNS)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&xosd_lock);
    r = xosd_get(handle);
    if (!r) {
        pthread_mutex_unlock(&xosd_lock);
        return RSS_ERR_INVAL;
    }
    if (r->vi >= 0 && r->vi != vi)
        xosd_hw_destroy(r);
    r->grp = grp;
    r->vi = vi;
    rc = r->hw < 0 ? xosd_hw_create(r, handle) : RSS_OK;
    pthread_mutex_unlock(&xosd_lock);
    return rc;
}

int hal_osd_unregister_region(void *ctx, int handle, int grp)
{
    xosd_region_t *r;

    (void)grp;
    if (!ctx)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&xosd_lock);
    r = xosd_get(handle);
    if (r) {
        xosd_hw_destroy(r);
        r->grp = r->vi = -1;
    }
    pthread_mutex_unlock(&xosd_lock);
    return r ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr)
{
    xosd_region_t *r;
    int rc = RSS_OK;

    if (!ctx || !attr)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&xosd_lock);
    r = xosd_get(handle);
    if (!r) {
        pthread_mutex_unlock(&xosd_lock);
        return RSS_ERR_INVAL;
    }
    if (r->hw >= 0 && (attr->width != r->w || attr->height != r->h)) {
        bool shown = r->shown;

        /* a new size is a new overlay in VI's memory */
        xosd_hw_destroy(r);
        r->w = attr->width;
        r->h = attr->height;
        free(r->packed);
        r->packed = NULL;
        r->packed_len = 0;
        r->x = attr->x;
        r->y = attr->y;
        rc = xosd_hw_create(r, handle);
        if (!rc && shown) {
            struct xrgn_show s = {(uint32_t)r->vi, (uint32_t)r->hw, 1};

            r->shown = !ioctl(rgn_fd, r->type == RSS_OSD_COVER ? XRGN_COVER_SHOW : XRGN_OSD_SHOW,
                              &s);
        }
    } else {
        r->w = attr->width;
        r->h = attr->height;
        r->x = attr->x;
        r->y = attr->y;
        r->cover_color = attr->cover_color;
        if (r->hw >= 0 && xosd_place(r))
            rc = RSS_ERR_IO;
    }
    if (!rc && r->type == RSS_OSD_PIC && attr->bitmap_data && r->hw >= 0 &&
        !xosd_pack(r, attr->bitmap_data) && xosd_load(r))
        rc = RSS_ERR_IO;
    pthread_mutex_unlock(&xosd_lock);
    return rc;
}

int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data)
{
    xosd_region_t *r;
    int rc = RSS_OK, e;

    if (!ctx || !data)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&xosd_lock);
    r = xosd_get(handle);
    if (!r || r->type != RSS_OSD_PIC) {
        pthread_mutex_unlock(&xosd_lock);
        return RSS_ERR_INVAL;
    }
    if (r->hw >= 0) {
        rc = xosd_pack(r, data);
        if (!rc && (e = xosd_load(r))) {
            HAL_LOG_WARN("osd: bitmap of overlay %d on VI %d: %s", r->hw, r->vi, strerror(-e));
            rc = RSS_ERR_IO;
        }
    }
    pthread_mutex_unlock(&xosd_lock);
    return rc;
}

int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer)
{
    xosd_region_t *r;
    struct xrgn_show s;
    int rc = RSS_OK;

    (void)grp;
    (void)layer; /* the overlays have no order of their own */
    if (!ctx)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&xosd_lock);
    r = xosd_get(handle);
    if (!r || r->hw < 0) {
        pthread_mutex_unlock(&xosd_lock);
        return r ? RSS_OK : RSS_ERR_INVAL;
    }
    if (!!show != r->shown) {
        s.chn = (uint32_t)r->vi;
        s.handle = (uint32_t)r->hw;
        s.on = show ? 1 : 0;
        if (ioctl(rgn_fd, r->type == RSS_OSD_COVER ? XRGN_COVER_SHOW : XRGN_OSD_SHOW, &s))
            rc = RSS_ERR_IO;
        else
            r->shown = !!show;
    }
    pthread_mutex_unlock(&xosd_lock);
    return rc;
}

int hal_osd_start(void *ctx, int grp)
{
    (void)grp;
    return ctx ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_stop(void *ctx, int grp)
{
    (void)grp;
    return ctx ? RSS_OK : RSS_ERR_INVAL;
}

void xm_osd_deinit(void)
{
    int i;

    pthread_mutex_lock(&xosd_lock);
    for (i = 0; i < XOSD_REGIONS; i++)
        if (xosd[i].used) {
            xosd_hw_destroy(&xosd[i]);
            free(xosd[i].packed);
            memset(&xosd[i], 0, sizeof(xosd[i]));
        }
    if (rgn_fd >= 0) {
        close(rgn_fd);
        rgn_fd = -1;
    }
    pthread_mutex_unlock(&xosd_lock);
}
