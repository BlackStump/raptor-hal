/*
 * fhan_v2/fh_osd.c -- OSD for the Fullhan backend: one overlay plane
 *
 * THE PLANE
 *
 * The FH885xV200 SDK draws its OSD with libadvapi_osd.so. Of its calls,
 * Divinus uses one on this chip (hal/fhan/f2_hal.c): a "logo", an
 * ARGB1555 picture the size of the main stream, in media-pool memory
 * (FH_SYS_VmmAlloc), handed over with FHAdv_Osd_SetLogo. Every region
 * rod makes is drawn by the CPU into that one plane and the plane is
 * handed over again after each change. The vendor's own app (ipcam)
 * initialises the library the same way, FHAdv_Osd_Init(1, layers, 0, 0);
 * Divinus's layer mask, 1, is the one known to work with the logo.
 *
 * FHAdv_Osd_Init calls into the media pipeline (VPSS), so it works only
 * inside the process that set the pipeline up -- rvd -- and only once the
 * VPSS is running: a separate test program got -1 (bench, 2026-11-21).
 * The plane is therefore set up on the first draw that finds the main
 * stream running; regions made before that are kept and drawn then.
 *
 * GROUPS
 *
 * rvd's OSD groups are its encoder groups (fh_encoder.c collapses
 * FS -> OSD -> ENC into one bind and records which FS feeds each group in
 * st->osd_src_fs). The plane is on VPSS channel 0, the main stream; only
 * regions in a group fed by FS 0 are drawn. Regions of the sub-stream's
 * group are kept and ignored: whatever the logo shows there is the main
 * stream's plane, scaled.
 *
 * PIXELS
 *
 * rod hands BGRA. A pixel is drawn when its alpha is at least half,
 * as 0x8000 | RGB555; the rest of the plane is 0 (transparent). Covers
 * (solid rectangles) are drawn in their colour.
 *
 * Memory: 2 bytes a pixel of the main stream (7.4 MB at 2560x1440) from
 * the media pool, once; each region keeps its own converted copy.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE

#include "fh_state.h"

#include <dlfcn.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#define FH_LIB_ADV "libadvapi.so"
#define FH_LIB_ADV_OSD "libadvapi_osd.so"
#define FOSD_REGIONS 16
#define FOSD_MAIN_FS 0

/* FHAdv_Osd_SetLogo's payload (Divinus f2_osd.h) */
typedef struct {
    unsigned int enable;
    unsigned int flags;
    unsigned int width;
    unsigned int height;
    unsigned int alpha;
    unsigned int rect_x;
    unsigned int rect_y;
    unsigned int rect_w;
    unsigned int rect_h;
    unsigned int bitmap; /* the plane's virtual address */
    unsigned int reserved;
    unsigned int show;
    unsigned int reserved2[4];
} fh_logo_t;

typedef struct {
    bool used;
    rss_osd_type_t type;
    int x, y, w, h;
    int grp; /* -1: not registered */
    bool shown;
    int layer;
    uint32_t cover_color; /* BGRA */
    uint16_t *px;         /* w*h ARGB1555, NULL until a bitmap arrives */
} fosd_region_t;

static pthread_mutex_t fosd_lock = PTHREAD_MUTEX_INITIALIZER;
static fosd_region_t fosd[FOSD_REGIONS];

static struct {
    bool tried, ready;
    void *h_adv, *h_osd;
    int (*fnInit)(void *, unsigned int, unsigned int, unsigned int);
    int (*fnSetLogo)(fh_logo_t *);
    uint16_t *plane;
    unsigned int w, h;
    fh_logo_t logo;
    bool warned_sub;
} fp;

/* ================================================================
 * SETUP
 * ================================================================ */

static int fosd_setup(fh_state_t *st)
{
    int (*alloc)(void *, void *, const char *, const char *, unsigned int);
    struct {
        unsigned int phys, size;
    } out = {0, 0};
    void *virt = NULL;
    int rc;

    if (fp.ready)
        return RSS_OK;
    if (fp.tried)
        return RSS_ERR_IO; /* failed once with the stream up; said so then */

    fp.w = st->vpss_dim[FOSD_MAIN_FS].width;
    fp.h = st->vpss_dim[FOSD_MAIN_FS].height;
    if (!st->vpss_open[FOSD_MAIN_FS] || !fp.w || !fp.h)
        return RSS_ERR_INVAL; /* not yet: the next draw tries again */
    fp.tried = true;

    fp.h_adv = dlopen(FH_LIB_ADV, RTLD_NOW | RTLD_GLOBAL);
    fp.h_osd = fp.h_adv ? dlopen(FH_LIB_ADV_OSD, RTLD_NOW | RTLD_GLOBAL) : NULL;
    if (!fp.h_osd) {
        HAL_LOG_ERR("osd: dlopen %s: %s", fp.h_adv ? FH_LIB_ADV_OSD : FH_LIB_ADV, dlerror());
        return RSS_ERR_NOTSUP;
    }
    fp.fnInit = (int (*)(void *, unsigned int, unsigned int, unsigned int))dlsym(
        fp.h_osd, "FHAdv_Osd_Init");
    fp.fnSetLogo = (int (*)(fh_logo_t *))dlsym(fp.h_osd, "FHAdv_Osd_SetLogo");
    alloc = (int (*)(void *, void *, const char *, const char *, unsigned int))dlsym(
        st->sys.h_vmm, "FH_SYS_VmmAlloc");
    if (!fp.fnInit || !fp.fnSetLogo || !alloc) {
        HAL_LOG_ERR("osd: %s", !alloc ? "libvmm.so has no FH_SYS_VmmAlloc"
                                      : "libadvapi_osd.so lacks FHAdv_Osd_Init/SetLogo");
        return RSS_ERR_NOTSUP;
    }

    rc = fp.fnInit((void *)1, 1, 0, 0);
    if (rc) {
        HAL_LOG_ERR("osd: FHAdv_Osd_Init failed %d", rc);
        return RSS_ERR_IO;
    }
    rc = alloc(&out, &virt, "raptor-osd", "", fp.w * fp.h * 2);
    if (rc || !out.phys || !virt) {
        HAL_LOG_ERR("osd: %u bytes from the media pool for the %ux%u plane: %d", fp.w * fp.h * 2,
                    fp.w, fp.h, rc);
        return RSS_ERR_NOMEM;
    }
    fp.plane = virt;
    memset(fp.plane, 0, (size_t)fp.w * fp.h * 2);

    memset(&fp.logo, 0, sizeof(fp.logo));
    fp.logo.enable = 1;
    fp.logo.width = fp.w;
    fp.logo.height = fp.h;
    fp.logo.alpha = 0xff;
    fp.logo.rect_w = fp.w;
    fp.logo.rect_h = fp.h;
    fp.logo.bitmap = (unsigned int)(uintptr_t)fp.plane;
    fp.logo.show = 1;
    rc = fp.fnSetLogo(&fp.logo);
    if (rc) {
        HAL_LOG_ERR("osd: FHAdv_Osd_SetLogo failed %d", rc);
        return RSS_ERR_IO;
    }
    fp.ready = true;
    HAL_LOG_INFO("osd: %ux%u ARGB1555 plane on the main stream (phys 0x%08x)", fp.w, fp.h,
                 out.phys);
    return RSS_OK;
}

/* ================================================================
 * DRAWING
 * ================================================================ */

static uint16_t bgra_px(uint8_t b, uint8_t g, uint8_t r, uint8_t a)
{
    if (a < 128)
        return 0;
    return (uint16_t)(0x8000 | (r >> 3) << 10 | (g >> 3) << 5 | (b >> 3));
}

/* Is the region drawn on the plane? */
static bool fosd_on_main(fh_state_t *st, const fosd_region_t *r)
{
    if (!r->used || !r->shown || r->grp < 0 || r->grp >= FH_VENC_CHN_MAX)
        return false;
    return st->osd_src_fs[r->grp] == FOSD_MAIN_FS;
}

/* Clip x,y,w,h to the plane; false when nothing is left. */
static bool clip(int *x, int *y, int *w, int *h)
{
    if (*x < 0) {
        *w += *x;
        *x = 0;
    }
    if (*y < 0) {
        *h += *y;
        *y = 0;
    }
    if (*x + *w > (int)fp.w)
        *w = (int)fp.w - *x;
    if (*y + *h > (int)fp.h)
        *h = (int)fp.h - *y;
    return *w > 0 && *h > 0;
}

static void fosd_draw(const fosd_region_t *r, int cx, int cy, int cw, int ch)
{
    /* the part of r inside the clip rectangle */
    int x0 = r->x > cx ? r->x : cx, y0 = r->y > cy ? r->y : cy;
    int x1 = r->x + r->w < cx + cw ? r->x + r->w : cx + cw;
    int y1 = r->y + r->h < cy + ch ? r->y + r->h : cy + ch;
    int w = x1 - x0, h = y1 - y0;

    if (!clip(&x0, &y0, &w, &h))
        return;
    if (r->type == RSS_OSD_COVER) {
        uint32_t c = r->cover_color;
        uint16_t p = bgra_px(c & 0xff, (c >> 8) & 0xff, (c >> 16) & 0xff, 0xff);

        for (int y = y0; y < y0 + h; y++)
            for (int x = x0; x < x0 + w; x++)
                fp.plane[(size_t)y * fp.w + x] = p;
        return;
    }
    if (!r->px)
        return;
    for (int y = y0; y < y0 + h; y++) {
        const uint16_t *s = r->px + (size_t)(y - r->y) * r->w + (x0 - r->x);
        uint16_t *d = fp.plane + (size_t)y * fp.w + x0;

        for (int i = 0; i < w; i++)
            if (s[i])
                d[i] = s[i];
    }
}

/* Clear a rectangle of the plane and draw back every shown region that
 * overlaps it, lowest layer first. Then hand the plane over. */
static void fosd_redraw(fh_state_t *st, int x, int y, int w, int h)
{
    int order[FOSD_REGIONS], n = 0;

    if (!fp.ready && fosd_setup(st))
        return;
    if (!clip(&x, &y, &w, &h))
        return;
    for (int r = y; r < y + h; r++)
        memset(fp.plane + (size_t)r * fp.w + x, 0, (size_t)w * 2);

    for (int i = 0; i < FOSD_REGIONS; i++)
        if (fosd_on_main(st, &fosd[i]))
            order[n++] = i;
    for (int i = 1; i < n; i++) /* by layer, a handful of regions */
        for (int j = i; j > 0 && fosd[order[j]].layer < fosd[order[j - 1]].layer; j--) {
            int t = order[j];
            order[j] = order[j - 1];
            order[j - 1] = t;
        }
    for (int i = 0; i < n; i++)
        fosd_draw(&fosd[order[i]], x, y, w, h);

    if (fp.fnSetLogo(&fp.logo))
        HAL_LOG_WARN("osd: FHAdv_Osd_SetLogo failed");
}

static void fosd_redraw_region(fh_state_t *st, const fosd_region_t *r)
{
    fosd_redraw(st, r->x, r->y, r->w, r->h);
}

static fosd_region_t *fosd_get(int handle)
{
    return handle >= 0 && handle < FOSD_REGIONS && fosd[handle].used ? &fosd[handle] : NULL;
}

static int fosd_convert(fosd_region_t *r, const uint8_t *bgra)
{
    size_t n = (size_t)r->w * r->h;

    if (r->w <= 0 || r->h <= 0)
        return RSS_ERR_INVAL;
    if (!r->px) {
        r->px = malloc(n * 2);
        if (!r->px)
            return RSS_ERR_NOMEM;
    }
    for (size_t i = 0; i < n; i++, bgra += 4)
        r->px[i] = bgra_px(bgra[0], bgra[1], bgra[2], bgra[3]);
    return RSS_OK;
}

/* ================================================================
 * HAL OPS
 * ================================================================ */

/* The plane is one allocation of its own, not a pool */
int hal_osd_set_pool_size(void *ctx, uint32_t bytes)
{
    (void)bytes;
    return ctx ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_create_group(void *ctx, int grp)
{
    (void)grp;
    return fh_state(ctx) ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_destroy_group(void *ctx, int grp)
{
    (void)grp;
    return ctx ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr)
{
    fh_state_t *st = fh_state(ctx);
    int i, rc;

    if (!st || !handle || !attr)
        return RSS_ERR_INVAL;
    if (attr->type != RSS_OSD_PIC && attr->type != RSS_OSD_COVER)
        return RSS_ERR_NOTSUP;
    pthread_mutex_lock(&fosd_lock);
    rc = RSS_OK;
    for (i = 0; i < FOSD_REGIONS && fosd[i].used; i++)
        ;
    if (i == FOSD_REGIONS)
        rc = RSS_ERR_NOMEM;
    if (!rc) {
        fosd_region_t *r = &fosd[i];

        memset(r, 0, sizeof(*r));
        r->used = true;
        r->type = attr->type;
        r->x = attr->x;
        r->y = attr->y;
        r->w = attr->width;
        r->h = attr->height;
        r->cover_color = attr->cover_color;
        r->layer = attr->layer;
        r->grp = -1;
        if (r->type == RSS_OSD_PIC && attr->bitmap_data)
            fosd_convert(r, attr->bitmap_data);
        *handle = i;
    }
    pthread_mutex_unlock(&fosd_lock);
    return rc;
}

int hal_osd_destroy_region(void *ctx, int handle)
{
    fh_state_t *st = fh_state(ctx);
    fosd_region_t *r;

    if (!st)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&fosd_lock);
    r = fosd_get(handle);
    if (r) {
        bool drawn = fosd_on_main(st, r);
        int x = r->x, y = r->y, w = r->w, h = r->h;

        free(r->px);
        memset(r, 0, sizeof(*r));
        if (drawn)
            fosd_redraw(st, x, y, w, h);
    }
    pthread_mutex_unlock(&fosd_lock);
    return r ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_register_region(void *ctx, int handle, int grp)
{
    fh_state_t *st = fh_state(ctx);
    fosd_region_t *r;

    if (!st || grp < 0 || grp >= FH_VENC_CHN_MAX)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&fosd_lock);
    r = fosd_get(handle);
    if (r) {
        r->grp = grp;
        if (st->osd_src_fs[grp] != FOSD_MAIN_FS && !fp.warned_sub) {
            HAL_LOG_INFO("osd: regions of group %d (not the main stream) are not drawn; the "
                         "plane is the main stream's",
                         grp);
            fp.warned_sub = true;
        }
        if (fosd_on_main(st, r))
            fosd_redraw_region(st, r);
    }
    pthread_mutex_unlock(&fosd_lock);
    return r ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_unregister_region(void *ctx, int handle, int grp)
{
    fh_state_t *st = fh_state(ctx);
    fosd_region_t *r;

    (void)grp;
    if (!st)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&fosd_lock);
    r = fosd_get(handle);
    if (r) {
        bool drawn = fosd_on_main(st, r);

        r->grp = -1;
        if (drawn)
            fosd_redraw_region(st, r);
    }
    pthread_mutex_unlock(&fosd_lock);
    return r ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr)
{
    fh_state_t *st = fh_state(ctx);
    fosd_region_t *r;
    int rc = RSS_OK;

    if (!st || !attr)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&fosd_lock);
    r = fosd_get(handle);
    if (!r) {
        pthread_mutex_unlock(&fosd_lock);
        return RSS_ERR_INVAL;
    }
    bool drawn = fosd_on_main(st, r);
    int ox = r->x, oy = r->y, ow = r->w, oh = r->h;

    if (attr->width != r->w || attr->height != r->h) {
        free(r->px);
        r->px = NULL;
    }
    r->x = attr->x;
    r->y = attr->y;
    r->w = attr->width;
    r->h = attr->height;
    r->cover_color = attr->cover_color;
    r->layer = attr->layer;
    if (r->type == RSS_OSD_PIC && attr->bitmap_data)
        rc = fosd_convert(r, attr->bitmap_data);
    if (drawn)
        fosd_redraw(st, ox, oy, ow, oh);
    if (fosd_on_main(st, r))
        fosd_redraw_region(st, r);
    pthread_mutex_unlock(&fosd_lock);
    return rc;
}

int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data)
{
    fh_state_t *st = fh_state(ctx);
    fosd_region_t *r;
    int rc;

    if (!st || !data)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&fosd_lock);
    r = fosd_get(handle);
    if (!r || r->type != RSS_OSD_PIC) {
        pthread_mutex_unlock(&fosd_lock);
        return RSS_ERR_INVAL;
    }
    rc = fosd_convert(r, data);
    if (!rc && fosd_on_main(st, r))
        fosd_redraw_region(st, r);
    pthread_mutex_unlock(&fosd_lock);
    return rc;
}

int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer)
{
    fh_state_t *st = fh_state(ctx);
    fosd_region_t *r;

    (void)grp;
    if (!st)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&fosd_lock);
    r = fosd_get(handle);
    if (r && (!!show != r->shown || layer != r->layer)) {
        bool was = fosd_on_main(st, r);

        r->shown = !!show;
        r->layer = layer;
        if (was || fosd_on_main(st, r))
            fosd_redraw_region(st, r);
    }
    pthread_mutex_unlock(&fosd_lock);
    return r ? RSS_OK : RSS_ERR_INVAL;
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

/* rvd stopping: take the plane off the picture */
void fh_osd_deinit(void)
{
    pthread_mutex_lock(&fosd_lock);
    for (int i = 0; i < FOSD_REGIONS; i++) {
        free(fosd[i].px);
        memset(&fosd[i], 0, sizeof(fosd[i]));
    }
    if (fp.ready) {
        memset(fp.plane, 0, (size_t)fp.w * fp.h * 2);
        fp.logo.enable = 0;
        fp.logo.show = 0;
        fp.fnSetLogo(&fp.logo);
    }
    fp.ready = false;
    fp.tried = false;
    pthread_mutex_unlock(&fosd_lock);
}
