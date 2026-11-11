/*
 * xiongmai/xm_pipeline.c -- the Xiongmai video pipeline: sensor, ISP, VI.
 *
 * Phase 2a. The bring-up is xmhost_test's, proved on M1 (2026-11-11) with
 * no majestic: libxmhost's xmhost_start (stop the 3D NR a previous program
 * left running, quiet VI channels 1 and 0, then XM_MPI_SYS_Init), then
 * libispapp's ispApp in majestic's arguments (mode 0, PAL, resolution 0),
 * which starts the sensor and the ISP and returns with them running. The
 * open AE and AWB in libisp_XM530 do the 3A (LIBISP_OPEN_AE=1,
 * LIBISP_OPEN_AWB=1 in the environment; S96raptor sets them).
 *
 * Framesources. VI channel 0 has no scaler (re_comm_vi.h, and majestic's
 * 1920x1080 from a 2600x1456 window dropped a quarter of the rows), so fs 0
 * is the sensor window at its own size, or a configured crop at its own
 * size, and nothing else. fs 1 is VI extended channel 1, which the kernel
 * scales from channel 0 to any size -- the sub-stream.
 *
 * Frames come from XM_MPI_VI_GetFrame, which gives physical addresses only;
 * they are mapped through /dev/mem for the time the caller holds them.
 *
 * What libmpi does not pass on: VI_CHN_ATTR_S's pixel format, mirror, flip
 * and rates. Channel 0 is NV12 (semi-planar 420) as the test showed.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE /* usleep, clock_gettime under -std=c11 */

#include "xm_state.h"

#include <re_sys.h>
#include <re_vi.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

/* libxmhost and libispapp_XM530, linked by raptor's Makefile */
int xmhost_start(void);
int ispApp(int argc, char *argv[]);

_Static_assert(sizeof(VIDEO_FRAME_INFO_S) <= sizeof(((xm_state_t *)0)->grab[0].frame),
               "grab frame slot too small for VIDEO_FRAME_INFO_S");

/* ================================================================
 * SENSORS
 * ================================================================ */

/*
 * SC5332 on the IVG-83X50PSA-B: 2600x1952 out of the sensor at 15 fps
 * (HTS 1500 x VTS 3300, openxiongmai 6148077). The ISP's clean picture in
 * that frame, measured on M1 (2026-11-11) from captures at 4,4 and 0,0:
 * columns 4-2592 and rows 0-1944. Outside it the edge filtering leaves
 * junk -- a halo in columns 0-3 on bright scenes, flat rows from 1945 (one
 * of them solid white) and pink columns from 2593 -- so 2592x1944 does not
 * fit anywhere. 2576x1936 at 8,4 does, with a margin on every side, and is
 * a multiple of 16 both ways for the encoder: 99% of the sensor, unscaled.
 * The first row is the default for an unknown name -- the only sensor this
 * backend has met.
 */
static const xm_sensor_mode_t xm_modes[] = {
    {"sc5332", 2576, 1936, 8, 4, 15},
};

static const xm_sensor_mode_t *xm_find_mode(const char *name)
{
    size_t i;

    for (i = 0; name && i < sizeof(xm_modes) / sizeof(xm_modes[0]); i++)
        if (strcasecmp(name, xm_modes[i].name) == 0)
            return &xm_modes[i];
    HAL_LOG_WARN("sensor: '%s' is not one this backend knows; framing it as %s", name ? name : "",
                 xm_modes[0].name);
    return &xm_modes[0];
}

static int64_t xm_now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* ================================================================
 * START / STOP
 * ================================================================ */

static void xm_selftest(xm_state_t *st);

int xm_pipe_start(xm_state_t *st)
{
    /* majestic's start: mode 0 (the application owns VI), PAL timing,
     * resolution 0, the sensor's own interface */
    char a0[] = "ispApp", a1[] = "-m", a2[] = "0", a3[] = "-v", a4[] = "p", a5[] = "-r",
         a6[] = "0";
    char *isp_argv[] = {a0, a1, a2, a3, a4, a5, a6, NULL};
    const char *ae = getenv("LIBISP_OPEN_AE"), *awb = getenv("LIBISP_OPEN_AWB");
    int ret;

    st->mem_fd = -1;
    st->mode = xm_find_mode(st->sensor_name);
    memset(st->fs_applied, 0, sizeof(st->fs_applied));
    xm_enc_init(st);

    if (!ae || strcmp(ae, "1") || !awb || strcmp(awb, "1"))
        HAL_LOG_WARN("isp: LIBISP_OPEN_AE=%s LIBISP_OPEN_AWB=%s; without both at 1 nothing runs "
                     "the 3A (majestic did) and the picture goes black or green",
                     ae ? ae : "(unset)", awb ? awb : "(unset)");

    ret = xmhost_start();
    if (ret) {
        HAL_LOG_ERR("sys: xmhost_start failed %d (0x%x)", ret, (unsigned)ret);
        return RSS_ERR_IO;
    }
    st->sys_inited = true;
    HAL_LOG_INFO("sys: 3D NR stopped, VI 1 and 0 quiet, XM_MPI_SYS_Init done");

    /* the AE reads its attributes from the start, so they go in first */
    xm_isp_exposure_defaults();

    ret = ispApp(7, isp_argv);
    if (ret)
        HAL_LOG_WARN("isp: ispApp returned %d (going on, as xmhost_test does)", ret);
    st->isp_started = true;
    HAL_LOG_INFO("isp: %s up, %ux%u at %u fps from channel 0", st->mode->name, st->mode->width,
                 st->mode->height, st->mode->fps);

    if (getenv("XM_FS_SELFTEST") && strcmp(getenv("XM_FS_SELFTEST"), "0"))
        xm_selftest(st);
    return RSS_OK;
}

static void xm_grab_drop(xm_state_t *st, int chn);

void xm_pipe_stop(xm_state_t *st)
{
    int chn;

    if (!st)
        return;
    xm_enc_deinit(st); /* feeders first: they hold VI frames */
    for (chn = XM_FS_CHN_MAX - 1; chn >= 0; chn--) {
        xm_grab_drop(st, chn);
        if (st->fs[chn].enabled)
            XM_MPI_VI_DisableChn(chn);
        st->fs[chn].enabled = false;
        free(st->grab[chn].copy);
        st->grab[chn].copy = NULL;
        st->grab[chn].copy_cap = 0;
    }
    if (st->mem_fd >= 0)
        close(st->mem_fd);
    st->mem_fd = -1;
    /* The ISP has no exit that leaves the hardware still: the next start's
     * xmhost_start quiets it, as it does after majestic. */
}

/* ================================================================
 * SENSOR ATTRIBUTES
 * ================================================================ */

int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height)
{
    xm_state_t *st = xm_state(ctx);

    if (!width || !height)
        return RSS_ERR_INVAL;
    if (!st || !st->mode)
        return RSS_ERR_NOTSUP;
    *width = st->mode->width;
    *height = st->mode->height;
    return RSS_OK;
}

int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den)
{
    xm_state_t *st = xm_state(ctx);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;
    if (!st || !st->mode)
        return RSS_ERR_NOTSUP;
    *fps_num = st->mode->fps;
    *fps_den = 1;
    return RSS_OK;
}

/* The rate is the sensor library's timing (VTS 3300 at 15 fps); asking for
 * it is fine, anything else is not something this backend can do. */
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den)
{
    xm_state_t *st = xm_state(ctx);

    if (!st || !st->mode || !fps_den)
        return RSS_ERR_NOTSUP;
    if (fps_num / fps_den == st->mode->fps)
        return RSS_OK;
    HAL_LOG_WARN("sensor fps %u/%u: the %s runs at %u fps, set by its library's timing", fps_num,
                 fps_den, st->mode->name, st->mode->fps);
    return RSS_ERR_NOTSUP;
}

/* ================================================================
 * FRAMESOURCES
 * ================================================================ */

static int xm_fs_set(xm_state_t *st, int chn)
{
    xm_fs_chn_t *f = &st->fs[chn];

    /* VI takes about 3 s over a channel 0 attribute set, even the same
     * one again (rvd's own create after the self-test, a restarted
     * stream): set only what changed */
    if (st->fs_applied[chn].valid && st->fs_applied[chn].w == f->width &&
        st->fs_applied[chn].h == f->height && st->fs_applied[chn].x == f->cap_x &&
        st->fs_applied[chn].y == f->cap_y)
        return RSS_OK;

    if (chn == 0) {
        VI_CHN_ATTR_S vi;

        memset(&vi, 0, sizeof(vi));
        vi.stCapRect.s32X = (XM_S32)f->cap_x;
        vi.stCapRect.s32Y = (XM_S32)f->cap_y;
        vi.stCapRect.u32Width = f->width;
        vi.stCapRect.u32Height = f->height;
        vi.stDestSize.u32Width = f->width;
        vi.stDestSize.u32Height = f->height;
        vi.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        vi.s32SrcFrameRate = -1;
        vi.s32FrameRate = -1;
        if (XM_MPI_VI_SetChnAttr(0, &vi) != XM_SUCCESS) {
            HAL_LOG_ERR("fs 0: XM_MPI_VI_SetChnAttr failed");
            return RSS_ERR_IO;
        }
    } else {
        VI_EXT_CHN_ATTR_S ext;

        memset(&ext, 0, sizeof(ext));
        ext.s32BindChn = 0;
        ext.stDestSize.u32Width = f->width;
        ext.stDestSize.u32Height = f->height;
        ext.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        ext.s32SrcFrameRate = -1;
        ext.s32FrameRate = -1;
        if (XM_MPI_VI_SetExtChnAttr(chn, &ext) != XM_SUCCESS) {
            HAL_LOG_ERR("fs %d: XM_MPI_VI_SetExtChnAttr failed", chn);
            return RSS_ERR_IO;
        }
    }
    st->fs_applied[chn].valid = true;
    st->fs_applied[chn].w = f->width;
    st->fs_applied[chn].h = f->height;
    st->fs_applied[chn].x = f->cap_x;
    st->fs_applied[chn].y = f->cap_y;
    return RSS_OK;
}

int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    xm_state_t *st = xm_state(ctx);
    const xm_sensor_mode_t *m;
    xm_fs_chn_t f;
    int ret;

    if (!st || !cfg || chn < 0 || chn >= XM_FS_CHN_MAX)
        return RSS_ERR_INVAL;
    if (!st->isp_started || !(m = st->mode))
        return RSS_ERR_BUSY;
    if (cfg->pixfmt != RSS_PIXFMT_NV12)
        HAL_LOG_WARN("fs %d: pixel format %d asked; VI gives NV12", chn, (int)cfg->pixfmt);

    memset(&f, 0, sizeof(f));
    if (chn == 0) {
        /* No scaler on channel 0: the sensor window, or a crop at its own
         * size. Anything else would be the row-dropping majestic did. */
        if (cfg->crop.enable && cfg->crop.w > 0 && cfg->crop.h > 0) {
            if ((uint32_t)(cfg->crop.x + cfg->crop.w) > m->width ||
                (uint32_t)(cfg->crop.y + cfg->crop.h) > m->height || cfg->crop.x < 0 ||
                cfg->crop.y < 0) {
                HAL_LOG_ERR("fs 0: crop %dx%d+%d+%d is outside the %ux%u sensor window",
                            cfg->crop.w, cfg->crop.h, cfg->crop.x, cfg->crop.y, m->width,
                            m->height);
                return RSS_ERR_INVAL;
            }
            f.cap_x = m->cap_x + (uint32_t)cfg->crop.x;
            f.cap_y = m->cap_y + (uint32_t)cfg->crop.y;
            f.width = (uint32_t)cfg->crop.w;
            f.height = (uint32_t)cfg->crop.h;
        } else {
            f.cap_x = m->cap_x;
            f.cap_y = m->cap_y;
            f.width = m->width;
            f.height = m->height;
        }
        if (cfg->width != f.width || cfg->height != f.height) {
            HAL_LOG_ERR("fs 0: %ux%u asked, but VI channel 0 cannot scale -- set stream0 to "
                        "%ux%u (or crop to the size wanted); smaller sizes belong on stream1",
                        cfg->width, cfg->height, f.width, f.height);
            return RSS_ERR_INVAL;
        }
    } else {
        if (cfg->width < 64 || cfg->height < 64 || cfg->width > m->width ||
            cfg->height > m->height || (cfg->width & 1) || (cfg->height & 1)) {
            HAL_LOG_ERR("fs %d: %ux%u is not a size channel %d can scale %ux%u to", chn,
                        cfg->width, cfg->height, chn, m->width, m->height);
            return RSS_ERR_INVAL;
        }
        /* more than 1% off the sensor's shape; rvd's own 640x480 for
         * 2576x1936 is 0.2% off and fine */
        long d = (long)cfg->width * m->height - (long)cfg->height * m->width;

        if ((d < 0 ? -d : d) * 100 > (long)cfg->height * m->width) {
            uint32_t a = m->width, b = m->height;

            while (b) {
                uint32_t t = a % b;

                a = b;
                b = t;
            }
            HAL_LOG_WARN("fs %d: %ux%u is not the sensor's %u:%u shape; the kernel stretches "
                         "the whole picture to it",
                         chn, cfg->width, cfg->height, m->width / a, m->height / a);
        }
        f.width = cfg->width;
        f.height = cfg->height;
    }

    if (st->fs[chn].enabled)
        hal_fs_disable_channel(ctx, chn);
    st->fs[chn] = f;
    ret = xm_fs_set(st, chn);
    if (ret)
        return ret;
    st->fs[chn].created = true;
    HAL_LOG_INFO("fs %d: VI %s %d, %ux%u%s", chn, chn ? "extended channel" : "channel", chn,
                 f.width, f.height,
                 chn == 0 ? (f.cap_x == m->cap_x && f.cap_y == m->cap_y ? " (sensor window)"
                                                                          : " (crop)")
                          : " (scaled from channel 0)");
    return RSS_OK;
}

int hal_fs_destroy_channel(void *ctx, int chn)
{
    xm_state_t *st = xm_state(ctx);

    if (!st || chn < 0 || chn >= XM_FS_CHN_MAX)
        return RSS_ERR_INVAL;
    hal_fs_disable_channel(ctx, chn);
    st->fs[chn].created = false;
    return RSS_OK;
}

int hal_fs_enable_channel(void *ctx, int chn)
{
    xm_state_t *st = xm_state(ctx);

    if (!st || chn < 0 || chn >= XM_FS_CHN_MAX || !st->fs[chn].created)
        return RSS_ERR_INVAL;
    if (st->fs[chn].enabled)
        return RSS_OK;
    /* an extended channel scales from channel 0, so 0 runs first */
    if (chn > 0 && !st->fs[0].enabled) {
        if (!st->fs[0].created) {
            HAL_LOG_ERR("fs %d: needs fs 0 (VI channel 0) created first", chn);
            return RSS_ERR_INVAL;
        }
        if (hal_fs_enable_channel(ctx, 0))
            return RSS_ERR_IO;
    }
    if (XM_MPI_VI_EnableChn(chn) != XM_SUCCESS) {
        HAL_LOG_ERR("fs %d: XM_MPI_VI_EnableChn failed", chn);
        return RSS_ERR_IO;
    }
    st->fs[chn].enabled = true;
    HAL_LOG_INFO("fs %d: enabled", chn);
    return RSS_OK;
}

int hal_fs_disable_channel(void *ctx, int chn)
{
    xm_state_t *st = xm_state(ctx);
    int c;

    if (!st || chn < 0 || chn >= XM_FS_CHN_MAX)
        return RSS_ERR_INVAL;
    if (!st->fs[chn].enabled)
        return RSS_OK;
    /* channel 0 takes its extended channels with it, as xmhost_start
     * stops them: highest first */
    for (c = XM_FS_CHN_MAX - 1; c >= chn; c--) {
        if (!st->fs[c].enabled || (c != chn && chn != 0))
            continue;
        xm_grab_drop(st, c);
        /* not under a frame the feeder holds */
        if (st->enc_ready)
            pthread_mutex_lock(&st->feed[c].lock);
        XM_MPI_VI_DisableChn(c);
        if (st->enc_ready)
            pthread_mutex_unlock(&st->feed[c].lock);
        st->fs[c].enabled = false;
    }
    return RSS_OK;
}

/* VI keeps its own buffers; there is no depth or fifo to set. */
int hal_fs_set_frame_depth(void *ctx, int chn, int depth)
{
    (void)depth;
    return xm_state(ctx) && chn >= 0 && chn < XM_FS_CHN_MAX ? RSS_OK : RSS_ERR_INVAL;
}

int hal_fs_set_fifo(void *ctx, int chn, int depth)
{
    (void)depth;
    return xm_state(ctx) && chn >= 0 && chn < XM_FS_CHN_MAX ? RSS_OK : RSS_ERR_INVAL;
}

/* ================================================================
 * RAW FRAMES
 * ================================================================ */

static void xm_grab_drop(xm_state_t *st, int chn)
{
    if (st->grab[chn].from_feeder) {
        st->grab[chn].from_feeder = false; /* a copy; no VI frame behind it */
        return;
    }
    if (!st->grab[chn].held)
        return;
    if (st->grab[chn].map)
        munmap(st->grab[chn].map, st->grab[chn].map_len);
    st->grab[chn].map = NULL;
    st->grab[chn].map_len = 0;
    XM_MPI_VI_ReleaseFrame(chn, (VIDEO_FRAME_INFO_S *)st->grab[chn].frame);
    st->grab[chn].held = false;
}

static int xm_mem_open(xm_state_t *st)
{
    if (st->mem_fd >= 0)
        return RSS_OK;
    st->mem_fd = open("/dev/mem", O_RDONLY | O_SYNC | O_CLOEXEC);
    if (st->mem_fd < 0) {
        HAL_LOG_ERR("fs: /dev/mem: %s", strerror(errno));
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

/*
 * Map a VI frame's planes: one mapping over both, which VI puts close
 * together, returned in map and map_len; y and c point at the planes in
 * it. The caller unmaps.
 */
static int xm_frame_map(xm_state_t *st, int chn, const VIDEO_FRAME_INFO_S *vf, uint8_t **map,
                        size_t *map_len, const uint8_t **y, const uint8_t **c)
{
    XM_U32 w = vf->stVFrame.u32Width, h = vf->stVFrame.u32Height;
    XM_U32 yphys = vf->stVFrame.u32PhyAddr[0], cphys = vf->stVFrame.u32PhyAddr[1], base;
    size_t ysize = (size_t)w * h, span;
    long page = sysconf(_SC_PAGESIZE);
    uint8_t *m;

    if (xm_mem_open(st))
        return RSS_ERR_IO;
    if (!yphys || !cphys || cphys < yphys || cphys - yphys > ysize + (XM_U32)page) {
        HAL_LOG_WARN("fs %d: chroma %08x is not near luma %08x; not mapped", chn, cphys, yphys);
        return RSS_ERR_IO;
    }
    base = yphys & ~(XM_U32)(page - 1);
    span = (size_t)(cphys - base) + ysize / 2;
    m = mmap(NULL, span, PROT_READ, MAP_SHARED, st->mem_fd, (off_t)base);
    if (m == MAP_FAILED) {
        HAL_LOG_ERR("fs %d: mmap %08x+%zu: %s", chn, base, span, strerror(errno));
        return RSS_ERR_IO;
    }
    *map = m;
    *map_len = span;
    *y = m + (yphys - base);
    *c = m + (cphys - base);
    return RSS_OK;
}

static int xm_copy_room(xm_state_t *st, int chn, size_t need)
{
    uint8_t *b;

    if (st->grab[chn].copy_cap >= need)
        return RSS_OK;
    b = realloc(st->grab[chn].copy, need);
    if (!b)
        return RSS_ERR_NOMEM;
    st->grab[chn].copy = b;
    st->grab[chn].copy_cap = need;
    return RSS_OK;
}

static void xm_fill_info(rss_frame_info_t *info, const VIDEO_FRAME_INFO_S *vf, void *data)
{
    size_t ysize = (size_t)vf->stVFrame.u32Width * vf->stVFrame.u32Height;

    memset(info, 0, sizeof(*info));
    info->width = (uint16_t)vf->stVFrame.u32Width;
    info->height = (uint16_t)vf->stVFrame.u32Height;
    info->pixfmt = RSS_PIXFMT_NV12;
    info->timestamp = xm_now_us();
    info->phys_addr = vf->stVFrame.u32PhyAddr[0];
    info->virt_addr = data;
    info->size = (uint32_t)(ysize + ysize / 2);
}

/* The feeder's half of a raw grab on a fed channel (xm_enc_grab): copy
 * the frame it holds into grab[chn].copy. */
int xm_frame_copy(xm_state_t *st, int chn, const void *vfp)
{
    const VIDEO_FRAME_INFO_S *vf = vfp;
    size_t ysize = (size_t)vf->stVFrame.u32Width * vf->stVFrame.u32Height, map_len;
    const uint8_t *y, *c;
    uint8_t *map;
    int rc;

    if ((rc = xm_copy_room(st, chn, ysize + ysize / 2)) ||
        (rc = xm_frame_map(st, chn, vf, &map, &map_len, &y, &c)))
        return rc;
    memcpy(st->grab[chn].copy, y, ysize);
    memcpy(st->grab[chn].copy + ysize, c, ysize / 2);
    munmap(map, map_len);
    xm_fill_info(&st->grab[chn].info, vf, st->grab[chn].copy);
    return RSS_OK;
}

/*
 * One frame of channel chn as contiguous NV12.
 *
 * On a channel the encoders are fed from, the feeder takes the frame (VI
 * gives each frame to one getter, and the encoders must not lose any):
 * it copies its next one for us. Otherwise the planes are mapped straight
 * from VI's buffer, and when chroma follows luma directly (as on M1) the
 * caller gets the mapping itself. VI_GetFrame does not wait: no frame yet
 * is RSS_ERR_BUSY after a short poll.
 */
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info)
{
    xm_state_t *st = xm_state(ctx);
    VIDEO_FRAME_INFO_S *vf;
    XM_U32 w, h, yphys, cphys;
    size_t ysize, csize, map_len;
    const uint8_t *y, *c;
    uint8_t *map;
    int tries, rc;

    if (!st || !frame_data || !info || chn < 0 || chn >= XM_FS_CHN_MAX)
        return RSS_ERR_INVAL;
    if (!st->fs[chn].enabled)
        return RSS_ERR_INVAL;
    xm_grab_drop(st, chn); /* a caller that forgot to release */

    if (st->enc_ready && st->feed[chn].running) {
        rc = xm_enc_grab(st, chn);
        if (rc)
            return rc;
        st->grab[chn].from_feeder = true;
        *frame_data = st->grab[chn].copy;
        *info = st->grab[chn].info;
        return RSS_OK;
    }

    vf = (VIDEO_FRAME_INFO_S *)st->grab[chn].frame;
    for (tries = 0;; tries++) {
        memset(vf, 0, sizeof(*vf));
        if (XM_MPI_VI_GetFrame(chn, vf) == XM_SUCCESS)
            break;
        if (tries >= 20) /* 200 ms: three frames at 15 fps */
            return RSS_ERR_BUSY;
        usleep(10000);
    }
    st->grab[chn].held = true;

    w = vf->stVFrame.u32Width;
    h = vf->stVFrame.u32Height;
    yphys = vf->stVFrame.u32PhyAddr[0];
    cphys = vf->stVFrame.u32PhyAddr[1];
    ysize = (size_t)w * h;
    csize = ysize / 2;
    if (w != st->fs[chn].width || h != st->fs[chn].height || !yphys || !cphys) {
        if (!st->fs[chn].logged)
            HAL_LOG_WARN("fs %d: VI frame %ux%u luma %08x chroma %08x, expected %ux%u", chn, w,
                         h, yphys, cphys, st->fs[chn].width, st->fs[chn].height);
        st->fs[chn].logged = true;
        xm_grab_drop(st, chn);
        return RSS_ERR_IO;
    }
    if ((rc = xm_frame_map(st, chn, vf, &map, &map_len, &y, &c))) {
        xm_grab_drop(st, chn);
        return rc;
    }
    st->grab[chn].map = map;
    st->grab[chn].map_len = map_len;
    if (c == y + ysize) {
        *frame_data = (void *)y;
    } else {
        if ((rc = xm_copy_room(st, chn, ysize + csize))) {
            xm_grab_drop(st, chn);
            return rc;
        }
        memcpy(st->grab[chn].copy, y, ysize);
        memcpy(st->grab[chn].copy + ysize, c, csize);
        *frame_data = st->grab[chn].copy;
    }

    if (!st->fs[chn].logged) {
        st->fs[chn].logged = true;
        HAL_LOG_INFO("fs %d: first frame %ux%u, luma %08x chroma %08x (%s), pts %llu", chn, w, h,
                     yphys, cphys, cphys == yphys + ysize ? "contiguous" : "apart, copied",
                     (unsigned long long)vf->stVFrame.u64pts);
    }
    xm_fill_info(info, vf, *frame_data);
    return RSS_OK;
}

int hal_fs_release_frame(void *ctx, int chn, void *frame_data)
{
    xm_state_t *st = xm_state(ctx);

    (void)frame_data;
    if (!st || chn < 0 || chn >= XM_FS_CHN_MAX)
        return RSS_ERR_INVAL;
    xm_grab_drop(st, chn);
    return RSS_OK;
}

/* ================================================================
 * BENCH SELF-TEST (XM_FS_SELFTEST=1)
 * ================================================================ */

/*
 * Phase 2a stops where the encoder is missing, before rvd enables any
 * framesource, so the frames are checked here: channel 0 at the sensor
 * window and channel 1 at 640x480 for a few seconds, mean luma logged, one
 * frame of each saved to /tmp/rvd-fs<n>.nv12. Then both are put back as
 * they were, for rvd to set up its own way.
 */
static void xm_selftest(xm_state_t *st)
{
    static const uint32_t sub_w = 640, sub_h = 480;
    rss_hal_ctx_t fake;
    int chn, s, seconds = 6;

    memset(&fake, 0, sizeof(fake));
    fake.platform = st;

    st->fs[0] = (xm_fs_chn_t){.created = true,
                              .width = st->mode->width,
                              .height = st->mode->height,
                              .cap_x = st->mode->cap_x,
                              .cap_y = st->mode->cap_y};
    st->fs[1] = (xm_fs_chn_t){.created = true, .width = sub_w, .height = sub_h};
    if (xm_fs_set(st, 0) || xm_fs_set(st, 1) || hal_fs_enable_channel(&fake, 0) ||
        hal_fs_enable_channel(&fake, 1)) {
        HAL_LOG_ERR("selftest: VI channels did not start");
        goto out;
    }

    for (s = 1; s <= seconds; s++) {
        sleep(1);
        for (chn = 0; chn < XM_FS_CHN_MAX; chn++) {
            rss_frame_info_t fi;
            void *data;
            unsigned long sum = 0, n = 0;
            size_t i, ysize;
            int ret = hal_fs_get_frame(&fake, chn, &data, &fi);

            if (ret) {
                HAL_LOG_WARN("selftest %d s: fs %d no frame (%d)", s, chn, ret);
                continue;
            }
            ysize = (size_t)fi.width * fi.height;
            for (i = 0; i < ysize; i += 97, n++)
                sum += ((const uint8_t *)data)[i];
            HAL_LOG_INFO("selftest %d s: fs %d %ux%u mean luma %lu", s, chn, fi.width,
                         fi.height, n ? sum / n : 0);
            if (s == seconds) {
                char path[32];
                FILE *o;

                snprintf(path, sizeof(path), "/tmp/rvd-fs%d.nv12", chn);
                o = fopen(path, "wb");
                if (o) {
                    fwrite(data, 1, fi.size, o);
                    fclose(o);
                    HAL_LOG_INFO("selftest: saved %s (%ux%u NV12)", path, fi.width, fi.height);
                }
            }
            hal_fs_release_frame(&fake, chn, data);
        }
    }

out:
    hal_fs_disable_channel(&fake, 0);
    memset(st->fs, 0, sizeof(st->fs));
}
