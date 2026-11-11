/*
 * xiongmai/xm_state.h -- live backend state for the Xiongmai backend.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef XM_STATE_H
#define XM_STATE_H

#include "xm_common.h"

#include <stdint.h>

/* fs channel 0 is VI channel 0 (no scaler); fs channel 1 is VI extended
 * channel 1, scaled by the kernel from channel 0. */
#define XM_FS_CHN_MAX 2

/* One sensor the backend knows how to frame. The capture window is the one
 * ISP leaves clean on M1 (8,4,2576,1936 of the SC5332's 2600x1952). */
typedef struct {
    const char *name;
    uint32_t width, height; /* what channel 0 delivers */
    uint32_t cap_x, cap_y;  /* where that window starts in the sensor frame */
    uint32_t fps;
} xm_sensor_mode_t;

typedef struct {
    bool created;
    bool enabled;
    uint32_t width, height;
    uint32_t cap_x, cap_y; /* channel 0 only */
    bool logged;           /* first frame of this channel logged */
} xm_fs_chn_t;

typedef struct {
    bool sys_inited;
    bool isp_started;

    char chip_name[24];
    char sdk_version[64];
    char sensor_name[64];

    const xm_sensor_mode_t *mode;
    xm_fs_chn_t fs[XM_FS_CHN_MAX];

    /* frame grabs: the held VI frame and its /dev/mem mapping per channel */
    int mem_fd;
    struct {
        bool held;
        uint8_t frame[96]; /* VIDEO_FRAME_INFO_S, opaque here */
        void *map;
        size_t map_len;
        uint8_t *copy;
        size_t copy_cap;
    } grab[XM_FS_CHN_MAX];
} xm_state_t;

static inline xm_state_t *xm_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (xm_state_t *)c->platform : NULL;
}

/* xm_pipeline.c */
int xm_pipe_start(xm_state_t *st);
void xm_pipe_stop(xm_state_t *st);
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height);
int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_destroy_channel(void *ctx, int chn);
int hal_fs_enable_channel(void *ctx, int chn);
int hal_fs_disable_channel(void *ctx, int chn);
int hal_fs_set_frame_depth(void *ctx, int chn, int depth);
int hal_fs_set_fifo(void *ctx, int chn, int depth);
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
int hal_fs_release_frame(void *ctx, int chn, void *frame_data);

/* xm_isp.c -- the ISP library's attributes, kept out of the MPI headers'
 * translation unit (the two header sets are not meant to meet). */
int xm_isp_exposure_defaults(void);
int hal_isp_set_ae_comp(void *ctx, int val);
int hal_isp_get_ae_comp(void *ctx, int *val);
int hal_isp_set_saturation(void *ctx, int val);
int hal_isp_get_saturation(void *ctx, int *val);

#endif /* XM_STATE_H */
