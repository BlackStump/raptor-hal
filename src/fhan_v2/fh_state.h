/*
 * fhan_v2/fh_state.h -- live backend state for the Fullhan backend.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef FH_STATE_H
#define FH_STATE_H

#include "fh_common.h"
#include "fh_pipe.h"

typedef struct {
    void *h_vmm;
    void *h_dsp;

    int (*fnSysInit)(void);
    int (*fnSysExit)(void);
    int (*fnSysGetVersion)(void *ver);
    int (*fnSysGetChipId)(void *id); /* optional */
} fh_sys_impl;

typedef struct {
    fh_sys_impl sys;
    bool sys_inited;

#ifdef HAL_MODULE_VIDEO
    /* Pipeline (Phase 2a: sensor, ISP, VPSS channel 0). */
    fh_pipe_impl pipe;
    const fh_snr_mode *mode;
    char snr_lib[32];
    bool vpss_enabled, isp_registered, isp_inited, snr_created;
    bool vpss_open[FH_VPSS_CHN_MAX];
    fh_dim vpss_dim[FH_VPSS_CHN_MAX];
    int vpss_count;
    bool mirror, flip;

    pthread_t isp_thread;
    int isp_thread_run;  /* atomic: the thread keeps calling API_ISP_Run */
    int isp_thread_done; /* atomic: the thread has returned */
    bool isp_thread_started;
#endif

    char chip_name[24];
    char sdk_version[64];
    char sensor_name[64];
} fh_state_t;

static inline fh_state_t *fh_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (fh_state_t *)c->platform : NULL;
}

#ifdef HAL_MODULE_VIDEO
/* fh_pipeline.c */
int fh_pipe_prepare(fh_state_t *st, const rss_sensor_config_t *snr);
int fh_pipe_start(fh_state_t *st);
void fh_pipe_stop(fh_state_t *st);
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height);
int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
#endif

#endif /* FH_STATE_H */
