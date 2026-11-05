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

#ifdef HAL_MODULE_VIDEO
#define FH_ENC_QDEPTH 3
#define FH_ENC_MAX_NALS 16

/* One encoded frame, copied out of the driver's buffer. */
typedef struct {
    uint8_t *buf;
    size_t len, cap;
    rss_nal_unit_t nals[FH_ENC_MAX_NALS];
    uint32_t nal_count;
    int64_t ts;
    uint32_t seq;
    bool key;
} fh_frame_t;

typedef struct {
    bool created, receiving, started, made;
    rss_codec_t codec;
    unsigned int fh_codec;
    unsigned int width, height, made_w, made_h;
    uint32_t fps_num, fps_den, gop, bitrate;
    rss_rc_mode_t rc_mode;
    int init_qp, jpeg_quality, profile, min_qp, max_qp;
    int bound_fs;

    /* slots[] hold frames; q[] is the FIFO of slot indexes; held is the
     * slot rvd has checked out, -1 if none. */
    fh_frame_t slots[FH_ENC_QDEPTH + 1];
    int q[FH_ENC_QDEPTH];
    int q_head, q_count, held;
    uint32_t seq, drops;
    bool first_logged, sc_logged;
} fh_enc_chn_t;
#endif

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

    int fs_bound_codec[FH_VPSS_CHN_MAX];
    fh_dim vpss_made[FH_VPSS_CHN_MAX];
    int osd_src_fs[FH_VENC_CHN_MAX];

    fh_enc_chn_t enc[FH_VENC_CHN_MAX];
    pthread_mutex_t enc_lock;
    pthread_cond_t enc_cond;
    bool enc_sync_ready;
    pthread_t collect_thread;
    int collect_run, collect_done;
    bool collect_started;

    pthread_t isp_thread;
    int isp_thread_run;  /* atomic: the thread keeps calling API_ISP_Run */
    int isp_thread_done; /* atomic: the thread has returned */
    int isp_pause;       /* atomic: skip API_ISP_Run (tuning swap) */
    int isp_busy;        /* atomic: inside API_ISP_Run */
    rss_isp_mode_t isp_mode;
    bool exp_logged, exp_warned;
    /* Exposure, sampled by the 3A thread itself (see fh_isp_sample_ae);
     * read by hal_isp_get_exposure under exp_lock. */
    pthread_mutex_t exp_lock;
    bool exp_lock_ready, exp_valid;
    uint32_t exp_luma, exp_gain, exp_time;
    uint32_t luma_hist[3];
    int luma_pos;
    int64_t exp_next_us;
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
FH_LOCAL int fh_pipe_prepare(fh_state_t *st, const rss_sensor_config_t *snr);
FH_LOCAL int fh_pipe_start(fh_state_t *st);
FH_LOCAL void fh_pipe_stop(fh_state_t *st);
FH_LOCAL int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height);
FH_LOCAL int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);
FH_LOCAL int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
FH_LOCAL int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);
FH_LOCAL int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode);
FH_LOCAL int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode);

/* fh_encoder.c */
FH_LOCAL int fh_enc_init(fh_state_t *st);
FH_LOCAL void fh_enc_teardown(fh_state_t *st);
FH_LOCAL int fh_vpss_configure(fh_state_t *st, int chn, unsigned int w, unsigned int h, unsigned int fps);
FH_LOCAL void fh_sleep_ms(unsigned int ms);

FH_LOCAL int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);
FH_LOCAL int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);
FH_LOCAL int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
FH_LOCAL int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg);
FH_LOCAL int hal_fs_enable_channel(void *ctx, int chn);
FH_LOCAL int hal_fs_disable_channel(void *ctx, int chn);
FH_LOCAL int hal_fs_destroy_channel(void *ctx, int chn);
FH_LOCAL int hal_enc_create_group(void *ctx, int grp);
FH_LOCAL int hal_enc_destroy_group(void *ctx, int grp);
FH_LOCAL int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg);
FH_LOCAL int hal_enc_destroy_channel(void *ctx, int chn);
FH_LOCAL int hal_enc_register_channel(void *ctx, int grp, int chn);
FH_LOCAL int hal_enc_unregister_channel(void *ctx, int chn);
FH_LOCAL int hal_enc_start(void *ctx, int chn);
FH_LOCAL int hal_enc_stop(void *ctx, int chn);
FH_LOCAL int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms);
FH_LOCAL int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame);
FH_LOCAL int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame);
FH_LOCAL int hal_enc_request_idr(void *ctx, int chn);
FH_LOCAL int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
FH_LOCAL int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
FH_LOCAL int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
FH_LOCAL int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp);
FH_LOCAL int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp);
FH_LOCAL int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
FH_LOCAL int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate);
FH_LOCAL int hal_sys_get_timestamp(void *ctx, int64_t *ts);

/* fh_crash.c */
FH_LOCAL void fh_crash_install(void);
#endif

#endif /* FH_STATE_H */
