/*
 * xiongmai/xm_state.h -- live backend state for the Xiongmai backend.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef XM_STATE_H
#define XM_STATE_H

#include "xm_common.h"

#include <pthread.h>
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
    uint32_t vts;           /* lines a frame, for the exposure time */
} xm_sensor_mode_t;

/* Encoder channels: rvd's main and sub H.264 (0, 1) and their JPEG
 * snapshots (2, 3), one libxmvenc channel id each (0-3). */
#define XM_ENC_CHN_MAX 4
#define XM_ENC_QDEPTH 4
#define XM_ENC_MAX_NALS 8

typedef struct {
    uint8_t *buf;
    size_t len, cap;
    rss_nal_unit_t nals[XM_ENC_MAX_NALS];
    uint32_t nal_count;
    int64_t ts;
    uint32_t seq;
    bool key;
} xm_frame_t;

typedef struct {
    bool created, started;
    rss_codec_t codec; /* H264, or JPEG for JPEG and MJPEG */
    uint32_t width, height, fps_num, fps_den, gop, bitrate;
    rss_rc_mode_t rc_mode;
    int profile, init_qp, min_qp, max_qp, quality;
    int vi;      /* the VI channel (fs) it encodes, -1: none yet */
    void *xchn;  /* struct xmvenc_chn */
    bool want_idr;
    int64_t next_due; /* pacing below the sensor rate (JPEG at 1 fps) */

    /* queue of encoded frames for rvd: slots[] hold them, q[] the order */
    xm_frame_t slots[XM_ENC_QDEPTH + 1];
    int q[XM_ENC_QDEPTH];
    int q_head, q_count, held;
    uint32_t seq, drops;

    /* 10 s statistics */
    uint32_t st_frames, st_fail;
    uint64_t st_bytes, st_hw_us;
    int st_qp;
    int64_t st_since;
    bool first_logged;
} xm_enc_chn_t;

/* One feeder per VI channel: it takes each frame once and hands it to every
 * started encoder channel bound to that VI channel (H.264 and its JPEG), so
 * nothing else competes for VI's buffers. rvd's raw grabs from a fed VI
 * channel are served by the feeder too, as copies. */
typedef struct {
    pthread_t thread;
    bool running, stop;
    pthread_mutex_t lock; /* held over one frame: encodes, grab, release */
    bool grab_req, grab_done;

    /* The JPEG worker: a snapshot of the frame the feeder hands it, encoded
     * on the encoder's spare time while the feeder goes on (a 5 MP JPEG
     * takes 35 ms of hardware, main's H.264 44 of the 67 a frame has).
     * The worker holds that VI frame until it is done. Under enc_lock. */
    pthread_t jthread;
    bool jrunning, jbusy;
    int jchn;
    int64_t jts;
    uint8_t jframe[96]; /* VIDEO_FRAME_INFO_S, opaque here */
} xm_feeder_t;

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
    /* what VI was last given, so an unchanged channel 0 is not set again
     * (setting it costs about 3 s) */
    struct {
        bool valid;
        uint32_t w, h, x, y;
    } fs_applied[XM_FS_CHN_MAX];

    /* encoders (xm_encoder.c) */
    void *venc; /* struct xmvenc */
    bool enc_ready;
    pthread_mutex_t enc_lock; /* queues and channel state */
    pthread_cond_t enc_cond;
    xm_enc_chn_t enc[XM_ENC_CHN_MAX];
    xm_feeder_t feed[XM_FS_CHN_MAX];
    int osd_src_fs[XM_ENC_CHN_MAX];

    /* frame grabs: the held VI frame and its /dev/mem mapping per channel */
    int mem_fd;
    struct {
        bool held;
        uint8_t frame[96]; /* VIDEO_FRAME_INFO_S, opaque here */
        void *map;
        size_t map_len;
        uint8_t *copy;
        size_t copy_cap;
        bool from_feeder;       /* the copy came from the feeder, no VI frame held */
        rss_frame_info_t info;  /* the feeder's copy */
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

/* used by the feeder (xm_encoder.c): copy the VI frame vf of channel chn
 * into grab[chn].copy as contiguous NV12, filling grab[chn].info */
int xm_frame_copy(xm_state_t *st, int chn, const void *vf);

/* xm_encoder.c */
int xm_enc_init(xm_state_t *st);
void xm_enc_deinit(xm_state_t *st);
int xm_enc_grab(xm_state_t *st, int chn); /* a raw frame from a fed channel */
/* VI channel vi with no frame held by its feeder or JPEG worker, until
 * xm_feed_release: for VI attribute changes and disables */
void xm_feed_hold(xm_state_t *st, int vi);
void xm_feed_release(xm_state_t *st, int vi);
int hal_enc_create_group(void *ctx, int grp);
int hal_enc_destroy_group(void *ctx, int grp);
int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg);
int hal_enc_destroy_channel(void *ctx, int chn);
int hal_enc_register_channel(void *ctx, int grp, int chn);
int hal_enc_unregister_channel(void *ctx, int chn);
int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);
int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst);
int hal_enc_start(void *ctx, int chn);
int hal_enc_stop(void *ctx, int chn);
int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms);
int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_request_idr(void *ctx, int chn);
int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp);
int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp);
int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate);

/* xm_isp.c -- the ISP library's attributes, kept out of the MPI headers'
 * translation unit (the two header sets are not meant to meet). */
int xm_isp_exposure_defaults(void);
int hal_isp_set_ae_comp(void *ctx, int val);
int hal_isp_get_ae_comp(void *ctx, int *val);
int hal_isp_set_saturation(void *ctx, int val);
int hal_isp_get_saturation(void *ctx, int *val);
void xm_isp_set_timing(unsigned fps, unsigned vts);
int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode);

#endif /* XM_STATE_H */
