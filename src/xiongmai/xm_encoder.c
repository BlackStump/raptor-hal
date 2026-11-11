/*
 * xiongmai/xm_encoder.c -- encoders on openxiongmai's libxmvenc.
 *
 * Phase 2b. libxmvenc drives the XM530/XM550's Hantro H1 directly (no
 * vendor libraries; it needs xm530_h264.ko's /dev/h264, /dev/mmz and
 * /dev/mem). One xmvenc per process, one xmvenc channel per raptor
 * encoder channel, the same id (0-3).
 *
 * WHY NOT libxmvenc's PIPES. A pipe takes its own frames from VI, and VI
 * hands each frame to one getter, so two pipes on one VI channel would
 * split its frames between them -- and rvd puts two encoders on each
 * framesource: the H.264 stream and its JPEG snapshots. (openxiongmai's
 * xmvenc-test, for the same reason, snapshots the other VI channel.)
 * So the backend feeds them itself: one feeder thread per VI channel
 * takes each frame once, encodes it on every started channel bound there
 * that is due, copies the output into that channel's queue, and gives the
 * frame back. H.264 is encoded by the feeder, every frame; a JPEG that is
 * due goes to the feeder's JPEG worker with its frame, so a 35 ms
 * snapshot never stands between main and its next frame (inline, the
 * snapshot frame took 79 ms of the 67 and main lost 2 frames a snapshot).
 * rvd's raw frame grabs on a fed channel are served by the feeder too.
 *
 * Frames go to rvd as rss_frame_t: Annex B NALs with their start codes,
 * contiguous in one buffer the backend owns (rvd edits the SPS in place).
 * H.264: SPS and PPS before every IDR, then the slice; libxmvenc writes no
 * VUI, so nothing claims a frame rate (majestic's SPS said 25 at 15 fps).
 * JPEG: one NAL, the header and the scan, ending in EOI.
 *
 * Rate control is libxmvenc's, frame-level, budgeted at the rate frames
 * are encoded at: the configured fps or the sensor's, whichever is lower.
 * libxmvenc has no setters, so bitrate, GOP, mode and JPEG quality
 * changes recreate the channel (same id, same MMZ blocks), between frames.
 *
 * H.265 (Mali-V550, mve_base.ko) is libxmvenc's too, but not this phase.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE

#include "xm_state.h"

#include <re_vi.h>
#include <xmvenc.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define XM_STATS_US 10000000LL

static int64_t now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static void xm_venc_log(const char *msg)
{
    HAL_LOG_INFO("xmvenc: %s", msg);
}

static bool is_jpeg(rss_codec_t c)
{
    return c == RSS_CODEC_JPEG || c == RSS_CODEC_MJPEG;
}

#define XM_ENC_ENTER(ctx, chn, st, enc)                                                            \
    xm_state_t *st = xm_state(ctx);                                                                \
    xm_enc_chn_t *enc;                                                                             \
    if (!st || !st->enc_ready || (chn) < 0 || (chn) >= XM_ENC_CHN_MAX)                             \
        return RSS_ERR_INVAL;                                                                      \
    enc = &st->enc[(chn)]

/* ================================================================
 * INIT / DEINIT
 * ================================================================ */

int xm_enc_init(xm_state_t *st)
{
    int i;

    memset(st->enc, 0, sizeof(st->enc));
    memset(st->feed, 0, sizeof(st->feed));
    for (i = 0; i < XM_ENC_CHN_MAX; i++) {
        st->enc[i].vi = -1;
        st->enc[i].held = -1;
        st->osd_src_fs[i] = -1;
    }
    pthread_mutex_init(&st->enc_lock, NULL);
    pthread_cond_init(&st->enc_cond, NULL);
    for (i = 0; i < XM_FS_CHN_MAX; i++)
        pthread_mutex_init(&st->feed[i].lock, NULL);
    st->enc_ready = true;
    return RSS_OK;
}

static void xm_feeder_stop(xm_state_t *st, int vi);

void xm_enc_deinit(xm_state_t *st)
{
    int i, s;

    if (!st->enc_ready)
        return;
    for (i = 0; i < XM_FS_CHN_MAX; i++)
        xm_feeder_stop(st, i);
    for (i = 0; i < XM_ENC_CHN_MAX; i++) {
        if (st->enc[i].xchn)
            xmvenc_chn_destroy(st->enc[i].xchn);
        st->enc[i].xchn = NULL;
        for (s = 0; s <= XM_ENC_QDEPTH; s++)
            free(st->enc[i].slots[s].buf);
        memset(&st->enc[i], 0, sizeof(st->enc[i]));
    }
    if (st->venc)
        xmvenc_close(st->venc);
    st->venc = NULL;
    st->enc_ready = false;
    pthread_cond_destroy(&st->enc_cond);
    pthread_mutex_destroy(&st->enc_lock);
    for (i = 0; i < XM_FS_CHN_MAX; i++)
        pthread_mutex_destroy(&st->feed[i].lock);
}

/* ================================================================
 * xmvenc CHANNELS
 * ================================================================ */

static int xm_open_venc(xm_state_t *st)
{
    if (st->venc)
        return RSS_OK;
    st->venc = xmvenc_open(xm_venc_log);
    if (!st->venc) {
        HAL_LOG_ERR("venc: xmvenc_open failed (is xm530_h264.ko loaded, /dev/h264 there?)");
        return RSS_ERR_IO;
    }
    HAL_LOG_INFO("venc: libxmvenc on %s, up to %d wide", xmvenc_chip(st->venc),
                 xmvenc_max_width(st->venc));
    return RSS_OK;
}

/* The rate frames are really encoded at: the configured one, but never
 * above the sensor's (rate control would budget for frames that never
 * come and undershoot). */
static double xm_enc_rate(const xm_state_t *st, const xm_enc_chn_t *enc)
{
    double fps = enc->fps_num && enc->fps_den ? (double)enc->fps_num / enc->fps_den : 0;
    double snr = st->mode ? st->mode->fps : 15;

    return fps <= 0 || fps > snr ? snr : fps;
}

static void xm_fill_attr(const xm_state_t *st, const xm_enc_chn_t *enc,
                         struct xmvenc_chn_attr *a)
{
    memset(a, 0, sizeof(*a));
    a->width = (int)enc->width;
    a->height = (int)enc->height;
    a->stride = (int)enc->width;
    if (is_jpeg(enc->codec)) {
        a->codec = XMVENC_JPEG;
        a->quality = enc->quality > 0 ? enc->quality : 80;
        /* a 5 MP JPEG at quality 75-90 is 1-2.5 MB */
        a->strm_size = enc->width * enc->height / 2;
        if (a->strm_size < 256 * 1024)
            a->strm_size = 256 * 1024;
        return;
    }
    a->codec = XMVENC_H264;
    a->cabac = enc->profile >= 1;
    a->gop = enc->gop ? (int)enc->gop : (int)(xm_enc_rate(st, enc) * 2);
    a->qp = enc->init_qp > 0 && enc->init_qp <= 51 ? enc->init_qp : 30;
    a->qmin = enc->min_qp >= 0 && enc->min_qp <= 51 ? enc->min_qp : 20;
    a->qmax = enc->max_qp >= 0 && enc->max_qp <= 51 ? enc->max_qp : 45;
    if (a->qmin > a->qmax)
        a->qmin = a->qmax;
    a->fps = xm_enc_rate(st, enc);
    if (enc->rc_mode != RSS_RC_FIXQP && enc->bitrate) {
        a->kbps = enc->bitrate / 1000.0;
        a->vbr = enc->rc_mode != RSS_RC_CBR;
    }
    /* one frame at a time leaves the stream buffer (the backend copies
     * it out); an I frame of 5 MP at a high rate is under 2 MB */
    a->strm_size = enc->width * enc->height >= 2000000 ? 2u << 20 : 512u << 10;
}

static const char *xm_rc_name(const xm_enc_chn_t *enc)
{
    if (enc->rc_mode == RSS_RC_FIXQP || !enc->bitrate)
        return "fixed QP";
    return enc->rc_mode == RSS_RC_CBR ? "CBR" : "VBR";
}

/* (Re)make the xmvenc channel. Callers hold the channel's feeder lock if
 * it has one, so no encode is under way on it. */
static int xm_enc_make(xm_state_t *st, int chn)
{
    xm_enc_chn_t *enc = &st->enc[chn];
    struct xmvenc_chn_attr a;
    struct xmvenc_chn *x;

    xm_fill_attr(st, enc, &a);
    if (enc->xchn) {
        xmvenc_chn_destroy(enc->xchn);
        enc->xchn = NULL;
    }
    x = xmvenc_chn_create(st->venc, chn, &a);
    if (!x) {
        HAL_LOG_ERR("venc chn %d: xmvenc_chn_create %s %dx%d failed", chn,
                    a.codec == XMVENC_JPEG ? "JPEG" : "H.264", a.width, a.height);
        return RSS_ERR_IO;
    }
    enc->xchn = x;
    if (a.codec == XMVENC_JPEG)
        HAL_LOG_INFO("venc chn %d: JPEG %dx%d quality %d at %.3g fps, %u KiB stream buffer", chn,
                     a.width, a.height, a.quality,
                     enc->fps_num && enc->fps_den ? (double)enc->fps_num / enc->fps_den : 1.0,
                     a.strm_size >> 10);
    else
        HAL_LOG_INFO("venc chn %d: H.264 %s %dx%d, %s %.0f kbps, QP %d (%d-%d), GOP %d at %.3g "
                     "fps",
                     chn, a.cabac ? "Main (CABAC)" : "Constrained Baseline", a.width, a.height,
                     xm_rc_name(enc), a.kbps, a.qp, a.qmin, a.qmax, a.gop, a.fps);
    return RSS_OK;
}

/* Recreate a channel with changed settings, between frames */
static int xm_enc_remake(xm_state_t *st, int chn)
{
    xm_enc_chn_t *enc = &st->enc[chn];
    int vi = enc->vi, rc;

    if (!enc->xchn)
        return RSS_OK; /* made at create; nothing yet */
    if (vi >= 0)
        xm_feed_hold(st, vi);
    rc = xm_enc_make(st, chn);
    if (vi >= 0)
        xm_feed_release(st, vi);
    return rc;
}

/* ================================================================
 * QUEUES
 * ================================================================ */

static bool xm_slot_room(xm_frame_t *f, size_t need)
{
    uint8_t *b;

    if (f->cap >= need)
        return true;
    b = realloc(f->buf, need + 4096);
    if (!b)
        return false;
    f->buf = b;
    f->cap = need + 4096;
    return true;
}

/* a free slot: not queued, not held */
static int xm_slot_free(const xm_enc_chn_t *enc)
{
    int s, i;
    bool used;

    for (s = 0; s <= XM_ENC_QDEPTH; s++) {
        used = s == enc->held;
        for (i = 0; i < enc->q_count && !used; i++)
            used = enc->q[(enc->q_head + i) % XM_ENC_QDEPTH] == s;
        if (!used)
            return s;
    }
    return -1;
}

static size_t sc_len(const uint8_t *d, size_t n)
{
    if (n >= 4 && !d[0] && !d[1] && !d[2] && d[3] == 1)
        return 4;
    if (n >= 3 && !d[0] && !d[1] && d[2] == 1)
        return 3;
    return 0;
}

static rss_nal_type_t h264_type(uint8_t hdr)
{
    switch (hdr & 0x1f) {
    case 7:
        return RSS_NAL_H264_SPS;
    case 8:
        return RSS_NAL_H264_PPS;
    case 6:
        return RSS_NAL_H264_SEI;
    case 5:
        return RSS_NAL_H264_IDR;
    case 1:
        return RSS_NAL_H264_SLICE;
    default:
        return RSS_NAL_UNKNOWN;
    }
}

/* Split f->buf[from..f->len) into NALs at their start codes */
static void xm_split(xm_frame_t *f, size_t from)
{
    size_t start = from;

    while (start < f->len && f->nal_count < XM_ENC_MAX_NALS) {
        size_t sc = sc_len(f->buf + start, f->len - start), next;

        /* the next start code after this one's header */
        for (next = start + sc + 1; next + 3 <= f->len; next++)
            if (!f->buf[next] && !f->buf[next + 1] &&
                (f->buf[next + 2] == 1 ||
                 (next + 4 <= f->len && !f->buf[next + 2] && f->buf[next + 3] == 1)))
                break;
        if (next + 3 > f->len)
            next = f->len;
        f->nals[f->nal_count].data = f->buf + start;
        f->nals[f->nal_count].length = (uint32_t)(next - start);
        f->nals[f->nal_count].type =
            start + sc < f->len ? h264_type(f->buf[start + sc]) : RSS_NAL_UNKNOWN;
        f->nals[f->nal_count].frame_end = false;
        f->nal_count++;
        start = next;
    }
}

/*
 * File one encoded frame into the channel's queue. A full queue (rvd
 * behind) drops the frame, and an H.264 channel then starts again from an
 * IDR, so what rvd does get stays decodable. Returns the bytes filed.
 */
static size_t xm_file(xm_state_t *st, int chn, const struct xmvenc_frame *out, int64_t ts)
{
    xm_enc_chn_t *enc = &st->enc[chn];
    static const uint8_t sc4[4] = {0, 0, 0, 1};
    size_t need, add_sc;
    xm_frame_t *f;
    int s;

    add_sc = !is_jpeg(enc->codec) && !sc_len(out->data, out->len) ? 4 : 0;
    need = out->hdr_len + out->len + add_sc;

    pthread_mutex_lock(&st->enc_lock);
    if (!enc->started || enc->q_count >= XM_ENC_QDEPTH || (s = xm_slot_free(enc)) < 0) {
        enc->drops++;
        if (!is_jpeg(enc->codec))
            enc->want_idr = true;
        pthread_mutex_unlock(&st->enc_lock);
        return 0;
    }
    f = &enc->slots[s];
    if (!xm_slot_room(f, need)) {
        enc->drops++;
        enc->want_idr = !is_jpeg(enc->codec);
        pthread_mutex_unlock(&st->enc_lock);
        return 0;
    }
    f->len = 0;
    f->nal_count = 0;
    if (out->hdr_len)
        memcpy(f->buf, out->hdr, out->hdr_len);
    f->len = out->hdr_len;
    if (add_sc) {
        memcpy(f->buf + f->len, sc4, 4);
        f->len += 4;
    }
    memcpy(f->buf + f->len, out->data, out->len);
    f->len += out->len;

    if (is_jpeg(enc->codec)) {
        f->nals[0].data = f->buf;
        f->nals[0].length = (uint32_t)f->len;
        f->nals[0].type = RSS_NAL_JPEG_FRAME;
        f->nal_count = 1;
        f->key = true;
    } else {
        xm_split(f, 0);
        f->key = out->type == 'I';
    }
    f->nals[f->nal_count - 1].frame_end = true;
    f->ts = ts;
    f->seq = enc->seq++;

    enc->q[(enc->q_head + enc->q_count) % XM_ENC_QDEPTH] = s;
    enc->q_count++;
    pthread_cond_broadcast(&st->enc_cond);

    if (!enc->first_logged) {
        enc->first_logged = true;
        if (is_jpeg(enc->codec))
            HAL_LOG_INFO("venc chn %d: first JPEG, %zu bytes, %ld us in hardware", chn, f->len,
                         out->us);
        else
            HAL_LOG_INFO("venc chn %d: first frame %c, %u NALs, %zu bytes, QP %d, %ld us in "
                         "hardware",
                         chn, out->type, f->nal_count, f->len, out->qp, out->us);
    }
    pthread_mutex_unlock(&st->enc_lock);
    return f->len;
}

/* ================================================================
 * FEEDERS
 * ================================================================ */

static bool xm_due(xm_enc_chn_t *enc, double rate, double snr_fps, int64_t now)
{
    int64_t period;

    if (rate >= snr_fps - 0.01)
        return true; /* every frame */
    period = (int64_t)(1e6 / rate);
    if (!enc->next_due || now >= enc->next_due + period)
        enc->next_due = now; /* first frame, or far behind */
    if (now + 1000000 / (int64_t)(snr_fps * 2) < enc->next_due)
        return false;
    enc->next_due += period;
    return true;
}

static void xm_stats(xm_state_t *st, int chn, int64_t now)
{
    xm_enc_chn_t *enc = &st->enc[chn];
    double secs;

    if (!enc->st_since) {
        enc->st_since = now;
        return;
    }
    if (now - enc->st_since < XM_STATS_US)
        return;
    secs = (now - enc->st_since) / 1e6;
    if (is_jpeg(enc->codec))
        HAL_LOG_INFO("venc chn %d: %u JPEG in %.0f s, %.0f KiB each, %.1f ms hardware, %u "
                     "failed, %u dropped",
                     chn, enc->st_frames, secs,
                     enc->st_frames ? enc->st_bytes / 1024.0 / enc->st_frames : 0.0,
                     enc->st_frames ? enc->st_hw_us / 1000.0 / enc->st_frames : 0.0,
                     enc->st_fail, enc->drops);
    else
        HAL_LOG_INFO("venc chn %d: %.2f fps, %.0f kbps, QP %d, %.1f ms hardware a frame, %u "
                     "failed, %u dropped",
                     chn, enc->st_frames / secs, enc->st_bytes * 8 / 1000.0 / secs, enc->st_qp,
                     enc->st_frames ? enc->st_hw_us / 1000.0 / enc->st_frames : 0.0,
                     enc->st_fail, enc->drops);
    enc->st_frames = enc->st_fail = 0;
    enc->st_bytes = enc->st_hw_us = 0;
    enc->st_since = now;
}

typedef struct {
    xm_state_t *st;
    int vi;
} xm_feeder_arg_t;

/* Encode one frame on one channel and file it */
static void xm_encode_one(xm_state_t *st, int chn, const VIDEO_FRAME_INFO_S *vf, bool idr,
                          int64_t now)
{
    xm_enc_chn_t *enc = &st->enc[chn];
    struct xmvenc_frame out;
    size_t n;

    if (idr)
        xmvenc_chn_request_idr(enc->xchn);
    memset(&out, 0, sizeof(out));
    if (xmvenc_encode(enc->xchn, vf->stVFrame.u32PhyAddr[0], vf->stVFrame.u32PhyAddr[1], &out)) {
        enc->st_fail++;
        if (enc->st_fail == 1)
            HAL_LOG_WARN("venc chn %d: encode failed (status 0x%x)", chn, out.status);
        return;
    }
    n = xm_file(st, chn, &out, now);
    enc->st_frames++;
    enc->st_bytes += n;
    enc->st_hw_us += (uint64_t)(out.us > 0 ? out.us : 0);
    enc->st_qp = out.qp;
    xm_stats(st, chn, now);
}

/*
 * The JPEG worker of a feeder: encodes the snapshot frame it was handed
 * while the feeder goes on with the next frames, then gives that frame
 * back to VI. libxmvenc lets channels on different threads take turns on
 * the encoder, so main's next frame waits at most one JPEG for it and
 * catches up in the frames after.
 */
static void *xm_jpeg_worker(void *p)
{
    xm_feeder_arg_t a = *(xm_feeder_arg_t *)p;
    xm_state_t *st = a.st;
    xm_feeder_t *fd = &st->feed[a.vi];

    free(p);
    pthread_mutex_lock(&st->enc_lock);
    for (;;) {
        VIDEO_FRAME_INFO_S *vf = (VIDEO_FRAME_INFO_S *)fd->jframe;
        int chn;
        int64_t ts;

        while (!fd->jbusy && !__atomic_load_n(&fd->stop, __ATOMIC_ACQUIRE))
            pthread_cond_wait(&st->enc_cond, &st->enc_lock);
        if (!fd->jbusy)
            break; /* stopping, nothing in hand */
        chn = fd->jchn;
        ts = fd->jts;
        pthread_mutex_unlock(&st->enc_lock);

        if (st->enc[chn].xchn)
            xm_encode_one(st, chn, vf, false, ts);
        XM_MPI_VI_ReleaseFrame(a.vi, vf);

        pthread_mutex_lock(&st->enc_lock);
        fd->jbusy = false;
        pthread_cond_broadcast(&st->enc_cond);
    }
    pthread_mutex_unlock(&st->enc_lock);
    return NULL;
}

static void *xm_feeder(void *p)
{
    xm_feeder_arg_t a = *(xm_feeder_arg_t *)p;
    xm_state_t *st = a.st;
    xm_feeder_t *fd = &st->feed[a.vi];
    double snr = st->mode ? st->mode->fps : 15;
    int misses = 0;

    free(p);
    while (!__atomic_load_n(&fd->stop, __ATOMIC_ACQUIRE)) {
        VIDEO_FRAME_INFO_S vf;
        int chn, idle = 1;
        bool handed = false;
        int64_t now;

        pthread_mutex_lock(&st->enc_lock);
        for (chn = 0; chn < XM_ENC_CHN_MAX; chn++)
            if (st->enc[chn].vi == a.vi && st->enc[chn].started)
                idle = 0;
        if (idle && !fd->grab_req) {
            struct timespec until;

            clock_gettime(CLOCK_REALTIME, &until);
            until.tv_nsec += 50000000L;
            if (until.tv_nsec >= 1000000000L) {
                until.tv_sec++;
                until.tv_nsec -= 1000000000L;
            }
            pthread_cond_timedwait(&st->enc_cond, &st->enc_lock, &until);
            pthread_mutex_unlock(&st->enc_lock);
            continue;
        }
        pthread_mutex_unlock(&st->enc_lock);

        memset(&vf, 0, sizeof(vf));
        if (XM_MPI_VI_GetFrame(a.vi, &vf) != XM_SUCCESS) {
            /* VI's get does not wait; a frame comes every 67 ms */
            if (++misses == 400)
                HAL_LOG_WARN("feeder %d: no VI frame for 2 s (is fs %d enabled?)", a.vi, a.vi);
            usleep(5000);
            continue;
        }
        misses = 0;
        now = now_us();

        pthread_mutex_lock(&fd->lock);
        for (chn = 0; chn < XM_ENC_CHN_MAX; chn++) {
            xm_enc_chn_t *enc = &st->enc[chn];
            bool go, idr, jpeg = is_jpeg(enc->codec);

            pthread_mutex_lock(&st->enc_lock);
            /* a JPEG goes to the worker, one at a time: while it is still
             * on the last one, this frame is not taken for the next */
            go = enc->vi == a.vi && enc->started && enc->xchn &&
                 vf.stVFrame.u32Width == enc->width && vf.stVFrame.u32Height == enc->height &&
                 !(jpeg && (fd->jbusy || handed)) &&
                 xm_due(enc, jpeg ? (enc->fps_num && enc->fps_den
                                         ? (double)enc->fps_num / enc->fps_den
                                         : 1.0)
                                  : xm_enc_rate(st, enc),
                        snr, now);
            idr = enc->want_idr;
            if (go)
                enc->want_idr = false;
            if (go && jpeg) {
                memcpy(fd->jframe, &vf, sizeof(vf));
                fd->jchn = chn;
                fd->jts = now;
                fd->jbusy = true;
                handed = true;
                pthread_cond_broadcast(&st->enc_cond);
            }
            pthread_mutex_unlock(&st->enc_lock);
            if (go && !jpeg)
                xm_encode_one(st, chn, &vf, idr, now);
        }

        if (fd->grab_req && !fd->grab_done) {
            xm_frame_copy(st, a.vi, &vf);
            pthread_mutex_lock(&st->enc_lock);
            fd->grab_done = true;
            pthread_cond_broadcast(&st->enc_cond);
            pthread_mutex_unlock(&st->enc_lock);
        }
        if (!handed) /* else the worker gives it back */
            XM_MPI_VI_ReleaseFrame(a.vi, &vf);
        pthread_mutex_unlock(&fd->lock);
    }
    return NULL;
}

static int xm_feeder_start(xm_state_t *st, int vi)
{
    xm_feeder_t *fd = &st->feed[vi];
    xm_feeder_arg_t *a, *b;

    if (fd->running)
        return RSS_OK;
    a = malloc(sizeof(*a));
    b = malloc(sizeof(*b));
    if (!a || !b) {
        free(a);
        free(b);
        return RSS_ERR_NOMEM;
    }
    a->st = b->st = st;
    a->vi = b->vi = vi;
    fd->stop = false;
    fd->jbusy = false;
    if (pthread_create(&fd->jthread, NULL, xm_jpeg_worker, b)) {
        free(a);
        free(b);
        HAL_LOG_ERR("feeder %d: pthread_create (JPEG worker) failed", vi);
        return RSS_ERR_IO;
    }
    fd->jrunning = true;
    if (pthread_create(&fd->thread, NULL, xm_feeder, a)) {
        free(a);
        HAL_LOG_ERR("feeder %d: pthread_create failed", vi);
        __atomic_store_n(&fd->stop, true, __ATOMIC_RELEASE);
        pthread_mutex_lock(&st->enc_lock);
        pthread_cond_broadcast(&st->enc_cond);
        pthread_mutex_unlock(&st->enc_lock);
        pthread_join(fd->jthread, NULL);
        fd->jrunning = false;
        return RSS_ERR_IO;
    }
    fd->running = true;
    HAL_LOG_INFO("feeder %d: VI channel %d feeds its encoders (JPEG on a worker)", vi, vi);
    return RSS_OK;
}

static void xm_feeder_stop(xm_state_t *st, int vi)
{
    xm_feeder_t *fd = &st->feed[vi];

    if (!fd->running && !fd->jrunning)
        return;
    __atomic_store_n(&fd->stop, true, __ATOMIC_RELEASE);
    pthread_mutex_lock(&st->enc_lock);
    pthread_cond_broadcast(&st->enc_cond);
    pthread_mutex_unlock(&st->enc_lock);
    if (fd->running)
        pthread_join(fd->thread, NULL);
    fd->running = false;
    /* the worker finishes a snapshot in hand (and gives its frame back)
     * before it sees the stop */
    if (fd->jrunning)
        pthread_join(fd->jthread, NULL);
    fd->jrunning = false;
}

void xm_feed_hold(xm_state_t *st, int vi)
{
    xm_feeder_t *fd = &st->feed[vi];

    pthread_mutex_lock(&fd->lock); /* the feeder between frames */
    pthread_mutex_lock(&st->enc_lock);
    while (fd->jbusy) /* and no snapshot frame out */
        pthread_cond_wait(&st->enc_cond, &st->enc_lock);
    pthread_mutex_unlock(&st->enc_lock);
}

void xm_feed_release(xm_state_t *st, int vi)
{
    pthread_mutex_unlock(&st->feed[vi].lock);
}

/* rvd's fs_get_frame on a fed channel: the feeder copies its next frame */
int xm_enc_grab(xm_state_t *st, int vi)
{
    xm_feeder_t *fd = &st->feed[vi];
    struct timespec until;
    int rc = 0;

    clock_gettime(CLOCK_REALTIME, &until);
    until.tv_nsec += 500000000L;
    if (until.tv_nsec >= 1000000000L) {
        until.tv_sec++;
        until.tv_nsec -= 1000000000L;
    }
    pthread_mutex_lock(&st->enc_lock);
    fd->grab_done = false;
    fd->grab_req = true;
    pthread_cond_broadcast(&st->enc_cond);
    while (!fd->grab_done && rc != ETIMEDOUT)
        rc = pthread_cond_timedwait(&st->enc_cond, &st->enc_lock, &until);
    rc = fd->grab_done ? RSS_OK : RSS_ERR_BUSY;
    fd->grab_req = false;
    pthread_mutex_unlock(&st->enc_lock);
    return rc;
}

/* ================================================================
 * HAL OPS
 * ================================================================ */

/* Groups are the framesources: nothing to make */
int hal_enc_create_group(void *ctx, int grp)
{
    xm_state_t *st = xm_state(ctx);

    return st && grp >= 0 && grp < XM_FS_CHN_MAX ? RSS_OK : RSS_ERR_INVAL;
}

int hal_enc_destroy_group(void *ctx, int grp)
{
    return hal_enc_create_group(ctx, grp);
}

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg)
{
    int rc;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!cfg)
        return RSS_ERR_INVAL;
    if (enc->created) {
        HAL_LOG_ERR("venc chn %d: already created", chn);
        return RSS_ERR_BUSY;
    }
    if (cfg->codec == RSS_CODEC_H265) {
        HAL_LOG_ERR("venc chn %d: H.265 (the Mali-V550) is not in this backend yet; use h264",
                    chn);
        return RSS_ERR_NOTSUP;
    }
    if (cfg->codec != RSS_CODEC_H264 && !is_jpeg(cfg->codec))
        return RSS_ERR_NOTSUP;
    if ((cfg->width & 7) || (cfg->height & 1) || cfg->width < 96 || cfg->height < 96) {
        HAL_LOG_ERR("venc chn %d: %ux%u -- the encoder takes widths in 8s, even heights, 96 "
                    "up",
                    chn, cfg->width, cfg->height);
        return RSS_ERR_INVAL;
    }
    rc = xm_open_venc(st);
    if (rc)
        return rc;
    if (cfg->width > (unsigned)xmvenc_max_width(st->venc)) {
        HAL_LOG_ERR("venc chn %d: %u wide is past the encoder's %d", chn, cfg->width,
                    xmvenc_max_width(st->venc));
        return RSS_ERR_INVAL;
    }

    pthread_mutex_lock(&st->enc_lock);
    enc->codec = is_jpeg(cfg->codec) ? RSS_CODEC_JPEG : RSS_CODEC_H264;
    enc->width = cfg->width;
    enc->height = cfg->height;
    enc->fps_num = cfg->fps_num;
    enc->fps_den = cfg->fps_den ? cfg->fps_den : 1;
    enc->gop = cfg->gop_length;
    enc->bitrate = cfg->bitrate;
    enc->rc_mode = cfg->rc_mode;
    enc->profile = cfg->profile;
    enc->init_qp = cfg->init_qp;
    enc->min_qp = cfg->min_qp;
    enc->max_qp = cfg->max_qp;
    enc->quality = is_jpeg(cfg->codec) && cfg->init_qp > 0 && cfg->init_qp <= 100 ? cfg->init_qp
                                                                                   : 80;
    enc->vi = -1;
    enc->held = -1;
    enc->q_head = enc->q_count = 0;
    enc->seq = enc->drops = 0;
    enc->first_logged = false;
    enc->next_due = 0;
    enc->st_since = 0;
    pthread_mutex_unlock(&st->enc_lock);

    if (!is_jpeg(cfg->codec) && cfg->profile >= 2)
        HAL_LOG_WARN("venc chn %d: High profile asked; the H1 encodes Main (CABAC)", chn);

    rc = xm_enc_make(st, chn);
    if (rc)
        return rc;
    enc->created = true;
    return RSS_OK;
}

int hal_enc_destroy_channel(void *ctx, int chn)
{
    int vi;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!enc->created)
        return RSS_OK;
    hal_enc_stop(ctx, chn);
    vi = enc->vi;
    if (vi >= 0)
        xm_feed_hold(st, vi);
    pthread_mutex_lock(&st->enc_lock);
    enc->vi = -1;
    enc->created = false;
    pthread_mutex_unlock(&st->enc_lock);
    if (enc->xchn)
        xmvenc_chn_destroy(enc->xchn);
    enc->xchn = NULL;
    if (vi >= 0)
        xm_feed_release(st, vi);
    return RSS_OK;
}

static int xm_bind(xm_state_t *st, int vi, int chn)
{
    xm_enc_chn_t *enc = &st->enc[chn];

    if (vi < 0 || vi >= XM_FS_CHN_MAX || !enc->created)
        return RSS_ERR_INVAL;
    if (enc->vi == vi)
        return RSS_OK;
    if (enc->vi >= 0) {
        HAL_LOG_ERR("venc chn %d: already on framesource %d", chn, enc->vi);
        return RSS_ERR_BUSY;
    }
    if (st->fs[vi].created &&
        (st->fs[vi].width != enc->width || st->fs[vi].height != enc->height)) {
        HAL_LOG_ERR("venc chn %d: %ux%u, but framesource %d gives %ux%u; this encoder does "
                    "not scale",
                    chn, enc->width, enc->height, vi, st->fs[vi].width, st->fs[vi].height);
        return RSS_ERR_INVAL;
    }
    pthread_mutex_lock(&st->enc_lock);
    enc->vi = vi;
    pthread_mutex_unlock(&st->enc_lock);
    HAL_LOG_INFO("bind: VI %d -> venc %d (%s)", vi, chn, is_jpeg(enc->codec) ? "JPEG" : "H.264");
    return xm_feeder_start(st, vi);
}

static int xm_unbind(xm_state_t *st, int chn)
{
    xm_enc_chn_t *enc = &st->enc[chn];
    int vi = enc->vi;

    if (vi < 0)
        return RSS_OK;
    xm_feed_hold(st, vi);
    pthread_mutex_lock(&st->enc_lock);
    enc->vi = -1;
    pthread_mutex_unlock(&st->enc_lock);
    xm_feed_release(st, vi);
    return RSS_OK;
}

int hal_enc_register_channel(void *ctx, int grp, int chn)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    (void)enc;
    return xm_bind(st, grp, chn);
}

int hal_enc_unregister_channel(void *ctx, int chn)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    (void)enc;
    return xm_unbind(st, chn);
}

/* rvd's FS -> [OSD ->] ENC chain, collapsed to one bind (no OSD yet) */
static int xm_bind_collapse(xm_state_t *st, const rss_cell_t *src, const rss_cell_t *dst,
                            int *fs, int *chn, bool *collapsed)
{
    *collapsed = false;
    if (!src || !dst)
        return RSS_ERR_INVAL;
    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_ENC) {
        *fs = src->group;
        *chn = dst->group;
    } else if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_OSD) {
        if (dst->group < 0 || dst->group >= XM_ENC_CHN_MAX)
            return RSS_ERR_INVAL;
        st->osd_src_fs[dst->group] = src->group;
        *collapsed = true;
        return RSS_OK;
    } else if (src->device == RSS_DEV_OSD && dst->device == RSS_DEV_ENC) {
        if (src->group < 0 || src->group >= XM_ENC_CHN_MAX)
            return RSS_ERR_INVAL;
        *fs = st->osd_src_fs[src->group];
        *chn = dst->group;
        if (*fs < 0)
            return RSS_ERR_INVAL;
    } else {
        HAL_LOG_ERR("bind: FS -> [OSD ->] ENC is the only chain this backend supports");
        return RSS_ERR_NOTSUP;
    }
    return *chn >= 0 && *chn < XM_ENC_CHN_MAX ? RSS_OK : RSS_ERR_INVAL;
}

int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    xm_state_t *st = xm_state(ctx);
    bool collapsed;
    int fs = -1, chn = -1, rc;

    if (!st || !st->enc_ready)
        return RSS_ERR_INVAL;
    rc = xm_bind_collapse(st, src, dst, &fs, &chn, &collapsed);
    if (rc || collapsed)
        return rc;
    return xm_bind(st, fs, chn);
}

int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    xm_state_t *st = xm_state(ctx);
    bool collapsed;
    int fs = -1, chn = -1, rc;

    if (!st || !st->enc_ready)
        return RSS_ERR_INVAL;
    rc = xm_bind_collapse(st, src, dst, &fs, &chn, &collapsed);
    if (rc || collapsed)
        return rc;
    return xm_unbind(st, chn);
}

int hal_enc_start(void *ctx, int chn)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!enc->created)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&st->enc_lock);
    if (!enc->started) {
        enc->started = true;
        enc->next_due = 0;
        enc->want_idr = !is_jpeg(enc->codec);
        enc->st_since = 0;
    }
    pthread_cond_broadcast(&st->enc_cond);
    pthread_mutex_unlock(&st->enc_lock);
    return RSS_OK;
}

int hal_enc_stop(void *ctx, int chn)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    pthread_mutex_lock(&st->enc_lock);
    enc->started = false;
    enc->q_count = 0; /* what was queued is stale by the next start */
    pthread_cond_broadcast(&st->enc_cond);
    pthread_mutex_unlock(&st->enc_lock);
    return RSS_OK;
}

int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms)
{
    struct timespec until;
    int rc = 0;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!enc->created)
        return RSS_ERR_INVAL;
    clock_gettime(CLOCK_REALTIME, &until);
    until.tv_sec += timeout_ms / 1000u;
    until.tv_nsec += (long)(timeout_ms % 1000u) * 1000000L;
    if (until.tv_nsec >= 1000000000L) {
        until.tv_sec++;
        until.tv_nsec -= 1000000000L;
    }
    pthread_mutex_lock(&st->enc_lock);
    while (!enc->q_count && rc != ETIMEDOUT && enc->created)
        rc = pthread_cond_timedwait(&st->enc_cond, &st->enc_lock, &until);
    rc = enc->q_count ? RSS_OK : -EAGAIN;
    pthread_mutex_unlock(&st->enc_lock);
    return rc;
}

int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame)
{
    xm_frame_t *f;
    int s;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!frame || !enc->created)
        return RSS_ERR_INVAL;
    pthread_mutex_lock(&st->enc_lock);
    if (enc->held >= 0) {
        pthread_mutex_unlock(&st->enc_lock);
        HAL_LOG_ERR("venc chn %d: get_frame with a frame still held", chn);
        return RSS_ERR_BUSY;
    }
    if (!enc->q_count) {
        pthread_mutex_unlock(&st->enc_lock);
        return -EAGAIN;
    }
    s = enc->q[enc->q_head];
    enc->q_head = (enc->q_head + 1) % XM_ENC_QDEPTH;
    enc->q_count--;
    enc->held = s;
    f = &enc->slots[s];
    frame->nals = f->nals;
    frame->nal_count = f->nal_count;
    frame->codec = enc->codec;
    frame->timestamp = f->ts;
    frame->seq = f->seq;
    frame->is_key = f->key;
    frame->_priv = f;
    pthread_mutex_unlock(&st->enc_lock);
    return RSS_OK;
}

int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    pthread_mutex_lock(&st->enc_lock);
    enc->held = -1;
    pthread_mutex_unlock(&st->enc_lock);
    if (frame) {
        frame->nals = NULL;
        frame->nal_count = 0;
        frame->_priv = NULL;
    }
    return RSS_OK;
}

int hal_enc_request_idr(void *ctx, int chn)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!enc->created)
        return RSS_ERR_INVAL;
    if (is_jpeg(enc->codec))
        return RSS_OK;
    pthread_mutex_lock(&st->enc_lock);
    enc->want_idr = true;
    pthread_mutex_unlock(&st->enc_lock);
    return RSS_OK;
}

int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate)
{
    rss_rc_mode_t old_mode;
    uint32_t old_rate;
    int rc;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!enc->created || is_jpeg(enc->codec))
        return RSS_ERR_INVAL;
    old_mode = enc->rc_mode;
    old_rate = enc->bitrate;
    enc->rc_mode = mode;
    if (bitrate)
        enc->bitrate = bitrate;
    rc = xm_enc_remake(st, chn);
    if (rc) {
        enc->rc_mode = old_mode;
        enc->bitrate = old_rate;
        xm_enc_remake(st, chn);
    }
    return rc;
}

int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate)
{
    uint32_t old;
    int rc;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!enc->created || is_jpeg(enc->codec) || !bitrate)
        return RSS_ERR_INVAL;
    if (bitrate == enc->bitrate)
        return RSS_OK;
    old = enc->bitrate;
    enc->bitrate = bitrate;
    rc = xm_enc_remake(st, chn);
    if (rc) {
        enc->bitrate = old;
        xm_enc_remake(st, chn);
    }
    return rc;
}

int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length)
{
    uint32_t old;
    int rc;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!enc->created || is_jpeg(enc->codec) || !gop_length)
        return RSS_ERR_INVAL;
    if (gop_length == enc->gop)
        return RSS_OK;
    old = enc->gop;
    enc->gop = gop_length;
    rc = xm_enc_remake(st, chn);
    if (rc) {
        enc->gop = old;
        xm_enc_remake(st, chn);
    }
    return rc;
}

/* rvd passes JPEG quality, 1..100 higher-better, despite the name */
int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp)
{
    int old, rc;

    XM_ENC_ENTER(ctx, chn, st, enc);
    if (qp < 1 || qp > 100)
        return RSS_ERR_INVAL;
    if (!enc->created || !is_jpeg(enc->codec))
        return RSS_ERR_NOTSUP;
    if (qp == enc->quality)
        return RSS_OK;
    old = enc->quality;
    enc->quality = qp;
    rc = xm_enc_remake(st, chn);
    if (rc) {
        enc->quality = old;
        xm_enc_remake(st, chn);
    }
    return rc;
}

int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!qp)
        return RSS_ERR_INVAL;
    if (!enc->created || !is_jpeg(enc->codec))
        return RSS_ERR_NOTSUP;
    *qp = enc->quality;
    return RSS_OK;
}

/* The rate frames come at: the configured one, capped at the sensor's */
int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;
    if (is_jpeg(enc->codec)) {
        *fps_num = enc->fps_num ? enc->fps_num : 1;
        *fps_den = enc->fps_den ? enc->fps_den : 1;
    } else {
        double r = xm_enc_rate(st, enc);

        *fps_num = (uint32_t)(r * 1000 + 0.5);
        *fps_den = 1000;
        if (*fps_num % 1000 == 0) {
            *fps_num /= 1000;
            *fps_den = 1;
        }
    }
    return RSS_OK;
}

/* The configured target, as the other backends report it */
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate)
{
    XM_ENC_ENTER(ctx, chn, st, enc);
    if (!bitrate)
        return RSS_ERR_INVAL;
    *bitrate = enc->bitrate;
    return RSS_OK;
}
