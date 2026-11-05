/*
 * fhan_v2/fh_encoder.c -- Phase 2b: framesources, encoders, stream delivery.
 *
 * MAPPING. raptor framesource N == VPSS output channel N; raptor encoder
 * channel N == VENC channel N. A VENC channel takes pictures from a VPSS
 * channel through FH_SYS_BindVpu2Enc, so nothing here moves pixels.
 *
 * THE ONE STRUCTURAL DIFFERENCE FROM EVERY OTHER BACKEND. FH_VENC_GetStream
 * is not per channel: it takes a codec mask and returns the next frame from
 * whichever channel has one, naming the channel in the result (field04).
 * rvd runs one thread per encoder channel, each polling its own channel. So
 * this file runs one collector thread that drains the driver, copies each
 * frame out (the driver's buffer is released straight away), and files it
 * in its channel's small FIFO; enc_poll waits on that FIFO, enc_get_frame
 * hands out its head. divinus's own delivery thread copies every frame
 * too, so the copy costs nothing it did not already pay.
 *
 * RATE CONTROL. CBR is the driver's CBR for both codecs. divinus 2c0a8bc
 * replaces H.264 CBR with a fixed quantizer, reporting that the FH8852's
 * loop ignores its target; a fixed quantizer has no ceiling at all, and on
 * the bench it produced 1.75 MB frames from a dark, noisy scene. So CBR is
 * CBR here, and the collector logs each channel's real bitrate every 10 s
 * to show whether the loop holds. rc_mode fixqp keeps Sergey's mapping
 * (38 at 1 Mbps .. 32 at 8, never below 30) when init_qp is not given.
 *
 * JPEG. Every JPEG-class channel is a continuously running MJPEG channel at
 * the snapshot rate, divinus's MJPEG attribute; rvd starts and stops it.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _POSIX_C_SOURCE 200809L

#ifndef HAL_MODULE_VIDEO
#define HAL_MODULE_VIDEO
#endif

#include "fh_state.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern long syscall(long number, ...);
#include <sys/syscall.h>

static int64_t fh_now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

#define FH_ENC_ENTER(ctx, chn, st, enc)                                                            \
    fh_state_t *st = fh_state(ctx);                                                                \
    fh_enc_chn_t *enc;                                                                             \
    do {                                                                                           \
        if (!st || !st->enc_sync_ready)                                                            \
            return RSS_ERR_INVAL;                                                                  \
        if ((chn) < 0 || (chn) >= FH_VENC_CHN_MAX)                                                 \
            return RSS_ERR_INVAL;                                                                  \
        enc = &st->enc[(chn)];                                                                     \
    } while (0)

/* ================================================================
 * NAL CLASSIFICATION
 * ================================================================ */

/* Offset of the NAL header past an Annex B start code, or 0 if none. */
static size_t fh_sc_len(const uint8_t *d, size_t n)
{
    if (n >= 4 && !d[0] && !d[1] && !d[2] && d[3] == 1)
        return 4;
    if (n >= 3 && !d[0] && !d[1] && d[2] == 1)
        return 3;
    return 0;
}

static rss_nal_type_t fh_nal_type(rss_codec_t codec, uint8_t hdr)
{
    if (codec == RSS_CODEC_H265) {
        unsigned int t = (hdr >> 1) & 0x3f;

        if (t == 32)
            return RSS_NAL_H265_VPS;
        if (t == 33)
            return RSS_NAL_H265_SPS;
        if (t == 34)
            return RSS_NAL_H265_PPS;
        if (t == 39 || t == 40)
            return RSS_NAL_H265_SEI;
        if (t >= 16 && t <= 21)
            return RSS_NAL_H265_IDR;
        if (t < 32)
            return RSS_NAL_H265_SLICE;
        return RSS_NAL_UNKNOWN;
    }

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

/* ================================================================
 * THE COLLECTOR
 * ================================================================ */

static int fh_slot_free(const fh_enc_chn_t *enc)
{
    int s, i;

    for (s = 0; s < FH_ENC_QDEPTH + 1; s++) {
        bool used = (s == enc->held);

        for (i = 0; i < enc->q_count && !used; i++)
            used = enc->q[(enc->q_head + i) % FH_ENC_QDEPTH] == s;
        if (!used)
            return s;
    }
    return -1;
}

static bool fh_slot_reserve(fh_frame_t *f, size_t need)
{
    if (need <= f->cap)
        return true;
    {
        uint8_t *b = (uint8_t *)realloc(f->buf, need + 16384);

        if (!b)
            return false;
        f->buf = b;
        f->cap = need + 16384;
    }
    return true;
}

typedef struct {
    const uint8_t *data;
    size_t len;
} fh_piece;

/* Copy one driver frame into a free slot of its channel, under enc_lock.
 * Returns true if a frame had to be dropped to make room. */
static bool fh_file_frame(fh_state_t *st, int chn, const fh_piece *pc, unsigned int npc)
{
    fh_enc_chn_t *enc = &st->enc[chn];
    bool jpeg = enc->codec == RSS_CODEC_JPEG || enc->codec == RSS_CODEC_MJPEG;
    bool dropped = false;
    fh_frame_t *f;
    size_t need = 0, off = 0;
    unsigned int i;
    int s;

    if (enc->q_count == FH_ENC_QDEPTH) {
        enc->q_head = (enc->q_head + 1) % FH_ENC_QDEPTH;
        enc->q_count--;
        enc->drops++;
        dropped = true;
    }
    s = fh_slot_free(enc);
    if (s < 0)
        return dropped;
    f = &enc->slots[s];

    for (i = 0; i < npc; i++)
        need += pc[i].len + 4;
    if (!fh_slot_reserve(f, need))
        return dropped;

    f->nal_count = 0;
    f->key = false;
    for (i = 0; i < npc && f->nal_count < FH_ENC_MAX_NALS; i++) {
        rss_nal_unit_t *nal = &f->nals[f->nal_count];
        size_t sc = jpeg ? 0 : fh_sc_len(pc[i].data, pc[i].len);
        size_t start = off;

        if (!pc[i].len)
            continue;
        /* raptor's NALs carry their Annex B start code, as every other
         * backend's do. Add one if the driver did not. */
        if (!jpeg && !sc) {
            static const uint8_t code[4] = {0, 0, 0, 1};

            memcpy(f->buf + off, code, 4);
            off += 4;
            sc = 4;
            if (!enc->sc_logged) {
                enc->sc_logged = true;
                HAL_LOG_INFO("venc chn %d: NALs arrive without start codes; adding them", chn);
            }
        } else if (!jpeg && !enc->sc_logged) {
            enc->sc_logged = true;
            HAL_LOG_INFO("venc chn %d: NALs arrive with %zu-byte start codes", chn, sc);
        }
        memcpy(f->buf + off, pc[i].data, pc[i].len);
        off += pc[i].len;

        nal->data = f->buf + start;
        nal->length = (uint32_t)(off - start);
        if (jpeg) {
            nal->type = RSS_NAL_JPEG_FRAME;
            f->key = true;
        } else {
            size_t hdr = (size_t)(nal->length > sc ? sc : 0);

            nal->type = hdr ? fh_nal_type(enc->codec, nal->data[hdr]) : RSS_NAL_UNKNOWN;
            if (nal->type == RSS_NAL_H264_IDR || nal->type == RSS_NAL_H265_IDR)
                f->key = true;
        }
        nal->frame_end = false;
        f->nal_count++;
    }
    if (!f->nal_count)
        return dropped;
    f->nals[f->nal_count - 1].frame_end = true;
    f->len = off;
    f->ts = fh_now_us();
    f->seq = enc->seq++;

    enc->q[(enc->q_head + enc->q_count) % FH_ENC_QDEPTH] = s;
    enc->q_count++;
    return dropped;
}

/*
 * fh_map_find -- the mapping that contains addr, from /proc/self/maps.
 *
 * Bench, 2026-11-05: the collector's memcpy faulted reading the first byte
 * past the end of the mapping holding a NAL, on a frame whose NAL lengths
 * were consistent with the frame's own size (big, noisy frames under the
 * night tuning). That is a NAL running off the end of the encoder's
 * stream buffer -- a wrap -- and nothing in the stream header marks it.
 * The collector therefore checks every NAL against the mapping it starts
 * in before copying it.
 */
static int fh_map_find(uintptr_t addr, uintptr_t *lo, uintptr_t *hi)
{
    char line[256];
    FILE *f = fopen("/proc/self/maps", "r");
    int found = -1;

    if (!f)
        return -1;
    while (fgets(line, sizeof(line), f)) {
        unsigned long a, b;

        if (sscanf(line, "%lx-%lx", &a, &b) != 2)
            continue;
        if (addr >= a && addr < b) {
            *lo = a;
            *hi = b;
            found = 0;
            break;
        }
    }
    fclose(f);
    return found;
}

static void *fh_collect_thread(void *arg)
{
    fh_state_t *st = (fh_state_t *)arg;
    const unsigned int mask = FH_CODEC_H264 | FH_CODEC_H265 | FH_CODEC_MJPEG;
    fh_venc_strm strm;
    fh_piece pc[40];
    int64_t last_drop_log = 0, last_bad_log = 0, last_wrap_log = 0;
    unsigned int bad_frames = 0, wrap_frames = 0;
    uintptr_t map_lo = 0, map_hi = 0;
    unsigned long long win_bytes[FH_VENC_CHN_MAX] = {0};
    unsigned int win_frames[FH_VENC_CHN_MAX] = {0}, win_max[FH_VENC_CHN_MAX] = {0};
    int64_t win_start = fh_now_us();

    HAL_LOG_INFO("venc: collector thread tid %ld", syscall(SYS_gettid));

    while (__atomic_load_n(&st->collect_run, __ATOMIC_ACQUIRE)) {
        unsigned int chn, npc = 0, i;
        bool dropped = false;
        fh_enc_chn_t *enc;

        memset(&strm, 0, sizeof(strm));
        strm.type = mask;
        if (st->pipe.fnVencGetStream(mask, &strm)) {
            fh_sleep_ms(5);
            continue;
        }

        chn = strm.field04;
        if (chn >= FH_VENC_CHN_MAX) {
            HAL_LOG_WARN("venc: stream names channel %u; releasing", chn);
            st->pipe.fnVencRelease(chn);
            continue;
        }
        enc = &st->enc[chn];

        if (strm.type == FH_CODEC_MJPEG || strm.type == FH_CODEC_JPEG) {
            pc[0].data = (const uint8_t *)(uintptr_t)strm.field08;
            pc[0].len = strm.field0c;
            npc = pc[0].data ? 1 : 0;
        } else {
            unsigned int n = strm.nalu_cnt > 40 ? 40 : strm.nalu_cnt;
            unsigned long long total = 0;

            for (i = 0; i < n; i++) {
                uintptr_t a = (uintptr_t)strm.nalu[i].addr;
                size_t len = strm.nalu[i].length;

                if (!a || !len)
                    continue;

                /* Inside the mapping it starts in, or not copied at all. */
                if (a < map_lo || a >= map_hi) {
                    if (fh_map_find(a, &map_lo, &map_hi)) {
                        map_lo = map_hi = 0;
                        HAL_LOG_WARN("venc chn %u: NAL at %08lx is in no mapping; frame dropped",
                                     strm.field04, (unsigned long)a);
                        npc = 0;
                        wrap_frames++;
                        break;
                    }
                    HAL_LOG_INFO("venc: stream data mapping %08lx-%08lx (%lu KiB)",
                                 (unsigned long)map_lo, (unsigned long)map_hi,
                                 (unsigned long)((map_hi - map_lo) / 1024));
                }
                if (len > map_hi - a) {
                    int64_t now = fh_now_us();

                    wrap_frames++;
                    if (now - last_wrap_log > 2000000) {
                        last_wrap_log = now;
                        HAL_LOG_WARN("venc chn %u: NAL %u (type %u, %zu bytes at %08lx) runs %zu "
                                     "bytes past the end of its mapping %08lx-%08lx; frame "
                                     "dropped (%u so far). header %08x %08x %08x %08x %08x "
                                     "nalu_cnt %u",
                                     strm.field04, i, strm.nalu[i].type, len, (unsigned long)a,
                                     len - (size_t)(map_hi - a), (unsigned long)map_lo,
                                     (unsigned long)map_hi, wrap_frames, strm.field08,
                                     strm.field0c, strm.field10, strm.field14, strm.field18,
                                     strm.nalu_cnt);
                    }
                    npc = 0;
                    break;
                }

                pc[npc].data = (const uint8_t *)a;
                pc[npc].len = len;
                total += len;
                npc++;
            }
            if (!npc && n && chn < FH_VENC_CHN_MAX &&
                (st->enc[chn].codec == RSS_CODEC_H264 || st->enc[chn].codec == RSS_CODEC_H265))
                st->pipe.fnVencRequestIdr(chn);

            /*
             * Bench, 2026-11-05: the collector died in memcpy reading past
             * the end of a vendor mapping with ~570 KB still to copy -- a
             * NAL whose length does not fit its buffer. field10 is the
             * frame's byte count (123483 against NALs summing to 123460 on
             * the first IDR), so a frame whose NALs add up to more than that
             * is not copied: it is logged whole, released and skipped. So is
             * anything over 1.5 MB, which no frame of these settings comes
             * near and which is most of the 2.5 MB stream buffer.
             */
            if (npc && (total > 1572864u ||
                        (strm.field10 && total > (unsigned long long)strm.field10 + 4096u))) {
                int64_t now = fh_now_us();

                if (now - last_bad_log > 2000000) {
                    last_bad_log = now;
                    HAL_LOG_WARN("venc chn %u: frame claims %llu bytes in %u NALs against a "
                                 "%u-byte frame; dropped. header %08x %08x %08x %08x %08x %08x "
                                 "%08x",
                                 strm.field04, total, n, strm.field10, strm.type, strm.field04,
                                 strm.field08, strm.field0c, strm.field10, strm.field14,
                                 strm.field18);
                    for (i = 0; i < n && i < 6; i++)
                        HAL_LOG_WARN("venc chn %u:   nalu %u: type %u len %u addr %08x",
                                     strm.field04, i, strm.nalu[i].type, strm.nalu[i].length,
                                     strm.nalu[i].addr);
                }
                bad_frames++;
                npc = 0;
                if (chn < FH_VENC_CHN_MAX &&
                    (st->enc[chn].codec == RSS_CODEC_H264 || st->enc[chn].codec == RSS_CODEC_H265))
                    st->pipe.fnVencRequestIdr(chn);
            }
        }

        pthread_mutex_lock(&st->enc_lock);
        if (!enc->first_logged) {
            enc->first_logged = true;
            HAL_LOG_INFO("venc chn %u: first frame: type 0x%x words %08x %08x %08x %08x %08x %08x "
                         "nalu_cnt %u",
                         chn, strm.type, strm.field04, strm.field08, strm.field0c, strm.field10,
                         strm.field14, strm.field18, strm.nalu_cnt);
            for (i = 0; i < npc && i < 4; i++)
                HAL_LOG_INFO("venc chn %u:   piece %u: %zu bytes at %p, %02x %02x %02x %02x %02x",
                             chn, i, pc[i].len, (const void *)pc[i].data, pc[i].len > 0 ? pc[i].data[0] : 0,
                             pc[i].len > 1 ? pc[i].data[1] : 0, pc[i].len > 2 ? pc[i].data[2] : 0,
                             pc[i].len > 3 ? pc[i].data[3] : 0, pc[i].len > 4 ? pc[i].data[4] : 0);
        }
        if (enc->created && enc->started && npc)
            dropped = fh_file_frame(st, (int)chn, pc, npc);
        pthread_mutex_unlock(&st->enc_lock);

        st->pipe.fnVencRelease(strm.field04);
        pthread_cond_broadcast(&st->enc_cond);

        /* What the encoder actually produces, per channel, every 10 s. */
        if (chn < FH_VENC_CHN_MAX && strm.field10) {
            win_bytes[chn] += strm.field10;
            win_frames[chn]++;
            if (strm.field10 > win_max[chn])
                win_max[chn] = strm.field10;
        }
        {
            int64_t now = fh_now_us();

            if (now - win_start >= 10000000) {
                unsigned int c;

                for (c = 0; c < FH_VENC_CHN_MAX; c++) {
                    if (!win_frames[c])
                        continue;
                    HAL_LOG_INFO("venc chn %u: %llu kbit/s, %u frames, largest %u bytes "
                                 "(target %u kbit/s)",
                                 c, win_bytes[c] * 8u / (unsigned long long)((now - win_start) / 1000),
                                 win_frames[c], win_max[c], st->enc[c].bitrate / 1000u);
                    win_bytes[c] = 0;
                    win_frames[c] = 0;
                    win_max[c] = 0;
                }
                win_start = now;
            }
        }

        /* A dropped H.26x frame is a hole in the reference chain; ask for a
         * keyframe so the stream heals at the next GOP's worth of time
         * rather than at the next scheduled IDR. */
        if (dropped) {
            int64_t now = fh_now_us();

            if (enc->codec == RSS_CODEC_H264 || enc->codec == RSS_CODEC_H265)
                st->pipe.fnVencRequestIdr(chn);
            if (now - last_drop_log > 5000000) {
                last_drop_log = now;
                HAL_LOG_WARN("venc chn %u: consumer behind, %u frame(s) dropped so far", chn,
                             enc->drops);
            }
        }
    }

    if (bad_frames || wrap_frames)
        HAL_LOG_WARN("venc: frames dropped: %u inconsistent, %u past their mapping", bad_frames,
                     wrap_frames);
    __atomic_store_n(&st->collect_done, 1, __ATOMIC_RELEASE);
    return NULL;
}

static int fh_collect_start(fh_state_t *st)
{
    int rc;

    if (st->collect_started)
        return RSS_OK;
    __atomic_store_n(&st->collect_run, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&st->collect_done, 0, __ATOMIC_RELEASE);
    rc = pthread_create(&st->collect_thread, NULL, fh_collect_thread, st);
    if (rc) {
        HAL_LOG_ERR("venc: cannot start the stream collector: %s", strerror(rc));
        return RSS_ERR_IO;
    }
    st->collect_started = true;
    return RSS_OK;
}

static void fh_collect_stop(fh_state_t *st)
{
    int waited;

    if (!st->collect_started)
        return;
    __atomic_store_n(&st->collect_run, 0, __ATOMIC_RELEASE);
    for (waited = 0; waited < 2000; waited += 10) {
        if (__atomic_load_n(&st->collect_done, __ATOMIC_ACQUIRE)) {
            pthread_join(st->collect_thread, NULL);
            st->collect_started = false;
            return;
        }
        fh_sleep_ms(10);
    }
    HAL_LOG_ERR("venc: the stream collector did not stop within 2 s; detaching it");
    pthread_detach(st->collect_thread);
    st->collect_started = false;
}

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

int fh_enc_init(fh_state_t *st)
{
    pthread_condattr_t ca;
    int i;

    if (st->enc_sync_ready)
        return RSS_OK;
    if (pthread_mutex_init(&st->enc_lock, NULL))
        return RSS_ERR_IO;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    if (pthread_cond_init(&st->enc_cond, &ca)) {
        pthread_condattr_destroy(&ca);
        pthread_mutex_destroy(&st->enc_lock);
        return RSS_ERR_IO;
    }
    pthread_condattr_destroy(&ca);

    for (i = 0; i < FH_VENC_CHN_MAX; i++) {
        st->enc[i].bound_fs = -1;
        st->enc[i].held = -1;
        st->osd_src_fs[i] = -1;
    }
    for (i = 0; i < FH_VPSS_CHN_MAX; i++)
        st->fs_bound_codec[i] = -1;
    st->enc_sync_ready = true;
    return RSS_OK;
}

void fh_enc_teardown(fh_state_t *st)
{
    int i, s;

    if (!st->enc_sync_ready)
        return;
    fh_collect_stop(st);

    for (i = 0; i < FH_VENC_CHN_MAX; i++) {
        fh_enc_chn_t *enc = &st->enc[i];

        if (enc->receiving)
            st->pipe.fnVencStop((unsigned)i);
        if (enc->bound_fs >= 0)
            st->pipe.fnUnbind((unsigned)i);
        for (s = 0; s < FH_ENC_QDEPTH + 1; s++)
            free(enc->slots[s].buf);
        memset(enc, 0, sizeof(*enc));
    }
    pthread_cond_destroy(&st->enc_cond);
    pthread_mutex_destroy(&st->enc_lock);
    st->enc_sync_ready = false;
}

/* ================================================================
 * FRAMESOURCE OPS
 * ================================================================ */

int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    fh_state_t *st = fh_state(ctx);
    unsigned int w, h, fps;

    if (!st || !cfg || !st->mode)
        return RSS_ERR_INVAL;
    if (chn < 0 || chn >= st->vpss_count) {
        HAL_LOG_ERR("fs %d: the V200 has %d VPSS output channels", chn, st->vpss_count);
        return RSS_ERR_INVAL;
    }

    w = cfg->width;
    h = cfg->height;
    if (!w || !h || w > st->mode->width || h > st->mode->height) {
        HAL_LOG_WARN("fs %d: %ux%u does not fit the %ux%u sensor mode; using the sensor size", chn,
                     w, h, st->mode->width, st->mode->height);
        w = st->mode->width;
        h = st->mode->height;
    }
    fps = cfg->fps_den ? cfg->fps_num / cfg->fps_den : 0;

    return fh_vpss_configure(st, chn, w, h, fps);
}

int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    return hal_fs_create_channel(ctx, chn, cfg);
}

/* fh_vpss_configure leaves the channel open, so enable has nothing to do
 * beyond confirming it. */
int hal_fs_enable_channel(void *ctx, int chn)
{
    fh_state_t *st = fh_state(ctx);

    if (!st || chn < 0 || chn >= FH_VPSS_CHN_MAX)
        return RSS_ERR_INVAL;
    return st->vpss_open[chn] ? RSS_OK : RSS_ERR_INVAL;
}

/*
 * Channel 0 stays open: the ISP only streams while a VPSS channel is open
 * (divinus), and the day/night and exposure work runs whether or not a
 * consumer wants pictures. Other channels close.
 */
int hal_fs_disable_channel(void *ctx, int chn)
{
    fh_state_t *st = fh_state(ctx);

    if (!st || chn < 0 || chn >= FH_VPSS_CHN_MAX)
        return RSS_ERR_INVAL;
    if (chn == 0 || !st->vpss_open[chn])
        return RSS_OK;
    st->pipe.fnVpssCloseChn((unsigned)chn);
    st->vpss_open[chn] = false;
    return RSS_OK;
}

int hal_fs_destroy_channel(void *ctx, int chn)
{
    return hal_fs_disable_channel(ctx, chn);
}

/* ================================================================
 * ENCODER CONFIGURATION
 * ================================================================ */

/* Sergey Novak's mapping (divinus 2c0a8bc): 42 at 256 kbps, two steps per
 * doubling, one more in the upper half of the octave, never below 30. */
static unsigned int fh_h264_qp_for(uint32_t bitrate_bps)
{
    unsigned int kbps = bitrate_bps ? bitrate_bps / 1000u : 1024u;
    unsigned int log = 0, x;
    int qp = 42;

    if (kbps < 256)
        kbps = 256;
    for (x = kbps; x > 1; x >>= 1)
        log++;
    if (log > 8) {
        qp -= 2 * (int)(log - 8);
        if (kbps & (1u << (log - 1)))
            qp--;
    }
    return qp < 30 ? 30u : (unsigned int)qp;
}

static void fh_fill_rc(fh_enc_chn_t *enc, fh_venc_rc *rc, int chn)
{
    bool h265 = enc->codec == RSS_CODEC_H265;
    unsigned int fps = enc->fps_den ? enc->fps_num / enc->fps_den : 25;
    unsigned int frw = (fps ? fps : 25) | (1u << 16);
    unsigned int kbps = enc->bitrate ? enc->bitrate / 1000u : 1024u;

    memset(rc, 0, sizeof(*rc));

    if (enc->rc_mode == RSS_RC_VBR) {
        rc->mode = h265 ? FH_RC_H265VBR : FH_RC_H264VBR;
        rc->param[0] = 35;
        rc->param[1] = kbps << 10;
        rc->param[2] = frw;
        rc->param[3] = 51;
        rc->param[6] = 5;
        rc->param[7] = 1;
        return;
    }

    if (enc->rc_mode == RSS_RC_FIXQP) {
        unsigned int qp = enc->init_qp >= 0 ? (unsigned int)enc->init_qp
                                            : fh_h264_qp_for(enc->bitrate);

        if (h265) {
            HAL_LOG_WARN("venc chn %d: no fixed-QP mode for H.265 here; using CBR", chn);
        } else {
            rc->mode = FH_RC_H264FIXQP;
            rc->param[0] = qp;
            rc->param[1] = qp;
            rc->param[2] = frw;
            HAL_LOG_INFO("venc chn %d: H.264 at fixed QP %u", chn, qp);
            return;
        }
    }

    /*
     * CBR, for both codecs, with the vendor's own CBR layout (divinus
     * f2_venc.h). Not the fixed-QP stand-in divinus 2c0a8bc uses for H.264:
     * bench, 2026-11-05, with the lens covered under the night tuning, a
     * fixed QP 35 turned sensor noise into 1.75 MB frames -- ~350 Mbit/s,
     * more than half the encoder's stream buffer per frame. A bitrate cap
     * the encoder actually enforces is the only thing that stops that;
     * whether the V200's H.264 loop enforces it is what the per-channel
     * bitrate log measures.
     */
    rc->mode = h265 ? FH_RC_H265CBR : FH_RC_H264CBR;
    HAL_LOG_INFO("venc chn %d: %s CBR %u kbps", chn, h265 ? "H.265" : "H.264", kbps);
    rc->param[0] = 35;
    rc->param[1] = kbps << 10;
    rc->param[2] = frw;
    rc->param[3] = 50;
    rc->param[4] = 35;
    rc->param[5] = 50;
    rc->param[6] = frw;
    rc->param[7] = 120;
    rc->param[10] = 5;
    rc->param[11] = 1;
}

static unsigned int fh_h264_profile(int profile)
{
    switch (profile) {
    case 0:
        return 66;
    case 2:
        return 100;
    default:
        return 77;
    }
}

/* Push the channel's whole configuration to the driver. */
static int fh_enc_apply(fh_state_t *st, int chn, fh_enc_chn_t *enc)
{
    int rc;

    if (enc->fh_codec == FH_CODEC_MJPEG) {
        fh_venc_mjpeg_attr a;
        unsigned int q = enc->jpeg_quality > 0 ? (unsigned int)enc->jpeg_quality : 80u;
        unsigned int fps = enc->fps_den ? enc->fps_num / enc->fps_den : 1;

        if (q > 99)
            q = 99;
        memset(&a, 0, sizeof(a));
        a.codec = FH_CODEC_MJPEG;
        a.width = enc->width;
        a.height = enc->height;
        a.enc_speed = 4;
        a.qp = 99 - q;
        if (a.qp > 98)
            a.qp = 98;
        a.frame_num = (unsigned short)(fps ? fps : 1);
        a.frame_time = 1;
        rc = st->pipe.fnVencSetAttr((unsigned)chn, &a);
        if (rc)
            HAL_LOG_ERR("venc chn %d: MJPEG attributes rejected 0x%x", chn, (unsigned)rc);
        else
            HAL_LOG_INFO("venc chn %d: MJPEG %ux%u, quality %u (qp %u), %u fps", chn, enc->width,
                         enc->height, q, a.qp, a.frame_num);
        return rc ? RSS_ERR_IO : RSS_OK;
    }

    {
        fh_venc_attr a;
        unsigned int fps = enc->fps_den ? enc->fps_num / enc->fps_den : 25;

        memset(&a, 0, sizeof(a));
        a.codec = enc->fh_codec;
        a.profile = enc->codec == RSS_CODEC_H265 ? 1u : fh_h264_profile(enc->profile);
        a.gop = enc->gop ? enc->gop : (fps ? fps : 25) * 2;
        a.width = enc->width;
        a.height = enc->height;
        fh_fill_rc(enc, &a.rate, chn);

        rc = st->pipe.fnVencSetAttr((unsigned)chn, &a);
        if (rc && enc->codec == RSS_CODEC_H264 && a.profile != 77) {
            HAL_LOG_WARN("venc chn %d: H.264 profile %u rejected (0x%x); using Main", chn,
                         a.profile, (unsigned)rc);
            a.profile = 77;
            rc = st->pipe.fnVencSetAttr((unsigned)chn, &a);
        }
        if (rc) {
            HAL_LOG_ERR("venc chn %d: channel attributes rejected 0x%x", chn, (unsigned)rc);
            return RSS_ERR_IO;
        }
        /* After the channel attribute, or the attribute's copy wins. */
        rc = st->pipe.fnVencSetRc((unsigned)chn, &a.rate);
        if (rc) {
            HAL_LOG_ERR("venc chn %d: FH_VENC_SetRCAttr 0x%x", chn, (unsigned)rc);
            return RSS_ERR_IO;
        }
        HAL_LOG_INFO("venc chn %d: %s %ux%u profile %u gop %u", chn,
                     enc->codec == RSS_CODEC_H265 ? "H.265" : "H.264", enc->width, enc->height,
                     a.profile, a.gop);
    }
    return RSS_OK;
}

/* Apply with reception paused, then resume. */
static int fh_enc_reconfigure(fh_state_t *st, int chn, fh_enc_chn_t *enc)
{
    int rc;

    if (enc->receiving)
        st->pipe.fnVencStop((unsigned)chn);
    rc = fh_enc_apply(st, chn, enc);
    if (enc->receiving && st->pipe.fnVencStart((unsigned)chn))
        enc->receiving = false;
    if (!rc && enc->codec != RSS_CODEC_JPEG && enc->codec != RSS_CODEC_MJPEG)
        st->pipe.fnVencRequestIdr((unsigned)chn);
    return rc;
}

/* ================================================================
 * ENCODER OPS
 * ================================================================ */

/* No encoder group on Fullhan; rvd calls these unconditionally. */
int hal_enc_create_group(void *ctx, int grp)
{
    fh_state_t *st = fh_state(ctx);

    if (!st || grp < 0 || grp >= FH_VENC_CHN_MAX)
        return RSS_ERR_INVAL;
    return RSS_OK;
}

int hal_enc_destroy_group(void *ctx, int grp)
{
    return hal_enc_create_group(ctx, grp);
}

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg)
{
    int rc;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!cfg)
        return RSS_ERR_INVAL;
    if (enc->created) {
        HAL_LOG_ERR("venc chn %d: already created", chn);
        return RSS_ERR_BUSY;
    }

    switch (cfg->codec) {
    case RSS_CODEC_H264:
        enc->fh_codec = FH_CODEC_H264;
        break;
    case RSS_CODEC_H265:
        enc->fh_codec = FH_CODEC_H265;
        break;
    case RSS_CODEC_JPEG:
    case RSS_CODEC_MJPEG:
        enc->fh_codec = FH_CODEC_MJPEG;
        break;
    default:
        return RSS_ERR_NOTSUP;
    }

    enc->codec = cfg->codec;
    enc->width = cfg->width;
    enc->height = cfg->height;
    enc->fps_num = cfg->fps_num ? cfg->fps_num : 25;
    enc->fps_den = cfg->fps_den ? cfg->fps_den : 1;
    enc->gop = cfg->gop_length;
    enc->bitrate = cfg->bitrate;
    enc->rc_mode = cfg->rc_mode;
    enc->init_qp = cfg->init_qp;
    enc->profile = cfg->profile;
    enc->jpeg_quality = cfg->init_qp > 0 ? cfg->init_qp : 0;

    /* The channel's memory is sized once, at its first creation. */
    if (!enc->made) {
        fh_venc_cap cap;

        memset(&cap, 0, sizeof(cap));
        cap.types = enc->fh_codec == FH_CODEC_MJPEG ? FH_CODEC_MJPEG
                                                    : (FH_CODEC_H264 | FH_CODEC_H265);
        cap.width = enc->width;
        cap.height = enc->height;
        rc = st->pipe.fnVencCreate((unsigned)chn, &cap);
        if (rc) {
            HAL_LOG_ERR("venc chn %d: FH_VENC_CreateChn(%ux%u) 0x%x", chn, cap.width, cap.height,
                        (unsigned)rc);
            return RSS_ERR_IO;
        }
        enc->made = true;
        enc->made_w = enc->width;
        enc->made_h = enc->height;
    } else if (enc->width > enc->made_w || enc->height > enc->made_h) {
        HAL_LOG_ERR("venc chn %d: created at %ux%u; %ux%u needs a restart", chn, enc->made_w,
                    enc->made_h, enc->width, enc->height);
        return RSS_ERR_INVAL;
    }

    rc = fh_enc_apply(st, chn, enc);
    if (rc)
        return rc;

    /* divinus's order: receiving before the bind, which follows in
     * enc_register_channel or bind. rvd's enc_start then lets frames
     * through to its rings. */
    rc = st->pipe.fnVencStart((unsigned)chn);
    if (rc) {
        HAL_LOG_ERR("venc chn %d: FH_VENC_StartRecvPic 0x%x", chn, (unsigned)rc);
        return RSS_ERR_IO;
    }
    enc->receiving = true;
    enc->created = true;
    return RSS_OK;
}

int hal_enc_destroy_channel(void *ctx, int chn)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_OK;
    if (enc->receiving)
        st->pipe.fnVencStop((unsigned)chn);
    if (enc->bound_fs >= 0)
        st->pipe.fnUnbind((unsigned)chn);

    pthread_mutex_lock(&st->enc_lock);
    enc->receiving = false;
    enc->started = false;
    enc->created = false;
    enc->bound_fs = -1;
    enc->q_count = 0;
    enc->held = -1;
    pthread_mutex_unlock(&st->enc_lock);
    pthread_cond_broadcast(&st->enc_cond);
    return RSS_OK;
}

static int fh_bind(fh_state_t *st, int fs, int chn)
{
    fh_enc_chn_t *enc = &st->enc[chn];
    int rc, tries;

    if (fs < 0 || fs >= st->vpss_count)
        return RSS_ERR_INVAL;
    if (enc->bound_fs == fs)
        return RSS_OK;
    if (enc->bound_fs >= 0) {
        HAL_LOG_ERR("venc chn %d: already bound to framesource %d", chn, enc->bound_fs);
        return RSS_ERR_BUSY;
    }

    /* The VPSS channel's output mode follows the codec of the video
     * encoder on it (divinus pr61: set on the *source* channel). */
    if ((enc->codec == RSS_CODEC_H264 || enc->codec == RSS_CODEC_H265) &&
        st->pipe.fnVpssSetVOMode) {
        rc = st->pipe.fnVpssSetVOMode((unsigned)fs, enc->codec == RSS_CODEC_H265 ? 1u : 0u);
        if (rc)
            HAL_LOG_WARN("vpss: FH_VPSS_SetVOMode(%d) 0x%x", fs, (unsigned)rc);
    }

    /* divinus retries the bind: it can be refused while the channel is
     * still settling. */
    for (tries = 0; tries < 5; tries++) {
        rc = st->pipe.fnBind((unsigned)fs, (unsigned)chn);
        if (!rc)
            break;
        fh_sleep_ms(100);
    }
    if (rc) {
        HAL_LOG_ERR("bind: FH_SYS_BindVpu2Enc(vpss %d, venc %d) 0x%x", fs, chn, (unsigned)rc);
        return RSS_ERR_IO;
    }
    enc->bound_fs = fs;
    HAL_LOG_INFO("bind: VPSS %d -> VENC %d", fs, chn);
    return RSS_OK;
}

static int fh_unbind(fh_state_t *st, int chn)
{
    fh_enc_chn_t *enc = &st->enc[chn];

    if (enc->bound_fs < 0)
        return RSS_OK;
    st->pipe.fnUnbind((unsigned)chn);
    enc->bound_fs = -1;
    return RSS_OK;
}

int hal_enc_register_channel(void *ctx, int grp, int chn)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created) {
        HAL_LOG_ERR("venc chn %d: register before create", chn);
        return RSS_ERR_INVAL;
    }
    return fh_bind(st, grp, chn);
}

int hal_enc_unregister_channel(void *ctx, int chn)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

    (void)enc;
    return fh_unbind(st, chn);
}

/* rvd's FS -> [OSD ->] ENC chain, collapsed to one bind as on HiSilicon. */
static int fh_bind_collapse(fh_state_t *st, const rss_cell_t *src, const rss_cell_t *dst,
                            int *fs, int *chn, bool *collapsed)
{
    *collapsed = false;
    if (!src || !dst)
        return RSS_ERR_INVAL;

    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_ENC) {
        *fs = src->group;
        *chn = dst->group;
    } else if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_OSD) {
        if (dst->group < 0 || dst->group >= FH_VENC_CHN_MAX)
            return RSS_ERR_INVAL;
        st->osd_src_fs[dst->group] = src->group;
        *collapsed = true;
        return RSS_OK;
    } else if (src->device == RSS_DEV_OSD && dst->device == RSS_DEV_ENC) {
        if (src->group < 0 || src->group >= FH_VENC_CHN_MAX)
            return RSS_ERR_INVAL;
        *fs = st->osd_src_fs[src->group];
        *chn = dst->group;
        if (*fs < 0)
            return RSS_ERR_INVAL;
    } else {
        HAL_LOG_ERR("bind: FS -> [OSD ->] ENC is the only chain this backend supports");
        return RSS_ERR_NOTSUP;
    }
    if (*chn < 0 || *chn >= FH_VENC_CHN_MAX)
        return RSS_ERR_INVAL;
    return RSS_OK;
}

int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    fh_state_t *st = fh_state(ctx);
    bool collapsed;
    int fs = -1, chn = -1, rc;

    if (!st || !st->enc_sync_ready)
        return RSS_ERR_INVAL;
    rc = fh_bind_collapse(st, src, dst, &fs, &chn, &collapsed);
    if (rc || collapsed)
        return rc;
    return fh_bind(st, fs, chn);
}

int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    fh_state_t *st = fh_state(ctx);
    bool collapsed;
    int fs = -1, chn = -1, rc;

    if (!st || !st->enc_sync_ready)
        return RSS_ERR_INVAL;
    rc = fh_bind_collapse(st, src, dst, &fs, &chn, &collapsed);
    if (rc || collapsed)
        return rc;
    return fh_unbind(st, chn);
}

int hal_enc_start(void *ctx, int chn)
{
    int rc;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;
    if (!enc->receiving) {
        rc = st->pipe.fnVencStart((unsigned)chn);
        if (rc) {
            HAL_LOG_ERR("venc chn %d: FH_VENC_StartRecvPic 0x%x", chn, (unsigned)rc);
            return RSS_ERR_IO;
        }
        enc->receiving = true;
    }
    pthread_mutex_lock(&st->enc_lock);
    enc->started = true;
    pthread_mutex_unlock(&st->enc_lock);

    rc = fh_collect_start(st);
    if (rc)
        return rc;
    if (enc->codec == RSS_CODEC_H264 || enc->codec == RSS_CODEC_H265)
        st->pipe.fnVencRequestIdr((unsigned)chn);
    return RSS_OK;
}

/* H.26x channels keep encoding while stopped (their reference chain stays
 * intact); MJPEG stops receiving, which is the CPU rvd's duty cycle is
 * there to save. */
int hal_enc_stop(void *ctx, int chn)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

    pthread_mutex_lock(&st->enc_lock);
    enc->started = false;
    enc->q_count = 0;
    pthread_mutex_unlock(&st->enc_lock);

    if (enc->codec != RSS_CODEC_H264 && enc->codec != RSS_CODEC_H265 && enc->receiving) {
        st->pipe.fnVencStop((unsigned)chn);
        enc->receiving = false;
    }
    return RSS_OK;
}

int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms)
{
    struct timespec until;
    int rc = 0;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;

    clock_gettime(CLOCK_MONOTONIC, &until);
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
    fh_frame_t *f;
    int s;

    FH_ENC_ENTER(ctx, chn, st, enc);

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
    enc->q_head = (enc->q_head + 1) % FH_ENC_QDEPTH;
    enc->q_count--;
    enc->held = s;
    pthread_mutex_unlock(&st->enc_lock);

    f = &enc->slots[s];
    memset(frame, 0, sizeof(*frame));
    frame->nals = f->nals;
    frame->nal_count = f->nal_count;
    frame->codec = enc->codec;
    frame->timestamp = f->ts;
    frame->seq = f->seq;
    frame->is_key = f->key;
    frame->_priv = f;
    return RSS_OK;
}

int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

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
    int rc;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;
    if (enc->codec != RSS_CODEC_H264 && enc->codec != RSS_CODEC_H265)
        return RSS_OK;
    rc = st->pipe.fnVencRequestIdr((unsigned)chn);
    return rc ? RSS_ERR_IO : RSS_OK;
}

int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate)
{
    uint32_t old;
    int rc;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created || enc->fh_codec == FH_CODEC_MJPEG)
        return RSS_ERR_INVAL;
    old = enc->bitrate;
    enc->bitrate = bitrate;
    rc = fh_enc_reconfigure(st, chn, enc);
    if (rc)
        enc->bitrate = old;
    return rc;
}

int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate)
{
    rss_rc_mode_t old_mode;
    uint32_t old_rate;
    int rc;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created || enc->fh_codec == FH_CODEC_MJPEG)
        return RSS_ERR_INVAL;
    old_mode = enc->rc_mode;
    old_rate = enc->bitrate;
    enc->rc_mode = mode;
    if (bitrate)
        enc->bitrate = bitrate;
    rc = fh_enc_reconfigure(st, chn, enc);
    if (rc) {
        enc->rc_mode = old_mode;
        enc->bitrate = old_rate;
    }
    return rc;
}

int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length)
{
    uint32_t old;
    int rc;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created || enc->fh_codec == FH_CODEC_MJPEG)
        return RSS_ERR_INVAL;
    old = enc->gop;
    enc->gop = gop_length;
    rc = fh_enc_reconfigure(st, chn, enc);
    if (rc)
        enc->gop = old;
    return rc;
}

/* rvd passes JPEG *quality*, 1..100 higher-better, despite the name. */
int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp)
{
    int old, rc;

    FH_ENC_ENTER(ctx, chn, st, enc);

    if (qp < 1 || qp > 100)
        return RSS_ERR_INVAL;
    if (!enc->created || enc->fh_codec != FH_CODEC_MJPEG)
        return RSS_ERR_NOTSUP;
    old = enc->jpeg_quality;
    enc->jpeg_quality = qp;
    rc = fh_enc_reconfigure(st, chn, enc);
    if (rc)
        enc->jpeg_quality = old;
    return rc;
}

int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!qp)
        return RSS_ERR_INVAL;
    if (!enc->created || enc->fh_codec != FH_CODEC_MJPEG)
        return RSS_ERR_NOTSUP;
    *qp = enc->jpeg_quality > 0 ? enc->jpeg_quality : 80;
    return RSS_OK;
}

int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;
    *fps_num = enc->fps_num;
    *fps_den = enc->fps_den ? enc->fps_den : 1;
    return RSS_OK;
}

/* The configured target, as the other backends report it. */
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate)
{
    FH_ENC_ENTER(ctx, chn, st, enc);

    if (!bitrate)
        return RSS_ERR_INVAL;
    *bitrate = enc->bitrate;
    return RSS_OK;
}

int hal_sys_get_timestamp(void *ctx, int64_t *ts)
{
    (void)ctx;
    if (!ts)
        return RSS_ERR_INVAL;
    *ts = fh_now_us();
    return RSS_OK;
}
