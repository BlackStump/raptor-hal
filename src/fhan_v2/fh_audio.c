/*
 * fhan_v2/fh_audio.c -- microphone capture for the Fullhan backend
 *
 * THE PATH
 *
 * The FH885xV200's audio codec is driven by the RT-Thread firmware on the
 * chip's ARC core, the same one the media pipeline runs on; Linux reaches
 * it through xbus_rpc (/dev/rtxbus). libacw_mpi.so wraps that: FH_AC_Init
 * asks the ARC for the shared PCM ring and maps it, FH_AC_Set_Config and
 * FH_AC_AI_Enable start capture, FH_AC_AI_GetFrameWithPts copies one
 * period out. No kernel driver of its own, no /dev node; what xbus_rpc
 * loads for video is enough (PB4 bench, 2026-11-20: 16 kHz mono beside a
 * running rvd).
 *
 * The library is a uClibc build like the rest of the vendor set, and
 * FH_AC_Init's mmap takes uClibc's 32-bit offset ("acw: mmap fail!" under
 * plain musl). hal_common.c's bridge serves it as it serves libdsp; rad
 * links --export-dynamic like every HAL daemon (LDFLAGS_VENDOR_fullhan).
 *
 * rad calls rss_hal_create and then audio_init, never hal_init, so this
 * file keeps its own state and needs nothing the video side sets up.
 *
 * THE HUM FILTER
 *
 * The PB4's mic picks up mains hum: on the bench recording (2026-11-20)
 * the quiet garage measured 2380 RMS, nearly all of it 50 Hz plus its
 * 150 Hz harmonic. A fourth-order Butterworth high-pass at 250 Hz (two
 * fixed-point biquads) brings that to ~100 RMS and leaves speech, which
 * lives above 300 Hz; a second-order one at 200 Hz left 232, most of it
 * the 150 Hz harmonic (measured on the same recording). It runs on every frame unless
 * audio_disable_hpf turns it off: rad only calls the effect ops when built
 * with AUDIO_EFFECTS, and the hum is there either way.
 *
 * SCALES
 *
 *   volume  rad's [-30..120], default 80  -> FH_AC_AI_SetVol [0..100],
 *           vol + 5 clamped (80 -> 85, Divinus's and the bench test's)
 *   gain    rad's [0..31], default 25     -> FH_AC_AI_MICIN_SetVol [0..3],
 *           gain * 2 / 25 clamped (25 -> 2, the bench test's)
 *
 * Mono, 16-bit, capture only. No AO, NS, AGC or AEC.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE /* usleep, clock_gettime under -std=c11 */

#include "fh_state.h"

#include <dlfcn.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define FH_LIB_ACW "libacw_mpi.so"

#define FH_AUD_READ_TIMEOUT_MS 1000
#define FH_AUD_POLL_US 5000
#define FH_AUD_HPF_HZ 250.0
#define FH_PI 3.14159265358979323846

typedef struct {
    unsigned int io;     /* 0 = capture */
    unsigned int rate;
    unsigned int bit;    /* 16 */
    unsigned int fmt;    /* 0 = PCM */
    unsigned int chn_num;
    unsigned int frm_num; /* samples per period, >= 80 */
    unsigned int volume; /* 0..100 */
} fh_ac_cfg;

typedef struct {
    unsigned int length; /* bytes, filled in */
    void *data;
} fh_ac_frm;

typedef struct {
    void *handle;
    int (*fnInit)(void);
    int (*fnDeInit)(void);
    int (*fnSetConfig)(fh_ac_cfg *cfg);
    int (*fnAiEnable)(void);
    int (*fnAiDisable)(void);
    int (*fnAiSetVol)(unsigned int vol);
    int (*fnMicInSetVol)(unsigned int vol);
    int (*fnAiGetFrame)(fh_ac_frm *frm, unsigned long long *pts);
} fh_acw_impl;

/* Biquad, Q2.28 coefficients, direct form I. */
typedef struct {
    int32_t b0, b1, b2, a1, a2;
    int32_t x1, x2, y1, y2;
} fh_hpf_t;

static struct {
    fh_acw_impl acw;
    bool loaded;
    bool inited;
    bool enabled;
    bool hpf_on;
    bool muted;
    int rate;
    unsigned int samples;
    int16_t *buf;
    size_t buf_bytes;
    bool held;
    uint32_t seq;
    int volume;
    int gain;
    fh_hpf_t hpf[2];
} g_aud = {.hpf_on = true, .volume = 80, .gain = 25};

static void *fh_acw_sym(void *h, const char *name)
{
    void *p = dlsym(h, name);

    if (!p)
        HAL_LOG_ERR("audio: %s has no %s", FH_LIB_ACW, name);
    return p;
}

static int fh_acw_load(fh_acw_impl *a)
{
    memset(a, 0, sizeof(*a));
    a->handle = dlopen(FH_LIB_ACW, RTLD_NOW | RTLD_GLOBAL);
    if (!a->handle) {
        HAL_LOG_ERR("audio: dlopen %s: %s", FH_LIB_ACW, dlerror());
        return RSS_ERR_NOTSUP;
    }
    if (!(a->fnInit = (int (*)(void))fh_acw_sym(a->handle, "FH_AC_Init")) ||
        !(a->fnDeInit = (int (*)(void))fh_acw_sym(a->handle, "FH_AC_DeInit")) ||
        !(a->fnSetConfig = (int (*)(fh_ac_cfg *))fh_acw_sym(a->handle, "FH_AC_Set_Config")) ||
        !(a->fnAiEnable = (int (*)(void))fh_acw_sym(a->handle, "FH_AC_AI_Enable")) ||
        !(a->fnAiDisable = (int (*)(void))fh_acw_sym(a->handle, "FH_AC_AI_Disable")) ||
        !(a->fnAiSetVol = (int (*)(unsigned int))fh_acw_sym(a->handle, "FH_AC_AI_SetVol")) ||
        !(a->fnMicInSetVol =
              (int (*)(unsigned int))fh_acw_sym(a->handle, "FH_AC_AI_MICIN_SetVol")) ||
        !(a->fnAiGetFrame = (int (*)(fh_ac_frm *, unsigned long long *))fh_acw_sym(
              a->handle, "FH_AC_AI_GetFrameWithPts"))) {
        dlclose(a->handle);
        memset(a, 0, sizeof(*a));
        return RSS_ERR_NOTSUP;
    }
    return RSS_OK;
}

/* ================================================================
 * HIGH-PASS
 * ================================================================ */

#define HPF_ONE (1 << 28)

static void fh_hpf_section(fh_hpf_t *f, int rate, double q)
{
    /* RBJ cookbook high-pass */
    double w0 = 2.0 * FH_PI * FH_AUD_HPF_HZ / rate;
    double alpha = sin(w0) / (2.0 * q);
    double c = cos(w0);
    double a0 = 1.0 + alpha;

    memset(f, 0, sizeof(*f));
    f->b0 = (int32_t)lround((1.0 + c) / 2.0 / a0 * HPF_ONE);
    f->b1 = (int32_t)lround(-(1.0 + c) / a0 * HPF_ONE);
    f->b2 = f->b0;
    f->a1 = (int32_t)lround(-2.0 * c / a0 * HPF_ONE);
    f->a2 = (int32_t)lround((1.0 - alpha) / a0 * HPF_ONE);
}

/* Fourth-order Butterworth: two sections, Q = 1/(2cos(pi/8)), 1/(2cos(3pi/8)) */
static void fh_hpf_setup(fh_hpf_t f[2], int rate)
{
    fh_hpf_section(&f[0], rate, 0.54119610014619698);
    fh_hpf_section(&f[1], rate, 1.30656296487637653);
}

static void fh_hpf_run(fh_hpf_t *f, int16_t *s, unsigned int n)
{
    for (unsigned int i = 0; i < n; i++) {
        int32_t x = s[i];
        int64_t acc = (int64_t)f->b0 * x + (int64_t)f->b1 * f->x1 + (int64_t)f->b2 * f->x2 -
                      (int64_t)f->a1 * f->y1 - (int64_t)f->a2 * f->y2;
        int32_t y = (int32_t)(acc >> 28);

        f->x2 = f->x1;
        f->x1 = x;
        f->y2 = f->y1;
        f->y1 = y;
        s[i] = (int16_t)(y > 32767 ? 32767 : y < -32768 ? -32768 : y);
    }
}

/* ================================================================
 * LEVELS
 * ================================================================ */

static int fh_clamp(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static void fh_apply_levels(void)
{
    if (!g_aud.enabled)
        return;
    g_aud.acw.fnAiSetVol((unsigned int)fh_clamp(g_aud.volume + 5, 0, 100));
    g_aud.acw.fnMicInSetVol((unsigned int)fh_clamp(g_aud.gain * 2 / 25, 0, 3));
}

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

static void fh_audio_stop(void)
{
    if (g_aud.enabled) {
        g_aud.acw.fnAiDisable();
        g_aud.enabled = false;
    }
    if (g_aud.inited) {
        g_aud.acw.fnDeInit();
        g_aud.inited = false;
    }
    g_aud.held = false;
}

int hal_audio_init(void *ctx, const rss_audio_config_t *cfg)
{
    fh_ac_cfg ac;
    unsigned int samples;
    int ret;

    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
    if (cfg->chn_count > 1) {
        HAL_LOG_ERR("audio: %d channels requested; the PB4 has one mic", cfg->chn_count);
        return RSS_ERR_NOTSUP;
    }
    if (cfg->input_type != RSS_AUDIO_INPUT_AMIC)
        HAL_LOG_WARN("audio: no digital mic on this backend; using the analog input");

    if (!g_aud.loaded) {
        /* Before the vendor dlopen: FH_AC_Init's mmap needs the bridge. */
        fh_check_exports();
        ret = fh_acw_load(&g_aud.acw);
        if (ret)
            return ret;
        g_aud.loaded = true;
    }
    fh_audio_stop();

    g_aud.rate = (int)cfg->sample_rate;
    samples = cfg->samples_per_frame > 0 ? (unsigned int)cfg->samples_per_frame
                                         : (unsigned int)g_aud.rate / 50;
    if (samples < 80)
        samples = 80; /* the codec's minimum period */
    g_aud.samples = samples;

    /* The library copies one period per call; leave room for a longer one. */
    size_t need = (size_t)samples * 2 * 2;
    if (need > g_aud.buf_bytes) {
        int16_t *b = realloc(g_aud.buf, need);
        if (!b)
            return RSS_ERR_NOMEM;
        g_aud.buf = b;
        g_aud.buf_bytes = need;
    }

    ret = g_aud.acw.fnInit();
    if (ret) {
        HAL_LOG_ERR("audio: FH_AC_Init failed 0x%x", (unsigned)ret);
        return RSS_ERR_IO;
    }
    g_aud.inited = true;

    memset(&ac, 0, sizeof(ac));
    ac.io = 0;
    ac.rate = (unsigned int)g_aud.rate;
    ac.bit = 16;
    ac.fmt = 0;
    ac.chn_num = 1;
    ac.frm_num = samples;
    ac.volume = (unsigned int)fh_clamp(g_aud.volume + 5, 0, 100);
    ret = g_aud.acw.fnSetConfig(&ac);
    if (ret) {
        HAL_LOG_ERR("audio: FH_AC_Set_Config(%d Hz, %u samples) failed 0x%x", g_aud.rate,
                    samples, (unsigned)ret);
        fh_audio_stop();
        return RSS_ERR_INVAL;
    }
    ret = g_aud.acw.fnAiEnable();
    if (ret) {
        HAL_LOG_ERR("audio: FH_AC_AI_Enable failed 0x%x", (unsigned)ret);
        fh_audio_stop();
        return RSS_ERR_IO;
    }
    g_aud.enabled = true;

    if (cfg->ai_vol)
        g_aud.volume = cfg->ai_vol;
    if (cfg->ai_gain)
        g_aud.gain = cfg->ai_gain;
    fh_apply_levels();
    fh_hpf_setup(g_aud.hpf, g_aud.rate);

    HAL_LOG_INFO("audio: mic up, %d Hz mono, %u samples/frame, %d Hz high-pass %s", g_aud.rate,
                 samples, (int)FH_AUD_HPF_HZ, g_aud.hpf_on ? "on" : "off");
    return RSS_OK;
}

int hal_audio_deinit(void *ctx)
{
    (void)ctx;
    fh_audio_stop();
    return RSS_OK;
}

/* ================================================================
 * FRAMES
 * ================================================================ */

int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block)
{
    unsigned long long pts = 0;
    int waited = 0;

    (void)ctx;
    (void)dev;
    if (!frame || chn != 0)
        return RSS_ERR_INVAL;
    if (!g_aud.enabled)
        return RSS_ERR_NOTSUP;

    for (;;) {
        fh_ac_frm f = {.length = 0, .data = g_aud.buf};
        int ret = g_aud.acw.fnAiGetFrame(&f, &pts);

        if (ret == 0 && f.length > 0 && f.length <= g_aud.buf_bytes) {
            unsigned int n = f.length / 2;

            if (g_aud.muted)
                memset(g_aud.buf, 0, f.length);
            else if (g_aud.hpf_on) {
                fh_hpf_run(&g_aud.hpf[0], g_aud.buf, n);
                fh_hpf_run(&g_aud.hpf[1], g_aud.buf, n);
            }

            if (!pts) {
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                pts = (unsigned long long)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
            }
            frame->data = g_aud.buf;
            frame->length = f.length;
            frame->timestamp = (int64_t)pts;
            frame->seq = g_aud.seq++;
            frame->_priv = NULL;
            g_aud.held = true;
            return RSS_OK;
        }
        if (!block || waited >= FH_AUD_READ_TIMEOUT_MS * 1000)
            return RSS_ERR_TIMEOUT;
        usleep(FH_AUD_POLL_US);
        waited += FH_AUD_POLL_US;
    }
}

int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    (void)frame;
    g_aud.held = false; /* the buffer is ours; nothing to hand back */
    return RSS_OK;
}

/* ================================================================
 * CONTROLS
 * ================================================================ */

int hal_audio_set_volume(void *ctx, int dev, int chn, int vol)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    g_aud.volume = fh_clamp(vol, -30, 120);
    fh_apply_levels();
    return RSS_OK;
}

int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    if (!vol)
        return RSS_ERR_INVAL;
    *vol = g_aud.volume;
    return RSS_OK;
}

int hal_audio_set_gain(void *ctx, int dev, int chn, int gain)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    g_aud.gain = fh_clamp(gain, 0, 31);
    fh_apply_levels();
    return RSS_OK;
}

int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    if (!gain)
        return RSS_ERR_INVAL;
    *gain = g_aud.gain;
    return RSS_OK;
}

int hal_audio_set_mute(void *ctx, int dev, int chn, int mute)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    g_aud.muted = mute != 0;
    return RSS_OK;
}

int hal_audio_enable_hpf(void *ctx)
{
    (void)ctx;
    if (!g_aud.hpf_on && g_aud.rate)
        fh_hpf_setup(g_aud.hpf, g_aud.rate);
    g_aud.hpf_on = true;
    return RSS_OK;
}

int hal_audio_disable_hpf(void *ctx)
{
    (void)ctx;
    g_aud.hpf_on = false;
    return RSS_OK;
}
