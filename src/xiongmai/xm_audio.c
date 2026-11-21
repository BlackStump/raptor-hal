/*
 * xiongmai/xm_audio.c -- microphone capture for the Xiongmai backend
 *
 * THE PATH
 *
 * The XM530/550 audio drivers (xm_i2s.ko, acodec.ko; vendor modules from
 * OpenIPC's xiongmai-osdrv-xm530, loaded by load_xiongmai) need no
 * library: /dev/ai has no ioctls, and read() blocks until that many bytes
 * of captured PCM are in the driver's FIFO, then copies them out. acodec
 * sets the codec up at load with sample_rate=8: 8 kHz, 16-bit, mono --
 * 16000 bytes a second measured on M1 (2026-11-21). So this backend
 * offers 8 kHz mono and nothing else; rad's G.711 default asks exactly
 * that.
 *
 * LEVELS
 *
 * acodec has an ioctl, but its commands are unknown; volume is applied
 * here in software instead (rad's [-30..120], 80 = as captured, linear),
 * and the analog gain stays where acodec put it. Mute zeroes the frames.
 *
 * THE HUM FILTER
 *
 * M1's mic picks up some 50 Hz hum (and 150 Hz), a tenth of the PB4's, and
 * a DC offset of about -190. The same fourth-order Butterworth high-pass
 * at 250 Hz as the Fullhan backend takes both out: the quiet bench
 * recording goes from 216 to ~52 RMS. On by default; audio_disable_hpf
 * turns it off.
 *
 * Capture only. No AO, NS, AGC or AEC.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE

#include "xm_state.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define XM_AI_DEV "/dev/ai"
#define XM_AI_RATE 8000
#define XM_AUD_HPF_HZ 250.0
#define XM_PI 3.14159265358979323846
#define XM_AUD_READ_TIMEOUT_MS 1000

/* Biquad, Q2.28 coefficients, direct form I. */
typedef struct {
    int32_t b0, b1, b2, a1, a2;
    int32_t x1, x2, y1, y2;
} xm_hpf_t;

static struct {
    int fd;
    bool hpf_on, muted;
    unsigned int samples;
    int16_t *buf;
    size_t buf_bytes;
    uint32_t seq;
    int volume, gain;
    int32_t scale; /* Q8: volume / 80 */
    xm_hpf_t hpf[2];
} g_aud = {.fd = -1, .hpf_on = true, .volume = 80, .gain = 25, .scale = 256};

#define HPF_ONE (1 << 28)

static void xm_hpf_section(xm_hpf_t *f, int rate, double q)
{
    /* RBJ cookbook high-pass */
    double w0 = 2.0 * XM_PI * XM_AUD_HPF_HZ / rate;
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
static void xm_hpf_setup(xm_hpf_t f[2], int rate)
{
    xm_hpf_section(&f[0], rate, 0.54119610014619698);
    xm_hpf_section(&f[1], rate, 1.30656296487637653);
}

static void xm_hpf_run(xm_hpf_t *f, int16_t *s, unsigned int n)
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

static int xm_clamp(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static void xm_set_scale(void)
{
    g_aud.scale = (int32_t)(g_aud.volume <= 0 ? 0 : g_aud.volume * 256 / 80);
}

static void xm_apply_scale(int16_t *s, unsigned int n)
{
    if (g_aud.scale == 256)
        return;
    for (unsigned int i = 0; i < n; i++) {
        int32_t v = ((int32_t)s[i] * g_aud.scale) >> 8;
        s[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
    }
}

int hal_audio_init(void *ctx, const rss_audio_config_t *cfg)
{
    unsigned int samples;

    (void)ctx;
    if (!cfg)
        return RSS_ERR_INVAL;
    if ((int)cfg->sample_rate != XM_AI_RATE) {
        HAL_LOG_ERR("audio: %d Hz asked; the XM530 codec runs at 8000 Hz only "
                    "([audio] sample_rate = 8000)", (int)cfg->sample_rate);
        return RSS_ERR_NOTSUP;
    }
    if (cfg->chn_count > 1) {
        HAL_LOG_ERR("audio: %d channels asked; one mic", cfg->chn_count);
        return RSS_ERR_NOTSUP;
    }
    if (g_aud.fd >= 0) {
        close(g_aud.fd);
        g_aud.fd = -1;
    }
    samples = cfg->samples_per_frame > 0 ? (unsigned int)cfg->samples_per_frame : XM_AI_RATE / 50;
    g_aud.samples = samples;
    if ((size_t)samples * 2 > g_aud.buf_bytes) {
        int16_t *b = realloc(g_aud.buf, (size_t)samples * 2);
        if (!b)
            return RSS_ERR_NOMEM;
        g_aud.buf = b;
        g_aud.buf_bytes = (size_t)samples * 2;
    }
    g_aud.fd = open(XM_AI_DEV, O_RDONLY | O_CLOEXEC);
    if (g_aud.fd < 0) {
        HAL_LOG_ERR("audio: %s: %s (are xm_i2s and acodec loaded?)", XM_AI_DEV, strerror(errno));
        return RSS_ERR_IO;
    }
    if (cfg->ai_vol)
        g_aud.volume = cfg->ai_vol;
    if (cfg->ai_gain)
        g_aud.gain = cfg->ai_gain;
    xm_set_scale();
    xm_hpf_setup(g_aud.hpf, XM_AI_RATE);
    HAL_LOG_INFO("audio: mic up, %d Hz mono, %u samples/frame, %d Hz high-pass %s", XM_AI_RATE,
                 samples, (int)XM_AUD_HPF_HZ, g_aud.hpf_on ? "on" : "off");
    return RSS_OK;
}

int hal_audio_deinit(void *ctx)
{
    (void)ctx;
    if (g_aud.fd >= 0) {
        close(g_aud.fd);
        g_aud.fd = -1;
    }
    return RSS_OK;
}

int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block)
{
    size_t want, got = 0;

    (void)ctx;
    (void)dev;
    if (!frame || chn != 0)
        return RSS_ERR_INVAL;
    if (g_aud.fd < 0)
        return RSS_ERR_NOTSUP;
    want = (size_t)g_aud.samples * 2;

    /* the driver's read blocks for the whole count; poll first so a stop
     * is not held up for ever */
    struct pollfd p = {.fd = g_aud.fd, .events = POLLIN};
    int pr = poll(&p, 1, block ? XM_AUD_READ_TIMEOUT_MS : 0);
    if (pr == 0)
        return RSS_ERR_TIMEOUT;
    while (got < want) {
        ssize_t r = read(g_aud.fd, (uint8_t *)g_aud.buf + got, want - got);
        if (r < 0) {
            if (errno == EINTR)
                continue;
            HAL_LOG_WARN("audio: read %s: %s", XM_AI_DEV, strerror(errno));
            return RSS_ERR_IO;
        }
        if (r == 0)
            return RSS_ERR_IO;
        got += (size_t)r;
    }

    unsigned int n = (unsigned int)(want / 2);
    if (g_aud.muted) {
        memset(g_aud.buf, 0, want);
    } else {
        if (g_aud.hpf_on) {
            xm_hpf_run(&g_aud.hpf[0], g_aud.buf, n);
            xm_hpf_run(&g_aud.hpf[1], g_aud.buf, n);
        }
        xm_apply_scale(g_aud.buf, n);
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    frame->data = g_aud.buf;
    frame->length = (uint32_t)want;
    frame->timestamp = (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    frame->seq = g_aud.seq++;
    frame->_priv = NULL;
    return RSS_OK;
}

int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    (void)frame;
    return RSS_OK;
}

int hal_audio_set_volume(void *ctx, int dev, int chn, int vol)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    g_aud.volume = xm_clamp(vol, -30, 120);
    xm_set_scale();
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

/* The analog gain is acodec's, set at load; kept only to report back. */
int hal_audio_set_gain(void *ctx, int dev, int chn, int gain)
{
    (void)ctx;
    (void)dev;
    (void)chn;
    g_aud.gain = xm_clamp(gain, 0, 31);
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
    if (!g_aud.hpf_on)
        xm_hpf_setup(g_aud.hpf, XM_AI_RATE);
    g_aud.hpf_on = true;
    return RSS_OK;
}

int hal_audio_disable_hpf(void *ctx)
{
    (void)ctx;
    g_aud.hpf_on = false;
    return RSS_OK;
}
