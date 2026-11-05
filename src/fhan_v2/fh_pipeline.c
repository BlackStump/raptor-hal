/*
 * fhan_v2/fh_pipeline.c -- Phase 2a: sensor, ISP and VPSS channel 0.
 *
 * The bring-up order is divinus's as carried on the PB4 branch (pr61), which
 * streams 2560x1440@25 from this board:
 *
 *   before FH_SYS_Init   /proc/driver/{vpu,isp,enc,jpeg} buffer sizing
 *   FH_SYS_Init          (hal_common.c)
 *   VPSS                 SetViAttr(sensor size), Enable(0), channel 0 open
 *                        -- "the ISP only streams once a VPSS channel is open"
 *   sensor               Sensor_Create
 *   ISP                  MemInit, FHAdv Init + SensorInit (MemInit FIRST:
 *                        pr61's ordering fix), SensorRegCb, SensorInit,
 *                        SetSensorFmt(mode), Init, SensorKick
 *   tuning               /etc/sensors/<lib>_attr.hex via API_ISP_LoadIspParam
 *   3A                   API_ISP_Run in a thread, every 10 ms
 *
 * Phase 2a stops there: no encoder. Its acceptance test is the ISP counting
 * frames at the mode's size, which this file measures itself (fh_pipe_probe)
 * so the first bench run answers it in the log.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _POSIX_C_SOURCE 200809L /* nanosleep */

/* Video-archive only, and it must see the state layout hal_common.c's video
 * build sees; the generic %.o rule does not pass -DHAL_MODULE_VIDEO. */
#ifndef HAL_MODULE_VIDEO
#define HAL_MODULE_VIDEO
#endif

#include "fh_state.h"

#include <dlfcn.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern long syscall(long number, ...);
#include <sys/syscall.h>
#include <unistd.h>

/* ================================================================
 * SENSOR MODES
 * ================================================================ */

static const fh_snr_mode fh_snr_modes[] = {
    /* measured on the PB4 (divinus pr61) */
    {"gc4653_mipi", 1920, 1080, 25, 0x203},
    {"gc4653_mipi", 1920, 1080, 30, 0x204},
    {"gc4653_mipi", 2560, 1440, 20, 0x602},
    {"gc4653_mipi", 2560, 1440, 25, 0x603},
    /* divinus defaults, never run here */
    {"gc5603_mipi", 2880, 1616, 0, 0x203},
    {"jxf32_mipi", 1920, 1080, 0, 0x203},
    {"mn34425_mipi", 1920, 1080, 0, 0x203},
    {"ovos02k_mipi", 1920, 1080, 0, 0x203},
    {"ovos04c10_mipi", 2560, 1440, 0, 0x203},
    {"ovos05_mipi", 2592, 1944, 0, 0x203},
    {"ovos08_mipi", 3840, 2160, 0, 0x203},
};

#define FH_N_MODES (sizeof(fh_snr_modes) / sizeof(fh_snr_modes[0]))

void fh_sleep_ms(unsigned int ms)
{
    struct timespec ts;

    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    while (nanosleep(&ts, &ts) && errno == EINTR)
        ;
}

/*
 * The mode for a library: the config's index among that library's rows if
 * it names one, otherwise the largest picture at the highest known rate --
 * 2560x1440@25 on a GC4653, the sensor's native size.
 */
static const fh_snr_mode *fh_mode_pick(const char *lib, int index)
{
    const fh_snr_mode *best = NULL;
    int seen = 0;
    size_t i;

    for (i = 0; i < FH_N_MODES; i++) {
        const fh_snr_mode *m = &fh_snr_modes[i];

        if (strcmp(m->lib, lib) != 0)
            continue;
        if (index >= 0) {
            if (seen++ == index)
                return m;
            continue;
        }
        if (!best || m->width * m->height > best->width * best->height ||
            (m->width * m->height == best->width * best->height && m->fps > best->fps))
            best = m;
    }
    if (index >= 0)
        HAL_LOG_WARN("sensor: %s has no mode %d (it has %d); choosing the largest", lib, index,
                     seen);
    return index >= 0 ? fh_mode_pick(lib, -1) : best;
}

/* The library stem for a configured sensor name: "gc4653" -> "gc4653_mipi". */
static int fh_snr_resolve(const char *name, char *stem, size_t stem_size)
{
    static const char *dirs[] = {"/usr/lib/sensors", "/lib/sensors"};
    static const char *suffix[] = {"", "_mipi"};
    char path[128];
    size_t d, s;

    if (!name || !name[0])
        return RSS_ERR_NOENT;

    for (s = 0; s < 2; s++) {
        for (d = 0; d < 2; d++) {
            snprintf(path, sizeof(path), "%s/lib%s%s.so", dirs[d], name, suffix[s]);
            if (access(path, R_OK) == 0) {
                snprintf(stem, stem_size, "%s%s", name, suffix[s]);
                return RSS_OK;
            }
        }
    }
    return RSS_ERR_NOENT;
}

/* ================================================================
 * BEFORE FH_SYS_Init: the drivers take their buffer sizes here
 * ================================================================ */

static void fh_proc_write(const char *path, const char *value)
{
    FILE *f = fopen(path, "w");

    if (!f) {
        HAL_LOG_WARN("proc: cannot open %s for '%s'", path, value);
        return;
    }
    if (fputs(value, f) < 0)
        HAL_LOG_WARN("proc: writing '%s' to %s failed", value, path);
    fclose(f);
}

int fh_pipe_prepare(fh_state_t *st, const rss_sensor_config_t *snr)
{
    char v[64];
    unsigned int w, h, jpg;
    int i;

    if (fh_snr_resolve(snr->name, st->snr_lib, sizeof(st->snr_lib)) != RSS_OK) {
        HAL_LOG_ERR("sensor: no /usr/lib/sensors/lib%s.so or lib%s_mipi.so -- set [sensor] name "
                    "in raptor.conf (gc4653 on a PB4)",
                    snr->name, snr->name);
        return RSS_ERR_NOENT;
    }

    st->mode = fh_mode_pick(st->snr_lib, snr->mode);
    if (!st->mode) {
        HAL_LOG_ERR("sensor: %s has no mode table entry", st->snr_lib);
        return RSS_ERR_NOTSUP;
    }
    st->mirror = snr->hflip != 0;
    st->flip = snr->vflip != 0;

    w = st->mode->width;
    h = st->mode->height;
    HAL_LOG_INFO("sensor: %s %ux%u%s%u fps, format 0x%x", st->snr_lib, w, h,
                 st->mode->fps ? " @ " : " (rate unknown", st->mode->fps, st->mode->format);

    /* Exactly divinus pr61's list; it dropped sublimit_off, so does this. */
    snprintf(v, sizeof(v), "vi_%u_%u", w, h);
    fh_proc_write("/proc/driver/vpu", v);
    for (i = 0; i < FH_VPSS_CHN_MAX; i++) {
        snprintf(v, sizeof(v), "cap_%d_0_0", i);
        fh_proc_write("/proc/driver/vpu", v);
        snprintf(v, sizeof(v), "buf_%d_2", i);
        fh_proc_write("/proc/driver/vpu", v);
    }
    fh_proc_write("/proc/driver/vpu", "support4k_off");
    fh_proc_write("/proc/driver/isp", "support4k_off");
    fh_proc_write("/proc/driver/isp", "wdr_off");
    fh_proc_write("/proc/driver/isp", "cir_off");
    fh_proc_write("/proc/driver/enc", "stm_2621440");
    jpg = (w * h / 2 + 8 + 4095) & ~4095u;
    /*
     * Two JPEG channels: raptor runs a snapshot encoder per stream (main and
     * sub). divinus asked for one ("mem_1_..."), and with that the second
     * channel's MJPEG attributes were rejected (bench, 2026-11-05:
     * venc chn 3, 0x800d4080) while the first worked. The size is the main
     * stream's, so it covers the sub-stream's too.
     */
    snprintf(v, sizeof(v), "mem_2_%u", jpg);
    fh_proc_write("/proc/driver/jpeg", v);
    snprintf(v, sizeof(v), "mjpg_%u_%u", 3u * 512 * 1024, 512u * 1024);
    fh_proc_write("/proc/driver/jpeg", v);

    return RSS_OK;
}

/* ================================================================
 * LIBRARIES
 * ================================================================ */

#define FH_SYM(h, lib, field, name, req)                                                           \
    do {                                                                                           \
        *(void **)&(field) = dlsym((h), (name));                                                   \
        if (!(field) && (req)) {                                                                   \
            HAL_LOG_ERR("%s: missing symbol %s", (lib), (name));                                   \
            return RSS_ERR_IO;                                                                     \
        }                                                                                          \
    } while (0)

static void *fh_open(const char *lib, bool required)
{
    void *h = dlopen(lib, RTLD_NOW | RTLD_GLOBAL);

    if (!h) {
        if (required)
            HAL_LOG_ERR("dlopen %s: %s", lib, dlerror());
        else
            HAL_LOG_WARN("dlopen %s: %s", lib, dlerror());
    }
    return h;
}

static void fh_pipe_close_libs(fh_pipe_impl *p)
{
    void **hs[] = {&p->h_snr, &p->h_advisp, &p->h_adv, &p->h_mipi, &p->h_isp, &p->h_ispcore};
    size_t i;

    for (i = 0; i < sizeof(hs) / sizeof(hs[0]); i++) {
        if (*hs[i])
            dlclose(*hs[i]);
        *hs[i] = NULL;
    }
    memset(p, 0, sizeof(*p));
}

static int fh_pipe_open_libs(fh_state_t *st)
{
    fh_pipe_impl *p = &st->pipe;
    void *dsp = st->sys.h_dsp;
    char path[128], sym[64];

    /* divinus's order: libisp resolves isp_core_* from libispcore at load,
     * libadvapi_isp resolves fh_sys_* from libadvapi. */
    if (!(p->h_ispcore = fh_open("libispcore.so", true)) || !(p->h_isp = fh_open("libisp.so", true)) ||
        !(p->h_mipi = fh_open("libmipi.so", true)) || !(p->h_adv = fh_open("libadvapi.so", true)) ||
        !(p->h_advisp = fh_open("libadvapi_isp.so", true)))
        return RSS_ERR_IO;

    FH_SYM(dsp, "libdsp", p->fnVpssSetViAttr, "FH_VPSS_SetViAttr", true);
    FH_SYM(dsp, "libdsp", p->fnVpssEnable, "FH_VPSS_Enable", true);
    FH_SYM(dsp, "libdsp", p->fnVpssDisable, "FH_VPSS_Disable", true);
    FH_SYM(dsp, "libdsp", p->fnVpssGetChnCap, "FH_VPSS_GetChnCapality", false);
    FH_SYM(dsp, "libdsp", p->fnVpssChnInitMem, "FH_VPSS_ChnInitMem", false);
    FH_SYM(dsp, "libdsp", p->fnVpssSetChnAttr, "FH_VPSS_SetChnAttr", true);
    FH_SYM(dsp, "libdsp", p->fnVpssSetFramectrl, "FH_VPSS_SetFramectrl", true);
    FH_SYM(dsp, "libdsp", p->fnVpssOpenChn, "FH_VPSS_OpenChn", true);
    FH_SYM(dsp, "libdsp", p->fnVpssCloseChn, "FH_VPSS_CloseChn", true);
    FH_SYM(dsp, "libdsp", p->fnVpssSetVOMode, "FH_VPSS_SetVOMode", false);
    FH_SYM(dsp, "libdsp", p->fnBind, "FH_SYS_BindVpu2Enc", true);
    FH_SYM(dsp, "libdsp", p->fnUnbind, "FH_SYS_UnBindbyDst", true);
    FH_SYM(dsp, "libdsp", p->fnVencCreate, "FH_VENC_CreateChn", true);
    FH_SYM(dsp, "libdsp", p->fnVencGetStream, "FH_VENC_GetStream", true);
    FH_SYM(dsp, "libdsp", p->fnVencRelease, "FH_VENC_ReleaseStream", true);
    FH_SYM(dsp, "libdsp", p->fnVencRequestIdr, "FH_VENC_RequestIDR", true);
    FH_SYM(dsp, "libdsp", p->fnVencSetAttr, "FH_VENC_SetChnAttr", true);
    FH_SYM(dsp, "libdsp", p->fnVencSetRc, "FH_VENC_SetRCAttr", true);
    FH_SYM(dsp, "libdsp", p->fnVencGetRc, "FH_VENC_GetRCAttr", false);
    FH_SYM(dsp, "libdsp", p->fnVencGetAttr, "FH_VENC_GetChnAttr", false);
    FH_SYM(dsp, "libdsp", p->fnVencStart, "FH_VENC_StartRecvPic", true);
    FH_SYM(dsp, "libdsp", p->fnVencStop, "FH_VENC_StopRecvPic", true);

    FH_SYM(p->h_isp, "libisp", p->fnIspMemInit, "API_ISP_MemInit", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspSensorRegCb, "API_ISP_SensorRegCb", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspSensorUnRegCb, "API_ISP_SensorUnRegCb", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspSensorInit, "API_ISP_SensorInit", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspSetSensorFmt, "API_ISP_SetSensorFmt", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspInit, "API_ISP_Init", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspSensorKick, "API_ISP_SensorKick", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspRun, "API_ISP_Run", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspExit, "API_ISP_Exit", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspLoadParam, "API_ISP_LoadIspParam", true);
    FH_SYM(p->h_isp, "libisp", p->fnIspSetMirrorAndflipEx, "API_ISP_SetMirrorAndflipEx", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspGetVIState, "API_ISP_GetVIState", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspGetAeInfo, "API_ISP_GetAeInfo", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspGetAeInfo2, "API_ISP_GetAeInfo2", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspGetAeStatus, "API_ISP_GetAeStatus", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspGetAeDefaultCfg, "API_ISP_GetAeDefaultCfg", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspSetAeDefaultCfg, "API_ISP_SetAeDefaultCfg", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspSetAeInfo, "API_ISP_SetAeInfo", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspGetSaturation, "API_ISP_GetSaturation", false);
    FH_SYM(p->h_isp, "libisp", p->fnIspSetSaturation, "API_ISP_SetSaturation", false);
    FH_SYM(p->h_advisp, "libadvapi_isp", p->fnAdvIspInit, "FHAdv_Isp_Init", false);
    FH_SYM(p->h_advisp, "libadvapi_isp", p->fnAdvIspSensorInit, "FHAdv_Isp_SensorInit", false);

    snprintf(path, sizeof(path), "/usr/lib/sensors/lib%s.so", st->snr_lib);
    p->h_snr = dlopen(path, RTLD_NOW | RTLD_GLOBAL);
    if (!p->h_snr) {
        snprintf(path, sizeof(path), "/lib/sensors/lib%s.so", st->snr_lib);
        p->h_snr = dlopen(path, RTLD_NOW | RTLD_GLOBAL);
    }
    if (!p->h_snr) {
        HAL_LOG_ERR("dlopen lib%s.so: %s", st->snr_lib, dlerror());
        return RSS_ERR_IO;
    }
    snprintf(sym, sizeof(sym), "%s_sensor_if", st->snr_lib);
    p->snr_ops = dlsym(p->h_snr, sym);
    if (!p->snr_ops) {
        HAL_LOG_ERR("lib%s.so: no sensor table %s", st->snr_lib, sym);
        return RSS_ERR_IO;
    }
    FH_SYM(p->h_snr, st->snr_lib, p->fnSnrCreate, "Sensor_Create", true);
    FH_SYM(p->h_snr, st->snr_lib, p->fnSnrDestroy, "Sensor_Destroy", false);

    return RSS_OK;
}

/* ================================================================
 * ISP TUNING AND 3A
 * ================================================================ */

/* divinus f2_config_load: the whole file, at least 8 KiB zero-padded. */
static int fh_tuning_load(fh_state_t *st, const char *path)
{
    unsigned char *buf;
    size_t cap = 8192, len;
    long size;
    FILE *f;
    int rc;

    f = fopen(path, "rb");
    if (!f)
        return RSS_ERR_NOENT;
    if (fseek(f, 0, SEEK_END) == 0 && (size = ftell(f)) > 0 && (size_t)size > cap)
        cap = (size_t)size;
    rewind(f);

    buf = (unsigned char *)calloc(1, cap);
    if (!buf) {
        fclose(f);
        return RSS_ERR_NOMEM;
    }
    len = fread(buf, 1, cap, f);
    fclose(f);

    if (len < 0x2c) {
        free(buf);
        HAL_LOG_WARN("isp: %s is only %zu bytes", path, len);
        return RSS_ERR_INVAL;
    }

    rc = st->pipe.fnIspLoadParam(buf);
    free(buf);
    HAL_LOG_INFO("isp: tuning %s (%zu bytes) -> API_ISP_LoadIspParam %d", path, len, rc);
    return rc ? RSS_ERR_IO : RSS_OK;
}

static int64_t fh_mono_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/*
 * fh_isp_sample_ae -- read the AE from the 3A thread, between two
 * API_ISP_Run calls, and cache what ric is given.
 *
 * Only this thread talks to the ISP library: called from rvd's control
 * thread while API_ISP_Run was running, API_ISP_GetAeStatus died inside
 * libc's memcpy within seconds (bench, 2026-11-05).
 *
 * What is cached, from the bench traces (lit room vs lens covered):
 *
 *   luma   GetAeStatus word 0 / picture area: an 8-bit mean, 145 lit, 7
 *          covered. Single reads come back 0 now and then with every AE
 *          value unchanged, so the reported luma is the largest of the last
 *          three samples (1.5 s) -- a real fall to dark still shows within
 *          that window, a lone zero never does.
 *
 *   gain   GetAeInfo: exposure x sensor gain x ISP gain, in exposure lines
 *          at 1x (word 0 x word 1/256 x word 3/64). 79-669 lit, 77648
 *          covered: a 1000:1 range, so ric's night->day test (below 25% of
 *          the settled night value) has room to fire. GetAeStatus's own
 *          exposure and gain words do not: its exposure word sits at 24000
 *          both lit and covered under the restored AE configuration, and its
 *          gain bottoms out at 1x.
 */
static void fh_isp_sample_ae(fh_state_t *st)
{
    unsigned int w[256], ai[64];
    unsigned long long area, luma, prod = 0;
    int64_t now = fh_mono_us();
    uint32_t l;
    int i;

    if (!st->pipe.fnIspGetAeStatus || now < st->exp_next_us)
        return;
    st->exp_next_us = now + 500000;

    memset(w, 0, sizeof(w));
    if (st->pipe.fnIspGetAeStatus(w))
        return;
    memset(ai, 0, sizeof(ai));
    if (st->pipe.fnIspGetAeInfo && !st->pipe.fnIspGetAeInfo(ai))
        prod = ((unsigned long long)ai[0] * ai[1] * ai[3]) / (256u * 64u);

    area = st->mode ? (unsigned long long)st->mode->width * st->mode->height : 0;
    luma = area ? w[0] / area : 0;
    if (luma > 255)
        luma = 255;

    st->luma_hist[st->luma_pos] = (uint32_t)luma;
    st->luma_pos = (st->luma_pos + 1) % 3;
    for (l = 0, i = 0; i < 3; i++)
        if (st->luma_hist[i] > l)
            l = st->luma_hist[i];

    pthread_mutex_lock(&st->exp_lock);
    st->exp_luma = l;
    st->exp_gain = prod > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)prod;
    st->exp_time = ai[0];
    st->exp_valid = prod != 0;
    pthread_mutex_unlock(&st->exp_lock);

    HAL_LOG_DBG("ae: luma %llu (reported %u) exposure x gain %llu | info %u %u %u %u (%s)", luma,
                l, prod, ai[0], ai[1], ai[2], ai[3],
                st->isp_mode == RSS_ISP_NIGHT ? "night" : "day");

    if (!st->exp_logged) {
        st->exp_logged = true;
        HAL_LOG_INFO("isp: first AE reading: luma %llu, exposure x gain %llu (info %u %u %u %u)",
                     luma, prod, ai[0], ai[1], ai[2], ai[3]);
    }
}

/*
 * The night tuning and auto-exposure.
 *
 * Bench, 2026-11-05: loading gc4653_mipi_night_attr.hex stops the AE dead --
 * exposure and gain hold the file's values whether the room is lit or the
 * lens covered. The night file rewrites the AE default configuration (and
 * clears bits in AlgCtrl, which is not a plain bitmask: OR-ing them back
 * read back as 0xbeef2210, so it is left alone). Putting back the AE default
 * configuration the day tuning set, right after the night load, brings the
 * AE back -- it then tracks the scene under the night tuning exactly as by
 * day (lit: exposure 79-669 at 1x; covered: 4853 at 4x x 4x) -- while the
 * night file's other settings stay.
 */
static unsigned int *fh_day_ae_cfg;
static void fh_sat_save_day(fh_state_t *st);

static void fh_day_ae_save(fh_state_t *st)
{
    if (fh_day_ae_cfg || !st->pipe.fnIspGetAeDefaultCfg)
        return;
    /* 16 KiB: the struct's size is unknown (it writes ~104 words); a buffer
     * smaller than it would be overrun. */
    fh_day_ae_cfg = (unsigned int *)calloc(4096, sizeof(unsigned int));
    if (fh_day_ae_cfg && st->pipe.fnIspGetAeDefaultCfg(fh_day_ae_cfg)) {
        free(fh_day_ae_cfg);
        fh_day_ae_cfg = NULL;
    }
    if (!fh_day_ae_cfg)
        HAL_LOG_WARN("isp: could not save the day AE configuration; the night tuning will run "
                     "with a fixed exposure");
}

static void fh_day_ae_restore(fh_state_t *st)
{
    unsigned int *b;
    int rc;

    if (!fh_day_ae_cfg || !st->pipe.fnIspSetAeDefaultCfg)
        return;
    b = (unsigned int *)malloc(4096 * sizeof(unsigned int));
    if (!b)
        return;
    memcpy(b, fh_day_ae_cfg, 4096 * sizeof(unsigned int));
    rc = st->pipe.fnIspSetAeDefaultCfg(b);
    free(b);
    if (rc)
        HAL_LOG_WARN("isp: restoring the AE configuration after the night tuning failed 0x%x",
                     (unsigned)rc);
}

static void *fh_isp_thread(void *arg)
{
    fh_state_t *st = (fh_state_t *)arg;

    HAL_LOG_INFO("isp: 3A thread tid %ld", syscall(SYS_gettid));

    while (__atomic_load_n(&st->isp_thread_run, __ATOMIC_ACQUIRE)) {
        /* divinus's gate: a tuning load must not race the 3A loop. */
        if (__atomic_load_n(&st->isp_pause, __ATOMIC_ACQUIRE)) {
            fh_sleep_ms(20);
            continue;
        }
        __atomic_store_n(&st->isp_busy, 1, __ATOMIC_RELEASE);
        st->pipe.fnIspRun();
        fh_isp_sample_ae(st);
        __atomic_store_n(&st->isp_busy, 0, __ATOMIC_RELEASE);
        fh_sleep_ms(10);
    }
    __atomic_store_n(&st->isp_thread_done, 1, __ATOMIC_RELEASE);
    return NULL;
}

/* Bounded join, for the reason the HiSilicon backends give: a hang at
 * shutdown is worse than a detached thread. */
static void fh_isp_thread_stop(fh_state_t *st)
{
    int waited;

    if (!st->isp_thread_started)
        return;
    __atomic_store_n(&st->isp_thread_run, 0, __ATOMIC_RELEASE);
    for (waited = 0; waited < 2000; waited += 10) {
        if (__atomic_load_n(&st->isp_thread_done, __ATOMIC_ACQUIRE)) {
            pthread_join(st->isp_thread, NULL);
            st->isp_thread_started = false;
            return;
        }
        fh_sleep_ms(10);
    }
    HAL_LOG_ERR("isp: API_ISP_Run did not return within 2 s; detaching the 3A thread");
    pthread_detach(st->isp_thread);
    st->isp_thread_started = false;
}

/*
 * The Phase 2a acceptance measurement: API_ISP_GetVIState twice, a second
 * apart. divinus's layout is {frameStart, frameEnd, framerate, width,
 * height, reserved, overflow}; unverified, so the raw words are logged and
 * only the first two are differenced.
 */
static void fh_pipe_probe(fh_state_t *st)
{
    unsigned int a[16], b[16];

    if (!st->pipe.fnIspGetVIState) {
        HAL_LOG_WARN("probe: API_ISP_GetVIState not exported; check /proc/driver/isp instead");
        return;
    }
    memset(a, 0, sizeof(a));
    memset(b, 0, sizeof(b));
    if (st->pipe.fnIspGetVIState(a)) {
        HAL_LOG_WARN("probe: API_ISP_GetVIState failed");
        return;
    }
    fh_sleep_ms(1000);
    st->pipe.fnIspGetVIState(b);
    HAL_LOG_INFO("probe: VI state %u %u %u %u %u %u %u -> %u %u %u %u %u %u %u", a[0], a[1], a[2],
                 a[3], a[4], a[5], a[6], b[0], b[1], b[2], b[3], b[4], b[5], b[6]);
    HAL_LOG_INFO("probe: %u frame starts, %u frame ends in 1 s", b[0] - a[0], b[1] - a[1]);

    /*
     * The AE readbacks, raw, once. divinus records only that GetAeInfo's
     * word 1 is a gain, word 3 the exposure and word 4 the total gain in
     * 1/64 units; the other two are unread anywhere. Heap buffers of 1 KiB
     * so an underestimated struct overruns nothing.
     */
    {
        static const char *names[] = {"API_ISP_GetAeInfo", "API_ISP_GetAeInfo2",
                                      "API_ISP_GetAeStatus"};
        int (*fns[3])(void *);
        unsigned int *w = (unsigned int *)calloc(256, sizeof(unsigned int));
        int i, rc, last, k;

        fns[0] = st->pipe.fnIspGetAeInfo;
        fns[1] = st->pipe.fnIspGetAeInfo2;
        fns[2] = st->pipe.fnIspGetAeStatus;
        for (i = 0; w && i < 3; i++) {
            if (!fns[i])
                continue;
            memset(w, 0, 256 * sizeof(unsigned int));
            rc = fns[i](w);
            for (last = -1, k = 0; k < 256; k++)
                if (w[k])
                    last = k;
            HAL_LOG_INFO("probe: %s rc=%d, %d words written; %u %u %u %u %u %u %u %u %u %u %u %u",
                         names[i], rc, last + 1, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
                         w[8], w[9], w[10], w[11]);
        }
        free(w);
    }
}

/* ================================================================
 * AFTER FH_SYS_Init: VPSS, sensor, ISP
 * ================================================================ */

/*
 * fh_vpss_configure -- divinus's f2_vpss_setup: (re)configure one VPSS output
 * channel and leave it open.
 *
 * A channel's memory is sized once, at its first configuration; asking for
 * a larger picture later needs a restart, so that is refused rather than
 * attempted. Channel 0 is first configured by fh_pipe_start at the full
 * sensor size, so the main stream can take any size up to it.
 */
int fh_vpss_configure(fh_state_t *st, int chn, unsigned int w, unsigned int h, unsigned int fps)
{
    fh_pipe_impl *p = &st->pipe;
    fh_dim dim = {w, h};
    fh_vpss_framectrl fc;
    int rc;

    if (chn < 0 || chn >= st->vpss_count)
        return RSS_ERR_INVAL;
    if (!fps)
        fps = st->mode && st->mode->fps ? st->mode->fps : 25;

    if (st->vpss_open[chn] && st->vpss_dim[chn].width == w && st->vpss_dim[chn].height == h)
        return RSS_OK;

    if (st->vpss_open[chn]) {
        p->fnVpssCloseChn((unsigned)chn);
        st->vpss_open[chn] = false;
    }

    if (st->vpss_made[chn].width) {
        if (w > st->vpss_made[chn].width || h > st->vpss_made[chn].height) {
            HAL_LOG_ERR("vpss: channel %d was sized %ux%u; %ux%u needs a restart", chn,
                        st->vpss_made[chn].width, st->vpss_made[chn].height, w, h);
            return RSS_ERR_INVAL;
        }
    } else if (p->fnVpssChnInitMem) {
        rc = p->fnVpssChnInitMem((unsigned)chn, w, h);
        if (rc)
            HAL_LOG_WARN("vpss: FH_VPSS_ChnInitMem(%d, %ux%u) %d (0x%x)", chn, w, h, rc,
                         (unsigned)rc);
        else
            st->vpss_made[chn] = dim;
    }

    rc = p->fnVpssSetChnAttr((unsigned)chn, &dim);
    if (rc) {
        HAL_LOG_ERR("vpss: FH_VPSS_SetChnAttr(%d, %ux%u) %d (0x%x)", chn, w, h, rc, (unsigned)rc);
        return RSS_ERR_IO;
    }

    /* divinus's values, as they are: {src = rate, dst = 1}; it streams at
     * the full rate with them. */
    fc.src_fps = (unsigned short)fps;
    fc.dst_fps = 1;
    rc = p->fnVpssSetFramectrl((unsigned)chn, &fc);
    if (rc) {
        HAL_LOG_ERR("vpss: FH_VPSS_SetFramectrl(%d) %d (0x%x)", chn, rc, (unsigned)rc);
        return RSS_ERR_IO;
    }

    rc = p->fnVpssOpenChn((unsigned)chn);
    if (rc) {
        HAL_LOG_ERR("vpss: FH_VPSS_OpenChn(%d) %d (0x%x)", chn, rc, (unsigned)rc);
        return RSS_ERR_IO;
    }
    st->vpss_open[chn] = true;
    st->vpss_dim[chn] = dim;
    HAL_LOG_INFO("vpss: channel %d open at %ux%u", chn, w, h);
    return RSS_OK;
}

#define FH_STEP(call, what)                                                                        \
    do {                                                                                           \
        int rc_ = (call);                                                                          \
        if (rc_) {                                                                                 \
            HAL_LOG_ERR("%s failed %d (0x%x)", (what), rc_, (unsigned)rc_);                        \
            return RSS_ERR_IO;                                                                     \
        }                                                                                          \
    } while (0)

int fh_pipe_start(fh_state_t *st)
{
    fh_pipe_impl *p = &st->pipe;
    fh_dim vi;
    char path[128];
    int rc;

    rc = fh_pipe_open_libs(st);
    if (rc)
        return rc;
    rc = fh_enc_init(st);
    if (rc)
        return rc;

    vi.width = st->mode->width;
    vi.height = st->mode->height;
    FH_STEP(p->fnVpssSetViAttr(&vi), "FH_VPSS_SetViAttr");
    FH_STEP(p->fnVpssEnable(0), "FH_VPSS_Enable(0)");
    st->vpss_enabled = true;

    st->vpss_count = FH_VPSS_CHN_MAX;
    if (p->fnVpssGetChnCap) {
        unsigned int cap[16];

        st->vpss_count = 0;
        while (st->vpss_count < FH_VPSS_CHN_MAX) {
            memset(cap, 0, sizeof(cap));
            if (p->fnVpssGetChnCap((unsigned)st->vpss_count, cap))
                break;
            HAL_LOG_INFO("vpss: channel %d capability %u %u %u %u %u %u %u %u", st->vpss_count,
                         cap[0], cap[1], cap[2], cap[3], cap[4], cap[5], cap[6], cap[7]);
            st->vpss_count++;
        }
        if (!st->vpss_count)
            st->vpss_count = 1;
    }
    HAL_LOG_INFO("vpss: %d output channel(s)", st->vpss_count);

    /* Channel 0 at the full sensor size, so its memory is sized for the
     * largest picture any later reconfiguration can ask of it. */
    rc = fh_vpss_configure(st, 0, vi.width, vi.height, st->mode->fps);
    if (rc)
        return rc;

    {
        void *obj = p->fnSnrCreate();

        HAL_LOG_INFO("sensor: Sensor_Create -> %p (sensor table %s_sensor_if at %p)", obj,
                     st->snr_lib, p->snr_ops);
        if (!obj)
            HAL_LOG_WARN("sensor: Sensor_Create returned NULL; continuing as divinus does");
    }
    st->snr_created = true;

    FH_STEP(p->fnIspMemInit(vi.width, vi.height), "API_ISP_MemInit");
    if (p->fnAdvIspInit && (rc = p->fnAdvIspInit()))
        HAL_LOG_WARN("FHAdv_Isp_Init %d (0x%x)", rc, (unsigned)rc);
    if (p->fnAdvIspSensorInit && (rc = p->fnAdvIspSensorInit()))
        HAL_LOG_WARN("FHAdv_Isp_SensorInit %d (0x%x)", rc, (unsigned)rc);
    FH_STEP(p->fnIspSensorRegCb(0, p->snr_ops), "API_ISP_SensorRegCb");
    st->isp_registered = true;
    FH_STEP(p->fnIspSensorInit(), "API_ISP_SensorInit");
    FH_STEP(p->fnIspSetSensorFmt(st->mode->format), "API_ISP_SetSensorFmt");
    FH_STEP(p->fnIspInit(), "API_ISP_Init");
    st->isp_inited = true;
    if (p->fnIspSensorKick && (rc = p->fnIspSensorKick()))
        HAL_LOG_WARN("API_ISP_SensorKick %d (0x%x)", rc, (unsigned)rc);

    snprintf(path, sizeof(path), "/etc/sensors/%s_attr.hex", st->snr_lib);
    if (fh_tuning_load(st, path) != RSS_OK) {
        snprintf(path, sizeof(path), "/usr/lib/sensors/params/%s_attr.hex", st->snr_lib);
        if (fh_tuning_load(st, path) != RSS_OK)
            HAL_LOG_WARN("isp: no day tuning loaded for %s", st->snr_lib);
    }

    if ((st->mirror || st->flip) && p->fnIspSetMirrorAndflipEx)
        p->fnIspSetMirrorAndflipEx(st->mirror, st->flip, 0);

    fh_day_ae_save(st);
    fh_sat_save_day(st);

    if (!st->exp_lock_ready) {
        if (pthread_mutex_init(&st->exp_lock, NULL))
            return RSS_ERR_IO;
        st->exp_lock_ready = true;
    }

    __atomic_store_n(&st->isp_thread_run, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&st->isp_thread_done, 0, __ATOMIC_RELEASE);
    rc = pthread_create(&st->isp_thread, NULL, fh_isp_thread, st);
    if (rc) {
        __atomic_store_n(&st->isp_thread_run, 0, __ATOMIC_RELEASE);
        HAL_LOG_ERR("isp: cannot start the 3A thread: %s", strerror(rc));
        return RSS_ERR_IO;
    }
    st->isp_thread_started = true;

    HAL_LOG_INFO("pipeline: sensor, ISP and VPSS channel 0 up at %ux%u", vi.width, vi.height);
    fh_pipe_probe(st);
    return RSS_OK;
}

/* Reverse of fh_pipe_start, flag-driven so a partial bring-up unwinds exactly
 * what came up. divinus's destroy order. */
void fh_pipe_stop(fh_state_t *st)
{
    fh_pipe_impl *p = &st->pipe;
    int i;

    /* Encoders first: an encoder still bound to a VPSS channel that is
     * about to close is the state that leaves the driver holding buffers. */
    fh_enc_teardown(st);
    fh_isp_thread_stop(st);
    if (st->exp_lock_ready && !st->isp_thread_started) {
        pthread_mutex_destroy(&st->exp_lock);
        st->exp_lock_ready = false;
        st->exp_valid = false;
    }
    if (st->isp_inited) {
        p->fnIspExit();
        st->isp_inited = false;
    }
    if (st->isp_registered) {
        p->fnIspSensorUnRegCb(0);
        st->isp_registered = false;
    }
    if (st->snr_created) {
        if (p->fnSnrDestroy)
            p->fnSnrDestroy();
        st->snr_created = false;
    }
    for (i = 0; i < FH_VPSS_CHN_MAX; i++) {
        if (st->vpss_open[i]) {
            p->fnVpssCloseChn((unsigned)i);
            st->vpss_open[i] = false;
        }
    }
    if (st->vpss_enabled) {
        p->fnVpssDisable(0);
        st->vpss_enabled = false;
    }
    memset(st->vpss_made, 0, sizeof(st->vpss_made));
    fh_pipe_close_libs(p);
}

/* ================================================================
 * OPS
 * ================================================================ */

int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height)
{
    fh_state_t *st = fh_state(ctx);

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
    fh_state_t *st = fh_state(ctx);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;
    if (!st || !st->mode || !st->mode->fps)
        return RSS_ERR_NOTSUP;
    *fps_num = st->mode->fps;
    *fps_den = 1;
    return RSS_OK;
}

/*
 * The rate is part of the sensor format word here, chosen with the mode at
 * init. Asking for the rate already running is fine; anything else needs the
 * other mode, which [sensor] mode selects -- say so rather than pretend.
 */
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den)
{
    fh_state_t *st = fh_state(ctx);

    if (!st || !st->mode || !st->mode->fps || !fps_den)
        return RSS_ERR_NOTSUP;
    if (fps_num / fps_den == st->mode->fps)
        return RSS_OK;
    HAL_LOG_WARN("sensor fps %u/%u: the running mode is %u fps; the rate comes with the mode "
                 "([sensor] mode)",
                 fps_num, fps_den, st->mode->fps);
    return RSS_ERR_NOTSUP;
}

/* ================================================================
 * DAY / NIGHT
 * ================================================================ */

/*
 * hal_isp_get_exposure -- what ric decides day and night from.
 *
 * Served from the 3A thread's cache (fh_isp_sample_ae, which documents the
 * sources), twice a second; this op never calls into the ISP library.
 * total_gain is the exposure x gain product from GetAeInfo; ae_luma the
 * picture's mean, reported by day only (see below).
 */
int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    fh_state_t *st = fh_state(ctx);
    bool valid;

    if (!st || !exposure)
        return RSS_ERR_INVAL;
    if (!st->isp_thread_started || !st->exp_lock_ready)
        return RSS_ERR_BUSY;
    if (!st->pipe.fnIspGetAeStatus)
        return RSS_ERR_NOTSUP;

    memset(exposure, 0, sizeof(*exposure));
    pthread_mutex_lock(&st->exp_lock);
    valid = st->exp_valid;
    exposure->ae_luma = st->exp_luma;
    exposure->total_gain = st->exp_gain;
    exposure->exposure_time = st->exp_time;
    pthread_mutex_unlock(&st->exp_lock);

    if (!valid)
        return RSS_ERR_BUSY;

    /*
     * Luma only under the day tuning. At night the picture's brightness is
     * not the scene's: the IR lamps light it (and before the AE fix, the
     * night tuning's frozen exposure read a covered lens as luma 53, which
     * ric took for daylight -- the double click). ric has no GPIO IR bank
     * on this board, so it would trust luma for night->day; withholding it
     * at night leaves ric its gain-ratio test (day below 25% of the settled
     * night exposure x gain), which a dark scene never passes.
     */
    exposure->valid_mask = RSS_EXPOSURE_VALID_TOTAL_GAIN | RSS_EXPOSURE_VALID_TIME;
    if (st->isp_mode != RSS_ISP_NIGHT)
        exposure->valid_mask |= RSS_EXPOSURE_VALID_AE_LUMA;
    return RSS_OK;
}

/*
 * fh_colour_set -- black-and-white at night, colour by day.
 *
 * Bench, 2026-11-05: FHAdv_Isp_SetColorMode(0) returns 0 and changes
 * nothing (FHAdv_Isp_Init fails on this board, so the advapi layer has no
 * hold on the ISP), and an all-zero saturation record hands saturation to
 * the tuning's table. The saturation record as read back:
 *
 *   day tuning    01 00 00 00  00 00 40 40  01 00 00 00  01 00 00 00
 *   night tuning  00 00 00 00  ca 00 a3 03  01 00 00 00  00 00 00 00
 *
 * The day form reads as word 0 = 1 (manual), bytes 6 and 7 = 0x40 (unity
 * colour gains). Black-and-white is that same manual record with both
 * gains 0: the day record is captured once, at boot, and written back at
 * night with bytes 6-7 cleared; the day load restores its own. Read back
 * and logged each time.
 */
static unsigned char fh_sat_day[1024];
static bool fh_sat_day_saved;

static void fh_sat_log(fh_state_t *st, const char *tag)
{
    unsigned char *sat = (unsigned char *)calloc(1, 1024);

    if (!sat)
        return;
    if (!st->pipe.fnIspGetSaturation(sat))
        HAL_LOG_INFO("isp: saturation record (%s): %02x %02x %02x %02x %02x %02x %02x %02x "
                     "%02x %02x %02x %02x %02x %02x %02x %02x",
                     tag, sat[0], sat[1], sat[2], sat[3], sat[4], sat[5], sat[6], sat[7], sat[8],
                     sat[9], sat[10], sat[11], sat[12], sat[13], sat[14], sat[15]);
    free(sat);
}

static void fh_sat_save_day(fh_state_t *st)
{
    if (fh_sat_day_saved || !st->pipe.fnIspGetSaturation)
        return;
    memset(fh_sat_day, 0, sizeof(fh_sat_day));
    if (!st->pipe.fnIspGetSaturation(fh_sat_day))
        fh_sat_day_saved = true;
}

static void fh_colour_set(fh_state_t *st, bool colour)
{
    const char *mono = getenv("FH_NIGHT_MONO");

    if (!st->pipe.fnIspGetSaturation || !st->pipe.fnIspSetSaturation)
        return;
    /*
     * Off unless FH_NIGHT_MONO=1. Bench, 2026-11-05: the manual record with
     * byte 6, byte 7 or both cleared is grey -- clearing either one alone
     * removes all colour, so they are not two independent chroma gains --
     * and every variant also squashes the picture's contrast (blacks lifted
     * to mid-grey). The record's layout is not understood well enough to
     * ship; night stays in colour (IR renders pink/purple) until libisp's
     * API_ISP_SetSaturation is read.
     */
    if (!colour && !(mono && *mono == '1')) {
        fh_sat_log(st, "night");
        return;
    }

    if (!colour && fh_sat_day_saved) {
        unsigned char *grey = (unsigned char *)malloc(1024);
        int rc;

        if (!grey)
            return;
        memcpy(grey, fh_sat_day, 1024);
        /* Bench switch: which record bytes to clear, e.g. FH_GREY_CLEAR=6
         * or =7 or =6,7 (default). The first try cleared both and came
         * out grey but with lifted blacks and little contrast. */
        {
            const char *e = getenv("FH_GREY_CLEAR");
            char buf[32];
            char *tok, *save = NULL;

            snprintf(buf, sizeof(buf), "%s", e ? e : "6,7");
            for (tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
                int i = atoi(tok);

                if (i >= 0 && i < 64)
                    grey[i] = 0;
            }
        }
        rc = st->pipe.fnIspSetSaturation(grey);
        free(grey);
        if (rc)
            HAL_LOG_WARN("isp: night black-and-white: SetSaturation failed 0x%x", (unsigned)rc);
    }
    fh_sat_log(st, colour ? "day" : "night");
}

/*
 * hal_isp_set_running_mode -- swap the ISP tuning, divinus's way: gate the
 * 3A loop, load <lib>_night_attr.hex or <lib>_attr.hex, put mirror/flip
 * back (the load resets them), let the loop run again, and ask every video
 * encoder for an IDR so players resync on the new look.
 *
 * The IR-cut filter and the lamps are not this op's business: ric moves
 * the filter, and the PB4's PWM LEDs are driven outside raptor.
 */
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode)
{
    fh_state_t *st = fh_state(ctx);
    char path[128];
    int rc, i, waited;

    if (!st || (mode != RSS_ISP_DAY && mode != RSS_ISP_NIGHT))
        return RSS_ERR_INVAL;
    if (!st->isp_inited || !st->isp_thread_started)
        return RSS_ERR_BUSY;
    if (mode == st->isp_mode)
        return RSS_OK;

    snprintf(path, sizeof(path), "/etc/sensors/%s%s", st->snr_lib,
             mode == RSS_ISP_NIGHT ? "_night_attr.hex" : "_attr.hex");
    if (access(path, R_OK) != 0) {
        snprintf(path, sizeof(path), "/usr/lib/sensors/params/%s%s", st->snr_lib,
                 mode == RSS_ISP_NIGHT ? "_night_attr.hex" : "_attr.hex");
        if (access(path, R_OK) != 0) {
            HAL_LOG_WARN("isp: no %s tuning for %s; keeping the current one",
                         mode == RSS_ISP_NIGHT ? "night" : "day", st->snr_lib);
            st->isp_mode = mode;
            return RSS_OK;
        }
    }

    __atomic_store_n(&st->isp_pause, 1, __ATOMIC_RELEASE);
    for (waited = 0; waited < 500 && __atomic_load_n(&st->isp_busy, __ATOMIC_ACQUIRE);
         waited += 10)
        fh_sleep_ms(10);

    /*
     * Carry the AE's operating point across the load. Bench, 2026-11-05: a
     * tuning load restarts the AE from the file's stored exposure, and in a
     * lit room the first seconds after the day file loaded read luma 2 --
     * ric's day check, 3 s after the switch, took that for darkness and
     * reverted. Reading the exposure/gain before the load and writing it
     * back after lets the picture continue where it was; the AE carries on
     * from there under the new tuning.
     */
    {
        unsigned int ae[64];
        bool have_ae = false;

        memset(ae, 0, sizeof(ae));
        if (st->pipe.fnIspGetAeInfo && st->pipe.fnIspSetAeInfo && !st->pipe.fnIspGetAeInfo(ae) &&
            ae[0])
            have_ae = true;

        rc = fh_tuning_load(st, path);
        if ((st->mirror || st->flip) && st->pipe.fnIspSetMirrorAndflipEx)
            st->pipe.fnIspSetMirrorAndflipEx(st->mirror, st->flip, 0);
        if (mode == RSS_ISP_NIGHT) {
            fh_day_ae_restore(st);
        }
        if (have_ae) {
            int r2 = st->pipe.fnIspSetAeInfo(ae);

            HAL_LOG_INFO("isp: AE point carried across the load: %u %u %u %u (rc %d)", ae[0],
                        ae[1], ae[2], ae[3], r2);
        }
        memset(st->luma_hist, 0, sizeof(st->luma_hist));
        fh_colour_set(st, mode != RSS_ISP_NIGHT);
    }

    __atomic_store_n(&st->isp_pause, 0, __ATOMIC_RELEASE);
    fh_sleep_ms(200);

    for (i = 0; i < FH_VENC_CHN_MAX; i++) {
        fh_enc_chn_t *enc = &st->enc[i];

        if (enc->created && (enc->codec == RSS_CODEC_H264 || enc->codec == RSS_CODEC_H265))
            st->pipe.fnVencRequestIdr((unsigned)i);
    }

    if (rc == RSS_OK)
        st->isp_mode = mode;
    HAL_LOG_INFO("isp: %s mode, tuning %s %s", mode == RSS_ISP_NIGHT ? "night" : "day", path,
                 rc == RSS_OK ? "loaded" : "FAILED");
    return rc;
}

int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode)
{
    fh_state_t *st = fh_state(ctx);

    if (!st || !mode)
        return RSS_ERR_INVAL;
    *mode = st->isp_mode;
    return RSS_OK;
}
