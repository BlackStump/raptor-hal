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

static void fh_sleep_ms(unsigned int ms)
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
    snprintf(v, sizeof(v), "mem_1_%u", jpg);
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

static void *fh_isp_thread(void *arg)
{
    fh_state_t *st = (fh_state_t *)arg;

    while (__atomic_load_n(&st->isp_thread_run, __ATOMIC_ACQUIRE)) {
        st->pipe.fnIspRun();
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
}

/* ================================================================
 * AFTER FH_SYS_Init: VPSS, sensor, ISP
 * ================================================================ */

static int fh_vpss_open(fh_state_t *st, int chn, unsigned int w, unsigned int h, unsigned int fps)
{
    fh_pipe_impl *p = &st->pipe;
    fh_dim dim = {w, h};
    fh_vpss_framectrl fc;
    int rc;

    if (p->fnVpssChnInitMem && (rc = p->fnVpssChnInitMem((unsigned)chn, w, h)))
        HAL_LOG_WARN("vpss: FH_VPSS_ChnInitMem(%d, %ux%u) %d (0x%x)", chn, w, h, rc, (unsigned)rc);

    rc = p->fnVpssSetChnAttr((unsigned)chn, &dim);
    if (rc) {
        HAL_LOG_ERR("vpss: FH_VPSS_SetChnAttr(%d, %ux%u) %d (0x%x)", chn, w, h, rc, (unsigned)rc);
        return RSS_ERR_IO;
    }

    /* divinus's values, as they are: {src = rate, dst = 1}. Phase 2b, with an
     * encoder counting output frames, is where their meaning gets measured. */
    fc.src_fps = (unsigned short)(fps ? fps : 25);
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
    rc = fh_vpss_open(st, 0, vi.width, vi.height, st->mode->fps);
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

    fh_isp_thread_stop(st);
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
