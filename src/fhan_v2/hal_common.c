/*
 * fhan_v2/hal_common.c -- Raptor HAL common layer, Fullhan FH885xV200
 *
 * Phase 1 skeleton, modelled on src/hisi_v5/hal_common.c at its own Phase 1
 * (e44c4c1): the vtable publishes only the ops that exist, RSS_HAL_CALL()
 * answers RSS_ERR_NOTSUP for the rest, and a Phase 1 rvd comes up, loads the
 * vendor libraries, reaches FH_SYS_Init, logs the SDK version and the
 * package, finds no framesource and exits cleanly.
 *
 * Also the home of the uClibc-ABI bridge, for the archive-extraction reason
 * gen4 sets out: raptor-hal is a static archive and only members something
 * already references get linked. This file defines rss_hal_create, so it is
 * always extracted, and so is everything the vendor libraries expect the
 * executable to export.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "fh_state.h"

#include <dlfcn.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <syslog.h>
#include <unistd.h>

/* ================================================================
 * LOGGING -- same shape as every other backend
 * ================================================================ */

static const char *hal_level_str[] = {"FTL", "ERR", "WRN", "INF", "DBG"};

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
{
    const char *basename;
    va_list ap;

    if (level < 0)
        level = 0;
    if (level > 4)
        level = 4;

    basename = strrchr(file, '/');
    if (basename)
        file = basename + 1;
    fprintf(stderr, "[HAL %s] %s:%d: ", hal_level_str[level], file, line);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

rss_hal_log_func_t rss_hal_log_fn = hal_log_stderr;

void rss_hal_set_log_func(rss_hal_log_func_t func)
{
    rss_hal_log_fn = func ? func : hal_log_stderr;
}

extern const rss_hal_caps_t g_hal_caps;

/* One HAL context per process, as everywhere in raptor. */
static fh_state_t *g_fh;

#ifdef HAL_MODULE_VIDEO
int hal_gpio_set(void *ctx, int pin, int value);
int hal_gpio_get(void *ctx, int pin, int *value);
int hal_ircut_set(void *ctx, int state);
#endif

/* ================================================================
 * THE uClIBC-ABI BRIDGE
 *
 * Every Fullhan vendor library is a uClibc build (NEEDED libc.so.0) running
 * on musl. Measured on the PB4 image, 2026-11-05: the undefined-symbol
 * closure over the vendor set, minus musl's exports and minus what the
 * libraries define for each other, leaves exactly:
 *
 *   __assert                                  real, defined below
 *   _ITM_{de,}registerTMCloneTable,
 *   _Jv_RegisterClasses,
 *   __{de,}register_frame_info                gcc crt weak refs, harmless
 *
 * __ctype_b, __fgetc_unlocked and _stdlib_mb_cur_max -- gen4's trio -- are
 * not in it: OpenIPC's musl exports them already.
 *
 * And, invisible to that closure exactly as on gen4, `mmap`: libdbi, libdsp,
 * libispcore, libmipi and libacw_mpi import it, it resolves, and it resolves
 * to the wrong ABI. Their call sites were read (2026-11-05): fd at [sp],
 * the offset as one 32-bit word at [sp,#4], nothing at [sp,#8]
 * (libdbi Mem_FindVirtAddress: `stm sp, {r3, r6}`, r6 the page-aligned
 * physical address). musl's mmap reads a 64-bit off_t at sp+8.
 * ================================================================ */

extern long syscall(long number, ...);

typedef struct {
    const char *dli_fname;
    void *dli_fbase;
    const char *dli_sname;
    void *dli_saddr;
} fh_dl_info;

extern int dladdr(const void *addr, fh_dl_info *info);

/*
 * SYS_mmap2 takes the offset in 4096-byte pages, so one function serves both
 * conventions below the libc layer. The page count stays a 32-bit argument:
 * GCC turns the shim into a sibling call that reuses the caller's outgoing
 * argument area, and a uClibc caller allocated eight bytes of it. gen4
 * measured what a wider argument does there (src/hisi_v4/hal_common.c).
 */
#if defined(__arm__)
static void *fh_mmap_pages(void *addr, size_t len, int prot, int flags, int fd,
                           unsigned long pgoff)
{
    return (void *)syscall(SYS_mmap2, addr, len, prot, flags, fd, pgoff);
}

static const char *fh_own_object(void)
{
    static const char *name;
    static int resolved;
    fh_dl_info info;

    if (!__atomic_load_n(&resolved, __ATOMIC_ACQUIRE)) {
        if (dladdr((const void *)(uintptr_t)&fh_mmap_pages, &info) && info.dli_fname)
            name = info.dli_fname;
        __atomic_store_n(&resolved, 1, __ATOMIC_RELEASE);
    }
    return name;
}

/*
 * A caller is a vendor library unless it is this executable or one of
 * raptor's librss_* libraries. An address the loader cannot place counts as
 * ours: guessing musl for a vendor library fails loudly at bring-up, guessing
 * uClibc for raptor's own code is a later SIGBUS somewhere unrelated.
 */
static bool fh_caller_is_vendor(const void *ra)
{
    const char *own = fh_own_object();
    fh_dl_info info;
    const char *base;

    if (!own)
        return false;
    if (!dladdr(ra, &info) || !info.dli_fname)
        return false;
    if (strcmp(info.dli_fname, own) == 0)
        return false;

    base = strrchr(info.dli_fname, '/');
    base = base ? base + 1 : info.dli_fname;
    return strncmp(base, "librss_", 7) != 0;
}

/*
 * Exported as `mmap` by assembler name, so it does not collide with the
 * prototype <sys/mman.h> may have introduced. w4/w8/w12 are the stack words
 * the two conventions disagree about; only the ones the caller wrote are read.
 */
void *fh_mmap_shim(void *addr, size_t len, int prot, int flags, int fd, unsigned long w4,
                   unsigned long w8, unsigned long w12) __asm__("mmap");

__attribute__((used)) void *fh_mmap_shim(void *addr, size_t len, int prot, int flags, int fd,
                                         unsigned long w4, unsigned long w8, unsigned long w12)
{
    void *ra = __builtin_return_address(0);
    unsigned long long off;

    if (fh_caller_is_vendor(ra))
        off = w4;
    else
        off = (unsigned long long)w8 | ((unsigned long long)w12 << 32);

    return fh_mmap_pages(addr, len, prot, flags, fd, (unsigned long)(off >> 12));
}
#endif /* __arm__ */

/*
 * uClibc's assert entry point; musl has only __assert_fail. A stub that
 * returns (divinus's choice) lets a failed vendor assertion carry on as if
 * it had passed, so this one reports and aborts, which is what the vendor's
 * own libc would have done.
 */
void fh_uclibc_assert(const char *expr, const char *file, unsigned int line,
                      const char *func) __asm__("__assert");

__attribute__((used, noreturn)) void fh_uclibc_assert(const char *expr, const char *file,
                                                     unsigned int line, const char *func)
{
    fprintf(stderr, "vendor assertion failed: %s (%s:%u, %s)\n", expr ? expr : "?",
            file ? file : "?", line, func ? func : "?");
    syslog(LOG_ERR, "vendor assertion failed: %s (%s:%u, %s)", expr ? expr : "?",
           file ? file : "?", line, func ? func : "?");
    abort();
}

/*
 * fh_check_exports -- prove the executable really exports the bridge.
 *
 * Looks the two names up the way a vendor library will, from the global
 * scope, and checks they land in this object. A miss means the link dropped
 * them (no --export-dynamic, member not extracted) and turns an
 * unattributable fault inside libdsp into one log line. Non-fatal.
 */
static void fh_check_exports(void)
{
#if defined(__arm__)
    static const char *names[] = {"mmap", "__assert"};
    const char *own = fh_own_object();
    void *self = dlopen(NULL, RTLD_NOW);
    size_t i;

    if (!self) {
        HAL_LOG_WARN("bridge: dlopen(NULL) failed, exports unchecked");
        return;
    }
    for (i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        void *sym = dlsym(self, names[i]);
        fh_dl_info info;

        if (!sym || !dladdr(sym, &info) || !info.dli_fname || !own ||
            strcmp(info.dli_fname, own) != 0)
            HAL_LOG_ERR("bridge: '%s' resolves to %s, not to this executable -- "
                        "was rvd linked with --export-dynamic?",
                        names[i], (sym && dladdr(sym, &info) && info.dli_fname) ?
                        info.dli_fname : "nothing");
        else
            HAL_LOG_DBG("bridge: '%s' exported from %s", names[i], own);
    }
    dlclose(self);
#endif
}

/* ================================================================
 * CHIP IDENTIFICATION
 * ================================================================ */

/* Copy one whitespace/comma-delimited token, upper-cased. */
static void fh_copy_token(char *out, size_t out_size, const char *p)
{
    size_t n = 0;

    while (*p == ' ' || *p == '\t' || *p == ':')
        p++;
    while (p[n] && p[n] != ' ' && p[n] != '\t' && p[n] != '\n' && p[n] != ',' &&
           n + 1 < out_size) {
        char ch = p[n];

        out[n] = (ch >= 'a' && ch <= 'z') ? (char)(ch - 'a' + 'A') : ch;
        n++;
    }
    out[n] = '\0';
}

/*
 * The package name: /proc/cpuinfo's Hardware line first (the kernel's own
 * board name, "FH8856V200" on the PB4), then any FH8 token in
 * /proc/driver/chip. See fh_common.h for why the PMU registers are not used.
 */
static int fh_read_chip_name(char *out, size_t out_size)
{
    char line[200];
    FILE *f;

    f = fopen(FH_CPUINFO, "r");
    if (f) {
        while (fgets(line, sizeof(line), f)) {
            if (!strncmp(line, "Hardware", 8)) {
                char *p = strchr(line, ':');

                if (p) {
                    fh_copy_token(out, out_size, p + 1);
                    if (!strncmp(out, "FH8", 3)) {
                        fclose(f);
                        return RSS_OK;
                    }
                }
            }
        }
        fclose(f);
    }

    f = fopen(FH_CHIP_PROC, "r");
    if (f) {
        while (fgets(line, sizeof(line), f)) {
            char *p = strstr(line, "FH8");

            if (!p)
                p = strstr(line, "fh8");
            if (p) {
                fh_copy_token(out, out_size, p);
                fclose(f);
                return RSS_OK;
            }
        }
        fclose(f);
    }

    return RSS_ERR_NOENT;
}

/* The V200 die, any package: FH8852V2xx, FH8856V2xx, FH8858V2xx. */
static bool fh_chip_is_v2(const char *name)
{
    return (!strncmp(name, "FH8852V2", 8) || !strncmp(name, "FH8856V2", 8) ||
            !strncmp(name, "FH8858V2", 8));
}

/* ================================================================
 * LIBRARIES
 * ================================================================ */

static void *fh_sym(void *h, const char *lib, const char *name, bool required)
{
    void *p = dlsym(h, name);

    if (!p && required)
        HAL_LOG_ERR("%s: missing symbol %s", lib, name);
    return p;
}

static void fh_libs_close(fh_sys_impl *sys)
{
    if (sys->h_dsp)
        dlclose(sys->h_dsp);
    if (sys->h_vmm)
        dlclose(sys->h_vmm);
    memset(sys, 0, sizeof(*sys));
}

static int fh_libs_open(fh_sys_impl *sys)
{
    sys->h_vmm = dlopen(FH_LIB_VMM, RTLD_NOW | RTLD_GLOBAL);
    if (!sys->h_vmm) {
        HAL_LOG_ERR("dlopen %s: %s", FH_LIB_VMM, dlerror());
        goto fail;
    }

    sys->h_dsp = dlopen(FH_LIB_DSP, RTLD_NOW | RTLD_GLOBAL);
    if (!sys->h_dsp) {
        HAL_LOG_ERR("dlopen %s: %s", FH_LIB_DSP, dlerror());
        goto fail;
    }

    *(void **)&sys->fnSysInit = fh_sym(sys->h_dsp, FH_LIB_DSP, "FH_SYS_Init", true);
    *(void **)&sys->fnSysExit = fh_sym(sys->h_dsp, FH_LIB_DSP, "FH_SYS_Exit", true);
    *(void **)&sys->fnSysGetVersion = fh_sym(sys->h_dsp, FH_LIB_DSP, "FH_SYS_GetVersion", true);
    *(void **)&sys->fnSysGetChipId = fh_sym(sys->h_dsp, FH_LIB_DSP, "FH_SYS_GetChipID", false);

    if (!sys->fnSysInit || !sys->fnSysExit || !sys->fnSysGetVersion)
        goto fail;

    return RSS_OK;

fail:
    fh_libs_close(sys);
    return RSS_ERR_IO;
}

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

static void fh_teardown(fh_state_t *st)
{
#ifdef HAL_MODULE_VIDEO
    fh_pipe_stop(st);
#endif
    if (st->sys_inited) {
        int rc = st->sys.fnSysExit();

        if (rc)
            HAL_LOG_WARN("sys: FH_SYS_Exit returned %d (0x%x)", rc, (unsigned)rc);
        st->sys_inited = false;
    }
}

/*
 * hal_init -- open the libraries and bring the SDK up as far as FH_SYS_Init.
 *
 * NOT TEARDOWN-FIRST, unlike both HiSilicon backends, and deliberately so
 * for now: whether FH_SYS_Exit is harmless on an SDK that was never
 * initialised has not been measured on this part, and Phase 1's job is to
 * find out what a clean start does. A daemon killed mid-run and restarted
 * without a module reload is the case that would need it; that is a Phase 2
 * measurement.
 *
 * Phase 1 writes nothing to /proc/driver/{vpu,isp,enc,jpeg}. divinus sizes
 * the drivers' buffers there *before* FH_SYS_Init, from the sensor geometry
 * and the stream configuration -- neither of which exists until Phase 2.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    fh_state_t *st;
    int ret;

    if (!c || !cfg || cfg->sensor_count < 1 || cfg->sensor_count > RSS_MAX_SENSORS)
        return RSS_ERR_INVAL;

    if (c->initialized) {
        HAL_LOG_ERR("hal_init: already initialized");
        return RSS_ERR_BUSY;
    }

    if (cfg->sensor_count > 1)
        HAL_LOG_WARN("hal_init: %d sensors requested, the Fullhan backend drives 1",
                     cfg->sensor_count);

    memcpy(&c->multi_cfg, cfg, sizeof(c->multi_cfg));
    c->sensor_count = 1;
    memcpy(&c->sensors[0], &cfg->sensors[0], sizeof(c->sensors[0]));

    st = (fh_state_t *)calloc(1, sizeof(*st));
    if (!st)
        return RSS_ERR_NOMEM;
    c->platform = st;

    snprintf(st->sensor_name, sizeof(st->sensor_name), "%s", cfg->sensors[0].name);

    /* Before the first vendor dlopen: RTLD_NOW binds libdsp's mmap at load. */
    fh_check_exports();

    ret = fh_libs_open(&st->sys);
    if (ret)
        goto err_free;

    if (fh_read_chip_name(st->chip_name, sizeof(st->chip_name)) != RSS_OK)
        HAL_LOG_WARN("chip: no FH8 name in %s or %s", FH_CPUINFO, FH_CHIP_PROC);
    HAL_LOG_INFO("chip: %s", st->chip_name[0] ? st->chip_name : "unknown");

#ifdef HAL_MODULE_VIDEO
    /* Phase 2a: the drivers take their buffer sizes before the SDK starts. */
    ret = fh_pipe_prepare(st, &cfg->sensors[0]);
    if (ret)
        goto err_unload;
#endif

    ret = st->sys.fnSysInit();
    if (ret) {
        HAL_LOG_ERR("sys: FH_SYS_Init failed %d (0x%x)", ret, (unsigned)ret);
        ret = RSS_ERR_IO;
        goto err_unload;
    }
    st->sys_inited = true;

    /*
     * Asked after FH_SYS_Init, not before: before it, the first bench run
     * (2026-11-05) got no version back, while libdsp prints its own version
     * during init. Raw words logged until the layouts are read.
     */
    {
        unsigned int w[FH_PROBE_WORDS];
        int rc;

        memset(w, 0, sizeof(w));
        rc = st->sys.fnSysGetVersion(w);
        HAL_LOG_INFO("probe: FH_SYS_GetVersion rc=%d words %08x %08x %08x %08x %08x %08x", rc,
                     w[0], w[1], w[2], w[3], w[4], w[5]);
        if (!rc)
            snprintf(st->sdk_version, sizeof(st->sdk_version),
                     "FH MPI %08x %08x pkg %08x", w[0], w[1], w[2]);

        /* FH_SYS_GetChipID is not called: on this kernel it fails
         * (0xa1014007) and the chip driver logs "Error: unknown chip" for
         * package 0x7850b every time. /proc/cpuinfo names the part. */
    }
    HAL_LOG_INFO("sdk: %s", st->sdk_version[0] ? st->sdk_version : "version unavailable");

#ifdef HAL_MODULE_VIDEO
    ret = fh_pipe_start(st);
    if (ret)
        goto err_unload;
    HAL_LOG_INFO("hal_init: sensor and ISP up; framesources and encoders are Phase 2b");
#else
    HAL_LOG_INFO("hal_init: SDK up to FH_SYS_Init");
#endif

    g_fh = st;
    c->initialized = true;
    return RSS_OK;

err_unload:
    fh_teardown(st);
    fh_libs_close(&st->sys);
err_free:
    free(st);
    c->platform = NULL;
    return ret;
}

static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    fh_state_t *st = fh_state(ctx);

    if (!c)
        return RSS_ERR_INVAL;
    if (!st)
        return RSS_OK;

    fh_teardown(st);
    if (g_fh == st)
        g_fh = NULL;
    fh_libs_close(&st->sys);

    free(st);
    c->platform = NULL;
    c->initialized = false;
    return RSS_OK;
}

static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? &c->caps : &g_hal_caps;
}

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    fh_state_t *st = fh_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->sdk_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->sdk_version);
    return RSS_OK;
}

static int hal_sys_get_cpu_info(void *ctx, char *buf, int len)
{
    fh_state_t *st = fh_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->chip_name[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->chip_name);
    return RSS_OK;
}

/* ================================================================
 * OPS VTABLE -- only what exists; the rest is RSS_ERR_NOTSUP
 * ================================================================ */

static const rss_hal_ops_t g_ops = {
    .init = hal_init,
    .deinit = hal_deinit,
    .get_caps = hal_get_caps,

    .sys_get_version = hal_sys_get_version,
    .sys_get_cpu_info = hal_sys_get_cpu_info,

#ifdef HAL_MODULE_VIDEO
    .isp_get_sensor_attr = hal_isp_get_sensor_attr,
    .isp_get_sensor_fps = hal_isp_get_sensor_fps,
    .isp_set_sensor_fps = hal_isp_set_sensor_fps,

    .gpio_set = hal_gpio_set,
    .gpio_get = hal_gpio_get,
    .ircut_set = hal_ircut_set,
#endif
};

/* ================================================================
 * FACTORY
 * ================================================================ */

rss_hal_ctx_t *rss_hal_create(void)
{
    return rss_hal_create_backend("imp");
}

/* "imp" is the default name every platform's config carries; here it means
 * "the built-in backend". Anything else is refused rather than substituted. */
rss_hal_ctx_t *rss_hal_create_backend(const char *backend)
{
    rss_hal_ctx_t *ctx;

    if (backend && strcmp(backend, "imp") != 0)
        return NULL;

    ctx = (rss_hal_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx)
        return NULL;

    ctx->ops = &g_ops;
    memcpy(&ctx->caps, &g_hal_caps, sizeof(ctx->caps));
    return ctx;
}

void rss_hal_destroy(rss_hal_ctx_t *ctx)
{
    int i;

    if (!ctx)
        return;

    for (i = 0; i < RSS_MAX_ENC_CHANNELS; i++) {
        free(ctx->scratch_buf[i]);
        ctx->scratch_buf[i] = NULL;
        free(ctx->nal_arrays[i]);
        ctx->nal_arrays[i] = NULL;
    }
    free(ctx);
}

const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx)
{
    return ctx ? ctx->ops : NULL;
}

/* ================================================================
 * SYSTEM INFO (no vtable, called directly by the daemons)
 * ================================================================ */

int rss_hal_get_imp_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;
    if (!g_fh || !g_fh->sdk_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)size, "%s", g_fh->sdk_version);
    return RSS_OK;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    (void)buf;
    (void)size;
    return RSS_ERR_NOTSUP;
}

const char *rss_hal_get_cpu_info(void)
{
    static char cpu[24];
    static bool loaded = false;

    if (loaded)
        return cpu;
    loaded = true;

    if (g_fh && g_fh->chip_name[0])
        snprintf(cpu, sizeof(cpu), "%s", g_fh->chip_name);
    else if (fh_read_chip_name(cpu, sizeof(cpu)) != RSS_OK)
        snprintf(cpu, sizeof(cpu), "%s", HAL_PLATFORM_NAME);

    return cpu;
}

const char *rss_hal_get_platform_name(void)
{
    return HAL_PLATFORM_NAME;
}

/*
 * rss_hal_check_platform -- refuse a part that is not an FH885xV200.
 *
 * Unreadable is a warning, not a mismatch: /proc/driver/chip exists only once
 * the vendor modules are loaded, and the dlopen will name that failure better.
 */
void rss_hal_check_platform(const char *name)
{
    char chip[24];

    if (fh_read_chip_name(chip, sizeof(chip)) != RSS_OK) {
        HAL_LOG_WARN("platform check: no FH8 name in %s or %s, assuming %s", FH_CPUINFO,
                     FH_CHIP_PROC, HAL_PLATFORM_NAME);
        return;
    }

    if (!fh_chip_is_v2(chip)) {
        fprintf(stderr, "FATAL: built for %s (FH885xV200) but the board reports '%s'\n",
                HAL_PLATFORM_NAME, chip);
        openlog(name ? name : "raptor", LOG_PID, LOG_DAEMON);
        syslog(LOG_ERR, "FATAL: built for %s (FH885xV200) but the board reports '%s'",
               HAL_PLATFORM_NAME, chip);
        closelog();
        _exit(1);
    }

    HAL_LOG_DBG("platform check: built for %s, running on %s", HAL_PLATFORM_NAME, chip);
}
