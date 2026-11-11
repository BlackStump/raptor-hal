/*
 * xiongmai/hal_common.c -- Raptor HAL common layer, Xiongmai XM530 family
 *
 * Phase 2a: sensor, ISP and the VI framesources (xm_pipeline.c, xm_isp.c).
 * Shaped like src/fhan_v2/hal_common.c: the vtable publishes only the ops
 * that exist,
 * RSS_HAL_CALL() answers RSS_ERR_NOTSUP for the rest. A Phase 2a rvd brings
 * the sensor and ISP up, creates its framesources and stops at the first
 * encoder, which is Phase 2b.
 *
 * What it does not need, unlike Fullhan: a uClibc bridge or any dlopen. rvd
 * links openxiongmai's libxmhost.so (the MPI built in), libispapp_XM530.so,
 * the open libisp_XM530.so and libsns.so, all built with the same musl
 * toolchain by raptor's Makefile.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "xm_state.h"

#include <re_sys.h>

#include <dirent.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
static xm_state_t *g_xm;

/* Defined in src/hal_gpio.c (vendor-neutral sysfs) */
int hal_gpio_set(void *ctx, int pin, int value);
int hal_gpio_get(void *ctx, int pin, int *value);
int hal_ircut_set(void *ctx, int state);

/* ================================================================
 * CHIP IDENTITY
 * ================================================================ */

/*
 * The package this image was built for, from SOC_MODEL (mk sets
 * -DSOC_MODEL_<MODEL>). The XM530 and XM550 run one kernel and one library
 * set, so this is what tells a log reader which board it came from.
 */
#if defined(SOC_MODEL_XM550)
#define XM_SOC_MODEL "XM550"
#elif defined(SOC_MODEL_XM530)
#define XM_SOC_MODEL "XM530"
#else
#define XM_SOC_MODEL HAL_PLATFORM_NAME
#endif

/*
 * The kernel's own name for the part: the "Hardware" line of /proc/cpuinfo.
 * Not yet measured on the XM550 -- Phase 1's log is where it gets read for
 * the first time, which is why it is reported and never enforced.
 */
static int xm_read_chip_name(char *out, size_t out_size)
{
    char line[128];
    FILE *f;
    int ret = RSS_ERR_IO;

    f = fopen("/proc/cpuinfo", "r");
    if (!f)
        return RSS_ERR_IO;

    while (fgets(line, sizeof(line), f)) {
        char *v;
        size_t n;

        if (strncmp(line, "Hardware", 8) != 0)
            continue;
        v = strchr(line, ':');
        if (!v)
            break;
        v++;
        while (*v == ' ' || *v == '\t')
            v++;
        n = strcspn(v, "\r\n");
        if (n == 0)
            break;
        if (n >= out_size)
            n = out_size - 1;
        memcpy(out, v, n);
        out[n] = '\0';
        ret = RSS_OK;
        break;
    }

    fclose(f);
    return ret;
}

/* ================================================================
 * PRE-FLIGHT
 * ================================================================ */

static bool xm_streamer_running(const char *name)
{
    struct dirent *de;
    bool found = false;
    DIR *d;

    d = opendir("/proc");
    if (!d)
        return false;

    while (!found && (de = readdir(d)) != NULL) {
        char path[64], comm[32];
        FILE *f;

        if (de->d_name[0] < '1' || de->d_name[0] > '9')
            continue;
        snprintf(path, sizeof(path), "/proc/%s/comm", de->d_name);
        f = fopen(path, "r");
        if (!f)
            continue;
        if (fgets(comm, sizeof(comm), f)) {
            comm[strcspn(comm, "\n")] = '\0';
            found = strcmp(comm, name) == 0;
        }
        fclose(f);
    }

    closedir(d);
    return found;
}

/*
 * Checked here because XM_MPI_SYS_Init does not check them all: a missing
 * /dev/vi is logged by libmpi and then reported as success.
 */
static int xm_check_devices(void)
{
    static const char *const nodes[] = {XM_DEV_MMZ, XM_DEV_VI, XM_DEV_RGN};
    int missing = 0;
    size_t i;

    for (i = 0; i < sizeof(nodes) / sizeof(nodes[0]); i++) {
        if (access(nodes[i], R_OK | W_OK) != 0) {
            HAL_LOG_ERR("sys: %s missing or not accessible -- were the modules loaded by "
                        "load_xiongmai?",
                        nodes[i]);
            missing++;
        }
    }

    return missing ? RSS_ERR_IO : RSS_OK;
}

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

/*
 * hal_init -- bring the system, the sensor and the ISP up.
 *
 * TEARDOWN-FIRST BY CONSTRUCTION: xmhost_start stops what a previous
 * program left running (the 3D NR, VI channels 1 and 0) and then
 * XM_MPI_SYS_Init resets MMZ, so a previous rvd that was killed rather than
 * closed leaves nothing a restart has to undo -- the reverse of HiMPP,
 * where hal_init has to call SYS_Exit and VB_Exit itself. The same reset is
 * why a running majestic is refused rather than shared with.
 *
 * There is no XM_MPI_SYS_Exit; hal_deinit disables the VI channels and
 * leaves the rest to the next start.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    xm_state_t *st;
    int ret;

    if (!c || !cfg || cfg->sensor_count < 1 || cfg->sensor_count > RSS_MAX_SENSORS)
        return RSS_ERR_INVAL;

    if (c->initialized) {
        HAL_LOG_ERR("hal_init: already initialized");
        return RSS_ERR_BUSY;
    }

    if (cfg->sensor_count > 1)
        HAL_LOG_WARN("hal_init: %d sensors requested, the Xiongmai backend drives 1",
                     cfg->sensor_count);

    memcpy(&c->multi_cfg, cfg, sizeof(c->multi_cfg));
    c->sensor_count = 1;
    memcpy(&c->sensors[0], &cfg->sensors[0], sizeof(c->sensors[0]));

    st = (xm_state_t *)calloc(1, sizeof(*st));
    if (!st)
        return RSS_ERR_NOMEM;
    c->platform = st;

    snprintf(st->sensor_name, sizeof(st->sensor_name), "%s", cfg->sensors[0].name);
    snprintf(st->sdk_version, sizeof(st->sdk_version), "openxiongmai libmpi (%s)",
             XM_SOC_MODEL);

    if (xm_read_chip_name(st->chip_name, sizeof(st->chip_name)) != RSS_OK)
        HAL_LOG_WARN("chip: no Hardware line in /proc/cpuinfo");
    HAL_LOG_INFO("chip: %s, built for %s, sensor %s",
                 st->chip_name[0] ? st->chip_name : "unknown", XM_SOC_MODEL,
                 st->sensor_name[0] ? st->sensor_name : "(unnamed)");

    if (xm_streamer_running(XM_RIVAL_STREAMER)) {
        HAL_LOG_ERR("sys: %s is running; its MMZ buffers would be reset under it. "
                    "Stop it first: /etc/init.d/S95majestic stop",
                    XM_RIVAL_STREAMER);
        ret = RSS_ERR_BUSY;
        goto err_free;
    }

    ret = xm_check_devices();
    if (ret)
        goto err_free;

#ifdef HAL_MODULE_VIDEO
    /* Phase 2a: libxmhost's start (3D NR stopped, VI quiet, SYS_Init),
     * then the sensor and the ISP through libispapp. */
    ret = xm_pipe_start(st);
    if (ret)
        goto err_free;
    HAL_LOG_INFO("hal_init: sensor and ISP up; framesources are VI 0 and extended 1; "
                 "the encoder is Phase 2b");
#else
    ret = XM_MPI_SYS_Init();
    if (ret) {
        HAL_LOG_ERR("sys: XM_MPI_SYS_Init failed %d (0x%x)", ret, (unsigned)ret);
        ret = RSS_ERR_IO;
        goto err_free;
    }
    st->sys_inited = true;
    HAL_LOG_INFO("hal_init: MPI up to XM_MPI_SYS_Init");
#endif

    g_xm = st;
    c->initialized = true;
    return RSS_OK;

err_free:
    free(st);
    c->platform = NULL;
    return ret;
}

static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    xm_state_t *st = xm_state(ctx);

    if (!c)
        return RSS_ERR_INVAL;
    if (!st)
        return RSS_OK;

#ifdef HAL_MODULE_VIDEO
    xm_pipe_stop(st);
#endif
    if (g_xm == st)
        g_xm = NULL;

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
    xm_state_t *st = xm_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->sdk_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->sdk_version);
    return RSS_OK;
}

static int hal_sys_get_cpu_info(void *ctx, char *buf, int len)
{
    xm_state_t *st = xm_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st)
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->chip_name[0] ? st->chip_name : XM_SOC_MODEL);
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
    .fs_create_channel = hal_fs_create_channel,
    .fs_destroy_channel = hal_fs_destroy_channel,
    .fs_enable_channel = hal_fs_enable_channel,
    .fs_disable_channel = hal_fs_disable_channel,
    .fs_set_fifo = hal_fs_set_fifo,
    .fs_set_frame_depth = hal_fs_set_frame_depth,
    .fs_get_frame = hal_fs_get_frame,
    .fs_release_frame = hal_fs_release_frame,

    .isp_get_sensor_attr = hal_isp_get_sensor_attr,
    .isp_get_sensor_fps = hal_isp_get_sensor_fps,
    .isp_set_sensor_fps = hal_isp_set_sensor_fps,
    .isp_set_ae_comp = hal_isp_set_ae_comp,
    .isp_get_ae_comp = hal_isp_get_ae_comp,
    .isp_set_saturation = hal_isp_set_saturation,
    .isp_get_saturation = hal_isp_get_saturation,

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
    if (!g_xm || !g_xm->sdk_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)size, "%s", g_xm->sdk_version);
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

    if (g_xm && g_xm->chip_name[0])
        snprintf(cpu, sizeof(cpu), "%s", g_xm->chip_name);
    else if (xm_read_chip_name(cpu, sizeof(cpu)) != RSS_OK)
        snprintf(cpu, sizeof(cpu), "%s", XM_SOC_MODEL);

    return cpu;
}

const char *rss_hal_get_platform_name(void)
{
    return HAL_PLATFORM_NAME;
}

/*
 * rss_hal_check_platform -- report, never refuse, in Phase 1.
 *
 * What an XM530-family kernel puts in /proc/cpuinfo has not been read yet,
 * so there is nothing safe to compare against. Once the bench log shows it,
 * this becomes a real check like the Fullhan one.
 */
void rss_hal_check_platform(const char *name)
{
    char chip[24];

    (void)name;
    if (xm_read_chip_name(chip, sizeof(chip)) != RSS_OK) {
        HAL_LOG_WARN("platform check: /proc/cpuinfo has no Hardware line, assuming %s",
                     XM_SOC_MODEL);
        return;
    }

    HAL_LOG_INFO("platform check: built for %s (%s), kernel says '%s'", HAL_PLATFORM_NAME,
                 XM_SOC_MODEL, chip);
}
