/*
 * hisi_v5/hal_common.c -- Raptor HAL common layer, HiSilicon HiMPP V5.0
 *
 * Counterpart to src/hisi_v4/hal_common.c, and a separate translation unit
 * from it for the reason PLAN-hi3516cv610.md sets out at length: V5 renamed
 * every entry point, moved every layout, and split one libmpi.so into four
 * libraries. Sharing a file with gen4 would mean hiding an argument list
 * behind a macro at nearly every call site.
 *
 * Everything here guards on HAL_HISI_GEN5, never on PLATFORM_HI3516CV610.
 * The generation is the unit of compatibility and the *platform name is the
 * ABI's, not the die's*: OpenIPC builds one MPP for the whole hi3516cv6xx
 * family, and this file was brought up on a CV608 whose libraries report
 * "HI3516CV610_MPP_V1.0.2.0 B051 Release". The die reaches raptor
 * separately, as soc_model, and reaches this file as the chip name read
 * from /proc/umap/sys.
 *
 * Current state: Phase 1 skeleton. The vtable publishes only the ops that
 * are implemented; RSS_HAL_CALL() NULL-guards every entry and returns
 * RSS_ERR_NOTSUP for the rest, so unimplemented subsystems need no stubs.
 * A Phase 1 daemon comes up, logs the MPP version and the part, reaches
 * ss_mpi_sys_init, finds no framesource and exits cleanly. The video
 * pipeline lands in Phase 2.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <syslog.h>
#include <unistd.h>

/* ================================================================
 * LOGGING
 *
 * Mirrors src/hal_common.c, star/hal_common.c and the gen4 file: log
 * through a function pointer that defaults to stderr, which daemons
 * redirect to syslog at init.
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

/* ── Per-SoC capability data (src/hal_caps.c, compiled per platform) ── */

extern const rss_hal_caps_t g_hal_caps;

/*
 * The live backend state.
 *
 * The daemons call rss_hal_get_imp_version() and friends with no context
 * argument, and the ISP-cycle forwarders below are entered from a vendor
 * library that has no context to pass either. One HAL context per process
 * is already assumed throughout raptor; this is the pointer that makes that
 * assumption usable. Set at the end of hal_init, cleared in hal_deinit.
 */
static hisi_state_t *g_hisi;

/* ── GPIO / IR-cut (src/hal_gpio.c — plain sysfs, no SDK dependency) ── */

#ifdef HAL_MODULE_VIDEO
int hal_gpio_set(void *ctx, int pin, int value);
int hal_gpio_get(void *ctx, int pin, int *value);
int hal_ircut_set(void *ctx, int state);
#endif

/* ================================================================
 * FORWARDERS -- breaking the ISP / algorithm-library cycle
 *
 * This block is why raptor-hal's HiSilicon builds pass
 * -Wl,--export-dynamic unconditionally, and why it lives in *this* file
 * rather than in a quirks.c of its own.
 *
 * raptor-hal ships as libraptor_hal_video.a. A static-archive member is
 * extracted only if something already linked references a symbol in it, and
 * nothing in raptor references isp_alg_register_drc. A standalone quirks.c
 * would therefore compile, archive, and never link -- and --export-dynamic
 * cannot export what was never linked. -Wl,--gc-sections and -flto compound
 * it. hal_common.c is the translation unit that defines rss_hal_create, so
 * it is always extracted; putting the definitions here is what makes them
 * exist at all. __attribute__((used)) then keeps LTO and --gc-sections from
 * discarding them, since nothing in the program refers to them either.
 *
 * THE CYCLE, AND WHY musl MAKES IT THE EXECUTABLE'S PROBLEM.
 *
 * libot_mpi_isp.so and the eight algorithm libraries reference each other:
 *
 *   libot_mpi_isp.so  -> isp_alg_register_ldci        (in libldci.so)
 *                        isp_alg_register_drc         (in libdrc.so)
 *                        isp_alg_register_dehaze      (in libdehaze.so)
 *                        isp_alg_register_bayer_nr    (in libbnr.so)
 *                        isp_alg_register_acs         (in libacs.so)
 *                        isp_ir_auto_run_once         (in libir_auto.so)
 *                        isp_be_stats_estimate        (in libextend_stats.so)
 *                        calc_flicker_type            (in libcalcflicker.so)
 *   libldci.so        -> 18 symbols in libot_mpi_isp.so
 *   libdrc.so         -> 18      "
 *   libdehaze.so      -> 18      "
 *   libbnr.so         -> 32      "
 *   libacs.so         -> 18      "
 *   libextend_stats.so-> 4       "
 *   libcalcflicker.so -> 3       "
 *   libir_auto.so     -> 1       "
 *
 * (Measured with readelf on the board's own 1.0.2.0 B051 libraries. Those
 * eight are *exactly* libot_mpi_isp.so's undefined set once libc,
 * libsecurec and libss_mpi are accounted for -- there is no ninth waiting
 * to be discovered.)
 *
 * musl implements no lazy binding at all: RTLD_LAZY is accepted and
 * ignored, and every dlopen relocates fully. So neither side of the cycle
 * can be opened first, and something outside it has to satisfy one
 * direction. The vendor never meets this, because Makefile.param links
 * libot_mpi_isp.a and libdrc.a statically; a shipping product that dlopens
 * would meet it the same way raptor does.
 *
 * So the executable defines the eight, and the cycle unrolls:
 *
 *   1. dlopen(libot_mpi_isp.so) -- its eight imports bind to these
 *      forwarders, already in the global scope. It loads.
 *   2. dlopen(libldci.so) and the rest -- their imports resolve against
 *      libot_mpi_isp.so, now loaded and RTLD_GLOBAL. They load.
 *   3. hisi_isp_open (Phase 2) dlsyms the real eight out of step 2 and
 *      stores them in the state.
 *   4. ss_mpi_isp_init calls isp_alg_register_drc, reaches this forwarder,
 *      and is passed through to the real one.
 *
 * Step 4 is why the forwarders must stay valid for the process lifetime and
 * cannot become one-shot: libot_mpi_isp.so's GOT entry was bound once, at
 * its own dlopen, and points here for good.
 *
 * Each forwards through g_hisi rather than through a file-static pointer,
 * so the forwarders and the ISP loader cannot disagree about which library
 * is current. Before Phase 2 fills those pointers -- and after hal_deinit
 * clears them -- every one returns V5_FAILURE, and a vendor caller that
 * gets TD_FAILURE from a registrar declines to register, which is the
 * correct outcome when there is no algorithm library to register with.
 *
 * gen4's other three trampoline groups are absent here, and deliberately:
 * the uClibc ABI group (__ctype_b and friends) and the mmap shim both
 * existed because gen4's vendor libraries are uClibc builds. The V5 set is
 * musl-built and DT_NEEDEDs libc.so; the unresolved-symbol closure over all
 * 60 vendor libraries on the board leaves nothing of either shape. The
 * Goke group has no V5 counterpart at all. See v5_common.h.
 * ================================================================ */

#define HISI_FORWARD(field, ...)                                                                   \
    do {                                                                                           \
        hisi_state_t *st = g_hisi;                                                                 \
        if (!st || !st->field) {                                                                   \
            HAL_LOG_WARN("%s called with no ISP loaded", __func__);                                \
            return V5_FAILURE;                                                                     \
        }                                                                                          \
        return st->field(__VA_ARGS__);                                                             \
    } while (0)

__attribute__((used)) int isp_alg_register_ldci(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_ldci, vi_pipe);
}

__attribute__((used)) int isp_alg_register_drc(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_drc, vi_pipe);
}

__attribute__((used)) int isp_alg_register_dehaze(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_dehaze, vi_pipe);
}

__attribute__((used)) int isp_alg_register_bayer_nr(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_bayer_nr, vi_pipe);
}

__attribute__((used)) int isp_alg_register_acs(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_acs, vi_pipe);
}

/*
 * The three that are not registrars.
 *
 * Their second argument is a payload this backend never dereferences, so it
 * stays void * -- giving it a type would mean transcribing three more
 * structs to no end. The arity is what matters and it was read off the
 * libraries: each of the three touches r0 and r1 before its first call, and
 * none of the eight appears in any public header in the 1.0.2.0 set,
 * because the vendor links this boundary statically and never has to name
 * it.
 */
__attribute__((used)) int isp_ir_auto_run_once(int vi_pipe, void *arg)
{
    HISI_FORWARD(fn_ir_auto_run_once, vi_pipe, arg);
}

__attribute__((used)) int isp_be_stats_estimate(int vi_pipe, void *arg)
{
    HISI_FORWARD(fn_be_stats_estimate, vi_pipe, arg);
}

__attribute__((used)) int calc_flicker_type(int vi_pipe, void *arg)
{
    HISI_FORWARD(fn_calc_flicker_type, vi_pipe, arg);
}

/*
 * hisi_check_trampolines -- prove the executable really exports them.
 *
 * Runs before the first vendor dlopen, and not at ISP-open time, because by
 * then it is too late to be informative: musl relocates libot_mpi_isp.so
 * fully at its dlopen, so all eight have already either resolved or not
 * before any ISP code runs. A missing one shows up as
 * "Error relocating /usr/lib/libot_mpi_isp.so: isp_alg_register_drc: symbol
 * not found" -- which names the symbol but not the reason.
 *
 * dlsym(RTLD_DEFAULT, name) searching the global scope and finding *this*
 * object is what the vendor library will do. A mismatch means the link
 * dropped the definition -- no --export-dynamic, a static link, the archive
 * member not extracted -- and turns an unattributable relocation failure
 * into one log line naming the cause.
 *
 * All eight are checked rather than one canary. They are exported by the
 * same mechanism, so in practice they fail together; but the loop costs
 * eight dlsyms once per process, and "which one" is the first question
 * anybody debugging this would ask.
 *
 * Non-fatal by design: a build that gets this wrong should say so loudly
 * and then fail where the failure is, rather than refusing to start and
 * leaving no evidence.
 *
 * Not static: Phase 4's hal_audio.c is a second entry point that reaches
 * the first vendor dlopen without ever running hal_init -- rad calls
 * audio_init directly -- and it will need the same check for the same
 * reason.
 */
void hisi_check_trampolines(void);

void hisi_check_trampolines(void)
{
    static const struct {
        const char *name;
        void *fn;
    } tramps[] = {
        {"isp_alg_register_ldci", (void *)isp_alg_register_ldci},
        {"isp_alg_register_drc", (void *)isp_alg_register_drc},
        {"isp_alg_register_dehaze", (void *)isp_alg_register_dehaze},
        {"isp_alg_register_bayer_nr", (void *)isp_alg_register_bayer_nr},
        {"isp_alg_register_acs", (void *)isp_alg_register_acs},
        {"isp_ir_auto_run_once", (void *)isp_ir_auto_run_once},
        {"isp_be_stats_estimate", (void *)isp_be_stats_estimate},
        {"calc_flicker_type", (void *)calc_flicker_type},
    };
    unsigned i;
    int bad = 0;

    for (i = 0; i < sizeof(tramps) / sizeof(tramps[0]); i++) {
        void *found = dlsym(RTLD_DEFAULT, tramps[i].name);

        if (found == tramps[i].fn)
            continue;

        bad++;
        HAL_LOG_ERR("trampolines: %s resolves to %p, not %p", tramps[i].name, found, tramps[i].fn);
    }

    if (bad)
        HAL_LOG_ERR("trampolines: %d of %u not exported by the executable. libot_mpi_isp.so will "
                    "fail to relocate. Link with -Wl,--export-dynamic and keep hal_common.o in "
                    "the link.",
                    bad, (unsigned)(sizeof(tramps) / sizeof(tramps[0])));
    else
        HAL_LOG_DBG("trampolines: all 8 ISP-cycle symbols exported and resolvable");
}

/* ================================================================
 * CHIP IDENTIFICATION
 *
 * Two sources answering two questions; see hisi_state.h for which is which.
 * ================================================================ */

/*
 * hisi_read_chip_name -- the part, out of /proc/umap/sys, before any dlopen.
 *
 * The open_sys module prints a banner there as soon as it is modprobed:
 *
 *   [SYS] Version: [HI3516CV610_MPP_V1.0.2.0 B051 Release], Build Time[...]
 *
 *   ----------------------------------------0X3516C608--------------------
 *
 * so the part is the run of non-dash characters on the dashed line. Parsed
 * as text because that is what the driver offers: V5 publishes no register
 * accessor, and gen4's /dev/mem read of SCSYSID0 is a gen4 address with no
 * documented V5 counterpart.
 *
 * Bounded everywhere and tolerant of the file being absent -- which is the
 * ordinary case before load_hisilicon has run, not an error. Returns
 * RSS_ERR_NOENT then, and the caller decides what that means.
 */
static int hisi_read_chip_name(char *out, size_t out_size)
{
    char line[256];
    FILE *f;
    int found = 0;

    if (!out || out_size == 0)
        return RSS_ERR_INVAL;

    out[0] = '\0';

    f = fopen(HISI_UMAP_SYS_PATH, "r");
    if (!f)
        return RSS_ERR_NOENT;

    while (!found && fgets(line, sizeof(line), f)) {
        size_t start, end, len;

        /* The banner line is the one that begins with a dash run. The
         * version line above it begins with '[' and the module-param
         * heading below it also begins with dashes -- so the *first* such
         * line is the part and the loop stops there. */
        if (line[0] != '-')
            continue;

        for (start = 0; line[start] == '-'; start++)
            ;
        if (line[start] == '\0' || line[start] == '\n')
            continue; /* a bare rule, no token in it */

        for (end = start; line[end] && line[end] != '-' && line[end] != '\n'; end++)
            ;

        len = end - start;
        if (len == 0 || len >= out_size)
            break;

        memcpy(out, line + start, len);
        out[len] = '\0';
        found = 1;
    }

    fclose(f);
    return found ? RSS_OK : RSS_ERR_NOENT;
}

/*
 * hisi_chip_is_gen5 -- is this token a part this backend's ABI serves?
 *
 * The family prefix rather than an exact list, because the point of the
 * check is to catch a *generation* mismatch -- a gen4 or gen3 part running
 * a gen5 binary, where every symbol would resolve and every argument list
 * would be wrong. A cv6xx die nobody has held yet is a caps question, not
 * an ABI one, and gets a warning from rss_hal_check_platform.
 */
static bool hisi_chip_is_gen5(const char *name)
{
    return name && strncmp(name, "0X3516C6", 8) == 0;
}

#ifdef HAL_MODULE_VIDEO

/* ================================================================
 * THE ISP TIER
 *
 * Opened separately from the MPI set and later than it, because opening it
 * is what closes the dlopen cycle the forwarders above exist to break: the
 * algorithm code inside libot_mpi_isp.so resolves eight symbols that only
 * the executable defines, so the forwarder targets have to be filled in
 * from the same handles in the same call.
 *
 * Four libraries, and where each entry point lives is not what its name
 * suggests -- see v5_isp.h. libot_mpi_isp.so is the implementation,
 * libss_mpi_isp.so a four-byte-per-entry facade over it, and the two 3A
 * libraries carry ss_mpi_ae_register / ss_mpi_awb_register plus, in ae
 * rather than isp, ss_mpi_isp_query_exposure_info.
 * ================================================================ */

/*
 * The forwarder targets, resolved out of the ISP tier once it is open.
 *
 * Every one is optional. A library build that has none of them is a build
 * where the cycle does not exist, and the forwarders then return
 * V5_FAILURE to a caller that never calls them. A missing *subset* is the
 * interesting case and is logged, because it means the algorithm set on
 * this image is not the one the survey measured.
 */
/*
 * One row per forwarder: the library that defines it, the symbol, and
 * where in hisi_state_t the pointer goes.
 *
 * The pairing is one-to-one and was read off the images' export tables
 * rather than out of a header -- none of the eight appears in any public
 * header in the 1.0.2.0 set, because the vendor links these statically in
 * its own build and never has to name the boundary.
 */
static const struct {
    const char *lib;
    const char *name;
    size_t offset;
} hisi_isp_alg_table[HISI_ISP_ALG_LIB_NUM] = {
    {"libldci.so", "isp_alg_register_ldci", offsetof(hisi_state_t, fn_alg_register_ldci)},
    {"libdrc.so", "isp_alg_register_drc", offsetof(hisi_state_t, fn_alg_register_drc)},
    {"libdehaze.so", "isp_alg_register_dehaze", offsetof(hisi_state_t, fn_alg_register_dehaze)},
    {"libbnr.so", "isp_alg_register_bayer_nr", offsetof(hisi_state_t, fn_alg_register_bayer_nr)},
    {"libacs.so", "isp_alg_register_acs", offsetof(hisi_state_t, fn_alg_register_acs)},
    {"libir_auto.so", "isp_ir_auto_run_once", offsetof(hisi_state_t, fn_ir_auto_run_once)},
    {"libextend_stats.so", "isp_be_stats_estimate", offsetof(hisi_state_t, fn_be_stats_estimate)},
    {"libcalcflicker.so", "calc_flicker_type", offsetof(hisi_state_t, fn_calc_flicker_type)},
};

/*
 * hisi_isp_open_alg_libs -- open the eight and point the forwarders at them.
 *
 * THE CYCLE, CONCRETELY. libot_mpi_isp.so leaves these eight symbols
 * undefined; each of the eight libraries below defines exactly one of them
 * and calls back into libot_mpi_isp.so (18 back-references for ldci, 32 for
 * bnr, 1 for ir_auto -- measured). Neither side can be opened first with
 * RTLD_NOW unless something else already answers for the missing
 * direction, and on musl there is no lazy binding to fall back on. The
 * executable's forwarders are that something: libot_mpi_isp.so opens
 * against them, the eight then open against libot_mpi_isp.so, and the
 * forwarders are repointed here at the real implementations.
 *
 * Every one is optional. A missing algorithm library costs its feature --
 * the ISP asks for it once per pipeline and takes V5_FAILURE for an
 * answer -- and a missing *subset* is worth a line, because it means the
 * image's algorithm set is not the one this backend was measured against.
 */
static void hisi_isp_open_alg_libs(hisi_state_t *st)
{
    static const int flags = RTLD_NOW | RTLD_GLOBAL;
    size_t i;
    int found = 0;

    for (i = 0; i < HISI_ISP_ALG_LIB_NUM; i++) {
        void *fn = NULL;

        st->isp_alg[i] = dlopen(hisi_isp_alg_table[i].lib, flags);
        if (!st->isp_alg[i]) {
            HAL_LOG_WARN("isp: %s: %s", hisi_isp_alg_table[i].lib, dlerror());
        } else if (!(fn = dlsym(st->isp_alg[i], hisi_isp_alg_table[i].name))) {
            HAL_LOG_WARN("isp: %s has no %s", hisi_isp_alg_table[i].lib,
                         hisi_isp_alg_table[i].name);
        } else {
            found++;
        }

        /* Writing through a byte offset rather than eight assignments: the
         * eight have different prototypes and a switch on the name would
         * be the same table with more places to mistype it. */
        memcpy((char *)st + hisi_isp_alg_table[i].offset, &fn, sizeof(fn));
    }

    if (found != HISI_ISP_ALG_LIB_NUM)
        HAL_LOG_WARN("isp: %d of %d algorithm libraries bound -- the rest of the ISP runs and "
                     "those features do not",
                     found, HISI_ISP_ALG_LIB_NUM);
    else
        HAL_LOG_DBG("isp: all %d algorithm libraries bound", HISI_ISP_ALG_LIB_NUM);
}

static void hisi_isp_close_alg_libs(hisi_state_t *st)
{
    size_t i;

    for (i = 0; i < HISI_ISP_ALG_LIB_NUM; i++) {
        void *none = NULL;

        /* The forwarder goes first: from here on it must decline rather
         * than call into a mapping being dropped. */
        memcpy((char *)st + hisi_isp_alg_table[i].offset, &none, sizeof(none));
        if (st->isp_alg[i])
            dlclose(st->isp_alg[i]);
        st->isp_alg[i] = NULL;
    }
}

static int hisi_isp_open(hisi_state_t *st)
{
    static const int flags = RTLD_NOW | RTLD_GLOBAL;
    int ret;

    /*
     * The implementation first, then the facade. Order matters only for
     * the log: opening libss_mpi_isp.so pulls nothing in by itself -- its
     * only DT_NEEDED is libc.so -- so a missing libot_mpi_isp.so would
     * otherwise surface as an unresolved symbol at the first ss_mpi_isp_*
     * call rather than here.
     */
    if (!(st->libs.isp_impl = dlopen("libot_mpi_isp.so", flags))) {
        HAL_LOG_ERR("hisi_isp: libot_mpi_isp.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }
    if (!(st->libs.isp = dlopen("libss_mpi_isp.so", flags))) {
        HAL_LOG_ERR("hisi_isp: libss_mpi_isp.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }
    if (!(st->libs.ae = dlopen("libss_mpi_ae.so", flags))) {
        HAL_LOG_ERR("hisi_isp: libss_mpi_ae.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }
    if (!(st->libs.awb = dlopen("libss_mpi_awb.so", flags))) {
        HAL_LOG_ERR("hisi_isp: libss_mpi_awb.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    v5_libs_add_search(&st->libs, st->libs.isp);
    v5_libs_add_search(&st->libs, st->libs.ae);
    v5_libs_add_search(&st->libs, st->libs.awb);
    v5_libs_add_search(&st->libs, st->libs.isp_impl);

    /* After libot_mpi_isp.so and not before: the eight need it. */
    hisi_isp_open_alg_libs(st);

    if ((ret = v5_isp_load(&st->isp, &st->libs)) != RSS_OK)
        return ret;

    HAL_LOG_DBG("hisi_isp: libot_mpi_isp.so + facade + ae + awb loaded");
    return RSS_OK;
}

/* ================================================================
 * MIPI
 * ================================================================ */

/*
 * The receiver is a character device, not an MPI module: /dev/ot_mipi_rx,
 * driven entirely by ioctl. The older spelling /dev/mipi_rx is tried too,
 * because OpenIPC's own module has shipped under both names.
 */
static int hisi_mipi_open(void)
{
    int fd = open(V5_MIPI_DEV_NAME, O_RDWR);

    if (fd >= 0)
        return fd;
    return open(V5_MIPI_DEV_NAME_ALT, O_RDWR);
}

static int hisi_mipi_ioctl(int fd, unsigned long req, void *arg, const char *what)
{
    if (ioctl(fd, req, arg) < 0) {
        HAL_LOG_ERR("mipi: %s failed: %s", what, strerror(errno));
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

/*
 * hisi_mipi_configure -- the receiver, in the vendor's order.
 *
 * The order is SAMPLE_COMM_VI_StartMIPI's and every step of it is
 * load-bearing. The one worth calling out is ENABLE_SENSOR_CLOCK: **until
 * it runs the sensor has no MCLK and does not answer on I2C at all**. That
 * is why a bench i2cdetect finds nothing on a board whose sensor is
 * perfectly well wired, and why "the sensor is missing" is not a diagnosis
 * that can be made before this function has run.
 *
 * lane_divide_mode is set through SET_HS_MODE before anything else,
 * because it decides whether the phy is one four-lane receiver or two
 * two-lane ones, and the device attribute that follows is interpreted
 * under it.
 *
 * devno is the receiver, and hisi_vi_bringup opens the VI device of the
 * same number: a sensor on the second receiver ([mipi] dev = 1) reaches VI
 * device 1 and nothing else. The sensor clock and reset stay on source 0.
 */
static int hisi_mipi_configure(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v5_combo_dev_attr attr;
    unsigned int devno = m->mipi_dev;
    unsigned int sns_src = 0;
    v5_lane_divide_mode hs_mode = m->lane_divide_mode;
    int fd;
    int ret;

    fd = hisi_mipi_open();
    if (fd < 0) {
        HAL_LOG_ERR("mipi: %s: %s", V5_MIPI_DEV_NAME, strerror(errno));
        return RSS_ERR_NOENT;
    }

    memset(&attr, 0, sizeof(attr));
    attr.devno = devno;
    attr.input_mode = m->input_mode;
    attr.data_rate = m->mipi_data_rate;

    /* (0,0), not the mode's DevRect_x/y: this window is a crop out of what
     * the sensor actually sends, and it sends exactly DevRect_w by
     * DevRect_h. See hisi_sensor_mode_load. */
    attr.img_rect.x = 0;
    attr.img_rect.y = 0;
    attr.img_rect.width = m->dev_rect.width;
    attr.img_rect.height = m->dev_rect.height;

    attr.mipi_attr.input_data_type = m->mipi_data_type;
    attr.mipi_attr.wdr_mode = V5_MIPI_WDR_MODE_NONE;
    memcpy(attr.mipi_attr.lane_id, m->lane_id, sizeof(attr.mipi_attr.lane_id));

    ret = hisi_mipi_ioctl(fd, V5_MIPI_SET_HS_MODE, &hs_mode, "SET_HS_MODE");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V5_MIPI_ENABLE_MIPI_CLOCK, &devno, "ENABLE_MIPI_CLOCK");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V5_MIPI_RESET_MIPI, &devno, "RESET_MIPI");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V5_MIPI_ENABLE_SENSOR_CLOCK, &sns_src, "ENABLE_SENSOR_CLOCK");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V5_MIPI_RESET_SENSOR, &sns_src, "RESET_SENSOR");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V5_MIPI_SET_DEV_ATTR, &attr, "SET_DEV_ATTR");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V5_MIPI_UNRESET_MIPI, &devno, "UNRESET_MIPI");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V5_MIPI_UNRESET_SENSOR, &sns_src, "UNRESET_SENSOR");
    if (ret)
        goto out;

    st->mipi_configured = true;
    HAL_LOG_INFO("mipi: dev %u, RAW%d, %ux%u, lanes %d|%d|%d|%d, divide %d", devno, m->raw_bitness,
                 m->dev_rect.width, m->dev_rect.height, m->lane_id[0], m->lane_id[1], m->lane_id[2],
                 m->lane_id[3], (int)m->lane_divide_mode);

out:
    close(fd);
    return ret;
}

/* Reverse of the above: reset the sensor, stop its clock, reset the
 * receiver, stop its clock. Failures are logged and never propagated --
 * this runs during teardown, where stopping halfway is worse than any
 * individual failure. */
static void hisi_mipi_shutdown(hisi_state_t *st)
{
    unsigned int devno = st->mode.mipi_dev;
    unsigned int sns_src = 0;
    int fd;

    if (!st->mipi_configured)
        return;

    fd = hisi_mipi_open();
    if (fd < 0) {
        HAL_LOG_WARN("mipi: %s on shutdown: %s", V5_MIPI_DEV_NAME, strerror(errno));
        st->mipi_configured = false;
        return;
    }

    hisi_mipi_ioctl(fd, V5_MIPI_RESET_SENSOR, &sns_src, "RESET_SENSOR");
    hisi_mipi_ioctl(fd, V5_MIPI_DISABLE_SENSOR_CLOCK, &sns_src, "DISABLE_SENSOR_CLOCK");
    hisi_mipi_ioctl(fd, V5_MIPI_RESET_MIPI, &devno, "RESET_MIPI");
    hisi_mipi_ioctl(fd, V5_MIPI_DISABLE_MIPI_CLOCK, &devno, "DISABLE_MIPI_CLOCK");

    close(fd);
    st->mipi_configured = false;
}

/* ================================================================
 * SENSOR
 * ================================================================ */

/*
 * hisi_sensor_bringup -- open the driver and hand it to the ISP.
 *
 * The order is the vendor's and it is not interchangeable:
 * pfn_set_bus_info before pfn_register_callback, because the library talks
 * I2C during registration and a library that does not yet know its bus
 * writes to adapter 0.
 *
 * Note what is *not* called. pfn_mirror_flip is null on every sensor
 * library that ships with this image -- checked by resolving the objects'
 * relocations, see v5_snr.h -- so orientation is the VPSS channels', which
 * is where raptor wants it anyway. pfn_set_fast_ae is not called either,
 * and there the reason is sharper: three of the nine libraries export a
 * 44-byte object with no such member, and calling it would read four bytes
 * past the end of the mapping.
 */
static int hisi_sensor_bringup(hisi_state_t *st, const rss_sensor_config_t *cfg)
{
    hisi_sensor_mode_t *m = &st->mode;
    v5_isp_sns_commbus bus;
    char path[288];
    int i2c;
    int ret;

    if (m->dll_file[0] == '/')
        snprintf(path, sizeof(path), "%s", m->dll_file);
    else
        snprintf(path, sizeof(path), "%s/%s", V5_SNS_LIB_DIR, m->dll_file);

    st->snr_handle = dlopen(path, RTLD_NOW | RTLD_GLOBAL);
    if (!st->snr_handle) {
        /* Bare name too: a library already in the loader's search path
         * opens under it, and a board that keeps its sensors elsewhere is
         * then not a board this backend refuses. */
        st->snr_handle = dlopen(m->dll_file, RTLD_NOW | RTLD_GLOBAL);
    }
    if (!st->snr_handle) {
        HAL_LOG_ERR("sensor: %s: %s", path, dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(st->snr_obj = hisi_sensor_obj_find(m, st->snr_handle)))
        return RSS_ERR_NOTSUP;

    if (!st->snr_obj->pfn_set_bus_info || !st->snr_obj->pfn_register_callback) {
        HAL_LOG_ERR("sensor: %s's object has no %s", m->obj_name,
                    st->snr_obj->pfn_set_bus_info ? "pfn_register_callback" : "pfn_set_bus_info");
        return RSS_ERR_NOTSUP;
    }

    /*
     * The I2C adapter, passed **by value** in one byte -- a struct that
     * small goes in a register, so the type has to be exactly right or the
     * calling convention changes rather than the contents.
     *
     * raptor's config wins over the mode file, because the bus is a
     * property of the board and the config is the board's file.
     */
    i2c = (cfg && cfg->i2c_adapter >= 0) ? cfg->i2c_adapter : m->i2c_dev;
    memset(&bus, 0, sizeof(bus));
    bus.i2c_dev = (signed char)i2c;
    ret = st->snr_obj->pfn_set_bus_info(HISI_VI_PIPE, bus);
    if (ret) {
        HAL_LOG_ERR("sensor: pfn_set_bus_info(pipe %d, i2c %d) failed: 0x%x", HISI_VI_PIPE, i2c,
                    ret);
        return RSS_ERR_IO;
    }

    /*
     * The 3A descriptors, and **the caller fills in the names**.
     *
     * This is the one place a gen4 habit is actively wrong. On gen4 the
     * sensor library filled the pair in and the caller passed them on; on
     * V5 the library *validates* them -- cis_register_callback calls
     * ae_check_lib_name, which compares against what the AE library
     * registered under -- and an empty name fails with 0xa01c8007 and
     * "Illegal lib name !" on stderr.
     *
     * The names are the vendor's constants: OT_AE_LIB_NAME
     * (ot_common_ae.h:18) and OT_AWB_LIB_NAME (ot_common_awb.h:18), and
     * both appear verbatim in the shipped libss_mpi_ae.so and
     * libss_mpi_awb.so, which is the check that matters -- the header is
     * a claim and the string table is the fact.
     *
     * id is the VI pipe, which is also the ISP index.
     */
    memset(&st->ae_lib, 0, sizeof(st->ae_lib));
    memset(&st->awb_lib, 0, sizeof(st->awb_lib));
    st->ae_lib.id = HISI_VI_PIPE;
    st->awb_lib.id = HISI_VI_PIPE;
    snprintf(st->ae_lib.lib_name, sizeof(st->ae_lib.lib_name), "%s", V5_AE_LIB_NAME);
    snprintf(st->awb_lib.lib_name, sizeof(st->awb_lib.lib_name), "%s", V5_AWB_LIB_NAME);

    ret = st->snr_obj->pfn_register_callback(HISI_VI_PIPE, &st->ae_lib, &st->awb_lib);
    if (ret) {
        HAL_LOG_ERR("sensor: pfn_register_callback(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->sensor_registered = true;

    ret = st->isp.fnAeRegister(HISI_VI_PIPE, &st->ae_lib);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_ae_register(pipe %d, \"%s\") failed: 0x%x -- the ISP registers "
                    "algorithms by name, so a mismatch here is a naming mismatch",
                    HISI_VI_PIPE, st->ae_lib.lib_name, ret);
        return RSS_ERR_IO;
    }
    st->ae_registered = true;

    ret = st->isp.fnAwbRegister(HISI_VI_PIPE, &st->awb_lib);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_awb_register(pipe %d, \"%s\") failed: 0x%x", HISI_VI_PIPE,
                    st->awb_lib.lib_name, ret);
        return RSS_ERR_IO;
    }
    st->awb_registered = true;

    HAL_LOG_INFO("sensor: %s registered on i2c %d, ae \"%s\", awb \"%s\"", m->obj_name, i2c,
                 st->ae_lib.lib_name, st->awb_lib.lib_name);
    return RSS_OK;
}

static void hisi_sensor_teardown(hisi_state_t *st)
{
    if (st->awb_registered && st->isp.fnAwbUnRegister)
        st->isp.fnAwbUnRegister(HISI_VI_PIPE, &st->awb_lib);
    st->awb_registered = false;

    if (st->ae_registered && st->isp.fnAeUnRegister)
        st->isp.fnAeUnRegister(HISI_VI_PIPE, &st->ae_lib);
    st->ae_registered = false;

    if (st->sensor_registered && st->snr_obj && st->snr_obj->pfn_un_register_callback)
        st->snr_obj->pfn_un_register_callback(HISI_VI_PIPE, &st->ae_lib, &st->awb_lib);
    st->sensor_registered = false;

    /* The object points into the mapping, so it dies with the handle. */
    st->snr_obj = NULL;
    if (st->snr_handle)
        dlclose(st->snr_handle);
    st->snr_handle = NULL;
}

/* ================================================================
 * ISP
 * ================================================================ */

/*
 * The 3A loop. ss_mpi_isp_run does not return while the ISP is up, so it
 * gets a thread of its own, and teardown stops it by calling
 * ss_mpi_isp_exit -- which is what makes run return -- rather than by
 * cancelling it. A thread cancelled inside the vendor library leaves its
 * locks held, and the next ss_mpi_isp_init in the same process then blocks
 * forever.
 */
static void *hisi_isp_thread(void *arg)
{
    hisi_state_t *st = (hisi_state_t *)arg;
    int ret;

    ret = st->isp.fnRun(HISI_VI_PIPE);

    if (__atomic_load_n(&st->isp_thread_running, __ATOMIC_ACQUIRE))
        HAL_LOG_ERR("ss_mpi_isp_run(pipe %d) returned 0x%x -- the ISP has stopped", HISI_VI_PIPE,
                    ret);

    __atomic_store_n(&st->isp_thread_done, 1, __ATOMIC_RELEASE);
    return NULL;
}

/*
 * Join the 3A thread, but not forever.
 *
 * ss_mpi_isp_exit is what makes ss_mpi_isp_run return, so the thread should
 * be gone within a frame or two. A plain pthread_join would be correct
 * whenever that holds -- and would hang rvd's shutdown permanently on any
 * board or library build where it does not, a failure mode raptor has
 * already paid for on another vendor. Cancelling is not the alternative:
 * it trades a hang at shutdown for a hang at the next start.
 */
#define HISI_ISP_JOIN_TIMEOUT_MS 2000
#define HISI_ISP_JOIN_POLL_MS 10

static bool hisi_isp_thread_stop(hisi_state_t *st)
{
    int waited;

    for (waited = 0; waited < HISI_ISP_JOIN_TIMEOUT_MS; waited += HISI_ISP_JOIN_POLL_MS) {
        struct timeval tv;

        if (__atomic_load_n(&st->isp_thread_done, __ATOMIC_ACQUIRE)) {
            pthread_join(st->isp_thread, NULL);
            return true;
        }

        /* select with no descriptors is the sleep this file can reach:
         * nanosleep and usleep are both behind POSIX feature macros under
         * -std=c11. */
        tv.tv_sec = 0;
        tv.tv_usec = HISI_ISP_JOIN_POLL_MS * 1000;
        select(0, NULL, NULL, NULL, &tv);
    }

    HAL_LOG_ERR("isp: ss_mpi_isp_run did not return %d ms after ss_mpi_isp_exit; detaching the "
                "3A thread rather than blocking shutdown",
                HISI_ISP_JOIN_TIMEOUT_MS);
    pthread_detach(st->isp_thread);
    return false;
}

static int hisi_isp_bringup(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v5_isp_pub_attr pub;
    int ret;

    ret = st->isp.fnMemInit(HISI_VI_PIPE);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_mem_init(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }

    memset(&pub, 0, sizeof(pub));
    pub.wnd_rect.x = 0;
    pub.wnd_rect.y = 0;
    pub.wnd_rect.width = m->dev_rect.width;
    pub.wnd_rect.height = m->dev_rect.height;
    pub.sns_size.width = m->dev_rect.width;
    pub.sns_size.height = m->dev_rect.height;
    pub.frame_rate = m->frame_rate;
    pub.bayer_format = m->bayer;
    pub.wdr_mode = V5_WDR_MODE_NONE;
    pub.sns_mode = m->sns_mode;
    /* The phase and the OB hint follow the sensor's turn, if it is the
     * sensor turning the picture; otherwise the VPSS channels are, and the
     * ISP sees the readout as the mode file describes it. */
    pub.sns_flip_en = 0;
    pub.sns_mirror_en = 0;
    hisi_snr_orien_pub(st, &pub);

    ret = st->isp.fnSetPubAttr(HISI_VI_PIPE, &pub);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_set_pub_attr(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }

    ret = st->isp.fnInit(HISI_VI_PIPE);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_init(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->isp_inited = true;

    /* The flag goes up before the thread starts, so the thread's own exit
     * path can never observe it unset while the ISP is genuinely running. */
    __atomic_store_n(&st->isp_thread_running, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&st->isp_thread_done, 0, __ATOMIC_RELEASE);
    ret = pthread_create(&st->isp_thread, NULL, hisi_isp_thread, st);
    if (ret) {
        __atomic_store_n(&st->isp_thread_running, 0, __ATOMIC_RELEASE);
        HAL_LOG_ERR("isp: cannot start the 3A thread: %s", strerror(ret));
        return RSS_ERR_IO;
    }

    /*
     * An orientation remembered before this point could not be written to
     * the sensor, which ss_mpi_isp_init has only now programmed. The
     * public attribute above already carries its phase, so this is the
     * register write alone; an [image] applied later comes through
     * hal_isp_set_hflip / set_vflip on its own.
     */
    if ((st->mirror || st->flip) && hisi_snr_orien_at_sensor(st))
        hisi_snr_orien_apply(st);
    st->isp_thread_started = true;

    HAL_LOG_INFO("isp: pipe %d running, %ux%u @ %.2f fps, bayer %d", HISI_VI_PIPE,
                 m->dev_rect.width, m->dev_rect.height, (double)m->frame_rate, (int)m->bayer);
    return RSS_OK;
}

static void hisi_isp_teardown(hisi_state_t *st)
{
    if (st->isp_thread_started)
        __atomic_store_n(&st->isp_thread_running, 0, __ATOMIC_RELEASE);

    /*
     * Unconditional, not gated on isp_inited: ss_mpi_isp_mem_init has
     * already taken the virtual registers by the time init can fail, and
     * only exit gives them back. Exiting an ISP that was never inited
     * returns an error and does nothing, which is the cheap half of the
     * trade.
     */
    if (st->isp.fnExit)
        st->isp.fnExit(HISI_VI_PIPE);
    st->isp_inited = false;

    if (st->isp_thread_started)
        hisi_isp_thread_stop(st);
    st->isp_thread_started = false;
}

/* ================================================================
 * VI -- DEVICE, PIPE, CHANNEL
 * ================================================================ */

/*
 * hisi_vi_vpss_mode -- read-modify-write the VI/VPSS coupling.
 *
 * Runs after ss_mpi_sys_init and before any pipe exists; the call site in
 * hisi_sys_bringup says why that order and not the other one.
 *
 * The two halves are independent and are chosen for different reasons.
 *
 * VI ONLINE is a memory decision. An offline pipe writes the raw Bayer
 * frame to DDR and reads it back into the ISP; an online one hands the
 * ISP its pixels on chip. That is one whole sensor-sized VB block per
 * frame in flight -- measured at 3,732,480 bytes of a 32 MB MMZ on the
 * CV608 -- bought for nothing but the loss of raw dump and raw send,
 * neither of which this backend offers. The tuning guide's own table
 * ("性能╱带宽╱延时╱内存调优指南", 各模块内存相关可优化配置 / VI) puts the
 * saving at 1-2 VB for the online couplings against full offline.
 *
 * VPSS ONLINE is the other half, and Phase 7 measured it rather than
 * inheriting gen4's caution. An online VPSS is fed in hardware, so no
 * sensor-sized frame lands in DDR between the two -- which is the whole
 * of the common pool the offline path needs, three blocks of 4,478,976 B,
 * 13,122 KiB of a 32 MiB MMZ. It is also the vendor's own low-memory
 * configuration: sample_venc.c's sample_venc_online_wrap_get_default_sys_cfg
 * sets OT_VI_ONLINE_VPSS_ONLINE together with vpss_wrap_en, and the VB
 * configuration beside it starts at pool 1 -- there is no sensor-sized
 * pool in it at all. What it costs is the VI channel's own frames, which
 * this backend never reads, and a group restricted to one pipe, which is
 * all raptor drives.
 *
 * st->vi_all_online carries the intent in and the answer out. The pool
 * configuration has to be fixed before SYS starts and this call is the
 * first point at which the driver can refuse, so hal_init asks for the
 * pair, reads back what it got, and rebuilds VB in the shape that matches
 * if the coupling did not take.
 *
 * Asking explicitly matters either way: the board does not necessarily
 * boot in any particular coupling.
 *
 * Read-modify-write rather than build-and-set, following the vendor: the
 * array has an entry per pipe and writing a fresh one would reset the mode
 * of a pipe this backend does not drive. **V5's array is four wide, not
 * gen4's two**: two physical pipes plus two virtual ones.
 *
 * Failure is a warning, not an error. The driver has a default, and
 * refusing to start a pipeline because the mode could not be confirmed
 * would trade a working camera for a tidier log.
 */
static void hisi_vi_vpss_mode(hisi_state_t *st)
{
    v5_vi_vpss_mode_type want =
        st->vi_all_online ? V5_VI_ONLINE_VPSS_ONLINE : V5_VI_ONLINE_VPSS_OFFLINE;
    v5_vi_vpss_mode mode;
    int ret;
    {
        /* local experiment: force the VI/VPSS coupling, 0 = both offline .. 3 = both online */
        const char *ov = getenv("RSS_VI_COUPLING");
        if (ov && ov[0] >= '0' && ov[0] <= '3' && ov[1] == '\0')
            want = (v5_vi_vpss_mode_type)(ov[0] - '0');
    }

    if (!st->sys.fnGetViVpssMode || !st->sys.fnSetViVpssMode) {
        HAL_LOG_DBG("vi: no ss_mpi_sys_get/set_vi_vpss_mode; leaving the coupling alone");
        st->vi_all_online = false;
        return;
    }

    memset(&mode, 0, sizeof(mode));
    if ((ret = st->sys.fnGetViVpssMode(&mode)) != 0) {
        HAL_LOG_WARN("ss_mpi_sys_get_vi_vpss_mode failed: 0x%x", ret);
        st->vi_all_online = false;
        return;
    }

    HAL_LOG_DBG("vi: coupling on entry %d|%d|%d|%d", (int)mode.mode[0], (int)mode.mode[1],
                (int)mode.mode[2], (int)mode.mode[3]);

    if (mode.mode[HISI_VI_PIPE] != want) {
        mode.mode[HISI_VI_PIPE] = want;

        if ((ret = st->sys.fnSetViVpssMode(&mode)) != 0)
            HAL_LOG_WARN("ss_mpi_sys_set_vi_vpss_mode(pipe %d = %d) failed: 0x%x -- continuing "
                         "on the board's default coupling",
                         HISI_VI_PIPE, (int)want, ret);
    }

    /*
     * Read back rather than assume, and log it at INFO rather than DBG.
     * The driver may clamp, and the value in force decides both whether
     * VI -> VPSS is a software bind at all and whether the VI pipe puts a
     * raw frame in a VB block -- which is to say it decides whether the
     * pool arithmetic in hisi_vb_fill_cfg was right. It is the first
     * thing to read when VI starts failing on vb_fail again.
     */
    if (st->sys.fnGetViVpssMode(&mode) == 0)
        st->vi_vpss_mode = mode;

    st->vi_all_online = st->vi_vpss_mode.mode[HISI_VI_PIPE] == V5_VI_ONLINE_VPSS_ONLINE;

    HAL_LOG_INFO("vi: pipe %d coupling %d (2 = VI online, VPSS offline; 3 = both online)",
                 HISI_VI_PIPE, (int)st->vi_vpss_mode.mode[HISI_VI_PIPE]);
}

/*
 * hisi_vi_enable_3dnr -- the pipe's temporal noise reduction, after the
 * channel is up (the sample's order, sample_comm_vi_start_chn).
 *
 * The pipe comes up ready for it -- nr_type NORM, compress FRAME, motion
 * NORM, enable 0 -- and the enable is what allocates the reference
 * frames, in MMZ rather than VB. On a 32 MB MMZ that allocation and
 * channel 0's 6.2 MB frame pool did not both fit (OT_ERR_NO_MEM, measured
 * 2026-09-08), which is why this is tied to the wrap ring: with channel 0
 * streaming from a 0.4 MB ring the room is there. The [static_3dnr]
 * ladder hal_nrx.c writes is what these frames get filtered with.
 *
 * Never fatal. A pipe without 3DNR is the pipe this backend shipped with
 * until today.
 */
static void hisi_vi_enable_3dnr(hisi_state_t *st)
{
    v5_3dnr_attr attr;
    int ret;

    if (!st->vb_wrap_blk || !st->vi.fnGetPipe3dnrAttr || !st->vi.fnSetPipe3dnrAttr)
        return;

    memset(&attr, 0, sizeof(attr));
    ret = st->vi.fnGetPipe3dnrAttr(HISI_VI_PIPE, &attr);
    if (ret) {
        HAL_LOG_WARN("ss_mpi_vi_get_pipe_3dnr_attr(%d) failed: 0x%x; 3DNR stays off", HISI_VI_PIPE,
                     ret);
        return;
    }
    if (attr.enable) {
        st->vi_3dnr_enabled = true;
        return;
    }
    attr.enable = 1;
    attr.nr_type = V5_NR_TYPE_VIDEO_NORM;
    attr.compress_mode = V5_COMPRESS_MODE_FRAME;
    attr.nr_motion_mode = V5_NR_MOTION_MODE_NORM;
    ret = st->vi.fnSetPipe3dnrAttr(HISI_VI_PIPE, &attr);
    if (ret) {
        HAL_LOG_WARN("ss_mpi_vi_set_pipe_3dnr_attr(%d, enable) failed: 0x%x%s; 3DNR stays off",
                     HISI_VI_PIPE, ret,
                     V5_ERR_ID(ret) == V5_ERR_NO_MEM ? " (NO_MEM: no room for the reference frames)"
                                                     : "");
        return;
    }
    st->vi_3dnr_enabled = true;
    HAL_LOG_INFO("vi: pipe %d 3DNR on (NORM, reference frames FRAME-compressed)", HISI_VI_PIPE);
}

/*
 * hisi_vi_bringup -- device, then bind, then pipe, then channel.
 *
 * **The bind is new against gen4 and it is the whole trap.** gen4 had
 * VI_PIPE == VI_DEV by construction; V5 requires ss_mpi_vi_bind(dev, pipe)
 * between enable_dev and create_pipe. Omit it and create_pipe succeeds, the
 * pipe never receives a frame, and /proc/umap/vi shows an empty "vi bind
 * attr" table with no error anywhere.
 */
static int hisi_vi_bringup(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v5_vi_dev_attr dev;
    v5_vi_pipe_attr pipe;
    v5_vi_chn_attr chn;
    int ret;

    memset(&dev, 0, sizeof(dev));
    dev.intf_mode = m->intf_mode;
    dev.work_mode = m->work_mode;
    dev.component_mask[0] = m->component_mask[0];
    dev.component_mask[1] = m->component_mask[1];
    dev.scan_mode = m->scan_mode;
    /* -1 on every entry: ad_chn_id names an analogue decoder channel and
     * there is none. memset would leave 0, which is a real channel. */
    dev.ad_chn_id[0] = -1;
    dev.ad_chn_id[1] = -1;
    dev.ad_chn_id[2] = -1;
    dev.ad_chn_id[3] = -1;
    dev.data_seq = m->data_seq;
    dev.sync_cfg = m->sync_cfg;
    dev.data_type = m->data_type;
    dev.data_reverse = m->data_reverse;
    dev.in_size.width = m->dev_rect.width;
    dev.in_size.height = m->dev_rect.height;
    dev.data_rate = m->data_rate;

    ret = st->vi.fnSetDevAttr(hisi_vi_dev(st), &dev);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vi_set_dev_attr(dev %d) failed: 0x%x", hisi_vi_dev(st), ret);
        return RSS_ERR_IO;
    }

    ret = st->vi.fnEnableDev(hisi_vi_dev(st));
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vi_enable_dev(dev %d) failed: 0x%x", hisi_vi_dev(st), ret);
        return RSS_ERR_IO;
    }
    st->vi_dev_enabled = true;

    ret = st->vi.fnBind(hisi_vi_dev(st), HISI_VI_PIPE);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vi_bind(dev %d, pipe %d) failed: 0x%x", hisi_vi_dev(st), HISI_VI_PIPE,
                    ret);
        return RSS_ERR_IO;
    }
    st->vi_bound = true;

    /*
     * The pipe. isp_bypass false and bypass_mode NONE: the ISP is the
     * point of the pipe, and the bypass modes exist for a YUV sensor that
     * needs none of it.
     *
     * pixel_format is the *pipe's* output to DDR, which in an offline
     * pipeline is the raw the ISP will read back, so it follows the
     * sensor's bit depth rather than the YUV the channel produces.
     */
    memset(&pipe, 0, sizeof(pipe));
    pipe.pipe_bypass_mode = V5_VI_PIPE_BYPASS_NONE;
    pipe.isp_bypass = 0;
    pipe.size.width = m->dev_rect.width;
    pipe.size.height = m->dev_rect.height;
    pipe.pixel_format = m->pixel_format;
    pipe.compress_mode = V5_COMPRESS_MODE_NONE;
    pipe.frame_rate_ctrl.src_frame_rate = -1;
    pipe.frame_rate_ctrl.dst_frame_rate = -1;

    ret = st->vi.fnCreatePipe(HISI_VI_PIPE, &pipe);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vi_create_pipe(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->vi_pipe_created = true;

    ret = st->vi.fnStartPipe(HISI_VI_PIPE);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vi_start_pipe(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->vi_pipe_started = true;

    /*
     * The channel, which is where YUV comes out.
     *
     * YVU_SEMIPLANAR_420 is NV21 -- V plane first -- and is what VPSS and
     * VENC expect throughout this backend. The neighbouring
     * YUV_SEMIPLANAR_420 (NV12) differs only in chroma order, so choosing
     * wrong costs swapped colours rather than an error.
     *
     * depth 0: nothing reads frames off the VI channel by hand. Every
     * nonzero value costs that many frames of VB for a queue no consumer
     * drains.
     */
    memset(&chn, 0, sizeof(chn));
    chn.size.width = m->dev_rect.width;
    chn.size.height = m->dev_rect.height;
    chn.pixel_format = V5_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    chn.dynamic_range = V5_DYNAMIC_RANGE_SDR8;
    chn.video_format = V5_VIDEO_FORMAT_LINEAR;
    chn.compress_mode = V5_COMPRESS_MODE_NONE;
    /*
     * Not [image]'s orientation. The VI channel takes mirror_en and
     * reports it in /proc/umap/vi, and in the all-online coupling it turns
     * nothing -- there is no VI write-out to reverse. Orientation is the
     * sensor's or the VPSS channels'; see hal_knob.c.
     */
    chn.mirror_en = 0;
    chn.flip_en = 0;
    chn.depth = 0;
    chn.frame_rate_ctrl.src_frame_rate = -1;
    chn.frame_rate_ctrl.dst_frame_rate = -1;

    ret = st->vi.fnSetChnAttr(HISI_VI_PIPE, HISI_VI_CHN, &chn);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vi_set_chn_attr(pipe %d, chn %d) failed: 0x%x", HISI_VI_PIPE,
                    HISI_VI_CHN, ret);
        return RSS_ERR_IO;
    }

    ret = st->vi.fnEnableChn(HISI_VI_PIPE, HISI_VI_CHN);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vi_enable_chn(pipe %d, chn %d) failed: 0x%x", HISI_VI_PIPE, HISI_VI_CHN,
                    ret);
        return RSS_ERR_IO;
    }
    st->vi_chn_enabled = true;

    HAL_LOG_INFO("vi: dev %d -> pipe %d -> chn %d, %ux%u", hisi_vi_dev(st), HISI_VI_PIPE,
                 HISI_VI_CHN, m->dev_rect.width, m->dev_rect.height);

    hisi_vi_enable_3dnr(st);
    return RSS_OK;
}

static void hisi_vi_teardown(hisi_state_t *st)
{
    if (st->vi_chn_enabled)
        st->vi.fnDisableChn(HISI_VI_PIPE, HISI_VI_CHN);
    st->vi_chn_enabled = false;

    if (st->vi_pipe_started)
        st->vi.fnStopPipe(HISI_VI_PIPE);
    st->vi_pipe_started = false;

    if (st->vi_pipe_created)
        st->vi.fnDestroyPipe(HISI_VI_PIPE);
    st->vi_pipe_created = false;

    if (st->vi_bound)
        st->vi.fnUnbind(hisi_vi_dev(st), HISI_VI_PIPE);
    st->vi_bound = false;

    if (st->vi_dev_enabled)
        st->vi.fnDisableDev(hisi_vi_dev(st));
    st->vi_dev_enabled = false;
}

/* ================================================================
 * VPSS
 * ================================================================ */

/*
 * hisi_vpss_bringup -- the group, and the bind that feeds it.
 *
 * One group per sensor. Its channels are created by the framesource ops as
 * rvd asks for streams; the group itself is a property of the pipeline.
 *
 * max_width/max_height are the group's *allocation*, so they are the
 * sensor's full output whatever the streams turn out to be: a channel
 * cannot ask for more than the group was built for, and growing the group
 * later means destroying it, which drops the bind with it.
 */
static int hisi_vpss_bringup(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v5_vpss_grp_attr grp;
    v5_mpp_chn src;
    v5_mpp_chn dst;
    int ret;

    memset(&grp, 0, sizeof(grp));
    grp.max_width = m->dev_rect.width;
    grp.max_height = m->dev_rect.height;
    grp.pixel_format = V5_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    grp.dynamic_range = V5_DYNAMIC_RANGE_SDR8;
    grp.dei_mode = V5_VPSS_DEI_MODE_OFF;
    /*
     * 0, not V5_VPSS_CHN_INVALID. buf_share_en is off, so the field is
     * inert -- but the driver range-checks it anyway and create_grp
     * returns OT_ERR_VPSS_ILLEGAL_PARAM (0xa0078007) for -1. Measured on
     * the bench; there is nothing in the header that says so.
     */
    grp.buf_share_chn = 0;
    grp.frame_rate.src_frame_rate = -1;
    grp.frame_rate.dst_frame_rate = -1;

    ret = st->vpss.fnCreateGrp(HISI_VPSS_GRP, &grp);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_create_grp(grp %d) failed: 0x%x", HISI_VPSS_GRP, ret);
        return RSS_ERR_IO;
    }
    st->vpss_grp_created = true;

    /*
     * The wrap ring's precondition under the all-online coupling, and the
     * reason channel 0 was taking a 6.1 MB private pool where the ring is
     * 0.4: CV610 accepts ss_mpi_vpss_set_chn_buf_wrap there only with the
     * group's frame interrupt at START or EARLY_END and early_line at half
     * the group's max height. Both the interrupt attribute's page and the
     * wrap attribute's say so ("4 视频处理子系统"), and the vendor's own
     * samples write exactly these two values
     * (sample_vie.c's sample_vie_vpss_set_wrap_grp_int_attr,
     * sample_ir_auto.c). Without it the wrap call answers NOT_PERM.
     *
     * Asked for only in that coupling -- the attribute is documented as
     * valid in no other -- and here, with the group created and no
     * physical channel enabled yet, which is the state a change of
     * interrupt mode requires.
     */
    if (st->vi_all_online && st->vb_wrap_blk && st->vpss.fnSetGrpFrameInterrupt) {
        v5_frame_interrupt_attr fi;

        fi.interrupt_type = V5_FRAME_INTERRUPT_EARLY_END;
        fi.early_line = grp.max_height / 2u;

        ret = st->vpss.fnSetGrpFrameInterrupt(HISI_VPSS_GRP, &fi);
        if (ret)
            HAL_LOG_WARN("vpss: ss_mpi_vpss_set_grp_frame_interrupt_attr(EARLY_END, line %u) "
                         "failed: 0x%x; channel 0 cannot take the wrap ring",
                         fi.early_line, ret);
        else
            HAL_LOG_INFO("vpss: frame interrupt EARLY_END at line %u -- what the all-online "
                         "coupling wants before channel 0 can wrap",
                         fi.early_line);
    }

    ret = st->vpss.fnStartGrp(HISI_VPSS_GRP);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_start_grp(grp %d) failed: 0x%x", HISI_VPSS_GRP, ret);
        return RSS_ERR_IO;
    }
    st->vpss_grp_started = true;

    /*
     * VI channel -> VPSS group. Called on either coupling, which is worth
     * saying because gen4's rule was the opposite: there, a VPSS-online
     * mode wired the two in hardware and the software bind had to be
     * skipped. V5's own all-online sample binds anyway
     * (sample_venc_online_wrap_start_sys_vi_vpss calls
     * sample_comm_vi_bind_vpss, and that helper checks no mode), so the
     * bind is the declaration of the edge rather than the plumbing of it.
     *
     * The destination channel is 0 by convention -- a group has one input
     * -- which is also the reason gen4 cannot use physical channel 0 as an
     * output. Whether that holds here is plan risk R4 and the framesource
     * is what answers it.
     */
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    src.mod_id = V5_MOD_VI;
    src.dev_id = HISI_VI_PIPE;
    src.chn_id = HISI_VI_CHN;
    dst.mod_id = V5_MOD_VPSS;
    dst.dev_id = HISI_VPSS_GRP;
    dst.chn_id = 0;

    if (!st->sys.fnBind) {
        HAL_LOG_ERR("vpss: no ss_mpi_sys_bind; VI cannot be connected to VPSS");
        return RSS_ERR_NOTSUP;
    }

    ret = st->sys.fnBind(&src, &dst);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_sys_bind(VI %d/%d -> VPSS %d/0) failed: 0x%x", HISI_VI_PIPE,
                    HISI_VI_CHN, HISI_VPSS_GRP, ret);
        return RSS_ERR_IO;
    }
    st->vi_vpss_bound = true;

    HAL_LOG_INFO("vpss: grp %d up, %ux%u, fed from VI %d/%d", HISI_VPSS_GRP, grp.max_width,
                 grp.max_height, HISI_VI_PIPE, HISI_VI_CHN);
    return RSS_OK;
}

static void hisi_vpss_teardown(hisi_state_t *st)
{
    v5_mpp_chn src;
    v5_mpp_chn dst;

    if (st->vi_vpss_bound && st->sys.fnUnbind) {
        memset(&src, 0, sizeof(src));
        memset(&dst, 0, sizeof(dst));
        src.mod_id = V5_MOD_VI;
        src.dev_id = HISI_VI_PIPE;
        src.chn_id = HISI_VI_CHN;
        dst.mod_id = V5_MOD_VPSS;
        dst.dev_id = HISI_VPSS_GRP;
        dst.chn_id = 0;
        st->sys.fnUnbind(&src, &dst);
    }
    st->vi_vpss_bound = false;

    if (st->vpss_grp_started)
        st->vpss.fnStopGrp(HISI_VPSS_GRP);
    st->vpss_grp_started = false;

    if (st->vpss_grp_created)
        st->vpss.fnDestroyGrp(HISI_VPSS_GRP);
    st->vpss_grp_created = false;
}

/* ================================================================
 * THE VPSS -> VENC EDGE
 *
 * Shared by the encoder's register path and by the generic bind op below,
 * because both express the same thing: a framesource's pictures going to
 * an encoder channel.
 * ================================================================ */

int hisi_bind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn)
{
    v5_mpp_chn src, dst;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (fs_chn < 0 || fs_chn >= HISI_VPSS_CHN_NUM || enc_chn < 0 || enc_chn >= HISI_VENC_CHN_NUM)
        return RSS_ERR_INVAL;
    if (!st->sys.fnBind)
        return RSS_ERR_NOTSUP;

    /*
     * Idempotence, because rvd asks twice per stream: once through
     * enc_register_channel and once through the FS -> ENC bind chain.
     * Without this the second bind of the same edge returns NOT_PERM,
     * which the recovery below would misread as another process's
     * leftovers -- it would unbind the edge made a moment ago and remake
     * it, turning every normal stream start into a spurious stale-bind
     * warning with a sourceless window in the middle.
     */
    if (st->enc[enc_chn].bound_fs == fs_chn)
        return RSS_OK;

    /*
     * An MJPEG channel that is not yet receiving records the edge instead
     * of making it. rvd binds its snapshot channels while they are idle,
     * and an idle-but-bound destination is the VB wedge hal_enc_stop
     * describes -- so the bind waits for enc_start, which makes the
     * channel receive and then calls back here.
     */
    if (st->enc[enc_chn].payload == V5_PT_MJPEG && !st->enc[enc_chn].receiving) {
        st->enc[enc_chn].idle_fs = fs_chn;
        HAL_LOG_DBG("bind: VPSS(%d,%d) -> VENC(%d) deferred until the channel receives",
                    HISI_VPSS_GRP, hisi_vpss_phy(fs_chn), enc_chn);
        return RSS_OK;
    }

    memset(&src, 0, sizeof(src));
    src.mod_id = V5_MOD_VPSS;
    src.dev_id = HISI_VPSS_GRP;
    src.chn_id = hisi_vpss_phy(fs_chn);

    memset(&dst, 0, sizeof(dst));
    dst.mod_id = V5_MOD_VENC;
    dst.dev_id = 0;
    dst.chn_id = enc_chn;

    ret = st->sys.fnBind(&src, &dst);

    /*
     * A bind that survived the last process.
     *
     * The kernel holds the bind table, so a VPSS -> VENC edge outlives the
     * rvd that made it: a crash or a kill during bring-up leaves
     * VENC(enc_chn) still bound, and the next start gets NOT_PERM from a
     * bind nothing in userspace remembers making. Same class of leak the
     * rest of this backend answers by tearing down first.
     *
     * ss_mpi_sys_unbind matches on the source as well as the destination,
     * so clearing the destination means naming a source that is exactly
     * what is not known here. Sweeping the group's channels covers it:
     * the stale edge can only have come from this group.
     */
    if (ret && V5_ERR_ID(ret) == V5_ERR_NOT_PERM && st->sys.fnUnbind) {
        int chn;

        HAL_LOG_WARN("VENC(%d) still bound from a previous process; clearing", enc_chn);

        for (chn = 0; chn < HISI_VPSS_CHN_NUM; chn++) {
            v5_mpp_chn stale = src;

            stale.chn_id = hisi_vpss_phy(chn);
            if (st->sys.fnUnbind(&stale, &dst) == 0)
                break;
        }

        ret = st->sys.fnBind(&src, &dst);
    }

    if (ret) {
        HAL_LOG_ERR("ss_mpi_sys_bind VPSS(%d,%d) -> VENC(%d) failed: 0x%x", HISI_VPSS_GRP,
                    hisi_vpss_phy(fs_chn), enc_chn, ret);
        return RSS_ERR_IO;
    }

    st->enc[enc_chn].bound_fs = fs_chn;
    st->enc[enc_chn].idle_fs = -1;
    /* The rc attribute written at create predates the bind and so named
     * the sensor's frame rate as its source; now that the source channel
     * is known, re-derive it. See hisi_enc_refresh_rc. */
    hisi_enc_refresh_rc(st, enc_chn);
    HAL_LOG_DBG("bind: VPSS(%d,%d) -> VENC(%d)", HISI_VPSS_GRP, hisi_vpss_phy(fs_chn), enc_chn);
    return RSS_OK;
}

int hisi_unbind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn)
{
    v5_mpp_chn src, dst;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (fs_chn < 0 || fs_chn >= HISI_VPSS_CHN_NUM || enc_chn < 0 || enc_chn >= HISI_VENC_CHN_NUM)
        return RSS_ERR_INVAL;
    if (!st->sys.fnUnbind)
        return RSS_ERR_NOTSUP;

    memset(&src, 0, sizeof(src));
    src.mod_id = V5_MOD_VPSS;
    src.dev_id = HISI_VPSS_GRP;
    src.chn_id = hisi_vpss_phy(fs_chn);

    memset(&dst, 0, sizeof(dst));
    dst.mod_id = V5_MOD_VENC;
    dst.dev_id = 0;
    dst.chn_id = enc_chn;

    ret = st->sys.fnUnbind(&src, &dst);
    if (ret)
        HAL_LOG_WARN("ss_mpi_sys_unbind VPSS(%d,%d) -> VENC(%d) failed: 0x%x", HISI_VPSS_GRP,
                     hisi_vpss_phy(fs_chn), enc_chn, ret);

    st->enc[enc_chn].bound_fs = -1;
    return RSS_OK;
}

/*
 * hisi_bind_collapse -- turn rvd's cell pair into (framesource, encoder).
 *
 * rvd expresses an overlaid stream as FS -> OSD -> ENC, two binds where
 * only the first names the framesource. HiMPP has no OSD stage in the
 * datapath: RGN regions attach to a VENC channel, so the pair collapses to
 * one FS -> VENC bind and the first half is recorded rather than acted on.
 * Same shape as the gen4 backend and the SigmaStar one, and for the same
 * reason. There is no IVS on this backend at all, so FS [-> OSD] -> ENC is
 * the whole grammar.
 */
static int hisi_bind_collapse(hisi_state_t *st, const rss_cell_t *src, const rss_cell_t *dst,
                              int *fs_chn, int *enc_chn, bool *collapsed)
{
    *collapsed = false;

    if (!src || !dst)
        return RSS_ERR_INVAL;

    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_ENC) {
        *fs_chn = src->group;
        *enc_chn = dst->group;
        return RSS_OK;
    }

    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_OSD) {
        if (dst->group < 0 || dst->group >= HISI_VENC_CHN_NUM)
            return RSS_ERR_INVAL;
        st->osd_src_fs[dst->group] = src->group;
        *collapsed = true;
        return RSS_OK;
    }

    if (src->device == RSS_DEV_OSD && dst->device == RSS_DEV_ENC) {
        if (src->group < 0 || src->group >= HISI_VENC_CHN_NUM)
            return RSS_ERR_INVAL;
        *fs_chn = st->osd_src_fs[src->group];
        *enc_chn = dst->group;
        if (*fs_chn < 0) {
            HAL_LOG_ERR("bind: OSD %d -> ENC %d without a preceding FS -> OSD", src->group,
                        dst->group);
            return RSS_ERR_INVAL;
        }
        return RSS_OK;
    }

    HAL_LOG_ERR("bind: FS -> [OSD ->] ENC is the only chain this backend supports (got %d -> %d)",
                src->device, dst->device);
    return RSS_ERR_NOTSUP;
}

/* hal_bind / hal_unbind -- rvd's cell pairs, through the collapse above. */
static int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    hisi_state_t *st = hisi_state(ctx);
    bool collapsed;
    int fs_chn = -1;
    int enc_chn = -1;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = hisi_bind_collapse(st, src, dst, &fs_chn, &enc_chn, &collapsed);
    if (ret)
        return ret;
    if (collapsed)
        return RSS_OK; /* Recorded; the OSD -> ENC step does the work. */

    ret = hisi_bind_vpss_venc(st, fs_chn, enc_chn);
    if (ret)
        return ret;

    /*
     * Attach any region registered on this encoder channel before the
     * channel could take one. rvd sets region attributes before the bind
     * exists on every platform; see hal_osd.c's WHY ATTACH IS DEFERRED.
     * Deliberately after the bind and deliberately not checked: a region
     * that will not attach costs an overlay, and failing the bind over it
     * would cost the stream.
     */
    hisi_osd_flush_pending(st, enc_chn);

    return RSS_OK;
}

static int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    hisi_state_t *st = hisi_state(ctx);
    bool collapsed;
    int fs_chn = -1;
    int enc_chn = -1;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = hisi_bind_collapse(st, src, dst, &fs_chn, &enc_chn, &collapsed);
    if (ret)
        return ret;
    if (collapsed) {
        st->osd_src_fs[dst->group] = -1;
        return RSS_OK;
    }

    return hisi_unbind_vpss_venc(st, fs_chn, enc_chn);
}

/* ================================================================
 * THE PIPELINE
 * ================================================================ */

/*
 * hisi_video_bringup -- MIPI, VI, sensor, ISP, VPSS, in that order.
 *
 * The order is sample_comm_vi_start_vi's and the three constraints inside
 * it are:
 *
 *   - MIPI before everything, because ENABLE_SENSOR_CLOCK is what gives
 *     the sensor an MCLK. Before it, every I2C write NAKs.
 *
 *   - **VI before the ISP, which is the reverse of gen4.** gen4 brought
 *     the ISP up first and attached the pipe to it; V5's
 *     ss_mpi_isp_mem_init reads the pipe's distribute-group attribute out
 *     of VI, so with no pipe it fails with a bare -1 (TD_FAILURE, not an
 *     MPP error word) after printing
 *
 *       [Func]:isp_get_wdr_dist_attr [Line]:372 [Info]:ISP[0] get WDR attr failed
 *
 *     -- measured on the bench. The message names WDR, which a linear
 *     pipeline has nothing to do with, and that is why it is worth
 *     writing down: the missing thing is the pipe, not a WDR setting.
 *
 *   - The sensor's 3A registration before ss_mpi_isp_mem_init, because
 *     mem_init allocates against the geometry it learns through those
 *     callbacks. Registering afterwards gives an ISP that comes up and
 *     produces a green frame.
 *
 * Partial failure unwinds through hisi_video_teardown, which is flag-driven
 * and safe to call at any point.
 */
static int hisi_video_bringup(hisi_state_t *st, const rss_sensor_config_t *cfg)
{
    int ret;

    if ((ret = hisi_mipi_configure(st)) != RSS_OK)
        return ret;

    /*
     * Teardown-first, for the ISP as for SYS and VB, and **before the
     * sensor registration rather than at the top of hisi_isp_bringup**.
     *
     * The ISP's virtual registers are not process memory:
     * ss_mpi_isp_mem_init asks the kernel for them out of MMZ, and only
     * ss_mpi_isp_exit gives them back. A previous consumer that was killed
     * rather than closed -- on a camera the normal case, not the
     * exceptional one -- leaves them allocated *and leaves its sensor
     * registered*. Exiting after registering ours then runs the stale
     * sensor's cmos_isp_exit over our registration, and mem_init fails
     * with a bare -1 while the vendor prints the *other* sensor's name:
     *
     *   [Func]:cmos_isp_exit [Line]:886 [Info]:SC500AI exit failed!
     *   ss_mpi_isp_mem_init(pipe 0) failed: 0xffffffff
     *
     * -- measured on the bench with an os04d10 in the socket. Exiting
     * first makes that message the stale registration's own epitaph.
     *
     * The result is ignored on purpose, as with sys_exit and vb_exit: on a
     * clean boot there is nothing to exit and the call fails.
     */
    if (st->isp.fnExit)
        st->isp.fnExit(HISI_VI_PIPE);

    if ((ret = hisi_vi_bringup(st)) != RSS_OK)
        return ret;
    if ((ret = hisi_sensor_bringup(st, cfg)) != RSS_OK)
        return ret;
    if ((ret = hisi_isp_bringup(st)) != RSS_OK)
        return ret;
    if ((ret = hisi_vpss_bringup(st)) != RSS_OK)
        return ret;

    return RSS_OK;
}

static void hisi_video_teardown(hisi_state_t *st)
{
    /* Overlays before the encoders, then framesources, then the group. The
     * first step is not tidiness: RGN refuses to let a VENC channel be
     * destroyed while a region is still attached to it (OT_ERR_RGN_BUSY is
     * documented as exactly that case). The rest is the old rule -- an
     * encoder still bound to a VPSS channel that is about to be disabled is
     * the state that leaves the kernel side holding buffers. */
    hisi_osd_release_all(st);
    hisi_enc_release_all(st);
    hisi_fs_release_all(st);
    hisi_vpss_teardown(st);
    hisi_isp_teardown(st);
    hisi_sensor_teardown(st);
    hisi_vi_teardown(st);
    hisi_mipi_shutdown(st);
}

#endif /* HAL_MODULE_VIDEO */

/* ================================================================
 * VB
 * ================================================================ */

/*
 * hisi_vb_fill_cfg -- the common pool configuration.
 *
 * ONE POOL PER CONSUMER, cut to that consumer's own frame. This is the
 * vendor's shape rather than an invention: sample_venc.c's get_vb_attr
 * walks the pipeline and calls update_vb_attr once per stage with that
 * stage's geometry, pixel format and compress mode, then sets
 * max_pool_cnt to however many stages there were. VB hands a request the
 * smallest pool whose blocks are big enough, so exact sizes are the
 * mechanism that stops one stage's frame from occupying another's block.
 *
 * The size-class scheme this replaces -- a full-sensor pool and a
 * quarter-scale one -- was measured on the CV608 and did not work. The
 * quarter pool's blocks came out at 576x324 = 279,936 bytes and the
 * smallest real stream was 640x360 = 345,600, so every sub-stream frame
 * took a 4.27 MiB full-size block and the whole second pool sat unused
 * from boot to teardown. Four blocks then had to serve four simultaneous
 * consumers, VI never saw a spare, and 42% of frames were dropped on
 * vb_fail. A size class only works if something is actually that size.
 *
 * WHAT hal_init KNOWS is the sensor, so what it configures is the VI
 * side:
 *
 *   pool 0   the VI channel's NV21 output at the sensor's size, which is
 *            also what the VPSS group reads over the bind -- **and only
 *            when the VPSS is offline**. Under the all-online coupling
 *            (hisi_vi_vpss_mode) nothing between VI and VPSS goes through
 *            DDR at all, so this pool would be 13,122 KiB of a 32 MiB MMZ
 *            that no consumer can ask for, and it is not configured. The
 *            vendor's own all-online configuration has no such pool
 *            either: sample_venc_online_wrap_get_default_vb_cfg starts its
 *            loop at pool 1.
 *
 * The VPSS channels are deliberately not here. Their geometry is rvd's
 * and arrives later, and each gets a pool cut to its own stream in
 * hisi_fs_pool_acquire. A channel whose pool could not be created falls
 * back to these common pools, which is the other reason pool 0 is sized
 * for the largest frame in the pipeline rather than the VI channel alone
 * -- and, where the all-online coupling leaves no pool 0, the reason that
 * function's failure path is worth reading: there is then nothing behind
 * it.
 *
 * WHY THERE IS NO RAW POOL. The VI pipe puts a raw Bayer frame in a VB
 * block only while it is offline, and hisi_vi_vpss_mode asks for VI
 * online. If that request is ever refused, the pipe falls back to
 * allocating raw from pool 0 -- and a sensor-sized NV21 block is larger
 * than a raw one at every bit depth this backend drives (4,478,976
 * against 3,732,480 at 2304x1296 RAW10), so the fallback costs a block
 * rather than failing to start. The coupling actually in force is logged
 * by hisi_vi_vpss_mode for exactly this reason.
 *
 * THE BLOCK COUNT is three, where the vendor's VI_VB_YUV_CNT is four.
 * The tuning guide's rule (DDR内存调优 / 主要模块工作占用MMZ情况 / VI) is
 * that VI holds at most three frame VBs -- one being filled, one ready for
 * the next frame, one in rotation downstream -- and that a VPSS group
 * holds its input frame plus a backup, which CV610 cannot disable.
 *
 * Four does not fit. This part has 32 MB of MMZ and four sensor-sized
 * blocks are 17.9 MB of it; with the two stream pools on top,
 * ss_mpi_venc_create_chn then fails 0xa0088014 -- OT_ERR_NO_MEM -- because
 * VENC's reference and reconstruction frames do not come out of VB and
 * there is nothing left for them. Three is 13.4 MB and leaves the encoder
 * its room.
 *
 * If this turns out to be one too few it says so precisely, and the fix
 * is a number: /proc/umap/vb reports min_free per pool, and the tuning
 * guide's own advice is to read it -- a min_free that never leaves 0 while
 * VI's vb_fail_cnt climbs is a pool one block short.
 *
 * The audio archive compiles this file too and has no sensor mode, so the
 * whole thing is video-only and hal_init falls back to no pools.
 */
#ifdef HAL_MODULE_VIDEO

#define HISI_VB_VI_BLK_CNT 3u
#define HISI_VB_VI_BLK_CNT_MIN 2u

/*
 * The share of the zone the VI pool may take. Three sensor-sized blocks
 * are 13,122 KiB of the CV608's 32 MiB, or 40%, and the rest of the
 * pipeline -- encoder reconstruction and stream buffers, JPEG, the VI
 * pipe's 3DNR and BNR references, ISP statistics, the sub-stream's own
 * pool -- measured 14.4 MiB beside it. 45% is that measurement rounded up
 * to the nearest number that still yields three here, so this board is
 * unchanged and a part with a smaller zone gets a count that fits rather
 * than a VB init that fails or an encoder that cannot find room.
 */
#define HISI_VB_MMZ_PERCENT 45u
#define HISI_MMZ_PROC "/proc/umap/media-mem"

/* The MMZ's total size in KiB, or 0 when it cannot be read -- an
 * unreadable node is not a reason to fail, only a reason to keep the
 * measured count. gen4 reads the same summary line out of
 * /proc/media-mem; V5 moved the node under umap. */
static unsigned long hisi_mmz_total_kib(void)
{
    char line[256];
    unsigned long kib = 0;
    FILE *f = fopen(HISI_MMZ_PROC, "r");

    if (!f)
        return 0;

    /* "total size=32768KB(32MB),used=32KB(...)" -- the first field of the
     * MMZ_USE_INFO summary. */
    while (fgets(line, sizeof(line), f)) {
        const char *p = strstr(line, "total size=");

        if (p && sscanf(p, "total size=%luKB", &kib) == 1)
            break;
        kib = 0;
    }

    fclose(f);
    return kib;
}

/*
 * How many sensor-sized blocks the VI pool gets, given what one costs and
 * what the wrap ring has already taken.
 *
 * The budget is the whole VB configuration's, so the ring comes off the
 * top: charging only this pool against the bound would put the total over
 * it by exactly the ring's size. Bounded above by the tuning guide's rule
 * -- a VI pipe holds at most three frame VBs, one filling, one ready, one
 * downstream -- and below by two, which is a single channel's working set
 * and the point past which the pipeline is not worth starting.
 */
static unsigned int hisi_vb_vi_blk_cnt(unsigned long long blk_size, unsigned long long reserved_kib)
{
    unsigned long total_kib = hisi_mmz_total_kib();
    unsigned long long budget_kib;
    unsigned long long blk_kib;
    unsigned int cnt;

    {
        /* local experiment: RSS_VB_VI_BLKS=<2..6> forces the block count */
        const char *ov = getenv("RSS_VB_VI_BLKS");
        if (ov && ov[0] >= '2' && ov[0] <= '6' && ov[1] == '\0') {
            HAL_LOG_INFO("vb: RSS_VB_VI_BLKS forces %c VI blocks", ov[0]);
            return (unsigned int)(ov[0] - '0');
        }
    }

    if (!total_kib || !blk_size) {
        HAL_LOG_INFO("vb: MMZ size unknown, asking for the measured %u blocks", HISI_VB_VI_BLK_CNT);
        return HISI_VB_VI_BLK_CNT;
    }

    blk_kib = (blk_size + 1023ull) / 1024ull;
    budget_kib = (unsigned long long)total_kib * HISI_VB_MMZ_PERCENT / 100ull;
    budget_kib = budget_kib > reserved_kib ? budget_kib - reserved_kib : 0ull;
    cnt = (unsigned int)(budget_kib / blk_kib);

    if (cnt >= HISI_VB_VI_BLK_CNT)
        return HISI_VB_VI_BLK_CNT;
    if (cnt < HISI_VB_VI_BLK_CNT_MIN)
        cnt = HISI_VB_VI_BLK_CNT_MIN;

    /* Worth a line at INFO: it decides how many frames VI has in flight,
     * so it is the first thing to look at if vb_fail climbs. */
    HAL_LOG_INFO("vb: MMZ is %lu KiB, so %u blocks of %llu KiB rather than %u -- three would be "
                 "%llu%% of the zone with the ring's %llu KiB beside them",
                 total_kib, cnt, blk_kib, HISI_VB_VI_BLK_CNT,
                 (blk_kib * HISI_VB_VI_BLK_CNT + reserved_kib) * 100ull / total_kib, reserved_kib);
    return cnt;
}

/*
 * hisi_vb_wrap_blk -- the block channel 0's wrap ring will take.
 *
 * The ring is drawn from the common pools when the channel enables its
 * wrap ("从公共VB池拿取合适的卷绕VB", the VPSS reference's note on
 * set_chn_buf_wrap), and the common pools are fixed before ss_mpi_sys_init
 * -- so the block has to be cut now, for a stream whose size rvd has not
 * said yet. What bounds it is the sensor: channel 0 cannot be larger, the
 * ring's line count is the driver's answer for the sensor-sized stream
 * (128 on this part whatever is asked), and the compressed header grows
 * with height, so the sensor-sized ring is the largest any stream can
 * want. About 0.4 MB at 2304x1296 against the 6.2 MB pool it stands in
 * for at 1080p. Returns 0, and no pool is cut, when the driver cannot be
 * asked.
 *
 * Cut for the *uncompressed* ring, which is the larger of the two and the
 * one a turned picture needs: orientation and SEG_COMPACT cannot both be
 * on (hisi_fs_compress), so an [image] hflip drops channel 0's
 * compression, and a block cut only for the compressed ring is then
 * 412,696 B against the 442,368 the ring wants -- the ring cannot be
 * re-cut, channel 0 falls off it, and in the all-online coupling there is
 * nothing underneath. 29,672 B at 2304x1296 buys a mirror that keeps the
 * ring; the compressed ring still fits in the larger block, VB handing
 * out the smallest block that fits.
 */
static unsigned long long hisi_vb_wrap_blk(const hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v5_vpss_venc_wrap_param p;
    unsigned int line = 0;
    int ret;

    if (!st->sys.fnGetVpssVencWrapBufLine || !st->vpss.fnSetChnBufWrap)
        return 0;

    /* The coupling is part of the question -- the sample fills this field
     * from the same mode value (sample_comm_vpss.c's
     * sample_comm_vpss_get_wrap_cfg) -- and under the all-online one the
     * driver checks the rest of it; hisi_wrap_param carries the measured
     * rules. */
    memset(&p, 0, sizeof(p));
    hisi_wrap_param(&p, st->vi_all_online ? 1 : 0,
                    m->frame_rate > 1.0f ? (unsigned int)(m->frame_rate + 0.5f) : 30u,
                    m->dev_rect.width, m->dev_rect.height);

    ret = st->sys.fnGetVpssVencWrapBufLine(&p, &line);
    if (ret || !line || line > m->dev_rect.height) {
        HAL_LOG_INFO("vb: ss_mpi_sys_get_vpss_venc_wrap_buf_line -> 0x%x, %u lines; channel 0 "
                     "streams from whole frames",
                     ret, line);
        return 0;
    }
    {
        unsigned long long plain =
            hisi_vb_wrap_size(m->dev_rect.width, m->dev_rect.height, line, V5_COMPRESS_MODE_NONE);
        unsigned long long packed = hisi_vb_wrap_size(m->dev_rect.width, m->dev_rect.height, line,
                                                      V5_COMPRESS_MODE_SEG_COMPACT);

        return plain > packed ? plain : packed;
    }
}

static void hisi_vb_fill_cfg(hisi_state_t *st, v5_vb_cfg *cfg)
{
    const hisi_sensor_mode_t *m = &st->mode;
    unsigned long long full = hisi_vb_nv12_size(m->dev_rect.width, m->dev_rect.height);
    unsigned int pool = 0;

    memset(cfg, 0, sizeof(*cfg));

    /* The ring first, because the VI pool's count is what is left of the
     * budget once the ring has taken its block. */
    st->vb_wrap_blk = hisi_vb_wrap_blk(st);

    if (!st->vi_all_online) {
        unsigned int cnt = hisi_vb_vi_blk_cnt(full, (st->vb_wrap_blk + 1023ull) / 1024ull);

        cfg->common_pool[pool].blk_size = full;
        cfg->common_pool[pool].blk_cnt = cnt;
        cfg->common_pool[pool].remap_mode = V5_VB_REMAP_NONE;
        HAL_LOG_INFO("vb: pool %u = VI chn %ux%u NV21, %llu B x%u = %llu KiB", pool,
                     m->dev_rect.width, m->dev_rect.height, full, cnt, (full * cnt) >> 10);
        pool++;
    } else {
        HAL_LOG_INFO("vb: VI and VPSS both online, so no sensor-sized common pool -- %llu KiB the "
                     "offline coupling would have spent on frames nothing reads",
                     (full * HISI_VB_VI_BLK_CNT) >> 10);
    }

    /*
     * channel 0's wrap ring, one block. VB hands a request the smallest
     * block that fits, so this one is never mistaken for a VI frame and a
     * VI frame never lands in it.
     */
    if (st->vb_wrap_blk) {
        cfg->common_pool[pool].blk_size = st->vb_wrap_blk;
        cfg->common_pool[pool].blk_cnt = 1;
        cfg->common_pool[pool].remap_mode = V5_VB_REMAP_NONE;
        HAL_LOG_INFO("vb: pool %u = VPSS chn 0 wrap ring, %llu B x1", pool, st->vb_wrap_blk);
        pool++;
    }

    cfg->max_pool_cnt = pool;

    /*
     * REMAP_NONE: nothing in the streaming path reads a VB block from
     * userspace -- VPSS feeds VENC over a bind and VENC's output is a
     * stream buffer, not a VB block. A mapping nobody uses costs address
     * space and cache-maintenance bookkeeping. The snapshot path is what
     * would want CACHED, and it gets a pool of its own when it exists.
     *
     * mmz_name left empty, which means the anonymous zone -- the only one
     * this board has (/proc/umap/media-mem shows one ZONE named
     * "anonymous").
     */
}

#endif /* HAL_MODULE_VIDEO */

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

/*
 * hisi_teardown -- put MPP back the way it was found.
 *
 * Reverse of bring-up, and the order is the vendor's rather than a
 * preference: ss_mpi_vb_exit with any module still holding a block returns
 * OT_ERR_VB_BUSY, so SYS has to go first. Phase 2 inserts the pipeline
 * reclaim above both.
 *
 * Idempotent and flag-driven, so hal_deinit after a failed hal_init tears
 * down exactly what came up.
 */
static void hisi_teardown(hisi_state_t *st)
{
#ifdef HAL_MODULE_VIDEO
    /* The pipeline before SYS: ss_mpi_sys_exit with VI or VPSS still
     * holding blocks is the same OT_ERR_VB_BUSY problem one level up. */
    hisi_video_teardown(st);
#endif

    if (st->sys_inited) {
        if (st->sys.fnExit)
            st->sys.fnExit();
        st->sys_inited = false;
    }

    if (st->vb_inited) {
        if (st->vb.fnExit) {
            int rc = st->vb.fnExit();

            if (rc)
                HAL_LOG_WARN("vb: ss_mpi_vb_exit returned 0x%x (err %u)", (unsigned)rc,
                             V5_ERR_ID(rc));
        }
        st->vb_inited = false;
    }
}

/*
 * hisi_sys_bringup -- VB configured, VB up, SYS up, and the coupling asked
 * for.
 *
 * Its own function because it runs more than once. The pool configuration
 * has to be handed to VB before SYS starts, the VI/VPSS coupling cannot be
 * asked for until after SYS has started, and the two have to agree: the
 * all-online coupling makes the sensor-sized pool dead weight, the offline
 * one cannot run without it. So hal_init lays the pools down for the
 * coupling it wants, this reports what the driver actually gave, and when
 * those differ the whole of VB and SYS goes back down (hisi_teardown, which
 * undoes exactly the three steps here) and comes up again in the shape that
 * matches.
 *
 * The audio archive has no sensor and no pools; it still needs VB
 * configured, because ss_mpi_sys_init will not run on an unconfigured VB.
 */
static int hisi_sys_bringup(hisi_state_t *st)
{
    v5_vb_cfg vb_cfg;
    int ret;

#ifdef HAL_MODULE_VIDEO
    hisi_vb_fill_cfg(st, &vb_cfg);
#else
    memset(&vb_cfg, 0, sizeof(vb_cfg));
    vb_cfg.max_pool_cnt = 0;
#endif

    ret = st->vb.fnSetCfg(&vb_cfg);
    if (ret) {
        HAL_LOG_ERR("vb: ss_mpi_vb_set_cfg failed 0x%x (err %u)", (unsigned)ret, V5_ERR_ID(ret));
        return RSS_ERR_IO;
    }

    ret = st->vb.fnInit();
    if (ret) {
        HAL_LOG_ERR("vb: ss_mpi_vb_init failed 0x%x (err %u)", (unsigned)ret, V5_ERR_ID(ret));
        return RSS_ERR_IO;
    }
    st->vb_inited = true;

    ret = st->sys.fnInit();
    if (ret) {
        HAL_LOG_ERR("sys: ss_mpi_sys_init failed 0x%x (err %u)", (unsigned)ret, V5_ERR_ID(ret));
        return RSS_ERR_IO;
    }
    st->sys_inited = true;

#ifdef HAL_MODULE_VIDEO
    /*
     * AFTER sys_init and before the VI bring-up, which is where the vendor
     * puts it: sample_vio_sys_init calls sample_comm_sys_init (vb_set_cfg,
     * vb_init, sys_init) and only then sample_comm_vi_set_vi_vpss_mode.
     *
     * This was between vb_init and sys_init and looked correct there --
     * the coupling does have to be settled before VI is created. It is
     * not: ss_mpi_sys_set_vi_vpss_mode returns 0xa002800d, NOT_PERM, on an
     * uninitialised SYS. Nothing said so, because the value asked for used
     * to be the value already in force and the call was never reached.
     */
    hisi_vi_vpss_mode(st);
#endif

    return RSS_OK;
}

/*
 * hal_init -- open the libraries and bring MPP up as far as SYS.
 *
 * The sequence, and every step of it is load-bearing:
 *
 *   check trampolines   before the first vendor dlopen; see that block
 *   dlopen              libsecurec, sysmem, libss_mpi, sysbind
 *   resolve             SYS and VB
 *   identify            chip id and MPP version, logged
 *   sys_exit + vb_exit  teardown-first
 *   vb_set_cfg          the pool configuration
 *   vb_init
 *   sys_init
 *
 * TEARDOWN-FIRST is inherited from gen4 and is not defensiveness. MPP state
 * lives in the kernel modules, not in the process: a previous run that was
 * killed, or a daemon restarted without the modules being reloaded, leaves
 * SYS initialised and VB holding its pools. ss_mpi_vb_set_cfg on a live VB
 * returns OT_ERR_VB_NOT_PERM, and the whole bring-up then fails for a
 * reason that has nothing to do with this run's configuration. Calling exit
 * on something that was never inited is harmless -- it returns
 * OT_ERR_*_NOT_READY, which is why the return values here are ignored on
 * purpose rather than by omission.
 *
 * PHASE 1 CONFIGURES NO POOLS. max_pool_cnt is 0, because the pool
 * arithmetic needs the sensor geometry and the stream configuration, and
 * neither exists until Phase 2 brings up hisi_sensor.c. VB comes up with no
 * common pools, SYS initialises on top of it, and the acceptance test --
 * reach sys_init, log the version, decline the pipeline -- is met. Phase 2
 * replaces the zero with hisi_vb_bringup's two-pool arithmetic, and Phase 7
 * measures it.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st;
    int ret;

    if (!c || !cfg || cfg->sensor_count < 1 || cfg->sensor_count > RSS_MAX_SENSORS)
        return RSS_ERR_INVAL;

    if (c->initialized) {
        HAL_LOG_ERR("hal_init: already initialized");
        return RSS_ERR_BUSY;
    }

    if (cfg->sensor_count > 1)
        HAL_LOG_WARN("hal_init: %d sensors requested, gen5 backend drives 1", cfg->sensor_count);

    memcpy(&c->multi_cfg, cfg, sizeof(c->multi_cfg));
    c->sensor_count = 1;
    memcpy(&c->sensors[0], &cfg->sensors[0], sizeof(c->sensors[0]));

    st = (hisi_state_t *)calloc(1, sizeof(*st));
    if (!st)
        return RSS_ERR_NOMEM;
    /* The codec fd too: a video-only run of an audio-enabled build would
     * otherwise close(0) in hal_deinit and ioctl(0, ...) from the volume
     * and gain setters before audio_init opened it. */
    st->acodec_fd = -1;
    /*
     * -1, not the 0 calloc left behind: framesource 0 is a real
     * framesource, so "no FS -> OSD half seen yet" needs a value of its
     * own. See hisi_bind_collapse.
     */
    {
        int i;

        for (i = 0; i < HISI_VENC_CHN_NUM; i++)
            st->osd_src_fs[i] = -1;
    }
    c->platform = st;

    /*
     * Published here, not at the end of a successful hal_init.
     *
     * The forwarders reach the algorithm libraries through g_hisi, and the
     * ISP calls them *during* bring-up -- ss_mpi_isp_init registers ldci,
     * drc, dehaze, bayer_nr and acs before it returns. With g_hisi still
     * NULL at that point every one of them declines and the ISP comes up
     * with no algorithms and five lines of "called with no ISP loaded" in
     * the log, which is exactly what the bench showed. hal_deinit clears
     * it again after the teardown, for the symmetric reason.
     */
    g_hisi = st;

    snprintf(st->sensor_name, sizeof(st->sensor_name), "%s", cfg->sensors[0].name);

    /*
     * Before the first vendor dlopen, for the relocation reason in the
     * FORWARDERS block. Nothing below may be reordered above this line.
     */
    hisi_check_trampolines();

    ret = hisi_mpi_open(&st->libs);
    if (ret)
        goto err_free;

    ret = v5_sys_load(&st->sys, &st->libs);
    if (ret)
        goto err_unload;

    ret = v5_vb_load(&st->vb, &st->libs);
    if (ret)
        goto err_unload;

#ifdef HAL_MODULE_VIDEO
    if ((ret = v5_vi_load(&st->vi, &st->libs)) != RSS_OK)
        goto err_unload;
    if ((ret = v5_vpss_load(&st->vpss, &st->libs)) != RSS_OK)
        goto err_unload;
    if ((ret = v5_venc_load(&st->venc, &st->libs)) != RSS_OK)
        goto err_unload;

    /*
     * RGN is the exception to the pattern above: its failure is recorded,
     * not propagated. A board whose libss_mpi.so has no RGN is a board
     * without overlays, and the supported way to say that is for the osd_*
     * ops to answer RSS_ERR_NOTSUP -- which rvd turns into "overlays
     * disabled" and keeps streaming. Refusing hal_init would take the video
     * down over the text drawn on top of it.
     */
    st->rgn_loaded = v5_rgn_load(&st->rgn, &st->libs) == RSS_OK;
    if (!st->rgn_loaded)
        HAL_LOG_WARN("osd: ss_mpi_rgn unavailable; overlays disabled");

    /*
     * Can a VPSS channel be given a pool of its own? All three calls are
     * needed together -- select USER, attach the pool, and be able to
     * detach it again at teardown -- and a board missing any of them
     * simply has every channel draw from the common pools, which is
     * correct and merely wasteful. See hisi_fs_pool_acquire.
     */
    st->vb_private_pools = st->vb.fnCreatePool && st->vb.fnDestroyPool && st->vpss.fnSetChnVbSrc &&
                           st->vpss.fnAttachChnVbPool && st->vpss.fnDetachChnVbPool;
    if (!st->vb_private_pools)
        HAL_LOG_INFO("vb: no per-channel pools on this image; VPSS channels draw common blocks");

    /*
     * The ISP tier last of the libraries, because opening it is what closes
     * the dlopen cycle -- see hisi_isp_open and the FORWARDERS block.
     */
    if ((ret = hisi_isp_open(st)) != RSS_OK)
        goto err_unload;
#endif

    /*
     * Identity, logged before anything is initialised so that a bring-up
     * failure below still leaves the two facts a bug report needs.
     *
     * The chip name was already read once by rss_hal_check_platform, which
     * runs before hal_init and has no state to cache it in. Reading it
     * again costs one small file and keeps the two paths independent.
     */
    if (hisi_read_chip_name(st->chip_name, sizeof(st->chip_name)) != RSS_OK)
        HAL_LOG_WARN("chip: %s unreadable; is the open_* module set loaded?", HISI_UMAP_SYS_PATH);

    if (st->sys.fnGetChipId) {
        unsigned int id = 0;

        if (!st->sys.fnGetChipId(&id)) {
            st->chip_id = id;
            st->chip_id_valid = true;
        }
    }

    if (st->sys.fnGetVersion) {
        v5_mpp_version ver;

        memset(&ver, 0, sizeof(ver));
        if (!st->sys.fnGetVersion(&ver)) {
            /*
             * The vendor fills this from its own OT_VERSION string and
             * leaves the assignment in: ss_mpi_sys_get_version hands back
             * "OT_VERSION=HI3516CV610_MPP_V1.0.2.0 B051 Release", where
             * gen4's HI_MPI_SYS_GetVersion returned the bare version. The
             * prefix reaches rvd's banner as "LIBIMP Version OT_VERSION=..."
             * if it is not taken off here, so it is taken off here rather
             * than at each of the three call sites that print it.
             *
             * Conditional, not unconditional: a build that ever stops
             * doing this should keep working, and a version string that
             * does not start with the prefix is used whole.
             */
            static const char pfx[] = "OT_VERSION=";
            const char *v = ver.version;

            if (!strncmp(v, pfx, sizeof(pfx) - 1))
                v += sizeof(pfx) - 1;

            snprintf(st->mpp_version, sizeof(st->mpp_version), "%.*s",
                     (int)(sizeof(ver.version) - (size_t)(v - ver.version)), v);
        }
    }

    HAL_LOG_INFO("mpp: %s", st->mpp_version[0] ? st->mpp_version : "version unavailable");
    if (st->chip_id_valid)
        HAL_LOG_INFO("chip: %s (id 0x%08x)", st->chip_name[0] ? st->chip_name : "unknown",
                     st->chip_id);
    else
        HAL_LOG_INFO("chip: %s", st->chip_name[0] ? st->chip_name : "unknown");

    /* Teardown-first; see the function comment. Return values ignored on
     * purpose: on a clean boot both report NOT_READY. */
    (void)st->sys.fnExit();
    (void)st->vb.fnExit();

#ifndef HAL_MODULE_VIDEO
    if ((ret = hisi_sys_bringup(st)) != RSS_OK)
        goto err_teardown;
#endif

#ifdef HAL_MODULE_VIDEO
    /*
     * The sensor mode, before VB and not with the rest of the video
     * bring-up, because the pool sizes come out of it and VB has to be
     * configured before ss_mpi_sys_init. Failing here costs nothing: no
     * MPP state has been created yet.
     */
    ret = hisi_sensor_mode_load(&st->mode, cfg->sensors[0].name, st->chip_name);
    if (ret)
        goto err_unload;
    /* The mode load is where a name the config left out gets resolved. */
    snprintf(st->sensor_name, sizeof(st->sensor_name), "%s", st->mode.name);

    /*
     * Ask for the coupling that costs the least memory, and be ready to be
     * told no. Neither of the two attempts below is error handling: the
     * pools are laid down for a coupling that has not been granted yet,
     * and the grant is the first thing hisi_sys_bringup can report.
     */
    st->vi_all_online = true;
    if ((ret = hisi_sys_bringup(st)) != RSS_OK)
        goto err_teardown;

    if (!st->vi_all_online) {
        HAL_LOG_INFO("vb: the all-online coupling was not granted; rebuilding VB with the "
                     "sensor-sized pool the offline path reads from");
        hisi_teardown(st);
        if ((ret = hisi_sys_bringup(st)) != RSS_OK)
            goto err_teardown;
    }

    ret = hisi_video_bringup(st, &cfg->sensors[0]);
    if (ret != RSS_OK && st->vi_all_online) {
        /*
         * The coupling was granted and the pipeline still did not come up
         * on it. Rather than guess which stage minded, put the whole of
         * MPP back down and bring it up on the coupling gen4 used, with
         * the pool that feeds it -- a camera on the expensive coupling
         * beats a correct diagnosis and no video.
         */
        HAL_LOG_WARN("video: bring-up failed (%d) with VI and VPSS both online; retrying on the "
                     "online/offline coupling",
                     ret);
        hisi_teardown(st);
        st->vi_all_online = false;
        if ((ret = hisi_sys_bringup(st)) == RSS_OK)
            ret = hisi_video_bringup(st, &cfg->sensors[0]);
    }
    if (ret != RSS_OK)
        goto err_teardown;

    /* Settle which IQ tuning file applies; the load itself waits for the
     * first encoded frame. Never a failure -- see hal_isp.c. */
    hisi_isp_resolve_iq(st);
#endif

    c->initialized = true;
    return RSS_OK;

err_teardown:
    hisi_teardown(st);
err_unload:
#ifdef HAL_MODULE_VIDEO
    hisi_sensor_teardown(st);
    hisi_isp_close_alg_libs(st);
    v5_isp_unload(&st->isp);
    v5_rgn_unload(&st->rgn);
    st->rgn_loaded = false;
    v5_venc_unload(&st->venc);
    v5_vpss_unload(&st->vpss);
    v5_vi_unload(&st->vi);
#endif
    v5_vb_unload(&st->vb);
    v5_sys_unload(&st->sys);
    hisi_mpi_close(&st->libs);
err_free:
    if (g_hisi == st)
        g_hisi = NULL;
#ifdef HAL_MODULE_VIDEO
    /* The dynamic sections' ladders: video-only state, and hal_dyn.c is in
     * the video archive only. */
    hisi_dyn_free(st);
    hisi_nrx_free(st);
    hisi_lad_free(st);
#endif
    free(st);
    c->platform = NULL;
    return ret;
}

static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st = hisi_state(ctx);

    if (!c)
        return RSS_ERR_INVAL;

    if (!st)
        return RSS_OK;

#ifdef HAL_MODULE_AUDIO
    /* A caller that skips the audio_deinit op still must not free state
     * under a running AI device or an open /dev/acodec. Idempotent. */
    hal_audio_deinit(ctx);
#endif

    hisi_teardown(st);

    /*
     * Cleared after the teardown, not before.
     *
     * The forwarders are entered from the vendor libraries, and from Phase 2
     * the last thing that legitimately does so is the ISP's own unregister
     * path inside the pipeline teardown. Clearing this first would make
     * those decline and leave the ISP holding callbacks into a library
     * about to be dlclosed.
     */
    if (g_hisi == st)
        g_hisi = NULL;

#ifdef HAL_MODULE_VIDEO
    /* After hisi_teardown, which has already unregistered 3A and stopped
     * the ISP thread: the sensor library is dlclosed here and the object
     * inside it dies with the mapping. */
    hisi_sensor_teardown(st);
    hisi_isp_close_alg_libs(st);
    v5_isp_unload(&st->isp);
    v5_rgn_unload(&st->rgn);
    st->rgn_loaded = false;
    v5_venc_unload(&st->venc);
    v5_vpss_unload(&st->vpss);
    v5_vi_unload(&st->vi);
#endif
    v5_vb_unload(&st->vb);
    v5_sys_unload(&st->sys);
    hisi_mpi_close(&st->libs);

#ifdef HAL_MODULE_VIDEO
    hisi_dyn_free(st);
    hisi_nrx_free(st);
    hisi_lad_free(st);
#endif
    free(st);
    c->platform = NULL;
    c->initialized = false;

    return RSS_OK;
}

/*
 * hal_get_caps -- return the per-SoC capability struct.
 *
 * The context's copy, not g_hal_caps directly: rss_hal_create_backend
 * adjusts the backend-surface flags on it.
 */
static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? &c->caps : &g_hal_caps;
}

/* ================================================================
 * SYSTEM UTILITIES
 * ================================================================ */

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->mpp_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->mpp_version);
    return RSS_OK;
}

static int hal_sys_get_cpu_info(void *ctx, char *buf, int len)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->chip_name[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->chip_name);
    return RSS_OK;
}

/*
 * Media clock. rvd_frame_loop.c uses these to publish the
 * media-clock-to-UTC mapping SEI timecodes are derived from; without them
 * the mapping early-returns, frames still flow, and timecodes silently
 * vanish.
 */
static int hal_sys_get_timestamp(void *ctx, int64_t *ts)
{
    hisi_state_t *st = hisi_state(ctx);
    unsigned long long pts = 0;

    if (!ts)
        return RSS_ERR_INVAL;
    if (!st || !st->sys.fnGetCurPts)
        return RSS_ERR_NOTSUP;

    if (st->sys.fnGetCurPts(&pts))
        return RSS_ERR_IO;

    *ts = (int64_t)pts;
    return RSS_OK;
}

static int hal_sys_rebase_timestamp(void *ctx, int64_t base)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !st->sys.fnInitPtsBase)
        return RSS_ERR_NOTSUP;

    if (st->sys.fnInitPtsBase((unsigned long long)base))
        return RSS_ERR_IO;

    return RSS_OK;
}

/* ================================================================
 * OPS VTABLE
 *
 * Only implemented ops are listed. Everything else stays NULL and resolves
 * to RSS_ERR_NOTSUP through RSS_HAL_CALL -- which is what makes a Phase 1
 * build useful rather than merely compilable: rvd starts, prints its
 * banner, finds no framesource and exits cleanly.
 *
 * The video pipeline (fs_*, enc_*, isp_*) landed in Phase 2, ISP tuning in
 * Phase 3, audio in Phase 4, OSD in Phase 5.
 * ================================================================ */

static const rss_hal_ops_t g_ops = {
    /* System lifecycle */
    .init = hal_init,
    .deinit = hal_deinit,
    .get_caps = hal_get_caps,

    /* System utilities */
    .sys_get_version = hal_sys_get_version,
    .sys_get_cpu_info = hal_sys_get_cpu_info,
    .sys_get_timestamp = hal_sys_get_timestamp,
    .sys_rebase_timestamp = hal_sys_rebase_timestamp,

#ifdef HAL_MODULE_VIDEO
    /* GPIO / IR-cut -- vendor-neutral sysfs, works as-is. The one part of
     * the video surface that owes nothing to MPP, which is why it is here
     * in Phase 1 and the rest is not. */
    .gpio_set = hal_gpio_set,
    .gpio_get = hal_gpio_get,
    .ircut_set = hal_ircut_set,

    /* Datapath. One edge, VPSS -> VENC. rvd's OSD stage collapses into it
     * (hisi_bind_collapse); the IVS stage does not exist on this backend. */
    .bind = hal_bind,
    .unbind = hal_unbind,

    /* Framesources -- VPSS channels. hal_framesource.c. */
    .fs_create_channel = hal_fs_create_channel,
    .fs_set_channel_attr = hal_fs_set_channel_attr,
    .fs_destroy_channel = hal_fs_destroy_channel,
    .fs_enable_channel = hal_fs_enable_channel,
    .fs_disable_channel = hal_fs_disable_channel,
    .fs_get_frame = hal_fs_get_frame,
    .fs_release_frame = hal_fs_release_frame,
    .fs_set_frame_depth = hal_fs_set_frame_depth,
    .fs_get_frame_depth = hal_fs_get_frame_depth,
    .fs_set_rotation = hal_fs_set_rotation,

    /* Encoders -- VENC channels. hal_encoder.c. */
    .enc_create_group = hal_enc_create_group,
    .enc_destroy_group = hal_enc_destroy_group,
    .enc_create_channel = hal_enc_create_channel,
    .enc_destroy_channel = hal_enc_destroy_channel,
    .enc_register_channel = hal_enc_register_channel,
    .enc_unregister_channel = hal_enc_unregister_channel,
    .enc_start = hal_enc_start,
    .enc_stop = hal_enc_stop,
    .enc_poll = hal_enc_poll,
    .enc_get_frame = hal_enc_get_frame,
    .enc_release_frame = hal_enc_release_frame,
    .enc_request_idr = hal_enc_request_idr,
    .enc_set_rc_mode = hal_enc_set_rc_mode,
    .enc_set_bitrate = hal_enc_set_bitrate,
    .enc_set_gop = hal_enc_set_gop,
    .enc_set_fps = hal_enc_set_fps,
    .enc_set_jpeg_qp = hal_enc_set_jpeg_qp,
    .enc_get_jpeg_qp = hal_enc_get_jpeg_qp,
    .enc_get_channel_attr = hal_enc_get_channel_attr,
    .enc_get_fps = hal_enc_get_fps,
    .enc_get_avg_bitrate = hal_enc_get_avg_bitrate,
    .enc_query = hal_enc_query,
    .enc_get_fd = hal_enc_get_fd,

    /* ISP -- geometry, rate, and the IQ tuning load. hal_isp.c. The knob
     * half of this surface (brightness, contrast, saturation and the
     * rest) is absent until hal_knob.c, and RSS_HAL_CALL turns that into
     * RSS_ERR_NOTSUP rather than a stub that lies. */
    .isp_get_sensor_attr = hal_isp_get_sensor_attr,
    .isp_set_sensor_fps = hal_isp_set_sensor_fps,
    .isp_get_sensor_fps = hal_isp_get_sensor_fps,
    .isp_get_exposure = hal_isp_get_exposure,
    .isp_set_brightness = hal_isp_set_brightness,
    .isp_get_brightness = hal_isp_get_brightness,
    .isp_set_contrast = hal_isp_set_contrast,
    .isp_get_contrast = hal_isp_get_contrast,
    .isp_set_saturation = hal_isp_set_saturation,
    .isp_set_running_mode = hal_isp_set_running_mode,
    .isp_get_running_mode = hal_isp_get_running_mode,
    .isp_get_saturation = hal_isp_get_saturation,
    .isp_set_ae_comp = hal_isp_set_ae_comp,
    .isp_get_ae_comp = hal_isp_get_ae_comp,
    .isp_set_drc_strength = hal_isp_set_drc_strength,
    .isp_get_drc_strength = hal_isp_get_drc_strength,
    .isp_get_knob_caps = hal_isp_get_knob_caps,
    .isp_set_hflip = hal_isp_set_hflip,
    .isp_set_vflip = hal_isp_set_vflip,
    .isp_get_hvflip = hal_isp_get_hvflip,

    /* OSD -- RGN overlays on a VENC channel. hal_osd.c, whose OP COVERAGE
     * block argues the five ops rvd never calls and the one region type
     * this silicon will not composite. */
    .osd_set_pool_size = hal_osd_set_pool_size,
    .osd_create_group = hal_osd_create_group,
    .osd_destroy_group = hal_osd_destroy_group,
    .osd_start = hal_osd_start,
    .osd_stop = hal_osd_stop,
    .osd_create_region = hal_osd_create_region,
    .osd_destroy_region = hal_osd_destroy_region,
    .osd_register_region = hal_osd_register_region,
    .osd_unregister_region = hal_osd_unregister_region,
    .osd_set_region_attr = hal_osd_set_region_attr,
    .osd_update_region_data = hal_osd_update_region_data,
    .osd_show_region = hal_osd_show_region,
#endif

#ifdef HAL_MODULE_AUDIO
    /* AI capture + the inner codec (src/hisi_v5/hal_audio.c). The OP
     * COVERAGE block there argues each absence -- VQE, AENC, AO. */
    .audio_init = hal_audio_init,
    .audio_deinit = hal_audio_deinit,
    .audio_read_frame = hal_audio_read_frame,
    .audio_release_frame = hal_audio_release_frame,
    .audio_set_volume = hal_audio_set_volume,
    .audio_get_volume = hal_audio_get_volume,
    .audio_set_gain = hal_audio_set_gain,
    .audio_get_gain = hal_audio_get_gain,
    .audio_set_mute = hal_audio_set_mute,
#endif
};

/* ================================================================
 * FACTORY FUNCTIONS
 * ================================================================ */

rss_hal_ctx_t *rss_hal_create(void)
{
    return rss_hal_create_backend("imp");
}

/*
 * rss_hal_create_backend -- pick a pipeline backend by name.
 *
 * This build carries one, and it is the vendor's: HiMPP is how a gen5 part
 * talks to its ISP and encoder at all. "imp" is the name the config's
 * default carries on every platform, so it means "the built-in one" here
 * rather than Ingenic's library. Any other name gets NULL, because a caller
 * that asked for a different pipeline is better told it is missing than
 * handed this one under its name.
 *
 * The backend-surface flags stay false in Phase 1 and are set as each
 * subsystem lands. That is load-bearing rather than cosmetic: a context
 * with has_framesource false gets no framesource created, so rvd starts,
 * reports what it cannot do, and exits cleanly instead of failing partway
 * into a pipeline that does not exist yet.
 */
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

#ifdef HAL_MODULE_VIDEO
    /*
     * The backend surface, as of Phase 2: there is a framesource and an
     * encoder, and the encoder can make JPEGs.
     *
     * These say what the *backend* has rather than what this board has --
     * whether the VENC symbols resolved is not known until hal_init, and
     * rvd reads caps before that. The honest report of a board without one
     * is the ops answering RSS_ERR_NOTSUP, which rvd already handles.
     *
     * has_osd is true from Phase 5, and it says the *backend* has an OSD
     * rather than that this board does: whether ss_mpi_rgn resolved is not
     * known until hal_init, and rvd reads caps before that. The honest
     * report of a board without RGN is the ops answering RSS_ERR_NOTSUP,
     * which rvd already handles -- rvd_osd.c takes a NOTSUP from
     * osd_create_region as "overlays disabled" and carries on.
     */
    ctx->caps.has_framesource = true;
    ctx->caps.has_jpeg = true;
    ctx->caps.has_osd = true;
#endif

    return ctx;
}

/*
 * rss_hal_destroy -- free a HAL context and internal resources.
 *
 * Does NOT call deinit() -- the caller must do that first.
 */
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
    if (!ctx)
        return NULL;

    return ctx->ops;
}

/* ================================================================
 * SYSTEM INFO (no vtable, called directly)
 * ================================================================ */

/*
 * rss_hal_get_imp_version / rss_hal_get_sysutils_version
 *
 * Both names are IMP-specific, but the daemons call them unconditionally to
 * print a build banner and pass no context. HiMPP's equivalent is
 * ss_mpi_sys_get_version, reached through g_hisi, so this answers only
 * after hal_init -- before that there is no loaded library to ask. There is
 * no sysutils equivalent at all, so that one is permanently unsupported.
 */
int rss_hal_get_imp_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    if (!g_hisi || !g_hisi->mpp_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)size, "%s", g_hisi->mpp_version);
    return RSS_OK;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_NOTSUP;

    return RSS_ERR_NOTSUP;
}

/*
 * rss_hal_get_cpu_info -- SoC identification string.
 *
 * Answers from /proc/umap/sys rather than /proc/cpuinfo, which on this
 * kernel reports "Hisilicon (Flattened Device Tree)" and names no part at
 * all. The caller treats the result as a borrowed static string, so the
 * value is cached after the first read; the part is not going to change.
 *
 * Deliberately independent of hal_init: rvd prints the banner before it
 * brings the pipeline up, and a platform check that only works afterwards
 * is a platform check that never runs on the build that needed it. That is
 * also why this reads the file itself instead of waiting for
 * st->chip_name -- though it prefers the state's copy when there is one,
 * so that the two can never disagree in a log.
 */
const char *rss_hal_get_cpu_info(void)
{
    static char cpu[24];
    static bool loaded = false;

    if (loaded)
        return cpu;

    loaded = true;

    if (g_hisi && g_hisi->chip_name[0]) {
        snprintf(cpu, sizeof(cpu), "%s", g_hisi->chip_name);
        return cpu;
    }

    if (hisi_read_chip_name(cpu, sizeof(cpu)) != RSS_OK)
        snprintf(cpu, sizeof(cpu), "%s", HAL_PLATFORM_NAME);

    return cpu;
}

const char *rss_hal_get_platform_name(void)
{
    return HAL_PLATFORM_NAME;
}

/*
 * rss_hal_check_platform -- verify the binary matches the running SoC.
 *
 * The same two-tier judgement gen4 makes, from a different source:
 *
 *   - a different hi3516cv6xx die. One MPP build serves the family -- the
 *     CV608 this backend was brought up on reports the *CV610* MPP version
 *     string -- so the binary is correct and only the caps numbers may be
 *     off. Warn, and keep running.
 *   - anything else. A gen4 or gen3 part has a different MPI ABI behind
 *     differently-named symbols; in practice the dlopen would fail first,
 *     but a part that somehow carried a V5 library set with a V4 kernel is
 *     exactly the mismatch that produces garbage rather than an error.
 *     Refuse.
 *
 * Unreadable is NOT a mismatch, and on V5 that case is ordinary rather than
 * exotic: /proc/umap/sys does not exist until load_hisilicon has modprobed
 * open_sys, and a daemon started before that -- or on a board where the
 * modules failed to load -- would be refused for a reason that has nothing
 * to do with the binary. Warn and continue; the failure will surface at
 * dlopen, where it can be named.
 */
void rss_hal_check_platform(const char *name)
{
    char chip[24];

    (void)name;

    if (hisi_read_chip_name(chip, sizeof(chip)) != RSS_OK) {
        HAL_LOG_WARN("platform check: %s unreadable, assuming %s", HISI_UMAP_SYS_PATH,
                     HAL_PLATFORM_NAME);
        return;
    }

    if (!hisi_chip_is_gen5(chip)) {
        /* Reported the way Ingenic reports its own fatal mismatch --
         * stderr and syslog both -- because rvd may already have detached
         * from the terminal by the time this runs, and a fatal nobody can
         * read is indistinguishable from a silent one. */
        fprintf(stderr,
                "FATAL: built for %s (HiMPP V5.0) but %s reports '%s', "
                "which is not a hi3516cv6xx part\n",
                HAL_PLATFORM_NAME, HISI_UMAP_SYS_PATH, chip);
        openlog(name ? name : "raptor", LOG_PID, LOG_DAEMON);
        syslog(LOG_ERR,
               "FATAL: built for %s (HiMPP V5.0) but %s reports '%s', "
               "which is not a hi3516cv6xx part",
               HAL_PLATFORM_NAME, HISI_UMAP_SYS_PATH, chip);
        closelog();
        _exit(1);
    }

    if (strcmp(chip, HISI_CHIP_HI3516CV608) != 0 && strcmp(chip, HISI_CHIP_HI3516CV610) != 0)
        HAL_LOG_WARN("platform check: %s reports '%s', a hi3516cv6xx die this build has not met. "
                     "One MPP serves the family, so the code is right; the capability numbers in "
                     "hal_caps.c may not be.",
                     HISI_UMAP_SYS_PATH, chip);
    else
        HAL_LOG_DBG("platform check: built for %s, running on %s", HAL_PLATFORM_NAME, chip);
}
