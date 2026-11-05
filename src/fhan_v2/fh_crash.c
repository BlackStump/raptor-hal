/*
 * fhan_v2/fh_crash.c -- say where a crash happened, then crash.
 *
 * The vendor libraries are stripped, run without a debugger on the board, and
 * a fault inside one leaves only "Segmentation fault". This handler prints
 * the faulting address, pc and lr, and the executable mappings from
 * /proc/self/maps, so the pc can be turned into library + offset on the host
 * (objdump, or addr2line on the unstripped rvd). Then it re-raises the signal
 * with the default action, so the process still dies the way it would have.
 *
 * Async-signal-safe throughout: write(), open(), read(), no stdio.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE

#ifndef HAL_MODULE_VIDEO
#define HAL_MODULE_VIDEO
#endif

#include "fh_state.h"

#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <ucontext.h>
#include <sys/syscall.h>
#include <unistd.h>

static void fh_put(const char *s)
{
    ssize_t r = write(2, s, strlen(s));

    (void)r;
}

static void fh_puthex(const char *label, unsigned long v)
{
    char b[32];
    int i;

    fh_put(label);
    b[0] = '0';
    b[1] = 'x';
    for (i = 0; i < 8; i++)
        b[2 + i] = "0123456789abcdef"[(v >> (28 - 4 * i)) & 0xf];
    b[10] = 0;
    fh_put(b);
}

#define FH_MAX_RANGES 40

static unsigned long fh_lo[FH_MAX_RANGES], fh_hi[FH_MAX_RANGES];
static char fh_names[FH_MAX_RANGES][48];
static int fh_nranges;

static unsigned long fh_parse_hex(const char *p, const char **end)
{
    unsigned long v = 0;

    for (;; p++) {
        char c = *p;

        if (c >= '0' && c <= '9')
            v = v * 16 + (unsigned long)(c - '0');
        else if (c >= 'a' && c <= 'f')
            v = v * 16 + (unsigned long)(c - 'a' + 10);
        else
            break;
    }
    *end = p;
    return v;
}

/* Remember an executable mapping and print it. */
static void fh_note_range(char *line)
{
    const char *p;
    unsigned long lo, hi;
    char *name;

    lo = fh_parse_hex(line, &p);
    if (*p != '-')
        return;
    hi = fh_parse_hex(p + 1, &p);
    fh_put(line);
    if (fh_nranges >= FH_MAX_RANGES)
        return;
    name = strrchr(line, '/');
    name = name ? name + 1 : strrchr(line, ' ');
    fh_lo[fh_nranges] = lo;
    fh_hi[fh_nranges] = hi;
    if (name) {
        size_t n = strlen(name);

        if (n && name[n - 1] == '\n')
            n--;
        if (n > sizeof(fh_names[0]) - 1)
            n = sizeof(fh_names[0]) - 1;
        memcpy(fh_names[fh_nranges], name, n);
        fh_names[fh_nranges][n] = 0;
    }
    fh_nranges++;
}

static void fh_crash_handler(int sig, siginfo_t *si, void *uc_v)
{
    char buf[512];
    unsigned long sp = 0;
    int fd, i, printed = 0;
    ssize_t n;

    fh_put("\n*** rvd: fatal signal ");
    fh_put(sig == SIGSEGV ? "SIGSEGV" : sig == SIGBUS ? "SIGBUS" : sig == SIGILL ? "SIGILL" : "?");
    fh_puthex(", fault addr ", (unsigned long)si->si_addr);
#if defined(__arm__)
    {
        ucontext_t *uc = (ucontext_t *)uc_v;

        fh_puthex(", pc ", uc->uc_mcontext.arm_pc);
        fh_puthex(", lr ", uc->uc_mcontext.arm_lr);
        fh_puthex(", sp ", uc->uc_mcontext.arm_sp);
        fh_puthex("\n*** r0 ", uc->uc_mcontext.arm_r0);
        fh_puthex(" r1 ", uc->uc_mcontext.arm_r1);
        fh_puthex(" r2 ", uc->uc_mcontext.arm_r2);
        fh_puthex(" r3 ", uc->uc_mcontext.arm_r3);
        sp = uc->uc_mcontext.arm_sp;
    }
#else
    (void)uc_v;
#endif
    fh_puthex(", tid ", (unsigned long)syscall(SYS_gettid));
    fh_put("\n*** executable mappings:\n");

    fd = open("/proc/self/maps", O_RDONLY);
    if (fd >= 0) {
        char line[256];
        size_t ll = 0;

        while ((n = read(fd, buf, sizeof(buf))) > 0) {
            ssize_t k;

            for (k = 0; k < n; k++) {
                if (ll < sizeof(line) - 1)
                    line[ll++] = buf[k];
                if (buf[k] == '\n') {
                    char *sp_ = memchr(line, ' ', ll);

                    line[ll] = 0;
                    if (sp_ && sp_ + 3 < line + ll && sp_[3] == 'x')
                        fh_note_range(line);
                    ll = 0;
                }
            }
        }
        close(fd);
    }

    /*
     * No unwinder: lr was clobbered by the copy routine, so walk the stack
     * and print every word that points into executable code. The first few
     * are the return addresses of the frames that led here.
     */
    if (sp) {
        fh_put("*** code addresses on the stack (sp+offset: library+offset):\n");
        for (i = 0; i < 2048 && printed < 24; i++) {
            unsigned long v = ((unsigned long *)sp)[i];
            int r;

            for (r = 0; r < fh_nranges; r++) {
                if (v >= fh_lo[r] && v < fh_hi[r]) {
                    fh_puthex("    sp+", (unsigned long)i * 4);
                    fh_put(": ");
                    fh_put(fh_names[r]);
                    fh_puthex("+", v - fh_lo[r]);
                    fh_put("\n");
                    printed++;
                    break;
                }
            }
        }
    }

    signal(sig, SIG_DFL);
    raise(sig);
}

void fh_crash_install(void)
{
    struct sigaction sa;
    static const int sigs[] = {SIGSEGV, SIGBUS, SIGILL};
    size_t i;

    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = fh_crash_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESETHAND;
    sigemptyset(&sa.sa_mask);
    for (i = 0; i < sizeof(sigs) / sizeof(sigs[0]); i++)
        sigaction(sigs[i], &sa, NULL);
}
