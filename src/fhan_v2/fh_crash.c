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

static void fh_crash_handler(int sig, siginfo_t *si, void *uc_v)
{
    char buf[512];
    int fd;
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
    }
#else
    (void)uc_v;
#endif
    fh_put("\n*** executable mappings:\n");

    fd = open("/proc/self/maps", O_RDONLY);
    if (fd >= 0) {
        /* Line filtering on a raw buffer: print lines whose permissions
         * field has an x. Lines split across reads are printed whole by
         * carrying the remainder. */
        char line[256];
        size_t ll = 0;

        while ((n = read(fd, buf, sizeof(buf))) > 0) {
            ssize_t i;

            for (i = 0; i < n; i++) {
                if (ll < sizeof(line) - 1)
                    line[ll++] = buf[i];
                if (buf[i] == '\n') {
                    char *sp = memchr(line, ' ', ll);

                    line[ll] = 0;
                    if (sp && sp + 3 < line + ll && sp[3] == 'x')
                        fh_put(line);
                    ll = 0;
                }
            }
        }
        close(fd);
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
