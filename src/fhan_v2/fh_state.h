/*
 * fhan_v2/fh_state.h -- live backend state for the Fullhan backend.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef FH_STATE_H
#define FH_STATE_H

#include "fh_common.h"

typedef struct {
    void *h_vmm;
    void *h_dsp;

    int (*fnSysInit)(void);
    int (*fnSysExit)(void);
    int (*fnSysGetVersion)(void *ver);
    int (*fnSysGetChipId)(void *id); /* optional */
} fh_sys_impl;

typedef struct {
    fh_sys_impl sys;
    bool sys_inited;

    char chip_name[24];
    char sdk_version[64];
    char sensor_name[64];
} fh_state_t;

static inline fh_state_t *fh_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (fh_state_t *)c->platform : NULL;
}

#endif /* FH_STATE_H */
