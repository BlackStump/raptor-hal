/*
 * xiongmai/xm_state.h -- live backend state for the Xiongmai backend.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef XM_STATE_H
#define XM_STATE_H

#include "xm_common.h"

typedef struct {
    bool sys_inited;

    char chip_name[24];
    char sdk_version[64];
    char sensor_name[64];
} xm_state_t;

static inline xm_state_t *xm_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (xm_state_t *)c->platform : NULL;
}

#endif /* XM_STATE_H */
