/**
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Kim Mui
 * @brief ATABL driver that manipulates the ES3's Accelerator Table
 */

#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/device.h>
#include <nuttx/device_atabl.h>
#include <nuttx/list.h>

#include "debug.h"
#include "up_arch.h"

#define NO_ATABL_HW_ACCESS

#define MAX_ATABL_REQN          16
#define GDMAC_PERIPHERAL_OFFSET 16
#define MAX_SUPPORTED_CPORT     40
#define MAX_ATABL_DEBUG_REG     4

struct tsb_atabl_regs {
    uint32_t gdmac_reqn_to_cp[MAX_ATABL_REQN];
    uint32_t admac_reqn_to_cp[MAX_ATABL_REQN];
    uint32_t summary_reserve_status;
    uint32_t pad0[31];
    uint32_t gdmac_reqn_start[MAX_ATABL_REQN];
    uint32_t admac_reqn_start[MAX_ATABL_REQN];
    uint32_t summary_req_start;
    uint32_t pad1[31];
    uint32_t cpn_to_reqn[MAX_SUPPORTED_CPORT];
    uint32_t debug_reg[MAX_ATABL_DEBUG_REG];
};

enum tsb_atabl_req_state {
    TSB_ATABL_REQ_STATE_FREE,
    TSB_ATABL_REQ_STATE_ALLOCATED,
    TSB_ATABL_REQ_STATE_CONNECTED,
//    TSB_ATABL_REQ_STATE_STOP,
    TSB_ATABL_REQ_STATE_RUNNING,
    TSB_ATABL_REQ_STATE_ERROR,
    TSB_ATABL_REQ_STATE_UNDEFINED
};

struct tsb_atabl_reqn {
    uint32_t req_id;
    enum tsb_atabl_req_state state;
};

/* structure for GDMAC device driver's private info. */
struct tsb_atabl_driver_info {
    pthread_mutex_t driver_mutext;
    uint32_t reg_base;

    int avail_reqn;
    struct tsb_atabl_reqn reqn[MAX_ATABL_REQN];
};

static int tsb_atabl_req_free_count(struct device *dev)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);

    return info->avail_reqn;
}

static int tsb_atabl_req_alloc(struct device *dev, void **req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    int retval = DEVICE_ATABL_ERROR_NO_REQ_AVAIL;
    int index;

    if (info == NULL) {
        return DEVICE_ATABL_ERROR_BAD_DEV;
    }

    pthread_mutex_lock(&info->driver_mutext);

    for (index = 0; index < MAX_ATABL_REQN; index++) {
        if (info->reqn[index].state == TSB_ATABL_REQ_STATE_FREE) {
            info->reqn[index].state = TSB_ATABL_REQ_STATE_ALLOCATED;

            *req = &info->reqn[index];
            info->avail_reqn--;

            retval = DEVICE_ATABL_ERROR_NONE;
            break;
        }
    }

    pthread_mutex_unlock(&info->driver_mutext);

    return retval;
}


static inline int tsb_atabl_validate_req(struct tsb_atabl_driver_info *info,
                                         struct tsb_atabl_reqn *reqn)
{
    if ((reqn == NULL) ||
        (reqn->req_id < GDMAC_PERIPHERAL_OFFSET) ||
        (reqn->req_id >= (GDMAC_PERIPHERAL_OFFSET + MAX_ATABL_REQN)) ||
        (reqn != &info->reqn[reqn->req_id - GDMAC_PERIPHERAL_OFFSET])) {
        return DEVICE_ATABL_ERROR_BAD_REQ;
    }

    return DEVICE_ATABL_ERROR_NONE;
}

static int tsb_atabl_req_free(struct device *dev, void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    int retval = DEVICE_ATABL_ERROR_BAD_DEV;
    struct tsb_atabl_reqn *reqn = req;

    if (info == NULL) {
        return retval;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return retval;
    }

    switch(reqn->state) {
        case TSB_ATABL_REQ_STATE_ALLOCATED:
            pthread_mutex_lock(&info->driver_mutext);

            reqn->state = TSB_ATABL_REQ_STATE_FREE;
            info->avail_reqn++;

            pthread_mutex_unlock(&info->driver_mutext);

            retval = DEVICE_ATABL_ERROR_NONE;
            break;
        case TSB_ATABL_REQ_STATE_FREE:
            retval = DEVICE_ATABL_ERROR_REQ_INVALID_STATE;
            break;
        default:
            retval = DEVICE_ATABL_ERROR_REQ_BUSY;
            break;
    }

    return retval;
}


static int tsb_atabl_connect(struct device *dev, unsigned int cportid,
                            void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    int retval = DEVICE_ATABL_ERROR_BAD_DEV;
    struct tsb_atabl_reqn *reqn = req;
    struct tsb_atabl_regs *atabl_regs;

    if (info == NULL) {
        return retval;
    }

    if (cportid >= MAX_SUPPORTED_CPORT) {
        return DEVICE_ATABL_ERROR_BAD_CPORTID;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return retval;
    }

    switch(reqn->state) {
        case TSB_ATABL_REQ_STATE_ALLOCATED:
            atabl_regs = (struct tsb_atabl_regs *)info->reg_base;
#ifndef NO_ATABL_HW_ACCESS
            atabl_regs->gdmac_reqn_to_cp[reqn->req_id] = 0x10000 | cportid;
#endif
            reqn->state = TSB_ATABL_REQ_STATE_CONNECTED;
            retval = DEVICE_ATABL_ERROR_NONE;
            break;
        case TSB_ATABL_REQ_STATE_FREE:
            retval = DEVICE_ATABL_ERROR_REQ_INVALID_STATE;
            break;
        default:
            retval = DEVICE_ATABL_ERROR_REQ_BUSY;
            break;
    }

    return retval;
}


static int tsb_atabl_disconnect(struct device *dev, void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    int retval = DEVICE_ATABL_ERROR_BAD_REQ;
    struct tsb_atabl_reqn *reqn = req;
    struct tsb_atabl_regs *atabl_regs;

    if (info == NULL) {
        return retval;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return retval;
    }

    switch(reqn->state) {
        case TSB_ATABL_REQ_STATE_CONNECTED:
            atabl_regs = (struct tsb_atabl_regs *)info->reg_base;

#ifndef NO_ATABL_HW_ACCESS
            atabl_regs->gdmac_reqn_to_cp[reqn->req_id] = 0;
#endif
            reqn->state = TSB_ATABL_REQ_STATE_ALLOCATED;
            retval = DEVICE_ATABL_ERROR_NONE;
            break;
        case TSB_ATABL_REQ_STATE_FREE:
        case TSB_ATABL_REQ_STATE_ALLOCATED:
            retval = DEVICE_ATABL_ERROR_REQ_INVALID_STATE;
            break;
        default:
            retval = DEVICE_ATABL_ERROR_BAD_REQ;
           break;
    }

    return retval;
}


static int tsb_atabl_activate(struct device *dev, void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    int retval = DEVICE_ATABL_ERROR_BAD_REQ;
    struct tsb_atabl_reqn *reqn = req;
    struct tsb_atabl_regs *atabl_regs;

    if (info == NULL) {
        return retval;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return retval;
    }

    switch(reqn->state) {
        case TSB_ATABL_REQ_STATE_CONNECTED:
            atabl_regs = (struct tsb_atabl_regs *)info->reg_base;

#ifndef NO_ATABL_HW_ACCESS
            atabl_regs->gdmac_reqn_start[reqn->req_id] = 1;
#endif
            reqn->state = TSB_ATABL_REQ_STATE_RUNNING;
            retval = DEVICE_ATABL_ERROR_NONE;
            break;
        case TSB_ATABL_REQ_STATE_FREE:
        case TSB_ATABL_REQ_STATE_ALLOCATED:
        case TSB_ATABL_REQ_STATE_RUNNING:
            retval = DEVICE_ATABL_ERROR_REQ_INVALID_STATE;
            break;
        default:
            retval = DEVICE_ATABL_ERROR_BAD_REQ;
           break;
    }

    return retval;
}


static int tsb_atabl_deactivate(struct device *dev, void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    int retval = DEVICE_ATABL_ERROR_BAD_REQ;
    struct tsb_atabl_reqn *reqn = req;
    struct tsb_atabl_regs *atabl_regs;

    if (info == NULL) {
        return retval;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return retval;
    }

    switch(reqn->state) {
        case TSB_ATABL_REQ_STATE_RUNNING:
            atabl_regs = (struct tsb_atabl_regs *)info->reg_base;

#ifndef NO_ATABL_HW_ACCESS
            atabl_regs->gdmac_reqn_start[reqn->req_id] = 0;
#endif
            reqn->state = TSB_ATABL_REQ_STATE_ALLOCATED;
            retval = DEVICE_ATABL_ERROR_NONE;
            break;
        case TSB_ATABL_REQ_STATE_CONNECTED:
            reqn->state = TSB_ATABL_REQ_STATE_ALLOCATED;
            retval = DEVICE_ATABL_ERROR_NONE;
            break;
        case TSB_ATABL_REQ_STATE_FREE:
        case TSB_ATABL_REQ_STATE_ALLOCATED:
            retval = DEVICE_ATABL_ERROR_REQ_INVALID_STATE;
            break;
        default:
            retval = DEVICE_ATABL_ERROR_BAD_REQ;
           break;
    }

    return retval;
}


static int tsb_atabl_transfer_completed(struct device *dev, void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    int retval = DEVICE_ATABL_ERROR_BAD_DEV;
    struct tsb_atabl_reqn *reqn = req;
    struct tsb_atabl_regs *atabl_regs;

    if (info == NULL) {
        return retval;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return retval;
    }

    switch (reqn->state) {
        case TSB_ATABL_REQ_STATE_RUNNING:
            atabl_regs = (struct tsb_atabl_regs *)info->reg_base;

#ifndef NO_ATABL_HW_ACCESS
            if ((atabl_regs->gdmac_reqn_start[reqn->req_id] & 0x01) == 0) {
                reqn->state = TSB_ATABL_REQ_STATE_CONNECTED;
                retval = DEVICE_ATABL_ERROR_NONE;
            } else {
                retval = DEVICE_ATABL_ERROR_REQ_INVALID_STATE;
            }
#else
            reqn->state = TSB_ATABL_REQ_STATE_CONNECTED;
            retval = DEVICE_ATABL_ERROR_NONE;
#endif
            break;
        case TSB_ATABL_REQ_STATE_FREE:
        case TSB_ATABL_REQ_STATE_ALLOCATED:
        case TSB_ATABL_REQ_STATE_CONNECTED:
            retval = DEVICE_ATABL_ERROR_REQ_INVALID_STATE;
            break;
        default:
            retval = DEVICE_ATABL_ERROR_BAD_REQ;
            break;
    }

    return retval;
}


static int tsb_atabl_req_is_activated(struct device *dev, void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    struct tsb_atabl_reqn *reqn = req;
    int retval;
    struct tsb_atabl_regs *atabl_regs;

    if (info == NULL) {
        return -EINVAL;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return -EINVAL;
    }

    atabl_regs = (struct tsb_atabl_regs *)info->reg_base;
#ifndef NO_ATABL_HW_ACCESS
    if (reqn->state != TSB_ATABL_REQ_STATE_RUNNING) {
        if ((atabl_regs->gdmac_reqn_start[reqn->req_id] & 0x01) != 0) {
            lldbg("SW internal state doesn't match with HW.\n");
        }
        return 0;
    }

    if ((atabl_regs->gdmac_reqn_start[reqn->req_id] & 0x01) == 0) {
        reqn->state = TSB_ATABL_REQ_STATE_RUNNING;
        return 0;
    }
#else
    if (reqn->state != TSB_ATABL_REQ_STATE_RUNNING) {
        return 0;
    }
#endif

    return 1;
}

static int tsb_atabl_req_to_peripheral_id(struct device *dev, void *req)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);
    struct tsb_atabl_reqn *reqn = req;
    int retval;

    if (info == NULL) {
        return -EINVAL;
    }

    retval = tsb_atabl_validate_req(info, reqn);
    if (retval != DEVICE_ATABL_ERROR_NONE) {
        return -EINVAL;
    }

    return reqn->req_id;
}

static int tsb_atabl_extract_resources(struct device *dev,
                                       struct tsb_atabl_driver_info * info)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOURCE_TYPE_REGS, "reg_base");
    if (!r) {
        return -EINVAL;
    }
    info->reg_base = r->start;

    return OK;
}

static int tsb_atabl_open(struct device *dev)
{
    struct tsb_atabl_driver_info *info;
    int retval;
    int index;

    info = zalloc(sizeof(struct tsb_atabl_driver_info));
    if (!info) {
        return -ENOMEM;
    }

    retval = tsb_atabl_extract_resources(dev, info);
    if (retval != OK) {
        free(info);

        return retval;
    }

    for (index = 0; index < MAX_ATABL_REQN; index++) {
        info->reqn[index].req_id = GDMAC_PERIPHERAL_OFFSET + index;
        info->reqn[index].state = TSB_ATABL_REQ_STATE_FREE;
    }
    info->avail_reqn = MAX_ATABL_REQN;

    pthread_mutex_init(&info->driver_mutext, NULL);
    pthread_mutex_unlock(&info->driver_mutext);

    device_set_private(dev, info);

    return DEVICE_ATABL_ERROR_NONE;
}

static void tsb_atabl_close(struct device *dev)
{
    struct tsb_atabl_driver_info *info = device_get_private(dev);

    if (!info) {
        return;
    }

    device_set_private(dev, NULL);

    free(info);

    return;
}

static struct device_atabl_type_ops tsb_atabl_type_ops = {
    .req_free_count = tsb_atabl_req_free_count,
    .req_alloc = tsb_atabl_req_alloc,
    .req_free = tsb_atabl_req_free,
    .connect_cport_to_req = tsb_atabl_connect,
    .disconnect_cport_from_req = tsb_atabl_disconnect,
    .activate_req = tsb_atabl_activate,
    .deactivate_req = tsb_atabl_deactivate,
    .transfer_completed = tsb_atabl_transfer_completed,
    .req_is_activated = tsb_atabl_req_is_activated,
    .req_to_peripheral_id = tsb_atabl_req_to_peripheral_id,
};

static struct device_driver_ops tsb_atabl_driver_ops = {
        .open = tsb_atabl_open,
        .close = tsb_atabl_close,
        .type_ops = &tsb_atabl_type_ops
};

struct device_driver tsb_atabl_driver = {
        .type = DEVICE_TYPE_ATABL_HW,
        .name = "tsb_atabl",
        .desc = "TSB ATABL Driver",
        .ops = &tsb_atabl_driver_ops
};
