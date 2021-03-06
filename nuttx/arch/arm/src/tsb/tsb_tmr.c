/*
 * Copyright (c) 2015 Google, Inc.
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
 * may be used to endorse or promote products derived from this
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
 * Author: Bartosz Golaszewski <bgolaszewski@baylibre.com>
 */

#include "tsb_tmr.h"
#include "chip.h"
#include "tsb_scm.h"
#include "up_arch.h"

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/util.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#define TSB_TMR_COUNTER_MAX             2880000000U
#define TSB_TMR_CNT_MAX_USEC            (USEC_PER_SEC * 60)

#define TSB_TMR_LOAD                    (0x0000)
#define TSB_TMR_CNT                     (0x0004)
#define TSB_TMR_CTRL                    (0x0008)
#define TSB_TMR_ISR                     (0x000C)
#define TSB_TMR_WDT                     (0x0040)
#define TSB_TMR_RSR                     (0x0080)

#define TSB_TMR1_OFF                    (0x0100)
#define TSB_TMR2_OFF                    (0x0200)
#define TSB_TMR3_OFF                    (0x0300)
#define TSB_TMR4_OFF                    (0x0400)

#define TSB_TMR_CLOCK_ENABLE_REG        (0x40000300)
#define TSB_TMR_SOFT_RESET_REL_REG      (0x40000100)

#define TSB_TMR_BUS_CLOCK_SUPPLY        BIT(16)
#define TSB_TMR_TIMER_CLOCK_SUPPLY      BIT(15)
#define TSB_TMR_RESET_DEASSERT          (0x00003800)

#define TSB_TMR_CTRL_IRQ_ENABLE         BIT(2)

struct tsb_tmr_ctx {
    const uint32_t base;
    int mode;
    uint32_t usec;
    const int irq;
};

struct tsb_tmr_ctx tsb_timers[] = {
    {
        .base = TMR_BASE + TSB_TMR1_OFF,
        .irq = TSB_IRQ_TMR1,
    },
    {
        .base = TMR_BASE + TSB_TMR2_OFF,
        .irq = TSB_IRQ_TMR2,
    },
    {
        .base = TMR_BASE + TSB_TMR3_OFF,
        .irq = TSB_IRQ_TMR3,
    },
    {
        .base = TMR_BASE + TSB_TMR4_OFF,
        .irq = TSB_IRQ_TMR4,
    },
};

static pthread_once_t tsb_tmr_once = PTHREAD_ONCE_INIT;

static void tsb_tmr_global_init(void)
{
    irqstate_t flags;

    flags = irqsave();

    tsb_clk_enable(TSB_CLK_TMR);
    tsb_reset(TSB_RST_TMR);

    /* Supply clock for the bus (but not for the timers yet). */
    putreg32(TSB_TMR_BUS_CLOCK_SUPPLY, TSB_TMR_CLOCK_ENABLE_REG);
    /* Reset deassert of the whole clock module. */
    putreg32(TSB_TMR_RESET_DEASSERT, TSB_TMR_SOFT_RESET_REL_REG);

    irqrestore(flags);
}

static void tsb_tmr_putreg32(struct tsb_tmr_ctx *tmr,
                             uint32_t val, uint32_t reg)
{
    putreg32(val, tmr->base + reg);
}

static uint32_t tsb_tmr_getreg32(struct tsb_tmr_ctx *tmr, uint32_t reg)
{
    return getreg32(tmr->base + reg);
}

static uint32_t tsb_tmr_usec_to_freq(uint32_t usec)
{
    uint64_t ratio;

    ratio = usec;
    ratio *= TSB_TMR_COUNTER_MAX;
    ratio = DIV_ROUND_CLOSEST(ratio, TSB_TMR_CNT_MAX_USEC);
    ASSERT(ratio <= UINT32_MAX);

    return (uint32_t)ratio;
}

static uint32_t tsb_tmr_freq_to_usec(uint32_t freq)
{
    uint64_t ratio;

    ratio = freq;
    ratio *= TSB_TMR_CNT_MAX_USEC;
    ratio = ratio / TSB_TMR_COUNTER_MAX;
    ASSERT(ratio <= UINT32_MAX);

    return (uint32_t)ratio;
}

/**
 * @brief Get a handle to a timer object.
 * @param id Timer identifier - must be a valid timer identifier (as defined
 *           in tsb_tmr.h).
 * @return Pointer to a static timer object or NULL for invalid id.
 */
struct tsb_tmr_ctx * tsb_tmr_get(int id)
{
    if (id < 0 || id >= ARRAY_SIZE(tsb_timers))
        return NULL;

    pthread_once(&tsb_tmr_once, tsb_tmr_global_init);

    return &tsb_timers[id];
}

/**
 * @brief Configure timer.
 * @param tmr Timer handle.
 * @param mode Timer mode (free-run, periodic or one-shot).
 * @param isr Timer interrupt handler. Can be NULL if no interrupt handling is
 *            required. The handler function must call tsb_tmr_ack_irq().
 */
void tsb_tmr_configure(struct tsb_tmr_ctx *tmr, int mode, xcpt_t isr)
{
    ASSERT(tmr);

    switch (mode) {
    case TSB_TMR_MODE_PERIODIC:
    case TSB_TMR_MODE_ONESHOT:
    case TSB_TMR_MODE_FREERUN:
        tmr->mode = mode;
        break;
    default:
        ASSERT(0);
    }

    if (isr) {
        irq_attach(tmr->irq, isr);
        up_enable_irq(tmr->irq);
    }

    /* Supply the timer clock to the timer. */
    putreg32(TSB_TMR_TIMER_CLOCK_SUPPLY, TSB_TMR_CLOCK_ENABLE_REG);
}

/**
 * @brief Set timer period (in microseconds).
 * @param tmr Timer handle.
 * @param usec Number of microseconds per timer period. Must be less or equal
 *             to one minute expressed in microseconds.
 */
void tsb_tmr_set_time(struct tsb_tmr_ctx *tmr, uint32_t usec)
{
    ASSERT(usec <= TSB_TMR_CNT_MAX_USEC);
    tmr->usec = usec;
}

/**
 * @brief Start the timer.
 * @param tmr Timer handle.
 */
void tsb_tmr_start(struct tsb_tmr_ctx *tmr)
{
    irqstate_t flags;

    flags = irqsave();

    tsb_tmr_putreg32(tmr, 0x00, TSB_TMR_LOAD);
    tsb_tmr_putreg32(tmr, tmr->mode, TSB_TMR_CTRL);
    tsb_tmr_putreg32(tmr, tsb_tmr_usec_to_freq(tmr->usec), TSB_TMR_LOAD);
    tsb_tmr_putreg32(tmr, TSB_TMR_CTRL_IRQ_ENABLE | tmr->mode, TSB_TMR_CTRL);

    irqrestore(flags);
}

/**
 * @brief Cancel and deactivate timer.
 * @param tmr Timer handle.
 * @return Number of microseconds still left until interrupt in freerunning
 *         mode or 0 for other modes.
 */
uint32_t tsb_tmr_cancel(struct tsb_tmr_ctx *tmr)
{
    irqstate_t flags;
    uint32_t counter;

    flags = irqsave();

    counter = tmr->mode == TSB_TMR_MODE_FREERUN ?
                                tsb_tmr_getreg32(tmr, TSB_TMR_CNT) : 0;
    tsb_tmr_putreg32(tmr, 0x00, TSB_TMR_CTRL);

    irqrestore(flags);

    return tsb_tmr_freq_to_usec(counter);
}

/**
 * @brief Read the counter register of a timer.
 * @param tmr Timer handle.
 * @return Number of microseconds still left until interrupt.
 *
 * Only works in free-running mode.
 */
uint32_t tsb_tmr_usec_left(struct tsb_tmr_ctx *tmr)
{
    irqstate_t flags;
    uint32_t counter;

    ASSERT(tmr->mode == TSB_TMR_MODE_FREERUN);

    flags = irqsave();
    counter = tsb_tmr_getreg32(tmr, TSB_TMR_CNT);
    irqrestore(flags);

    return tsb_tmr_freq_to_usec(counter);
}

/**
 * @brief Clear the interrupt bit from timer's interrupt register.
 * @param tmr Timer handle.
 */
void tsb_tmr_ack_irq(struct tsb_tmr_ctx *tmr)
{
    tsb_tmr_putreg32(tmr, 0x01, TSB_TMR_ISR);
}
