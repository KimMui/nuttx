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
 * @author Mark Greer
 * @brief Pseudo DMA driver that uses memcpy instead of real DMA
 */

#ifndef __TSB_I2S_H
#define __TSB_I2S_H

#define TSB_I2S_DRIVER_NAME                             "tsb i2s driver"

/*
 * Limit register polling loops so they don't go forever.  1000 should be
 * large enough to handle any case where something isn't seriously wrong.
 */
#define TSB_I2S_POLL_LIMIT                              1000

#define TSB_I2S_PLLA_ID                                 0

#define TSB_I2S_TX_START_THRESHOLD                      0x20

/* System Controller/Bridge Registers */
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR                0x0440

#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL   BIT(2)
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL    BIT(1)
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASTER_CLOCK_SEL BIT(0)
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASK                                 \
                        (TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL      | \
                         TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL       | \
                         TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASTER_CLOCK_SEL)

/* I2S Registers */
#define TSB_I2S_REG_START                               0x0004
#define TSB_I2S_REG_BUSY                                0x0008
#define TSB_I2S_REG_STOP                                0x000c
#define TSB_I2S_REG_AUDIOSET                            0x0010
#define TSB_I2S_REG_INTSTAT                             0x0014
#define TSB_I2S_REG_INTMASK                             0x0018
#define TSB_I2S_REG_INTCLR                              0x001c
#define TSB_I2S_REG_MUTE                                0x0024
#define TSB_I2S_REG_EPTR                                0x0028
#define TSB_I2S_REG_TX_SSIZE                            0x0030
#define TSB_I2S_REG_REGBUSY                             0x0040
#define TSB_I2S_REG_MODESET                             0x00f8
#define TSB_I2S_REG_LMEM00                              0x0100

#define TSB_I2S_REG_START_START                         BIT(8)
#define TSB_I2S_REG_START_SPK_MIC_START                 BIT(0)

#define TSB_I2S_REG_BUSY_LRERRBUSY                      BIT(17)
#define TSB_I2S_REG_BUSY_ERRBUSY                        BIT(16)
#define TSB_I2S_REG_BUSY_BUSY                           BIT(8)
#define TSB_I2S_REG_BUSY_SERIBUSY                       BIT(1)
#define TSB_I2S_REG_BUSY_SPK_MIC_BUSY                   BIT(0)

#define TSB_I2S_REG_STOP_I2S_STOP                       BIT(0)

#define TSB_I2S_REG_AUDIOSET_DTFMT                      BIT(16)
#define TSB_I2S_REG_AUDIOSET_SDEDGE                     BIT(12)
#define TSB_I2S_REG_AUDIOSET_EDGE                       BIT(11)
#define TSB_I2S_REG_AUDIOSET_SCLKTOWS                   BIT(8)
#define TSB_I2S_REG_AUDIOSET_WORDLEN_MASK               0x3f
#define TSB_I2S_REG_AUDIOSET_MASK           (TSB_I2S_REG_AUDIOSET_DTFMT     | \
                                             TSB_I2S_REG_AUDIOSET_SDEDGE    | \
                                             TSB_I2S_REG_AUDIOSET_EDGE      | \
                                             TSB_I2S_REG_AUDIOSET_SCLKTOWS  | \
                                             TSB_I2S_REG_AUDIOSET_WORDLEN_MASK)

#define TSB_I2S_REG_INT_DMACMSK                         BIT(16)
#define TSB_I2S_REG_INT_LRCK                            BIT(3)
#define TSB_I2S_REG_INT_UR                              BIT(2)
#define TSB_I2S_REG_INT_OR                              BIT(1)
#define TSB_I2S_REG_INT_INT                             BIT(0)
#define TSB_I2S_REG_INT_ERROR_MASK          (TSB_I2S_REG_INT_LRCK   | \
                                             TSB_I2S_REG_INT_UR     | \
                                             TSB_I2S_REG_INT_OR)

#define TSB_I2S_REG_MUTE_MUTEN                          BIT(0)

#define TSB_I2S_REG_EPTR_ERRPOINTER_GET(a)              ((a) & 0x3f)
#define TSB_I2S_REG_EPTR_ERRPOINTER_SET(a)              ((a) & 0x3f)

#define TSB_I2S_REG_TX_SSIZE_TXSTARTSIZE_GET(r)         ((r) & 0x3f)
#define TSB_I2S_REG_TX_SSIZE_TXSTARTSIZE_SET(v)         ((v) & 0x3f)

#define TSB_I2S_REG_REGBUSY_MODESETPEND                 BIT(19)
#define TSB_I2S_REG_REGBUSY_TXSSIZEPEND                 BIT(18)
#define TSB_I2S_REG_REGBUSY_MUTEPEND                    BIT(17)
#define TSB_I2S_REG_REGBUSY_AUDIOSETPEND                BIT(16)
#define TSB_I2S_REG_REGBUSY_MODESETBUSY                 BIT(3)
#define TSB_I2S_REG_REGBUSY_TXSSIZEBUSY                 BIT(2)
#define TSB_I2S_REG_REGBUSY_MUTEBUSY                    BIT(1)
#define TSB_I2S_REG_REGBUSY_AUDIOSETBUSY                BIT(0)

#define TSB_I2S_REG_MODESET_I2S_STEREO                  0x0
#define TSB_I2S_REG_MODESET_LR_STEREO                   0x2
#define TSB_I2S_REG_MODESET_LR_STEREO_REV_POL           0x3
#define TSB_I2S_REG_MODESET_PCM_MONO                    0x4
#define TSB_I2S_REG_MODESET_PCM_MONO_REV_POL            0x5
#define TSB_I2S_REG_MODESET_WS_MASK                     0x7

#define TSB_I2S_FLAG_OPEN                               BIT(0)
#define TSB_I2S_FLAG_CONFIGURED                         BIT(1)
#define TSB_I2S_FLAG_ENABLED                            BIT(2)
#define TSB_I2S_FLAG_RX_PREPARED                        BIT(3)
#define TSB_I2S_FLAG_RX_ACTIVE                          BIT(4)
#define TSB_I2S_FLAG_TX_PREPARED                        BIT(5)
#define TSB_I2S_FLAG_TX_ACTIVE                          BIT(6)

enum tsb_i2s_block {
    TSB_I2S_BLOCK_INVALID,
    TSB_I2S_BLOCK_BRIDGE,
    TSB_I2S_BLOCK_SC,
    TSB_I2S_BLOCK_SO,
    TSB_I2S_BLOCK_SI,
};

struct tsb_i2s_info {
    struct device                   *dev;
    struct device                   *pll_dev;
    sem_t                           lock;
    uint32_t                        flags;
    struct ring_buf                 *rx_rb;
    device_i2s_callback             rx_callback;
    void                            *rx_arg;
    struct ring_buf                 *tx_rb;
    device_i2s_callback             tx_callback;
    void                            *tx_arg;
    uint32_t                        cg_base;
    uint32_t                        sc_base;
    uint32_t                        so_base;
    uint32_t                        si_base;
    int                             soerr_irq;
    int                             so_irq;
    int                             sierr_irq;
    int                             si_irq;
    struct device_i2s_configuration config;
};

/* Utility functions used by the data movement routines */
uint32_t tsb_i2s_read_raw(struct tsb_i2s_info *info,
                          enum tsb_i2s_block block, unsigned int reg);
uint32_t tsb_i2s_read(struct tsb_i2s_info *info,
                      enum tsb_i2s_block block, unsigned int reg);
void tsb_i2s_write_raw(struct tsb_i2s_info *info,
                       enum tsb_i2s_block block,
                       unsigned int reg, uint32_t val);
void tsb_i2s_clear_irqs(struct tsb_i2s_info *info,
                        enum tsb_i2s_block block, uint32_t mask);
void tsb_i2s_mask_irqs(struct tsb_i2s_info *info,
                       enum tsb_i2s_block block, uint32_t mask);
void tsb_i2s_unmask_irqs(struct tsb_i2s_info *info,
                         enum tsb_i2s_block block, uint32_t mask);
void tsb_i2s_stop_receiver(struct tsb_i2s_info *info, int is_err);
void tsb_i2s_stop_transmitter(struct tsb_i2s_info *info, int is_err);

enum device_i2s_event tsb_i2s_intstat2event(uint32_t intstat);


/* Data movement routines provided by tsb_i2s_xfer,c or tsb_i2s_xfer_dma.c */
int tsb_attach_xfer_irqs(struct tsb_i2s_info *info);
int tsb_detach_xfer_irqs(struct tsb_i2s_info *info);
int tsb_i2s_rx_data(struct tsb_i2s_info *info);
int tsb_i2s_tx_data(struct tsb_i2s_info *info);


/*
 * The nuttx IRQ handler interface doesn't provide an 'arg' parameter
 * where 'dev' can be passed so use a global for now.  Must be fixed to
 * support more than one tsb i2s controller.
 */
struct device *saved_dev;

#endif /* __TSB_I2S_H */
