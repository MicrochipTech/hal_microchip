/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_QSPI_API_H
#define _MEC_QSPI_API_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mec_defs.h"
#include "mec_retval.h"

/* Interfaces to any C modules */
#ifdef __cplusplus
extern "C"
{
#endif

#define MEC_QSPI_NUM_INSTANCES 1

#define MEC_QSPI_SHD_PORT      0u
#define MEC_QSPI_PVT_PORT      1u
#define MEC_QSPI_INTERNAL_PORT 2u
#define MEC_QSPI_MAX_PORT      3u

/* SPI Flash Command structure */
struct mec_spi_flash_cmd {
    uint8_t opcode;
    uint8_t npc;          /* number of pins for opcode xmit */
    uint8_t npa;          /* number of pins for command parameter xmit */
    uint8_t npd;          /* number of pins for data r/w */
    uint8_t addr_nb;      /* byte length of address */
    uint8_t mode_byte;    /* optional mode byte */
    uint8_t mode_nbits;   /* num bits in mode byte. 0 = no mode byte */
    uint8_t dummy_clocks; /* num dummy clocks to xmit after mode byte */
};

/* QMSPI API */

enum mec_qspi_signal_mode {    /* clock idle state, TX clock edge, RX clock edge */
    MEC_SPI_SIGNAL_MODE_0 = 0, /*  low               falling        rising       */
    MEC_SPI_SIGNAL_MODE_1,     /*  low               rising         falling      */
    MEC_SPI_SIGNAL_MODE_2,     /*  high              rising         falling      */
    MEC_SPI_SIGNAL_MODE_3,     /*  high              falling        rising       */
    MEC_SPI_SIGNAL_MODE_MAX,
};

enum mec_qspi_cs {
    MEC_QSPI_CS_0 = 0,
    MEC_QSPI_CS_1,
    MEC_QSPI_CS_MAX,
};

enum mec_qspi_io {
    MEC_QSPI_IO_FULL_DUPLEX = 0,
    MEC_QSPI_IO_DUAL,
    MEC_QSPI_IO_QUAD,
    MEC_QSPI_IO_MAX,
};

enum mec_qspi_cstm {
    MEC_QSPI_CSTM_CSA2CLK = 0,
    MEC_QSPI_CSTM_CLK2CSD,
    MEC_QSPI_CSTM_LD2H,
    MEC_QSPI_CSTM_CSD2CSA,
    MEC_QSPI_CSTM_MAX,
};

enum mec_qspi_status {
    MEC_QSPI_STS_XFR_DONE = BIT(0),
    MEC_QSPI_STS_DMA_DONE = BIT(1),
    MEC_QSPI_STS_TXB_ERR = BIT(2), /* overflow TX FIFO or DMA buffer */
    MEC_QSPI_STS_RXB_ERR = BIT(3), /* underflow RX FIFO or DMA buffer */
    MEC_QSPI_STS_PROG_ERR = BIT(4), /* software misconfigured transfer */
    MEC_QSPI_STS_LDMA_RX_ERR = BIT(5), /* Local-DMA error on receive */
    MEC_QSPI_STS_LDMA_TX_ERR = BIT(6), /* Local-DMA error on transmit */
    MEC_QSPI_STS_TXB_FULL = BIT(8), /* TX FIFO full */
    MEC_QSPI_STS_TXB_EMPTY = BIT(9), /* TX FIFO empty */
    MEC_QSPI_STS_TXB_REQ = BIT(10), /* TX FIFO reached high water mark */
    MEC_QSPI_STS_TXB_STALL = BIT(11), /* TX FIFO empty when engine requests more data */
    MEC_QSPI_STS_RXB_FULL = BIT(12), /* RX FIFO full */
    MEC_QSPI_STS_RXB_EMPTY = BIT(13), /* RX FIFO empty */
    MEC_QSPI_STS_RXB_REQ = BIT(14), /* RX FIFO reached high water mark */
    MEC_QSPI_STS_RXB_STALL = BIT(15), /* No clocks generated due to full RX FIFO */
    MEC_QSPI_STS_ACTIVE = BIT(16), /* QSPI is asserting its chip select */
};

enum mec_qspi_intr_enables {
    MEC_QSPI_IEN_XFR_DONE = BIT(0),
    MEC_QSPI_IEN_DMA_DONE = BIT(1),
    MEC_QSPI_IEN_TXB_ERR = BIT(2),
    MEC_QSPI_IEN_RXB_ERR = BIT(3),
    MEC_QSPI_IEN_PROG_ERR = BIT(4),
    MEC_QSPI_IEN_LDMA_RX_ERR = BIT(5),
    MEC_QSPI_IEN_LDMA_TX_ERR = BIT(6),
    MEC_QSPI_IEN_TXB_FULL = BIT(8),
    MEC_QSPI_IEN_TXB_EMPTY = BIT(9),
    MEC_QSPI_IEN_TXB_REQ = BIT(10),
    MEC_QSPI_IEN_RXB_FULL = BIT(12),
    MEC_QSPI_IEN_RXB_EMPTY = BIT(13),
    MEC_QSPI_IEN_RXB_REQ = BIT(14),
};

#define MEC_QSPI_STATE_CLOSED  0
#define MEC_QSPI_STATE_OPEN_TX 1
#define MEC_QSPI_STATE_OPEN_RX 2
#define MEC_QSPI_STATE_MAX     3

#define MEC_QSPI_FLAG_TX_OPCODE BIT(0)
#define MEC_QSPI_FLAG_TX_ADDR   BIT(1)
#define MEC_QSPI_FLAG_TX_DUMCLK BIT(2)
#define MEC_QSPI_FLAG_TX_MODEB  BIT(3)

#define MEC_QSPI_CTX_DIR_RD 0U
#define MEC_QSPI_CTX_DIR_WR 1U

#define MEC_QSPI_BUF_FLAG_IFM_POS 0
#define MEC_QSPI_BUF_FLAG_IFM_MSK 0x3u
#define MEC_QSPI_BUF_FLAG_IFM_FD 0u /* full-duplex */
#define MEC_QSPI_BUF_FLAG_IFM_DUAL 1u /* half-duplex, dual I/O */
#define MEC_QSPI_BUF_FLAG_IFM_QUAD 2u /* half-duplex, quad I/O */
#define MEC_QSPI_BUF_FLAG_DIR_TX_POS 4

struct mec_qspi_buf {
    void *buf;
    uint32_t len;
    uint8_t flags;
};

#if 0 /* TODO not used */
struct mec_qspi_context {
    uintptr_t qspi_base;
    uint32_t devi;
    uint32_t mode;
    uint32_t cstm;
    uint32_t flags;
    uint32_t ien;
    uint8_t state;
    uint8_t ldma_rd_chan;
    uint8_t ldma_wr_chan;
    uint8_t dir;
};
#endif

struct mec_qspi_timing {
    uint32_t freqhz;
    uint8_t dly_csa_to_clk;
    uint8_t dly_clk_to_csd;
    uint8_t dly_csd_to_wph;
    uint8_t dly_start_to_csa;
    uint32_t taps;
};

/* forward reference */
struct qspi_regs;

/* Return QSPI controller SPI clock source in Hz */
uint32_t mec_qspi_max_spi_clock(void);

/* Return current QSPI frequency in Hz */
uint32_t mec_qspi_get_freq(struct qspi_regs *base);
int mec_qspi_set_freq(struct qspi_regs *base, uint32_t freqhz);
int mec_qspi_byte_time_ns(struct qspi_regs *base, uint32_t *btime_ns);

/* Reset QMSPI block and clear interrupt status. */
int mec_qspi_reset(struct qspi_regs *base);

/* 1 = enable clock input to QMSPI block
 * 0 = disable clock input to QMSPI block
 */
int mec_qspi_clk_gate(struct qspi_regs *base, uint8_t gate_clocks_on);

int mec_qspi_init(struct qspi_regs *base,
                  uint32_t freq_hz,
                  enum mec_qspi_signal_mode spi_mode,
                  enum mec_qspi_io iom,
                  enum mec_qspi_cs cs);

int mec_qspi_cs_select(struct qspi_regs *base, enum mec_qspi_cs cs);

int mec_qspi_spi_signal_mode(struct qspi_regs *base, enum mec_qspi_signal_mode spi_mode);

int mec_qspi_io(struct qspi_regs *base, enum mec_qspi_io io);

int mec_qspi_cs_timing_adjust(struct qspi_regs *base, enum mec_qspi_cstm field, uint8_t val);

int mec_qspi_cs_timing(struct qspi_regs *base, uint32_t cs_timing);

int mec_qspi_cs1_freq(struct qspi_regs *base, uint32_t freq);

int mec_qspi_force_stop(struct qspi_regs *base);

int mec_qspi_done(struct qspi_regs *base);

uint32_t mec_qspi_hw_status(struct qspi_regs *base);
int mec_qspi_hw_status_clr(struct qspi_regs *base, uint32_t msk);
int mec_qspi_intr_ctrl(struct qspi_regs *base, int enable);
int mec_qspi_intr_ctrl_msk(struct qspi_regs *base, int enable, uint32_t msk);

int mec_qspi_tx_fifo_is_empty(struct qspi_regs *base);
int mec_qspi_tx_fifo_is_full(struct qspi_regs *base);
int mec_qspi_rx_fifo_is_empty(struct qspi_regs *base);
int mec_qspi_rx_fifo_is_full(struct qspi_regs *base);

/* Start previously configured QSPI transaction.
 * ien_mask == 0 disabled interrupts.
 * ien_mask != 0 should use one or more values from enum mec_qspi_intr_enables
 */
int mec_qspi_start(struct qspi_regs *base, uint32_t ien_mask);

/* Store data bytes into QSPI TX FIFO until full or bufsz reached.
 * Store number of bytes written into nwr if not NULL.
 */
int mec_qspi_wr_tx_fifo(struct qspi_regs *regs, const uint8_t *buf, uint32_t bufsz,
                        uint32_t *nwr);

int mec_qspi_rd_rx_fifo(struct qspi_regs *regs, uint8_t *buf, uint32_t bufsz, uint32_t *nrd);


#define MEC5_QSPI_BUILD_DESCR_TX_DATA BIT(0)
#define MEC5_QSPI_BUILD_DESCR_TX_ZEROS BIT(1)
#define MEC5_QSPI_BUILD_DESCR_TX_ONES BIT(2)
#define MEC5_QSPI_BUILD_DESCR_RX_DATA BIT(3)

/* Build a 32-bit QSPI descriptor based on inputs.
 * ifm is type enum mec_qspi_io specifying the data bus: full-duplex, dual, or quad
 * nunits is the number of units (bytes or bits) to transfer.
 * flags contains bits indicating units are bytes or bits, enable transmit, enable
 *   receive, etc.
 * remunits pointer to uint32_t containing the number of remaining units if the
 * descriptor number of units was exceeded.
 */
uint32_t mec_qspi_build_descr(enum mec_qspi_io ifm, uint32_t nunits, uint32_t *remunits,
                              uint32_t flags);

#if 0
#define MEC5_QSPI_LOAD_DESCR_SET_START BIT(0)
#define MEC5_QSPI_LOAD_DESCR_SET_LAST  BIT(1)

int mec_qspi_load_descrs(struct qspi_regs *base, uint8_t start_descr_idx,
                         const uint32_t *descrs, size_t ndescrs, uint32_t flags);
#endif

#define MEC_QSPI_XFR_FLAG_CLR_FIFOS_POS 0
#define MEC_QSPI_XFR_FLAG_IEN_POS 1
#define MEC_QSPI_XFR_FLAG_CLOSE_POS 2
#define MEC_QSPI_XFR_FLAG_START_POS 3

int mec_qspi_ldma(struct qspi_regs *base, const uint8_t *txb,
                  uint8_t* rxb, size_t lenb, uint32_t flags);


/* -------- 2024-02-24 new API's -------- */
struct mec_qspi_context {
    uint8_t ndescrs;
    uint8_t ntxdma;
    uint8_t nrxdma;
    uint8_t xflags;
    uint32_t descrs[MEC5_QSPI_NUM_DESCRS];
};

void mec_qspi_context_init(struct mec_qspi_context *ctx);

uint8_t mec_qspi_ctx_alloc_ldma_chan(struct mec_qspi_context *ctx, uint8_t is_tx);

#define MEC5_QSPI_DCFG1_FLAG_IFM_POS        0
#define MEC5_QSPI_DCFG1_FLAG_IFM_MSK        0x3u
#define MEC5_QSPI_DCFG1_FLAG_IFM_FD         0u
#define MEC5_QSPI_DCFG1_FLAG_IFM_DUAL       0x1
#define MEC5_QSPI_DCFG1_FLAG_IFM_QUAD       0x2u
#define MEC5_QSPI_DCFG1_FLAG_DIR_TX         BIT(2)
#define MEC5_QSPI_DCFG1_FLAG_DIR_RX         BIT(3)
#define MEC5_QSPI_DCFG1_FLAG_DMA_TX_POS     4
#define MEC5_QSPI_DCFG1_FLAG_DMA_TX_MSK     0x30u
#define MEC5_QSPI_DCFG1_FLAG_DMA_RX_POS     8
#define MEC5_QSPI_DCFG1_FLAG_DMA_RX_MSK     0x300u
#define MEC5_QSPI_DCFG1_FLAG_DMA_MSK0       0x3u

#define MEC5_QSPI_DCFG1_FLAG_IFM(ifm)   \
    (((uint32_t)(ifm) & MEC5_QSPI_DCFG1_FLAG_IFM_MSK) << MEC5_QSPI_DCFG1_FLAG_IFM_POS)

/* chan = 0 (disabled), 1-3 is Local-DMA channel */
#define MEC5_QSPI_DCFG1_FLAG_DMA_TX(chan)   \
    (((uint32_t)(chan) & MEC5_QSPI_DCFG1_FLAG_DMA_MSK0) << MEC5_QSPI_DCFG1_FLAG_DMA_TX_POS)

#define MEC5_QSPI_DCFG1_FLAG_DMA_RX(chan)   \
    (((uint32_t)(chan) & MEC5_QSPI_DCFG1_FLAG_DMA_MSK0) << MEC5_QSPI_DCFG1_FLAG_DMA_RX_POS)

uint32_t mec_qspi_descrs_cfg1(struct mec_qspi_context *ctx, uint32_t nbytes, uint32_t flags);

/* Use same flags as mec_qspi_descrs_cfg1 except only set one of DIR_TX or DIR_RX
 * and the corresponding DMA channel field
 */
int mec_qspi_ldma_cfg1(struct qspi_regs *regs, uintptr_t buf_addr,
                       uint32_t nbytes, uint32_t ldflags);

/* Configure next free descriptor to generate clocks with all I/O pins tri-stated.
 * nclocks < 1000
 * nio_pins = [1, 2, 4]
 */
int mec_qspi_cfg_gen_ts_clocks(struct mec_qspi_context *ctx, uint32_t nclocks, uint8_t nio_pins);

/* Load descriptors from context structure into QSPI descriptor registers
 * NOTE: loads all descriptors.
 */
#define MEC5_QSPI_LD_FLAGS_LAST_POS             0
#define MEC5_QSPI_LD_FLAGS_CLOSE_ON_LAST_POS    1

int mec_qspi_load_descrs(struct qspi_regs *regs, struct mec_qspi_context *ctx, uint32_t flags);

int mec_qspi_load_descrs_at(struct qspi_regs *regs, struct mec_qspi_context *ctx, uint32_t flags,
                            uint8_t load_descr_index);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_QSPI_API_H */