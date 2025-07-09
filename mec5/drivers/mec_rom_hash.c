/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include "mec_pcfg.h"
#include "mec_defs.h"
#include "mec_ecia_api.h"
#include "mec_rom_api.h"
#include "mec_retval.h"

/* Hash and HMAC */
#if defined(MEC5_ROM_API_HASH_ENABLED)

#define MEC_HASH_GIRQ           16
#define MEC_HASH_GIRQ_POS       3
#define MEC_HASH_GIRQ_AGGR_NVIC 8
#define MEC_HASH_DIRECT_NVIC    68

#define MEC_HASH_ECIA_INFO MEC5_ECIA_INFO(MEC_HASH_GIRQ, MEC_HASH_GIRQ_POS, \
                                           MEC_HASH_GIRQ_AGGR_NVIC, MEC_HASH_DIRECT_NVIC)

#define MEC_ROM_API_HASH_CREATE_SHA1_ADDR    0x1f179u
#define MEC_ROM_API_HASH_CREATE_SHA224_ADDR  0x1f17du
#define MEC_ROM_API_HASH_CREATE_SHA256_ADDR  0x1f181u
#define MEC_ROM_API_HASH_CREATE_SHA384_ADDR  0x1f185u
#define MEC_ROM_API_HASH_CREATE_SHA512_ADDR  0x1f189u
#define MEC_ROM_API_HASH_CREATE_SM3_ADDR     0x1f18du
#define MEC_ROM_API_HASH_INIT_STATE_ADDR     0x1f191u
#define MEC_ROM_API_HASH_RESUME_STATE_ADDR   0x1f195u
#define MEC_ROM_API_HASH_SAVE_STATE_ADDR     0x1f199u
#define MEC_ROM_API_HASH_FEED_DATA_ADDR      0x1f19du
#define MEC_ROM_API_HASH_COMPUTE_DIGEST_ADDR 0x1f1a1u
#define MEC_ROM_API_HASH_WAIT_ADDR           0x1f1a5u
#define MEC_ROM_API_HASH_GET_DIGEST_SZ_ADDR  0x1f1a9u
#define MEC_ROM_API_HASH_GET_STATUS_ADDR     0x1f1adu

#define MEC_ROM_API_HMAC2_INIT_ADDR          0x1f1bdu
#define MEC_ROM_API_HMAC2_ADD_DATA_BLK_ADDR  0x1f1c1u
#define MEC_ROM_API_HMAC2_FINAL_ADDR         0x1f1c5u

typedef int (*rom_hash_create_sha1_td)(struct mchphash *c);
typedef int (*rom_hash_create_sha224_td)(struct mchphash *c);
typedef int (*rom_hash_create_sha256_td)(struct mchphash *c);
typedef int (*rom_hash_create_sha384_td)(struct mchphash *c);
typedef int (*rom_hash_create_sha512_td)(struct mchphash *c);
typedef int (*rom_hash_create_sm3_td)(struct mchphash *c);
typedef void (*rom_hash_init_state_td)(struct mchphash *c, struct mchphashstate *h, uint8_t *dmamem);
typedef void (*rom_hash_resume_state_td)(struct mchphash *c, struct mchphashstate *h);
typedef int (*rom_hash_save_state_td)(struct mchphash *c);
typedef int (*rom_hash_feed_td)(struct mchphash *c, const uint8_t *msg, size_t sz);
typedef int (*rom_hash_digest_td)(struct mchphash *c, uint8_t *digest);
typedef int (*rom_hash_wait_td)(struct mchphash *c);
typedef size_t (*rom_hash_get_digestsz_td)(struct mchphash *c);
typedef int (*rom_hash_status_td)(struct mchphash *c);

typedef int (*rom_hmac2_init_td)(enum mchp_hash_alg_id alg_id, struct mchphmac2 *m,
                                 const uint8_t *key, size_t keysz, uint32_t *k0, size_t k0sz);

typedef int (*rom_hmac2_add_datablk_td)(struct mchphmac2 *m, uint8_t *state, size_t statesz,
                                        uint32_t *k0, size_t k0sz, const uint8_t *datablk,
                                        size_t datablksz, bool last_data);

typedef int (*rom_hmac2_final_td)(struct mchphmac2 *m, uint8_t *state, size_t statesz, uint32_t *k0,
                                  size_t k0sz, uint8_t *hmac, size_t hmacsz);

struct mec_rom_hash_api {
    rom_hash_create_sha1_td rom_hash_create_sha1;
    rom_hash_create_sha224_td rom_hash_create_sha224;
    rom_hash_create_sha256_td rom_hash_create_sha256;
    rom_hash_create_sha384_td rom_hash_create_sha384;
    rom_hash_create_sha512_td rom_hash_create_sha512;
    rom_hash_create_sm3_td rom_hash_create_sm3;
    rom_hash_init_state_td rom_hash_init_state;
    rom_hash_resume_state_td rom_hash_resume_state;
    rom_hash_save_state_td rom_hash_save_state;
    rom_hash_feed_td rom_hash_feed;
    rom_hash_digest_td rom_hash_digest;
    rom_hash_wait_td rom_hash_wait;
    rom_hash_get_digestsz_td rom_hash_digest_size;
    rom_hash_status_td rom_hash_status;
#if defined(MEC5_ROM_API_HMAC_ENABLED)
    rom_hmac2_init_td rom_hmac2_init;
    rom_hmac2_add_datablk_td rom_hmac2_add_datablk;
    rom_hmac2_final_td rom_hmac2_final;
#endif
};

static const struct mec_rom_api mec_rom_hash_api_tbl = {
    .rom_hash_create_sha1 =   (rom_hash_create_sha1_td)MEC_ROM_API_HASH_CREATE_SHA1_ADDR,
    .rom_hash_create_sha224 = (rom_hash_create_sha224_td)MEC_ROM_API_HASH_CREATE_SHA224_ADDR,
    .rom_hash_create_sha256 = (rom_hash_create_sha256_td)MEC_ROM_API_HASH_CREATE_SHA256_ADDR,
    .rom_hash_create_sha384 = (rom_hash_create_sha384_td)MEC_ROM_API_HASH_CREATE_SHA384_ADDR,
    .rom_hash_create_sha512 = (rom_hash_create_sha512_td)MEC_ROM_API_HASH_CREATE_SHA512_ADDR,
    .rom_hash_create_sm3 =    (rom_hash_create_sm3_td)MEC_ROM_API_HASH_CREATE_SM3_ADDR,
    .rom_hash_init_state =    (rom_hash_init_state_td)MEC_ROM_API_HASH_INIT_STATE_ADDR,
    .rom_hash_resume_state =  (rom_hash_resume_state_td)MEC_ROM_API_HASH_RESUME_STATE_ADDR,
    .rom_hash_save_state =    (rom_hash_save_state_td)MEC_ROM_API_HASH_SAVE_STATE_ADDR,
    .rom_hash_feed =          (rom_hash_feed_td)MEC_ROM_API_HASH_FEED_DATA_ADDR,
    .rom_hash_digest =        (rom_hash_digest_td)MEC_ROM_API_HASH_COMPUTE_DIGEST_ADDR,
    .rom_hash_wait =          (rom_hash_wait_td)MEC_ROM_API_HASH_WAIT_ADDR,
    .rom_hash_digest_size =   (rom_hash_get_digestsz_td)MEC_ROM_API_HASH_GET_DIGEST_SZ_ADDR,
    .rom_hash_status =        (rom_hash_status_td)MEC_ROM_API_HASH_GET_STATUS_ADDR,
#if defined(MEC5_ROM_API_HMAC_ENABLED)
    .rom_hmac2_init =         (rom_hmac2_init_td)MEC_ROM_API_HMAC2_INIT_ADDR,
    .rom_hhmac2_add_datablk = (rom_hmac2_add_datablk_td)MEC_ROM_API_HMAC2_ADD_DATA_BLK_ADDR,
    .rom_hmac2_final =        (rom_hmac2_final_td)MEC_ROM_API_HMAC2_FINAL_ADDR,
#endif
};

/* ---- Public API ---- */
int mec_hal_rom_hash_create_sha1(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_create_sha1(c);

    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_create_sha224(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_create_sha224(c);

    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_create_sha256(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_create_sha256(c);

    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_create_sha384(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_create_sha384(c);

    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_create_sha512(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_create_sha512(c);

    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_create_sm3(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_create_sm3(c);

    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_init_state(struct mchphash *c, struct mchphashstate *h, uint8_t *dmamem)
{
    if (!c || !h || !dmamem) {
        return MEC_RET_ERR_INVAL;
    }

    mec_rom_api_tbl.rom_hash_init_state(c, h, dmamem);

    return MEC_RET_OK;
}

int mec_hal_rom_hash_resume_state(struct mchphash *c, struct mchphashstate *h)
{
    if (!c || !h) {
        return MEC_RET_ERR_INVAL;
    }

    mec_rom_api_tbl.rom_hash_resume_state(c, h);

    return MEC_RET_OK;
}

int mec_hal_rom_hash_save_state(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_save_state(c);

    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_add_data(struct mchphash *c, const uint8_t *data, size_t datasz)
{
    int ret = 0;

    if (!c || (!data && datasz)) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_feed(c, data, datasz);
    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hash_compute_digest(struct mchphash *c, uint8_t *result)
{
    int ret = 0;

    if (!c || !result) {
        return MEC_RET_ERR;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_digest(c, result);
    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

/* blocking */
int mec_hal_rom_hash_wait(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_wait(c);
    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

size_t mec_hal_rom_hash_get_digest_size(struct mchphash *c)
{
    if (!c) {
        return 0;
    }

    return mec_rom_hash_api_tbl.rom_hash_digest_size(c);
}

int mec_hal_rom_hash_get_status(struct mchphash *c)
{
    int ret = 0;

    if (!c) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hash_status(c);
    if (ret == -1) {
        return MEC_RET_ERR_BUSY;
    } else if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}
#endif /* #if defined(MEC5_ROM_API_HASH_ENABLED) */

#if defined(MEC5_ROM_API_HMAC_ENABLED)

int mec_hal_rom_hmac2_init(enum mchp_hash_alg_id alg_id, struct mchphmac2 *m,
                           const uint8_t *key, size_t keysz, uint32_t *k0, size_t k0sz)
{
    int ret = 0;

    if (!m) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hmac2_init(alg_id, m, key, keysz, k0, k0sz);
    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hmac2_add_data_block(struct mchphmac2 *m, uint8_t *state, size_t statesz,
                                     uint32_t *k0, size_t k0sz, const uint8_t *datablk,
                                     size_t datablksz, bool last_data)
{
    int ret = 0;

    if (!m || !state || !k0) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hmac2_add_datablk(m, state, statesz, k0, k0sz,
                                                     datablk, datablksz, last_data);
    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

int mec_hal_rom_hmac2_final(struct mchphmac2 *m, uint8_t *state, size_t statesz, uint32_t *k0,
                            size_t k0sz, uint8_t *hmac, size_t hmacsz)
{
    int ret = 0;

    if (!m || !state || !k0 || !hmac) {
        return MEC_RET_ERR_INVAL;
    }

    ret = mec_rom_hash_api_tbl.rom_hmac2_final(m, state, statesz, k0, k0sz, hmac, hmacsz);
    if (ret != 0) {
        return MEC_RET_ERR;
    }

    return MEC_RET_OK;
}

#endif /* defined(MEC5_ROM_API_HMAC_ENABLED) */

/* end mec_rom_hash.c */
