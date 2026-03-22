#include "stm32_spi.h"
#include "no_os_error.h"
#include <stdlib.h>
#include <string.h>

/*
 * Peer CS table: all chip-select pins sharing SPI4 on GPIOG.
 * Before asserting any CS, we deassert all peers on the same port to
 * prevent SDO bus contention from devices that share the MISO line.
 * Also includes AD9523 CS on GPIOF and ADAR CS pins on GPIOA.
 */
#include "main.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} cs_peer_entry_t;

static const cs_peer_entry_t spi_cs_peers[] = {
    { GPIOG, ADF4382_TX_CS_Pin },   /* GPIO_PIN_14 */
    { GPIOG, ADF4382_RX_CS_Pin },   /* GPIO_PIN_10 */
    { GPIOF, AD9523_CS_Pin },       /* GPIO_PIN_7  */
    { GPIOA, ADAR_1_CS_3V3_Pin },   /* GPIO_PIN_0  */
    { GPIOA, ADAR_2_CS_3V3_Pin },   /* GPIO_PIN_1  */
    { GPIOA, ADAR_3_CS_3V3_Pin },   /* GPIO_PIN_2  */
    { GPIOA, ADAR_4_CS_3V3_Pin },   /* GPIO_PIN_3  */
};
#define SPI_CS_PEER_COUNT (sizeof(spi_cs_peers) / sizeof(spi_cs_peers[0]))

/**
 * @brief Deassert all peer CS pins (set HIGH) except the one we're about
 *        to assert.  This prevents bus contention on shared MISO/SDO lines.
 */
static void deassert_peer_cs(GPIO_TypeDef *target_port, uint16_t target_pin)
{
    for (unsigned i = 0; i < SPI_CS_PEER_COUNT; i++) {
        if (spi_cs_peers[i].port == target_port &&
            spi_cs_peers[i].pin  == target_pin)
            continue;  /* skip the one we're about to assert */
        HAL_GPIO_WritePin(spi_cs_peers[i].port, spi_cs_peers[i].pin,
                          GPIO_PIN_SET);
    }
}

/**
 * @brief Detect whether 'extra' points to a stm32_spi_extra struct (with
 *        cs_port != NULL) or is a bare SPI_HandleTypeDef* (legacy path).
 *
 * Heuristic: if cs_port is a valid-looking pointer (non-NULL, non-trivial)
 * we treat extra as stm32_spi_extra.  Legacy callers pass &hspi4 directly,
 * whose first field (Instance) is a peripheral base address — never a small
 * number, but also never a GPIO_TypeDef*.  We use the struct's own cs_port
 * field to discriminate: stm32_spi_extra always sets cs_port explicitly.
 */

int32_t stm32_spi_init(struct no_os_spi_desc **desc,
                       const struct no_os_spi_init_param *param)
{
    if (!desc || !param)
        return -EINVAL;

    *desc = calloc(1, sizeof(**desc));
    if (!*desc)
        return -ENOMEM;

    /*
     * If the caller provides a stm32_spi_extra with cs_port set, allocate
     * a copy so the descriptor owns the data.  Otherwise, store the raw
     * SPI_HandleTypeDef* for backward compatibility.
     */
    const stm32_spi_extra *in_extra = (const stm32_spi_extra *)param->extra;
    if (in_extra && in_extra->cs_port != NULL) {
        /* Caller provided full stm32_spi_extra with software CS */
        stm32_spi_extra *own = calloc(1, sizeof(stm32_spi_extra));
        if (!own) {
            free(*desc);
            *desc = NULL;
            return -ENOMEM;
        }
        memcpy(own, in_extra, sizeof(stm32_spi_extra));
        (*desc)->extra = own;
    } else {
        /* Legacy: extra is a bare SPI_HandleTypeDef* */
        (*desc)->extra = param->extra;
    }

    (*desc)->max_speed_hz = param->max_speed_hz;
    (*desc)->mode = param->mode;
    (*desc)->chip_select = param->chip_select;

    return 0;
}

int32_t stm32_spi_write_and_read(struct no_os_spi_desc *desc,
                                 uint8_t *data,
                                 uint16_t bytes_number)
{
    if (!desc || !data || bytes_number == 0)
        return -EINVAL;

    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port = NULL;
    uint16_t      cs_pin  = 0;

    /*
     * Determine HAL handle and optional CS info.
     * If extra is a stm32_spi_extra with cs_port set, use its fields.
     * Otherwise treat extra as a bare SPI_HandleTypeDef*.
     */
    const stm32_spi_extra *sx = (const stm32_spi_extra *)desc->extra;
    if (sx && sx->cs_port != NULL) {
        hspi    = sx->hspi;
        cs_port = sx->cs_port;
        cs_pin  = sx->cs_pin;
    } else {
        hspi = (SPI_HandleTypeDef *)desc->extra;
    }

    if (!hspi)
        return -EINVAL;

    /* Deassert all peer CS pins first to prevent SDO bus contention,
     * then assert our target CS (active low). */
    if (cs_port) {
        deassert_peer_cs(cs_port, cs_pin);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    }

    HAL_StatusTypeDef hal_ret;
    hal_ret = HAL_SPI_TransmitReceive(hspi, data, data, bytes_number, 200);

    /* Deassert CS */
    if (cs_port)
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    if (hal_ret != HAL_OK)
        return -EIO;

    return 0;
}

int32_t stm32_spi_remove(struct no_os_spi_desc *desc)
{
    if (!desc)
        return -EINVAL;

    /*
     * If we allocated a stm32_spi_extra copy during init, free it.
     * Detect by checking cs_port (same heuristic as init).
     */
    if (desc->extra) {
        const stm32_spi_extra *sx = (const stm32_spi_extra *)desc->extra;
        if (sx->cs_port != NULL)
            free(desc->extra);
    }

    free(desc);
    return 0;
}

/* platform ops struct */
const struct no_os_spi_platform_ops stm32_spi_ops = {
    .init = &stm32_spi_init,
    .write_and_read = &stm32_spi_write_and_read,
    .remove = &stm32_spi_remove,
};
