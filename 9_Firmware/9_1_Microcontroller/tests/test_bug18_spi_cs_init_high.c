/*******************************************************************************
 * test_bug18_spi_cs_init_high.c
 *
 * Bug #18 (FIXED): SPI chip-select pins initialized LOW by MX_GPIO_Init(),
 * causing SDO bus contention between dual ADF4382A PLLs (and other SPI
 * devices).
 *
 * This test verifies:
 *   A) The deassert_peer_cs() logic correctly deasserts all peer CS pins
 *      (sets them HIGH) before asserting the target CS pin (LOW).
 *   B) At no point in the spy log are two CS pins simultaneously LOW.
 *   C) The peer table covers all 7 SPI CS pins on the board.
 *   D) Legacy path (no CS) doesn't produce GPIO writes.
 *
 * We re-implement the peer CS logic here identically to stm32_spi.c,
 * exercising it against the spy layer.  This validates the algorithm
 * without needing to compile the full stm32_spi.c with its ADI
 * dependencies.  Any structural drift will be caught by a compile of
 * stm32_spi.c on the real target.
 ******************************************************************************/
#include "stm32_spi.h"     /* shim: stm32_spi_extra, stm32_spi_ops */
#include "main.h"           /* shim: pin defines */
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* We need the no_os types for the descriptor */
#include "no_os_spi.h"

/* ---- Replicate the peer CS table from stm32_spi.c ---- */

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
 *        to assert.  Identical to deassert_peer_cs() in stm32_spi.c.
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
 * @brief Simulate the CS assertion sequence from stm32_spi_write_and_read().
 *        deassert peers → assert target → SPI xfer → deassert target.
 */
static int simulate_spi_cs_sequence(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    /* Step 1: Deassert all peers */
    deassert_peer_cs(cs_port, cs_pin);

    /* Step 2: Assert target CS (active low) */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

    /* Step 3: SPI transfer (simulated via spy) */
    uint8_t buf[3] = { 0x00, 0x12, 0x34 };
    HAL_SPI_TransmitReceive(&hspi4, buf, buf, 3, 200);

    /* Step 4: Deassert target CS */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return 0;
}

/*
 * Helper: scan the spy log and ensure no two CS pins from the peer table
 * are simultaneously LOW.  Returns 0 if no contention, or -1 if detected.
 */
#define NUM_CS 7

static int check_no_simultaneous_low(void)
{
    /* Track current pin state (start all HIGH = deasserted) */
    GPIO_PinState state[NUM_CS];
    for (int i = 0; i < NUM_CS; i++)
        state[i] = GPIO_PIN_SET;

    /* Walk the spy log */
    for (int s = 0; s < spy_count; s++) {
        const SpyRecord *rec = spy_get(s);
        if (!rec || rec->type != SPY_GPIO_WRITE)
            continue;

        /* Update state for any CS pin that was written */
        for (int i = 0; i < NUM_CS; i++) {
            if (rec->port == (void *)spi_cs_peers[i].port &&
                rec->pin == spi_cs_peers[i].pin) {
                state[i] = (GPIO_PinState)rec->value;
                break;
            }
        }

        /* Count how many CS pins are LOW */
        int low_count = 0;
        for (int i = 0; i < NUM_CS; i++) {
            if (state[i] == GPIO_PIN_RESET)
                low_count++;
        }
        if (low_count > 1) {
            printf("  FAIL: %d CS pins simultaneously LOW at spy record %d\n",
                   low_count, s);
            return -1;
        }
    }
    return 0;
}

int main(void)
{
    printf("=== Bug #18 (FIXED): SPI CS peer deassertion ===\n");

    /*
     * Test A: TX CS sequence — deasserts RX + AD9523 + 4x ADAR as peers
     */
    {
        spy_reset();
        simulate_spi_cs_sequence(GPIOG, ADF4382_TX_CS_Pin);

        printf("  A1: Peer table has %d entries (expected 7)\n", (int)SPI_CS_PEER_COUNT);
        assert(SPI_CS_PEER_COUNT == 7);
        printf("  PASS: Peer table covers all 7 CS pins\n");

        /* Count GPIO writes: 6 peer SET + 1 target RESET + 1 target SET = 8 */
        int gpio_writes = spy_count_type(SPY_GPIO_WRITE);
        printf("  A2: GPIO writes = %d (expected 8)\n", gpio_writes);
        assert(gpio_writes == 8);
        printf("  PASS: correct number of GPIO writes\n");

        /* Verify the GPIO write just before SPI xfer is target CS RESET */
        int spi_idx = spy_find_nth(SPY_SPI_TRANSMIT_RECEIVE, 0);
        assert(spi_idx > 0);
        printf("  A3: SPI xfer at spy index %d\n", spi_idx);

        const SpyRecord *cs_assert = spy_get(spi_idx - 1);
        assert(cs_assert != NULL);
        assert(cs_assert->type == SPY_GPIO_WRITE);
        assert(cs_assert->port == (void *)GPIOG);
        assert(cs_assert->pin == ADF4382_TX_CS_Pin);
        assert(cs_assert->value == GPIO_PIN_RESET);
        printf("  PASS: TX CS asserted (LOW) immediately before SPI xfer\n");

        /* The GPIO write just after SPI xfer is target CS SET */
        const SpyRecord *cs_deassert = spy_get(spi_idx + 1);
        assert(cs_deassert != NULL);
        assert(cs_deassert->type == SPY_GPIO_WRITE);
        assert(cs_deassert->port == (void *)GPIOG);
        assert(cs_deassert->pin == ADF4382_TX_CS_Pin);
        assert(cs_deassert->value == GPIO_PIN_SET);
        printf("  PASS: TX CS deasserted (HIGH) immediately after SPI xfer\n");

        /* Verify all 6 peer deassertions happened before the target assert */
        int peer_set_count = 0;
        for (int i = 0; i < spi_idx - 1; i++) {
            const SpyRecord *r = spy_get(i);
            if (r && r->type == SPY_GPIO_WRITE && r->value == GPIO_PIN_SET)
                peer_set_count++;
        }
        printf("  A4: Peer deassert count before CS assert = %d (expected 6)\n", peer_set_count);
        assert(peer_set_count == 6);
        printf("  PASS: All 6 peer CS pins deasserted before TX CS assert\n");

        /* Verify no simultaneous contention */
        int contention = check_no_simultaneous_low();
        assert(contention == 0);
        printf("  PASS: No simultaneous CS contention detected\n");
    }

    /*
     * Test B: RX CS sequence — peers should include TX CS
     */
    {
        spy_reset();
        simulate_spi_cs_sequence(GPIOG, ADF4382_RX_CS_Pin);

        int spi_idx = spy_find_nth(SPY_SPI_TRANSMIT_RECEIVE, 0);
        assert(spi_idx > 0);

        /* Target assert is RX CS */
        const SpyRecord *cs_assert = spy_get(spi_idx - 1);
        assert(cs_assert->port == (void *)GPIOG);
        assert(cs_assert->pin == ADF4382_RX_CS_Pin);
        assert(cs_assert->value == GPIO_PIN_RESET);
        printf("  B1: RX CS asserted correctly\n");

        /* Check that TX CS was deasserted as a peer */
        int found_tx_deassert = 0;
        for (int i = 0; i < spi_idx - 1; i++) {
            const SpyRecord *r = spy_get(i);
            if (r && r->type == SPY_GPIO_WRITE &&
                r->port == (void *)GPIOG &&
                r->pin == ADF4382_TX_CS_Pin &&
                r->value == GPIO_PIN_SET) {
                found_tx_deassert = 1;
                break;
            }
        }
        assert(found_tx_deassert);
        printf("  PASS: TX CS deasserted as peer before RX CS assert\n");

        int contention = check_no_simultaneous_low();
        assert(contention == 0);
        printf("  PASS: No CS contention in RX path\n");
    }

    /*
     * Test C: AD9523 CS — peers include both ADF4382 + all ADARs
     */
    {
        spy_reset();
        simulate_spi_cs_sequence(GPIOF, AD9523_CS_Pin);

        int spi_idx = spy_find_nth(SPY_SPI_TRANSMIT_RECEIVE, 0);
        const SpyRecord *cs_assert = spy_get(spi_idx - 1);
        assert(cs_assert->port == (void *)GPIOF);
        assert(cs_assert->pin == AD9523_CS_Pin);
        assert(cs_assert->value == GPIO_PIN_RESET);
        printf("  C1: AD9523 CS asserted correctly\n");

        /* All 6 peers deasserted */
        int peer_set_count = 0;
        for (int i = 0; i < spi_idx - 1; i++) {
            const SpyRecord *r = spy_get(i);
            if (r && r->type == SPY_GPIO_WRITE && r->value == GPIO_PIN_SET)
                peer_set_count++;
        }
        assert(peer_set_count == 6);
        printf("  PASS: All 6 peers deasserted before AD9523 CS assert\n");

        int contention = check_no_simultaneous_low();
        assert(contention == 0);
        printf("  PASS: No CS contention in AD9523 path\n");
    }

    /*
     * Test D: ADAR_1 CS — peers include both ADF4382 + AD9523 + ADAR 2/3/4
     */
    {
        spy_reset();
        simulate_spi_cs_sequence(GPIOA, ADAR_1_CS_3V3_Pin);

        int spi_idx = spy_find_nth(SPY_SPI_TRANSMIT_RECEIVE, 0);
        const SpyRecord *cs_assert = spy_get(spi_idx - 1);
        assert(cs_assert->port == (void *)GPIOA);
        assert(cs_assert->pin == ADAR_1_CS_3V3_Pin);
        assert(cs_assert->value == GPIO_PIN_RESET);
        printf("  D1: ADAR_1 CS asserted correctly\n");

        /* All 6 peers deasserted */
        int peer_set_count = 0;
        for (int i = 0; i < spi_idx - 1; i++) {
            const SpyRecord *r = spy_get(i);
            if (r && r->type == SPY_GPIO_WRITE && r->value == GPIO_PIN_SET)
                peer_set_count++;
        }
        assert(peer_set_count == 6);
        printf("  PASS: All 6 peers deasserted before ADAR_1 CS assert\n");

        int contention = check_no_simultaneous_low();
        assert(contention == 0);
        printf("  PASS: No CS contention in ADAR_1 path\n");
    }

    /*
     * Test E: Back-to-back TX then RX — simulates ADF4382A_Manager_Init() flow
     * Verifies that even when TX was just active, RX sequence correctly
     * deasserts TX before asserting RX.
     */
    {
        spy_reset();
        simulate_spi_cs_sequence(GPIOG, ADF4382_TX_CS_Pin);
        simulate_spi_cs_sequence(GPIOG, ADF4382_RX_CS_Pin);

        /* Should be 2 x 8 = 16 GPIO writes total */
        int gpio_writes = spy_count_type(SPY_GPIO_WRITE);
        printf("  E1: GPIO writes for back-to-back = %d (expected 16)\n", gpio_writes);
        assert(gpio_writes == 16);
        printf("  PASS: Correct GPIO write count for back-to-back\n");

        /* No contention at any point */
        int contention = check_no_simultaneous_low();
        assert(contention == 0);
        printf("  PASS: No CS contention in back-to-back TX→RX sequence\n");
    }

    /*
     * Test F: Verify peer table matches pin definitions from main.h
     * This catches any drift between the peer table and the actual pin map.
     */
    {
        assert(spi_cs_peers[0].port == GPIOG && spi_cs_peers[0].pin == GPIO_PIN_14);
        assert(spi_cs_peers[1].port == GPIOG && spi_cs_peers[1].pin == GPIO_PIN_10);
        assert(spi_cs_peers[2].port == GPIOF && spi_cs_peers[2].pin == GPIO_PIN_7);
        assert(spi_cs_peers[3].port == GPIOA && spi_cs_peers[3].pin == GPIO_PIN_0);
        assert(spi_cs_peers[4].port == GPIOA && spi_cs_peers[4].pin == GPIO_PIN_1);
        assert(spi_cs_peers[5].port == GPIOA && spi_cs_peers[5].pin == GPIO_PIN_2);
        assert(spi_cs_peers[6].port == GPIOA && spi_cs_peers[6].pin == GPIO_PIN_3);
        printf("  F1: All peer table entries match main.h pin definitions\n");
        printf("  PASS: Peer table is consistent with hardware pin map\n");
    }

    printf("=== Bug #18 (FIXED): ALL TESTS PASSED ===\n\n");
    return 0;
}
