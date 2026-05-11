/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "USBHandler.h"
#include "usbd_cdc_if.h"
#include "ADAR1000_Manager.h"
#include "ADAR1000_AGC.h"
extern "C" {
#include "ad9523.h"
}
#include "no_os_delay.h"
#include "no_os_alloc.h"
#include "no_os_print_log.h"
#include "no_os_error.h"
#include "no_os_units.h"
#include "no_os_dma.h"
#include "no_os_spi.h"
#include "no_os_uart.h"
#include "no_os_util.h"
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <iostream>
#include <vector>
#include "stm32_spi.h"
#include "stm32_delay.h"
extern "C" {
#include "um982_gps.h"
}
extern "C" {
#include "GY_85_HAL.h"
}
#include "BMP180.h"
extern "C" {
#include "platform_noos_stm32.h"
}
extern "C" {

#include "adf4382a_manager.h"
#include "adf4382.h"
#include "no_os_delay.h"
#include "no_os_error.h"
#include "diag_log.h"
}
#include <cmath>
#include "DAC5578.h"
#include "ADS7830.h"
#include "gps_handler.h"

#include "stm32f7xx_hal.h"




/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* -------- GPIO index mapping for no-OS interface -------- */
#define GPIO_IDX_TX_CS       0
#define GPIO_IDX_RX_CS       1
#define GPIO_IDX_TX_DELSTR   2
#define GPIO_IDX_TX_DELADJ   3
#define GPIO_IDX_RX_DELSTR   4
#define GPIO_IDX_RX_DELADJ   5
#define GPIO_IDX_TX_CE       6
#define GPIO_IDX_RX_CE       7
#define GPIO_IDX_SW_SYNC     8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// UM982 dual-antenna GPS receiver
UM982_GPS_t um982;

// Global data structures
GPS_Data_t current_gps_data = {0};

// Global USB handler
USBHandler usbHandler;
extern USBD_HandleTypeDef hUsbDeviceFS;

// Buffer for USB reception
uint8_t usb_rx_buffer[64];

//IMU variable
GY85_t imu;

float Pitch_Sensor, Yaw_Sensor, Roll_Sensor;

float abias[3] = {-0.108, -0.038, -0.006}, gbias[3] = {-10, 6, -12}, mbias[3]={0.0,0.0,0.0};
float M11=1.0,  M12=-0.0,  M13=0.0,
      M21=0.0,  M22=1.0,   M23=0.0,
      M31=0.0,  M32=-0.0,  M33=1.0;
float ax, ay, az, gx, gy, gz, mx, my, mz,mxc,myc,mzc; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float temperature;
/* MCU-A2: site-specific magnetic declination. Was a hardcoded -0.61° literal
 * baked in for one deployment location. Default kept for backward compatibility
 * with existing sites; runtime override flows through set_mag_declination_deg()
 * and persists across reset in BKPSRAM (slots 1+2 — slot 0 is the MCU-A7
 * emergency flag). Reads via get_mag_declination_deg() at the use site. */
#define MAG_DECLINATION_DEFAULT_DEG  (-0.61f)
#define MAG_DECLINATION_LIMIT_DEG    (30.0f)
float Mag_Declination = MAG_DECLINATION_DEFAULT_DEG;  /* legacy global, retained for any external linkage */

float RxAcc,RyAcc,RzAcc;
float Rate_Ayz_1,Rate_Axz_1,Rate_Axy_1;
float Axz_1,Ayz_1,Axy_1, Axz_0,Ayz_0,Axy_0;
float RxEst_0,RyEst_0,RzEst_0, RxEst_1,RyEst_1,RzEst_1,Azr_Est;
float RxGyro,RyGyro,RzGyro, R_Est;

unsigned long now_timeperiod;
unsigned long lasttime_timeperiod = 0;
int Time_Period = 0;

//Barometer BMP180

/*resolution:
BMP180_ULTRALOWPOWER - pressure oversampled 1 time  & power consumption 3μA
BMP180_STANDARD      - pressure oversampled 2 times & power consumption 5μA
BMP180_HIGHRES       - pressure oversampled 4 times & power consumption 7μA
BMP180_ULTRAHIGHRES  - pressure oversampled 8 times & power consumption 12μA, library default
*/
BMP180 myBMP(BMP180_ULTRAHIGHRES);
float RADAR_Altitude;
/* AUDIT-CAL: latched true after a successful myBMP.begin() at boot.
 * Gates the boot-time altitude-baseline loop and the runtime health
 * watchdog so a missing or unresponsive BMP180 does NOT churn ERROR_BMP180_COMM
 * every health-check pass and trip the error_count > 10 SAFE-MODE latch.
 * Without this gate the radar self-shuts-down within ~25 s of boot whenever
 * the sensor is absent or its calibration could not be read. */
static bool bmp180_operational = false;

//GPS
double RADAR_Longitude = 0;
double RADAR_Latitude = 0;

extern uint8_t GUI_start_flag_received;  // [STM32-006] Legacy, unused -- kept for linker compat


//RADAR
// Radar parameters — match the FPGA chirp_scheduler defaults defined in
// 9_Firmware/9_2_FPGA/radar_params.vh (RP_DEF_*_CHIRP/LISTEN_CYCLES_V2).
// PR-F: 48 chirps/frame = 3 sub-frames * 16 chirps each (SHORT/MEDIUM/LONG).
// PR-Q.1: MEDIUM PRI staggered to 161 us so the 3 PRIs share no small
// common period (175/161/167) — needed by the host CRT Doppler unfolder.
const int m_max = 48;   // Chirps per frame (PR-F: 3 sub-frames * 16)
const int n_max = 31;   // Number of beam positions
const float T1 = 30.0f;     // LONG chirp duration in microseconds
const float PRI1 = 167.0f;  // LONG PRI in microseconds
const float T2 = 1.0f;      // SHORT chirp duration (PR-E: 0.5 us -> 1.0 us at 100 MHz)
const float PRI2 = 175.0f;  // SHORT PRI in microseconds
const float T_MEDIUM = 5.0f;     // MEDIUM chirp duration (PR-F)
const float PRI_MEDIUM = 161.0f; // MEDIUM PRI (PR-Q.1 stagger)
const float Guard = 175.4f; // Guard time in microseconds

uint8_t	m = 1; // m = N° of chirp/ position = 16 (made of T1 and PRF1)+ Guard = 175µs +16 (made of T2 and PRF2)
uint8_t	n = 1; // n =[1-31] N° of elevations/azimuth
uint8_t	y = 1;// y = N° of azimuths/revolution = 50 controlled by a microcontroller connected to a stepper motor
uint8_t y_max = 50;// y = N° of azimuths/revolution
uint64_t IF_freq = 120000000ULL;//120MHz

#define PowerAmplifier 1

//Stepper Motor
float Stepper_steps = 200.0f;//step per revolution

// DAC5578 handles (RF Power Amplifier DAC controlling Vg)
DAC5578_HandleTypeDef hdac1, hdac2;
/* Phase accumulators */
float phase_dac1[8] = {0};
float phase_dac2[8] = {0};
const uint32_t sampleRate = 370;    // Sample rate in Hz
const uint32_t period = 2700;       // Period in microseconds
uint8_t DAC_val = 126;

// ADC handles (RF Power Amplifier ADC measuring Idq)
ADS7830_HandleTypeDef hadc1, hadc2;
uint8_t adc1_readings[8] = {0};
uint8_t adc2_readings[8] = {0};
float Idq_reading[16]={0.0f};

/* [GAP-3 FIX 2] Hardware IWDG watchdog handle
 * Prescaler=256, Reload=500 → timeout ≈ 4.096 s from 32 kHz LSI.
 * If the main loop stalls, the MCU resets automatically. */
IWDG_HandleTypeDef hiwdg;


// Global manager instance ADF4382A
ADF4382A_Manager lo_manager;
extern SPI_HandleTypeDef hspi4;

// AD9523 device handle. Retained at file scope so the recovery path can free
// the previous instance before re-running setup (F-4.5: ad9523_setup() malloc's
// a new dev + SPI desc each call; without retention each recovery cycle leaks).
static struct ad9523_dev *g_ad9523_dev = NULL;

//ADAR1000

ADAR1000Manager adarManager;
ADAR1000_AGC    outerAgc;
static uint8_t matrix1[15][16];
static uint8_t matrix2[15][16];
static uint8_t vector_0[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//Temperature Sensors
ADS7830_HandleTypeDef hadc3;
float Temperature_1 = 0.0f, Temperature_2 = 0.0f, Temperature_3 = 0.0f, Temperature_4 = 0.0f;
float Temperature_5 = 0.0f, Temperature_6 = 0.0f, Temperature_7 = 0.0f, Temperature_8 = 0.0f;

// Phase differences for 31 beam positions
const float phase_differences[31] = {
    160.0f, 80.0f, 53.333f, 40.0f, 32.0f, 26.667f, 22.857f, 20.0f, 17.778f, 16.0f,
    14.545f, 13.333f, 12.308f, 11.429f, 10.667f, 0.0f,
    -10.667f, -11.429f, -12.308f, -13.333f, -14.545f, -16.0f, -17.778f, -20.0f,
    -22.857f, -26.667f, -32.0f, -40.0f, -53.333f, -80.0f, -160.0f
};

// Convenience HAL wrappers for AD9523 control pins (GPIOF as you said)
static inline void AD9523_PD(bool v)        { HAL_GPIO_WritePin(GPIOF, AD9523_PD_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void AD9523_REF_SEL(bool v)   { HAL_GPIO_WritePin(GPIOF, AD9523_REF_SEL_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void AD9523_SYNC_PULSE(void)  { HAL_GPIO_WritePin(GPIOF, AD9523_SYNC_Pin, GPIO_PIN_SET); HAL_Delay(1); HAL_GPIO_WritePin(GPIOF, AD9523_SYNC_Pin, GPIO_PIN_RESET); }
static inline void AD9523_RESET_ASSERT()    { HAL_GPIO_WritePin(GPIOF, AD9523_RESET_Pin, GPIO_PIN_RESET); } // check polarity
static inline void AD9523_RESET_RELEASE()   { HAL_GPIO_WritePin(GPIOF, AD9523_RESET_Pin, GPIO_PIN_SET); }
static inline void AD9523_CS(bool v)        { HAL_GPIO_WritePin(GPIOF, AD9523_CS_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void AD9523_EEPROM_SEL(bool v){ HAL_GPIO_WritePin(GPIOF, AD9523_EEPROM_SEL_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_IWDG_Init(void);  /* GAP-3 FIX 2: hardware watchdog */
/* USER CODE BEGIN PFP */

// Function prototypes
void systemPowerUpSequence();
void systemPowerDownSequence();
void initializeBeamMatrices();
void runRadarPulseSequence();

/* PR-AB.b: FPGA chirp_scheduler frame_pulse (DIG_6 / PD14, ~100 ns stretched)
 * is wired to EXTI15_10. Each rising edge bumps g_frame_pulse_count from the
 * ISR; the dwell loop in runRadarPulseSequence waits for the counter to
 * advance instead of HAL_Delay-padding, so per-pattern dwell tracks the
 * actual mask-aware ladder length (drift-free, mask-aware). The ladder is
 * ~8 ms; we use a 20 ms timeout so a missed pulse cannot stall the loop. */
static volatile uint32_t g_frame_pulse_count = 0;

static inline void waitForFramePulse(uint32_t timeout_ms)
{
    uint32_t prev = g_frame_pulse_count;
    uint32_t t0   = HAL_GetTick();
    while (g_frame_pulse_count == prev) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return;  // timeout — caller continues; missed-pulse degrades to wall-clock cadence
        }
    }
}
/* F-2.1: executeChirpSequence() removed. The MCU is no longer in the chirp
 * dispatch path -- production runs FPGA mode 2'b01 (auto-scan) where
 * chirp_scheduler.v owns chirp timing and adar_tr_x. See runRadarPulseSequence
 * header for the rationale and the chirp_scheduler.v RP_MODE_AUTO_3KM block
 * (line ~317) for the FPGA-side behaviour. */
void printSystemStatus();

//////////////////////////////////////////////
////////////////////micros()//////////////////
//////////////////////////////////////////////
unsigned long micros(void){
unsigned long Micros = __HAL_TIM_GET_COUNTER(&htim1);
return Micros; //Clock TIM1 -> AHB/APB1 is set to 72MHz/presc+1   presc = 71
}

//////////////////////////////////////////////
//////////////////Delay_us()//////////////////
//////////////////////////////////////////////

void delay_us(volatile uint32_t us){
// MCU-N4: chunk requests larger than the TIM1 ARR (0xffff-1 = 65534 ticks
// at 1 MHz = ~65 ms). Without this, GET_COUNTER never reaches `us` once
// it wraps and the loop spins forever — a hazard with the PA energized.
const uint32_t ARR_TICKS = 0xffff - 1;
while (us > ARR_TICKS) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < ARR_TICKS);
    us -= ARR_TICKS;
}
__HAL_TIM_SET_COUNTER(&htim1, 0);  // set the counter value a
while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // //Clock TIMx -> AHB/APB1 is set to 72MHz/presc+1   presc = 71
}

//////////////////////////////////////////////
//////////////////delay_ns()//////////////////
//////////////////////////////////////////////
void delay_ns(uint32_t nanoseconds)
{
    // Get the starting cycle count
    uint32_t start_cycles = DWT->CYCCNT;

    // Convert nanoseconds to the number of CPU clock cycles
    // Use `SystemCoreClock` to get the current CPU frequency.
    // Use 64-bit math to prevent overflow for large nanosecond values.
    uint32_t cycles = (uint32_t)(((uint64_t)nanoseconds * SystemCoreClock) / 1000000000);

    // Busy-wait until the required number of cycles has passed
    while ((DWT->CYCCNT - start_cycles) < cycles);
}

//////////////////////////////////////////////
//////////////////////RADAR///////////////////
//////////////////////////////////////////////
uint8_t degreesTo7BitPhase(float degrees) {
    // Normalize to 0-360 range
    while(degrees < 0) degrees += 360.0f;
    while(degrees >= 360.0f) degrees -= 360.0f;

    // Convert to 7-bit (0-127)
    uint8_t phase_7bit = (uint8_t)((degrees / 360.0f) * 128.0f);

    return phase_7bit % 128;
}

extern "C" {

    // USB CDC receive callback (called by STM32 HAL)
    void CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
        DIAG("USB", "CDC_Receive_FS callback: %lu bytes received", *Len);
        // Process received USB data
        usbHandler.processUSBData(Buf, *Len);

        // Prepare for next reception
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &usb_rx_buffer[0]);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }

    // PR-AB.b: EXTI callback for FPGA frame_pulse on PD14 (DIG_6).
    // EXTI15_10_IRQHandler in stm32f7xx_it.c calls HAL_GPIO_EXTI_IRQHandler,
    // which dispatches here. Bumping g_frame_pulse_count releases the dwell
    // loop's waitForFramePulse spin. PD15 (DIG_7 fault flag) stays polled
    // by checkSystemHealth — no EXTI on that line.
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        if (GPIO_Pin == FPGA_FRAME_PULSE_Pin) {
            g_frame_pulse_count++;
        }
    }
}

void systemPowerUpSequence() {
    DIAG_SECTION("PWR: systemPowerUpSequence");
    uint8_t msg[] = "Starting Power Up Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg)-1, 1000);

    // Step 1: ADTR1107 GPIO + power-rail bring-up. Bias-register writes are
    // deferred to applyADTRBiasDefaults() because the next step's soft reset
    // would wipe them.
    DIAG("PWR", "Step 1: initializeADTR1107Sequence()  (rails only)");
    if (!adarManager.initializeADTR1107Sequence()) {
        DIAG_ERR("PWR", "ADTR1107 power sequence FAILED -- calling Error_Handler()");
        uint8_t err[] = "ERROR: ADTR1107 power sequence failed!\r\n";
        HAL_UART_Transmit(&huart3, err, sizeof(err)-1, 1000);
        Error_Handler();
    }
    DIAG("PWR", "Step 1 OK: ADTR1107 rails up");

    // Step 2: Soft-reset + per-device init (sets TR_SOURCE=1, ADC, VGA gains).
    // Ends with setAllDevicesTXMode() inside initializeAllDevices, which sets
    // PA bias to operational; the next step trims to the Idq-cal target.
    DIAG("PWR", "Step 2: initializeAllDevices()");
    if (!adarManager.initializeAllDevices()) {
        DIAG_ERR("PWR", "ADAR1000 initialization FAILED -- calling Error_Handler()");
        uint8_t err[] = "ERROR: ADAR1000 initialization failed!\r\n";
        HAL_UART_Transmit(&huart3, err, sizeof(err)-1, 1000);
        Error_Handler();
    }
    DIAG("PWR", "Step 2 OK: All ADAR1000 devices initialized");

    // Step 3: Re-emit ADTR1107 LNA-off + PA-Idq-cal bias values (the Step 5/7/9
    // writes that the original 9-step sequence put before init, where they
    // were getting wiped by the soft reset). F-1.7 in startup audit.
    DIAG("PWR", "Step 3: applyADTRBiasDefaults()  (F-1.7: post-soft-reset bias trim)");
    if (!adarManager.applyADTRBiasDefaults()) {
        DIAG_WARN("PWR", "ADTR bias defaults returned issues (non-fatal)");
    }

    // Step 4: Verify ADAR1000 SPI scratchpad on every device.
    DIAG("PWR", "Step 4: performSystemCalibration()");
    if (!adarManager.performSystemCalibration()) {
        DIAG_WARN("PWR", "System calibration returned issues (non-fatal)");
        uint8_t warn[] = "WARNING: System calibration issues\r\n";
        HAL_UART_Transmit(&huart3, warn, sizeof(warn)-1, 1000);
    } else {
        DIAG("PWR", "Step 4 OK: System calibration passed");
    }

    // (Former Step 4 setAllDevicesTXMode removed — F-1.8: it is already
    // called at the end of initializeAllDevices(). The chip's TX/RX state is
    // overridden per-chirp by the FPGA TR pin (TR_SOURCE=1), so the static
    // mode chosen here is mostly cosmetic.)

    uint8_t success[] = "Power Up Sequence Completed Successfully\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success)-1, 1000);
    DIAG("PWR", "systemPowerUpSequence COMPLETE -- ready for radar loop");
}

void systemPowerDownSequence() {
    DIAG_SECTION("PWR: systemPowerDownSequence");
    uint8_t msg[] = "Starting Power Down Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg)-1, 1000);

    // Step 1: Set all devices to RX mode (safest state)
    DIAG("PWR", "Step 1: setAllDevicesRXMode()");
    adarManager.setAllDevicesRXMode();
    HAL_Delay(10);

    // Step 2: Disable PA power supplies
    DIAG("PWR", "Step 2: Disable PA 5V supplies (EN_P_5V0_PA1/PA2/PA3 LOW)");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin | EN_P_5V0_PA2_Pin | EN_P_5V0_PA3_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Step 3: Set PA biases to safe values
    DIAG("PWR", "Step 3: Setting PA bias to safe value 0x20 on all 4 devices, 4 channels each");
    for (uint8_t dev = 0; dev < 4; dev++) {
        adarManager.adarWrite(dev, REG_PA_CH1_BIAS_ON, 0x20, BROADCAST_OFF);
        adarManager.adarWrite(dev, REG_PA_CH2_BIAS_ON, 0x20, BROADCAST_OFF);
        adarManager.adarWrite(dev, REG_PA_CH3_BIAS_ON, 0x20, BROADCAST_OFF);
        adarManager.adarWrite(dev, REG_PA_CH4_BIAS_ON, 0x20, BROADCAST_OFF);
    }
    HAL_Delay(10);

    // Step 4: Disable LNA power supply
    DIAG("PWR", "Step 4: Disable LNA 3.3V supply (EN_P_3V3_ADTR LOW)");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Step 5: Set LNA bias to safe value
    DIAG("PWR", "Step 5: Setting LNA bias to 0x00 on all 4 devices");
    for (uint8_t dev = 0; dev < 4; dev++) {
        adarManager.adarWrite(dev, REG_LNA_BIAS_ON, 0x00, BROADCAST_OFF);
    }
    HAL_Delay(10);

    // Step 6: Disable switch power supplies
    DIAG("PWR", "Step 6: Disable switch supplies (EN_P_3V3_VDD_SW, EN_P_3V3_SW LOW)");
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    uint8_t success[] = "Power Down Sequence Completed\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success)-1, 1000);
    DIAG("PWR", "systemPowerDownSequence COMPLETE");
}

void initializeBeamMatrices() {
    uint8_t msg[] = "Initializing Beam Matrices...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg)-1, 1000);

    // Matrix1: phase_differences[0..14] (+160°..+0°). Array math gives beam
    // peak at NEGATIVE θ (matrix1[0] → −62°, matrix1[14] → broadside).
    // Physical sky/ground sign vs antenna mount is unverified (audit D2).
    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
        float phase_diff_degrees = phase_differences[beam_pos];

        for(int element = 0; element < 16; element++) {
            float cumulative_phase_degrees = element * phase_diff_degrees;
            matrix1[beam_pos][element] = degreesTo7BitPhase(cumulative_phase_degrees);
        }
    }

    // Matrix2: phase_differences[16..30] (−10.67°..−160°). Array math gives
    // beam peak at POSITIVE θ (matrix2[0] → +4°, matrix2[14] → +62°).
    // Same orientation caveat as Matrix1.
    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
        float phase_diff_degrees = phase_differences[beam_pos + 16];

        for(int element = 0; element < 16; element++) {
            float cumulative_phase_degrees = element * phase_diff_degrees;
            matrix2[beam_pos][element] = degreesTo7BitPhase(cumulative_phase_degrees);
        }
    }

    // Vector_0: Position 16 (zero phase - broadside)
    for(int element = 0; element < 16; element++) {
        vector_0[element] = 0; // 0 in 7-bit = 0° phase
    }

    uint8_t done[] = "Beam Matrices Initialized\r\n";
    HAL_UART_Transmit(&huart3, done, sizeof(done)-1, 1000);
}

/* F-2.1 (delete branch): runRadarPulseSequence — per-frame beam-steering
 * loop + stepper rotation.
 *
 * The MCU does NOT drive chirp timing. Production runs FPGA mode 2'b01
 * (`RP_MODE_AUTO_3KM`, the cold-reset default at radar_system_top.v:1058).
 * In that mode chirp_scheduler.v owns the SHORT/MEDIUM/LONG sub-frame walk
 * and plfm_chirp_controller.v drives `adar_tr_x` per chirp. The MCU's only
 * live per-frame work is:
 *   1. Updating ADAR1000 beam patterns (matrix1 / vector_0 / matrix2)
 *      via setCustomBeamPattern16 — once per pattern, three patterns per
 *      beam position, 15 beam positions per frame = 90 ADAR SPI writes.
 *   2. Rotating the stepper to the next azimuth slot at end-of-frame.
 *
 * History: this function previously called executeChirpSequence() to emit
 * GPIO toggles on PD8/PD9/PD10 intended for FPGA mode 2'b00 (STM32
 * pass-through). Mode 2'b00 was never end-to-end validated — the GPIO loop
 * emitted only 2 PRIs (LONG, SHORT) and was structured for the pre-PR-F
 * m_max=32 frame, while the production receiver chain (PR-F doppler/mti/
 * cfar) expects 3 sub-frames (SHORT/MEDIUM/LONG) for the CRT Doppler
 * unfolder. Building out a real 3-PRI pass-through would require a hardware
 * timer driving PD8 (software delay_us is 3% jittery, unacceptable for
 * Doppler) and full agreement between MCU and FPGA on chirps_per_subframe.
 * That work was decided not worth doing — see audit memory
 * project_aeris10_startup_audit_2026-05-04.md F-2.1 rationale.
 *
 * The m / n / y globals are kept and incremented (they feed
 * getSystemStatusForGUI as ChirpCount / BeamPos / Azimuth). m advances by
 * the same per-frame amount (3 × m_max/2) it did before so the GUI counter
 * stays continuous; the actual count of chirps fired is owned by the FPGA
 * and surfaces via the bulk-frame status word, not via this counter.
 */
void runRadarPulseSequence() {
    static int sequence_count = 0;
    char msg[50];

    snprintf(msg, sizeof(msg), "Starting RADAR Sequence #%d\r\n", ++sequence_count);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
    DIAG("SYS", "runRadarPulseSequence #%d: m_max=%d n_max=%d y_max=%d",
         sequence_count, m_max, n_max, y_max);

    DIAG("BF", "Beam sweep start (FPGA owns per-chirp T/R switching)");

    // MCU-N5/C4: do NOT redeclare m/n/y here — they are file-scope globals
    // (see lines ~190-192) that getStatusString() emits to the GUI as
    // BeamPos/Azimuth/ChirpCount. Local declarations would shadow them and
    // freeze the telemetry at 1.

    /* Main beam steering sequence. Per azimuth slot we fire one broadside
     * reference (vector_0) and then sweep 15 beam positions, each firing
     * matrix1 (negative-θ scan) and matrix2 (positive-θ scan). PR-AB.a
     * pulled vector_0 out of the inner loop — previously fired 15× per
     * azimuth (between every matrix1/matrix2 pair) which dominated the
     * 18.4 s revisit time without serving a documented per-pos purpose.
     * One broadside frame per azimuth is sufficient as a clutter
     * reference; if multiple per-pos broadside frames are ever needed
     * they should re-enter behind a runtime switch.
     *
     * Per-pattern dwell tracks the actual chirp_scheduler ladder length
     * (drift-free, mask-aware) by waiting on the FPGA frame_pulse on
     * DIG_6 / PD14 — see waitForFramePulse + HAL_GPIO_EXTI_Callback above.
     * Nominal ladder is:
     *   16 × PRI_SHORT  (175 us) +
     *   16 × PRI_MEDIUM (161 us) +
     *   16 × PRI_LONG   (167 us) = 8048 us
     * but with subframe_enable masking a single PRI it can drop to ~2.7 ms,
     * which a fixed HAL_Delay(8) used to over-pad. The 20 ms timeout below
     * is a safety floor — if a frame_pulse ever goes missing (FPGA reset,
     * PD14 disconnected) the loop degrades to wall-clock cadence rather
     * than hanging. */
    static const uint32_t FRAME_PULSE_TIMEOUT_MS = 20u;

    // One broadside (vector_0) reference frame per azimuth — replaces the
    // 15 in-loop fires that PR-AB.a removed.
    DIAG("SYS", "Broadside reference (vector_0) — 1× per azimuth");
    adarManager.setCustomBeamPattern16(vector_0, ADAR1000Manager::BeamDirection::TX);
    adarManager.setCustomBeamPattern16(vector_0, ADAR1000Manager::BeamDirection::RX);
    // PR-AB.b expanded commit 5: announce "beam pattern ready" to the FPGA so
    // chirp_scheduler can release S_BEAM_WAIT. Harmless when the FPGA's
    // host_handshake_enable=0 (legacy open-loop default).
    HAL_GPIO_TogglePin(FPGA_BEAM_READY_GPIO_Port, FPGA_BEAM_READY_Pin);
    waitForFramePulse(FRAME_PULSE_TIMEOUT_MS);
    m += m_max/2;

    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
    	// PD9 toggle (formerly new_elevation) retired in PR-AB.b expanded —
    	// FPGA-side input port + counter were removed, so the toggle was a no-op.
    	DIAG("SYS", "Beam pos %d/15: patterns matrix1/matrix2", beam_pos);
        // Pattern 1: matrix1 (negative-θ scan, peak at -62°..-3°)
        adarManager.setCustomBeamPattern16(matrix1[beam_pos], ADAR1000Manager::BeamDirection::TX);
        adarManager.setCustomBeamPattern16(matrix1[beam_pos], ADAR1000Manager::BeamDirection::RX);
        HAL_GPIO_TogglePin(FPGA_BEAM_READY_GPIO_Port, FPGA_BEAM_READY_Pin);  // commit 5 handshake
        waitForFramePulse(FRAME_PULSE_TIMEOUT_MS);
        m += m_max/2;

        // Pattern 2: matrix2 (positive-θ scan, peak at +3°..+62°)
        adarManager.setCustomBeamPattern16(matrix2[beam_pos], ADAR1000Manager::BeamDirection::TX);
        adarManager.setCustomBeamPattern16(matrix2[beam_pos], ADAR1000Manager::BeamDirection::RX);
        HAL_GPIO_TogglePin(FPGA_BEAM_READY_GPIO_Port, FPGA_BEAM_READY_Pin);  // commit 5 handshake
        waitForFramePulse(FRAME_PULSE_TIMEOUT_MS);
        m += m_max/2;

        // Reset chirp counter if needed
        if(m > m_max) m = 1;

        n++;
        if(n > n_max) n = 1;

    }

    // PD10 toggle (formerly new_azimuth) retired in PR-AB.b expanded — see PD9 note above.
    DIAG("SYS", "Azimuth step, stepping motor");

    y++; if(y>y_max)y=1;
	  //Rotate stepper to next y position
	  for(int k= 0;k<(int)(Stepper_steps/y_max);k++){
		  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_SET);
		  delay_us(500);
		  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_RESET);
		  delay_us(500);
	  }
    DIAG("MOT", "Stepper moved %d steps for azimuth position y=%d", (int)(Stepper_steps/y_max), y);



    snprintf(msg, sizeof(msg), "RADAR Sequence #%d Completed\r\n", sequence_count);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
    DIAG("SYS", "runRadarPulseSequence #%d COMPLETE", sequence_count);
}




void printSystemStatus() {
    char status_msg[100];

    // Print device status
    for(int i = 0; i < 4; i++) {
        if(adarManager.verifyDeviceCommunication(i)) {
            float temp = adarManager.readTemperature(i);
            snprintf(status_msg, sizeof(status_msg),
                    "ADAR1000 #%d: OK, Temperature: %.1fC\r\n", i+1, temp);
            HAL_UART_Transmit(&huart3, (uint8_t*)status_msg, strlen(status_msg), 1000);
        } else {
            snprintf(status_msg, sizeof(status_msg),
                    "ADAR1000 #%d: COMMUNICATION ERROR\r\n", i+1);
            HAL_UART_Transmit(&huart3, (uint8_t*)status_msg, strlen(status_msg), 1000);
        }
    }

    // Print matrix info
    snprintf(status_msg, sizeof(status_msg),
            "Beam Matrices: 31 positions, %d chirps/position\r\n", m_max);
    HAL_UART_Transmit(&huart3, (uint8_t*)status_msg, strlen(status_msg), 1000);
}

/***************************************************************/
/**********************SYSTEM ERROR HANDLER*********************/
/***************************************************************/

typedef enum {
    ERROR_NONE = 0,
    ERROR_AD9523_CLOCK,
    ERROR_ADF4382_TX_UNLOCK,
    ERROR_ADF4382_RX_UNLOCK,
    ERROR_ADAR1000_COMM,
    ERROR_ADAR1000_TEMP,
    ERROR_IMU_COMM,
    ERROR_BMP180_COMM,
    ERROR_GPS_COMM,
    ERROR_RF_PA_OVERCURRENT,
    ERROR_RF_PA_BIAS,
    ERROR_STEPPER_MOTOR,
    ERROR_FPGA_COMM,
    ERROR_POWER_SUPPLY,
    ERROR_TEMPERATURE_HIGH,
    ERROR_MEMORY_ALLOC,
    ERROR_WATCHDOG_TIMEOUT,
    /* AUDIT-S10 follow-up: gpio_dig7 (PD15) reports FPGA DSP control faults
     * (range-decimator watchdog, CIC->FIR CDC overrun). Placed AFTER the
     * ERROR_RF_PA_OVERCURRENT..ERROR_POWER_SUPPLY critical range so the
     * dispatcher routes to attemptErrorRecovery (FPGA reset pulse) instead
     * of Emergency_Stop. */
    ERROR_FPGA_DSP_STALL,
    ERROR_COUNT  // must be last — used for bounds checking error_strings[]
} SystemError_t;

static SystemError_t last_error = ERROR_NONE;
static uint32_t error_count = 0;
static bool system_emergency_state = false;

/* MCU-A5: gate that suppresses Idq overcurrent / bias-fault evaluation in
 * checkSystemHealth while the boot-time PA calibration walk is running.
 * The walk starts each channel at DAC_val=126 and steps DOWN; the early
 * iterations sit well above the 2.5 A overcurrent threshold by design,
 * and a channel that hits the safety-counter timeout (50 iters) can be
 * left at >2.5 A. Without this gate, the next periodic health check
 * trips ERROR_RF_PA_OVERCURRENT → Emergency_Stop. Set TRUE around the
 * cal loops; checkSystemHealth treats overcurrent/bias as advisory while
 * set. Temperature, comm, and lock checks remain fully active. */
static volatile bool pa_calibration_in_progress = false;

// Error handler function
SystemError_t checkSystemHealth(void) {
    SystemError_t current_error = ERROR_NONE;

    // 0. Watchdog: detect main-loop stall (checkSystemHealth not called for >60 s).
    //    Timestamp is captured at function ENTRY and updated unconditionally, so
    //    any early return from a sub-check below cannot leave a stale value that
    //    would later trip a spurious ERROR_WATCHDOG_TIMEOUT. A dedicated cold-start
    //    branch ensures the first call after boot never trips (last_health_check==0
    //    would otherwise make `HAL_GetTick() - 0 > 60000` true forever after the
    //    60-s mark of the init sequence).
    static uint32_t last_health_check = 0;
    uint32_t now_tick = HAL_GetTick();
    if (last_health_check == 0) {
        last_health_check = now_tick;          // cold start: seed only
    } else {
        uint32_t elapsed = now_tick - last_health_check;
        last_health_check = now_tick;          // update BEFORE any early return
        if (elapsed > 60000) {
            current_error = ERROR_WATCHDOG_TIMEOUT;
            DIAG_ERR("SYS", "Health check: Watchdog timeout (>60s since last check)");
            return current_error;
        }
    }

    // 1. Check AD9523 Clock Generator
    //
    // AUDIT-CAL follow-up: update last_clock_check on BOTH success and error
    // paths; pre-fix the assignment was below the early return so a STATUS0/1
    // RESET would re-fire ERROR_AD9523_CLOCK every main-loop iteration. Same
    // bug class as AUDIT-CAL (BMP180 watchdog at main.cpp:771-780).
    static uint32_t last_clock_check = 0;
    if (HAL_GetTick() - last_clock_check > 5000) {
        last_clock_check = HAL_GetTick();   // commit the rate-limit window first
        GPIO_PinState s0 = HAL_GPIO_ReadPin(AD9523_STATUS0_GPIO_Port, AD9523_STATUS0_Pin);
        GPIO_PinState s1 = HAL_GPIO_ReadPin(AD9523_STATUS1_GPIO_Port, AD9523_STATUS1_Pin);
        DIAG_GPIO("CLK", "AD9523 STATUS0", s0);
        DIAG_GPIO("CLK", "AD9523 STATUS1", s1);
        if (s0 == GPIO_PIN_RESET || s1 == GPIO_PIN_RESET) {
            current_error = ERROR_AD9523_CLOCK;
            DIAG_ERR("CLK", "AD9523 clock health check FAILED (STATUS0=%d STATUS1=%d)", s0, s1);
            return current_error;
        }
    }

    // 2. Check ADF4382 Lock Status
    bool tx_locked, rx_locked;
    if (ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked) == ADF4382A_MANAGER_OK) {
        if (!tx_locked) {
            current_error = ERROR_ADF4382_TX_UNLOCK;
            DIAG_ERR("LO", "Health check: TX LO UNLOCKED");
            return current_error;
        }
        if (!rx_locked) {
            current_error = ERROR_ADF4382_RX_UNLOCK;
            DIAG_ERR("LO", "Health check: RX LO UNLOCKED");
            return current_error;
        }
    }

    // 3. Check ADAR1000 Communication and Temperature
    // [MCU-N7 FIX] verifyDeviceCommunication() writes the SCRATCHPAD register
    // and HAL_Delay(1) per device. Across 4 devices that is >=4 ms of
    // blocking SPI per main-loop iteration (chirp jitter source). Rate-limit
    // the comm check to every 2 s (matches the clock-check pattern at line
    // 658). readTemperature() is single-register SPI read with no HAL_Delay,
    // so it stays per-loop to keep PA over-temperature detection responsive.
    // AUDIT-CAL follow-up: commit the comm-check rate-limit window BEFORE the
    // for-loop, not after. Pre-fix, the `last_adar_comm_check = HAL_GetTick();`
    // sat below the loop so any early return (comm fail, overtemp) skipped it
    // and the 2 s rate-limit became broken — every subsequent main-loop
    // iteration re-ran the SCRATCHPAD verify+HAL_Delay sequence (4 ms blocking
    // SPI per device × 4 devices = 16 ms of chirp jitter per iteration), AND
    // re-fired ERROR_ADAR1000_COMM. Overtemp is intentionally per-loop (not
    // rate-limited) so PA over-temperature stays responsive; that path is
    // unaffected by this fix.
    static uint32_t last_adar_comm_check = 0;
    bool run_comm_check = (HAL_GetTick() - last_adar_comm_check > 2000);
    if (run_comm_check) {
        last_adar_comm_check = HAL_GetTick();   // commit window first
    }
    for (int i = 0; i < 4; i++) {
        if (run_comm_check) {
            if (!adarManager.verifyDeviceCommunication(i)) {
                current_error = ERROR_ADAR1000_COMM;
                DIAG_ERR("BF", "Health check: ADAR1000 #%d comm FAILED", i);
                return current_error;
            }
        }

        float temp = adarManager.readTemperature(i);
        if (temp > 85.0f) {
            current_error = ERROR_ADAR1000_TEMP;
            DIAG_ERR("BF", "Health check: ADAR1000 #%d OVERTEMP %.1fC > 85C", i, temp);
            return current_error;
        }
    }

    // 4. Check IMU Communication
    //
    // AUDIT-CAL follow-up: update last_imu_check on BOTH success and error
    // paths so a transient IMU failure (I2C noise / cable pull) does not
    // re-fire ERROR_IMU_COMM every main-loop iteration. Pre-fix, the
    // assignment was below the early return and only ran on success — once
    // GY85_Update started failing, the 10 s rate-limit never engaged and
    // ~10 main-loop iterations later error_count > 10 latched
    // system_emergency_state per MCU-N1. Same bug class as AUDIT-CAL
    // (BMP180 watchdog at main.cpp:771-780); same fix pattern.
    static uint32_t last_imu_check = 0;
    if (HAL_GetTick() - last_imu_check > 10000) {
        last_imu_check = HAL_GetTick();   // commit the rate-limit window first
        if (!GY85_Update(&imu)) {
            current_error = ERROR_IMU_COMM;
            DIAG_ERR("IMU", "Health check: GY85_Update() FAILED");
            return current_error;
        }
    }

    // 5. Check BMP180 Communication
    //
    // AUDIT-CAL fix: gate on bmp180_operational so an absent or boot-failed
    // sensor does not churn ERROR_BMP180_COMM (which would trip
    // error_count > 10 → SAFE-MODE latch within seconds). Also update
    // last_bmp_check on BOTH success AND error paths — previously it was
    // only assigned on success, so an out-of-range pressure caused the
    // watchdog to re-fire every iteration of the main while(1) loop instead
    // of the intended once-per-15s cadence.
    static uint32_t last_bmp_check = 0;
    if (bmp180_operational && (HAL_GetTick() - last_bmp_check > 15000)) {
        double pressure = myBMP.getPressure();
        last_bmp_check = HAL_GetTick();   // commit the rate-limit window first
        if (pressure < 30000.0 || pressure > 110000.0 || isnan(pressure)) {
            current_error = ERROR_BMP180_COMM;
            DIAG_ERR("SYS", "Health check: BMP180 pressure out of range: %.0f", pressure);
            return current_error;
        }
    }

    // 6. Check GPS Communication (30s grace period from boot / last valid fix)
    uint32_t gps_fix_age = um982_position_age(&um982);
    if (gps_fix_age > 30000) {
        current_error = ERROR_GPS_COMM;
        DIAG_WARN("SYS", "Health check: GPS no fix for >30s (age=%lu ms)", (unsigned long)gps_fix_age);
        return current_error;
    }

    // 7. Check RF Power Amplifier Current
    //    MCU-A5: skip the Idq window while pa_calibration_in_progress is set.
    //    The cal walk starts at DAC_val=126 (well into overcurrent) and steps
    //    DOWN to the 1.68 A target; mid-walk readings are not a fault. A
    //    channel left high by the safety-counter timeout is logged separately
    //    and surfaces on the next post-cal health check.
    if (PowerAmplifier && !pa_calibration_in_progress) {
        for (int i = 0; i < 16; i++) {
            if (Idq_reading[i] > 2.5f) {
                current_error = ERROR_RF_PA_OVERCURRENT;
                DIAG_ERR("PA", "Health check: PA ch%d OVERCURRENT Idq=%.3fA > 2.5A", i, Idq_reading[i]);
                return current_error;
            }
            if (Idq_reading[i] < 0.1f) {
                current_error = ERROR_RF_PA_BIAS;
                DIAG_ERR("PA", "Health check: PA ch%d BIAS FAULT Idq=%.3fA < 0.1A", i, Idq_reading[i]);
                return current_error;
            }
        }
    }

    // 8. Check System Temperature
    if (temperature > 75.0f) {
        current_error = ERROR_TEMPERATURE_HIGH;
        DIAG_ERR("SYS", "Health check: System OVERTEMP %.1fC > 75C", temperature);
        return current_error;
    }

    // 9. Watchdog check is performed at function entry (see step 0).

    // 10. AUDIT-S10 follow-up: FPGA DSP control-fault flag (PD15 / FPGA_DIG7).
    //     gpio_dig7 = (range_decim_watchdog | cic_fir_overrun), sticky in the
    //     FPGA source domain until a full bitstream reset clears it. Distinct
    //     from PD13 (signal-saturation; AGC reacts) — these are control-path
    //     stalls requiring an FPGA reset, not a gain change. AGC was previously
    //     mis-reacting to these on the old aggregated PD13 line.
    //
    //     Recovery: attemptErrorRecovery(ERROR_FPGA_DSP_STALL) pulses PD12 to
    //     reload the bitstream — that clears all sticky monitors as a side
    //     effect (no MCU-driven reset_monitors path exists).
    //
    //     Debounce: 2 consecutive HIGH samples on a 1 s cadence so a single
    //     glitch does not provoke an FPGA reset. Counter restarts at 0 once
    //     fired (and naturally resets when PD15 goes LOW after reset).
    //     last_dsp_check is committed BEFORE the early return per the
    //     AUDIT-CAL pattern, so a flapping fault never bypasses rate-limit.
    static uint32_t last_dsp_check = 0;
    static uint8_t  dsp_stall_streak = 0;
    if (HAL_GetTick() - last_dsp_check > 1000) {
        last_dsp_check = HAL_GetTick();
        bool dsp_fault = (HAL_GPIO_ReadPin(FPGA_DIG7_GPIO_Port,
                                           FPGA_DIG7_Pin) == GPIO_PIN_SET);
        if (dsp_fault) {
            if (dsp_stall_streak < 2) dsp_stall_streak++;
        } else {
            dsp_stall_streak = 0;
        }
        if (dsp_stall_streak >= 2) {
            dsp_stall_streak = 0;          // arm for next assertion post-recovery
            current_error = ERROR_FPGA_DSP_STALL;
            DIAG_ERR("FPGA",
                "Health check: gpio_dig7 (PD15) HIGH 2x — DSP control fault");
            return current_error;
        }
    }

    if (current_error != ERROR_NONE) {
        DIAG_ERR("SYS", "checkSystemHealth returning error code %d", current_error);
    }
    return current_error;
}

// Error recovery function
void attemptErrorRecovery(SystemError_t error) {
    DIAG_SECTION("SYS: attemptErrorRecovery");
    DIAG("SYS", "Attempting recovery from error code %d", error);
    char recovery_msg[80];
    snprintf(recovery_msg, sizeof(recovery_msg),
             "Attempting recovery from error: %d\r\n", error);
    HAL_UART_Transmit(&huart3, (uint8_t*)recovery_msg, strlen(recovery_msg), 1000);

    switch (error) {
        case ERROR_ADF4382_TX_UNLOCK:
        case ERROR_ADF4382_RX_UNLOCK:
            // Re-initialize LO
            DIAG("LO", "Recovery: Re-initializing LO manager (SYNC_METHOD_TIMED)");
            ADF4382A_Manager_Init(&lo_manager, SYNC_METHOD_TIMED);
            HAL_Delay(100);
            DIAG("LO", "Recovery: LO re-init complete");
            break;

        case ERROR_ADAR1000_COMM:
            // Reset ADAR1000 communication
            DIAG("BF", "Recovery: Re-initializing all ADAR1000 devices");
            adarManager.initializeAllDevices();
            HAL_Delay(50);
            DIAG("BF", "Recovery: ADAR1000 re-init complete");
            break;

        case ERROR_IMU_COMM:
            // Re-initialize IMU
            DIAG("IMU", "Recovery: Re-initializing GY85 IMU");
            GY85_Init();
            HAL_Delay(100);
            DIAG("IMU", "Recovery: IMU re-init complete");
            break;

        case ERROR_GPS_COMM:
            // GPS will auto-recover when signal returns
            DIAG("SYS", "Recovery: GPS error -- no action (auto-recover on signal)");
            break;

        case ERROR_AD9523_CLOCK:
            /* MCU-A6: AD9523 lost lock (STATUS0/1 LOW). Assert reset, allow
             * the chip to settle, then re-run the full configure path —
             * configure_ad9523() releases reset, selects REFB, and reprograms
             * registers + waits for lock. If the second attempt also fails
             * the next health-check pass re-fires ERROR_AD9523_CLOCK and
             * downstream policy (handleSystemError) decides whether to
             * escalate; we deliberately do NOT escalate inline so a single
             * transient brown-out on the 100 MHz reference does not drop
             * straight into Emergency_Stop. */
            DIAG("CLK", "Recovery: asserting AD9523 reset");
            AD9523_RESET_ASSERT();
            HAL_Delay(10);
            DIAG("CLK", "Recovery: re-running configure_ad9523()");
            if (configure_ad9523() != 0) {
                DIAG_ERR("CLK", "Recovery: configure_ad9523() FAILED -- next health check will re-fire");
            } else {
                DIAG("CLK", "Recovery: AD9523 re-configure complete (lock pending verification)");
            }
            break;

        case ERROR_FPGA_COMM:
        case ERROR_FPGA_DSP_STALL:
            /* MCU-A6 + AUDIT-S10 follow-up:
             *   ERROR_FPGA_COMM: FPGA stopped responding (USB-CDC silence,
             *     status timeout).
             *   ERROR_FPGA_DSP_STALL: gpio_dig7 (PD15) HIGH for 2+ samples;
             *     range_decim_watchdog or cic_fir_overrun is sticky in the
             *     FPGA source domain. No standalone MCU->FPGA reset_monitors
             *     path exists, so the cheapest recovery is a full bitstream
             *     reload — same PD12 pulse as ERROR_FPGA_COMM, which clears
             *     all sticky monitors as a side effect.
             * Pulse FPGA reset line PD12 LOW->10 ms->HIGH (same pattern as
             * boot sequence, line ~2733). Bitstream re-initializes from flash.
             * We do NOT touch PA rails — MCU-N2/N11 sequences cold-boot reset
             * BEFORE PA Vdd, but at runtime the PAs are live and re-resetting
             * the FPGA briefly leaves adar_tr_x undefined for ~10 ms. The
             * trade-off is acceptable vs losing the radar entirely; for a
             * power-cycle-clean recovery the operator can issue Emergency_Stop. */
            DIAG("FPGA", "Recovery: pulsing FPGA reset on PD12 (LOW for 10 ms)");
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            DIAG("FPGA", "Recovery: FPGA reset released -- bitstream reload in progress");
            break;

        default:
            // For other errors, just log and continue
            DIAG_WARN("SYS", "Recovery: No specific handler for error %d", error);
            break;
    }

    snprintf(recovery_msg, sizeof(recovery_msg),
             "Recovery attempt completed.\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)recovery_msg, strlen(recovery_msg), 1000);
    DIAG("SYS", "attemptErrorRecovery COMPLETE");
}

////////////////////////////////////////////////////////////////////////////////
// MCU-A7: persistent emergency-state flag in BKPSRAM
//
// Survives any MCU reset (incl. IWDG, NVIC SYSRESETREQ, brown-out) but is
// lost on full power removal — exactly the recovery semantics we want for
// emergency stop: power-cycle to clear, every other reset path keeps the
// PAs disabled. Written once in Emergency_Stop and checked very early in
// main(), before any PA enable code runs.
////////////////////////////////////////////////////////////////////////////////
#define EMERGENCY_PERSIST_MAGIC   0xDEAD5A5AU
#define EMERGENCY_PERSIST_ADDR    ((volatile uint32_t *)BKPSRAM_BASE)

static void emergency_persist_init_clocks(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
}

static void emergency_persist_set(void) {
    emergency_persist_init_clocks();
    *EMERGENCY_PERSIST_ADDR = EMERGENCY_PERSIST_MAGIC;
}

static bool emergency_persist_check(void) {
    emergency_persist_init_clocks();
    return *EMERGENCY_PERSIST_ADDR == EMERGENCY_PERSIST_MAGIC;
}

////////////////////////////////////////////////////////////////////////////////
// MCU-A2: site-configurable magnetic declination, persisted in BKPSRAM
//
// Slot layout (BKPSRAM, 4-byte words):
//   [0] MCU-A7 emergency-state magic
//   [1] MCU-A2 mag-declination magic
//   [2] MCU-A2 mag-declination value (float, bit-cast as uint32)
//
// Survives every reset path; cleared only on main-power removal. Range
// clamped to ±30° before persisting (anything beyond that is a calibration
// error rather than a legitimate site value — physical declination ranges
// are roughly -25° to +25° worldwide).
////////////////////////////////////////////////////////////////////////////////
#define MAG_DECL_PERSIST_MAGIC    0xCAFEFACEU
#define MAG_DECL_MAGIC_ADDR       ((volatile uint32_t *)(BKPSRAM_BASE + 4))
#define MAG_DECL_VALUE_ADDR       ((volatile uint32_t *)(BKPSRAM_BASE + 8))

void set_mag_declination_deg(float deg) {
    if (deg >  MAG_DECLINATION_LIMIT_DEG) deg =  MAG_DECLINATION_LIMIT_DEG;
    if (deg < -MAG_DECLINATION_LIMIT_DEG) deg = -MAG_DECLINATION_LIMIT_DEG;
    emergency_persist_init_clocks();
    union { float f; uint32_t u; } cvt = { .f = deg };
    *MAG_DECL_VALUE_ADDR = cvt.u;
    *MAG_DECL_MAGIC_ADDR = MAG_DECL_PERSIST_MAGIC;
    Mag_Declination = deg;  /* keep legacy global in sync for any external readers */
}

////////////////////////////////////////////////////////////////////////////////
// MCU-A4: warm-restart flag for the OCXO 180 s warmup loop
//
// BKPSRAM survives any MCU reset (IWDG, SYSRESETREQ, brown-out) but is
// cleared by main-power removal. The OCXO oven keeps the crystal hot for
// at least the few seconds an MCU-only reset takes, so a warm-restart
// boot does not need the full 180 s frequency-settling soak. Power-cycle
// always forces the full warmup again, which is the safe default.
//
// Slot layout (BKPSRAM, 4-byte words):
//   [0] MCU-A7 emergency-state magic
//   [1] MCU-A2 mag-declination magic
//   [2] MCU-A2 mag-declination value
//   [3] MCU-A4 warmup-complete magic
////////////////////////////////////////////////////////////////////////////////
#define WARMUP_PERSIST_MAGIC      0xCA1C1F1EU
#define WARMUP_PERSIST_ADDR       ((volatile uint32_t *)(BKPSRAM_BASE + 12))

void warmup_persist_set(void) {
    emergency_persist_init_clocks();
    *WARMUP_PERSIST_ADDR = WARMUP_PERSIST_MAGIC;
}

bool warmup_persist_check(void) {
    emergency_persist_init_clocks();
    return *WARMUP_PERSIST_ADDR == WARMUP_PERSIST_MAGIC;
}

float get_mag_declination_deg(void) {
    emergency_persist_init_clocks();
    if (*MAG_DECL_MAGIC_ADDR == MAG_DECL_PERSIST_MAGIC) {
        union { float f; uint32_t u; } cvt = { .u = *MAG_DECL_VALUE_ADDR };
        /* defensive clamp in case BKPSRAM was corrupted by VBAT brown-out */
        float v = cvt.f;
        if (v >  MAG_DECLINATION_LIMIT_DEG) v =  MAG_DECLINATION_LIMIT_DEG;
        if (v < -MAG_DECLINATION_LIMIT_DEG) v = -MAG_DECLINATION_LIMIT_DEG;
        return v;
    }
    return MAG_DECLINATION_DEFAULT_DEG;
}

////////////////////////////////////////////////////////////////////////////////
//:::::RF POWER AMPLIFIER DAC5578 Emergency stop function using CLR pin/////////
////////////////////////////////////////////////////////////////////////////////
void Emergency_Stop(void) {
    DIAG_ERR("PA", ">>> EMERGENCY_STOP ACTIVATED <<<");
    /* MCU-A7: persist emergency state in BKPSRAM BEFORE the rail-cut sequence
     * so even if a stuck interrupt or bus fault prevents the cuts from
     * completing, the next reset boots straight into safe-hold rather than
     * re-running startup (which would re-energize the PAs). */
    emergency_persist_set();
    /* Immediately clear all DAC outputs to zero using hardware CLR */
    DIAG_ERR("PA", "Clearing DAC1 outputs via CLR pin");
    DAC5578_ActivateClearPin(&hdac1);
    DIAG_ERR("PA", "Clearing DAC2 outputs via CLR pin");
    DAC5578_ActivateClearPin(&hdac2);

    /* [GAP-3 FIX 1] Cut RF and PA power rails — DAC CLR alone is not enough.
     * With gate voltage cleared but VDD still energized, PAs can self-bias
     * or oscillate.  Disable everything in fast-to-slow order:
     *   1. TX mixers (stop RF immediately)
     *   2. PA 5V per-element supplies
     *   3. PA 5.5V bulk supply
     *   4. RFPA VDD enable
     */
    DIAG_ERR("PA", "Disabling TX mixers (GPIOD pin 11 LOW)");
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

    DIAG_ERR("PA", "Cutting PA 5V supplies (PA1/PA2/PA3 LOW)");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_RESET);

    DIAG_ERR("PA", "Cutting PA 5.5V supply (EN_P_5V5_PA LOW)");
    HAL_GPIO_WritePin(EN_P_5V5_PA_GPIO_Port, EN_P_5V5_PA_Pin, GPIO_PIN_RESET);

    DIAG_ERR("PA", "Disabling RFPA VDD (EN_DIS_RFPA_VDD LOW)");
    HAL_GPIO_WritePin(EN_DIS_RFPA_VDD_GPIO_Port, EN_DIS_RFPA_VDD_Pin, GPIO_PIN_RESET);

    DIAG_ERR("PA", "All PA rails cut -- entering infinite hold loop (power-cycle to clear)");
    /* MCU-A7: refresh IWDG in the healthy hold loop so a deliberate
     * Emergency_Stop does not cycle the MCU pointlessly. If the loop itself
     * hangs (stack corruption, bus fault), refresh stops, IWDG fires, and
     * the BKPSRAM persist flag set above routes the reset back into safe-
     * hold without re-enabling the PAs. */
    while (1) {
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(100);
    }
}

// Error response function
void handleSystemError(SystemError_t error) {
    if (error == ERROR_NONE) return;

    error_count++;
    last_error = error;
    DIAG_ERR("SYS", "handleSystemError: error=%d error_count=%lu", error, error_count);

    char error_msg[100];
    static const char* const error_strings[] = {
        "No error",
        "AD9523 Clock failure",
        "ADF4382 TX LO unlocked",
        "ADF4382 RX LO unlocked",
        "ADAR1000 communication error",
        "ADAR1000 temperature high",
        "IMU communication error",
        "BMP180 communication error",
        "GPS communication lost",
        "RF PA overcurrent detected",
        "RF PA bias fault",
        "Stepper motor fault",
        "FPGA communication error",
        "Power supply fault",
        "System temperature high",
        "Memory allocation failed",
        "Watchdog timeout",
        "FPGA DSP control fault"
    };

    static_assert(sizeof(error_strings) / sizeof(error_strings[0]) == ERROR_COUNT,
                  "error_strings[] and SystemError_t enum are out of sync");

    const char* err_name = (error >= 0 && error < (int)(sizeof(error_strings) / sizeof(error_strings[0])))
                           ? error_strings[error]
                           : "Unknown error";

    snprintf(error_msg, sizeof(error_msg),
             "ERROR #%d: %s (Count: %lu)\r\n",
             error, err_name, error_count);
    HAL_UART_Transmit(&huart3, (uint8_t*)error_msg, strlen(error_msg), 1000);

    // Blink LED pattern based on error code
    DIAG("SYS", "Blinking LED3 %d times for error code %d", (error % 5) + 1, error);
    for (int i = 0; i < (error % 5) + 1; i++) {
        HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
        HAL_Delay(200);
    }

    // Critical errors trigger emergency shutdown.
    //
    // Safety-critical range: any fault that can damage the PAs or leave the
    // system in an undefined state must cut the RF rails via Emergency_Stop().
    // This covers:
    //   ERROR_RF_PA_OVERCURRENT .. ERROR_POWER_SUPPLY (9..13) -- PA/supply faults
    //   ERROR_TEMPERATURE_HIGH  (14) -- >75 C on the PA thermal sensors;
    //                                  without cutting bias + 5V/5V5/RFPA rails
    //                                  the GaN QPA2962 stage can thermal-runaway.
    //   ERROR_WATCHDOG_TIMEOUT  (16) -- health-check loop has stalled (>60 s);
    //                                  transmitter state is unknown, safest to
    //                                  latch Emergency_Stop rather than rely on
    //                                  IWDG reset (which re-energises the rails).
    if ((error >= ERROR_RF_PA_OVERCURRENT && error <= ERROR_POWER_SUPPLY) ||
        error == ERROR_TEMPERATURE_HIGH ||
        error == ERROR_WATCHDOG_TIMEOUT) {
        DIAG_ERR("SYS", "CRITICAL ERROR (code %d: %s) -- initiating Emergency_Stop()", error, err_name);
        snprintf(error_msg, sizeof(error_msg),
                 "CRITICAL ERROR! Initiating emergency shutdown.\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)error_msg, strlen(error_msg), 1000);

        /* [GAP-3 FIX 5] Set flag BEFORE Emergency_Stop() — the function
         * never returns (infinite loop), so the line after it was dead code. */
        system_emergency_state = true;
        Emergency_Stop();
        /* NOTREACHED — Emergency_Stop() loops forever */
    }

    // For non-critical errors, attempt recovery
    if (!system_emergency_state) {
        DIAG("SYS", "Non-critical error -- attempting recovery");
        attemptErrorRecovery(error);
    }
}


// System health monitoring function
bool checkSystemHealthStatus(void) {
    SystemError_t error = checkSystemHealth();

    if (error != ERROR_NONE) {
        DIAG_ERR("SYS", "checkSystemHealthStatus: error detected (code %d), calling handleSystemError()", error);
        handleSystemError(error);

        // If we're in emergency state or too many errors, shutdown.
        // [MCU-N1 FIX] Latch system_emergency_state=true on the error_count>10
        // path too — otherwise the SAFE-MODE blink loop in main() exits in one
        // pass (its predicate is `while(system_emergency_state)`) and the main
        // loop continues running with PA rails already cut by
        // systemPowerDownSequence(), still toggling new_chirp via PD8.
        if (system_emergency_state || error_count > 10) {
            if (!system_emergency_state) {
                system_emergency_state = true;
                DIAG_ERR("SYS", "Latching system_emergency_state due to error_count > 10");
            }
            /* F-2.5 FIX: persist emergency state to BKPSRAM so the IWDG reset
             * that follows the SAFE-MODE blink loop in main() boots straight
             * back into safe-hold (Emergency_Stop on the persist-check at
             * ~line 1630) instead of re-running init and re-energising the
             * PA rails. The critical-error path through Emergency_Stop()
             * already persists; the non-critical-escalation path (this one)
             * was the missing leg, leaving the system in a reset cycle with
             * intermittent PA-on time while the underlying fault is unfixed.
             * Idempotent (just writes a magic word). */
            emergency_persist_set();
            DIAG_ERR("SYS", "checkSystemHealthStatus returning FALSE (emergency=true error_count=%lu)",
                     error_count);
            return false;
        }
    } else {
        /* F-2.7a FIX: telemetry hygiene — clear last_error / error_count
         * after recovery is confirmed stable (5 consecutive clean health
         * checks, ~1.3 s at 258 ms frame cadence). Without this,
         * getSystemStatusForGUI keeps reporting the stale last_error
         * indefinitely after a transient fault auto-recovers (e.g. an
         * ADAR1000 SPI glitch resolved by attemptErrorRecovery). The 5-iter
         * confirmation prevents an intermittently-failing peripheral from
         * looking healed between flaps. error_count is also reset so the
         * `error_count > 10` escalation gate measures *current* run-rate,
         * not lifetime cumulative since boot. */
        static uint32_t clean_streak = 0;
        if (last_error != ERROR_NONE) {
            if (++clean_streak >= 5) {
                DIAG("SYS", "Recovery confirmed stable -- clearing last_error=%d error_count=%lu",
                     (int)last_error, (unsigned long)error_count);
                last_error = ERROR_NONE;
                error_count = 0;
                clean_streak = 0;
            }
        }
    }

    return true;
}

// Get system status for GUI
// Get system status for GUI with 8 temperature variables
void getSystemStatusForGUI(char* status_buffer, size_t buffer_size) {
    // Build status string directly in the output buffer using offset-tracked
    // snprintf.  Each call returns the number of chars written (excluding NUL),
    // so we advance 'off' and shrink 'rem' to guarantee we never overflow.
    size_t off = 0;
    size_t rem = buffer_size;
    int w;

    // Basic status
    if (system_emergency_state) {
        w = snprintf(status_buffer + off, rem, "System Status: EMERGENCY_STOP|");
    } else {
        w = snprintf(status_buffer + off, rem, "System Status: NORMAL|");
    }
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // Error information
    w = snprintf(status_buffer + off, rem, "LastError:%d|ErrorCount:%lu|",
                 last_error, error_count);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // Sensor status
    w = snprintf(status_buffer + off, rem, "IMU:%.1f,%.1f,%.1f|GPS:%.6f,%.6f|ALT:%.1f|",
                 Pitch_Sensor, Roll_Sensor, Yaw_Sensor,
                 RADAR_Latitude, RADAR_Longitude, RADAR_Altitude);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // LO Status
    bool tx_locked, rx_locked;
    ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked);
    w = snprintf(status_buffer + off, rem, "LO_TX:%s|LO_RX:%s|",
                 tx_locked ? "LOCKED" : "UNLOCKED",
                 rx_locked ? "LOCKED" : "UNLOCKED");
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // Temperature readings (8 variables)
    Temperature_1 = ADS7830_Measure_SingleEnded(&hadc3, 0);
    Temperature_2 = ADS7830_Measure_SingleEnded(&hadc3, 1);
    Temperature_3 = ADS7830_Measure_SingleEnded(&hadc3, 2);
    Temperature_4 = ADS7830_Measure_SingleEnded(&hadc3, 3);
    Temperature_5 = ADS7830_Measure_SingleEnded(&hadc3, 4);
    Temperature_6 = ADS7830_Measure_SingleEnded(&hadc3, 5);
    Temperature_7 = ADS7830_Measure_SingleEnded(&hadc3, 6);
    Temperature_8 = ADS7830_Measure_SingleEnded(&hadc3, 7);

    // Format all 8 temperature variables
    w = snprintf(status_buffer + off, rem,
                 "T1:%.1f|T2:%.1f|T3:%.1f|T4:%.1f|T5:%.1f|T6:%.1f|T7:%.1f|T8:%.1f|",
                 Temperature_1, Temperature_2, Temperature_3, Temperature_4,
                 Temperature_5, Temperature_6, Temperature_7, Temperature_8);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // RF Power Amplifier status (if enabled)
    if (PowerAmplifier) {
        float avg_current = 0.0f;
        for (int i = 0; i < 16; i++) {
            avg_current += Idq_reading[i];
        }
        avg_current /= 16.0f;

        w = snprintf(status_buffer + off, rem, "PA_AvgCurrent:%.2f|PA_Enabled:%d|",
                     avg_current, PowerAmplifier);
        if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }
    }

    // Radar operation status
    w = snprintf(status_buffer + off, rem, "BeamPos:%d|Azimuth:%d|ChirpCount:%d|",
                 n, y, m);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // NUL termination guaranteed by snprintf, but be safe
    status_buffer[buffer_size - 1] = '\0';
}

/* ---------- UART printing helpers (Bug #8 FIXED: uncommented) ---------- */
static void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

static void uart_println(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    const char crlf[] = "\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)crlf, 2, HAL_MAX_DELAY);
}

/* ---------- Helper delay wrappers ---------- */
static inline void delay_ms(uint32_t ms) { HAL_Delay(ms); }



// smartDelay removed -- replaced by non-blocking um982_process() in main loop

// Small helper to enable DWT cycle counter for microdelay
static void DWT_Init(void)
{
    // Enable DWT CYCCNT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Configure and program AD9523 via no-os driver
static int configure_ad9523(void)
{
    int32_t ret;
    struct ad9523_dev *dev = NULL;

    // F-4.5: recovery path re-runs configure_ad9523(); free the previous
    // dev/SPI-desc allocation before letting setup() malloc a new pair.
    if (g_ad9523_dev != NULL) {
        DIAG("CLK", "Releasing previous ad9523_dev before re-setup");
        ad9523_remove(g_ad9523_dev);
        g_ad9523_dev = NULL;
    }

    static struct ad9523_platform_data pdata;
    memset(&pdata, 0, sizeof(pdata));

    // Channels array (must allocate AD9523_NUM_CHAN entries) — assigned to
    // pdata BEFORE ad9523_init() so its per-channel default loop iterates
    // over the right num_channels.
    static struct ad9523_channel_spec channels[AD9523_NUM_CHAN];
    pdata.channels = channels;
    pdata.num_channels = AD9523_NUM_CHAN;

    // Fill ad9523 init param
    struct ad9523_init_param init_param;
    memset(&init_param, 0, sizeof(init_param));

    // SPI init (no_os type)
    init_param.spi_init.max_speed_hz = 10000000; // 10 MHz SPI
    init_param.spi_init.chip_select = 0;
    init_param.spi_init.mode = NO_OS_SPI_MODE_0;
    init_param.spi_init.platform_ops = &stm32_spi_ops;
    init_param.spi_init.extra = &hspi4; // pass HAL handle via extra

    init_param.pdata = &pdata;

    // F-4.1: ad9523_init() unconditionally overwrites every field in pdata
    // (vcxo_freq=0, pll1_bypass_en=1, pll2_ndiv_b_cnt=4, every channel
    // field). Call it BEFORE the user overrides below. The previous
    // "customize → init → setup" ordering wiped every user value and left
    // PLL2 N=16 (target VCO 1.6 GHz, far below the 3.6-4.0 GHz band) →
    // PLL2 never locked → boot halted. This call seeds defaults; user
    // overrides land on top.
    DIAG("CLK", "Calling ad9523_init() -- seeds pdata defaults (must precede overrides)");
    {
        int32_t init_ret = ad9523_init(&init_param);
        DIAG("CLK", "ad9523_init() returned %ld", (long)init_ret);
        if (init_ret != 0) {
            DIAG_ERR("CLK", "ad9523_init() FAILED (ret=%ld)", (long)init_ret);
            return -1;
        }
    }

    // VCXO + refs
    pdata.vcxo_freq = 100000000; // 100 MHz VCXO on OSC_IN
    pdata.refa_diff_rcv_en = 0;  // REFA 10 MHz single-ended
    pdata.refb_diff_rcv_en = 0;  // REFB 100 MHz single-ended
    pdata.osc_in_diff_en = 0;

    // PLL1: bypassed for first bring-up — VCXO free-runs. ad9523_status()
    // skips PLL1/REFA/REFB checks (ad9523.c:262 gates on pll1_bypass_en==0).
    // F-4.3/4.4 deferred: switching to bypass=0 (PLL1 active, VCXO
    // disciplined to OCXO REFB) requires pll1_charge_pump_current_nA and
    // pll1_loop_filter_rzero values tuned against the actual VCXO Kvco
    // and loop-filter network — design after bench-verified OCXO + VCXO.
    pdata.pll1_bypass_en = 1;
    pdata.refa_r_div = 1;
    pdata.refb_r_div = 1;

    // PLL2: 100 MHz PFD (R2=1), N = 36 to get VCO = 3.6 GHz
    pdata.pll2_ndiv_a_cnt = 0;
    pdata.pll2_ndiv_b_cnt = 9; // 4*9 + 0 = 36
    pdata.pll2_r2_div = 0;     // R2=1
    pdata.pll2_charge_pump_current_nA = 3500;

    // F-4.2 + F-4.7: AD9523 OUT4-OUT9 source from M1 or M2 only (no VCO
    // direct path); M1/M2 ∈ {3,4,5}. m1=0 (memset/init default) sets
    // M1_PWR_DOWN_EN, killing channels 4-9. With m1=3 and VCO 3.6 GHz,
    // M1 output = 1.2 GHz; channel dividers below operate on this. m1=3
    // is the unique choice for the labelled frequency set (m1=4 fails
    // OUT4=400MHz; m1=5 fails OUT0/1=300MHz).
    pdata.pll2_vco_diff_m1 = 3;
    pdata.pll2_vco_diff_m2 = 3;

    // Loop filters: reasonable starting values from examples
    pdata.rpole2 = RPOLE2_900_OHM;
    pdata.rzero = RZERO_2000_OHM;
    pdata.cpole1 = CPOLE1_24_PF;
    pdata.rzero_bypass_en = 0;

    // Initialize channels to disabled defaults
    for (int i=0; i<AD9523_NUM_CHAN; ++i) {
        channels[i].channel_num = i;
        channels[i].driver_mode = TRISTATE;
        channels[i].channel_divider = 0;
        channels[i].divider_phase = 0;
        channels[i].use_alt_clock_src = 0;
        channels[i].output_dis = 1;
    }

    // Map your required outputs (VCO=3.6 GHz, m1=3 → M1 output 1.2 GHz)
    // OUT0 = ADF4382A_TX = 300 MHz LVDS (1.2 GHz / 4)
    channels[0].channel_divider = 4;
    channels[0].driver_mode = LVDS_7mA;
    channels[0].divider_phase = 0;
    channels[0].output_dis = 0;

    // OUT1 = ADF4382A_RX = 300 MHz LVDS (phase aligned with OUT0)
    channels[1].channel_divider = 4;
    channels[1].driver_mode = LVDS_7mA;
    channels[1].divider_phase = 0;
    channels[1].output_dis = 0;

    // OUT4 = ADC = 400 MHz LVDS (1.2 GHz / 3)
    channels[4].channel_divider = 3;
    channels[4].driver_mode = LVDS_7mA;
    channels[4].divider_phase = 0;
    channels[4].output_dis = 0;

    // OUT5 = FPGA_ADC = 400 MHz LVDS (phase aligned with OUT4)
    channels[5].channel_divider = 3;
    channels[5].driver_mode = LVDS_7mA;
    channels[5].divider_phase = 0;
    channels[5].output_dis = 0;

    // OUT6 = FPGA_SYSTEM_CLOCK = 100 MHz LVCMOS (1.2 GHz / 12)
    channels[6].channel_divider = 12;
    channels[6].driver_mode = CMOS_CONF1;
    channels[6].divider_phase = 0;
    channels[6].output_dis = 0;

    // OUT7 = FPGA_TEST_CLOCK = 20 MHz LVCMOS (1.2 GHz / 60)
    channels[7].channel_divider = 60;
    channels[7].driver_mode = CMOS_CONF1;
    channels[7].divider_phase = 0;
    channels[7].output_dis = 0;

    // OUT8 = SYNC_TX = 60 MHz LVDS (1.2 GHz / 20)
    channels[8].channel_divider = 20;
    channels[8].driver_mode = LVDS_4mA;
    channels[8].divider_phase = 0;
    channels[8].output_dis = 0;

    // OUT9 = SYNC_RX = 60 MHz LVDS (phase aligned with OUT8)
    channels[9].channel_divider = 20;
    channels[9].driver_mode = LVDS_4mA;
    channels[9].divider_phase = 0;
    channels[9].output_dis = 0;

    // OUT10 = DAC = 120 MHz LVCMOS (1.2 GHz / 10)
    channels[10].channel_divider = 10;
    channels[10].driver_mode = CMOS_CONF1;
    channels[10].divider_phase = 0;
    channels[10].output_dis = 0;

    // OUT11 = FPGA_DAC = 120 MHz LVCMOS (phase aligned with OUT10)
    channels[11].channel_divider = 10;
    channels[11].driver_mode = CMOS_CONF1;
    channels[11].divider_phase = 0;
    channels[11].output_dis = 0;

    DIAG_SECTION("AD9523 CONFIGURE");
    DIAG("CLK", "VCXO=%lu Hz, PLL2 N=%d (4*%d+%d)=%d, R2=%d, m1=%d -> VCO 3.6 GHz, M1 1.2 GHz",
         (unsigned long)pdata.vcxo_freq,
         4 * pdata.pll2_ndiv_b_cnt + pdata.pll2_ndiv_a_cnt,
         pdata.pll2_ndiv_b_cnt, pdata.pll2_ndiv_a_cnt,
         4 * pdata.pll2_ndiv_b_cnt + pdata.pll2_ndiv_a_cnt,
         pdata.pll2_r2_div,
         pdata.pll2_vco_diff_m1);
    DIAG("CLK", "Enabled channels: 0,1(/4=300M) 4,5(/3=400M) 6(/12=100M) 7(/60=20M) 8,9(/20=60M) 10,11(/10=120M)");

    /* [Bug #2 FIXED] Removed first ad9523_setup() call that was here.
     * It wrote to the chip while still in reset — writes were lost.
     * Only the post-reset setup call below is needed. */

    // Bring AD9523 out of reset
    DIAG("CLK", "Releasing AD9523 reset (AD9523_RESET_RELEASE)");
    AD9523_RESET_RELEASE();
    HAL_Delay(5);
    DIAG("CLK", "AD9523 reset released, waited 5 ms");

    // Select REFB (100MHz) using REF_SEL pin (true selects REFB here)
    DIAG("CLK", "Selecting REFB (100 MHz) via REF_SEL pin");
    AD9523_REF_SEL(true);

    // Call setup which uses no_os_spi to write registers, io_update, calibrate, sync
    DIAG("CLK", "Calling ad9523_setup() -- post-reset configuration");
    uint32_t setup_start = HAL_GetTick();
    ret = ad9523_setup(&dev, &init_param);
    DIAG("CLK", "ad9523_setup() returned %ld (took %lu ms)",
         (long)ret, (unsigned long)(HAL_GetTick() - setup_start));
    if (ret != 0) {
        // handle error: lock missing or SPI error
        DIAG_ERR("CLK", "ad9523_setup() FAILED (ret=%ld) -- lock missing or SPI error", (long)ret);
        return -1;
    }

    // Final check
    DIAG("CLK", "Checking AD9523 status (PLL lock, etc.)");
    ret = ad9523_status(dev);
    DIAG("CLK", "ad9523_status() returned %ld", (long)ret);
    if (ret != 0) {
        // log/handle
        DIAG_ERR("CLK", "AD9523 status check FAILED (ret=%ld)", (long)ret);
        return -1;
    }

    // optionally manual sync
    DIAG("CLK", "Triggering manual ad9523_sync()");
    ad9523_sync(dev);
    DIAG("CLK", "AD9523 configuration complete -- all outputs should be active");

    // F-4.5: retain the dev handle so recovery can free it before re-setup.
    g_ad9523_dev = dev;
    return 0;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();  /* GAP-3 FIX 2: start hardware watchdog (~4 s timeout) */

  /* MCU-A7: check BKPSRAM emergency-persist flag BEFORE any PA enable code
   * runs. MX_GPIO_Init above has already forced EN_P_5V0_PA1/2/3,
   * EN_P_5V5_PA, and EN_DIS_RFPA_VDD LOW (line 2783); calling Emergency_Stop
   * here re-asserts that, sets the BKPSRAM flag (idempotent), and enters
   * the hold loop. The only way out is removing main power — IWDG-/SYSRESET-
   * driven boots all see the flag and stay in safe-hold. */
  if (emergency_persist_check()) {
      DIAG_ERR("SYS", "BKPSRAM emergency flag SET on boot -- entering safe-hold (power-cycle to clear)");
      Emergency_Stop();  /* NOTREACHED */
  }

  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);

  /* --- Enable DWT cycle counter for accurate microsecond delays --- */
  DWT_Init();

  DIAG_SECTION("SYSTEM INIT");
  DIAG("SYS", "DWT cycle counter initialized, TIM1 started");
  DIAG("SYS", "HAL tick at init start: %lu ms", (unsigned long)HAL_GetTick());

  /* PR-AB.b: production AGC policy is ALWAYS-ON. Bench builds compile this
   * out via -DMCU_AGC_FORCE_DISABLED so engineers can drive the ADAR VGA
   * by hand. Default class state is enabled=false (see ADAR1000_AGC ctor),
   * so we have to flip it here for the production polling loop to do work. */
#ifndef MCU_AGC_FORCE_DISABLED
  outerAgc.enabled = true;
  DIAG("AGC", "Outer-loop AGC ALWAYS-ON (production policy)");
#else
  DIAG("AGC", "Outer-loop AGC compiled out (MCU_AGC_FORCE_DISABLED bench build)");
#endif

  /* MCU-A4: skip the full 180 s OCXO warmup on warm restart. BKPSRAM
   * survives MCU-only resets (IWDG, SYSRESETREQ, brown-out) so the warmup
   * flag from the previous boot tells us the OCXO oven is still hot and
   * the crystal frequency is already settled. Power-cycle clears BKPSRAM
   * and we fall through to the full warmup, which is the safe default. */
  uint32_t ocxo_start = HAL_GetTick();
  if (warmup_persist_check()) {
      DIAG("CLK", "OCXO warm-restart detected -- skipping full warmup, 5 s settle only");
      /* [GAP-3 FIX 2] Cannot use HAL_Delay(5000) — IWDG would reset MCU.
       * Loop in 1-second steps so the watchdog stays kicked. */
      for (int s = 0; s < 5; s++) {
          HAL_IWDG_Refresh(&hiwdg);
          HAL_Delay(1000);
      }
  } else {
      DIAG("CLK", "OCXO cold start -- waiting full 180 s warmup (3 min)");
      /* [GAP-3 FIX 2] Cannot use HAL_Delay(180000) — IWDG would reset MCU.
       * Loop in 1-second steps, kicking the watchdog each iteration. */
      for (int ocxo_sec = 0; ocxo_sec < 180; ocxo_sec++) {
          HAL_IWDG_Refresh(&hiwdg);
          HAL_Delay(1000);
      }
      warmup_persist_set();  /* arm warm-restart bypass for any future reset */
  }
  DIAG_ELAPSED("CLK", "OCXO warmup", ocxo_start);

  DIAG_SECTION("AD9523 POWER SEQUENCING");
  DIAG("PWR", "Asserting AD9523 reset (pin LOW)");
  HAL_GPIO_WritePin(AD9523_RESET_GPIO_Port,AD9523_RESET_Pin,GPIO_PIN_RESET);

  //Power sequencing AD9523
  DIAG("PWR", "Enabling 1.8V clock rail");
  HAL_GPIO_WritePin(EN_P_1V8_CLOCK_GPIO_Port,EN_P_1V8_CLOCK_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "Enabling 3.3V clock rail");
  HAL_GPIO_WritePin(EN_P_3V3_CLOCK_GPIO_Port,EN_P_3V3_CLOCK_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "AD9523 rails up; reset will be released inside configure_ad9523()");

  //Set planned Clocks on AD9523
  /*
  o Channel 0 = 300MHz LVDS (ADF4382 TX)
  o Channel 1 = 300MHz LVDS (ADF4382 RX) (Phase aligned with Channel 0)
  o Channel 4 = 400MHz LVDS (ADC)
  o Channel 5 = 400MHz LVDS (FPGA_ADC)(Phase aligned with Channel 4)
  o Channel 6 = 100MHz LVCMOS (FPGA system clock)
  o Channel 7 = 20MHz LVCMOS (FPGA test)
  o Channel 8 = 60MHz LVDS (ADF4382 TX sync)
  o Channel 9 = 60MHz LVDS (ADF4382 RX sync)(Phase aligned with Channel 8)
  o Channel 10 = 120MHz LVCMOS
  o Channel 11 = 120MHz LVCMOS (Phase aligned with Channel 10)
   */


  // Ensure stm32_spi.c and stm32_delay.c are compiled and linked

  DIAG("CLK", "Calling configure_ad9523()...");
  uint32_t ad9523_cfg_start = HAL_GetTick();
  if (configure_ad9523() != 0) {
      // configuration error - handle as needed
      DIAG_ERR("CLK", "configure_ad9523() FAILED -- entering infinite halt loop");
      while (1) { HAL_Delay(1000); }
  }
  DIAG_ELAPSED("CLK", "configure_ad9523()", ad9523_cfg_start);
  DIAG("CLK", "AD9523 configured successfully");

  //Power sequencing FPGA
  DIAG_SECTION("FPGA POWER SEQUENCING");
  DIAG("PWR", "Enabling 1.0V FPGA rail");
  HAL_GPIO_WritePin(EN_P_1V0_FPGA_GPIO_Port,EN_P_1V0_FPGA_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "Enabling 1.8V FPGA rail");
  HAL_GPIO_WritePin(EN_P_1V8_FPGA_GPIO_Port,EN_P_1V8_FPGA_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "Enabling 3.3V FPGA rail");
  HAL_GPIO_WritePin(EN_P_3V3_FPGA_GPIO_Port,EN_P_3V3_FPGA_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "FPGA power sequencing complete -- 1.0V -> 1.8V -> 3.3V");

  /* Pulse FPGA reset (PD12) immediately after rails are stable so the FPGA
   * outputs (`adar_tr_x`, `mixers_enable`, `new_chirp/elevation/azimuth`)
   * are driven to known defaults BEFORE any ADAR1000 SPI traffic looks at
   * the TR pin. PA Vdd remains gated by `EN_DIS_RFPA_VDD` later, so PA
   * protection is unchanged. */
  DIAG("FPGA", "Resetting FPGA (GPIOD pin 12: LOW -> 10ms -> HIGH)");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  DIAG("FPGA", "FPGA reset complete -- adar_tr_x driven LOW (RX commanded)");

  /* F-2.1: this firmware build supports only FPGA mode 2'b01 (RP_MODE_AUTO_3KM,
   * the cold-reset default at radar_system_top.v:1058). The MCU does not
   * dispatch chirps -- chirp_scheduler.v owns the SHORT/MEDIUM/LONG ladder.
   * Mode 2'b00 (STM32 pass-through) is supported on the FPGA side but is
   * NOT implemented in this firmware: a real pass-through would need a
   * hardware-timer-driven PD8 emitter (software delay_us is too jittery
   * for Doppler) plus MCU<->FPGA agreement on host_chirps_per_subframe.
   * Operators must not change host_radar_mode away from 2'b01. */
  DIAG("FPGA", "Production mode: host_radar_mode = 2'b01 (auto-scan, FPGA-owned chirp dispatch)");

// Initialize module IMU
  DIAG_SECTION("IMU INIT (GY-85)");
  DIAG("IMU", "Initializing GY-85 IMU...");
  if (!GY85_Init()) {
      DIAG_ERR("IMU", "GY85_Init() FAILED -- calling Error_Handler()");
      Error_Handler();
  }
  DIAG("IMU", "GY-85 initialized OK, running 10 calibration samples");
  for(int i=0; i<10;i++){
  if (!GY85_Update(&imu)) {
      Error_Handler();
  }

  ax = imu.ax;
  ay = imu.ay;
  az = imu.az;
  gx = -imu.gx;
  gy = -imu.gy;
  gz = imu.gz;
  mx = imu.mx;
  my = imu.my;
  mz = imu.mz;

  ax *= 6.10351e-5;   //2.0 / 32768.0;
  ay *= 6.10351e-5;
  az *= 6.10351e-5;

  ax -= abias[0];   // Convert to g's, remove accelerometer biases
  ay -= abias[1];
  az -= abias[2];

  gx *= 0.0152588f;   //500.0 / 32768.0;
  gy *= 0.0152588f;
  gz *= 0.0152588f;

  gx -= gbias[0];   // Convert to degrees per seconds, remove gyro biases
  gy -= gbias[1];
  gz -= gbias[2];

  mx *= 6.10351e-5;   //2.0 / 32768.0;
  my *= 6.10351e-5;
  mz *= 6.10351e-5;

  mxc = M11*(mx-mbias[0]) + M12*(my-mbias[1]) +M13*(mz-mbias[2]);
  myc = M21*(mx-mbias[0]) + M22*(my-mbias[1]) +M23*(mz-mbias[2]);
  mzc = M31*(mx-mbias[0]) + M32*(my-mbias[1]) +M33*(mz-mbias[2]);

  mx = mxc;
  my = myc;
  mz = mzc;

  float norm = sqrt((mx*mx) + (my*my) + (mz*mz));
  mx /=norm;
  my /=norm;
  mz /=norm;

  /***************************************************************/
  /*********************IMU Complementary Filter******************/
  /***************************************************************/
    RxAcc = ax;
    RyAcc = ay;
    RzAcc = az;

    Rate_Ayz_1 = gx;
    Rate_Axz_1 = gy;
    Rate_Axy_1 = gz;

    Rate_Ayz_1 = Rate_Ayz_1*0.01745;//Convertion from °/s to Radians/s
    Rate_Axz_1 = Rate_Axz_1*0.01745;
    Rate_Axy_1 = Rate_Axy_1*0.01745;

    /*Combining Accelerometer & Gyroscope Data*/
    Axz_0 = atan2(RxEst_0,RzEst_0);
    Ayz_0 = atan2(RyEst_0,RzEst_0);
    Axy_0 = atan2(RxEst_0,RyEst_0);

    now_timeperiod = micros();

    Time_Period = now_timeperiod - lasttime_timeperiod;

    Axz_1= Axz_0 + Rate_Axz_1*Time_Period*0.000001;
    Ayz_1= Ayz_0 + Rate_Ayz_1*Time_Period*0.000001;
    Axy_1= Axy_0 + Rate_Axy_1*Time_Period*0.000001;

    lasttime_timeperiod = now_timeperiod;

    RxGyro = sin(Axz_1)/(sqrt(1.0 + cos(Axz_1)*cos(Axz_1) * tan(Ayz_1)*tan(Ayz_1)));
    RyGyro = sin(Ayz_1)/(sqrt(1.0 + cos(Ayz_1)*cos(Ayz_1) * tan(Axz_1)*tan(Axz_1)));
    if(RzEst_0 >= 0){
      RzGyro = 1.0*sqrt(1 - RxGyro*RxGyro - RyGyro*RyGyro);
    }
    else {
      RzGyro = -1*sqrt(1 - RxGyro*RxGyro - RyGyro*RyGyro);
    }

    RxEst_1 = RxAcc*0.5 + RxGyro*0.5 ;// Weight of Gyro. over Acc.
    RyEst_1 = RyAcc*0.5 + RyGyro*0.5 ;
    RzEst_1 = RzAcc*0.5 + RzGyro*0.5 ;

    R_Est = sqrt(RxEst_1*RxEst_1+RyEst_1*RyEst_1+RzEst_1*RzEst_1);

    Pitch_Sensor = atan2(RxEst_1,sqrt(RyEst_1*RyEst_1)+(RzEst_1*RzEst_1))*180/PI;
    Roll_Sensor = atan2(RyEst_1,sqrt(RxEst_1*RxEst_1)+(RzEst_1*RzEst_1))*180/PI;

    float magRawX = mx*cos(Pitch_Sensor*PI/180.0f)  - mz*sin(Pitch_Sensor*PI/180.0f);
	float magRawY = mx*sin(Roll_Sensor*PI/180.0f)*sin(Pitch_Sensor*PI/180.0f) + my*cos(Roll_Sensor*PI/180.0f)- mz*sin(Roll_Sensor*PI/180.0f)*cos(Pitch_Sensor*PI/180.0f);
    Yaw_Sensor = (180*atan2(magRawY,magRawX)/PI) - get_mag_declination_deg();  /* MCU-A2 */

    if(Yaw_Sensor<0)Yaw_Sensor+=360;

    // Override magnetometer heading with UM982 dual-antenna heading when available
    if (um982_is_heading_valid(&um982)) {
        Yaw_Sensor = um982_get_heading(&um982);
    }

    RxEst_0 = RxEst_1;
    RyEst_0 = RyEst_1;
    RzEst_0 = RzEst_1;

    HAL_Delay(300);

  }

    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////BAROMETER BMP180////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    DIAG_SECTION("BAROMETER INIT (BMP180)");

    /* AUDIT-CAL fix: probe the BMP180 over I2C and read its 11 factory calibration
     * coefficients. The driver was previously instantiated without this step, so
     * `_calCoeff` ran at zero defaults — `computeB5()` short-circuited via 0/0
     * (Cortex-M7 SDIV with DIV_0_TRP=0 returns 0 silently), `getPressure()` always
     * returned the I2C-error sentinel, the health watchdog fired
     * ERROR_BMP180_COMM every iteration, and `error_count > 10` latched
     * `system_emergency_state = true` within ~25 s of boot. Try up to 3 times
     * with 50 ms backoff to ride out a noisy boot-time I2C bus. */
    bmp180_operational = false;
    for (int attempt = 0; attempt < 3 && !bmp180_operational; attempt++) {
        if (myBMP.begin()) {
            bmp180_operational = true;
            DIAG("IMU", "BMP180 begin() OK on attempt %d (cal coefficients loaded)", attempt + 1);
        } else if (attempt < 2) {
            DIAG_WARN("IMU", "BMP180 begin() failed attempt %d, retrying...", attempt + 1);
            HAL_Delay(50);
        }
    }

    if (bmp180_operational) {
        DIAG("IMU", "Reading 5 barometer samples for altitude baseline");
        for (int i = 0; i < 5; i++) {
            float BMP_Perssure = myBMP.getPressure();
            RADAR_Altitude = 44330 * (1 - (pow((BMP_Perssure / 101325), (1 / 5.255))));
            HAL_Delay(100);
        }
        DIAG("IMU", "Barometer init complete, RADAR_Altitude baseline set");
    } else {
        /* Sensor absent or cal could not be read. Leave RADAR_Altitude at its
         * default (0.0f) so downstream gps_data telemetry sees a clean value
         * instead of a NaN propagated from pow(negative, fractional). The
         * watchdog will skip the BMP180 check (gated on bmp180_operational). */
        RADAR_Altitude = 0.0f;
        DIAG_ERR("IMU", "BMP180 begin() FAILED after 3 attempts -- sensor will be marked absent; altitude baseline = 0.0 m; watchdog will skip BMP180 check");
    }

    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////ADF4382/////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    DIAG_SECTION("ADF4382A LO INIT");
    printf("Starting ADF4382A Radar LO System...\n");
    printf("Using SPI4 with TIMED SYNCHRONIZATION (60 MHz clocks)\n");

    // Initialize LO manager with TIMED synchronization
    DIAG("LO", "Calling ADF4382A_Manager_Init(sync=SYNC_METHOD_TIMED)");
    uint32_t lo_init_start = HAL_GetTick();
    int ret = ADF4382A_Manager_Init(&lo_manager, SYNC_METHOD_TIMED);
    DIAG("LO", "ADF4382A_Manager_Init returned %d (took %lu ms)",
         ret, (unsigned long)(HAL_GetTick() - lo_init_start));

    /* [Bug #4 FIXED] Check init return code BEFORE calling phase shift.
     * Previously SetPhaseShift/Strobe were called before this check. */
    if (ret != ADF4382A_MANAGER_OK) {
        printf("LO Manager initialization failed: %d\n", ret);
        DIAG_ERR("LO", "Manager init FAILED (ret=%d) -- calling Error_Handler()", ret);
        Error_Handler();
    }

    // Set phase shift (e.g., 500 ps for TX, 500 ps for RX)
    int ps_ret = ADF4382A_SetPhaseShift(&lo_manager, 500, 500);
    DIAG("LO", "ADF4382A_SetPhaseShift(500, 500) returned %d", ps_ret);

    // Strobe to apply phase shifts
    int strobe_tx_ret = ADF4382A_StrobePhaseShift(&lo_manager, 0); // TX device
    int strobe_rx_ret = ADF4382A_StrobePhaseShift(&lo_manager, 1); // RX device
    DIAG("LO", "StrobePhaseShift TX returned %d, RX returned %d", strobe_tx_ret, strobe_rx_ret);

    // Check initial lock status
    bool tx_locked, rx_locked;
    DIAG("LO", "Checking initial lock status...");
    ret = ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked);
    if (ret == ADF4382A_MANAGER_OK) {
        printf("Initial Lock Status - TX: %s, RX: %s\n",
               tx_locked ? "LOCKED" : "UNLOCKED",
               rx_locked ? "LOCKED" : "UNLOCKED");
        DIAG("LO", "Initial lock: TX=%s RX=%s", tx_locked ? "LOCKED" : "UNLOCKED", rx_locked ? "LOCKED" : "UNLOCKED");
    } else {
        DIAG_ERR("LO", "CheckLockStatus returned %d", ret);
    }
    // Wait for both LOs to lock
        DIAG("LO", "Entering lock-wait loop (max 10 s = 100 x 100 ms)");
        uint32_t lock_wait_start = HAL_GetTick();
        uint32_t lock_timeout = 0;
        while (!(tx_locked && rx_locked) && lock_timeout < 100) {
            HAL_Delay(100);
            ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked);
            lock_timeout++;

            if (lock_timeout % 10 == 0) {
                printf("Waiting for LO lock... TX: %s, RX: %s\n",
                       tx_locked ? "LOCKED" : "UNLOCKED",
                       rx_locked ? "LOCKED" : "UNLOCKED");
                DIAG("LO", "Lock poll #%lu: TX=%s RX=%s",
                     (unsigned long)lock_timeout,
                     tx_locked ? "LOCKED" : "UNLOCKED",
                     rx_locked ? "LOCKED" : "UNLOCKED");
            }
        }
        DIAG_ELAPSED("LO", "Lock wait loop", lock_wait_start);

        if (tx_locked && rx_locked) {
            printf("Both LOs locked successfully!\n");
            printf("Synchronization ready - 60 MHz clocks on SYNCP/SYNCN pins\n");
            DIAG("LO", "Both LOs LOCKED after %lu iterations", (unsigned long)lock_timeout);
            // - 60 MHz phase-aligned clocks on SYNCP/SYNCN pins
            // - SYNC pin connected for triggering

            // When ready to synchronize:
            DIAG("LO", "Calling TriggerTimedSync to pulse sw_sync on both PLLs");
            ADF4382A_TriggerTimedSync(&lo_manager);
            // At this point, the SYNC pin can be toggled to trigger synchronization
        } else {
            printf("LO lock timeout! TX: %s, RX: %s\n",
                   tx_locked ? "LOCKED" : "UNLOCKED",
                   rx_locked ? "LOCKED" : "UNLOCKED");
            DIAG_ERR("LO", "Lock TIMEOUT after %lu iterations! TX=%s RX=%s",
                     (unsigned long)lock_timeout,
                     tx_locked ? "LOCKED" : "UNLOCKED",
                     rx_locked ? "LOCKED" : "UNLOCKED");
            /* F-1.6: latch emergency state. PA Vdd enable + TX mixer enable
             * later in this function, and the radar loop in main(), all gate
             * on system_emergency_state — keying the PA into an unlocked
             * synth would radiate spurious/free-running RF at full power. */
            last_error = (!tx_locked) ? ERROR_ADF4382_TX_UNLOCK : ERROR_ADF4382_RX_UNLOCK;
            system_emergency_state = true;
        }

  // check if there is a lock via direct GPIO (independent of register read above)
  DIAG("LO", "Direct GPIO lock detect pins:");
  DIAG_GPIO("LO", "ADF4382_TX_LKDET", ADF4382_TX_LKDET_GPIO_Port, ADF4382_TX_LKDET_Pin);
  DIAG_GPIO("LO", "ADF4382_RX_LKDET", ADF4382_RX_LKDET_GPIO_Port, ADF4382_RX_LKDET_Pin);
  if(HAL_GPIO_ReadPin (ADF4382_TX_LKDET_GPIO_Port, ADF4382_TX_LKDET_Pin))
  {
      // Set The LED_1 ON!
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
      DIAG("LO", "TX lock detected via GPIO -- LED_1 ON");
  }
  else
  {
      // Else .. Turn LED_1 OFF!
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
      DIAG_WARN("LO", "TX NOT locked via GPIO -- LED_1 OFF");
  }

  if(HAL_GPIO_ReadPin (ADF4382_RX_LKDET_GPIO_Port, ADF4382_RX_LKDET_Pin))
  {
      // Set The LED_2 ON!
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      DIAG("LO", "RX lock detected via GPIO -- LED_2 ON");
  }
  else
  {
      // Else .. Turn LED_2 OFF!
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
      DIAG_WARN("LO", "RX NOT locked via GPIO -- LED_2 OFF");
  }


  //////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////ADAR1000/////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////

  //Power sequencing ADAR1000
  DIAG_SECTION("ADAR1000 POWER SEQUENCING");

  //Tell FPGA to turn off TX RF signal by disabling Mixers
  DIAG("BF", "Disabling TX mixers (GPIOD pin 11 LOW)");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

  DIAG("PWR", "Enabling 3.3V ADAR12 + ADAR34 rails");
  HAL_GPIO_WritePin(EN_P_3V3_ADAR12_GPIO_Port,EN_P_3V3_ADAR12_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_P_3V3_ADAR34_GPIO_Port,EN_P_3V3_ADAR34_Pin,GPIO_PIN_SET);
  HAL_Delay(500);
  DIAG("PWR", "Enabling 5.0V ADAR rail");
  HAL_GPIO_WritePin(EN_P_5V0_ADAR_GPIO_Port,EN_P_5V0_ADAR_Pin,GPIO_PIN_SET);
  HAL_Delay(500);
  DIAG("PWR", "ADAR1000 power sequencing complete");

  // System startup message
      uint8_t startup_msg[] = "Starting Phased Array RADAR System...\r\n";
      HAL_UART_Transmit(&huart3, startup_msg, sizeof(startup_msg)-1, 1000);

      DIAG("BF", "Calling systemPowerUpSequence()");
      // Power up sequence
      systemPowerUpSequence();

      DIAG("BF", "Calling initializeBeamMatrices()");
      // Initialize beam matrices
      initializeBeamMatrices();

      // Print system status
      printSystemStatus();
      DIAG("SYS", "System init complete -- entering main loop");

  	//////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////GPS/////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
  DIAG_SECTION("GPS INIT (UM982)");
  DIAG("GPS", "Initializing UM982 on UART5 @ 115200 (baseline=50cm, tol=3cm)");
  if (!um982_init(&um982, &huart5, 50.0f, 3.0f)) {
      DIAG_WARN("GPS", "UM982 init: no VERSIONA response -- module may need more time");
      // Not fatal: module may still start sending NMEA data after boot
  } else {
      DIAG("GPS", "UM982 init OK -- VERSIONA received");
  }

  // Collect GPS data for a few seconds (non-blocking pump)
  DIAG("GPS", "Pumping GPS for 5 seconds to acquire initial fix...");
  {
      uint32_t gps_start = HAL_GetTick();
      while (HAL_GetTick() - gps_start < 5000) {
          um982_process(&um982);
          HAL_Delay(10);
      }
  }
  RADAR_Longitude = um982_get_longitude(&um982);
  RADAR_Latitude = um982_get_latitude(&um982);
  DIAG("GPS", "Initial position: lat=%.6f lon=%.6f fix=%d sats=%d",
       RADAR_Latitude, RADAR_Longitude,
       um982_get_fix_quality(&um982), um982_get_num_sats(&um982));

  // Re-apply heading after GPS init so the north-alignment stepper move uses
  // UM982 dual-antenna heading when available.
  if (um982_is_heading_valid(&um982)) {
      Yaw_Sensor = um982_get_heading(&um982);
  }

  //move Stepper to position 1 = 0°
  HAL_GPIO_WritePin(STEPPER_CW_P_GPIO_Port, STEPPER_CW_P_Pin, GPIO_PIN_RESET);//Set stepper motor spinning direction to CCW
  //Point Stepper to North
  for(int i= 0;i<(int)(Yaw_Sensor*Stepper_steps/360);i++){
	  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_SET);
	  delay_us(500);
	  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_RESET);
	  delay_us(500);
  }

  /***************************************************************/
  /**********wait for GUI start flag and Send Lat/Long/alt********/
  /***************************************************************/

  GPS_Data_t gps_data;
  // Binary packet structure:
  // [Header 4 bytes][Latitude 8 bytes][Longitude 8 bytes][Altitude 4 bytes][Pitch 4 bytes][CRC 2 bytes]
  gps_data = {RADAR_Latitude, RADAR_Longitude, RADAR_Altitude, Pitch_Sensor, HAL_GetTick()};
  if (!GPS_SendBinaryToGUI(&gps_data)) {
      const uint8_t gps_send_error[] = "GPS binary send failed\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)gps_send_error, sizeof(gps_send_error) - 1, 1000);
  }

  /* [STM32-006 FIXED] Removed blocking do-while loop that waited for
   * usbHandler.isStartFlagReceived().  The production V7 PyQt GUI does not
   * send the legacy 4-byte start flag [23,46,158,237], so this loop hung
   * the MCU at boot indefinitely.  The USB settings handshake (if ever
   * re-enabled) should be handled non-blocking in the main loop. */

  /***************************************************************/
  /************RF Power Amplifier Powering up sequence************/
  /***************************************************************/
  /* FPGA was reset right after its 3V3 rail came up — adar_tr_x has been
   * driven LOW (RX) for the entire interval since then. PA Vdd is still
   * gated by EN_DIS_RFPA_VDD below. F-1.6: PA enable also gates on
   * !system_emergency_state — set true earlier on LO lock failure. */
  if(PowerAmplifier && !system_emergency_state){
	  DIAG_SECTION("PA: RF Power Amplifier power-up sequence");
	  /* Initialize DACs */
	  /* DAC1: Address 0b1001000 = 0x48 */
	  DIAG("PA", "Initializing DAC1 (I2C1, addr=0x48, 8-bit)");
	  if (!DAC5578_Init(&hdac1, &hi2c1, 0x48, 8,
			  DAC_1_VG_LDAC_GPIO_Port, DAC_1_VG_LDAC_Pin,
			  DAC_1_VG_CLR_GPIO_Port, DAC_1_VG_CLR_Pin)) {
		  DIAG_ERR("PA", "DAC1 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "DAC1 init OK");

	  /* DAC2: Address 0b1001001 = 0x49 */
	  DIAG("PA", "Initializing DAC2 (I2C1, addr=0x49, 8-bit)");
	  if (!DAC5578_Init(&hdac2, &hi2c1, 0x49, 8,
			  DAC_2_VG_LDAC_GPIO_Port, DAC_2_VG_LDAC_Pin,
			  DAC_2_VG_CLR_GPIO_Port, DAC_2_VG_CLR_Pin)) {
		  DIAG_ERR("PA", "DAC2 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "DAC2 init OK");

	  /* Configure clear code behavior */
	  DIAG("PA", "Setting clear code to ZERO on both DACs");
	  DAC5578_SetClearCode(&hdac1, DAC5578_CLR_CODE_ZERO); // Clear to 0V on CLR pulse
	  DAC5578_SetClearCode(&hdac2, DAC5578_CLR_CODE_ZERO); // Clear to 0V on CLR pulse

	  /* Configure LDAC so all channels update simultaneously on hardware LDAC */
	  DIAG("PA", "Setting LDAC mask=0xFF on both DACs (all channels respond)");
	  DAC5578_SetupLDAC(&hdac1, 0xFF); // All channels respond to LDAC
	  DAC5578_SetupLDAC(&hdac2, 0xFF); // All channels respond to LDAC

	  //set Vg [1-8] to -3.98V -> input opamp = 1.63058V->126(8bits)
	  DIAG("PA", "Writing initial DAC_val=%d to DAC1 channels 0-7", DAC_val);
	  for(int channel = 0; channel < 8; channel++){
		  DAC5578_WriteAndUpdateChannelValue(&hdac1, channel, DAC_val);
	  }

	  //set Vg [9-16] to -3.98V -> input opamp = 1.63058V->126(8bits)
	  DIAG("PA", "Writing initial DAC_val=%d to DAC2 channels 0-7", DAC_val);
	  for(int channel = 0; channel < 8; channel++){
		  DAC5578_WriteAndUpdateChannelValue(&hdac2, channel, DAC_val);
	  }

	  /* Optional: Use hardware LDAC for simultaneous update of all channels */
	  DIAG("PA", "Pulsing LDAC on both DACs for simultaneous update");
	  HAL_GPIO_WritePin(DAC_1_VG_LDAC_GPIO_Port, DAC_1_VG_LDAC_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(DAC_2_VG_LDAC_GPIO_Port, DAC_2_VG_LDAC_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(DAC_1_VG_LDAC_GPIO_Port, DAC_1_VG_LDAC_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DAC_2_VG_LDAC_GPIO_Port, DAC_2_VG_LDAC_Pin, GPIO_PIN_SET);

	  //Enable RF Power Amplifier VDD = 22V
	  /* FPGA reset was issued right after its 3V3 rail came up; adar_tr_x has
	   * been LOW (RX) ever since. TX mixer enable (PD11) is still LOW until
	   * later, gating any FPGA-driven chirps. Safe to bring PA Vdd to 22 V. */
	  DIAG("PA", "Enabling RFPA VDD=22V (EN_DIS_RFPA_VDD HIGH)");
	  HAL_GPIO_WritePin(EN_DIS_RFPA_VDD_GPIO_Port, EN_DIS_RFPA_VDD_Pin, GPIO_PIN_SET);

	  /* Initialize ADCs with correct addresses */
	  /* ADC1: Address 0x48, Single-Ended mode, Internal Ref ON + ADC ON */
	  DIAG("PA", "Initializing ADC1 (I2C2, addr=0x48, single-ended, ref+adc ON)");
	  if (!ADS7830_Init(&hadc1, &hi2c2, 0x48,
						ADS7830_SDMODE_SINGLE, ADS7830_PDIRON_ADON)) {
		  DIAG_ERR("PA", "ADC1 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "ADC1 init OK");

	  /* ADC2: Address 0x4A, Single-Ended mode, Internal Ref ON + ADC ON */
	  DIAG("PA", "Initializing ADC2 (I2C2, addr=0x4A, single-ended, ref+adc ON)");
	  if (!ADS7830_Init(&hadc2, &hi2c2, 0x4A,
						ADS7830_SDMODE_SINGLE, ADS7830_PDIRON_ADON)) {
		  DIAG_ERR("PA", "ADC2 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "ADC2 init OK");

	  /* Read all 8 channels from ADC1 and calculate Idq */
	  DIAG("PA", "Reading initial Idq from ADC1 channels 0-7");
	  for (uint8_t channel = 0; channel < 8; channel++) {
		  adc1_readings[channel] = ADS7830_Measure_SingleEnded(&hadc1, channel);
		  Idq_reading[channel]= (3.3/255)*adc1_readings[channel]/(50*0.005);//Idq=Vadc/(GxRshunt)//G_INA241A3=50;Rshunt=5mOhms
		  DIAG("PA", "  ADC1 ch%d: raw=%d Idq=%.3fA", channel, adc1_readings[channel], Idq_reading[channel]);
	  }

	  /* Read all 8 channels from ADC2 and calculate Idq*/
	  DIAG("PA", "Reading initial Idq from ADC2 channels 0-7");
	  for (uint8_t channel = 0; channel < 8; channel++) {
		  adc2_readings[channel] = ADS7830_Measure_SingleEnded(&hadc2, channel);
		  Idq_reading[channel+8]= (3.3/255)*adc2_readings[channel]/(50*0.005);//Idq=Vadc/(GxRshunt)//G_INA241A3=50;Rshunt=5mOhms
		  DIAG("PA", "  ADC2 ch%d: raw=%d Idq=%.3fA", channel, adc2_readings[channel], Idq_reading[channel+8]);
	  }

	  /* MCU-A5: gate Idq health-check window across both DAC1 + DAC2 cal walks. */
	  pa_calibration_in_progress = true;
	  DIAG("PA", "Starting Idq calibration loop for DAC1 channels 0-7 (target=1.680A)");
	  for (uint8_t channel = 0; channel < 8; channel++){
	      uint8_t safety_counter = 0;
	      DAC_val = 126; // Reset for each channel

	      do {
	          if (safety_counter++ > 50) { // Prevent infinite loop
	              DIAG_WARN("PA", "  DAC1 ch%d: safety limit reached (50 iterations), DAC_val=%d Idq=%.3fA",
	                        channel, DAC_val, Idq_reading[channel]);
	              break;
	          }
	          DAC_val = DAC_val - 4;
	          DAC5578_WriteAndUpdateChannelValue(&hdac1, channel, DAC_val);
	          adc1_readings[channel] = ADS7830_Measure_SingleEnded(&hadc1, channel);
	          Idq_reading[channel] = (3.3/255) * adc1_readings[channel] / (50 * 0.005);
	      } while (DAC_val > 38 && abs(Idq_reading[channel] - 1.680) > 0.2); // B12 fix: loop while FAR from target
	      DIAG("PA", "  DAC1 ch%d calibrated: DAC_val=%d Idq=%.3fA iters=%d",
	           channel, DAC_val, Idq_reading[channel], safety_counter);
	  }

	  DIAG("PA", "Starting Idq calibration loop for DAC2 channels 0-7 (target=1.680A)");
	  for (uint8_t channel = 0; channel < 8; channel++){
	      uint8_t safety_counter = 0;
	      DAC_val = 126; // Reset for each channel

	      do {
	          if (safety_counter++ > 50) { // Prevent infinite loop
	              DIAG_WARN("PA", "  DAC2 ch%d: safety limit reached (50 iterations), DAC_val=%d Idq=%.3fA",
	                        channel, DAC_val, Idq_reading[channel+8]);
	              break;
	          }
	          DAC_val = DAC_val - 4;
	          DAC5578_WriteAndUpdateChannelValue(&hdac2, channel, DAC_val);
	          adc2_readings[channel] = ADS7830_Measure_SingleEnded(&hadc2, channel); // B13 fix: was adc1_readings
	          Idq_reading[channel+8] = (3.3/255) * adc2_readings[channel] / (50 * 0.005);
	      } while (DAC_val > 38 && abs(Idq_reading[channel+8] - 1.680) > 0.2); // B12 fix: loop while FAR from target
	      DIAG("PA", "  DAC2 ch%d calibrated: DAC_val=%d Idq=%.3fA iters=%d",
	           channel, DAC_val, Idq_reading[channel+8], safety_counter);
	  }
	  /* MCU-A5: cal walks complete -- re-arm Idq health checks. Any channel
	   * left out-of-window by safety_counter timeout will surface on the
	   * next checkSystemHealth pass and route through the normal handler. */
	  pa_calibration_in_progress = false;
	  DIAG("PA", "PA IDQ calibration sequence COMPLETE");
  }

  //Tell FPGA to apply TX RF by enabling Mixers (F-1.6: skip if emergency)
  if (!system_emergency_state) {
    DIAG("FPGA", "Enabling TX mixers (GPIOD pin 11 HIGH)");
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
  } else {
    DIAG_ERR("FPGA", "TX mixer enable SKIPPED -- system_emergency_state=true (LO lock failed?)");
  }

  /* T°C sensor TMP37 ADC3: Address 0x49, Single-Ended mode, Internal Ref ON + ADC ON */
  DIAG("SYS", "Initializing temperature sensor ADC3 (I2C2, addr=0x49, TMP37)");
  if (!ADS7830_Init(&hadc3, &hi2c2, 0x49,
					ADS7830_SDMODE_SINGLE, ADS7830_PDIRON_ADON)) {
	  DIAG_ERR("SYS", "Temperature sensor ADC3 init FAILED -- calling Error_Handler()");
	  Error_Handler();
  }
  DIAG("SYS", "Temperature sensor ADC3 init OK");

  /***************************************************************/
  /************************ERRORS HANDLERS************************/
  /***************************************************************/

  // Initialize error handler system. Only clear emergency_state / last_error
  // when no fatal init error was latched — F-1.6 sets system_emergency_state
  // on LO lock failure earlier in main(), and we must not clear that here or
  // the PA / TX-mixer guards become a one-shot that's already been cleared
  // by the time the main-loop health check runs.
  DIAG("SYS", "Initializing error handler: clearing error_count; preserving any latched emergency state");
  error_count = 0;
  if (last_error == ERROR_NONE) {
      system_emergency_state = false;
  } else {
      DIAG_ERR("SYS", "Init left last_error=%d, system_emergency_state=%d -- preserving for main-loop SAFE handling",
               (int)last_error, (int)system_emergency_state);
  }

  /***************************************************************/
  /*********************SEND TO GUI STATUS************************/
  /***************************************************************/

  // Send initial status to GUI
  DIAG("USB", "Sending initial system status to GUI via USB CDC");
  char initial_status[500];
  getSystemStatusForGUI(initial_status, sizeof(initial_status));
  // Send via USB to GUI
  CDC_Transmit_FS((uint8_t*)initial_status, strlen(initial_status));
  DIAG("USB", "Initial status sent (%d bytes)", (int)strlen(initial_status));

  DIAG("SYS", "=== INIT COMPLETE -- entering main loop ===");






  // main loop

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //////////////////////////////////////////////////////////////////////////////////////
	  //////////////////////// Check system health at the start of each loop////////////////
	  //////////////////////////////////////////////////////////////////////////////////////

	    if (!checkSystemHealthStatus()) {
	        // System is in bad state, enter safe mode
	    	//Tell FPGA to turn off TX RF signal by disabling Mixers
	    	DIAG_ERR("SYS", "checkSystemHealthStatus() FAILED -- entering SAFE MODE");
	    	DIAG("SYS", "Disabling TX mixers (GPIOD pin 11 LOW)");
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	    	DIAG("SYS", "Calling systemPowerDownSequence()");
	        systemPowerDownSequence();

	        char emergency_msg[] = "SYSTEM IN SAFE MODE - Manual intervention required\r\n";
	        HAL_UART_Transmit(&huart3, (uint8_t*)emergency_msg, strlen(emergency_msg), 1000);
	        DIAG_ERR("SYS", "SAFE MODE ACTIVE -- blinking all LEDs until IWDG resets the MCU");

	        /* F-2.4 FIX: blink-then-reset semantics. No main-loop path clears
	         * system_emergency_state — the loop body deliberately omits
	         * HAL_IWDG_Refresh() so the watchdog fires (~4 s) and resets the
	         * MCU. checkSystemHealthStatus() has already called
	         * emergency_persist_set() (F-2.5), so the next boot sees the
	         * BKPSRAM flag at line ~1630 and routes straight into
	         * Emergency_Stop's safe-hold (PA rails cut, infinite IWDG-
	         * refreshed loop, power-cycle to clear). The previous comment
	         * "waiting for system_emergency_state clear" implied a runtime
	         * exit that never existed. */
	        while (system_emergency_state) {
	            HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	            HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
	            HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
	            HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
	            HAL_Delay(250);
	        }
	        /* NOTREACHED — IWDG resets before this line. Kept as defensive
	         * marker for future refactors that might add a runtime clear. */
	        DIAG("SYS", "Exited safe mode blink loop -- unexpected, system_emergency_state cleared");
	    }

	  //////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////// GPS: Non-blocking NMEA processing ////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////
	  um982_process(&um982);

	  // Update position globals continuously
	  if (um982_is_position_valid(&um982)) {
	      RADAR_Latitude = um982_get_latitude(&um982);
	      RADAR_Longitude = um982_get_longitude(&um982);
	  } else if (um982_position_age(&um982) > 30000) {
	      /* F-2.7b FIX: GPS fix has aged past the 30 s health-check threshold.
	       * Invalidate the cached lat/lon globals so getSystemStatusForGUI()
	       * does not keep emitting the last-known fix as if it were current.
	       * NaN propagates through %.6f as "nan" which the GUI parser treats
	       * as missing. The 30 s window matches the ERROR_GPS_COMM gate in
	       * checkSystemHealth() at line 842. */
	      RADAR_Latitude = NAN;
	      RADAR_Longitude = NAN;
	  }

	  /* F-2.1b: 5 s LO lock poll deleted. checkSystemHealth() at line ~749
	   * already calls ADF4382A_CheckLockStatus(&lo_manager, ...) every
	   * main-loop iteration (no rate-limit) and routes any unlock through
	   * handleSystemError -> attemptErrorRecovery. The 5 s poll only added
	   * DIAG noise -- it didn't feed the recovery path. (void)tx_locked /
	   * (void)rx_locked silences the unused-but-set warning on the init
	   * declaration at line ~1974, which only appears here in non-test builds. */
	  (void)tx_locked;
	  (void)rx_locked;
	  //////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////// Monitor Temperature Sensors periodically//////////////////
	  //////////////////////////////////////////////////////////////////////////////////////
	  /* Read all 8 channels from ADC2 and calculate Idq*/
      // Monitor T°C periodically
      static uint32_t last_check1 = 0;
		  if (HAL_GetTick() - last_check1 > 5000) {
		  //TMP37 3.3V-->165°C & ADS7830 3.3V-->255 => Temperature_n °C = ADC_val_n * 165/255
		  Temperature_1 = ADS7830_Measure_SingleEnded(&hadc3, 0)*0.64705f;
		  Temperature_2 = ADS7830_Measure_SingleEnded(&hadc3, 1)*0.64705f;
		  Temperature_3 = ADS7830_Measure_SingleEnded(&hadc3, 2)*0.64705f;
		  Temperature_4 = ADS7830_Measure_SingleEnded(&hadc3, 3)*0.64705f;
		  Temperature_5 = ADS7830_Measure_SingleEnded(&hadc3, 4)*0.64705f;
		  Temperature_6 = ADS7830_Measure_SingleEnded(&hadc3, 5)*0.64705f;
		  Temperature_7 = ADS7830_Measure_SingleEnded(&hadc3, 6)*0.64705f;
		  Temperature_8 = ADS7830_Measure_SingleEnded(&hadc3, 7)*0.64705f;

		  DIAG("PA", "Temps: T1=%.1f T2=%.1f T3=%.1f T4=%.1f T5=%.1f T6=%.1f T7=%.1f T8=%.1f",
		       (double)Temperature_1, (double)Temperature_2, (double)Temperature_3, (double)Temperature_4,
		       (double)Temperature_5, (double)Temperature_6, (double)Temperature_7, (double)Temperature_8);

		  /* [GAP-3 FIX 3] Populate `temperature` with hottest sensor reading.
		   * checkSystemHealth() uses `temperature` for the >75 °C overtemp check;
		   * previously it was uninitialized. */
		  {
		      float temps[8] = { Temperature_1, Temperature_2, Temperature_3, Temperature_4,
		                         Temperature_5, Temperature_6, Temperature_7, Temperature_8 };
		      temperature = temps[0];
		      for (int ti = 1; ti < 8; ti++) {
		          if (temps[ti] > temperature) temperature = temps[ti];
		      }
		      DIAG("PA", "System temperature (max of 8 sensors) = %.1f C", (double)temperature);
		  }

		  // MCU-A1: production thermal limits for QPA2962 (TBASE max +85 °C).
		  // Cooling fan turns ON at 70 °C and OFF at 60 °C — 10 °C hysteresis
		  // prevents relay/fan chatter near the threshold. The system-level
		  // hard overtemp gate at checkSystemHealth() (>75 °C → SAFE mode)
		  // sits above the cooling ON point so the fan gets a chance to act
		  // before shutdown. Was previously a 25 °C dev-bench stub that kept
		  // the fan latched on at room temperature.
		  static bool cooling_on = false;
		  const float COOLING_ON_C  = 70.0f;
		  const float COOLING_OFF_C = 60.0f;
		  float t_max = temperature;  // populated above as max of 8 sensors
		  if (!cooling_on && t_max > COOLING_ON_C) {
			  cooling_on = true;
			  HAL_GPIO_WritePin(EN_DIS_COOLING_GPIO_Port, EN_DIS_COOLING_Pin, GPIO_PIN_SET);
			  DIAG_WARN("PA", "Over-temp %.1f C > %.0f C -- cooling ENABLED", (double)t_max, (double)COOLING_ON_C);
		  } else if (cooling_on && t_max < COOLING_OFF_C) {
			  cooling_on = false;
			  HAL_GPIO_WritePin(EN_DIS_COOLING_GPIO_Port, EN_DIS_COOLING_Pin, GPIO_PIN_RESET);
			  DIAG("PA", "Temp %.1f C < %.0f C -- cooling DISABLED", (double)t_max, (double)COOLING_OFF_C);
		  }

		  /* [GAP-3 FIX 4] Periodic IDQ re-read — the Idq_reading[] array was only
		   * populated during startup/calibration.  checkSystemHealth() compares
		   * stale values for overcurrent (>2.5 A) and bias fault (<0.1 A) checks.
		   * Re-read all 16 channels every 5 s alongside temperature.
		   * F-2.8: extend the gate with !system_emergency_state to mirror the
		   * F-1.6 pattern. If we entered safe mode mid-loop, systemPowerDown-
		   * Sequence has cut the PA rails but the PowerAmplifier flag may
		   * still be true — reading near-zero Idq values then would trip a
		   * spurious ERROR_RF_PA_BIAS on the next checkSystemHealth pass. */
		  if (PowerAmplifier && !system_emergency_state) {
		      DIAG("PA", "Periodic IDQ re-read (ADC1 + ADC2, 16 channels)");
		      for (uint8_t ch = 0; ch < 8; ch++) {
		          adc1_readings[ch] = ADS7830_Measure_SingleEnded(&hadc1, ch);
		          Idq_reading[ch] = (3.3f/255.0f) * adc1_readings[ch] / (50.0f * 0.005f);
		      }
		      for (uint8_t ch = 0; ch < 8; ch++) {
		          adc2_readings[ch] = ADS7830_Measure_SingleEnded(&hadc2, ch);
		          Idq_reading[ch + 8] = (3.3f/255.0f) * adc2_readings[ch] / (50.0f * 0.005f);
		      }
		      DIAG("PA", "IDQ[0..3]=%.3f %.3f %.3f %.3f  [4..7]=%.3f %.3f %.3f %.3f",
		           (double)Idq_reading[0], (double)Idq_reading[1],
		           (double)Idq_reading[2], (double)Idq_reading[3],
		           (double)Idq_reading[4], (double)Idq_reading[5],
		           (double)Idq_reading[6], (double)Idq_reading[7]);
		      DIAG("PA", "IDQ[8..11]=%.3f %.3f %.3f %.3f  [12..15]=%.3f %.3f %.3f %.3f",
		           (double)Idq_reading[8], (double)Idq_reading[9],
		           (double)Idq_reading[10], (double)Idq_reading[11],
		           (double)Idq_reading[12], (double)Idq_reading[13],
		           (double)Idq_reading[14], (double)Idq_reading[15]);
		  }


		  /* [BUG #6 FIXED] Was 'last_check' — now correctly writes 'last_check1'
		   * so the temperature timer runs independently of the lock-check timer. */
		  last_check1 = HAL_GetTick();
		  }
	  //////////////////////////////////////////////////////////////////////////////////////
	  /////////////////////////////////////ADAR1000/////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////
	 //phase_step = 0 => phase = 0°
	 //phase_step = 127 => phase = 360°
	 //steering angle (rad)= arcsin(phase_dif/Pi)

      runRadarPulseSequence();

      /* [AGC] Outer-loop AGC: read saturation flag (DIG_5 / PD13) once per
       * radar frame and adjust the ADAR1000 VGA common gain. PR-AB.b
       * repurposed DIG_6 to carry the FPGA frame_pulse, so the runtime
       * enable/debounce path is gone — production firmware runs AGC
       * unconditionally (always-on policy). Bench builds compile this body
       * out by defining MCU_AGC_FORCE_DISABLED, which lets engineers drive
       * the ADAR VGA manually via adarManager / GUI gain widgets without
       * the AGC fighting them. The host opcode-0x28 register stays plumbed
       * for telemetry display; the MCU just no longer reads it. */
#ifndef MCU_AGC_FORCE_DISABLED
      {
          bool sat = HAL_GPIO_ReadPin(FPGA_DIG5_SAT_GPIO_Port,
                                      FPGA_DIG5_SAT_Pin) == GPIO_PIN_SET;
          outerAgc.update(sat);
          outerAgc.applyGain(adarManager);
      }
#endif

      /* [GAP-3 FIX 2] Kick hardware watchdog — if we don't reach here within
       * ~4 s, the IWDG resets the MCU automatically. */
      HAL_IWDG_Refresh(&hiwdg);

      /* F-2.3: removed stale "Optional: Add system monitoring here / Check
       * temperatures, power levels, etc." TODO -- those checks are already
       * performed above (temperature + IDQ at the 5 s tick, LO lock at the
       * 5 s tick, ADF4382/ADAR/IMU/BMP/GPS/FPGA-DSP via checkSystemHealth). */


    /* USER CODE END WHILE */




    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* F-3.7: VOS3 + 72 MHz SYSCLK is intentional -- MCU is control-plane only,
   * heavy DSP runs FPGA-side. F7 can hit 216 MHz @ VOS1 if MCU work demands. */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00808CD2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00808CD2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00808CD2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, AD9523_PD_Pin|AD9523_REF_SEL_Pin|AD9523_SYNC_Pin|AD9523_RESET_Pin
                          |AD9523_CS_Pin|AD9523_EEPROM_SEL_Pin|LED_1_Pin|LED_2_Pin
                          |LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADAR_1_CS_3V3_Pin|ADAR_2_CS_3V3_Pin|ADAR_3_CS_3V3_Pin|ADAR_4_CS_3V3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EN_P_5V0_PA1_Pin|EN_P_5V0_PA2_Pin|EN_P_5V0_PA3_Pin|EN_P_5V5_PA_Pin
                          |EN_P_1V8_CLOCK_Pin|EN_P_3V3_CLOCK_Pin|ADF4382_RX_DELADJ_Pin|ADF4382_RX_DELSTR_Pin
                          |ADF4382_RX_CE_Pin|ADF4382_RX_CS_Pin|ADF4382_TX_DELSTR_Pin|ADF4382_TX_DELADJ_Pin
                          |ADF4382_TX_CS_Pin|ADF4382_TX_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_P_1V0_FPGA_Pin|EN_P_1V8_FPGA_Pin|EN_P_3V3_FPGA_Pin|EN_P_5V0_ADAR_Pin
                          |EN_P_3V3_ADAR12_Pin|EN_P_3V3_ADAR34_Pin|EN_P_3V3_ADTR_Pin|EN_P_3V3_SW_Pin
                          |EN_P_3V3_VDD_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  // PD9 + PD10 retired in PR-AB.b expanded (formerly new_elevation / new_azimuth).
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12
                          |STEPPER_CW_P_Pin|STEPPER_CLK_P_Pin|EN_DIS_RFPA_VDD_Pin|EN_DIS_COOLING_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAC_1_VG_CLR_Pin|DAC_1_VG_LDAC_Pin|DAC_2_VG_CLR_Pin|DAC_2_VG_LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AD9523_PD_Pin AD9523_REF_SEL_Pin AD9523_SYNC_Pin AD9523_RESET_Pin
                           AD9523_CS_Pin AD9523_EEPROM_SEL_Pin LED_1_Pin LED_2_Pin
                           LED_3_Pin LED_4_Pin */
  GPIO_InitStruct.Pin = AD9523_PD_Pin|AD9523_REF_SEL_Pin|AD9523_SYNC_Pin|AD9523_RESET_Pin
                          |AD9523_CS_Pin|AD9523_EEPROM_SEL_Pin|LED_1_Pin|LED_2_Pin
                          |LED_3_Pin|LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : AD9523_STATUS0_Pin AD9523_STATUS1_Pin */
  GPIO_InitStruct.Pin = AD9523_STATUS0_Pin|AD9523_STATUS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ADAR_1_CS_3V3_Pin ADAR_2_CS_3V3_Pin ADAR_3_CS_3V3_Pin ADAR_4_CS_3V3_Pin */
  GPIO_InitStruct.Pin = ADAR_1_CS_3V3_Pin|ADAR_2_CS_3V3_Pin|ADAR_3_CS_3V3_Pin|ADAR_4_CS_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_P_5V0_PA1_Pin EN_P_5V0_PA2_Pin EN_P_5V0_PA3_Pin EN_P_5V5_PA_Pin
                           EN_P_1V8_CLOCK_Pin EN_P_3V3_CLOCK_Pin ADF4382_RX_DELADJ_Pin ADF4382_RX_DELSTR_Pin
                           ADF4382_RX_CE_Pin ADF4382_RX_CS_Pin ADF4382_TX_DELSTR_Pin ADF4382_TX_DELADJ_Pin
                           ADF4382_TX_CS_Pin ADF4382_TX_CE_Pin */
  GPIO_InitStruct.Pin = EN_P_5V0_PA1_Pin|EN_P_5V0_PA2_Pin|EN_P_5V0_PA3_Pin|EN_P_5V5_PA_Pin
                          |EN_P_1V8_CLOCK_Pin|EN_P_3V3_CLOCK_Pin|ADF4382_RX_DELADJ_Pin|ADF4382_RX_DELSTR_Pin
                          |ADF4382_RX_CE_Pin|ADF4382_RX_CS_Pin|ADF4382_TX_DELSTR_Pin|ADF4382_TX_DELADJ_Pin
                          |ADF4382_TX_CS_Pin|ADF4382_TX_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_P_1V0_FPGA_Pin EN_P_1V8_FPGA_Pin EN_P_3V3_FPGA_Pin EN_P_5V0_ADAR_Pin
                           EN_P_3V3_ADAR12_Pin EN_P_3V3_ADAR34_Pin EN_P_3V3_ADTR_Pin EN_P_3V3_SW_Pin
                           EN_P_3V3_VDD_SW_Pin */
  GPIO_InitStruct.Pin = EN_P_1V0_FPGA_Pin|EN_P_1V8_FPGA_Pin|EN_P_3V3_FPGA_Pin|EN_P_5V0_ADAR_Pin
                          |EN_P_3V3_ADAR12_Pin|EN_P_3V3_ADAR34_Pin|EN_P_3V3_ADTR_Pin|EN_P_3V3_SW_Pin
                          |EN_P_3V3_VDD_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD11 PD12
                           STEPPER_CW_P_Pin STEPPER_CLK_P_Pin EN_DIS_RFPA_VDD_Pin EN_DIS_COOLING_Pin
                           (PD9 / PD10 retired in PR-AB.b expanded — default high-Z input) */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12
                          |STEPPER_CW_P_Pin|STEPPER_CLK_P_Pin|EN_DIS_RFPA_VDD_Pin|EN_DIS_COOLING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD15 — polled inputs
   *   PD13 (FPGA_DIG5_SAT): saturation flag for AGC outer loop
   *   PD15 (FPGA_DIG7):     control-fault OR, polled by checkSystemHealth */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 — PR-AB.b FPGA frame_pulse, EXTI rising edge.
   * Pulldown so a disconnected FPGA can't ring the line and starve the dwell
   * loop with phantom pulses (the FPGA-side stretcher drives 3.3V CMOS active
   * high for ~100 ns). NVIC enable below routes EXTI15_10_IRQn to the
   * handler in stm32f7xx_it.c. */
  GPIO_InitStruct.Pin  = FPGA_FRAME_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(FPGA_FRAME_PULSE_GPIO_Port, &GPIO_InitStruct);

  /* Priority below USB_OTG_FS (0) and SysTick — frame_pulse ISR is short
   * (one volatile increment) and a couple of ms of jitter is invisible to
   * the dwell loop, so it doesn't need top priority. */
  HAL_NVIC_SetPriority(FPGA_FRAME_PULSE_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FPGA_FRAME_PULSE_EXTI_IRQn);

  /*Configure GPIO pins : ADF4382_RX_LKDET_Pin ADF4382_TX_LKDET_Pin */
  GPIO_InitStruct.Pin = ADF4382_RX_LKDET_Pin|ADF4382_TX_LKDET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_DRDY_Pin ACC_INT_Pin GYR_INT_Pin */
  GPIO_InitStruct.Pin = MAG_DRDY_Pin|ACC_INT_Pin|GYR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_1_VG_CLR_Pin DAC_1_VG_LDAC_Pin DAC_2_VG_CLR_Pin DAC_2_VG_LDAC_Pin */
  GPIO_InitStruct.Pin = DAC_1_VG_CLR_Pin|DAC_1_VG_LDAC_Pin|DAC_2_VG_CLR_Pin|DAC_2_VG_LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ============================================================
 *                  DEVICE INITIALIZATION
 * ============================================================ */

/**
 * [GAP-3 FIX 2] Initialize Independent Watchdog (IWDG).
 *
 * LSI clock ≈ 32 kHz.
 * Prescaler = 256  → IWDG counter clock ≈ 125 Hz (8 ms/tick).
 * Reload   = 500   → timeout ≈ 500 × 8 ms = 4.0 s.
 *
 * The main loop must call HAL_IWDG_Refresh() within this window
 * or the MCU hard-resets — protecting against firmware hangs when
 * PA rails may be energized.
 */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload    = 500;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        DIAG_ERR("SYS", "IWDG init FAILED -- continuing without hardware watchdog");
    } else {
        DIAG("SYS", "IWDG hardware watchdog started (timeout ~4s)");
    }
}



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  /* F-3.8: SRD=0x87 -> subregions {0,1,2,7} bypass NO_ACCESS (code/SRAM/APB/
   * system region pass via background privileged default); subregions 3-6
   * (0x6000_0000-0xDFFF_FFFF, FMC/QSPI/AHB3 aliases unused here) get blocked. */
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* F-3.1: reset instead of hang. Error_Handler is invoked by every
   * MX_*_Init() helper before MX_IWDG_Init() runs, so an infinite spin
   * here would brick the MCU on any transient boot-time glitch with no
   * watchdog to recover. SystemReset turns a hard-to-debug brick into a
   * visible reboot loop. */
  NVIC_SystemReset();
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
