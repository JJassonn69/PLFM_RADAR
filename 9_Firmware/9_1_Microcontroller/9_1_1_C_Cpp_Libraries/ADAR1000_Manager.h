// ADAR1000_Manager.h
#ifndef ADAR1000_MANAGER_H
#define ADAR1000_MANAGER_H

#include "stm32f7xx_hal.h"
#include "main.h"

#include <vector>
#include <memory>

class ADAR1000Manager {
public:
    enum class BeamDirection {
        TX = 0,
        RX = 1
    };

    ADAR1000Manager();
    ~ADAR1000Manager();

    // System Management
    bool initializeAllDevices();
    bool performSystemCalibration();

    // Beam Steering
    bool setCustomBeamPattern16(const uint8_t phase_pattern[16], BeamDirection direction);

    // Device Control
    bool setAllDevicesTXMode();
    bool setAllDevicesRXMode();

    // Monitoring and Diagnostics
    float readTemperature(uint8_t deviceIndex);
    bool verifyDeviceCommunication(uint8_t deviceIndex);
    uint8_t readRegister(uint8_t deviceIndex, uint32_t address);
    void writeRegister(uint8_t deviceIndex, uint32_t address, uint8_t value);

    struct ADAR1000Device {
        uint8_t dev_addr;
        bool initialized;
        float temperature;

        ADAR1000Device(uint8_t addr)
            : dev_addr(addr), initialized(false), temperature(25.0f) {
        }
    };

    // Device Management
    std::vector<std::unique_ptr<ADAR1000Device>> devices_;

    // Vector Modulator lookup tables (see ADAR1000_Manager.cpp for provenance).
    // Indexed as VM_*[phase % 128] on a uniform 2.8125 deg grid.
    // No VM_GAIN[] table exists: VM magnitude is bits [4:0] of the I/Q bytes
    // themselves; per-channel VGA gain uses a separate register.
    static const uint8_t VM_I[128];
    static const uint8_t VM_Q[128];

    // Named defaults for the ADTR1107 and ADAR1000 power sequence.
    static constexpr uint8_t kDefaultTxVgaGain = 0x7F;
    static constexpr uint8_t kDefaultRxVgaGain = 30;
    static constexpr uint8_t kLnaBiasOff = 0x00;
    static constexpr uint8_t kLnaBiasOperational = 0x30;
    static constexpr uint8_t kPaBiasTxSafe = 0x5D;
    static constexpr uint8_t kPaBiasIdqCalibration = 0x0D;
    static constexpr uint8_t kPaBiasOperational = 0x7F;
    static constexpr uint8_t kPaBiasRxSafe = 0x20;
    static constexpr uint8_t kTxBiasCurrent = 0x2D;
    static constexpr uint8_t kTxDriverBiasCurrent = 0x06;

    // Private Methods
    bool initializeSingleDevice(uint8_t deviceIndex);
    bool initializeADTR1107Sequence();
    // Re-emit ADTR1107 LNA-off + PA-safe + PA-Idq-cal bias values AFTER
    // initializeAllDevices() has run its soft-reset-then-init flow. The soft
    // reset wipes any bias values set before it, so this MUST be called
    // post-init for the ADTR1107 driver to land at the Idq-tuned bias point.
    bool applyADTRBiasDefaults();
    void delayUs(uint32_t microseconds);

    // Power Management
    void enablePASupplies();
    void disablePASupplies();
    void enableLNASupplies();
    void disableLNASupplies();

    // SPI Communication
    void setChipSelect(uint8_t deviceIndex, bool state);
    uint32_t spiTransfer(uint8_t* txData, uint8_t* rxData, uint32_t size);
    void adarWrite(uint8_t deviceIndex, uint32_t mem_addr, uint8_t data, uint8_t broadcast);
    uint8_t adarRead(uint8_t deviceIndex, uint32_t mem_addr);
    void adarSetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast);
    void adarResetBit(uint8_t deviceIndex, uint32_t mem_addr, uint8_t bit, uint8_t broadcast);
    void adarSoftReset(uint8_t deviceIndex);
    void adarWriteConfigA(uint8_t deviceIndex, uint8_t flags, uint8_t broadcast);
    void adarSetRamBypass(uint8_t deviceIndex, uint8_t broadcast);

    // Channel Configuration
    void adarSetRxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast);
    void adarSetTxPhase(uint8_t deviceIndex, uint8_t channel, uint8_t phase, uint8_t broadcast);
    void adarSetRxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast);
    void adarSetTxVgaGain(uint8_t deviceIndex, uint8_t channel, uint8_t gain, uint8_t broadcast);
    void adarSetTxBias(uint8_t deviceIndex, uint8_t broadcast);
    uint8_t adarAdcRead(uint8_t deviceIndex, uint8_t broadcast);

private:
    // setADTR1107Mode is the wrapper-internal mode switch. Public callers must
    // go through setAllDevicesTXMode() / setAllDevicesRXMode() so the per-channel
    // ADAR1000 RX/TX enables stay in sync with the ADTR1107 bias state.
    void setADTR1107Mode(BeamDirection direction);
};

// Register Definitions
#define BROADCAST_OFF                      0
#define BROADCAST_ON                       1

#define REG_INTERFACE_CONFIG_A             0x000
#define REG_SCRATCHPAD                     0x00A
#define REG_CH1_RX_GAIN                    0x010
#define REG_CH2_RX_GAIN                    0x011
#define REG_CH3_RX_GAIN                    0x012
#define REG_CH4_RX_GAIN                    0x013
#define REG_CH1_RX_PHS_I                   0x014
#define REG_CH1_RX_PHS_Q                   0x015
#define REG_CH2_RX_PHS_I                   0x016
#define REG_CH2_RX_PHS_Q                   0x017
#define REG_CH3_RX_PHS_I                   0x018
#define REG_CH3_RX_PHS_Q                   0x019
#define REG_CH4_RX_PHS_I                   0x01A
#define REG_CH4_RX_PHS_Q                   0x01B
#define REG_CH1_TX_GAIN                    0x01C
#define REG_CH2_TX_GAIN                    0x01D
#define REG_CH3_TX_GAIN                    0x01E
#define REG_CH4_TX_GAIN                    0x01F
#define REG_CH1_TX_PHS_I                   0x020
#define REG_CH1_TX_PHS_Q                   0x021
#define REG_CH2_TX_PHS_I                   0x022
#define REG_CH2_TX_PHS_Q                   0x023
#define REG_CH3_TX_PHS_I                   0x024
#define REG_CH3_TX_PHS_Q                   0x025
#define REG_CH4_TX_PHS_I                   0x026
#define REG_CH4_TX_PHS_Q                   0x027
#define REG_LOAD_WORKING                   0x028
#define REG_PA_CH1_BIAS_ON                 0x029
#define REG_PA_CH2_BIAS_ON                 0x02A
#define REG_PA_CH3_BIAS_ON                 0x02B
#define REG_PA_CH4_BIAS_ON                 0x02C
#define REG_LNA_BIAS_ON                    0x02D
#define REG_RX_ENABLES                     0x02E
#define REG_TX_ENABLES                     0x02F
#define REG_MISC_ENABLES                   0x030
#define REG_SW_CONTROL                     0x031
#define REG_ADC_CONTROL                    0x032
#define REG_ADC_OUT                        0x033
#define REG_BIAS_CURRENT_TX                0x036
#define REG_BIAS_CURRENT_TX_DRV            0x037
#define REG_MEM_CTL                        0x038
#define REG_PA_CH1_BIAS_OFF                0x046
#define REG_PA_CH2_BIAS_OFF                0x047
#define REG_PA_CH3_BIAS_OFF                0x048
#define REG_PA_CH4_BIAS_OFF                0x049
#define REG_LNA_BIAS_OFF                   0x04A

// Register Constants
#define INTERFACE_CONFIG_A_SDO_ACTIVE      ((1 << 4) | (1 << 3))
#define ADAR1000_ADC_2MHZ_CLK              0x00
#define ADAR1000_ADC_EN                    0x60
#define ADAR1000_ADC_ST_CONV               0x70
#define MEM_CTRL_BIAS_RAM_BYPASS           (1 << 5)
#define MEM_CTRL_BEAM_RAM_BYPASS           (1 << 6)
#define LD_WRK_REGS_LDTX_OVERRIDE          (1 << 1)

#endif // ADAR1000_MANAGER_H
