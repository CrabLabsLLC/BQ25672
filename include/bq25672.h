/**
 * @file bq25672.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#ifndef BQ25672_H
#define BQ25672_H

#include <stdint.h>
#include <stdbool.h>

#define BQ25672_I2C_ADDR 0x6B ///< Default I2C address of the BQ25672

/**
 * @brief Register map for BQ25672
 */
#define BQ25672_REG_MINIMAL_SYSTEM_VOLTAGE     0x00
#define BQ25672_REG_CHARGE_VOLTAGE_LIMIT       0x01
#define BQ25672_REG_CHARGE_CURRENT_LIMIT       0x03
#define BQ25672_REG_INPUT_VOLTAGE_LIMIT        0x05
#define BQ25672_REG_INPUT_CURRENT_LIMIT        0x06
#define BQ25672_REG_PRECHARGE_CONTROL          0x08
#define BQ25672_REG_TERMINATION_CONTROL        0x09
#define BQ25672_REG_RECHARGE_CONTROL           0x0A
#define BQ25672_REG_VOTG_REGULATION            0x0B
#define BQ25672_REG_IOTG_REGULATION            0x0D
#define BQ25672_REG_TIMER_CONTROL              0x0E
#define BQ25672_REG_CHARGER_CONTROL_0          0x0F
#define BQ25672_REG_CHARGER_CONTROL_1          0x10
#define BQ25672_REG_CHARGER_CONTROL_2          0x11
#define BQ25672_REG_CHARGER_CONTROL_3          0x12
#define BQ25672_REG_CHARGER_CONTROL_4          0x13
#define BQ25672_REG_CHARGER_CONTROL_5          0x14
#define BQ25672_REG_MPPT_CONTROL               0x15
#define BQ25672_REG_TEMPERATURE_CONTROL        0x16
#define BQ25672_REG_NTC_CONTROL_0              0x17
#define BQ25672_REG_NTC_CONTROL_1              0x18
#define BQ25672_REG_ICO_CURRENT_LIMIT          0x19
#define BQ25672_REG_CHARGER_STATUS_0           0x1B
#define BQ25672_REG_CHARGER_STATUS_1           0x1C
#define BQ25672_REG_CHARGER_STATUS_2           0x1D
#define BQ25672_REG_CHARGER_STATUS_3           0x1E
#define BQ25672_REG_CHARGER_STATUS_4           0x1F
#define BQ25672_REG_FAULT_STATUS_0             0x20
#define BQ25672_REG_FAULT_STATUS_1             0x21
#define BQ25672_REG_CHARGER_FLAG_0             0x22
#define BQ25672_REG_CHARGER_FLAG_1             0x23
#define BQ25672_REG_CHARGER_FLAG_2             0x24
#define BQ25672_REG_CHARGER_FLAG_3             0x25
#define BQ25672_REG_FAULT_FLAG_0               0x26
#define BQ25672_REG_FAULT_FLAG_1               0x27
#define BQ25672_REG_CHARGER_MASK_0             0x28
#define BQ25672_REG_CHARGER_MASK_1             0x29
#define BQ25672_REG_CHARGER_MASK_2             0x2A
#define BQ25672_REG_CHARGER_MASK_3             0x2B
#define BQ25672_REG_FAULT_MASK_0               0x2C
#define BQ25672_REG_FAULT_MASK_1               0x2D
#define BQ25672_REG_ADC_CONTROL                0x2E
#define BQ25672_REG_ADC_FUNCTION_DISABLE_0     0x2F
#define BQ25672_REG_ADC_FUNCTION_DISABLE_1     0x30
#define BQ25672_REG_IBUS_ADC                   0x31
#define BQ25672_REG_IBAT_ADC                   0x33
#define BQ25672_REG_VBUS_ADC                   0x35
#define BQ25672_REG_VAC1_ADC                   0x37
#define BQ25672_REG_VAC2_ADC                   0x39
#define BQ25672_REG_VBAT_ADC                   0x3B
#define BQ25672_REG_VSYS_ADC                   0x3D
#define BQ25672_REG_TS_ADC                     0x3F
#define BQ25672_REG_TDIE_ADC                   0x41
#define BQ25672_REG_DP_ADC                     0x43
#define BQ25672_REG_DM_ADC                     0x45
#define BQ25672_REG_DPDM_DRIVER                0x47
#define BQ25672_REG_PART_INFORMATION           0x48

/**
 * @brief User-implemented I2C write function type
 * @param device_addr I2C address
 * @param reg_addr Register address
 * @param data Pointer to data buffer
 * @param len Number of bytes to write
 * @return true on success, false on failure
 */
typedef bool (*bq25672_i2c_write_fn)(const uint8_t device_addr, const uint8_t reg_addr, const void* const data, const uint8_t len);

/**
 * @brief User-implemented I2C read function type
 * @param device_addr I2C address
 * @param reg_addr Register address
 * @param data Pointer to data buffer to fill
 * @param len Number of bytes to read
 * @return true on success, false on failure
 */
typedef bool (*bq25672_i2c_read_fn)(const uint8_t device_addr, const uint8_t reg_addr, void* const data, const uint8_t len);

/**
 * @brief General status codes returned by driver functions
 */
typedef enum {
    BQ25672_OK = 0,              ///< Operation successful
    BQ25672_ERROR = -1,          ///< General error
    BQ25672_INVALID_PARAM = -2,  ///< Invalid parameter
    BQ25672_COMM_FAIL = -3       ///< Communication failure
} bq25672_status_t;

/**
 * @brief Battery cell count options
 */
typedef enum {
    BQ25672_CELL_1S = 0, ///< Single cell (1S)
    BQ25672_CELL_2S,     ///< Two cells (2S)
    BQ25672_CELL_3S,     ///< Three cells (3S)
    BQ25672_CELL_4S      ///< Four cells (4S)
} bq25672_battery_cells_t;

/**
 * @brief 
 * 
 */
typedef enum {
    BQ25672_PRECHARGE_VOLTAGE_15_VREG   = 0x0,  ///< 15% of VREG is the precharge cuttoff
    BQ25672_PRECHARGE_VOLTAGE_62_2_VREG = 0x1,  ///< 62.2% of VREG is the precharge cutoff
    BQ25672_PRECHARGE_VOLTAGE_66_7_VREG = 0x2,  ///< 66.7% of VREG is the precharge cutoff
    BQ25672_PRECHARGE_VOLTAGE_71_4_VREG = 0x3,  ///< 71.4% of VREG is the precharge cutoff
} bq25672_precharge_voltage_t;

typedef enum {
    BQ25672_RECHARGE_DEGLITCH_64MS  = 0x0,
    BQ25672_RECHARGE_DEGLITCH_256MS = 0x1,
    BQ25672_RECHARGE_DEGLITCH_1024MS= 0x2,
    BQ25672_RECHARGE_DEGLITCH_2048MS= 0x3
} bq25672_recharge_deglitch_t;

typedef enum {
    BQ25672_OVP_VOLTAGE_26V = 0x0,
    BQ25672_OVP_VOLTAGE_22V = 0x1,
    BQ25672_OVP_VOLTAGE_12V = 0x2,
    BQ25672_OVP_VOLTAGE_7V  = 0x3
} bq25672_ovp_voltage_t;

typedef enum {
    BQ25672_WATCHDOG_TIME_500MS     = 0x0, 
    BQ25672_WATCHDOG_TIME_1S        = 0x1,
    BQ25672_WATCHDOG_TIME_2S        = 0x2, 
    BQ25672_WATCHDOG_TIME_10S       = 0x3,
    BQ25672_WATCHDOG_TIME_20S       = 0x4,
    BQ25672_WATCHDOG_TIME_40S       = 0x5,
    BQ25672_WATCHDOG_TIME_80S       = 0x6,
    BQ25672_WATCHDOG_TIME_160S      = 0x7
} bq25672_watchdog_time_t;

/**
 * @brief Interrupt flag result structure
 */
typedef struct {
    // Charger Flags
    bool input_voltage_limit;
    bool input_current_limit;
    bool precharge_timer_expired;
    bool charge_termination;
    bool jeita_region;
    bool safety_timer_fault;

    // Fault Flags
    bool input_overvoltage;
    bool battery_overvoltage;
    bool battery_undervoltage;
    bool thermal_shutdown;
    bool ts_fault;
    bool watchdog_expired;
    bool battery_not_connected;
} bq25672_irq_flags_t;

/**
 * @brief Charger context containing function pointers for I2C access
 */
typedef struct {
    bq25672_i2c_write_fn i2c_write; ///< Function to perform I2C write
    bq25672_i2c_read_fn i2c_read;   ///< Function to perform I2C read
} bq25672_dev_t;

/**
 * @brief Configuration structure for initializing BQ25672 with default values
 */
typedef struct {
    uint16_t charge_voltage_max_mv;     ///< Charge voltage in millivolts
    uint16_t charge_current_max_ma;     ///< Charge current in milliamps
    uint16_t input_voltage_max_mv;      ///< Input voltage limit in millivolts
    uint16_t input_voltage_min_mv;      ///< Minium voltage limit in millivolts
    uint16_t input_current_max_ma;      ///< Input current limit in milliamps
} bq25672_charge_config_t;

typedef struct bq25672
{
    bq25672_precharge_voltage_t precharge_voltage;
    uint8_t termination_current;
    bool discharge_current_enable;
    bool force_discharge_current;
    bool optimize_input_current;
    bool force_optimize_input_current;
    bool enable_floating_mode;
    bool enable_termination;
} bq25672_adv_config_t;

typedef struct {
    uint8_t cell_count: 2; 
    bq25672_recharge_deglitch_t deglitch;
    uint16_t threshold_offset;
} bq25672_recharge_config_t;

typedef struct {
    uint16_t voltage_mv;        ///< 
    uint16_t current_max_ma;    ///<
    bool precharge_timer_short; ///<

} bq25672_otg_config_t;

typedef struct {
    bool top_off_timer_enable;
    bool trickle_timer_enable;
    bool precharge_timer_enable;
    bool fast_charge_timer_enable;
    bool timer_slowdown_enable;
} bq25672_timer_config_t;

typedef struct {
    bool watchdog_disable_charging;
} bq25672_watchdog_config_t;



// Initialization
/**
 * @brief Initialize the BQ25672 device
 * @param dev Pointer to the device configuration structure
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_init(const bq25672_dev_t *dev);



/**
 * @brief 
 * 
 * @param dev 
 * @return bq25672_status_t 
 */
bq25672_status_t bq25672_hw_enable(const bq25672_dev_t* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return bq25672_status_t 
 */
bq25672_status_t bq25672_hw_disable(const bq25672_dev_t* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return bq25672_status_t 
 */
bq25672_status_t bq25672_sw_enable(const bq25672_dev_t* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return bq25672_status_t 
 */
bq25672_status_t bq25672_sw_disable(const bq25672_dev_t* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @param config 
 * @return bq25672_status_t 
 */
bq25672_status_t bq25672_configure(const bq25672_dev_t* const dev, const bq25672_config_t* const config);

/**
 * @brief Set the charge voltage limit
 * @param dev Pointer to the device configuration structure
 * @param voltage_mv Charge voltage in millivolts (range: 3000 to 18800, step: 10)
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_charge_voltage(const bq25672_dev_t *dev, const uint16_t voltage_mv);

/**
 * @brief Set the charge current limit
 * @param dev Pointer to the device configuration structure
 * @param current_ma Charge current in milliamps (range: 50 to 3000, step: 10)
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_charge_current(const bq25672_dev_t *dev, const uint16_t current_ma);

/**
 * @brief Enable or disable the charger
 * @param dev Pointer to the device configuration structure
 * @param enable Set to true to enable charging, false to disable
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_enable_charging(const bq25672_dev_t *dev, const bool enable);

/**
 * @brief Read the charger status register
 * @param dev Pointer to the device configuration structure
 * @param status Pointer to a variable to store the status
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_read_status(const bq25672_dev_t *dev, uint8_t *status);

/**
 * @brief Read ADC value from a specified register (e.g. VBAT, VBUS, etc.)
 * @param dev Pointer to device structure
 * @param reg_lsb Register address of ADC LSB byte (must be even address)
 * @param value Pointer to store the 10-bit ADC result
 * @return BQ25672_OK on success, error otherwise
 */
bq25672_status_t bq25672_read_adc(const bq25672_dev_t *dev, uint8_t reg_lsb, uint16_t *value);

/**
 * @brief Read fault status registers
 * @param dev Pointer to device
 * @param fault0 Pointer to store Fault Status 0
 * @param fault1 Pointer to store Fault Status 1
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_read_fault_status(const bq25672_dev_t *dev, uint8_t *fault0, uint8_t *fault1);

/**
 * @brief Clear latched fault and charger flags by reading them
 * @param dev Pointer to device
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_clear_flags(const bq25672_dev_t *dev);

/**
 * @brief Set the watchdog timer
 * @param dev Pointer to the device structure
 * @param timeout_minutes Timeout value in minutes (0 disables watchdog)
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_watchdog_timer(const bq25672_dev_t *dev, uint8_t timeout_minutes);

/**
 * @brief Set MPPT regulation voltage
 * @param dev Pointer to the device structure
 * @param mppt_voltage_mv Voltage in millivolts (must match supported step/range)
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_mppt_voltage(const bq25672_dev_t *dev, uint16_t mppt_voltage_mv);

/**
 * @brief Configure DPDM line driver enable
 * @param dev Pointer to the device structure
 * @param enable Whether to enable DPDM driver
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_dpdm_driver_enable(const bq25672_dev_t *dev, bool enable);

/**
 * @brief Set safety timer duration
 * @param dev Pointer to device
 * @param timeout_hours Timeout value in hours (0 disables)
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_safety_timer(const bq25672_dev_t *dev, uint8_t timeout_hours);

/**
 * @brief Configure NTC threshold and behavior
 * @param dev Pointer to the device structure
 * @param cold_threshold_bits Raw register bits for cold threshold
 * @param hot_threshold_bits Raw register bits for hot threshold
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_ntc_thresholds(const bq25672_dev_t *dev, uint8_t cold_threshold_bits, uint8_t hot_threshold_bits);

/**
 * @brief Mask or unmask interrupt sources
 * @param dev Pointer to the device structure
 * @param reg_offset Mask register offset (0x28 to 0x2D)
 * @param mask_bits Bits to set in the mask
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_set_interrupt_mask(const bq25672_dev_t *dev, uint8_t reg_offset, uint8_t mask_bits);

/**
 * @brief Read a single register value
 * @param dev Pointer to the device configuration structure
 * @param reg Register address
 * @param value Pointer to store the register value
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_read_register(const bq25672_dev_t *dev, const uint8_t reg, uint8_t *value);
/**
 * @brief Write a value to a single register
 * @param dev Pointer to the device configuration structure
 * @param reg Register address
 * @param value Value to write
 * @return BQ25672_OK on success, error code otherwise
 */
bq25672_status_t bq25672_write_register(const bq25672_dev_t *dev, const uint8_t reg, const uint8_t value);


#ifdef __cplusplus
}
#endif

#endif // BQ25672_H