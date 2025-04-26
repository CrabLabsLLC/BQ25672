/**
 * @file bq25672_types.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#ifndef BQ25672_TYPES_H
#define BQ25672_TYPES_H

#include <stdint.h>
#include <stdbool.h>

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

/// @brief 
typedef bool (*bq25672_gpio_write_fn)(const bool state);

/// @brief 
typedef bool (*bq25672_gpio_read_fn)(bool* const state);

/**
 * @brief Status return codes
 */
typedef enum {
    BQ25672_OK = 0,              ///< Operation successful
    BQ25672_ERROR = -1,          ///< General error
    BQ25672_INVALID_PARAM = -2,  ///< Invalid parameter provided
    BQ25672_COMM_FAIL = -3       ///< Communication with device failed
} bq25672_status_t;

/**
 * @brief Number of battery cells configuration
 */
typedef enum {
    BQ25672_CELL_1S = 0x0, ///< 1-cell configuration
    BQ25672_CELL_2S = 0x1, ///< 2-cell configuration
    BQ25672_CELL_3S = 0x2, ///< 3-cell configuration
    BQ25672_CELL_4S = 0x3  ///< 4-cell configuration
} bq25672_battery_cells_t;

/**
 * @brief Switching frequency options
 */
typedef enum {
    BQ25672_SWITCHING_FREQ_750KHZ  = 0x0, ///< 750kHz switching
    BQ25672_SWITCHING_FREQ_1500KHZ = 0x1  ///< 1.5MHz switching
} bq25672_switching_freq_t;

/**
 * @brief Battery discharge current limit options
 */
typedef enum {
    BQ25672_DISCHARGE_LIMIT_3A   = 0x0, ///< 3A limit
    BQ25672_DISCHARGE_LIMIT_4A   = 0x1, ///< 4A limit
    BQ25672_DISCHARGE_LIMIT_5A   = 0x2, ///< 5A limit
    BQ25672_DISCHARGE_LIMIT_NONE = 0x3  ///< No limit
} bq25672_discharge_limit_t;

/**
 * @brief Precharge voltage cutoff thresholds
 */
typedef enum {
    BQ25672_PRECHARGE_VOLTAGE_15_VREG    = 0x0, ///< 15% of regulation voltage
    BQ25672_PRECHARGE_VOLTAGE_62_2_VREG  = 0x1, ///< 62.2% of regulation voltage
    BQ25672_PRECHARGE_VOLTAGE_66_7_VREG  = 0x2, ///< 66.7% of regulation voltage
    BQ25672_PRECHARGE_VOLTAGE_71_4_VREG  = 0x3  ///< 71.4% of regulation voltage
} bq25672_precharge_voltage_t;

/**
 * @brief Recharge deglitch times
 */
typedef enum {
    BQ25672_RECHARGE_DEGLITCH_64MS   = 0x0, ///< 64 ms
    BQ25672_RECHARGE_DEGLITCH_256MS  = 0x1, ///< 256 ms
    BQ25672_RECHARGE_DEGLITCH_1024MS = 0x2, ///< 1024 ms
    BQ25672_RECHARGE_DEGLITCH_2048MS = 0x3  ///< 2048 ms
} bq25672_recharge_deglitch_t;

/**
 * @brief VAC overvoltage protection thresholds
 */
typedef enum {
    BQ25672_OVP_VOLTAGE_26V = 0x0, ///< 26V threshold
    BQ25672_OVP_VOLTAGE_22V = 0x1, ///< 22V threshold
    BQ25672_OVP_VOLTAGE_12V = 0x2, ///< 12V threshold
    BQ25672_OVP_VOLTAGE_7V  = 0x3  ///< 7V threshold
} bq25672_ovp_voltage_t;

/**
 * @brief Watchdog timer timeout durations
 */
typedef enum {
    BQ25672_WATCHDOG_TIME_500MS  = 0x0, ///< 500ms
    BQ25672_WATCHDOG_TIME_1S     = 0x1, ///< 1 second
    BQ25672_WATCHDOG_TIME_2S     = 0x2, ///< 2 seconds
    BQ25672_WATCHDOG_TIME_10S    = 0x3, ///< 10 seconds
    BQ25672_WATCHDOG_TIME_20S    = 0x4, ///< 20 seconds
    BQ25672_WATCHDOG_TIME_40S    = 0x5, ///< 40 seconds
    BQ25672_WATCHDOG_TIME_80S    = 0x6, ///< 80 seconds
    BQ25672_WATCHDOG_TIME_160S   = 0x7  ///< 160 seconds
} bq25672_watchdog_time_t;

/**
 * @brief MPPT open-circuit voltage tracking ratio
 */
typedef enum {
    BQ25672_MPPT_RATIO_56_25 = 0x0, ///< 56.25%
    BQ25672_MPPT_RATIO_62_5  = 0x1, ///< 62.5%
    BQ25672_MPPT_RATIO_68_75 = 0x2, ///< 68.75%
    BQ25672_MPPT_RATIO_75    = 0x3, ///< 75%
    BQ25672_MPPT_RATIO_81_25 = 0x4, ///< 81.25%
    BQ25672_MPPT_RATIO_87_5  = 0x5, ///< 87.5%
    BQ25672_MPPT_RATIO_93_75 = 0x6, ///< 93.75%
    BQ25672_MPPT_RATIO_100   = 0x7  ///< 100%
} bq25672_mppt_ratio_t;

/**
 * @brief Top-off timer settings
 */
typedef enum {
    BQ25672_TOPOFF_TIMER_DISABLE = 0x0, ///< Disabled
    BQ25672_TOPOFF_TIMER_15MIN   = 0x1, ///< 15 minutes
    BQ25672_TOPOFF_TIMER_30MIN   = 0x2, ///< 30 minutes
    BQ25672_TOPOFF_TIMER_45MIN   = 0x3  ///< 45 minutes
} bq25672_topoff_timer_t;

/**
 * @brief JEITA voltage adjustment options at high temperature
 */
typedef enum {
    BQ25672_JEITA_VSET_NO_CHANGE   = 0x0, ///< No change
    BQ25672_JEITA_VSET_MINUS_100MV = 0x1, ///< -100mV
    BQ25672_JEITA_VSET_MINUS_200MV = 0x2, ///< -200mV
    BQ25672_JEITA_VSET_MINUS_300MV = 0x3, ///< -300mV
    BQ25672_JEITA_VSET_MINUS_400MV = 0x4, ///< -400mV
    BQ25672_JEITA_VSET_MINUS_600MV = 0x5, ///< -600mV
    BQ25672_JEITA_VSET_MINUS_800MV = 0x6, ///< -800mV
    BQ25672_JEITA_VSET_SUSPEND     = 0x7  ///< Suspend charging
} bq25672_jeita_vset_t;

/**
 * @brief JEITA current scaling for high temperature
 */
typedef enum {
    BQ25672_JEITA_ISETH_NO_CHANGE = 0x0, ///< No change
    BQ25672_JEITA_ISETH_20_PERCENT = 0x1, ///< Reduce to 20%
    BQ25672_JEITA_ISETH_40_PERCENT = 0x2, ///< Reduce to 40%
    BQ25672_JEITA_ISETH_SUSPEND    = 0x3  ///< Suspend charging
} bq25672_jeita_iseth_t;

/**
 * @brief JEITA current scaling for cold temperature
 */
typedef enum {
    BQ25672_JEITA_ISETC_NO_CHANGE = 0x0, ///< No change
    BQ25672_JEITA_ISETC_20_PERCENT = 0x1, ///< Reduce to 20%
    BQ25672_JEITA_ISETC_40_PERCENT = 0x2, ///< Reduce to 40%
    BQ25672_JEITA_ISETC_SUSPEND    = 0x3  ///< Suspend charging
} bq25672_jeita_isetc_t;

/**
 * @brief NTC temperature threshold levels
 */
typedef enum {
    BQ25672_NTC_THRESHOLD_NEG20C = 0x0, ///< -20°C threshold
    BQ25672_NTC_THRESHOLD_NEG10C = 0x1, ///< -10°C threshold
    BQ25672_NTC_THRESHOLD_5C     = 0x2, ///< +5°C threshold
    BQ25672_NTC_THRESHOLD_10C    = 0x3, ///< +10°C threshold
    BQ25672_NTC_THRESHOLD_15C    = 0x4, ///< +15°C threshold
    BQ25672_NTC_THRESHOLD_20C    = 0x5, ///< +20°C threshold
    BQ25672_NTC_THRESHOLD_40C    = 0x6, ///< +40°C threshold
    BQ25672_NTC_THRESHOLD_45C    = 0x7, ///< +45°C threshold
    BQ25672_NTC_THRESHOLD_50C    = 0x8, ///< +50°C threshold
    BQ25672_NTC_THRESHOLD_55C    = 0x9, ///< +55°C threshold
    BQ25672_NTC_THRESHOLD_60C    = 0xA, ///< +60°C threshold
    BQ25672_NTC_THRESHOLD_65C    = 0xB  ///< +65°C threshold
} bq25672_ntc_threshold_t;


typedef struct {
    uint16_t charge_voltage_max_mv;     ///< Maximum charge voltage in millivolts
    uint16_t charge_current_max_ma;     ///< Maximum charge current in milliamps
    uint16_t input_voltage_max_mv;      ///< Input voltage limit in millivolts
    uint16_t input_voltage_min_mv;      ///< Minimum input voltage limit in millivolts
    uint16_t input_current_max_ma;      ///< Maximum input current limit in milliamps
    uint8_t termination_current_ma;     ///< Termination current in milliamps
    bq25672_precharge_voltage_t precharge_voltage; ///< Precharge voltage cutoff setting
} bq25672_charge_config_t;

/**
 * @brief Charging configuration parameters
 */
typedef struct {
    uint16_t charge_voltage_max_mv;        ///< Maximum charge voltage in millivolts
    uint16_t charge_current_max_ma;        ///< Maximum charge current in milliamps
    uint16_t input_voltage_max_mv;         ///< Input voltage limit in millivolts
    uint16_t input_voltage_min_mv;         ///< Minimum input voltage limit in millivolts
    uint16_t input_current_max_ma;         ///< Maximum input current limit in milliamps
    uint8_t termination_current_ma;        ///< Termination current in milliamps
    bq25672_precharge_voltage_t precharge_voltage; ///< Precharge voltage cutoff threshold
} bq25672_charge_config_t;

/**
 * @brief System behavior configuration
 */
typedef struct {
    bool disable_pfm;                      ///< Disable Pulse Frequency Modulation mode
    bool disable_ldo;                      ///< Disable internal LDO
    bool disable_ooa;                      ///< Disable Out-Of-Audio mode
    bool disable_stat;                     ///< Disable STAT pin indicator
    bool enable_floating_mode;             ///< Enable floating mode operation
    bool enable_termination;               ///< Enable charge termination
    bool enable_internal_iindpm;            ///< Enable internal input current limit
    bool enable_external_ilim;             ///< Enable external ILIM setting
    bool enable_battery_ocp;               ///< Enable battery overcurrent protection
    bool discharge_current_enable;         ///< Enable discharge current regulation
    bool force_discharge_current;          ///< Force discharge current limit
    bool optimize_input_current;           ///< Enable input current optimization
    bool force_optimize_input_current;     ///< Force input current optimization
    bq25672_switching_freq_t switching_freq; ///< Switching frequency
} bq25672_system_config_t;

/**
 * @brief USB/OTG boost mode configuration
 */
typedef struct {
    bool enable_otg;                       ///< Enable OTG boost mode
    uint16_t voltage_mv;                   ///< OTG output voltage in millivolts
    uint16_t current_max_ma;                ///< OTG maximum output current in milliamps
    bool disable_pfm;                      ///< Disable PFM during OTG
    bool disable_ooa;                      ///< Disable OOA during OTG
    bool enable_12V_hvdcp;                 ///< Enable 12V HVDCP support
    bool enable_9V_hvdcp;                  ///< Enable 9V HVDCP support
    bool enable_hvdcp;                     ///< Enable generic HVDCP support
} bq25672_usb_config_t;

/**
 * @brief Charging timers configuration
 */
typedef struct {
    bool top_off_timer_enable;             ///< Enable top-off timer
    bool trickle_timer_enable;             ///< Enable trickle timer
    bool precharge_timer_short;            ///< Enable short precharge timer
    bool precharge_timer_enable;           ///< Enable precharge timer
    bool fast_charge_timer_enable;         ///< Enable fast charge timer
    bool timer_slowdown_enable;             ///< Enable timer slowdown for low temperatures
} bq25672_timer_config_t;

/**
 * @brief Watchdog timer configuration
 */
typedef struct {
    bool watchdog_disable_charging;        ///< Disable charging if watchdog expires
    bq25672_watchdog_time_t time;           ///< Watchdog timer duration
} bq25672_watchdog_config_t;

/**
 * @brief MPPT (Maximum Power Point Tracking) configuration
 */
typedef struct {
    bool enable_mppt;                      ///< Enable MPPT function
    bq25672_mppt_ratio_t voc_ratio;         ///< Open-circuit voltage tracking ratio
    uint8_t voc_delay_ms;                   ///< MPPT sampling delay in milliseconds
    uint8_t voc_interval_sec;               ///< MPPT sampling interval in seconds
} bq25672_mppt_config_t;

/**
 * @brief NTC (Negative Temperature Coefficient) monitoring configuration
 */
typedef struct {
    bq25672_ntc_threshold_t cold_threshold; ///< Cold temperature threshold
    bq25672_ntc_threshold_t hot_threshold;  ///< Hot temperature threshold
    bool enable_ntc_monitoring;             ///< Enable NTC monitoring
} bq25672_ntc_config_t;

/**
 * @brief JEITA (Temperature based charging adjustment) configuration
 */
typedef struct {
    bq25672_jeita_vset_t high_temp_voltage_setting; ///< Voltage reduction setting at high temp
    bq25672_jeita_iseth_t high_temp_current_setting; ///< Current reduction at high temp
    bq25672_jeita_isetc_t cold_temp_current_setting; ///< Current reduction at cold temp
    bool jeita_enabled;                      ///< Enable JEITA profile
} bq25672_jeita_config_t;

/**
 * @brief Master configuration structure for BQ25672
 */
typedef struct {
    bq25672_system_config_t system_config;   ///< System behavior settings
    bq25672_charge_config_t charge_config;   ///< Charging settings
    bq25672_usb_config_t usb_config;          ///< OTG/USB settings
    bq25672_timer_config_t timer_config;      ///< Timer settings
    bq25672_watchdog_config_t watchdog_config;///< Watchdog settings
    bq25672_mppt_config_t mppt_config;        ///< MPPT settings
    bq25672_ntc_config_t ntc_config;          ///< NTC thresholds
    bq25672_jeita_config_t jeita_config;      ///< JEITA profile settings
} bq25672_config_t;

/**
 * @brief 
 */
typedef struct 
{
    bq25672_i2c_read_fn i2c_read;
    bq25672_i2c_write_fn i2c_write;

#ifdef BQ25672_SIMPLE_HAL

#endif

} bq25672_hal_t;

typedef struct 
{
    bq25672_hal_t hal;
    bq25672_config_t config;
} bq25672_t;

#endif