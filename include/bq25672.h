/**
 * @file bq25672.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#ifndef BQ25672_H
#define BQ25672_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bq25672_types.h"
#include "bq25672_registers.h"

#define BQ25672_I2C_ADDRESS     0x6B ///< I2C Slave Address

/*==============================================================================
* Initialization & Reset
*============================================================================*/

/**
 * @brief Initialize driver handle, HAL callbacks, and apply config.
 * @param dev     Pointer to driver handle (allocated by user)
 * @param hal     Pointer to HAL callback table (const pointer to const data)
 * @param config  Pointer to config; pass BQ25672_DEFAULT_CFG_PTR or NULL for default
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_init(bq25672_t* const dev,
                            const bq25672_hal_t* const hal,
                            const bq25672_config_t* const config);

/**
 * @brief Perform software reset (toggles RESET bit).
 * @param dev Pointer to driver handle (const pointer to const data)
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_soft_reset(const bq25672_t* const dev);

/*==============================================================================
* Full-structure Configuration
*============================================================================*/

/**
 * @brief Apply a complete configuration struct to the device.
 * @param dev    Pointer to driver handle (const pointer to const data)
 * @param config Pointer to config struct (const pointer to const data)
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_apply_config(const bq25672_t* const dev,
                                    const bq25672_config_t* const config);

/**
 * @brief Read the device’s current configuration into a struct.
 * @param dev    Pointer to driver handle (const pointer to const data)
 * @param config Pointer to config struct to populate
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_read_config(const bq25672_t* const dev,
                                    bq25672_config_t* const config);

/*==============================================================================
* Per-block Configuration
*============================================================================*/
/** @name System Configuration @{ */
bq25672_status_t bq25672_set_system_config(const bq25672_t* const dev, const bq25672_system_config_t* const sys_cfg);
bq25672_status_t bq25672_get_system_config(const bq25672_t* const dev, bq25672_system_config_t* const sys_cfg);
/** @} */

/** @name Charge Configuration @{ */
bq25672_status_t bq25672_set_charge_config(const bq25672_t* const dev, const bq25672_charge_config_t* const chg_cfg);
bq25672_status_t bq25672_get_charge_config(const bq25672_t* const dev, bq25672_charge_config_t* const chg_cfg);
/** @} */

/** @name USB/OTG Configuration @{ */
bq25672_status_t bq25672_set_usb_config(const bq25672_t* const dev, const bq25672_usb_config_t* const usb_cfg);
bq25672_status_t bq25672_get_usb_config(const bq25672_t* const dev, bq25672_usb_config_t* const usb_cfg);
/** @} */

/** @name Timer Configuration @{ */
bq25672_status_t bq25672_set_timer_config(const bq25672_t* const dev, const bq25672_timer_config_t* const t_cfg);
bq25672_status_t bq25672_get_timer_config(const bq25672_t* const dev, bq25672_timer_config_t* const t_cfg);
/** @} */

/** @name Watchdog Configuration @{ */
bq25672_status_t bq25672_set_watchdog_config(const bq25672_t* const dev, const bq25672_watchdog_config_t* const wdt_cfg);
bq25672_status_t bq25672_get_watchdog_config(const bq25672_t* const dev, bq25672_watchdog_config_t* const wdt_cfg);
/** @} */

/** @name MPPT Configuration @{ */
bq25672_status_t bq25672_set_mppt_config(const bq25672_t* const dev, const bq25672_mppt_config_t* const mppt_cfg);
bq25672_status_t bq25672_get_mppt_config(const bq25672_t* const dev, bq25672_mppt_config_t* const mppt_cfg);
/** @} */

/** @name NTC Configuration @{ */
bq25672_status_t bq25672_set_ntc_config(const bq25672_t* const dev, const bq25672_ntc_config_t* const ntc_cfg);
bq25672_status_t bq25672_get_ntc_config(const bq25672_t* const dev, bq25672_ntc_config_t* const ntc_cfg);
/** @} */

/** @name JEITA Configuration @{ */
bq25672_status_t bq25672_set_jeita_config(const bq25672_t* const dev, const bq25672_jeita_config_t* const jeita_cfg);
bq25672_status_t bq25672_get_jeita_config(const bq25672_t* const dev, bq25672_jeita_config_t* const jeita_cfg);
/** @} */

/** @name Interrupt Configuration @{ */
bq25672_status_t bq25672_set_interrupt_config(const bq25672_t* const dev, const bq25672_interrupt_config_t* const cfg);
bq25672_status_t bq25672_get_interrupt_config(const bq25672_t* const dev, bq25672_interrupt_config_t* const cfg);
/** @} */

/*==============================================================================
* Enable / Disable Controls
*============================================================================*/

/**
 * @brief Enable or disable battery charging via CE pin.
 * @param dev    Pointer to driver handle (const pointer to const data)
 * @param enable true = enable charging
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_set_charging_state(const bq25672_t* const dev, const bool enabled);

/**
 * @brief Enable or disable OTG (VBUS boost) mode.
 * @param dev    Pointer to driver handle (const pointer to const data)
 * @param enabled true = enter OTG
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_set_otg_state(const bq25672_t* const dev, const bool enabled);

/**
 * @brief Enter or exit Hi-Z (bypass) mode.
 * @param dev    Pointer to driver handle (const pointer to const data)
 * @param enabled true = Hi-Z
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_set_hiz_state(const bq25672_t* const dev, const bool enabled);

/*==============================================================================
* ADC Readouts
*============================================================================*/

/**
 * @brief Read raw 16-bit ADC from any channel register (MSB at reg, LSB at reg+1).
 * @param dev Pointer to driver handle (const pointer to const data)
 * @param reg Register address (must be one of IBUS_ADC…DM_ADC)
 * @param raw Pointer to receive raw 16-bit result
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_read_adc_raw(const bq25672_t* const dev, const uint8_t reg_low, const uint8_t reg_hi, uint16_t* const raw);

/**
 * @brief Read input current in mA (IBUS_ADC LSB = 1mA).
 * @param dev Pointer to driver handle (const pointer to const data)
 * @param ma  Pointer to receive current in mA
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_get_input_current_ma(const bq25672_t* const dev, uint16_t* const ma);

/**
 * @brief Read input voltage in mV (VBUS_ADC LSB = 1mV).
 * @param dev Pointer to driver handle (const pointer to const data)
 * @param mv  Pointer to receive voltage in mV
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_get_input_voltage_mv(const bq25672_t* const dev, uint16_t* const mv);

/**
 * @brief Read battery current in mA (IBAT_ADC LSB = 1mA).
 * @param dev Pointer to driver handle (const pointer to const data)
 * @param ma  Pointer to receive battery current in mA
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_get_battery_current_ma(const bq25672_t* const dev, uint16_t* const ma);

/**
 * @brief Read battery voltage in mV (VBAT_ADC LSB = 1mV).
 * @param dev Pointer to driver handle (const pointer to const data)
 * @param mv  Pointer to receive battery voltage in mV
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_get_battery_voltage_mv(const bq25672_t* const dev, uint16_t* const mv);

/*==============================================================================
* Status & Interrupt Snapshots
*============================================================================*/

/**
 * @brief Read a snapshot of the device status.
 * @param dev    Pointer to driver handle (const pointer to const data)
 * @param status Pointer to status struct
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_get_status(const bq25672_t* const dev, bq25672_device_status_t* const status);

/**
 * @brief Read a snapshot of all interrupt flags.
 * @param dev   Pointer to driver handle (const pointer to const data)
 * @param intr  Pointer to interrupt-status struct
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_get_interrupt_status(const bq25672_t* const dev, bq25672_interrupt_status_t* const intr);

/*==============================================================================
* Low-level register access
*============================================================================*/

/**
 * @brief Read an arbitrary register.
 * @param dev   Pointer to driver handle (const pointer to const data)
 * @param reg   Register address
 * @param value Pointer to receive byte
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_read_register(const bq25672_t* const dev, uint8_t reg, uint8_t* const value);

/**
 * @brief Write an arbitrary register.
 * @param dev   Pointer to driver handle (const pointer to const data)
 * @param reg   Register address
 * @param value Byte to write
 * @return BQ25672_OK on success
 */
bq25672_status_t bq25672_write_register(const bq25672_t* const dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // BQ25672_H
