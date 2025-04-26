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
 * @brief Initialize the BQ25672 device.
 *
 * @param dev Pointer to device instance structure.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_init(bq25672_t* const dev, const bq25672_hal_t* const hal);

/**
 * @brief Reset the BQ25672 registers to default values.
 *
 * @param dev Pointer to device instance structure.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_reset_device(const bq25672_t * const dev);

/**
 * @brief Configure the BQ25672 with a full configuration set.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to constant configuration settings.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure(bq25672_t* const dev, const bq25672_config_t * const config);

/**
 * @brief Configure charging profile parameters.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to charging configuration settings.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_charging(bq25672_t* const dev, const bq25672_charge_config_t * const config);

/**
 * @brief Configure system-level behavior.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to system configuration settings.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_system(bq25672_t* const dev, const bq25672_system_config_t * const config);

/**
 * @brief Configure the watchdog timer.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to watchdog timer configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_watchdog(bq25672_t* const dev, const bq25672_watchdog_config_t * const config);

/**
 * @brief Configure OTG/USB boost operation.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to USB configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_usb(bq25672_t* const dev, const bq25672_usb_config_t * const config);

/**
 * @brief Configure the safety timers.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to timer configuration settings.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_timers(bq25672_t* const dev, const bq25672_timer_config_t * const config);

/**
 * @brief Configure Maximum Power Point Tracking (MPPT).
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to MPPT configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_mppt(bq25672_t* const dev, const bq25672_mppt_config_t * const config);

/**
 * @brief Configure NTC thermal monitoring thresholds.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to NTC configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_ntc(bq25672_t* const dev, const bq25672_ntc_config_t * const config);

/**
 * @brief Configure JEITA temperature-based behavior.
 *
 * @param dev Pointer to device instance structure.
 * @param config Pointer to JEITA configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_configure_jeita(bq25672_t* const dev, const bq25672_jeita_config_t * const config);

/**
 * @brief Read the current charging configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved charging configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_charging_config(const bq25672_t* const dev, bq25672_charge_config_t * const config);

/**
 * @brief Read the current system configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved system configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_system_config(const bq25672_t* const dev, bq25672_system_config_t * const config);

/**
 * @brief Read the current USB/OTG configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved USB configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_usb_config(const bq25672_t * const dev, bq25672_usb_config_t * const config);

/**
 * @brief Read the current watchdog timer configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved watchdog timer configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_watchdog_config(const bq25672_t* const dev, bq25672_watchdog_config_t * const config);

/**
 * @brief Read the current timer configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved timer configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_timer_config(const bq25672_t* const dev, bq25672_timer_config_t * const config);

/**
 * @brief Read the current MPPT configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved MPPT configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_mppt_config(const bq25672_t* const dev, bq25672_mppt_config_t * const config);

/**
 * @brief Read the current NTC configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved NTC configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_ntc_config(const bq25672_t* const dev, bq25672_ntc_config_t * const config);

/**
 * @brief Read the current JEITA configuration from the device.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved JEITA configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_jeita_config(const bq25672_t* const dev, bq25672_jeita_config_t * const config);

/**
 * @brief Read all configuration settings from the device into a full config structure.
 *
 * @param dev Pointer to device instance.
 * @param config Pointer to store the retrieved full device configuration.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_config(const bq25672_t* const dev, bq25672_config_t * const config);

/**
 * @brief Set the charge voltage limit.
 *
 * @param dev Pointer to device instance structure.
 * @param voltage_mv Charge voltage in millivolts.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_set_charge_voltage(const bq25672_t * const dev, const uint16_t voltage_mv);

/**
 * @brief Set the charge current limit.
 *
 * @param dev Pointer to device instance structure.
 * @param current_ma Charge current in milliamps.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_set_charge_current(const bq25672_t * const dev, const uint16_t current_ma);

/**
 * @brief Enable battery charging.
 *
 * @param dev Pointer to device instance structure.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_enable_charging(const bq25672_t * const dev);

/**
 * @brief Disable battery charging.
 *
 * @param dev Pointer to device instance structure.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_disable_charging(const bq25672_t * const dev);

/**
 * @brief Enable OTG (boost) operation.
 *
 * @param dev Pointer to device instance structure.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_enable_otg(const bq25672_t * const dev);

/**
 * @brief Disable OTG (boost) operation.
 *
 * @param dev Pointer to device instance structure.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_disable_otg(const bq25672_t * const dev);

/**
 * @brief Read fault status registers.
 *
 * @param dev Pointer to device instance structure.
 * @param fault0 Pointer to store Fault Status 0 value.
 * @param fault1 Pointer to store Fault Status 1 value.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_fault_status(const bq25672_t * const dev, uint8_t * const fault0, uint8_t * const fault1);

/**
 * @brief Clear latched charger and fault flags.
 *
 * @param dev Pointer to device instance structure.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_clear_flags(const bq25672_t * const dev);

/**
 * @brief Read general charger status.
 *
 * @param dev Pointer to device instance structure.
 * @param status Pointer to store charger status byte.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_status(const bq25672_t * const dev, uint8_t * const status);

/**
 * @brief Read ADC channel measurement.
 *
 * @param dev Pointer to device instance structure.
 * @param reg_lsb Register address for ADC LSB byte.
 * @param value Pointer to store 10-bit ADC result.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_adc(const bq25672_t * const dev, const uint8_t reg_lsb, uint16_t * const value);

/**
 * @brief Read a single device register.
 *
 * @param dev Pointer to device instance structure.
 * @param reg Register address to read.
 * @param value Pointer to store register value.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_read_register(const bq25672_t * const dev, const uint8_t reg, uint8_t * const value);

/**
 * @brief Write a single device register.
 *
 * @param dev Pointer to device instance structure.
 * @param reg Register address to write.
 * @param value Value to write to register.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_write_register(const bq25672_t * const dev, const uint8_t reg, const uint8_t value);

/**
 * @brief Update (read-modify-write) bits in a device register.
 *
 * @param dev Pointer to device instance structure.
 * @param reg Register address.
 * @param mask Bit mask selecting bits to modify.
 * @param value New value to apply to masked bits.
 * @return BQ25672_OK on success, error code otherwise.
 */
bq25672_status_t bq25672_update_register(const bq25672_t * const dev, const uint8_t reg, const uint8_t mask, const uint8_t value);

#ifdef __cplusplus
}
#endif

#endif