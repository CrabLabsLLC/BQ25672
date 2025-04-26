/**
 * @file bq25672.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #include "bq25672.h"
 #include "bq25672_register.h" // Include the new register definitions
 
/* -------------------------------------------------------------------------- */
/*                              Private Helpers                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Internal helper to read a byte from a device register.
 */
static bq25672_status_t _bq25672_read_byte(bq25672_t * const dev, const bq25672_reg_t reg, uint8_t * const data)
{
    if ((dev == NULL) || (dev->hal.i2c_read == NULL) || (data == NULL)) {
        return BQ25672_INVALID_PARAM;
    }
    if (!dev->hal.i2c_read(BQ25672_I2C_ADDR, (uint8_t)reg, data, 1U)) {
        return BQ25672_COMM_FAIL;
    }
    return BQ25672_OK;
}

/**
 * @brief Internal helper to write a byte to a device register.
 */
static bq25672_status_t _bq25672_write_byte(bq25672_t * const dev, const bq25672_reg_t reg, const uint8_t data)
{
    if ((dev == NULL) || (dev->hal.i2c_write == NULL)) {
        return BQ25672_INVALID_PARAM;
    }
    if (!dev->hal.i2c_write(BQ25672_I2C_ADDR, (uint8_t)reg, &data, 1U)) {
        return BQ25672_COMM_FAIL;
    }
    return BQ25672_OK;
}

/* -------------------------------------------------------------------------- */
/*                                Initialization                              */
/* -------------------------------------------------------------------------- */

bq25672_status_t bq25672_init(bq25672_t * const dev)
{
    // Initialize driver instance (no hardware reset)
    if (dev == NULL) {
        return BQ25672_INVALID_PARAM;
    }
    return BQ25672_OK;
}

bq25672_status_t bq25672_reset_device(bq25672_t * const dev)
{
    // Perform a software reset by setting RESET bit in CHARGER_CONTROL_5 register
    return _bq25672_write_byte(dev, BQ25672_REG_CHARGER_CONTROL_5, BQ25672_RESET_MASK);
}

bq25672_status_t bq25672_configure(bq25672_t * const dev, const bq25672_config_t * const config)
{
    // Apply complete configuration to the device
    if ((dev == NULL) || (config == NULL)) {
        return BQ25672_INVALID_PARAM;
    }

    dev->config = *config;

    bq25672_status_t status = BQ25672_OK;
    status |= bq25672_configure_system(dev, &config->system_config);
    status |= bq25672_configure_charging(dev, &config->charge_config);
    status |= bq25672_configure_usb(dev, &config->usb_config);
    status |= bq25672_configure_timers(dev, &config->timer_config);
    status |= bq25672_configure_watchdog(dev, &config->watchdog_config);
    status |= bq25672_configure_mppt(dev, &config->mppt_config);
    status |= bq25672_configure_ntc(dev, &config->ntc_config);
    status |= bq25672_configure_jeita(dev, &config->jeita_config);

    return status;
}

 /* -------------------------------------------------------------------------- */
/*                               Charger Control                              */
/* -------------------------------------------------------------------------- */

bq25672_status_t bq25672_set_charge_voltage(bq25672_t * const dev, const uint16_t voltage_mv)
{
    // Set charge voltage (VREG)
    // Encoded as (voltage_mv - 3000mV) / 10mV step
    if ((voltage_mv < 3000U) || (voltage_mv > 18800U)) {
        return BQ25672_INVALID_PARAM;
    }

    uint8_t reg_val = (voltage_mv - 3000U) / 10U;
    return _bq25672_write_byte(dev, BQ25672_REG_CHARGE_VOLTAGE_LIMIT, reg_val);
}

bq25672_status_t bq25672_set_charge_current(bq25672_t * const dev, const uint16_t current_ma)
{
    // Set fast charge current
    // Encoded as (current_ma - 50mA) / 10mA step
    if ((current_ma < 50U) || (current_ma > 3000U)) {
        return BQ25672_INVALID_PARAM;
    }

    uint8_t reg_val = (current_ma - 50U) / 10U;
    return _bq25672_write_byte(dev, BQ25672_REG_CHARGE_CURRENT_LIMIT, reg_val);
}

bq25672_status_t bq25672_enable_charging(bq25672_t * const dev)
{
    // Enable battery charging
    // Clear DISABLE_CHARGER bit in CHARGER_CONTROL_4
    return bq25672_update_register(dev, BQ25672_REG_CHARGER_CONTROL_4, BQ25672_DISABLE_CHARGER_MASK, 0x00U);
}

bq25672_status_t bq25672_disable_charging(bq25672_t * const dev)
{
    // Disable battery charging
    // Set DISABLE_CHARGER bit in CHARGER_CONTROL_4
    return bq25672_update_register(dev, BQ25672_REG_CHARGER_CONTROL_4, BQ25672_DISABLE_CHARGER_MASK, BQ25672_DISABLE_CHARGER_MASK);
}

bq25672_status_t bq25672_enable_otg(bq25672_t * const dev)
{
    // Enable OTG boost mode
    // Set ENABLE_OTG bit in CHARGER_CONTROL_0
    return bq25672_update_register(dev, BQ25672_REG_CHARGER_CONTROL_0, BQ25672_ENABLE_OTG_MASK, BQ25672_ENABLE_OTG_MASK);
}

bq25672_status_t bq25672_disable_otg(bq25672_t * const dev)
{
    // Disable OTG boost mode
    // Clear ENABLE_OTG bit in CHARGER_CONTROL_0
    return bq25672_update_register(dev, BQ25672_REG_CHARGER_CONTROL_0, BQ25672_ENABLE_OTG_MASK, 0x00U);
}

/* -------------------------------------------------------------------------- */
/*                                 Status APIs                                */
/* -------------------------------------------------------------------------- */

bq25672_status_t bq25672_read_fault_status(bq25672_t * const dev, uint8_t * const fault0, uint8_t * const fault1)
{
    // Read fault status registers
    bq25672_status_t status = BQ25672_OK;
    status |= _bq25672_read_byte(dev, BQ25672_REG_FAULT_STATUS_0, fault0);
    status |= _bq25672_read_byte(dev, BQ25672_REG_FAULT_STATUS_1, fault1);
    return status;
}

bq25672_status_t bq25672_clear_flags(bq25672_t * const dev)
{
    // Clear all latched flags by reading flag registers
    uint8_t dummy = 0;
    (void)_bq25672_read_byte(dev, BQ25672_REG_CHARGER_FLAG_0, &dummy);
    (void)_bq25672_read_byte(dev, BQ25672_REG_CHARGER_FLAG_1, &dummy);
    (void)_bq25672_read_byte(dev, BQ25672_REG_CHARGER_FLAG_2, &dummy);
    (void)_bq25672_read_byte(dev, BQ25672_REG_CHARGER_FLAG_3, &dummy);
    (void)_bq25672_read_byte(dev, BQ25672_REG_FAULT_FLAG_0, &dummy);
    (void)_bq25672_read_byte(dev, BQ25672_REG_FAULT_FLAG_1, &dummy);
    return BQ25672_OK;
}

bq25672_status_t bq25672_read_status(bq25672_t * const dev, uint8_t * const status)
{
    // Read general charger status register
    return _bq25672_read_byte(dev, BQ25672_REG_CHARGER_STATUS_0, status);
}

bq25672_status_t bq25672_read_adc(bq25672_t * const dev, const bq25672_reg_t reg_lsb, uint16_t * const value)
{
    // Read a 10-bit ADC measurement (VBAT, VBUS, IBUS, etc.)
    uint8_t lsb = 0, msb = 0;
    bq25672_status_t status = _bq25672_read_byte(dev, reg_lsb, &lsb);
    status |= _bq25672_read_byte(dev, (bq25672_reg_t)((uint8_t)reg_lsb + 1U), &msb);

    *value = ((uint16_t)msb << 2U) | (lsb >> 6U);
    return status;
}

/* -------------------------------------------------------------------------- */
/*                             Low-Level Register Access                      */
/* -------------------------------------------------------------------------- */

bq25672_status_t bq25672_read_register(bq25672_t * const dev, const bq25672_reg_t reg, uint8_t * const value)
{
    // Read a single register
    return _bq25672_read_byte(dev, reg, value);
}

bq25672_status_t bq25672_write_register(bq25672_t * const dev, const bq25672_reg_t reg, const uint8_t value)
{
    // Write a single register
    return _bq25672_write_byte(dev, reg, value);
}

bq25672_status_t bq25672_update_register(bq25672_t * const dev, const bq25672_reg_t reg, const uint8_t mask, const uint8_t value)
{
    // Update specific bits in a register
    uint8_t reg_val = 0;
    bq25672_status_t status = _bq25672_read_byte(dev, reg, &reg_val);
    if (status != BQ25672_OK) {
        return status;
    }
    reg_val = (reg_val & ~mask) | (value & mask);
    return _bq25672_write_byte(dev, reg, reg_val);
}

/* -------------------------------------------------------------------------- */
/*                        Subsystem Configuration (Write)                    */
/* -------------------------------------------------------------------------- */

bq25672_status_t bq25672_configure_charging(bq25672_t * const dev, const bq25672_charge_config_t * const config)
{
    // Configure charge settings (VREG, ICHG, input limits, precharge, termination)
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    bq25672_status_t status = BQ25672_OK;
    status |= bq25672_set_charge_voltage(dev, config->charge_voltage_max_mv);
    status |= bq25672_set_charge_current(dev, config->charge_current_max_ma);
    status |= bq25672_write_register(dev, BQ25672_REG_INPUT_VOLTAGE_LIMIT, (config->input_voltage_max_mv - 3200U) / 100U);
    status |= bq25672_write_register(dev, BQ25672_REG_INPUT_CURRENT_LIMIT, config->input_current_max_ma / 50U);
    status |= bq25672_write_register(dev, BQ25672_REG_PRECHARGE_CONTROL, (uint8_t)config->precharge_voltage);
    status |= bq25672_write_register(dev, BQ25672_REG_TERMINATION_CONTROL, config->termination_current_ma / 10U);
    return status;
}

bq25672_status_t bq25672_configure_system(bq25672_t * const dev, const bq25672_system_config_t * const config)
{
    // Configure system-level charger behavior (PFM, LDO, switching frequency)
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg_val = 0;
    reg_val |= (config->disable_pfm         ? BQ25672_DISABLE_PFM_MASK : 0x00U);
    reg_val |= (config->disable_ldo          ? BQ25672_DISABLE_LDO_MASK : 0x00U);
    reg_val |= (config->disable_ooa          ? BQ25672_DISABLE_OOA_MASK : 0x00U);
    reg_val |= (config->disable_stat         ? BQ25672_DISABLE_STAT_MASK : 0x00U);
    reg_val |= (config->enable_floating_mode ? 0x08U : 0x00U); // Floating bit (not explicitly masked)
    reg_val |= (uint8_t)(config->switching_freq);

    return bq25672_write_register(dev, BQ25672_REG_CHARGER_CONTROL_0, reg_val);
}

bq25672_status_t bq25672_configure_usb(bq25672_t * const dev, const bq25672_usb_config_t * const config)
{
    // Configure OTG (boost) output settings
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    bq25672_status_t status = BQ25672_OK;
    if (config->enable_otg) {
        status |= bq25672_enable_otg(dev);
        status |= bq25672_write_register(dev, BQ25672_REG_VOTG_REGULATION, (config->voltage_mv - 4800U) / 64U);
        status |= bq25672_write_register(dev, BQ25672_REG_IOTG_REGULATION, config->current_max_ma / 50U);
    } else {
        status |= bq25672_disable_otg(dev);
    }
    return status;
}

bq25672_status_t bq25672_configure_watchdog(bq25672_t * const dev, const bq25672_watchdog_config_t * const config)
{
    // Configure watchdog timer timeout
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg_val = (uint8_t)(config->time) << 4U;
    return bq25672_update_register(dev, BQ25672_REG_CHARGER_CONTROL_2, BQ25672_WATCHDOG_TIMER_MASK, reg_val);
}

bq25672_status_t bq25672_configure_timers(bq25672_t * const dev, const bq25672_timer_config_t * const config)
{
    // Configure charger timer settings (precharge, fast charge, top-off)
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg_val = 0;
    reg_val |= (config->top_off_timer_enable     ? BQ25672_TOP_OFF_TIMER_ENABLE_MASK : 0x00U);
    reg_val |= (config->trickle_timer_enable     ? BQ25672_TRICKLE_TIMER_ENABLE_MASK : 0x00U);
    reg_val |= (config->precharge_timer_short    ? BQ25672_PRECHARGE_TIMER_SHORT_MASK : 0x00U);
    reg_val |= (config->precharge_timer_enable   ? BQ25672_PRECHARGE_TIMER_ENABLE_MASK : 0x00U);
    reg_val |= (config->fast_charge_timer_enable ? BQ25672_FAST_CHARGE_TIMER_ENABLE_MASK : 0x00U);

    return bq25672_write_register(dev, BQ25672_REG_TIMER_CONTROL, reg_val);
}

bq25672_status_t bq25672_configure_mppt(bq25672_t * const dev, const bq25672_mppt_config_t * const config)
{
    // Configure MPPT voltage ratio (maximum power point tracking)
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg_val = (uint8_t)(config->voc_ratio & BQ25672_MPPT_VOC_RATIO_MASK);
    return bq25672_write_register(dev, BQ25672_REG_MPPT_CONTROL, reg_val);
}

bq25672_status_t bq25672_configure_ntc(bq25672_t * const dev, const bq25672_ntc_config_t * const config)
{
    // Configure NTC cold and hot temperature thresholds
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    bq25672_status_t status = BQ25672_OK;
    status |= bq25672_write_register(dev, BQ25672_REG_NTC_CONTROL_0, (uint8_t)(config->cold_threshold & BQ25672_NTC_COLD_THRESHOLD_MASK));
    status |= bq25672_write_register(dev, BQ25672_REG_NTC_CONTROL_1, (uint8_t)(config->hot_threshold & BQ25672_NTC_HOT_THRESHOLD_MASK));
    return status;
}

bq25672_status_t bq25672_configure_jeita(bq25672_t * const dev, const bq25672_jeita_config_t * const config)
{
    // Configure JEITA profile (temperature-based voltage/current adjustments)
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg_val = 0;
    reg_val |= ((uint8_t)(config->high_temp_voltage_setting << 5U) & BQ25672_HIGH_TEMP_VOLTAGE_SETTING_MASK);
    reg_val |= ((uint8_t)(config->high_temp_current_setting << 3U) & BQ25672_HIGH_TEMP_CURRENT_SETTING_MASK);
    reg_val |= ((uint8_t)(config->cold_temp_current_setting << 1U) & BQ25672_COLD_TEMP_CURRENT_SETTING_MASK);
    reg_val |= (config->jeita_enabled ? BQ25672_JEITA_ENABLE_MASK : 0x00U);

    return bq25672_write_register(dev, BQ25672_REG_TEMPERATURE_CONTROL, reg_val);
}

/* -------------------------------------------------------------------------- */
/*                        Subsystem Configuration (Read)                     */
/* -------------------------------------------------------------------------- */

bq25672_status_t bq25672_read_charging_config(bq25672_t * const dev, bq25672_charge_config_t * const config)
{
    // Read charge voltage, current, and input limits
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg = 0;
    bq25672_status_t status = BQ25672_OK;

    status |= bq25672_read_register(dev, BQ25672_REG_CHARGE_VOLTAGE_LIMIT, &reg);
    config->charge_voltage_max_mv = (reg * 10U) + 3000U;

    status |= bq25672_read_register(dev, BQ25672_REG_CHARGE_CURRENT_LIMIT, &reg);
    config->charge_current_max_ma = (reg * 10U) + 50U;

    status |= bq25672_read_register(dev, BQ25672_REG_INPUT_VOLTAGE_LIMIT, &reg);
    config->input_voltage_max_mv = (reg * 100U) + 3200U;

    status |= bq25672_read_register(dev, BQ25672_REG_INPUT_CURRENT_LIMIT, &reg);
    config->input_current_max_ma = reg * 50U;

    status |= bq25672_read_register(dev, BQ25672_REG_PRECHARGE_CONTROL, &reg);
    config->precharge_voltage = (bq25672_precharge_voltage_t)(reg & 0x03U);

    status |= bq25672_read_register(dev, BQ25672_REG_TERMINATION_CONTROL, &reg);
    config->termination_current_ma = reg * 10U;

    return status;
}

bq25672_status_t bq25672_read_system_config(bq25672_t * const dev, bq25672_system_config_t * const config)
{
    // Read charger system settings
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg = 0;
    bq25672_status_t status = bq25672_read_register(dev, BQ25672_REG_CHARGER_CONTROL_0, &reg);

    config->disable_pfm          = (reg & BQ25672_DISABLE_PFM_MASK) ? true : false;
    config->disable_ldo          = (reg & BQ25672_DISABLE_LDO_MASK) ? true : false;
    config->disable_ooa          = (reg & BQ25672_DISABLE_OOA_MASK) ? true : false;
    config->disable_stat         = (reg & BQ25672_DISABLE_STAT_MASK) ? true : false;
    config->enable_floating_mode = (reg & 0x08U) ? true : false;
    config->switching_freq       = (bq25672_switching_freq_t)(reg & BQ25672_SWITCH_FREQ_MASK);

    return status;
}

bq25672_status_t bq25672_read_usb_config(bq25672_t * const dev, bq25672_usb_config_t * const config)
{
    // Read OTG boost settings
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg = 0;
    bq25672_status_t status = BQ25672_OK;

    status |= bq25672_read_register(dev, BQ25672_REG_CHARGER_CONTROL_0, &reg);
    config->enable_otg = (reg & BQ25672_ENABLE_OTG_MASK) ? true : false;

    if (config->enable_otg) {
        status |= bq25672_read_register(dev, BQ25672_REG_VOTG_REGULATION, &reg);
        config->voltage_mv = (reg * 64U) + 4800U;

        status |= bq25672_read_register(dev, BQ25672_REG_IOTG_REGULATION, &reg);
        config->current_max_ma = reg * 50U;
    }

    return status;
}

bq25672_status_t bq25672_read_watchdog_config(bq25672_t * const dev, bq25672_watchdog_config_t * const config)
{
    // Read watchdog timer settings
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg = 0;
    bq25672_status_t status = bq25672_read_register(dev, BQ25672_REG_CHARGER_CONTROL_2, &reg);

    config->time = (bq25672_watchdog_time_t)((reg & BQ25672_WATCHDOG_TIMER_MASK) >> 4U);

    return status;
}

bq25672_status_t bq25672_read_timer_config(bq25672_t * const dev, bq25672_timer_config_t * const config)
{
    // Read charger timer settings
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg = 0;
    bq25672_status_t status = bq25672_read_register(dev, BQ25672_REG_TIMER_CONTROL, &reg);

    config->top_off_timer_enable     = (reg & BQ25672_TOP_OFF_TIMER_ENABLE_MASK) ? true : false;
    config->trickle_timer_enable     = (reg & BQ25672_TRICKLE_TIMER_ENABLE_MASK) ? true : false;
    config->precharge_timer_short    = (reg & BQ25672_PRECHARGE_TIMER_SHORT_MASK) ? true : false;
    config->precharge_timer_enable   = (reg & BQ25672_PRECHARGE_TIMER_ENABLE_MASK) ? true : false;
    config->fast_charge_timer_enable = (reg & BQ25672_FAST_CHARGE_TIMER_ENABLE_MASK) ? true : false;

    return status;
}

bq25672_status_t bq25672_read_mppt_config(bq25672_t * const dev, bq25672_mppt_config_t * const config)
{
    // Read MPPT settings
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg = 0;
    bq25672_status_t status = bq25672_read_register(dev, BQ25672_REG_MPPT_CONTROL, &reg);

    config->voc_ratio = (bq25672_mppt_ratio_t)(reg & BQ25672_MPPT_VOC_RATIO_MASK);

    return status;
}

bq25672_status_t bq25672_read_ntc_config(bq25672_t * const dev, bq25672_ntc_config_t * const config)
{
    // Read NTC cold/hot thresholds
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    bq25672_status_t status = BQ25672_OK;
    status |= bq25672_read_register(dev, BQ25672_REG_NTC_CONTROL_0, (uint8_t*)&config->cold_threshold);
    status |= bq25672_read_register(dev, BQ25672_REG_NTC_CONTROL_1, (uint8_t*)&config->hot_threshold);

    return status;
}

bq25672_status_t bq25672_read_jeita_config(bq25672_t * const dev, bq25672_jeita_config_t * const config)
{
    // Read JEITA settings
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    uint8_t reg = 0;
    bq25672_status_t status = bq25672_read_register(dev, BQ25672_REG_TEMPERATURE_CONTROL, &reg);

    config->high_temp_voltage_setting = (bq25672_jeita_voltage_setting_t)((reg & BQ25672_HIGH_TEMP_VOLTAGE_SETTING_MASK) >> 5U);
    config->high_temp_current_setting = (bq25672_jeita_current_setting_t)((reg & BQ25672_HIGH_TEMP_CURRENT_SETTING_MASK) >> 3U);
    config->cold_temp_current_setting = (bq25672_jeita_current_setting_t)((reg & BQ25672_COLD_TEMP_CURRENT_SETTING_MASK) >> 1U);
    config->jeita_enabled = (reg & BQ25672_JEITA_ENABLE_MASK) ? true : false;

    return status;
}

bq25672_status_t bq25672_read_config(bq25672_t * const dev, bq25672_config_t * const config)
{
    // Read full configuration of the charger into provided structure
    if ((dev == NULL) || (config == NULL)) return BQ25672_INVALID_PARAM;

    bq25672_status_t status = BQ25672_OK;
    status |= bq25672_read_charging_config(dev, &config->charge_config);
    status |= bq25672_read_system_config(dev, &config->system_config);
    status |= bq25672_read_usb_config(dev, &config->usb_config);
    status |= bq25672_read_watchdog_config(dev, &config->watchdog_config);
    status |= bq25672_read_timer_config(dev, &config->timer_config);
    status |= bq25672_read_mppt_config(dev, &config->mppt_config);
    status |= bq25672_read_ntc_config(dev, &config->ntc_config);
    status |= bq25672_read_jeita_config(dev, &config->jeita_config);

    return status;
}
