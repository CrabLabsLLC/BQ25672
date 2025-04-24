/**
 * @file bq25672.c
 * @author Orion Serup (orion@crablabs.io)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 #include "bq25672.h"

 bq25672_status_t bq25672_set_ntc_thresholds(const bq25672_dev_t *dev, uint8_t cold_threshold_bits, uint8_t hot_threshold_bits) {
    if (!dev) return BQ25672_INVALID_PARAM;
    bq25672_status_t status = bq25672_write_register(dev, BQ25672_REG17_NTC_CONTROL_0, cold_threshold_bits);
    if (status != BQ25672_OK) return status;
    return bq25672_write_register(dev, BQ25672_REG18_NTC_CONTROL_1, hot_threshold_bits);
}

bq25672_status_t bq25672_set_interrupt_mask(const bq25672_dev_t *dev, uint8_t reg_offset, uint8_t mask_bits) {
    if (!dev || reg_offset < BQ25672_REG28_CHARGER_MASK_0 || reg_offset > BQ25672_REG2D_FAULT_MASK_1)
        return BQ25672_INVALID_PARAM;
    return bq25672_write_register(dev, reg_offset, mask_bits);
}

bq25672_status_t bq25672_set_watchdog_timer(const bq25672_dev_t *dev, uint8_t timeout_minutes) {
    if (!dev || timeout_minutes > 40) return BQ25672_INVALID_PARAM;
    uint8_t value = 0;
    if (timeout_minutes >= 40) value = 0x03;
    else if (timeout_minutes >= 20) value = 0x02;
    else if (timeout_minutes >= 10) value = 0x01;
    else value = 0x00; // disable watchdog
    return bq25672_write_register(dev, BQ25672_REG0E_TIMER_CONTROL, value);
}

bq25672_status_t bq25672_set_mppt_voltage(const bq25672_dev_t *dev, uint16_t mppt_voltage_mv) {
    if (!dev || mppt_voltage_mv < 3200 || mppt_voltage_mv > 15800 || mppt_voltage_mv % 100 != 0) return BQ25672_INVALID_PARAM;
    uint8_t value = (mppt_voltage_mv - 3200) / 100;
    return bq25672_write_register(dev, BQ25672_REG15_MPPT_CONTROL, value);
}

bq25672_status_t bq25672_set_dpdm_driver_enable(const bq25672_dev_t *dev, bool enable) {
    if (!dev) return BQ25672_INVALID_PARAM;
    return bq25672_write_register(dev, BQ25672_REG47_DPDM_DRIVER, enable ? 0x01 : 0x00);
}

bq25672_status_t bq25672_set_safety_timer(const bq25672_dev_t *dev, uint8_t timeout_hours) {
    if (!dev || timeout_hours > 20) return BQ25672_INVALID_PARAM;
    uint8_t value = timeout_hours / 5; // 0=disable, 1=5h, 2=10h, 3=15h, 4=20h
    return bq25672_write_register(dev, BQ25672_REG0E_TIMER_CONTROL, value << 2);
}

/**
 * @brief Apply the full configuration to the charger
 */
bq25672_status_t bq25672_apply_config(const bq25672_dev_t *dev, const bq25672_config_t *config) {
    if (!dev || !config) return BQ25672_INVALID_PARAM;

    bq25672_status_t status = bq25672_set_charge_voltage(dev, config->charge_voltage_mv);
    if (status != BQ25672_OK) return status;

    status = bq25672_set_charge_current(dev, config->charge_current_ma);
    if (status != BQ25672_OK) return status;

    status = bq25672_set_input_voltage_limit(dev, config->input_voltage_mv);
    if (status != BQ25672_OK) return status;

    status = bq25672_set_input_current_limit(dev, config->input_current_ma);
    if (status != BQ25672_OK) return status;

    return bq25672_enable_charging(dev, config->enable_charging);
}

/**
 * @brief Set input voltage limit
 */
bq25672_status_t bq25672_set_input_voltage_limit(const bq25672_dev_t *dev, const uint16_t voltage_mv) {
    if (!dev || voltage_mv < 3200 || voltage_mv > 16000 || voltage_mv % 100 != 0) return BQ25672_INVALID_PARAM;
    const uint8_t value = (voltage_mv - 3200) / 100;
    return bq25672_write_register(dev, BQ25672_REG05_INPUT_VOLTAGE_LIMIT, value);
}

/**
 * @brief Set input current limit
 */
bq25672_status_t bq25672_set_input_current_limit(const bq25672_dev_t *dev, const uint16_t current_ma) {
    if (!dev || current_ma < 100 || current_ma > 6400 || current_ma % 50 != 0) return BQ25672_INVALID_PARAM;
    const uint8_t value = (current_ma - 100) / 50;
    return bq25672_write_register(dev, BQ25672_REG06_INPUT_CURRENT_LIMIT, value);
}

/**
 * @brief Read an ADC register and return 10-bit value
 */
bq25672_status_t bq25672_read_adc(const bq25672_dev_t *dev, uint8_t reg_lsb, uint16_t *value) {
    if (!dev || !value) return BQ25672_INVALID_PARAM;
    uint8_t buf[2];
    bq25672_status_t status = read_register(dev, reg_lsb, buf, 2);
    if (status != BQ25672_OK) return status;
    *value = ((uint16_t)buf[1] << 8 | buf[0]) & 0x03FF; // 10-bit ADC
    return BQ25672_OK;
}

/**
 * @brief Read fault status registers (0 and 1)
 */
bq25672_status_t bq25672_read_fault_status(const bq25672_dev_t *dev, uint8_t *fault0, uint8_t *fault1) {
    if (!dev || !fault0 || !fault1) return BQ25672_INVALID_PARAM;
    bq25672_status_t status = bq25672_read_register(dev, BQ25672_REG20_FAULT_STATUS_0, fault0);
    if (status != BQ25672_OK) return status;
    return bq25672_read_register(dev, BQ25672_REG21_FAULT_STATUS_1, fault1);
}

/**
 * @brief Clear fault and status flags by reading their registers
 */
bq25672_status_t bq25672_clear_flags(const bq25672_dev_t *dev) {
    uint8_t discard;
    for (uint8_t reg = BQ25672_REG22_CHARGER_FLAG_0; reg <= BQ25672_REG27_FAULT_FLAG_1; ++reg) {
        bq25672_status_t status = bq25672_read_register(dev, reg, &discard);
        if (status != BQ25672_OK) return status;
    }
    return BQ25672_OK;
}

/**
 * @brief Write multiple bytes to a register
 */
/**
 * @brief Internal helper to write to a BQ25672 register
 * @param dev Pointer to the device configuration structure
 * @param reg Register address to write to
 * @param data Pointer to data buffer to write
 * @param len Number of bytes to write
 * @return BQ25672_OK on success, error code otherwise
 */
static bq25672_status_t write_register(const bq25672_dev_t *dev, const uint8_t reg, const uint8_t *data, const uint8_t len) {
    if (!dev || !dev->i2c_write || !data) return BQ25672_INVALID_PARAM;
    return dev->i2c_write(BQ25672_I2C_ADDR, reg, data, len) ? BQ25672_OK : BQ25672_COMM_FAIL;
}

/**
 * @brief Read multiple bytes from a register
 */
/**
 * @brief Internal helper to read from a BQ25672 register
 * @param dev Pointer to the device configuration structure
 * @param reg Register address to read from
 * @param data Pointer to buffer where data will be stored
 * @param len Number of bytes to read
 * @return BQ25672_OK on success, error code otherwise
 */
static bq25672_status_t read_register(const bq25672_dev_t *dev, const uint8_t reg, uint8_t *data, const uint8_t len) {
    if (!dev || !dev->i2c_read || !data) return BQ25672_INVALID_PARAM;
    return dev->i2c_read(BQ25672_I2C_ADDR, reg, data, len) ? BQ25672_OK : BQ25672_COMM_FAIL;
}

bq25672_status_t bq25672_init(const bq25672_dev_t *dev) {
    if (!dev || !dev->i2c_read || !dev->i2c_write) return BQ25672_INVALID_PARAM;
    uint8_t part_info;
    return bq25672_read_register(dev, BQ25672_REG48_PART_INFORMATION, &part_info);
}

bq25672_status_t bq25672_set_charge_voltage(const bq25672_dev_t *dev, const uint16_t voltage_mv) {
    if (voltage_mv < 3000 || voltage_mv > 18800 || (voltage_mv % 10) != 0) return BQ25672_INVALID_PARAM;
    const uint16_t val = voltage_mv / 10;
    const uint8_t buf[2] = { val & 0xFF, (val >> 8) & 0xFF };
    return write_register(dev, BQ25672_REG_CHARGE_VOLTAGE_LIMIT, buf, 2);
}

bq25672_status_t bq25672_set_charge_current(const bq25672_dev_t *dev, const uint16_t current_ma) {
    if (current_ma < 50 || current_ma > 3000 || (current_ma % 10) != 0) return BQ25672_INVALID_PARAM;
    const uint16_t val = current_ma / 10;
    const uint8_t buf[2] = { val & 0xFF, (val >> 8) & 0xFF };
    return write_register(dev, BQ25672_REG_CHARGE_CURRENT_LIMIT, buf, 2);
}

bq25672_status_t bq25672_enable_charging(const bq25672_dev_t *dev, const bool enable) {
    uint8_t ctrl;
    const bq25672_status_t status = bq25672_read_register(dev, BQ25672_REG_CHARGER_CONTROL_0, &ctrl);
    if (status != BQ25672_OK) return status;
    ctrl = enable ? (ctrl | 0x20) : (ctrl & ~0x20);
    return bq25672_write_register(dev, BQ25672_REG_CHARGER_CONTROL_0, ctrl);
}

/**
 * @brief Read charger status register 0
 */
bq25672_status_t bq25672_read_status(const bq25672_dev_t *dev, uint8_t * const status) {
    return bq25672_read_register(dev, BQ25672_REG1B_CHARGER_STATUS_0, status);
}

/**
 * @brief Read a single byte from a register
 */
bq25672_status_t bq25672_read_register(const bq25672_dev_t *dev, const uint8_t reg, uint8_t * const value) {
    return read_register(dev, reg, value, 1);
}

/**
 * @brief Write a single byte to a register
 */
bq25672_status_t bq25672_write_register(const bq25672_dev_t *dev, const uint8_t reg, const uint8_t value) {
    return write_register(dev, reg, &value, 1);
}