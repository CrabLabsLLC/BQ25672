/**
 * @file test.c
 * @author Orion Serup (orion@crablabs.io)
 * @brief Unit tests for the BQ25672 using FFF and Unity
 * @version 0.1
 * @date 2025-04-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "unity.h"
#include "fff.h"

#include "bq25672.h"
#include "bq25672_registers.h"

DEFINE_FFF_GLOBALS;

/* -------------------------------------------------------------------------- */
/*                               Fakes                                        */
/* -------------------------------------------------------------------------- */

FAKE_VALUE_FUNC4(bool, mock_i2c_write, uint8_t, uint8_t, const void*, uint8_t);
FAKE_VALUE_FUNC4(bool, mock_i2c_read, uint8_t, uint8_t, void*, uint8_t);

/* -------------------------------------------------------------------------- */
/*                               Test Context                                 */
/* -------------------------------------------------------------------------- */

static bq25672_t dev;

/* -------------------------------------------------------------------------- */
/*                          Unity Setup/Teardown                              */
/* -------------------------------------------------------------------------- */

void setUp(void)
{
    FFF_RESET_HISTORY();
    memset(&dev, 0, sizeof(dev));
    dev.hal.i2c_read = mock_i2c_read;
    dev.hal.i2c_write = mock_i2c_write;

    mock_i2c_write_fake.return_val = true;
    mock_i2c_read_fake.return_val = true;
}

void tearDown(void) {}

/* -------------------------------------------------------------------------- */
/*                           Initialization Tests                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialization fails with NULL device.
 */
void test_bq25672_init_null_fails(void)
{
    TEST_ASSERT_EQUAL(BQ25672_INVALID_PARAM, bq25672_init(NULL));
}

/**
 * @brief Initialization succeeds with valid device pointer.
 */
void test_bq25672_init_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_init(&dev));
}

/* -------------------------------------------------------------------------- */
/*                         Low-Level Access Tests                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read register with valid parameters.
 */
void test_bq25672_read_register_success(void)
{
    uint8_t value = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_register(&dev, BQ25672_REG_CHARGE_VOLTAGE_LIMIT, &value));
    TEST_ASSERT_EQUAL(1, mock_i2c_read_fake.call_count);
}

/**
 * @brief Read register with communication failure.
 */
void test_bq25672_read_register_comm_fail(void)
{
    mock_i2c_read_fake.return_val = false;
    uint8_t value = 0;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_read_register(&dev, BQ25672_REG_CHARGE_VOLTAGE_LIMIT, &value));
}

/**
 * @brief Write register with valid parameters.
 */
void test_bq25672_write_register_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_write_register(&dev, BQ25672_REG_INPUT_CURRENT_LIMIT, 0x5AU));
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Write register with communication failure.
 */
void test_bq25672_write_register_comm_fail(void)
{
    mock_i2c_write_fake.return_val = false;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_write_register(&dev, BQ25672_REG_INPUT_CURRENT_LIMIT, 0x5AU));
}

/**
 * @brief Update register with mask and value.
 */
void test_bq25672_update_register_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_update_register(&dev, BQ25672_REG_CHARGER_CONTROL_0, BQ25672_DISABLE_PFM_MASK, 0x00U));
    TEST_ASSERT_EQUAL(1, mock_i2c_read_fake.call_count);
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Update register failure on read.
 */
void test_bq25672_update_register_read_fail(void)
{
    mock_i2c_read_fake.return_val = false;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_update_register(&dev, BQ25672_REG_CHARGER_CONTROL_0, BQ25672_DISABLE_PFM_MASK, 0x00U));
}

/**
 * @brief Update register failure on write.
 */
void test_bq25672_update_register_write_fail(void)
{
    mock_i2c_write_fake.return_val = false;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_update_register(&dev, BQ25672_REG_CHARGER_CONTROL_0, BQ25672_DISABLE_PFM_MASK, 0x00U));
}


 /* -------------------------------------------------------------------------- */
/*                          Charger Control Tests                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Set charge voltage fails when out of valid range.
 */
void test_bq25672_set_charge_voltage_invalid_range(void)
{
    TEST_ASSERT_EQUAL(BQ25672_INVALID_PARAM, bq25672_set_charge_voltage(&dev, 2000U)); // Too low
    TEST_ASSERT_EQUAL(BQ25672_INVALID_PARAM, bq25672_set_charge_voltage(&dev, 19000U)); // Too high
}

/**
 * @brief Set charge voltage succeeds in valid range.
 */
void test_bq25672_set_charge_voltage_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_set_charge_voltage(&dev, 4200U));
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Set charge current fails when out of valid range.
 */
void test_bq25672_set_charge_current_invalid_range(void)
{
    TEST_ASSERT_EQUAL(BQ25672_INVALID_PARAM, bq25672_set_charge_current(&dev, 20U));   // Too low
    TEST_ASSERT_EQUAL(BQ25672_INVALID_PARAM, bq25672_set_charge_current(&dev, 4000U)); // Too high
}

/**
 * @brief Set charge current succeeds in valid range.
 */
void test_bq25672_set_charge_current_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_set_charge_current(&dev, 1500U));
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Enable battery charging succeeds.
 */
void test_bq25672_enable_charging_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_enable_charging(&dev));
    TEST_ASSERT_EQUAL(1, mock_i2c_read_fake.call_count);
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Enable battery charging fails when I2C read fails.
 */
void test_bq25672_enable_charging_i2c_fail_read(void)
{
    mock_i2c_read_fake.return_val = false;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_enable_charging(&dev));
}

/**
 * @brief Disable battery charging succeeds.
 */
void test_bq25672_disable_charging_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_disable_charging(&dev));
    TEST_ASSERT_EQUAL(1, mock_i2c_read_fake.call_count);
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Disable battery charging fails when I2C write fails.
 */
void test_bq25672_disable_charging_i2c_fail_write(void)
{
    mock_i2c_write_fake.return_val = false;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_disable_charging(&dev));
}

/**
 * @brief Enable OTG (boost) mode succeeds.
 */
void test_bq25672_enable_otg_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_enable_otg(&dev));
}

/**
 * @brief Disable OTG (boost) mode succeeds.
 */
void test_bq25672_disable_otg_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_disable_otg(&dev));
}

/* -------------------------------------------------------------------------- */
/*                          Status, Fault, and ADC Tests                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read charger status register successfully.
 */
void test_bq25672_read_status_success(void)
{
    uint8_t status = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_status(&dev, &status));
    TEST_ASSERT_EQUAL(1, mock_i2c_read_fake.call_count);
}

/**
 * @brief Read fault status registers successfully.
 */
void test_bq25672_read_fault_status_success(void)
{
    uint8_t fault0 = 0;
    uint8_t fault1 = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_fault_status(&dev, &fault0, &fault1));
    TEST_ASSERT_EQUAL(2, mock_i2c_read_fake.call_count); // two reads: fault0 and fault1
}

/**
 * @brief Clear charger flags by reading all flag registers.
 */
void test_bq25672_clear_flags_success(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_clear_flags(&dev));
    TEST_ASSERT_EQUAL(6, mock_i2c_read_fake.call_count); // 6 reads for all flags
}

/**
 * @brief Read ADC value successfully.
 */
void test_bq25672_read_adc_success(void)
{
    uint16_t adc_value = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_adc(&dev, BQ25672_REG_VBAT_ADC, &adc_value));
    TEST_ASSERT_NOT_EQUAL(0U, adc_value); // Should not be zero due to dummy read values
    TEST_ASSERT_EQUAL(2, mock_i2c_read_fake.call_count); // two reads: LSB + MSB
}

/**
 * @brief Read ADC fails if first read fails.
 */
void test_bq25672_read_adc_fail_first_read(void)
{
    mock_i2c_read_fake.return_val = false;
    uint16_t adc_value = 0;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_read_adc(&dev, BQ25672_REG_VBAT_ADC, &adc_value));
}

/**
 * @brief Read ADC fails if second read fails.
 */
void test_bq25672_read_adc_fail_second_read(void)
{
    // First call succeeds, second call fails
    mock_i2c_read_fake.return_val = true;
    mock_i2c_read_fake.return_val_seq[0] = true;
    mock_i2c_read_fake.return_val_seq[1] = false;
    mock_i2c_read_fake.return_val_seq_len = 2;

    uint16_t adc_value = 0;
    TEST_ASSERT_EQUAL(BQ25672_COMM_FAIL, bq25672_read_adc(&dev, BQ25672_REG_VBAT_ADC, &adc_value));
}
/* -------------------------------------------------------------------------- */
/*                        Subsystem Configuration (Write) Tests               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configure charging settings successfully.
 */
void test_bq25672_configure_charging_success(void)
{
    bq25672_charge_config_t config = {
        .charge_voltage_max_mv = 4200U,
        .charge_current_max_ma = 1500U,
        .input_voltage_max_mv = 5000U,
        .input_current_max_ma = 2000U,
        .precharge_voltage = BQ25672_PRECHARGE_VOLTAGE_62_2_VREG,
        .termination_current_ma = 150U
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_charging(&dev, &config));
    TEST_ASSERT_TRUE(mock_i2c_write_fake.call_count >= 5); // Minimum 5 register writes
}

/**
 * @brief Configure system settings successfully.
 */
void test_bq25672_configure_system_success(void)
{
    bq25672_system_config_t config = {
        .disable_pfm = true,
        .disable_ldo = false,
        .disable_ooa = true,
        .disable_stat = false,
        .enable_floating_mode = true,
        .switching_freq = BQ25672_SWITCHING_FREQ_750KHZ
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_system(&dev, &config));
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Configure USB (OTG) settings successfully.
 */
void test_bq25672_configure_usb_success(void)
{
    bq25672_usb_config_t config = {
        .enable_otg = true,
        .voltage_mv = 5200U,
        .current_max_ma = 1200U,
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_usb(&dev, &config));
    TEST_ASSERT_TRUE(mock_i2c_write_fake.call_count >= 2); // enable OTG, voltage, current
}

/**
 * @brief Configure watchdog timer settings successfully.
 */
void test_bq25672_configure_watchdog_success(void)
{
    bq25672_watchdog_config_t config = {
        .time = BQ25672_WATCHDOG_TIME_80S
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_watchdog(&dev, &config));
    TEST_ASSERT_EQUAL(1, mock_i2c_read_fake.call_count);
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Configure timer settings successfully.
 */
void test_bq25672_configure_timers_success(void)
{
    bq25672_timer_config_t config = {
        .top_off_timer_enable = true,
        .trickle_timer_enable = false,
        .precharge_timer_short = true,
        .precharge_timer_enable = true,
        .fast_charge_timer_enable = true
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_timers(&dev, &config));
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Configure MPPT settings successfully.
 */
void test_bq25672_configure_mppt_success(void)
{
    bq25672_mppt_config_t config = {
        .voc_ratio = BQ25672_MPPT_VOC_76PERCENT
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_mppt(&dev, &config));
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/**
 * @brief Configure NTC settings successfully.
 */
void test_bq25672_configure_ntc_success(void)
{
    bq25672_ntc_config_t config = {
        .cold_threshold = 0x3CU,
        .hot_threshold = 0x50U
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_ntc(&dev, &config));
    TEST_ASSERT_EQUAL(2, mock_i2c_write_fake.call_count);
}

/**
 * @brief Configure JEITA settings successfully.
 */
void test_bq25672_configure_jeita_success(void)
{
    bq25672_jeita_config_t config = {
        .high_temp_voltage_setting = BQ25672_JEITA_VOLTAGE_REDUCED,
        .high_temp_current_setting = BQ25672_JEITA_CURRENT_HALF,
        .cold_temp_current_setting = BQ25672_JEITA_CURRENT_HALF,
        .jeita_enabled = true
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_jeita(&dev, &config));
    TEST_ASSERT_EQUAL(1, mock_i2c_write_fake.call_count);
}

/* -------------------------------------------------------------------------- */
/*                        Subsystem Configuration (Readback) Tests            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read charging configuration successfully.
 */
void test_bq25672_read_charging_config_success(void)
{
    bq25672_charge_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_charging_config(&dev, &config));
    TEST_ASSERT_TRUE(config.charge_voltage_max_mv >= 3000U);
    TEST_ASSERT_TRUE(config.charge_current_max_ma >= 50U);
}

/**
 * @brief Read system configuration successfully.
 */
void test_bq25672_read_system_config_success(void)
{
    bq25672_system_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_system_config(&dev, &config));
}

/**
 * @brief Read USB (OTG) configuration successfully.
 */
void test_bq25672_read_usb_config_success(void)
{
    bq25672_usb_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_usb_config(&dev, &config));
}

/**
 * @brief Read watchdog timer configuration successfully.
 */
void test_bq25672_read_watchdog_config_success(void)
{
    bq25672_watchdog_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_watchdog_config(&dev, &config));
}

/**
 * @brief Read timer configuration successfully.
 */
void test_bq25672_read_timer_config_success(void)
{
    bq25672_timer_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_timer_config(&dev, &config));
}

/**
 * @brief Read MPPT configuration successfully.
 */
void test_bq25672_read_mppt_config_success(void)
{
    bq25672_mppt_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_mppt_config(&dev, &config));
}

/**
 * @brief Read NTC thresholds successfully.
 */
void test_bq25672_read_ntc_config_success(void)
{
    bq25672_ntc_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_ntc_config(&dev, &config));
}

/**
 * @brief Read JEITA configuration successfully.
 */
void test_bq25672_read_jeita_config_success(void)
{
    bq25672_jeita_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_jeita_config(&dev, &config));
}

/**
 * @brief Read entire configuration successfully.
 */
void test_bq25672_read_full_config_success(void)
{
    bq25672_config_t config;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_config(&dev, &config));
}
