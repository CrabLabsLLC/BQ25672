/**
 * @file test_bq25672_system.c
 * @brief System tests for BQ25672 driver (simulate full behaviors)
 */

#include "unity.h"
#include "fff.h"
#include "bq25672.h"
#include "bq25672_register.h"
#include <string.h>

DEFINE_FFF_GLOBALS;

/* -------------------------------------------------------------------------- */
/*                         Fake Device Memory                                 */
/* -------------------------------------------------------------------------- */

static uint8_t fake_device_memory[256];

/* -------------------------------------------------------------------------- */
/*                      Fake I2C Layer (simulate registers)                   */
/* -------------------------------------------------------------------------- */

bool mock_i2c_write_fake(uint8_t addr, uint8_t reg, const void* const data, uint8_t len)
{
    (void)addr;
    if ((reg + len) > sizeof(fake_device_memory)) return false;
    memcpy(&fake_device_memory[reg], data, len);
    return true;
}

bool mock_i2c_read_fake(uint8_t addr, uint8_t reg, void* const data, uint8_t len)
{
    (void)addr;
    if ((reg + len) > sizeof(fake_device_memory)) return false;
    memcpy(data, &fake_device_memory[reg], len);
    return true;
}

/* -------------------------------------------------------------------------- */
/*                          Test Context Setup                                */
/* -------------------------------------------------------------------------- */

static bq25672_t dev;

void setUp(void)
{
    FFF_RESET_HISTORY();
    memset(fake_device_memory, 0, sizeof(fake_device_memory));
    memset(&dev, 0, sizeof(dev));
    dev.hal.i2c_read = mock_i2c_read_fake;
    dev.hal.i2c_write = mock_i2c_write_fake;
}

void tearDown(void)
{
    // No action needed
}

/* -------------------------------------------------------------------------- */
/*                            System Tests                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Full device initialization and configuration scenario.
 */
void test_system_powerup_full_configuration(void)
{
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_init(&dev));

    bq25672_config_t full_config = {0};

    full_config.charge_config.charge_voltage_max_mv = 4200U;
    full_config.charge_config.charge_current_max_ma = 1500U;
    full_config.charge_config.input_voltage_max_mv = 5000U;
    full_config.charge_config.input_current_max_ma = 2000U;
    full_config.charge_config.precharge_voltage = BQ25672_PRECHARGE_VOLTAGE_62_2_VREG;
    full_config.charge_config.termination_current_ma = 150U;

    full_config.usb_config.enable_otg = false;
    full_config.usb_config.voltage_mv = 5200U;
    full_config.usb_config.current_max_ma = 1200U;

    full_config.watchdog_config.time = BQ25672_WATCHDOG_TIME_80S;

    full_config.system_config.disable_pfm = false;
    full_config.system_config.switching_freq = BQ25672_SWITCHING_FREQ_750KHZ;

    full_config.timer_config.top_off_timer_enable = true;

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure(&dev, &full_config));
}

/**
 * @brief Simulate normal battery charging operation.
 */
void test_system_normal_charging_cycle(void)
{
    fake_device_memory[BQ25672_REG_CHARGER_STATUS_0] = BQ25672_VBUS_PRESENT_MASK | BQ25672_CHARGER_ACTIVE_MASK;

    uint8_t status = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_status(&dev, &status));
    TEST_ASSERT_TRUE(status & BQ25672_CHARGER_ACTIVE_MASK);
}

/**
 * @brief Simulate fault injection and recovery.
 */
void test_system_fault_injection_and_recovery(void)
{
    fake_device_memory[BQ25672_REG_FAULT_FLAG_0] = BQ25672_INPUT_OVP_FAULT_MASK;

    uint8_t fault0 = 0, fault1 = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_fault_status(&dev, &fault0, &fault1));
    TEST_ASSERT_TRUE(fault0 & BQ25672_INPUT_OVP_FAULT_MASK);

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_clear_flags(&dev));
}

/**
 * @brief Simulate USB OTG boost mode activation.
 */
void test_system_usb_otg_output(void)
{
    bq25672_usb_config_t config = {
        .enable_otg = true,
        .voltage_mv = 5200U,
        .current_max_ma = 1200U
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_usb(&dev, &config));

    uint8_t ctrl0 = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_register(&dev, BQ25672_REG_CHARGER_CONTROL_0, &ctrl0));
    TEST_ASSERT_TRUE(ctrl0 & BQ25672_ENABLE_OTG_MASK);
}

/**
 * @brief Simulate JEITA temperature region handling.
 */
void test_system_jeita_temperature_adjustments(void)
{
    bq25672_jeita_config_t jeita_config = {
        .high_temp_voltage_setting = BQ25672_JEITA_VOLTAGE_REDUCED,
        .high_temp_current_setting = BQ25672_JEITA_CURRENT_HALF,
        .cold_temp_current_setting = BQ25672_JEITA_CURRENT_HALF,
        .jeita_enabled = true
    };

    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_jeita(&dev, &jeita_config));

    uint8_t reg_val;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_register(&dev, BQ25672_REG_TEMPERATURE_CONTROL, &reg_val));
    TEST_ASSERT_TRUE(reg_val & BQ25672_JEITA_ENABLE_MASK);
}

/**
 * @brief Simulate watchdog expiration and reset.
 */
void test_system_watchdog_expiration(void)
{
    fake_device_memory[BQ25672_REG_FAULT_FLAG_0] = 0x04U;

    uint8_t fault0 = 0, fault1 = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_fault_status(&dev, &fault0, &fault1));
    TEST_ASSERT_TRUE(fault0 & 0x04U);
}

/**
 * @brief Simulate safety timer expiration event.
 */
void test_system_safety_timer_expired(void)
{
    fake_device_memory[BQ25672_REG_FAULT_FLAG_1] = 0x04U;

    uint8_t fault0 = 0, fault1 = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_fault_status(&dev, &fault0, &fault1));
    TEST_ASSERT_TRUE(fault1 & 0x04U);
}

/**
 * @brief Simulate NTC hot/cold shutdown.
 */
void test_system_ntc_hot_cold_shutdown(void)
{
    fake_device_memory[BQ25672_REG_FAULT_FLAG_0] = 0x30U;

    uint8_t fault0 = 0, fault1 = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_fault_status(&dev, &fault0, &fault1));
    TEST_ASSERT_TRUE(fault0 & 0x10U);
    TEST_ASSERT_TRUE(fault0 & 0x20U);
}

/**
 * @brief Simulate VBUS disconnection event.
 */
void test_system_vbus_disconnected(void)
{
    fake_device_memory[BQ25672_REG_CHARGER_STATUS_0] = 0x00U;

    uint8_t status = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_status(&dev, &status));
    TEST_ASSERT_FALSE(status & BQ25672_VBUS_PRESENT_MASK);
}

/**
 * @brief Simulate ADC VBAT reading full scale edge case.
 */
void test_system_adc_vbat_full_scale(void)
{
    fake_device_memory[BQ25672_REG_VBAT_ADC] = 0xFFU;
    fake_device_memory[BQ25672_REG_VBAT_ADC + 1] = 0x03U;

    uint16_t vbat = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_adc(&dev, BQ25672_REG_VBAT_ADC, &vbat));
    TEST_ASSERT_GREATER_THAN_UINT16(0U, vbat);
}

/**
 * @brief Simulate DPDM detection active.
 */
void test_system_dpdm_detection_active(void)
{
    fake_device_memory[BQ25672_REG_DP_ADC] = 0xC0U;
    fake_device_memory[BQ25672_REG_DM_ADC] = 0x80U;

    uint16_t dp = 0, dm = 0;
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_adc(&dev, BQ25672_REG_DP_ADC, &dp));
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_read_adc(&dev, BQ25672_REG_DM_ADC, &dm));
    TEST_ASSERT_TRUE(dp > 0U);
    TEST_ASSERT_TRUE(dm > 0U);
}

/**
 * @brief Simulate optimize input current mode being active.
 */
void test_system_optimize_input_current(void)
{
    bq25672_system_config_t sys_cfg = {
        .optimize_input_current = true
    };
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_system(&dev, &sys_cfg));
}

/**
 * @brief Simulate floating battery operation.
 */
void test_system_floating_mode_operation(void)
{
    bq25672_system_config_t sys_cfg = {
        .enable_floating_mode = true
    };
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_system(&dev, &sys_cfg));
}

/**
 * @brief Simulate MPPT VOC adjustment.
 */
void test_system_mppt_voc_change(void)
{
    bq25672_mppt_config_t mppt = {
        .voc_ratio = BQ25672_MPPT_VOC_80PERCENT
    };
    TEST_ASSERT_EQUAL(BQ25672_OK, bq25672_configure_mppt(&dev, &mppt));
}
 