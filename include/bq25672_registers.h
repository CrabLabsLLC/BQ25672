/**
 * @file bq25672_register.h
 * @brief Register map and bit mask definitions for BQ25672 battery charger.
 */

#pragma once

#ifndef BQ25672_REGISTERS_H
#define BQ25672_REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*                             Register Address Map                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Register addresses for BQ25672 device.
 */
typedef enum
{
    BQ25672_REG_MINIMAL_SYSTEM_VOLTAGE     = 0x00U,
    BQ25672_REG_CHARGE_VOLTAGE_LIMIT       = 0x01U,
    BQ25672_REG_CHARGE_CURRENT_LIMIT       = 0x03U,
    BQ25672_REG_INPUT_VOLTAGE_LIMIT        = 0x05U,
    BQ25672_REG_INPUT_CURRENT_LIMIT        = 0x06U,
    BQ25672_REG_PRECHARGE_CONTROL          = 0x08U,
    BQ25672_REG_TERMINATION_CONTROL        = 0x09U,
    BQ25672_REG_RECHARGE_CONTROL           = 0x0AU,
    BQ25672_REG_VOTG_REGULATION            = 0x0BU,
    BQ25672_REG_IOTG_REGULATION            = 0x0DU,
    BQ25672_REG_TIMER_CONTROL              = 0x0EU,
    BQ25672_REG_CHARGER_CONTROL_0          = 0x0FU,
    BQ25672_REG_CHARGER_CONTROL_1          = 0x10U,
    BQ25672_REG_CHARGER_CONTROL_2          = 0x11U,
    BQ25672_REG_CHARGER_CONTROL_3          = 0x12U,
    BQ25672_REG_CHARGER_CONTROL_4          = 0x13U,
    BQ25672_REG_CHARGER_CONTROL_5          = 0x14U,
    BQ25672_REG_MPPT_CONTROL               = 0x15U,
    BQ25672_REG_TEMPERATURE_CONTROL        = 0x16U,
    BQ25672_REG_NTC_CONTROL_0              = 0x17U,
    BQ25672_REG_NTC_CONTROL_1              = 0x18U,
    BQ25672_REG_ICO_CURRENT_LIMIT          = 0x19U,
    BQ25672_REG_CHARGER_STATUS_0           = 0x1BU,
    BQ25672_REG_CHARGER_STATUS_1           = 0x1CU,
    BQ25672_REG_CHARGER_STATUS_2           = 0x1DU,
    BQ25672_REG_CHARGER_STATUS_3           = 0x1EU,
    BQ25672_REG_CHARGER_STATUS_4           = 0x1FU,
    BQ25672_REG_FAULT_STATUS_0             = 0x20U,
    BQ25672_REG_FAULT_STATUS_1             = 0x21U,
    BQ25672_REG_CHARGER_FLAG_0             = 0x22U,
    BQ25672_REG_CHARGER_FLAG_1             = 0x23U,
    BQ25672_REG_CHARGER_FLAG_2             = 0x24U,
    BQ25672_REG_CHARGER_FLAG_3             = 0x25U,
    BQ25672_REG_FAULT_FLAG_0               = 0x26U,
    BQ25672_REG_FAULT_FLAG_1               = 0x27U,
    BQ25672_REG_CHARGER_MASK_0             = 0x28U,
    BQ25672_REG_CHARGER_MASK_1             = 0x29U,
    BQ25672_REG_CHARGER_MASK_2             = 0x2AU,
    BQ25672_REG_CHARGER_MASK_3             = 0x2BU,
    BQ25672_REG_FAULT_MASK_0               = 0x2CU,
    BQ25672_REG_FAULT_MASK_1               = 0x2DU,
    BQ25672_REG_ADC_CONTROL                = 0x2EU,
    BQ25672_REG_ADC_FUNCTION_DISABLE_0     = 0x2FU,
    BQ25672_REG_ADC_FUNCTION_DISABLE_1     = 0x30U,
    BQ25672_REG_IBUS_ADC                   = 0x31U,
    BQ25672_REG_IBAT_ADC                   = 0x33U,
    BQ25672_REG_VBUS_ADC                   = 0x35U,
    BQ25672_REG_VAC1_ADC                   = 0x37U,
    BQ25672_REG_VAC2_ADC                   = 0x39U,
    BQ25672_REG_VBAT_ADC                   = 0x3BU,
    BQ25672_REG_VSYS_ADC                   = 0x3DU,
    BQ25672_REG_TS_ADC                     = 0x3FU,
    BQ25672_REG_TDIE_ADC                   = 0x41U,
    BQ25672_REG_DP_ADC                     = 0x43U,
    BQ25672_REG_DM_ADC                     = 0x45U,
    BQ25672_REG_DPDM_DRIVER                = 0x47U,
    BQ25672_REG_PART_INFORMATION           = 0x48U,
} bq25672_reg_t;
 
/* -------------------------------------------------------------------------- */
/*                     CHARGER_CONTROL_0 Register Bit Fields                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Charger Control 0 Register (0x0F) Bit Masks.
 */
typedef enum
{
    BQ25672_DISABLE_PFM_MASK      = 0x80U, /**< Disable PFM mode */
    BQ25672_DISABLE_LDO_MASK      = 0x40U, /**< Disable LDO regulator */
    BQ25672_DISABLE_OOA_MASK      = 0x20U, /**< Disable OOA mode */
    BQ25672_DISABLE_STAT_MASK     = 0x10U, /**< Disable STAT pin output */
    BQ25672_ENABLE_OTG_MASK       = 0x08U, /**< Enable OTG boost mode */
    BQ25672_SWITCH_FREQ_MASK      = 0x07U, /**< Switching frequency selection bits */
} bq25672_charger_control_0_mask_t;

/* -------------------------------------------------------------------------- */
/*                     CHARGER_CONTROL_1 Register Bit Fields                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Charger Control 1 Register (0x10) Bit Masks.
 */
typedef enum
{
    BQ25672_INPUT_VOLTAGE_DPM_MASK = 0xF0U, /**< Input voltage DPM threshold */
    BQ25672_FORCE_DPM_DET_MASK     = 0x08U, /**< Force DPM detection */
    BQ25672_HVDCP_MASK             = 0x07U, /**< HVDCP setting bits */
} bq25672_charger_control_1_mask_t;
 
/**
 * @brief Charger Control 2 Register (0x11) Bit Masks.
 */
typedef enum
{
    BQ25672_DISABLE_FORCE_VINDPM_MASK = 0x80U, /**< Disable force VINDPM loop */
    BQ25672_ENABLE_VINDPM_MASK         = 0x40U, /**< Enable VINDPM loop */
    BQ25672_DISABLE_FORCE_IINDPM_MASK  = 0x20U, /**< Disable force IINDPM loop */
    BQ25672_ENABLE_IINDPM_MASK         = 0x10U, /**< Enable IINDPM loop */
    BQ25672_WATCHDOG_TIMER_MASK        = 0x70U, /**< Watchdog timer setting */
} bq25672_charger_control_2_mask_t;

/**
 * @brief Charger Control 3 Register (0x12) Bit Masks.
 */
typedef enum
{
    BQ25672_DISABLE_VSYS_MIN_MASK  = 0x80U, /**< Disable VSYS minimum regulation */
    BQ25672_VSYS_REGULATION_MASK   = 0x7FU, /**< Minimum system voltage setting */
} bq25672_charger_control_3_mask_t;

/**
 * @brief Charger Control 4 Register (0x13) Bit Masks.
 */
typedef enum
{
    BQ25672_DISABLE_CHARGER_MASK   = 0x80U, /**< Disable entire charger function */
    BQ25672_ENABLE_HIZ_MASK        = 0x40U, /**< Enable Hi-Z mode */
    BQ25672_BATFET_DIS_MASK        = 0x20U, /**< BATFET disable control */
} bq25672_charger_control_4_mask_t;

/**
 * @brief Charger Control 5 Register (0x14) Bit Masks.
 */
typedef enum
{
    BQ25672_RESET_MASK         = 0x80U, /**< Software reset trigger */
    BQ25672_FORCE_ICO_MASK      = 0x40U, /**< Force ICO (Input Current Optimizer) */
} bq25672_charger_control_5_mask_t;

/**
 * @brief Timer Control Register (0x0E) Bit Masks.
 */
typedef enum
{
    BQ25672_TOP_OFF_TIMER_ENABLE_MASK     = 0x10U, /**< Enable top-off timer */
    BQ25672_TRICKLE_TIMER_ENABLE_MASK     = 0x08U, /**< Enable trickle charger timer */
    BQ25672_PRECHARGE_TIMER_SHORT_MASK    = 0x04U, /**< Enable short precharge timer */
    BQ25672_PRECHARGE_TIMER_ENABLE_MASK   = 0x02U, /**< Enable precharge timer */
    BQ25672_FAST_CHARGE_TIMER_ENABLE_MASK = 0x01U, /**< Enable fast charge timer */
} bq25672_timer_control_mask_t;

/**
 * @brief MPPT Control Register (0x15) Bit Masks.
 */
typedef enum
{
    BQ25672_MPPT_VOC_RATIO_MASK = 0x07U, /**< MPPT open circuit voltage ratio setting */
} bq25672_mppt_control_mask_t;

/**
 * @brief NTC Control 0 Register (0x17) Bit Masks.
 */
typedef enum
{
    BQ25672_NTC_COLD_THRESHOLD_MASK = 0xFFU, /**< Raw bits for cold NTC threshold */
} bq25672_ntc_control_0_mask_t;

/**
 * @brief NTC Control 1 Register (0x18) Bit Masks.
 */
typedef enum
{
    BQ25672_NTC_HOT_THRESHOLD_MASK = 0xFFU, /**< Raw bits for hot NTC threshold */
} bq25672_ntc_control_1_mask_t;

/**
 * @brief Temperature Control (JEITA) Register (0x16) Bit Masks.
 */
typedef enum
{
    BQ25672_HIGH_TEMP_VOLTAGE_SETTING_MASK = 0xE0U, /**< High temperature voltage setting (bits 7:5) */
    BQ25672_HIGH_TEMP_CURRENT_SETTING_MASK = 0x18U, /**< High temperature current setting (bits 4:3) */
    BQ25672_COLD_TEMP_CURRENT_SETTING_MASK = 0x06U, /**< Cold temperature current setting (bits 2:1) */
    BQ25672_JEITA_ENABLE_MASK              = 0x01U, /**< JEITA enable bit */
} bq25672_temperature_control_mask_t;


/**
 * @brief Fault Status 0 Register (0x20) Bit Masks.
 */
typedef enum
{
    BQ25672_INPUT_OVP_FAULT_MASK      = 0x80U, /**< Input overvoltage protection fault */
    BQ25672_INPUT_UVP_FAULT_MASK      = 0x40U, /**< Input undervoltage protection fault */
    BQ25672_THERMAL_SHUTDOWN_MASK     = 0x20U, /**< Thermal shutdown fault */
    BQ25672_TS_COLD_FAULT_MASK        = 0x10U, /**< Battery temperature cold fault */
    BQ25672_TS_HOT_FAULT_MASK         = 0x08U, /**< Battery temperature hot fault */
    BQ25672_BATTERY_OVP_FAULT_MASK    = 0x04U, /**< Battery overvoltage fault */
    BQ25672_WATCHDOG_TIMER_FAULT_MASK = 0x02U, /**< Watchdog expiration fault */
} bq25672_fault_status_0_mask_t;

/**
 * @brief Fault Status 1 Register (0x21) Bit Masks.
 */
typedef enum
{
    BQ25672_INPUT_SHORT_FAULT_MASK = 0x01U, /**< Input short fault */
} bq25672_fault_status_1_mask_t;

/**
 * @brief Fault Status 1 Register (0x21) Bit Masks.
 */
typedef enum
{
    BQ25672_INPUT_SHORT_FAULT_MASK = 0x01U, /**< Input short fault */
} bq25672_fault_status_1_mask_t;

/**
 * @brief Charger Status 0 Register (0x1B) Bit Masks.
 */
typedef enum
{
    BQ25672_VBUS_PRESENT_MASK       = 0x80U, /**< VBUS voltage present */
    BQ25672_CHARGER_ACTIVE_MASK     = 0x40U, /**< Charger active */
    BQ25672_OTG_MODE_MASK           = 0x20U, /**< OTG mode active */
} bq25672_charger_status_0_mask_t;

/**
 * @brief ADC Control Register (0x2E) Bit Masks.
 */
typedef enum
{
    BQ25672_ENABLE_ADC_MASK         = 0x80U, /**< Enable ADC operation */
    BQ25672_START_ADC_CONVERSION_MASK = 0x40U, /**< Start ADC single-shot */
    BQ25672_ADC_SAMPLE_RATE_MASK    = 0x03U, /**< ADC sample rate control */
} bq25672_adc_control_mask_t;

/**
 * @brief DPDM Driver Register (0x47) Bit Masks.
 */
typedef enum
{
    BQ25672_DPDM_DRV_ENABLE_MASK    = 0x01U, /**< Enable DPDM line driver */
} bq25672_dpdm_driver_mask_t;



#ifdef __cplusplus
}
#endif
 