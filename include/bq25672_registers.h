/**
 * @file bq25672_registers.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef BQ25672_REGISTERS_H
#define BQ25672_REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif

/** @name Register Addresses */
/** @{ */
#define BQ25672_REG_MINIMAL_SYSTEM_VOLTAGE     0x00U ///< System minimum voltage register (VSYSMIN)
#define BQ25672_REG_CHARGE_VOLTAGE_LIMIT       0x01U ///< Charge voltage limit register (VREG)
#define BQ25672_REG_CHARGE_CURRENT_LIMIT       0x03U ///< Charge current limit register (ICHG)
#define BQ25672_REG_INPUT_VOLTAGE_LIMIT        0x05U ///< Input voltage limit register (VINDPM)
#define BQ25672_REG_INPUT_CURRENT_LIMIT        0x06U ///< Input current limit register (IINDPM)
#define BQ25672_REG_PRECHARGE_CONTROL          0x08U ///< Precharge control register (IPRECHG)
#define BQ25672_REG_TERMINATION_CONTROL        0x09U ///< Termination control register (ITERM)
#define BQ25672_REG_RECHARGE_CONTROL           0x0AU ///< Recharge threshold register (VRECHG)
#define BQ25672_REG_VOTG_REGULATION            0x0BU ///< OTG voltage regulation register (VOTG)
#define BQ25672_REG_IOTG_REGULATION            0x0DU ///< OTG current regulation register (IOTG)
#define BQ25672_REG_TIMER_CONTROL              0x0EU ///< Charger safety timer control (TIMER)
#define BQ25672_REG_CHARGER_CONTROL_0          0x0FU ///< Charger control register 0
#define BQ25672_REG_CHARGER_CONTROL_1          0x10U ///< Charger control register 1
#define BQ25672_REG_CHARGER_CONTROL_2          0x11U ///< Charger control register 2
#define BQ25672_REG_CHARGER_CONTROL_3          0x12U ///< Charger control register 3
#define BQ25672_REG_CHARGER_CONTROL_4          0x13U ///< Charger control register 4
#define BQ25672_REG_CHARGER_CONTROL_5          0x14U ///< Charger control register 5
#define BQ25672_REG_MPPT_CONTROL               0x15U ///< MPPT control register
#define BQ25672_REG_TEMPERATURE_CONTROL        0x16U ///< JEITA temperature control register
#define BQ25672_REG_NTC_CONTROL_0              0x17U ///< NTC cold threshold register
#define BQ25672_REG_NTC_CONTROL_1              0x18U ///< NTC hot threshold register
#define BQ25672_REG_ICO_CURRENT_LIMIT          0x19U ///< ICO current limit register
#define BQ25672_REG_CHARGER_STATUS_0           0x1BU ///< Charger status register 0
#define BQ25672_REG_CHARGER_STATUS_1           0x1CU ///< Charger status register 1
#define BQ25672_REG_CHARGER_STATUS_2           0x1DU ///< Charger status register 2
#define BQ25672_REG_CHARGER_STATUS_3           0x1EU ///< Charger status register 3
#define BQ25672_REG_CHARGER_STATUS_4           0x1FU ///< Charger status register 4
#define BQ25672_REG_FAULT_STATUS_0             0x20U ///< Fault status register 0
#define BQ25672_REG_FAULT_STATUS_1             0x21U ///< Fault status register 1
#define BQ25672_REG_CHARGER_FLAG_0             0x22U ///< Charger flag register 0
#define BQ25672_REG_CHARGER_FLAG_1             0x23U ///< Charger flag register 1
#define BQ25672_REG_CHARGER_FLAG_2             0x24U ///< Charger flag register 2
#define BQ25672_REG_CHARGER_FLAG_3             0x25U ///< Charger flag register 3
#define BQ25672_REG_FAULT_FLAG_0               0x26U ///< Fault flag register 0
#define BQ25672_REG_FAULT_FLAG_1               0x27U ///< Fault flag register 1
#define BQ25672_REG_CHARGER_MASK_0             0x28U ///< Charger interrupt mask register 0
#define BQ25672_REG_CHARGER_MASK_1             0x29U ///< Charger interrupt mask register 1
#define BQ25672_REG_CHARGER_MASK_2             0x2AU ///< Charger interrupt mask register 2
#define BQ25672_REG_CHARGER_MASK_3             0x2BU ///< Charger interrupt mask register 3
#define BQ25672_REG_FAULT_MASK_0               0x2CU ///< Fault interrupt mask register 0
#define BQ25672_REG_FAULT_MASK_1               0x2DU ///< Fault interrupt mask register 1
#define BQ25672_REG_ADC_CONTROL                0x2EU ///< ADC control register
#define BQ25672_REG_ADC_FUNCTION_DISABLE_0     0x2FU ///< ADC function disable register 0
#define BQ25672_REG_ADC_FUNCTION_DISABLE_1     0x30U ///< ADC function disable register 1
#define BQ25672_REG_IBUS_ADC                   0x31U ///< ADC VBUS current reading register
#define BQ25672_REG_IBAT_ADC                   0x33U ///< ADC BAT current reading register
#define BQ25672_REG_VBUS_ADC                   0x35U ///< ADC VBUS voltage reading register
#define BQ25672_REG_VAC1_ADC                   0x37U ///< ADC VAC1 voltage reading register
#define BQ25672_REG_VAC2_ADC                   0x39U ///< ADC VAC2 voltage reading register
#define BQ25672_REG_VBAT_ADC                   0x3BU ///< ADC BAT voltage reading register
#define BQ25672_REG_VSYS_ADC                   0x3DU ///< ADC SYS voltage reading register
#define BQ25672_REG_TS_ADC                     0x3FU ///< ADC TS voltage reading register
#define BQ25672_REG_TDIE_ADC                   0x41U ///< ADC die temperature reading register
#define BQ25672_REG_DP_ADC                     0x43U ///< ADC D+ voltage reading register
#define BQ25672_REG_DM_ADC                     0x45U ///< ADC D- voltage reading register
#define BQ25672_REG_DPDM_DRIVER                0x47U ///< DPDM line driver control register
#define BQ25672_REG_PART_INFORMATION           0x48U ///< Part information register (PN/DEVREV)
/** @} */
/*==============================================================================
 * Bit shifts & masks
 *============================================================================*/

/** @name REG00_Minimum_System_Voltage (0x01) */
///@{
#define BQ25672_VSYSMIN_SHIFT    0U ///< VREG[6:0] start bit
#define BQ25672_VSYSMIN_MASK     (0x3FU << BQ25672_VSYSMIN_SHIFT) ///< 7-bit charge voltage mask
///@}

/** @name REG01_Charge_Voltage_Limit (0x01) */
///@{
#define BQ25672_VREG_SHIFT    0U ///< VREG[6:0] start bit
#define BQ25672_VREG_MASK     (0x7FFU << BQ25672_VREG_SHIFT) ///< 7-bit charge voltage mask
///@}

/** @name REG03_Charge_Current_Limit (0x03) */
///@{
#define BQ25672_ICHG_SHIFT    0U ///< ICHG[5:0] start bit 
#define BQ25672_ICHG_MASK     (0x1FFU << BQ25672_ICHG_SHIFT) ///< 6-bit charge current mask
///@}

/** @name REG05_Input_Voltage_Limit (0x05) */
///@{
#define BQ25672_VINDPM_SHIFT  0U ///< VINDPM[7:0] start bit :contentReference[oaicite:4]{index=4}&#8203;:contentReference[oaicite:5]{index=5}
#define BQ25672_VINDPM_MASK   (0xFFU << BQ25672_VINDPM_SHIFT) ///< 8-bit input voltage mask
///@}

/** @name REG06_Input_Current_Limit (0x06) */
///@{
#define BQ25672_IINDPM_SHIFT  0U ///< IINDPM[8:0] start bit :contentReference[oaicite:6]{index=6}&#8203;:contentReference[oaicite:7]{index=7}
#define BQ25672_IINDPM_MASK   (0x1FFU << BQ25672_IINDPM_SHIFT) ///< 8-bit input current mask
///@}

/** @name REG08_Precharge_Control (0x08) */
///@{
#define BQ25672_VBAT_LOWV_SHIFT   6U ///< VBAT_LOWV[1:0] start bit :contentReference[oaicite:8]{index=8}&#8203;:contentReference[oaicite:9]{index=9}
#define BQ25672_VBAT_LOWV_MASK    (0x03U << BQ25672_VBAT_LOWV_SHIFT) ///< 2-bit precharge threshold
#define BQ25672_IPRECHG_SHIFT     0U ///< IPRECHG[5:0] start bit :contentReference[oaicite:10]{index=10}&#8203;:contentReference[oaicite:11]{index=11}
#define BQ25672_IPRECHG_MASK      (0x3FU << BQ25672_IPRECHG_SHIFT) ///< 6-bit precharge current mask
///@}

/** @name REG09_Termination_Control (0x09) */
///@{
#define BQ25672_ITERM_SHIFT         0U ///< ITERM[4:0] start bit :contentReference[oaicite:12]{index=12}&#8203;:contentReference[oaicite:13]{index=13}
#define BQ25672_ITERM_MASK          (0x1FU << BQ25672_ITERM_SHIFT) ///< 5-bit termination current mask
#define BQ25672_STOP_WD_CHG_MASK    (1U << 5) ///< Stop on watchdog expiry mask
#define BQ25672_REG_RST_MASK        (1U << 6) ///< Register reset bit

/** @name REG0A_Re-charge_Control (0x0A) */
///@{
#define BQ25672_CELL_SHIFT     6U ///< CELL[1:0] start bit 
#define BQ25672_CELL_MASK      (0x03U << BQ25672_CELL_SHIFT) ///< 2-bit cell count mask
#define BQ25672_TRECHG_SHIFT   4U ///< TRECHG[1:0] start bit 
#define BQ25672_TRECHG_MASK    (0x03U << BQ25672_TRECHG_SHIFT) ///< 2-bit recharge deglitch mask
#define BQ25672_VRECHG_SHIFT   0U ///< VRECHG[3:0] start bit :contentReference[oaicite:18]{index=18}&#8203;:contentReference[oaicite:19]{index=19}
#define BQ25672_VRECHG_MASK    (0x0FU << BQ25672_VRECHG_SHIFT) ///< 4-bit recharge threshold mask
///@}

/** @name REG0B_VOTG_regulation (0x0B) */
///@{
#define BQ25672_VOTG_SHIFT     0U ///< VOTG[10:0] start bit :contentReference[oaicite:20]{index=20}&#8203;:contentReference[oaicite:21]{index=21}
#define BQ25672_VOTG_MASK      (0x7FU << BQ25672_VOTG_SHIFT) ///< 7-bit OTG voltage mask (LSB)
///@}

/** @name REG0D_IOTG_regulation (0x0D) */
///@{
#define BQ25672_PRECHRG_TMR_MASK    (0x80)
#define BQ25672_IOTG_SHIFT          0U ///< IOTG[6:0] start bit :contentReference[oaicite:22]{index=22}&#8203;:contentReference[oaicite:23]{index=23}
#define BQ25672_IOTG_MASK           (0x7FU << BQ25672_IOTG_SHIFT) ///< 7-bit OTG current mask
///@}

/** @name REG0E_Timer_Control (0x0E) */
///@{
#define BQ25672_TOP_OFF_TIMER_SHIFT             (6)
#define BQ25672_TOP_OFF_TIMER_MASK              (0x3 << BQ25672_TOP_OFF_TIMER_SHIFT) ///< Top-off timer enable
#define BQ25672_CHARGE_TIMER_ENABLE_MASK        (0x08)
#define BQ25672_TRICKLE_TIMER_ENABLE_MASK       (1U << 5) ///< Trickle timer enable
#define BQ25672_PRECHARGE_TIMER_ENABLE_MASK     (1U << 4) ///< Precharge timer enable
#define BQ25672_FAST_CHARGE_TIMER_SHIFT         (1)
#define BQ25672_FAST_CHARGE_TIMER_MASK          (0x3 << BQ25672_FAST_CHARGE_TIMER_SHIFT) ///< Fast-charge timer enable
#define BQ25672_SLOW_TIMERS_MASK                (0x1)
///@}

/** @name REG0F_Charger_Control_0 (0x0F) */
/** @{ */
#define BQ25672_EN_DISCHARGE_SHIFT      (0x7) ///< Bit position for full-chip reset
#define BQ25672_EN_DISCHARGE_MASK       (0x1 << BQ25672_EN_DISCHARGE_SHIFT) ///< Mask for full-chip reset
#define BQ25672_FORCE_DISCHARGE_SHIFT   (0x6)
#define BQ25672_FORCE_DISCHARGE_MASK    (0x1 << BQ25672_FORCE_DISCHARGE_SHIFT)
#define BQ25672_FORCE_ICO_SHIFT         (0x3) ///< Bit position for force ICO start
#define BQ25672_FORCE_ICO_MASK          (0x1 << BQ25672_FORCE_ICO_SHIFT) ///< Mask for force ICO start
#define BQ25672_EN_ICO_SHIFT            (0x4) ///< Bit position for termination enable
#define BQ25672_EN_ICO_MASK             (0x1 << BQ25672_EN_ICO_SHIFT) ///< Mask for termination enable
#define BQ25672_EN_TERM_SHIFT           (0x1) ///< Bit position for termination enable
#define BQ25672_EN_TERM_MASK            (0x1 << BQ25672_EN_TERM_SHIFT) ///< Mask for termination enable
#define BQ25672_EN_HIZ_SHIFT            (0x2) ///< Bit position for HIZ enable
#define BQ25672_EN_HIZ_MASK             (0x1 << BQ25672_EN_HIZ_SHIFT) ///< Mask for HIZ enable
/** @} */

/** @name REG0F_Charger_Control_0 (0x0F) */
/** @{ */
#define BQ25672_DISABLE_PFM_MASK   (1U << 7) ///< Disable PFM mode
#define BQ25672_DISABLE_LDO_MASK   (1U << 6) ///< Disable LDO regulator
#define BQ25672_DISABLE_OOA_MASK   (1U << 5) ///< Disable OOA mode
#define BQ25672_DISABLE_STAT_MASK  (1U << 4) ///< Disable STAT pin output
#define BQ25672_ENABLE_OTG_MASK    (1U << 3) ///< Enable OTG mode
#define BQ25672_SWITCH_FREQ_SHIFT  0U       ///< Switch frequency bits shift
#define BQ25672_SWITCH_FREQ_MASK   (0x07U << BQ25672_SWITCH_FREQ_SHIFT) ///< 3-bit switch freq mask
///@}

/** @name CHARGER_CONTROL_1 Register Bit Fields (0x10) */
/** @{ */
#define BQ25672_INPUT_VOLTAGE_DPM_SHIFT  (0x4) ///< Bit position for input voltage DPM
#define BQ25672_INPUT_VOLTAGE_DPM_MASK   (0xF << BQ25672_INPUT_VOLTAGE_DPM_SHIFT) ///< Mask for input voltage DPM
#define BQ25672_FORCE_DPM_DET_SHIFT      (0x3) ///< Bit position for force DPM detect
#define BQ25672_FORCE_DPM_DET_MASK       (0x1 << BQ25672_FORCE_DPM_DET_SHIFT) ///< Mask for force DPM detect
#define BQ25672_HVDCP_SHIFT              (0x0) ///< Bit position for HVDCP setting
#define BQ25672_HVDCP_MASK               (0x7 << BQ25672_HVDCP_SHIFT) ///< Mask for HVDCP setting
/** @} */

/** @name CHARGER_CONTROL_2 Register Bit Fields (0x11) */
/** @{ */
#define BQ25672_VAC_OVP_THRESH_SHIFT        (0x4) 
#define BQ25672_VAC_OVP_THRESH_MASK         (0x3 << BQ25672_VAC_OVP_THRESH_SHIFT) ///< Mask for setting OVP threshold
#define BQ25672_WDOG_RESET_SHIFT            (0x3) 
#define BQ25672_WDOG_RESET_MASK             (0x1 << BQ25672_WDOG_RESET_SHIFT) ///< Mask for enable VINDPM
#define BQ25672_WATCHDOG_TIMER_SHIFT        (0x0) ///< Bit position for watchdog timer setting
#define BQ25672_WATCHDOG_TIMER_MASK         (0x7 << BQ25672_WATCHDOG_TIMER_SHIFT) ///< Mask for watchdog timer setting
/** @} */

/** @name CHARGER_CONTROL_3 Register Bit Fields (0x12) */
/** @{ */
#define BQ25672_FORCE_USB_DET_SHIFT     (0x7) ///< Bit position for disable VSYS_MIN regulation
#define BQ25672_FORCE_USB_DET_MASK      (0x1 << BQ25672_FORCE_USB_DET_SHIFT)
#define BQ25672_AUTO_USB_DET_SHIFT      ()
#define BQ25672_DISABLE_VSYS_MIN_MASK   (0x1 << BQ25672_DISABLE_VSYS_MIN_SHIFT) ///< Mask for disable VSYS_MIN regulation
#define BQ25672_VSYS_REG_SHIFT          (0x0) ///< Bit position for VSYS_MIN setting
#define BQ25672_VSYS_REG_MASK           (0x7F << BQ25672_VSYS_REG_SHIFT) ///< Mask for VSYS_MIN setting
/** @} */

/** @name CHARGER_CONTROL_4 Register Bit Fields (0x13) */
/** @{ */
#define BQ25672_DISABLE_CHARGER_SHIFT   (0x7) ///< Bit position for disable charger
#define BQ25672_DISABLE_CHARGER_MASK    (0x1 << BQ25672_DISABLE_CHARGER_SHIFT) ///< Mask for disable charger
#define BQ25672_ENABLE_HIZ_SHIFT        (0x6) ///< Bit position for HIZ mode enable
#define BQ25672_ENABLE_HIZ_MASK         (0x1 << BQ25672_ENABLE_HIZ_SHIFT) ///< Mask for HIZ mode enable
#define BQ25672_BATFET_DIS_SHIFT        (0x5) ///< Bit position for BATFET disable
#define BQ25672_BATFET_DIS_MASK         (0x1 << BQ25672_BATFET_DIS_SHIFT) ///< Mask for BATFET disable
/** @} */

/** @name CHARGER_CONTROL_5 Register Bit Fields (0x14) */
/** @{ */
#define BQ25672_RESET_SHIFT         (0x7) ///< Bit position for full-chip reset
#define BQ25672_RESET_MASK          (0x1 << BQ25672_RESET_SHIFT) ///< Mask for full-chip reset
#define BQ25672_FORCE_ICO_SHIFT     (0x6) ///< Bit position for force ICO start
#define BQ25672_FORCE_ICO_MASK      (0x1 << BQ25672_FORCE_ICO_SHIFT) ///< Mask for force ICO start
#define BQ25672_EN_TERM_SHIFT       (0x1) ///< Bit position for termination enable
#define BQ25672_EN_TERM_MASK        (0x1 << BQ25672_EN_TERM_SHIFT) ///< Mask for termination enable
#define BQ25672_EN_HIZ_SHIFT        (0x2) ///< Bit position for HIZ enable
#define BQ25672_EN_HIZ_MASK         (0x1 << BQ25672_EN_HIZ_SHIFT) ///< Mask for HIZ enable
/** @} */

/** @name REG15_MPPT_Control (0x15) */
///@{
#define BQ25672_MPPT_VOC_RATIO_SHIFT 0U
#define BQ25672_MPPT_VOC_RATIO_MASK  (0x07U << BQ25672_MPPT_VOC_RATIO_SHIFT) ///< 3-bit VOC_PCT :contentReference[oaicite:4]{index=4}&#8203;:contentReference[oaicite:5]{index=5}
///@}

/** @name TEMPERATURE_CONTROL Register Bit Fields (0x16) */  
/** @{ */
#define BQ25672_TREG_SHIFT       (0x6) ///< Bit position for thermal regulation threshold
#define BQ25672_TREG_MASK        (0x3 << BQ25672_TREG_SHIFT) ///< Mask for thermal regulation threshold 
#define BQ25672_TSHUT_SHIFT      (0x4) ///< Bit position for thermal shutdown threshold
#define BQ25672_TSHUT_MASK       (0x3 << BQ25672_TSHUT_SHIFT) ///< Mask for thermal shutdown threshold
#define BQ25672_VBUS_PD_EN_SHIFT (0x3) ///< Bit position for VBUS pull-down enable
#define BQ25672_VBUS_PD_EN_MASK  (0x1 << BQ25672_VBUS_PD_EN_SHIFT) ///< Mask for VBUS pull-down enable
#define BQ25672_VAC1_PD_EN_SHIFT (0x2) ///< Bit position for VAC1 pull-down enable
#define BQ25672_VAC1_PD_EN_MASK  (0x1 << BQ25672_VAC1_PD_EN_SHIFT) ///< Mask for VAC1 pull-down enable
#define BQ25672_VAC2_PD_EN_SHIFT (0x1) ///< Bit position for VAC2 pull-down enable
#define BQ25672_VAC2_PD_EN_MASK  (0x1 << BQ25672_VAC2_PD_EN_SHIFT) ///< Mask for VAC2 pull-down enable
/** @} */

/** @name NTC_CONTROL_0 Register Bit Fields (0x17) */
/** @{ */
#define BQ25672_JEITA_VSET_SHIFT   (0x5) ///< Bit position for JEITA high-temp voltage adjustment
#define BQ25672_JEITA_VSET_MASK    (0x7 << BQ25672_JEITA_VSET_SHIFT) ///< Mask for JEITA high-temp voltage adjustment 
#define BQ25672_JEITA_ISETH_SHIFT  (0x3) ///< Bit position for JEITA high-temp current adjustment
#define BQ25672_JEITA_ISETH_MASK   (0x3 << BQ25672_JEITA_ISETH_SHIFT) ///< Mask for JEITA high-temp current adjustment
#define BQ25672_JEITA_ISETC_SHIFT  (0x1) ///< Bit position for JEITA low-temp current adjustment
#define BQ25672_JEITA_ISETC_MASK   (0x3 << BQ25672_JEITA_ISETC_SHIFT) ///< Mask for JEITA low-temp current adjustment
/** @} */

/** @name REG17_NTC_Control_0 (0x17) */
///@{
#define BQ25672_NTC_COLD_THRESHOLD_SHIFT 0U
#define BQ25672_NTC_COLD_THRESHOLD_MASK  (0xFFU << BQ25672_NTC_COLD_THRESHOLD_SHIFT) ///< 8-bit cold threshold :contentReference[oaicite:6]{index=6}&#8203;:contentReference[oaicite:7]{index=7}
///@}

/** @name NTC_CONTROL_1 Register Bit Fields (0x18) */
/** @{ */
#define BQ25672_TS_COOL_SHIFT      (0x6) ///< Bit position for TS cool threshold
#define BQ25672_TS_COOL_MASK       (0x3 << BQ25672_TS_COOL_SHIFT) ///< Mask for TS cool threshold
#define BQ25672_TS_WARM_SHIFT      (0x4) ///< Bit position for TS warm threshold
#define BQ25672_TS_WARM_MASK       (0x3 << BQ25672_TS_WARM_SHIFT) ///< Mask for TS warm threshold
#define BQ25672_TS_HOT_SHIFT       (0x2) ///< Bit position for TS hot threshold
#define BQ25672_TS_HOT_MASK        (0x3 << BQ25672_TS_HOT_SHIFT) ///< Mask for TS hot threshold
#define BQ25672_TS_COLD_SHIFT      (0x1) ///< Bit position for TS cold threshold
#define BQ25672_TS_COLD_MASK       (0x1 << BQ25672_TS_COLD_SHIFT) ///< Mask for TS cold threshold
#define BQ25672_TS_IGNORE_SHIFT    (0x0) ///< Bit position for TS ignore flag
#define BQ25672_TS_IGNORE_MASK     (0x1 << BQ25672_TS_IGNORE_SHIFT) ///< Mask for TS ignore flag
/** @} */

/** @name ICO_CURRENT_LIMIT Register Bit Fields (0x19) */
/** @{ */
#define BQ25672_ICO_CUR_SHIFT      (0x0) ///< Bit position for ICO current limit
#define BQ25672_ICO_CUR_MASK       (0x7F << BQ25672_ICO_CUR_SHIFT) ///< Mask for ICO current limit
/** @} */

/** @name CHARGER_STATUS_0 Register Bit Fields (0x1B) */
/** @{ */
#define BQ25672_VBUS_PRESENT_SHIFT (0x7) ///< Bit position for VBUS present status
#define BQ25672_VBUS_PRESENT_MASK  (0x1 << BQ25672_VBUS_PRESENT_SHIFT) ///< Mask for VBUS present status
#define BQ25672_CHARGER_ACTIVE_SHIFT (0x6) ///< Bit position for charger active status
#define BQ25672_CHARGER_ACTIVE_MASK  (0x1 << BQ25672_CHARGER_ACTIVE_SHIFT) ///< Mask for charger active status
#define BQ25672_OTG_MODE_SHIFT       (0x5) ///< Bit position for OTG mode status
#define BQ25672_OTG_MODE_MASK        (0x1 << BQ25672_OTG_MODE_SHIFT) ///< Mask for OTG mode status
/** @} */

/** @name CHARGER_STATUS_1 Register Bit Fields (0x1C) */
/** @{ */
#define BQ25672_CHG_STAT_SHIFT       (0x5) ///< Bit position for charge phase status
#define BQ25672_CHG_STAT_MASK        (0x7 << BQ25672_CHG_STAT_SHIFT) ///< Mask for charge phase status
#define BQ25672_VBUS_STAT_SHIFT      (0x1) ///< Bit position for VBUS adapter type status
#define BQ25672_VBUS_STAT_MASK       (0xF << BQ25672_VBUS_STAT_SHIFT) ///< Mask for VBUS adapter type status
#define BQ25672_BC12_DONE_SHIFT      (0x0) ///< Bit position for BC1.2 detection done
#define BQ25672_BC12_DONE_MASK       (0x1 << BQ25672_BC12_DONE_SHIFT) ///< Mask for BC1.2 detection done
/** @} */

/** @name REG1D_Charger_Status_2 (0x1D) */
///@{
#define BQ25672_ICO_STAT_SHIFT           6U
#define BQ25672_ICO_STAT_MASK            (0x03U << BQ25672_ICO_STAT_SHIFT)
#define BQ25672_TREG_STAT_MASK           0x04U
#define BQ25672_DPDM_STAT_MASK           0x02U
#define BQ25672_VBAT_PRESENT_STAT_MASK   0x01U
///@}

/** @name REG1E_Charger_Status_3 (0x1E) */
///@{
#define BQ25672_ACRB2_STAT_MASK          0x80U
#define BQ25672_ACRB1_STAT_MASK          0x40U
#define BQ25672_ADC_DONE_STAT_MASK       0x20U
#define BQ25672_VSYS_STAT_MASK           0x10U
#define BQ25672_CHG_TMR_STAT_MASK        0x08U
#define BQ25672_TRICKLE_TMR_STAT_MASK    0x04U
#define BQ25672_PRECHG_TMR_STAT_MASK     0x02U
///@}

/** @name REG1F_Charger_Status_4 (0x1F) */
///@{
#define BQ25672_VBATOTG_LOW_STAT_MASK    0x10U
#define BQ25672_TS_COLD_STAT_MASK        0x08U
#define BQ25672_TS_COOL_STAT_MASK        0x04U
#define BQ25672_TS_WARM_STAT_MASK        0x02U
#define BQ25672_TS_HOT_STAT_MASK         0x01U
///@}

/** @name FAULT_STATUS_0 Register Bit Fields (0x20) */
/** @{ */
#define BQ25672_INPUT_OVP_FAULT_SHIFT     (0x7) ///< Bit position for input OVP fault
#define BQ25672_INPUT_OVP_FAULT_MASK      (0x1 << BQ25672_INPUT_OVP_FAULT_SHIFT) ///< Mask for input OVP fault
#define BQ25672_INPUT_UVP_FAULT_SHIFT     (0x6) ///< Bit position for input UVP fault
#define BQ25672_INPUT_UVP_FAULT_MASK      (0x1 << BQ25672_INPUT_UVP_FAULT_SHIFT) ///< Mask for input UVP fault
#define BQ25672_THERMAL_SHUTDOWN_FAULT_SHIFT (0x5) ///< Bit position for thermal shutdown fault
#define BQ25672_THERMAL_SHUTDOWN_FAULT_MASK  (0x1 << BQ25672_THERMAL_SHUTDOWN_FAULT_SHIFT) ///< Mask for thermal shutdown fault
#define BQ25672_TS_COLD_FAULT_SHIFT       (0x4) ///< Bit position for TS cold fault
#define BQ25672_TS_COLD_FAULT_MASK        (0x1 << BQ25672_TS_COLD_FAULT_SHIFT) ///< Mask for TS cold fault
#define BQ25672_TS_HOT_FAULT_SHIFT        (0x3) ///< Bit position for TS hot fault
#define BQ25672_TS_HOT_FAULT_MASK         (0x1 << BQ25672_TS_HOT_FAULT_SHIFT) ///< Mask for TS hot fault
#define BQ25672_BATTERY_OVP_FAULT_SHIFT   (0x2) ///< Bit position for battery OVP fault
#define BQ25672_BATTERY_OVP_FAULT_MASK    (0x1 << BQ25672_BATTERY_OVP_FAULT_SHIFT) ///< Mask for battery OVP fault
#define BQ25672_WATCHDOG_TIMER_FAULT_SHIFT (0x1) ///< Bit position for watchdog expiry fault
#define BQ25672_WATCHDOG_TIMER_FAULT_MASK  (0x1 << BQ25672_WATCHDOG_TIMER_FAULT_SHIFT) ///< Mask for watchdog expiry
/** @} */

/** @name FAULT_STATUS_1 Register Bit Fields (0x21) */
/** @{ */
#define BQ25672_INPUT_SHORT_FAULT_SHIFT (0x0) ///< Bit position for input short fault
#define BQ25672_INPUT_SHORT_FAULT_MASK  (0x1 << BQ25672_INPUT_SHORT_FAULT_SHIFT) ///< Mask for input short fault
/** @} */

/** @name REG22_CHARGER_FLAG_0 (0x22): basic charger events */
///@{
#define BQ25672_IINDPM_FLAG_MASK        0x80U ///< bit7: IINDPM event (input current DPM)
#define BQ25672_VINDPM_FLAG_MASK        0x40U ///< bit6: VINDPM event (input voltage DPM)
#define BQ25672_WATCHDOG_FLAG_MASK      0x20U ///< bit5: WATCHDOG timer event
#define BQ25672_POORSRC_FLAG_MASK       0x10U ///< bit4: POORSRC event (bad adapter)
#define BQ25672_PG_FLAG_MASK            0x08U ///< bit3: PG_STAT event (power-good)
#define BQ25672_VAC2_PRESENT_FLAG_MASK  0x04U ///< bit2: VAC2_PRESENT event
#define BQ25672_VAC1_PRESENT_FLAG_MASK  0x02U ///< bit1: VAC1_PRESENT event
#define BQ25672_VBUS_PRESENT_FLAG_MASK  0x01U ///< bit0: VBUS_PRESENT event
///@}

/** @name REG26_FAULT_FLAG_0 (0x26): basic fault events */
///@{
#define BQ25672_INPUT_OVP_FLAG_MASK         0x80U ///< bit7: Input OVP fault
#define BQ25672_INPUT_UVP_FLAG_MASK         0x40U ///< bit6: Input UVP fault
#define BQ25672_THERMAL_SHUTDOWN_FLAG_MASK  0x20U ///< bit5: Thermal shutdown fault
#define BQ25672_TS_COLD_FLAG_MASK           0x10U ///< bit4: TS cold fault
#define BQ25672_TS_HOT_FLAG_MASK            0x08U ///< bit3: TS hot fault
#define BQ25672_BATTERY_OVP_FLAG_MASK       0x04U ///< bit2: Battery OVP fault
#define BQ25672_WATCHDOG_TIMER_FLAG_MASK    0x02U ///< bit1: Watchdog timer fault
#define BQ25672_INPUT_SHORT_FLAG_MASK       0x01U ///< bit0: Input short fault
///@}


/** @name CHARGER_MASK_0 Register Bit Fields (0x28) */
/** @{ */
#define BQ25672_IINDPM_MASK_SHIFT      (0x7) ///< Bit position for IINDPM mask
#define BQ25672_IINDPM_MASK_MASK       (0x1 << BQ25672_IINDPM_SHIFT) ///< Mask for IINDPM mask
#define BQ25672_VINDPM_MASK_SHIFT      (0x6) ///< Bit position for VINDPM mask
#define BQ25672_VINDPM_MASK_MASK       (0x1 << BQ25672_VINDPM_SHIFT) ///< Mask for VINDPM mask
#define BQ25672_WD_MASK_SHIFT          (0x5) ///< Bit position for watchdog mask
#define BQ25672_WD_MASK_MASK           (0x1 << BQ25672_WD_MASK_SHIFT) ///< Mask for watchdog mask
#define BQ25672_POORSRC_MASK_SHIFT     (0x4) ///< Bit position for poor-source mask
#define BQ25672_POORSRC_MASK_MASK      (0x1 << BQ25672_POORSRC_MASK_SHIFT) ///< Mask for poor-source mask
#define BQ25672_PG_MASK_SHIFT          (0x3) ///< Bit position for power-good mask
#define BQ25672_PG_MASK_MASK           (0x1 << BQ25672_PG_MASK_SHIFT) ///< Mask for power-good mask
#define BQ25672_AC2_PRESENT_MASK_SHIFT (0x2) ///< Bit position for VAC2 present mask
#define BQ25672_AC2_PRESENT_MASK_MASK  (0x1 << BQ25672_AC2_PRESENT_MASK_SHIFT) ///< Mask for VAC2 present mask
#define BQ25672_AC1_PRESENT_MASK_SHIFT (0x1) ///< Bit position for VAC1 present mask
#define BQ25672_AC1_PRESENT_MASK_MASK  (0x1 << BQ25672_AC1_PRESENT_MASK_SHIFT) ///< Mask for VAC1 present mask
#define BQ25672_VBUS_PRESENT_MASK_SHIFT (0x0) ///< Bit position for VBUS present mask
#define BQ25672_VBUS_PRESENT_MASK_MASK (0x1 << BQ25672_VBUS_PRESENT_SHIFT) ///< Mask for VBUS present mask
/** @} */

/** @name FAULT_MASK_0 Register Bit Fields (0x2C) */
/** @{ */
#define BQ25672_IBAT_REG_MASK_SHIFT    (0x7) ///< Bit position for IBAT regulation mask
#define BQ25672_IBAT_REG_MASK_MASK     (0x1 << BQ25672_IBAT_REG_MASK_SHIFT) ///< Mask for IBAT regulation mask
#define BQ25672_VBUS_OVP_MASK_SHIFT    (0x6) ///< Bit position for VBUS OVP mask
#define BQ25672_VBUS_OVP_MASK_MASK     (0x1 << BQ25672_VBUS_OVP_MASK_SHIFT) ///< Mask for VBUS OVP mask
#define BQ25672_VBAT_OVP_MASK_SHIFT    (0x5) ///< Bit position for VBAT OVP mask
#define BQ25672_VBAT_OVP_MASK_MASK     (0x1 << BQ25672_VBAT_OVP_MASK_SHIFT) ///< Mask for VBAT OVP mask
#define BQ25672_IBUS_OCP_MASK_SHIFT    (0x4) ///< Bit position for IBUS OCP mask
#define BQ25672_IBUS_OCP_MASK_MASK     (0x1 << BQ25672_IBUS_OCP_MASK_SHIFT) ///< Mask for IBUS OCP mask
#define BQ25672_IBAT_OCP_MASK_SHIFT    (0x3) ///< Bit position for IBAT OCP mask
#define BQ25672_IBAT_OCP_MASK_MASK     (0x1 << BQ25672_IBAT_OCP_MASK_SHIFT) ///< Mask for IBAT OCP mask
#define BQ25672_CONV_OCP_MASK_SHIFT    (0x2) ///< Bit position for converter OCP mask
#define BQ25672_CONV_OCP_MASK_MASK     (0x1 << BQ25672_CONV_OCP_MASK_SHIFT) ///< Mask for converter OCP mask
#define BQ25672_VAC2_OVP_MASK_SHIFT    (0x1) ///< Bit position for VAC2 OVP mask
#define BQ25672_VAC2_OVP_MASK_MASK     (0x1 << BQ25672_VAC2_OVP_MASK_SHIFT) ///< Mask for VAC2 OVP mask
#define BQ25672_VAC1_OVP_MASK_SHIFT    (0x0) ///< Bit position for VAC1 OVP mask
#define BQ25672_VAC1_OVP_MASKMASK     (0x1 << BQ25672_VAC1_OVP_MASK_SHIFT) ///< Mask for VAC1 OVP mask
/** @} */

/** @name FAULT_MASK_1 Register Bit Fields (0x2D) */
/** @{ */
#define BQ25672_VSYS_SHORT_MASK_SHIFT  (0x7) ///< Bit position for VSYS short mask
#define BQ25672_VSYS_SHORT_MASK_MASK   (0x1 << BQ25672_VSYS_SHORT_MASK_SHIFT) ///< Mask for VSYS short mask 
#define BQ25672_VSYS_OVP_MASK_SHIFT    (0x6) ///< Bit position for VSYS OVP mask
#define BQ25672_VSYS_OVP_MASK_MASK     (0x1 << BQ25672_VSYS_OVP_MASK_SHIFT) ///< Mask for VSYS OVP mask
#define BQ25672_OTG_OVP_MASK_SHIFT     (0x5) ///< Bit position for OTG OVP mask
#define BQ25672_OTG_OVP_MASK_MASK      (0x1 << BQ25672_OTG_OVP_MASK_SHIFT) ///< Mask for OTG OVP mask
#define BQ25672_OTG_UVP_MASK_SHIFT     (0x4) ///< Bit position for OTG UVP mask
#define BQ25672_OTG_UVP_MASK_MASK      (0x1 << BQ25672_OTG_UVP_MASK_SHIFT) ///< Mask for OTG UVP mask
#define BQ25672_TSHUT_MASK_SHIFT       (0x2) ///< Bit position for thermal shutdown mask
#define BQ25672_TSHUT_MASK_MASK        (0x1 << BQ25672_TSHUT_SHIFT) ///< Mask for thermal shutdown mask
/** @} */

/** @name ADC_CONTROL Register Bit Fields (0x2E) */
/** @{ */
#define BQ25672_ADC_EN_SHIFT           (0x7) ///< Bit position to enable ADC
#define BQ25672_ADC_EN_MASK            (0x1 << BQ25672_ADC_EN_SHIFT) ///< Mask to enable ADC
#define BQ25672_ADC_RATE_SHIFT         (0x6) ///< Bit position for ADC rate mode
#define BQ25672_ADC_RATE_MASK          (0x1 << BQ25672_ADC_RATE_SHIFT) ///< Mask for ADC rate mode
#define BQ25672_ADC_SAMPLE_SHIFT       (0x4) ///< Bit position for ADC sample bits
#define BQ25672_ADC_SAMPLE_MASK        (0x3 << BQ25672_ADC_SAMPLE_SHIFT) ///< Mask for ADC sample bits
#define BQ25672_ADC_AVG_SHIFT          (0x3) ///< Bit position for ADC averaging enable
#define BQ25672_ADC_AVG_MASK           (0x1 << BQ25672_ADC_AVG_SHIFT) ///< Mask for ADC averaging enable
#define BQ25672_ADC_AVG_INIT_SHIFT     (0x2) ///< Bit position for ADC average init
#define BQ25672_ADC_AVG_INIT_MASK      (0x1 << BQ25672_ADC_AVG_INIT_SHIFT) ///< Mask for ADC average init
/** @} */

/** @name ADC_FUNCTION_DISABLE_0 Register Bit Fields (0x2F) */
/** @{ */
#define BQ25672_IBUS_ADC_DIS_SHIFT     (0x7) ///< Bit position to disable IBUS ADC
#define BQ25672_IBUS_ADC_DIS_MASK      (0x1 << BQ25672_IBUS_ADC_DIS_SHIFT) ///< Mask to disable IBUS ADC
#define BQ25672_IBAT_ADC_DIS_SHIFT     (0x6) ///< Bit position to disable IBAT ADC
#define BQ25672_IBAT_ADC_DIS_MASK      (0x1 << BQ25672_IBAT_ADC_DIS_SHIFT) ///< Mask to disable IBAT ADC
#define BQ25672_VBUS_ADC_DIS_SHIFT     (0x5) ///< Bit position to disable VBUS ADC
#define BQ25672_VBUS_ADC_DIS_MASK      (0x1 << BQ25672_VBUS_ADC_DIS_SHIFT) ///< Mask to disable VBUS ADC
#define BQ25672_VBAT_ADC_DIS_SHIFT     (0x4) ///< Bit position to disable VBAT ADC
#define BQ25672_VBAT_ADC_DIS_MASK      (0x1 << BQ25672_VBAT_ADC_DIS_SHIFT) ///< Mask to disable VBAT ADC
#define BQ25672_SYS_ADC_DIS_SHIFT      (0x3) ///< Bit position to disable VSYS ADC
#define BQ25672_SYS_ADC_DIS_MASK       (0x1 << BQ25672_SYS_ADC_DIS_SHIFT) ///< Mask to disable VSYS ADC
#define BQ25672_TS_ADC_DIS_SHIFT       (0x2) ///< Bit position to disable TS ADC
#define BQ25672_TS_ADC_DIS_MASK        (0x1 << BQ25672_TS_ADC_DIS_SHIFT) ///< Mask to disable TS ADC
#define BQ25672_TDIE_ADC_DIS_SHIFT     (0x1) ///< Bit position to disable TDIE ADC
#define BQ25672_TDIE_ADC_DIS_MASK      (0x1 << BQ25672_TDIE_ADC_DIS_SHIFT) ///< Mask to disable TDIE ADC
/** @} */

/** @name ADC_FUNCTION_DISABLE_1 Register Bit Fields (0x30) */
/** @{ */
#define BQ25672_DP_ADC_DIS_SHIFT       (0x7) ///< Bit position to disable D+ ADC
#define BQ25672_DP_ADC_DIS_MASK        (0x1 << BQ25672_DP_ADC_DIS_SHIFT) ///< Mask to disable D+ ADC
#define BQ25672_DM_ADC_DIS_SHIFT       (0x6) ///< Bit position to disable D- ADC
#define BQ25672_DM_ADC_DIS_MASK        (0x1 << BQ25672_DM_ADC_DIS_SHIFT) ///< Mask to disable D- ADC
#define BQ25672_VAC2_ADC_DIS_SHIFT     (0x5) ///< Bit position to disable VAC2 ADC
#define BQ25672_VAC2_ADC_DIS_MASK      (0x1 << BQ25672_VAC2_ADC_DIS_SHIFT) ///< Mask to disable VAC2 ADC
#define BQ25672_VAC1_ADC_DIS_SHIFT     (0x4) ///< Bit position to disable VAC1 ADC
#define BQ25672_VAC1_ADC_DIS_MASK      (0x1 << BQ25672_VAC1_ADC_DIS_SHIFT) ///< Mask to disable VAC1 ADC
/** @} */

/** @name DPDM_DRIVER Register Bit Fields (0x47) */
/** @{ */
#define BQ25672_DPLUS_DAC_SHIFT        (0x5) ///< Bit position for D+ driver level
#define BQ25672_DPLUS_DAC_MASK         (0x7 << BQ25672_DPLUS_DAC_SHIFT) ///< Mask for D+ driver level 
#define BQ25672_DMINUS_DAC_SHIFT       (0x2) ///< Bit position for D- driver level
#define BQ25672_DMINUS_DAC_MASK        (0x7 << BQ25672_DMINUS_DAC_SHIFT) ///< Mask for D- driver level
/** @} */

/** @name PART_INFORMATION Register Bit Fields (0x48) */
/** @{ */
#define BQ25672_PN_SHIFT               (0x3) ///< Bit position for part number
#define BQ25672_PN_MASK                (0x7 << BQ25672_PN_SHIFT) ///< Mask for part number
#define BQ25672_DEV_REV_SHIFT          (0x0) ///< Bit position for device revision
#define BQ25672_DEV_REV_MASK           (0x7 << BQ25672_DEV_REV_SHIFT) ///< Mask for device revision
/** @} */

#ifdef __cplusplus
}
#endif

#endif // BQ25672_REGISTERS_H
