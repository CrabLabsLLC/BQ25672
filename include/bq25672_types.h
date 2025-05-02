/**
 * @file bq25672_types.h
 * @author Orion Serup (orion@crablabs.io)
 * @brief Defines the types and default values for BQ25672
 * @version 0.1
 * @date 2025-04-29
 * 
 * @license MIT
 * @copyright Copyright (c) Crab Labs LLC 2025
 * 
 */

#pragma once

#ifndef BQ25672_TYPES_H
#define BQ25672_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/*==============================================================================
* Return codes
*============================================================================*/
/**
 * @brief Driver status return codes
 */
typedef enum 
{
    BQ25672_OK             =  0,   ///< Operation successful
    BQ25672_ERROR          = -1,   ///< General failure
    BQ25672_INVALID_PARAM  = -2,   ///< Bad parameter provided
    BQ25672_COMM_FAIL      = -3    ///< I2C transaction failed
} bq25672_status_t;

/*==============================================================================
* HAL / platform callbacks
*============================================================================*/
/**
 * @brief I2C write function prototype
 * @param dev_addr 7-bit device I2C address
 * @param reg_addr Register address to write
 * @param data     Pointer to data buffer
 * @param len      Number of bytes to write
 * @return true on success, false on failure
 */
typedef bool (*bq25672_i2c_write_fn)(const uint8_t dev_addr,
                                    const uint8_t reg_addr,
                                    const void* const data,
                                    const uint8_t len);

/**
 * @brief I2C read function prototype
 * @param dev_addr 7-bit device I2C address
 * @param reg_addr Register address to read
 * @param data     Pointer to buffer to fill
 * @param len      Number of bytes to read
 * @return true on success, false on failure
 */
typedef bool (*bq25672_i2c_read_fn)(const uint8_t dev_addr,
                                   const  uint8_t reg_addr,
                                   void* const data,
                                   const uint8_t len);

/**
 * @brief Enumerates the charger's GPIO pins for CE, QON, INT, STAT
 */
typedef enum 
{
    BQ25672_PIN_CE,     ///< CE input (active-low charge enable)
    BQ25672_PIN_QON,    ///< QON input (ship-mode control)
    BQ25672_PIN_INT,    ///< INT output (open-drain interrupt)
    BQ25672_PIN_STAT    ///< STAT output (open-drain status)
} bq25672_pin_t;

/**
 * @brief GPIO write callback
 * @param pin   Which pin to drive
 * @param state True = high, False = low
 * @return true on success, false on failure
 */
typedef bool (*bq25672_pin_write_fn)(bq25672_pin_t pin, bool state);

/**
 * @brief GPIO read callback
 * @param pin   Which pin to sample
 * @param state Pointer to receive pin level
 * @return true on success, false on failure
 */
typedef bool (*bq25672_pin_read_fn)(bq25672_pin_t pin, bool *state);

/**
 * @brief Millisecond delay function
 * @param ms Number of milliseconds to delay
 */
typedef void (*bq25672_delay_ms_fn)(uint32_t ms);

/*==============================================================================
* Fundamental enums for register fields
*============================================================================*/
/**
 * @brief Number of battery cells configuration
 */
typedef enum 
{
    BQ25672_CELL_1S = 0x0,  ///< Single-cell
    BQ25672_CELL_2S = 0x1,  ///< Two-cell
    BQ25672_CELL_3S = 0x2,  ///< Three-cell
    BQ25672_CELL_4S = 0x3   ///< Four-cell
} bq25672_battery_cells_t;

/**
 * @brief Switching frequency options
 */
typedef enum 
{
    BQ25672_SWITCH_FREQ_750KHZ  = 0x0, ///< 750 kHz PWM
    BQ25672_SWITCH_FREQ_1500KHZ = 0x1  ///< 1.5 MHz PWM
} bq25672_switching_freq_t;

/**
 * @brief Precharge voltage cutoff as percentage of regulation
 */
typedef enum 
{
    BQ25672_PRECHG_15PCT = 0x0, ///< 15%
    BQ25672_PRECHG_62P2  = 0x1, ///< 62.2%
    BQ25672_PRECHG_66P7  = 0x2, ///< 66.7%
    BQ25672_PRECHG_71P4  = 0x3  ///< 71.4%
} bq25672_precharge_pct_t;

/**
 * @brief Watchdog timer timeout durations
 */
typedef enum 
{
    BQ25672_WDT_500MS  = 0x0, ///<  500 ms
    BQ25672_WDT_1S     = 0x1, ///<    1 s
    BQ25672_WDT_2S     = 0x2, ///<    2 s
    BQ25672_WDT_10S    = 0x3, ///<   10 s
    BQ25672_WDT_20S    = 0x4, ///<   20 s
    BQ25672_WDT_40S    = 0x5, ///<   40 s
    BQ25672_WDT_80S    = 0x6, ///<   80 s
    BQ25672_WDT_160S   = 0x7  ///<  160 s
} bq25672_watchdog_time_t;

/**
 * @brief MPPT (solar) tracking ratio options
 */
typedef enum 
{
    BQ25672_MPPT_56P25 = 0x0, ///< 56.25%
    BQ25672_MPPT_62P5  = 0x1, ///< 62.5%
    BQ25672_MPPT_68P75 = 0x2, ///< 68.75%
    BQ25672_MPPT_75P0  = 0x3, ///< 75.0%
    BQ25672_MPPT_81P25 = 0x4, ///< 81.25%
    BQ25672_MPPT_87P5  = 0x5, ///< 87.5%
    BQ25672_MPPT_93P75 = 0x6, ///< 93.75%
    BQ25672_MPPT_100P0 = 0x7  ///< 100.0%
} bq25672_mppt_pct_t;

/**
 * @brief NTC (battery thermistor) threshold levels
 */
typedef enum 
{
    BQ25672_NTC_NEG20C = 0x0, ///< −20 °C
    BQ25672_NTC_NEG10C = 0x1, ///< −10 °C
    BQ25672_NTC_5C    = 0x2, ///<  +5 °C
    BQ25672_NTC_10C   = 0x3, ///< +10 °C
    BQ25672_NTC_15C   = 0x4, ///< +15 °C
    BQ25672_NTC_20C   = 0x5, ///< +20 °C
    BQ25672_NTC_40C   = 0x6, ///< +40 °C
    BQ25672_NTC_45C   = 0x7, ///< +45 °C
    BQ25672_NTC_50C   = 0x8, ///< +50 °C
    BQ25672_NTC_55C   = 0x9, ///< +55 °C
    BQ25672_NTC_60C   = 0xA, ///< +60 °C
    BQ25672_NTC_65C   = 0xB  ///< +65 °C
} bq25672_ntc_thresh_t;

/**
 * @brief JEITA profile voltage reduction at high temperature
 */
typedef enum 
{
    BQ25672_JEITA_NO_CHANGE   = 0x0, ///< No change
    BQ25672_JEITA_MINUS_100MV = 0x1, ///< −100 mV
    BQ25672_JEITA_MINUS_200MV = 0x2, ///< −200 mV
    BQ25672_JEITA_MINUS_300MV = 0x3, ///< −300 mV
    BQ25672_JEITA_MINUS_400MV = 0x4, ///< −400 mV
    BQ25672_JEITA_MINUS_600MV = 0x5, ///< −600 mV
    BQ25672_JEITA_MINUS_800MV = 0x6, ///< −800 mV
    BQ25672_JEITA_SUSPEND     = 0x7  ///< Suspend charging
} bq25672_jeita_vset_t;

/**
 * @brief JEITA profile current scaling
 */
typedef enum 
{
    BQ25672_ISET_NO_CHANGE = 0x0, ///< No change
    BQ25672_ISET_20PCT     = 0x1, ///< 20 %
    BQ25672_ISET_40PCT     = 0x2, ///< 40 %
    BQ25672_ISET_SUSPEND   = 0x3  ///< Suspend charging
} bq25672_jeita_iset_t;

/*==============================================================================
 * Additional enums for advanced STATUS_* fields
 *============================================================================*/
/**
 * @brief Input Current Optimizer (ICO) state from STATUS_2[7:6]
 */
typedef enum 
{
    BQ25672_ICO_DISABLED        = 0, ///< ICO disabled
    BQ25672_ICO_IN_PROGRESS     = 1, ///< ICO optimization in progress
    BQ25672_ICO_PEAK_DETECTED   = 2, ///< Maximum input current detected
    BQ25672_ICO_RESERVED        = 3  ///< Reserved
} bq25672_ico_status_t;

/**
 * @brief Charge phase (STATUS_1[7:5])
 */
typedef enum 
{
    BQ25672_CHARGE_STAT_NOT_CHARGING = 0, ///< Not charging
    BQ25672_CHARGE_STAT_PRECHARGE    = 1, ///< Trickle/pre‐charge
    BQ25672_CHARGE_STAT_FAST_CHARGE  = 2, ///< Constant‐current
    BQ25672_CHARGE_STAT_TAPER        = 3, ///< Constant‐voltage taper
    BQ25672_CHARGE_STAT_DONE         = 4, ///< Terminated (I < ITERM)
    BQ25672_CHARGE_STAT_UNKNOWN      = 7  ///< Reserved/unknown
} bq25672_charge_phase_t;

/**
 * @brief Adapter type (STATUS_1[4:1])
 */
typedef enum 
{
    BQ25672_ADAPTER_NONE     = 0,  ///< No adapter
    BQ25672_ADAPTER_SDP      = 1,  ///< USB SDP
    BQ25672_ADAPTER_CDP      = 2,  ///< USB CDP
    BQ25672_ADAPTER_DCP      = 3,  ///< Dedicated charging port
    BQ25672_ADAPTER_HVDCP5   = 4,  ///< HVDCP 5 V
    BQ25672_ADAPTER_HVDCP9   = 5,  ///< HVDCP 9 V
    BQ25672_ADAPTER_HVDCP12  = 6,  ///< HVDCP 12 V
    BQ25672_ADAPTER_APPLE2P4 = 7,  ///< Apple 2.4 A
    BQ25672_ADAPTER_APPLE1   = 8,  ///< Apple 1 A
    BQ25672_ADAPTER_APPLE0P5 = 9,  ///< Apple 0.5 A
    BQ25672_ADAPTER_UNKNOWN  = 0xF ///< Reserved/unknown
} bq25672_adapter_type_t;

/*==============================================================================
 * Device status snapshot
 *============================================================================*/

/**
 * @brief Snapshot of BQ25672 device status
 */
typedef struct 
{
    bool vbus_present;   ///< STATUS_0[7]: VBUS_PRESENT
    bool charger_active; ///< STATUS_0[6]: CHARGER_ACTIVE
    bool otg_mode;       ///< STATUS_0[5]: OTG_MODE
    bool bc12_done;      ///< STATUS_1[0]: BC1.2_DONE_STAT
    
    bq25672_charge_phase_t charge_phase;   ///< STATUS_1[7:5]: CHG_STAT
    bq25672_adapter_type_t adapter_type;   ///< STATUS_1[4:1]: VBUS_STAT
    struct 
    {
        /* STATUS_2 (0x1D) */
        bq25672_ico_status_t    ico_state;         ///< ICO_STAT[1:0]

        bool thermal_regulation;///< TREG_STAT (in thermal fold-back)
        bool dpdm_detection;    ///< DPDM_STAT (D+/D- detection ongoing)
        bool vbat_present;      ///< VBAT_PRESENT_STAT

        /* STATUS_3 (0x1E) */
        bool acrb2_placed;      ///< ACRB2_STAT (port-2 FETs on)
        bool acrb1_placed;      ///< ACRB1_STAT (port-1 FETs on)
        bool adc_done;          ///< ADC_DONE_STAT (one-shot complete)
        bool vsys_min_reg;      ///< VSYS_STAT (in VSYSMIN regulation)
        bool fast_timer_exp;    ///< CHG_TMR_STAT
        bool trickle_timer_exp; ///< TRICHG_TMR_STAT
        bool precharge_timer_exp;///< PRECHG_TMR_STAT

        /* STATUS_4 (0x1F) */
        bool vbat_otg_too_low;  ///< VBATOTG_LOW_STAT
        bool ts_cold;           ///< TS_COLD_STAT
        bool ts_cool;           ///< TS_COOL_STAT
        bool ts_warm;           ///< TS_WARM_STAT
        bool ts_hot;            ///< TS_HOT_STAT
    } advanced;
} bq25672_device_status_t;

/**
 * @brief Snapshot of all BQ25672 interrupt flags
 *        Basic flags = CHARGER_FLAG_0 (0x22)
 *        Advanced flags = CHARGER_FLAG_1…3 + FAULT_FLAG_0…1
 */
typedef struct 
{

    bool iindpm;       ///< IINDPM event (input‐current DPM triggered)
    bool vindpm;       ///< VINDPM event (input‐voltage DPM triggered)
    bool watchdog;     ///< WATCHDOG_TIMER event
    bool poor_source;  ///< POORSRC event (bad adapter)
    bool power_good;   ///< PG_STAT event (power‐good after OVP)
    bool ac2_present;  ///< VAC2_PRESENT event
    bool ac1_present;  ///< VAC1_PRESENT event
    bool vbus_present; ///< VBUS_PRESENT event
    bool input_ovp;        ///< INPUT_OVP fault
    bool input_uvp;        ///< INPUT_UVP fault
    bool thermal_shutdown; ///< THERMAL_SHUTDOWN fault
    bool ts_cold;          ///< TS_COLD fault
    bool ts_hot;           ///< TS_HOT fault
    bool battery_ovp;      ///< BATTERY_OVP fault
    bool watchdog_fault;   ///< WATCHDOG_TIMER fault
    bool input_short;      ///< INPUT_SHORT fault

} bq25672_interrupt_status_t;

/*==============================================================================
* Configuration structures
*============================================================================*/
/**
 * @brief Core system behavior configuration
 */
typedef struct 
{
    /* Basic: major behavior */
    bool disable_pfm;       ///< Turn off PFM
    bool disable_ldo;       ///< Turn off LDO
    bool disable_ooa;       ///< Turn off OOA
    bool external_ac1_fet;  ///< If AC1 is externally switched
    bool external_ac2_fet;  ///< If AC2 is externally switched
    bq25672_switching_freq_t   switching_freq;   ///< PWM frequency

    /* Advanced: fine-grained controls */
    struct 
    {
        bool disable_stat;       ///< Disable STAT pin
        bool enable_hiz;         ///< Enable Hi-Z mode
        bool enable_termination; ///< Terminate on I < ITERM
        bool enable_iindpm;      ///< Enable IINDPM loop
        bool force_iindpm;       ///< Force IINDPM measurement
        bool force_vindpm;       ///< Force VINDPM measurement 
        bool enable_ico;         ///< Enable Input Current Optimizer
        bool force_ico;          ///< Force ICO measurement
    } advanced;
} bq25672_system_config_t;

/**
 * @brief Enable/disable configuration of all interrupts
 */
typedef struct
{
    /* Basic charger events */
    bool iindpm;       ///< Enable IINDPM interrupt
    bool vindpm;       ///< Enable VINDPM interrupt
    bool watchdog;     ///< Enable WATCHDOG_TIMER interrupt
    bool poor_source;  ///< Enable POORSRC interrupt
    bool power_good;   ///< Enable PG_STAT interrupt
    bool ac2_present;  ///< Enable VAC2_PRESENT interrupt
    bool ac1_present;  ///< Enable VAC1_PRESENT interrupt
    bool vbus_present; ///< Enable VBUS_PRESENT interrupt

    /* Basic fault events */
    bool input_ovp;        ///< Enable INPUT_OVP fault interrupt
    bool input_uvp;        ///< Enable INPUT_UVP fault interrupt
    bool thermal_shutdown; ///< Enable THERMAL_SHUTDOWN fault interrupt
    bool ts_cold;          ///< Enable TS_COLD fault interrupt
    bool ts_hot;           ///< Enable TS_HOT fault interrupt
    bool battery_ovp;      ///< Enable BATTERY_OVP fault interrupt
    bool watchdog_fault;   ///< Enable WATCHDOG_TIMER fault interrupt
    bool input_short;      ///< Enable INPUT_SHORT fault interrupt

} bq25672_interrupt_config_t;

/**
 * @brief Charge regulation parameters
 */
typedef struct 
{
    /* Basic: battery charge targets */
    uint16_t charge_voltage_max_mv;  ///< VREG target (mV)
    uint16_t charge_current_max_ma;  ///< ICHG target (mA)

    /* Advanced: input & termination */
    struct 
    {
        uint16_t input_voltage_max_mv;          ///< VINDPM max (mV)
        uint16_t input_voltage_min_mv;          ///< VBUS UVLO (mV)
        uint16_t input_current_max_ma;          ///< IINDPM max (mA)
        uint8_t  termination_current_ma;        ///< ITERM (mA)
        bq25672_precharge_pct_t precharge_pct;  ///< VRECHG %
    } advanced;
} bq25672_charge_config_t;

/**
 * @brief USB/OTG boost-mode settings
 */
typedef struct 
{
    /* Basic: OTG enable & targets */
    bool                        enable_otg;       ///< Enable OTG
    uint16_t                    voltage_mv;       ///< VOTG target (mV)
    uint16_t                    current_max_ma;   ///< IOTG limit (mA)

    /* Advanced: mode tuning */
    struct 
    {
        bool disable_pfm;      ///< Disable PFM in OTG
        bool disable_ooa;      ///< Disable OOA in OTG
    } advanced;
} bq25672_usb_config_t;

/**
 * @brief Charger timer enables
 */
typedef struct 
{
    /* Basic timers */
    bool top_off_enable;    ///< Top-off timer
    bool trickle_enable;    ///< Trickle timer
    bool fast_charge_enable;///< Fast-charge timer
    struct 
    {
        bool precharge_short;   ///< Short pre-charge timer
        bool precharge_enable;  ///< Pre-charge timer
    } advanced;
} bq25672_timer_config_t;

/**
 * @brief Watchdog timer configuration
 */
typedef struct 
{
    /* Basic: period */
    bq25672_watchdog_time_t     timeout;          ///< Watchdog period
    bool disable_on_expiry; ///< Halt charging when WDT fires
} bq25672_watchdog_config_t;

/**
 * @brief MPPT (solar) tracking configuration
 */
typedef struct {
    /* Basic: enable & ratio */
    bool   enable_mppt;      ///< Enable VOC sampling
    bq25672_mppt_pct_t ratio;            ///< VOC tracking ratio

    /* Advanced timing */
    struct 
    {
        uint32_t delay_ms;         ///< Delay after disconnect (ms)
        uint32_t interval_s;       ///< Interval between samples (s)
    } advanced;
} bq25672_mppt_config_t;

/**
 * @brief NTC (battery-thermistor) monitoring configuration
 */
typedef struct 
{
    /* Basic thresholds */
    bq25672_ntc_thresh_t        cold_thresh;      ///< Suspend below
    bq25672_ntc_thresh_t        hot_thresh;       ///< Suspend above
    bool enable_monitor;    ///< Enable TS pin comparators

} bq25672_ntc_config_t;

/**
 * @brief JEITA (temperature-based charging) profile
 */
typedef struct 
{
    bq25672_jeita_vset_t    vset_high;        ///< High-temp voltage offset
    bq25672_jeita_iset_t    iseth_high;       ///< High-temp current offset
    bq25672_jeita_iset_t    isetc_cold;       ///< Cold-temp current offset

} bq25672_jeita_config_t;

/*==============================================================================
* Top-level driver configuration
*============================================================================*/
/**
 * @brief Complete driver configuration
 */
typedef struct 
{
    bq25672_system_config_t     system;     ///< System behavior
    bq25672_charge_config_t     charge;     ///< Charging parameters
    bq25672_usb_config_t        usb;        ///< USB/OTG settings
    bq25672_timer_config_t      timer;      ///< Timer enables
    bq25672_watchdog_config_t   watchdog;   ///< Watchdog
    bq25672_mppt_config_t       mppt;       ///< MPPT tracking
    bq25672_ntc_config_t        ntc;        ///< Thermistor monitoring
    bq25672_jeita_config_t      jeita;      ///< JEITA profile
    bq25672_interrupt_config_t interrupt;   ///< Interrupt Config
} bq25672_config_t;

/**
 * @brief Hardware abstraction layer callbacks
 */
typedef struct {
    bq25672_i2c_read_fn    i2c_read;   ///< I2C read
    bq25672_i2c_write_fn   i2c_write;  ///< I2C write
#ifndef BQ25672_MINIMAL_HAL
    bq25672_pin_read_fn    gpio_read;  ///< GPIO read
    bq25672_pin_write_fn   gpio_write; ///< GPIO write
#endif
} bq25672_hal_t;

/**
 * @brief Main driver handle
 */
typedef struct {
    bq25672_hal_t          hal;       ///< HAL callbacks
#ifndef BQ25672_MINIMAL_RAM
    bq25672_config_t       cfg;       ///< Active configuration
#endif
} bq25672_t;

/*==============================================================================
 * Default configuration instances (C99 static const)
 *============================================================================*/
/** @brief Default system configuration (balanced performance) */
static const bq25672_system_config_t BQ25672_DEFAULT_SYSTEM_CONFIG = 
{
    .disable_pfm       = false,
    .disable_ldo       = false,
    .disable_ooa       = false,
    .external_ac1_fet  = false,
    .external_ac2_fet  = false,
    .switching_freq    = BQ25672_SWITCH_FREQ_750KHZ,
    .advanced = 
    {
        .disable_stat        = false,
        .enable_hiz          = false,
        .enable_termination  = true,
        .force_vindpm        = false,
        .enable_iindpm       = true,
        .force_iindpm        = false,
        .enable_ico          = true,
        .force_ico           = false
    }
};

/** @brief Default charging parameters (1 A/4.2 V, moderate input limits) */
static const bq25672_charge_config_t BQ25672_DEFAULT_CHARGE_CONFIG = 
{
    .charge_voltage_max_mv   = 4200u,
    .charge_current_max_ma   = 1000u,
    .advanced = {
        .input_voltage_max_mv    = 5000u,
        .input_voltage_min_mv    = 3600u,
        .input_current_max_ma    = 500u,
        .termination_current_ma  = 120u,
        .precharge_pct           = BQ25672_PRECHG_15PCT
    }
};

/** @brief Default USB/OTG settings (OTG disabled) */
static const bq25672_usb_config_t BQ25672_DEFAULT_USB_CONFIG = 
{
    .enable_otg         = false,
    .voltage_mv         = 5000u,
    .current_max_ma     = 500u,
    .advanced = 
    {
        .disable_pfm    = false,
        .disable_ooa    = false
    }
};

/** @brief Default timer configuration (all timers on) */
static const bq25672_timer_config_t BQ25672_DEFAULT_TIMER_CONFIG = 
{
    .top_off_enable     = true,
    .trickle_enable     = true,
    .fast_charge_enable = true,
    .advanced = {
        .precharge_short  = false,
        .precharge_enable = true
    }
};

/** @brief Default watchdog configuration (160 s, halt on expiry) */
static const bq25672_watchdog_config_t BQ25672_DEFAULT_WDT_CONFIG = 
{
    .timeout = BQ25672_WDT_160S,
    .disable_on_expiry = true
};

/** @brief Default MPPT configuration (MPPT disabled) */
static const bq25672_mppt_config_t BQ25672_DEFAULT_MPPT_CONFIG = 
{
    .enable_mppt        = false,
    .ratio              = BQ25672_MPPT_75P0,
    .advanced = 
    {
        .delay_ms       = 5000u,
        .interval_s     = 30u
    }
};

/** @brief Default NTC thresholds (–20 °C…+50 °C) */
static const bq25672_ntc_config_t BQ25672_DEFAULT_NTC_CONFIG = 
{
    .enable_monitor = true,
    .cold_thresh = BQ25672_NTC_NEG20C,
    .hot_thresh  = BQ25672_NTC_50C,
};

/** @brief Default JEITA profile (JEITA disabled) */
static const bq25672_jeita_config_t BQ25672_DEFAULT_JEITA_CONFIG = 
{
    .vset_high = BQ25672_JEITA_NO_CHANGE,
    .iseth_high = BQ25672_ISET_NO_CHANGE,
    .isetc_cold = BQ25672_ISET_NO_CHANGE
};

/** @brief High-level default driver configuration */
static const bq25672_config_t BQ25672_DEFAULT_CONFIG = 
{
    .system = 
    {
        .disable_pfm       = false,
        .disable_ldo       = false,
        .disable_ooa       = false,
        .switching_freq    = BQ25672_SWITCH_FREQ_750KHZ,
        .advanced = 
        {
            .disable_stat        = false,
            .enable_hiz          = false,
            .enable_termination  = true,
            .enable_iindpm       = true,
            .force_iindpm        = false,
            .enable_ico          = true,
            .force_ico           = false
        }
    },
    .charge = 
    {
        .charge_voltage_max_mv   = 4200u,
        .charge_current_max_ma   = 1000u,
        .advanced = {
            .input_voltage_max_mv    = 5000u,
            .input_voltage_min_mv    = 3600u,
            .input_current_max_ma    = 500u,
            .termination_current_ma  = 120u,
            .precharge_pct           = BQ25672_PRECHG_15PCT
        }
    },
    .usb = 
    {
        .enable_otg         = false,
        .voltage_mv         = 5000u,
        .current_max_ma     = 500u,
        .advanced = 
        {
            .disable_pfm    = false,
            .disable_ooa    = false
        }
    },
    .timer = 
    {
        .top_off_enable     = true,
        .trickle_enable     = true,
        .fast_charge_enable = true,
        .advanced = {
            .precharge_short  = false,
            .precharge_enable = true
        }
    },
    .watchdog =
    {
        .timeout = BQ25672_WDT_160S,
        .disable_on_expiry = true
    },
    .mppt = 
    {
        .enable_mppt        = false,
        .ratio              = BQ25672_MPPT_75P0,
        .advanced = 
        {
            .delay_ms       = 5000u,
            .interval_s     = 30u
        }
    },
    .ntc = 
    {
        .enable_monitor = true,
        .cold_thresh = BQ25672_NTC_NEG20C,
        .hot_thresh  = BQ25672_NTC_50C,
    },
    .jeita  = 
    {
        .vset_high      = BQ25672_JEITA_NO_CHANGE,
        .iseth_high     = BQ25672_ISET_NO_CHANGE,
        .isetc_cold     = BQ25672_ISET_NO_CHANGE
    }
};

#endif