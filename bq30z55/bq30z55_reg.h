/*
 * bq30z55_reg.h
 *
 *  Created on: 27 янв. 2023 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

// "Texas Instruments BQ30z55 chip"

#ifndef BQ30Z55_REG_H_
#define BQ30Z55_REG_H_

/**
  * @defgroup Регистры BQ30Z55.
  * @{
  */

/*
 * Smart Battery Data Specification 1.1 commands list
 * This is a list taken directly from specification.
 */

// BQ30Z55 SBS_CMD

typedef enum {
  MFR_ACCESS  =  0x00,              // ManufacturerAccess
  REMAIN_CAP_ALRM  =  0x01,         // RemainingCapacityAlarm
  REMAIN_TIME_ALRM  =  0x02,        // RemainingTimeAlarm
  BATT_MODE  =  0x03,               // BatteryMode
  AT_RATE  =  0x04,                 // AtRate
  AT_RATE_TO_FILL  =  0x05,         // AtRateToFull
  AT_RATE_TO_EMPTY  =  0x06,        // AtRateToEmpty
  AT_RATE_OK  =  0x07,              // AtRateOK
  TEMPERATURE  =  0x08,             // Temperature
  VOLT  =  0x09,                    // Voltage
  CURR  =  0x0a,                    // Current
  AVG_CURR  =  0x0b,                // AverageCurrent
  MAX_ERR  =  0x0c,                 // MaxError
  RELAT_STATE_OF_CHARGE  =  0x0d,   // RelativeStateOfCharge
  ABS_STATE_OF_CHARGE  =  0x0e,     // AbsoluteStateOfCharge
  REMAIN_CAP  =  0x0f,              // RemainingCapacity
  FULL_CHARGE_CAP  =  0x10,         // FullChargeCapacity
  RUN_TIME_TO_EMPTY  =  0x11,       // RunTimeToEmpty
  AVG_TIME_TO_EMPTY  =  0x12,       // AverageTimeToEmpty
  AVG_TIME_TO_FULL  =  0x13,        // AverageTimeToFull
  CHARGE_CURR  =  0x14,             // ChargingCurrent
  CHARG_VOLT  =  0x15,              // ChargingVoltage
  BATT_STATUS  =  0x16,             // BatteryStatus
  CYCLE_COUNT  =  0x17,             // CycleCount
  DESIGN_CAP  =  0x18,              // DesignCapacity
  DESIGN_VOLT  =  0x19,             // DesignVoltage
  SPEC_INFO  =  0x1a,               // SpecificationInfo
  MFR_DATE  =  0x1b,                // ManufactureDate
  SERIAL_NUM  =  0x1c,              // SerialNumber
  MFR_NAME  =  0x20,                // ManufacturerName
  DEVICE_NAME  =  0x21,             // DeviceName
  DEVICE_CHEMISTRY  =  0x22,        // DeviceChemistry
  MFR_DATA  =  0x23,                // ManufacturerData
  OPT_MFR_FUNC5  =  0x2f,           // OptionalMfgFunction5
  OPT_MFR_FUNC4  =  0x3c,           // OptionalMfgFunction4
  OPT_MFR_FUNC3  =  0x3d,           // OptionalMfgFunction3
  OPT_MFR_FUNC2  =  0x3e,           // OptionalMfgFunction2
  OPT_MFR_FUNC1  =  0x3f            // OptionalMfgFunction1
} eSbsCmd;


// Address spaces used in BQ family SBS chips
typedef enum {
  DATA_FLASH    = 0,
  INSTR_FLASH   = 1,
} eRawAddrSpace;


// ManufacturerAccess sub-commands used in all BQ family SBS chips
typedef enum {
  MFG_DATA = 0x00,          // ManufacturerData    = 0x00
  DEVICE_TYPE = 0x01,       // DeviceType        = 0x01
  FW_VERSION = 0x02,        // FirmwareVersion     = 0x02
  HW_VERSION = 0x03,        // HardwareVersion     = 0x03
} eMfrAccess_Cmd;


// Flags used in BatteryMode command
typedef enum {
  INT_CHARGE_CTRL = 0,        // INTERNAL_CHARGE_CONTROLLER  = 0
  PRIM_BATT_SUPP = 1,         // PRIMARY_BATTERY_SUPPORT   = 1
  RESERVED2 = 2,              // RESERVED2 = 2
  RESERVED3 = 3,              // RESERVED3 = 3
  RESERVED4 = 4,              // RESERVED4 = 4
  RESERVED5 = 5,              // RESERVED5 = 5
  RESERVED6 = 6,              // RESERVED6 = 6
  OCND_FLAG = 7,              // CONDITION_FLAG = 7
  CHARGE_CTRL_EN = 8,         // CHARGE_CONTROLLER_ENABLED = 8
  PRIM_BATT = 9,              // PRIMARY_BATTERY       = 9
  RESERVED10 = 10,            // RESERVED10 = 10
  RESERVED11 = 11,            // RESERVED11 = 11
  RESERVED12 = 12,            // RESERVED12 = 12
  ALRM_MODE = 13,             // ALARM_MODE = 13
  CHARGER_MODE = 14,          // CHARGER_MODE = 14
  CAPACITY_MODE = 15
} eSbsFlagBatt;

// Flags used in BatteryStatus command
typedef enum {
  ERROR_CODE = 0,
  FULLY_DISCHARGED = 4,
  FULLY_CHARGED = 5,
  DISCHARGING = 6,
  INITIALIZED = 7,
  REMAINING_TIME_ALRM = 8,
  REMAINING_CAPACITY_ALRM = 9,
  RESERVED10 = 10,
  TERMINATE_DISCHARGE_ALRM = 11,
  OT_ALRM = 12,
  RESERVED13 = 13,
  TERMINATE_CHARGE_ALRM = 14,
  OVER_CHARGED_ALARM = 15,
} eSbsBattStatus;

// Flags used in SpecificationInfo command
typedef enum {
  REVISION = 0,
  VERSION = 4,
  VSCALE = 8,
  IPSCALE = 12
} eSbsSpecInfo;

// List of groups of commands/offsets.
typedef enum {
  DEV_INFO = 0x00,
  USE_INFO = 0x01,
  COMPUTED_INFO = 0x02,
  STATUSBITS = 0x03,
  AT_RATES = 0x04,
  BQ_STATUS_BIT = 0x06,
  BQ_STATUS_BIT_MA = 0x07,
  BQ_CELL_VOLT = 0x08,
  BQ_LIFE_TIME_DATA = 0x09,
  BQ_LIFE_TIME_DATA_MA = 0x0a,
  IMPEDANCE_TRACK = 0x0b,
  IMPEDANCE_TRACK_MA = 0x0c,
  BQ_TURBO_MODE = 0x0f,
} eMonGroup;




#define BQ30Z55_REG_MFR_VOUT_MAX        0xA5
#define BQ30Z55_REG_MFR_PIN_ACCURACY    0xAC  /**<  A NVM word available for the user. */

#define BQ30Z55_REG_USER_DATA_03    (0xB3)  /**<  A NVM word available for the user. */
#define BQ30Z55_REG_USER_DATA_04    (0xB4)  /**<  A NVM word available for the user. */

#define BQ30Z55_REG_MFR_CHAN_CONFIG       (0xD0)  /**< Channel-specific configuration bits. */
#define BQ30Z55_REG_MFR_CONFIG_ALL        (0xD1)  /**< Configuration bits that are common to all pages. */
#define BQ30Z55_REG_MFR_GPIO_PROPAGATE    (0xD2)  /**< Configuration that determines which faults are propagated to the GPIO pins. */
#define LTM4700_REG_MFR_PWM_COMP          (0xD3)
#define BQ30Z55_REG_MFR_PWM_MODE          (0xD4)  /**< Configuration for the PWM engine of each VOUT channel. */
#define LTM46XX_REG_MFR_GPIO_RESPONSE     (0xD5)  /**< Action to be taken by the device when the GPIO pin is asserted low. */
#define LTM47XX_REG_MFR_FAULT_RESPONSE    (0xD5)  /**< Action to be taken by the device when the FAULT pin is asserted low. */
#define BQ30Z55_REG_MFR_OT_FAULT_RESPONSE (0xD6)  /**< Action to be taken by the device when OT is asserted low. */
#define BQ30Z55_REG_MFR_IOUT_PEAK         (0xD7)
#define BQ30Z55_REG_MFR_ADC_CONTROL       (0xD8)
#define BQ30Z55_REG_MFR_RETRY_DELAY       (0xDB)
#define BQ30Z55_REG_MFR_RESTART_DELAY     (0xDC)
#define BQ30Z55_REG_MFR_VOUT_PEAK         (0xDD)

#define BQ30Z55_REG_MFR_CLEAR_PEAKS      0xE3  /** < Clears all peak values.*/
#define BQ30Z55_REG_MFR_PADS             0xE5
#define BQ30Z55_REG_MFR_SPECIAL_ID      (0xE7)  /**< Manufacturer code representing IC silicon and revision. */
#define LTM47XX_REG_MFR_IIN_CAL_GAIN    (0xE8) /** < The resistance value of the input current sense element in mΩ. */
#define LTM46XX_REG_MFR_IIN_OFFSET       0xE9  /** < Coefficient used in calculations of READ_IIN and MFR_READ_IINn. */
#define BQ30Z55_REG_MFR_FAULT_LOG_CLEAR  0xEC
#define LTM46XX_REG_MFR_READ_IIN        (0xED)  /**< Measured input current per channel. */
#define BQ30Z55_REG_MFR_ADDRESS          0xE6   /**< Specify right-justified 7-bit device address. */

#define BQ30Z55_REG_MFR_COMMON           0xEF

#define BQ30Z55_REG_MFR_PWM_CONFIG          0xF5  /**< Set numerous parameters for the DC/DC controller including phasing. */
#define BQ30Z55_REG_MFR_IOUT_CAL_GAIN_TC   0xF6
#define BQ30Z55_REG_MFR_RVIN               0xF7  /**< The resistance value of the VIN pin filter element in mΩ. */
#define BQ30Z55_REG_MFR_TEMP_1_GAIN        0xF8
#define BQ30Z55_REG_MFR_TEMP_1_OFFSET      0xF9
#define BQ30Z55_REG_MFR_RAIL_ADDRESS       0xFA
#define BQ30Z55_REG_MFR_RESET              0xFD


#define VOUT_MAX(vout)   ((uint16_t)(((((vout * 100L) + (vout * 10L)) / 100L) * 4096L) / 1000))
#define VOUT_MARGIN_HIGH(vout)  ((uint16_t)(((((vout * 100L) + (vout * 5L)) / 100L) * 4096L) / 1000))
#define VOUT_MARGIN_LOW(vout)   ((uint16_t)(((((vout * 100L) - (vout * 5L)) / 100L) * 4096L) / 1000))
#define VOUT_COMMAND(vout)    ((uint16_t)((vout * 4096L) / 1000))
#define VOUT_OV_LIMIT(vout)   ((uint16_t)(((((vout * 100L) + (vout * 9L)) / 100L) * 4096L) / 1000))
#define VOUT_UV_LIMIT(vout)   ((uint16_t)(((((vout * 100L) - (vout * 9L)) / 100L) * 4096L) / 1000))
#define PGOOD_ON(vout)        ((uint16_t)(((((vout * 100L) - (vout * 6L)) / 100L) * 4096L) / 1000))
#define PGOOD_OFF(vout)       ((uint16_t)(((((vout * 100L) - (vout * 7L)) / 100L) * 4096L) / 1000))

typedef struct {
  uint8_t i2cAddr;
  uint8_t page;
  uint16_t vout;
  float ioutMin;
  float ioutMax;
} sLtmPmbusReg;

extern const sBq30PmbusReg bq30PmbusReg;

#endif /* BQ30Z55_REG_H_ */
