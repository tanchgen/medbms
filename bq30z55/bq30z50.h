/*
 * bq30z50.h
 *
 *  Created on: 25 дек. 2023 г.
 *      Author: jet
 */

#ifndef BQ30Z50_H_
#define BQ30Z50_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f10x.h"

#include "bq30z55_reg.h"


#define BQ30Z55_CHIP_TYPE   0x010550

typedef struct {
   uint16_t inChargeCtrl: 1;
   uint16_t primBattSupp: 1;
   uint16_t reserv2: 1;
   uint16_t reserv3: 1;
   uint16_t reserv4: 1;
   uint16_t reserv5: 1;
   uint16_t reserv6: 1;
   uint16_t condFlag: 1;          // Condition Flag
   uint16_t chargeCtrlEn: 1;      // Battery pack's internal charge controller state
   uint16_t primBatt: 1;          // Primary battery
   uint16_t reserv10: 1;
   uint16_t reserv11: 1;
   uint16_t reserv12: 1;
   uint16_t alrmMode: 1;
   uint16_t capMode: 1;           // Capacity mode: 0 - Report in mAh", 1 - "Report in 10mWh
 } sBattStatusFlag;

 typedef enum {
   BATT_ERR_OK,
   BATT_ERR_BUSY,
   BATT_ERR_RESERV_CMD,
   BATT_ERR_UNSUPP_CMD,
   BATT_ERR_ACCESS,
   BATT_ERR_OF_UF,            // Over/Underflow
   BATT_ERR_SIZE,             // Bad Size
   BATT_ERR_UNKNOWN           // Unknown Error
 } eBattErr;

 typedef enum {
   BATT_INIT_CORRECT,
   BATT_INIT_RECALIBR
 } eBattInited;

 // SBS_BATTERY_STATUS_INFO
 typedef struct {
   eBattErr errCode: 4;
   uint16_t fullDischarged: 1;
   uint16_t fullCharged: 1;
   uint16_t Discharge;
   uint16_t battInited: 1;
   uint16_t remainTimeAlrm: 1;         // Remaining time to depletion alarm tripped.
   uint16_t remainCapAlrm: 1;          // Remaining capacity alarm tripped.
   uint16_t reserv10: 1;
   uint16_t treminDischargeAlrm: 1;
   uint16_t otAlrm: 1;
   uint16_t reserv13: 1;
   uint16_t terminChargeAlrm: 1;
   uint16_t overChargeAlrm: 1;

 } sBattStatusInfo;

 // SBS_SPECIFICATION_INFO
 typedef enum {
   SPEC_REV_V0,
   SPEC_REV_V1_0,
   SPEC_REV_V1_1,
   SPEC_REV_V1_1PEC,
   SPEC_REV_VF4,
   SPEC_REV_VF5,
   SPEC_REV_VF6,
   SPEC_REV_VF7,
 } eSpecInfoVers;

typedef struct {
  uint16_t revision: 4;
  eSpecInfoVers version: 4;
  uint16_t vScale: 4;         // Voltage scaling exponent. Multiplies voltages by 10 ^ VScale.
  uint16_t ipScale: 4;        // Current/capacity scaling exp. Multiplies currents and capacities by 10 ^ IPScale.
} sSbsSpecInfo;




#endif /* BQ30Z50_H_ */
