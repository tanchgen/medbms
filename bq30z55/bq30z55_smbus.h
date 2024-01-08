/*
 * bq30z55_pmbus.h
 *
 *  Created on: 4 окт. 2022 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef BQ30Z55_SMBUS_H_
#define BQ30Z55_SMBUS_H_

//#include "bq30z55.h"
#include "smbus.h"

#define BQ30Z55_ADDR      (0x6D << 1)
#define BQ30Z55_PEC       PEC_DISABLE

#include "bq30z55_write.h"
#include "bq30z55_read.h"


// ================================== AUTH SEQUENCE ============================================
static inline sI2cTrans * bq30zAuthSeq( sI2cTrans * trans, const bool pec ){
  uint8_t authArr[512] = {0};

  memcpy( authArr, bq30zIdKey, 16);
  bq30zRng( &(authArr[16]) );
  authArr[36] = 0x01;
  authArr[56] = 0x01;
  authArr[59] = 0x01;


  trans = bq30zAuth();
  return trans;
}
// ===================================================================================================

// ================================== RESET_SEQUENCE ============================================
static inline sI2cTrans * bq30zRstSeq( sI2cTrans * trans ){
  const bool pec = BQ30Z55_PEC;

//  reg = ltmRstPeaks( reg, idx, pec );
  return trans;
}
// ===================================================================================================================

// ================================== INPUT_SEQUENCE ============================================
static inline sI2cTrans * bq30zInSeq( sI2cTrans * trans ){
  // BattaryMode
  trans = bq30zBattMode( trans );          // 1
  // Temperature
  trans = bq30zT( trans );          // 1
  // Voltage
  trans = bq30zV( trans );          // 1
  // Current
  trans = bq30zI( trans );          // 1
  // AverageCurrent
  trans = bq30zAvgI( trans );          // 1
  // RemainingCapacity
  trans = bq30zRemainCap( trans );          // 1
  // FullChargeCapacity
  trans = bq30zFullChCap( trans );          // 1
  // RunTimeEmpty
  trans = bq30zRunTimeEmpty( trans );          // 1
  // AverageRunTimeEmpty
  trans = bq30zAvgRunTimeEmpty( trans );          // 1
  // ChargingCurrent
  trans = bq30zChI( trans );          // 1
  // ChargingVoltage
  trans = bq30zChV( trans );          // 1
  // BatteryStatus
  trans = bq30zBattStatus( trans );          // 1
  // CycleCount
  trans = bq30zCycleCount( trans );          // 1
  // DesignCapacity
  trans = bq30zDesCap( trans );          // 1
  // DesignVoltage
  trans = bq30zDesV( trans );          // 1
  // CellVoltage_3
  trans = bq30zCellV3( trans );          // 1
  // CellVoltage_2
  trans = bq30zCellV2( trans );          // 1
  // CellVoltage_1
  trans = bq30zCellV1( trans );          // 1
  // CellVoltage_0
  trans = bq30zCellV0( trans );          // 1
  // Voltage_0
  trans = bq30zV0( trans );          // 1
  // Temperature_0
  trans = bq30zT0( trans );          // 1

  return trans;
}

#endif /* LTM4XXX_PMBUS_H_ */
