/*
 * bq30z55_pmbus.h
 *
 *  Created on: 4 окт. 2022 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef BQ30Z55_SMBUS_H_
#define BQ30Z55_SMBUS_H_

#include <string.h>

#include "tinyalloc.h"
#include "debug.h"
//#include "bq30z55.h"
#include "smbus.h"

#define BQ30Z55_ADDR      (0x5A << 1) //(0x6D << 1)
#define BQ30Z55_PEC       PEC_DISABLE

#include "bq30z55_write.h"
#include "bq30z55_read.h"

extern const uint8_t bq30zIdKey[16];

eErr bq30zRng( uint8_t * rngM );

// ================================== AUTH SEQUENCE ============================================
static inline sI2cTrans * bq30zAuthSeq( sI2cTrans * trans ){
  uint8_t authArr[512] = {0};
  const FlagStatus pec = BQ30Z55_PEC;

  memcpy( authArr, bq30zIdKey, 16);
  bq30zRng( &(authArr[16]) );
  authArr[36] = 0x01;
  authArr[56] = 0x01;
  authArr[59] = 0x01;


  trans = bq30zAuth( trans, authArr, pec );
  return trans;
}
// ===================================================================================================

// ================================== RESET_SEQUENCE ============================================
static inline sI2cTrans * mpr121CfgSeq( sI2cTrans * trans ){
  const bool pec = BQ30Z55_PEC;
  uint8_t * pdata;

  assert_param( (pdata = ta_alloc( 1 )) != 0 );
  *pdata = 0x51;        // FFI = 0x1, CDC = 0x11
  trans = mpr121afe1Write( trans, pdata, pec );
  assert_param( (pdata = ta_alloc( 1 )) != 0 );
  *pdata = 0x2B;        // CDT = 0x1, SFI = 0x1, ESI = 0x101
  trans = mpr121afe2Write( trans, pdata, pec );
  return trans;
}

static inline sI2cTrans * bq30zCfgSeq( sI2cTrans * trans ){
//  const bool pec = BQ30Z55_PEC;

//  reg = ltmRstPeaks( reg, idx, pec );
  return trans;
}
// ===================================================================================================================

// ================================== INPUT_SEQUENCE ============================================
static inline sI2cTrans * mpr121InSeq( sI2cTrans * trans ){
  uint8_t * pdata;

  assert_param( (pdata = ta_alloc( 1 )) != 0 );
  *pdata = 0;        // FFI = 0x1, CDC = 0x11
  trans = mpr121afe1( trans, pdata );
  assert_param( (pdata = ta_alloc( 1 )) != 0 );
  *pdata = 0;        // FFI = 0x1, CDC = 0x11
  trans = mpr121afe2( trans, pdata );
  return trans;
}

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
  trans = bq30zCellV3( trans );           // CellVoltage_3
  trans = bq30zCellV2( trans );           // CellVoltage_2
  trans = bq30zCellV1( trans );           // CellVoltage_1
  trans = bq30zCellV0( trans );           // CellVoltage_0
//  trans = bq30zV0( trans );               // Voltage_0
//  trans = bq30zT0( trans );               // Temperature_0

  return trans;
}

#endif /* LTM4XXX_PMBUS_H_ */
