/*
 * ltmreg_read.h
 *
 *  Created on: 27 янв. 2023 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LTMREG_READ_H_
#define LTMREG_READ_H_

#include "i2c.h"
#include "bq30z55.h"
#include "bq30z55_reg.h"

// ==============================================================================================

// ================================== INPUT_SEQUENCE ============================================
// BatteryMode
static inline sI2cTrans * bq30zBattMode( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = BATT_MODE;
  trans->regLen = 1;
  trans->pec = BQ30Z55_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Temperature
static inline sI2cTrans * bq30zT( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = TEMPERATURE;
  trans->regLen = 1;
  trans->pec = BQ30Z55_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Voltage
static inline sI2cTrans * bq30zV( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = VOLT;
  trans->regLen = 1;
  trans->pec = BQ30Z55_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// Current
static inline sI2cTrans * bq30zI( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = CURR;
  trans->regLen = 1;
  trans->pec = BQ30Z55_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// AverageCurrent
static inline sI2cTrans * bq30zAvgI( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = AVG_CURR;
  trans->regLen = 1;
  trans->pec = BQ30Z55_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

// RemainingCapacity
trans = bq30zRemainCap( trans );          // 1
static inline sI2cTrans * bq30zRemainCap( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = REMAIN_CAP;
  trans->regLen = 1;
  trans->pec = BQ30Z55_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}

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

static inline sI2cTrans * bq30zBattMode( sI2cTrans * trans ) {
  trans->state = I2C_STATE_READ;
  trans->reg.cmd = BATT_MODE;
  trans->regLen = 1;
  trans->pec = BQ30Z55_PEC;
  trans->len = 2;
  trans->data = 0;
  trans->tout = 0;
  trans->parsed = true;
  trans++;
  return trans;
}


#endif /* LTMREG_READ_H_ */
