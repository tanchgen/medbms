/*
 * ltmreg_write.h
 *
 *  Created on: 27 янв. 2023 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LTMREG_WRITE_H_
#define LTMREG_WRITE_H_

#include "i2c.h"
#include "bq30z55_smbus.h"
#include "bq30z55_reg.h"

static inline sI2cTrans * bq30zAuth( sI2cTrans * trans, uint8_t * data ) {
  trans->state = I2C_STATE_WRITE;
  trans->reg.cmd = MFR_ACCESS;
  trans->reg.subcmd1 = MFR_ACC_DATA;
//  trans->reg.subcmd2 = 0x00;
  trans->pec = BQ30Z55_PEC;
  trans->len = 512;
  trans->data = data;
  trans->tout = 250;
  trans->parsed = false;
  trans++;
  return trans;
}


//static inline sI2cTrans * bq30zAuth( sI2cTrans * trans, uint8_t * data, const bool pec ) {
//  trans->state = I2C_STATE_WRITE;
//  trans->reg.cmd = 0x00;
//  trans->reg.subcmd1 = 0x00;
//  trans->reg.subcmd2 = 0x01;
//  trans->pec = pec;
//  trans->len = 512;
//  trans->data = data;
//  trans->tout = 250;
//  trans->parsed = false;
//  trans++;
//  return trans;
//}



#endif /* LTMREG_WRITE_H_ */
