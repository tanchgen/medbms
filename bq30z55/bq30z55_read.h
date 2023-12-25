/*
 * ltmreg_read.h
 *
 *  Created on: 27 янв. 2023 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LTMREG_READ_H_
#define LTMREG_READ_H_

#include "i2c.h"
#include "i2c/ltm4xxx.h"
#include "i2c/ltm4xxx_reg.h"

static inline struct i2c_io_register * ltmI2cAddrRead( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_ADDRESS;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutCmdRead( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_COMMAND;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

// ==============================================================================================

// ================================== INPUT_SEQUENCE ============================================
static inline struct i2c_io_register * ltmReadStatus( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_WORD;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutUvFault( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_UV_FAULT_LIMIT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutOvFault( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_OV_FAULT_LIMIT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadIoutPeak( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_IOUT_PEAK;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadVoutPeak( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_VOUT_PEAK;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadVin( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_READ_VIN;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadVout( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_READ_VOUT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadIout( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_READ_IOUT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadPout( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_READ_POUT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadT1( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_READ_TEMPERATURE_1;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadT2( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_READ_TEMPERATURE_2;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadSpec( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_MFR_SPECIFIC;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadStatT( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_TEMPERATURE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadStatInput( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_INPUT;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadStatIout( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_IOUT;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadStatVout( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_VOUT;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmReadStatCml( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_CML;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

//  {ReadingDataI2cDeviceState, LTM4XXX_3_ADDR, PMBUS_REG_READ_IIN, false, 2,  0,  0, true}
static inline struct i2c_io_register * ltm4700ReadIin( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_READ_IIN;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 0;
  reg->parsed = true;
  reg++;
  return reg;
}

//  {ReadingDataI2cDeviceState, LTM4677_4_ADDR, LTM46XX_REG_MFR_READ_IIN,  PEC_ENABLE, 2,  0,  0, true}
static inline struct i2c_io_register * ltm46xxReadIin( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = ReadingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM46XX_REG_MFR_READ_IIN;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0;
  reg->t_pending = 5;
  reg->parsed = true;
  reg++;
  return reg;
}


#endif /* LTMREG_READ_H_ */
