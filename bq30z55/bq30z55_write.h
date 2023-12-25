/*
 * ltmreg_write.h
 *
 *  Created on: 27 янв. 2023 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LTMREG_WRITE_H_
#define LTMREG_WRITE_H_

#include "i2c.h"
#include "i2c/ltm4xxx.h"
#include "i2c/ltm4xxx_reg.h"

static inline struct i2c_io_register * ltmClearPeaks( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_CLEAR_PEAKS;
  reg->transfer_pec = pec;
  reg->data_size = 0;
  reg->u32data = 0x00;
  reg->t_pending = 1;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmClearFaults( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_CLEAR_FAULTS;
  reg->transfer_pec = pec;
  reg->data_size = 0;
  reg->u32data = 0x00;
  reg->t_pending = 1;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmI2cAddr( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = 0xB4;
  reg->code = LTM4XXX_REG_MFR_ADDRESS;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = addr >> 1;
  reg->t_pending = 1;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmPage( struct i2c_io_register * reg, const uint8_t addr, uint8_t page, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_PAGE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = page;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmConfigAll( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_CONFIG_ALL;
  reg->transfer_pec = PEC_DISABLE;
  reg->data_size = 1;
//  assert_param( pec <= 1 );
  reg->u32data = 0x1b | (pec << 2);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_CONFIG_ALL;
  reg->transfer_pec = PEC_ENABLE;
  reg->data_size = 1;
//  assert_param( pec <= 1 );
  reg->u32data = 0x1b | (pec << 2);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

// For External_Frequency
static inline struct i2c_io_register * ltmConfigAll_2( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_CONFIG_ALL;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x19; //0x5f;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmChainConfig( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_CHAN_CONFIG;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x1b;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmPwmMode( struct i2c_io_register * reg, const uint8_t addr, uint8_t mode, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_PWM_MODE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = mode;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmPwmConfig( struct i2c_io_register * reg, const uint8_t addr, const uint8_t cfg, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_PWM_CONFIG;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = cfg;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmPwmComp( struct i2c_io_register * reg, const uint8_t addr, uint8_t data ,const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4700_REG_MFR_PWM_COMP;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = data; //0xf5;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmFreqSwitch( struct i2c_io_register * reg, const uint8_t addr, uint16_t freq, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_FREQUENCY_SWITCH;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = toLinear11( freq, 1 );
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltm46xxIinOffset350( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  /* IIN_OFFSET 24.4 рекомендовано для 350 кГц */
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM46XX_REG_MFR_IIN_OFFSET;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0x8b20;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltm46xxIinOffset425( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  /* IIN_OFFSET 24.4 рекомендовано для 350 кГц */
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM46XX_REG_MFR_IIN_OFFSET;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0x8b82;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltm46xxIinOffset500( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  /* IIN_OFFSET 30.5 рекомендовано для 500 кГц */
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM46XX_REG_MFR_IIN_OFFSET;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0x8be7;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltm46xxIinOffset1000( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  /* IIN_OFFSET 30.5 рекомендовано для 500 кГц */
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM46XX_REG_MFR_IIN_OFFSET;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0x9315;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

// OV_FAULT_LIMIT - 13.8V
static inline struct i2c_io_register * ltmVinOvFaultLim( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VIN_OV_FAULT_LIMIT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0xd373;     // 13.8 V
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltm4700IinCalGain( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM47XX_REG_MFR_IIN_CAL_GAIN;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0xC200;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmOnOffConfig( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_ON_OFF_CONFIG;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x16;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmGpioPropagate( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = LTM4XXX_REG_MFR_GPIO_PROPAGATE;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = 0x6997;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutOvFaultResp( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_OV_FAULT_RESPONSE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x80;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutUvFaultResp( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_UV_FAULT_RESPONSE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x80;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmIoutOcFaultResp( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_IOUT_OC_FAULT_RESPONSE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0xC0;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmOtFaultResp( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_OT_FAULT_RESPONSE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x80;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmUtFaultResp( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_UT_FAULT_RESPONSE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x00;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVinOvFaultResp( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VIN_OV_FAULT_RESPONSE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x80;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmTonMaxFaultResp( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_TON_MAX_FAULT_RESPONSE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x80;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

// VOUT
static inline struct i2c_io_register * ltmVoutMax( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_MAX;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_MAX(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutMarginHi( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_MARGIN_HIGH;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_MARGIN_HIGH(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutMarginLow( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_MARGIN_LOW;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_MARGIN_LOW(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutCommand( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_COMMAND;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_COMMAND(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutOvWarnLim( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_OV_WARN_LIMIT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_OV_LIMIT(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutOvFaultLim( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_OV_FAULT_LIMIT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_OV_LIMIT(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutUvWarnLim( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_UV_WARN_LIMIT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_UV_LIMIT(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmVoutUvFaultLim( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_VOUT_UV_FAULT_LIMIT;
  reg->transfer_pec = pec;
  reg->data_size = 2;
  reg->u32data = VOUT_UV_LIMIT(vout);
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmRstPeaks( struct i2c_io_register * reg, const eI2cDev idx, const bool pec){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
  const uint8_t page = ltmPmbusReg[idx].page;

  reg = ltmPage( reg, addr, page, pec );
  reg = ltmClearPeaks( reg, addr, pec );
  return reg;
}

static inline struct i2c_io_register * ltmRstFaults( struct i2c_io_register * reg,  const eI2cDev idx, const bool pec ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;

  const uint8_t page = ltmPmbusReg[idx].page;

  reg = ltmPage( reg, addr, page, pec );
  reg = ltmClearFaults( reg, addr, pec );

  return reg;
}

static inline struct i2c_io_register * ltmOnOffCmd( struct i2c_io_register * reg, const eI2cDev idx, const bool pec ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;

  return ltmClearPeaks( reg, addr, pec );
}

static inline struct i2c_io_register * ltmWriteStatus( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_BYTE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x80;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmWriteSpec( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_MFR_SPECIFIC;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x82;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmWriteStatT( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_TEMPERATURE;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x90;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmWriteStatInput( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_INPUT;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x88;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmWriteStatIout( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_IOUT;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0xc0;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmWriteStatVout( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_VOUT;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0x94;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

static inline struct i2c_io_register * ltmWriteStatCml( struct i2c_io_register * reg, const uint8_t addr, const bool pec ) {
  reg->state = WritingDataI2cDeviceState;
  reg->i2c_address = addr;
  reg->code = PMBUS_REG_STATUS_CML;
  reg->transfer_pec = pec;
  reg->data_size = 1;
  reg->u32data = 0xFF;
  reg->t_pending = 0;
  reg->parsed = false;
  reg++;
  return reg;
}

#endif /* LTMREG_WRITE_H_ */
