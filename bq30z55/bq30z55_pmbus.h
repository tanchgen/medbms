/*
 * ltm4xxx_pmbus.h
 *
 *  Created on: 4 окт. 2022 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#ifndef LTM4XXX_PMBUS_H_
#define LTM4XXX_PMBUS_H_

#include "bq30z55.h"
#include "pmbus.h"

#define LTM_RST_SEQUENCE_LEN      28

#define LTM46XX_IN_SEQUENCE_PATTERN(addr, pec)                  \
    LTM4XXX_IN_SEQUENCE_PATTERN((addr), (pec)),                  \
    {WritingDataI2cDeviceState, LTM4677_4_ADDR, PMBUS_REG_VOUT_OV_FAULT_RESPONSE, true, 1, 0x80, 0, false}

#define LTM_PEC             PEC_ENABLE
#define LTM_PWM_CONFIG      0x00
#define LTM_PWM_COMP        0xf9

#include "i2c/ltmreg_write.h"
#include "i2c/ltmreg_read.h"
// ===================================================================================================

// ================================== RESET_SEQUENCE ============================================
static inline struct i2c_io_register * ltm4xxxRstSeq( struct i2c_io_register * reg, const uint8_t addr, const uint16_t vout, const bool pec ){
  reg = ltmOnOffConfig( reg, addr, pec );          // 1
  reg = ltmGpioPropagate( reg, addr, pec );        // 2
  reg = ltmVoutOvFaultResp( reg, addr, pec );      // 3
  reg = ltmVoutUvFaultResp( reg, addr, pec );      // 4
  reg = ltmIoutOcFaultResp( reg, addr, pec );      // 5
  reg = ltmOtFaultResp( reg, addr, pec );          // 6
  reg = ltmUtFaultResp( reg, addr, pec );          // 7
  reg = ltmVinOvFaultResp( reg, addr, pec );       // 8
  reg = ltmTonMaxFaultResp( reg, addr, pec );      // 9
  // VOUT
  reg = ltmVoutMax( reg, addr, vout, pec );        // 10
  reg = ltmVoutMarginHi( reg, addr, vout, pec );   // 11
  reg = ltmVoutMarginLow( reg, addr, vout, pec );  // 12
  reg = ltmVoutCommand( reg, addr, vout, pec );    // 13
  reg = ltmVoutOvWarnLim( reg, addr, vout, pec );  // 14
  reg = ltmVoutOvFaultLim( reg, addr, vout, pec ); // 15
  reg = ltmVoutUvWarnLim( reg, addr, vout, pec );  // 16
  reg = ltmVoutUvFaultLim( reg, addr, vout, pec ); // 17
  return reg;
}

static inline struct i2c_io_register * ltm4700_0RstSeq( struct i2c_io_register * reg, eI2cDev idx, const bool pec ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
  const uint8_t page = ltmPmbusReg[idx].page;
  const uint16_t vout = ltmPmbusReg[idx].vout;

//  assert_param( (idx == I2C_LTM_DD1_0V85_0) || (idx == I2C_LTM_DD1_0V85_1)
//                || (idx == I2C_LTM_DD2_0V85_0) || (idx == I2C_LTM_DD2_0V85_1)
//                || (idx == I2C_LTM_DD3_0V85_0) || (idx <= I2C_LTM_DD3_0V85_1) );
  reg = ltmI2cAddr( reg, addr, pec );            // 1
//  reg = ltmI2cAddrRead( reg, pec );        // 1
  reg = ltmPage( reg, addr, page, pec );           // 2
  reg = ltmClearPeaks( reg, addr, pec );           // 3
  reg = ltmClearFaults( reg, addr, pec );          // 4
  reg = ltmConfigAll( reg, addr, pec );            // 5
  reg = ltmChainConfig( reg, addr, pec );          // 6
  reg = ltmPwmMode( reg, addr, 0xc6, pec );        // 7
  reg = ltmPwmConfig( reg, addr, 0x30, pec );      // 8
  reg = ltmPwmComp( reg, addr, LTM_PWM_COMP, pec );     // 9
  reg = ltmFreqSwitch( reg, addr, 425, pec );      // 10
  reg = ltmVinOvFaultLim( reg, addr, pec );        // 11
  reg = ltm4700IinCalGain( reg, addr, pec );       // 12

  reg = ltm4xxxRstSeq( reg, addr, vout, pec );    //17 регистров
  return reg;
}


static inline struct i2c_io_register * ltm4700_1RstSeq( struct i2c_io_register * reg, eI2cDev idx, const bool pec ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
  const uint8_t page = ltmPmbusReg[idx].page;
  const uint16_t vout = ltmPmbusReg[idx].vout;

//  assert_param( (idx == I2C_LTM_DD1_0V85_2) || (idx == I2C_LTM_DD1_0V85_3)
//                || (idx == I2C_LTM_DD2_0V85_2) || (idx == I2C_LTM_DD2_0V85_3)
//                || (idx == I2C_LTM_DD3_0V85_2) || (idx <= I2C_LTM_DD3_0V85_3) );
  reg = ltmI2cAddr( reg, addr, pec );            // 1
//  reg = ltmI2cAddrRead( reg, pec );                // 1
  reg = ltmPage( reg, addr, page, pec );           // 2
  reg = ltmClearPeaks( reg, addr, pec );           // 3
  reg = ltmClearFaults( reg, addr, pec );          // 4
  reg = ltmConfigAll_2( reg, addr, pec );          // 5
  reg = ltmChainConfig( reg, addr, pec );          // 6
  reg = ltmPwmMode( reg, addr, 0xc6, pec );        // 7
  reg = ltmPwmConfig( reg, addr, 0x31, pec );      // 8
  reg = ltmPwmComp( reg, addr, LTM_PWM_COMP, pec );     // 9
  reg = ltmFreqSwitch( reg, addr, 425, pec );      // 10
  reg = ltmVinOvFaultLim( reg, addr, pec );        // 11
  reg = ltm4700IinCalGain( reg, addr, pec );       // 12

  reg = ltm4xxxRstSeq( reg, addr, vout, pec );     // 17 регистров
  return reg;
}

//static inline struct i2c_io_register * ltm0v9RstSeq( struct i2c_io_register * reg, eI2cDev idx ){
//  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
//  const uint8_t page = ltmPmbusReg[idx].page;
//  const uint16_t vout = ltmPmbusReg[idx].vout;
//
//  if( page == 0 ){
//    reg = ltmI2cAddr( reg, addr );        // 1
//  }
////  reg = ltmI2cAddrRead( reg );        // 1
//  reg = ltmPage( reg, addr, page );           // 1
//  reg = ltmClearPeaks( reg, addr );           // 2
//  reg = ltmClearFaults( reg, addr );          // 3
//  reg = ltmConfigAll( reg, addr );            // 4
//  reg = ltmChainConfig( reg, addr );          // 5
//  reg = ltmPwmMode( reg, addr, 0xc3 );        // 6
//  reg = ltmPwmConfig( reg, addr, 0x00 );      // 7
//  reg = ltmFreqSwitch( reg, addr, 500 );      // 9
//  reg = ltm46xxIinOffset500( reg, addr );     // 10
//  reg = ltmVinOvFaultLim( reg, addr );        // 11
//
//  reg = ltm4xxxRstSeq( reg, addr, vout );    //17 регистров
//  return reg;
//}

static inline struct i2c_io_register * ltm46xxRstSeqNull( struct i2c_io_register * reg, eI2cDev idx ){
  (void)idx;
  return reg;
}

static inline struct i2c_io_register * ltm46xxInSeqNull( struct i2c_io_register * reg, eI2cDev idx ){
  (void)idx;
  return reg;
}

static inline struct i2c_io_register * ltm4678RstSeq( struct i2c_io_register * reg, eI2cDev idx ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
  const uint8_t page = ltmPmbusReg[idx].page;
  const uint16_t vout = ltmPmbusReg[idx].vout;
  const bool pec = LTM_PEC;
  uint8_t pmode;

//  reg = ltmI2cAddr( reg, addr, pec );           // 1
  reg = ltmConfigAll( reg, addr, pec );          // 4
  reg = ltmClearFaults( reg, addr, pec );       // 2
  reg = ltmClearPeaks( reg, addr, pec );        // 3
  reg = ltmPage( reg, addr, page, pec );             // 5
  reg = ltmChainConfig( reg, addr, pec );        // 6
//  pmode = ((idx == I2C_LTM_DD1_0V9) || (idx == I2C_LTM_DD2_0V9) || (idx == I2C_LTM_DD3_0V9) )? 0xc3 : 0xc2;
  pmode = 0xc0;
  reg = ltmPwmMode( reg, addr, pmode, pec );     // 7
  reg = ltmPwmConfig( reg, addr, LTM_PWM_CONFIG, pec );    // 8
  reg = ltmPwmComp( reg, addr, LTM_PWM_COMP, pec );     // 9
  reg = ltmFreqSwitch( reg, addr, 350, pec );    // 10
  reg = ltmVinOvFaultLim( reg, addr, pec );      // 11

  reg = ltm4xxxRstSeq( reg, addr, vout, pec );   //17 регистров
  return reg;
}

static inline struct i2c_io_register * ltm46xxRstSeq( struct i2c_io_register * reg, eI2cDev idx ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
  const uint8_t page = ltmPmbusReg[idx].page;
  const uint16_t vout = ltmPmbusReg[idx].vout;
  const bool pec = LTM_PEC;
  uint8_t pmode;

//  reg = ltmI2cAddr( reg, addr, pec );         // 1
  reg = ltmConfigAll( reg, addr, pec );         // 4
  reg = ltmClearFaults( reg, addr, pec );       // 2
  reg = ltmClearPeaks( reg, addr, pec );        // 3
  reg = ltmPage( reg, addr, page, pec );        // 5
  reg = ltmChainConfig( reg, addr, pec );       // 6
//  pmode = ((idx == I2C_LTM_DD1_0V9) || (idx == I2C_LTM_DD2_0V9) || (idx == I2C_LTM_DD3_0V9) )? 0xc3 : 0xc2;
  pmode = 0xc0;
  reg = ltmPwmMode( reg, addr, pmode, pec );              // 7
  reg = ltmPwmConfig( reg, addr, LTM_PWM_CONFIG, pec );   // 8
  reg = ltmFreqSwitch( reg, addr, 350, pec );             // 9
  reg = ltmVinOvFaultLim( reg, addr, pec );               // 10

  reg = ltm4xxxRstSeq( reg, addr, vout, pec );            //17 регистров
  return reg;
}


static inline struct i2c_io_register * ltm46xxPeakRstSeq( struct i2c_io_register * reg, eI2cDev idx ){
  const bool pec = LTM_PEC;

  reg = ltmRstPeaks( reg, idx, pec );
  return reg;
}
// ===================================================================================================================

// ================================== INPUT_SEQUENCE ============================================
static inline struct i2c_io_register * ltm4xxxInSeq( struct i2c_io_register * reg, const uint8_t addr, const uint8_t page, const bool pec ){
  reg = ltmPage( reg, addr, page, pec );          // 1
  reg = ltmReadStatus( reg, addr, PEC_DISABLE );     // 2
//  reg = ltmWriteStatus( reg, addr, pec );         // 3
  reg = ltmReadSpec( reg, addr, PEC_DISABLE );       // 4
//  reg = ltmWriteSpec( reg, addr, pec );           // 5
  reg = ltmReadStatT( reg, addr, PEC_DISABLE );      // 6
//  reg = ltmWriteStatT( reg, addr, pec );          // 7
  reg = ltmReadStatInput( reg, addr, PEC_DISABLE );  // 8
//  reg = ltmWriteStatInput( reg, addr, pec );      // 9
  reg = ltmReadStatIout( reg, addr, PEC_DISABLE );   // 10
//  reg = ltmWriteStatIout( reg, addr, pec );       // 11
  reg = ltmReadStatVout( reg, addr, PEC_DISABLE );   // 12
//  reg = ltmWriteStatVout( reg, addr, pec );       // 13
  reg = ltmVoutOvFault( reg, addr, PEC_DISABLE );    // 14
  reg = ltmVoutUvFault( reg, addr, PEC_DISABLE );    // 15
  reg = ltmReadIoutPeak( reg, addr, PEC_DISABLE );        // 18
  reg = ltmReadVoutPeak( reg, addr, PEC_DISABLE );        // 19
  reg = ltmReadVin( reg, addr, PEC_DISABLE );             // 20
  reg = ltmReadVout( reg, addr, PEC_DISABLE );            // 21
  reg = ltmReadIout( reg, addr, PEC_DISABLE );            // 22
  reg = ltmReadPout( reg, addr, PEC_DISABLE );            // 23
  reg = ltmReadT1( reg, addr, PEC_DISABLE );              // 24
  reg = ltmReadT2( reg, addr, PEC_DISABLE );              // 25
  reg = ltmReadStatCml( reg, addr, PEC_DISABLE );    // 16
//  reg = ltmWriteStatCml( reg, addr, pec );        // 17
  reg = ltmClearFaults( reg, addr, pec );       // 2
  return reg;
}

//static inline struct i2c_io_register * ltm4677InSeq( struct i2c_io_register * reg, const uint8_t addr, const uint8_t page, const bool pec ){
//  reg = ltmPage( reg, addr, page );          // 1
//  reg = ltmReadStatus( reg, addr, pec );     // 2
//  reg = ltmWriteStatus( reg, addr );         // 3
//  reg = ltmReadSpec( reg, addr, pec );       // 4
//  reg = ltmWriteSpec( reg, addr );           // 5
//  reg = ltmReadStatT( reg, addr, pec );      // 6
//  reg = ltmWriteStatT( reg, addr );          // 7
//  reg = ltmReadStatInput( reg, addr, pec );  // 8
//  reg = ltmWriteStatInput( reg, addr );      // 9
//  reg = ltmReadStatIout( reg, addr, pec );   // 10
//  reg = ltmWriteStatIout( reg, addr );       // 11
//  reg = ltmReadStatVout( reg, addr, pec );   // 12
//  reg = ltmWriteStatVout( reg, addr );       // 13
//  reg = ltmReadStatCml( reg, addr, pec );    // 14
//  reg = ltmWriteStatCml( reg, addr );        // 15
//  reg = ltmVoutOvFault( reg, addr, pec );    // 16
//  reg = ltmVoutUvFault( reg, addr, pec );    // 17
//  reg = ltmReadIoutPeak( reg, addr, pec );        // 18
//  reg = ltmReadVoutPeak( reg, addr, pec );        // 19
//  reg = ltmReadVin( reg, addr, pec );             // 20
//  reg = ltmReadVout( reg, addr, pec );            // 21
//  reg = ltmReadIout( reg, addr, pec );            // 22
//  reg = ltmReadPout( reg, addr, pec );            // 23
//  reg = ltmReadT1( reg, addr, pec );              // 24
//  reg = ltmReadT2( reg, addr, pec );              // 25
//  return reg;
//}


static inline struct i2c_io_register * ltm4700InSeq( struct i2c_io_register * reg, eI2cDev idx ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
  const uint8_t page = ltmPmbusReg[idx].page;

  reg = ltm4xxxInSeq( reg, addr, page, LTM_PEC );     // 25 регистра
  reg = ltm4700ReadIin( reg, addr, LTM_PEC );                      // 26
  return reg;
}

static inline struct i2c_io_register * ltm46xxInSeq( struct i2c_io_register * reg, eI2cDev idx ){
  const uint8_t addr = ltmPmbusReg[idx].i2cAddr;
  const uint8_t page = ltmPmbusReg[idx].page;

//  reg = ltmI2cAddrRead( reg, addr, LTM_PEC );   // 1
  reg = ltm4xxxInSeq( reg, addr, page, LTM_PEC );      // 25 регистра
  reg = ltm4700ReadIin( reg, addr, PEC_DISABLE );                      // 26
  return reg;
}


#endif /* LTM4XXX_PMBUS_H_ */
