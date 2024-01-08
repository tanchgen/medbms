/*
 * ltm4xxx.c
 *
 *  Created on: 18 февр. 2020 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */
#include <math.h>

#include "main.h"
#include "tm.h"

#include "arch_zynq.h"
#include "i2c_arch.h"
#include "i2cdev_idx.h"
#include "gpio_arch.h"
#include "i2c/ltm4xxx_pmbus.h"

#if CAN_ENABLE
#error "This file \"ltm4xxx.c\" NOT for CAN"
#endif //CAN_ENABLE

// При выключении системы выключается/не_выключается питание LTM4XXX
#define LTM_VIN_OFF         0

uint16_t tmpi2cdata;

const sLtmPmbusReg ltmPmbusReg[CONFIG_LTM4XXX_I2C_NUM_MAX] = {
  { 0x80, 0, 1000, -0.2, 50.0 },	// I2C_LTM_1V_0,
  { 0x80, 1, 1000, -0.2, 50.0 },	// I2C_LTM_1V_1,
};

const sDevState ltmDevState[CONFIG_LTM4XXX_I2C_NUM_MAX] = {
    {
      .pwrLine = PWR_LINE_VCC_0,
      .devId = DEVID_FPGA,        \
      /* .i2cRst = I2C_RST_1,  */ \
      .runnum = RUN_1,            \
    },
};

struct {
  uint16_t ioutMax;
  uint16_t voutMax;
  uint16_t voutMarHi;
  uint16_t voutMarLow;
  uint16_t voutCmd;
  uint16_t voutOvWarn;
  uint16_t voutUvWarn;
} voutx[2];

extern sI2cDevInfo i2cDevInfo[];

struct timer_list ltmOntimer;

// ======================== Протипы функций =======================================
void arch_addPartialPout(float partial_p_out, uint8_t src);
// ================================================================================

//// Расчет скользящего среднего
//void fMovAvg( float *avg, float pt ){
//  const float a = 2.0 / (1+10);
//
//  *avg = pt * a + (*avg * (1 - a));
//}
//

void bq3588PeakReset( void ) {
  sLtm4xxxDev  *ltm = (sLtm4xxxDev *)i2cDevInfo[idx].srcData;

  ltm->status.u32stat |= LTM4XXX_PEAK_DATA_NOK_STATUS_BIT;
  paramPeakReset( &ltm->vin );
  paramPeakReset( &ltm->vin );
  paramPeakReset( &ltm->iin );
  paramPeakReset( &ltm->pin );
  paramPeakReset( &ltm->vout );
  paramPeakReset( &ltm->iout );
  paramPeakReset( &ltm->pout );
  paramPeakReset( &ltm->tch );           // Tch LTM
  paramPeakReset( &ltm->tic );           // Tic LTM

  // Т.к. мы считываем
  if( i2cDevInfo[idx].devState.cfgFlag == true ){
    i2cDevInfo[idx].cfgSequence = ltm46xxPeakRstSeq;
    i2cDevInfo[idx].devState.cfgFlag = false;
    i2cDevInfo[idx].devState.cfgProbe = RESET;
  }

}

void ltm4xxxStatusReset( sLtm4xxxDev *ltmdev ){

  ltmdev->status.u32stat &= LTM4XXX_DATA_NOK_STATUS_BIT | LTM4XXX_PEAK_DATA_NOK_STATUS_BIT;
  paramStatusReset( &ltmdev->vin );
  paramStatusReset( &ltmdev->iin );
  paramStatusReset( &ltmdev->pin );
  paramStatusReset( &ltmdev->vout );
  paramStatusReset( &ltmdev->iout );
  paramStatusReset( &ltmdev->pout );
  paramStatusReset( &ltmdev->tch );
  paramStatusReset( &ltmdev->tic );
}


/**
  * @brief Обновляем флаги и статусы LTM4XXX в зависимости от состояния системы.
  *
  * @param[in]  idx Индекс устройства I2C
  * @param[in]  newstate Состояние системы
  *
  * @retval none
  */
void ltmI2cUpdate( eI2cDev idx, FlagStatus newstate ){
  sI2cDevInfo * i2cinfo = &(i2cDevInfo[idx]);
  sLtm4xxxDev  *ltmdev = (sLtm4xxxDev *)(i2cinfo->srcData);

#ifdef  USE_FULL_ASSERT
  assert_param( idx < I2C_DEV_NUM);
#else
  (void)idx;
#endif  //USE_FULL_ASSERT

  /* Сбрасываем биты аварий при переходе в рабочий режим. */
  ltmdev = i2cDevInfo[idx].srcData;

  if ( newstate ){
    ltm4xxxStatusReset( ltmdev );
    i2cinfo->devState.n_faults = 0;
//    i2cinfo->devState.isOn = SET;
    assert_param( i2cinfo->devState.pgood == RESET );
  }
  else {
    i2cRstDefConf( idx );
#if LTM_VIN_OFF
    i2cinfo->devState.pgood = RESET;
    i2cinfo->devState.isOn = RESET;
    ltmdev->status.u32stat |= LTM4XXX_DATA_NOK_STATUS_MASK | LTM4XXX_PEAK_DATA_NOK_STATUS_MASK;
    i2cinfo->devState.cfgFlag = RESET;  /**< Признак готовности конфигурации. */
#endif // LTM_VIN_OFF
  }
}


/**
  * @brief Логирование ошибок LTM4XXX.
  *
  * @param[in]  idx Индекс устройства I2C
  * @param[in]  status Статус LTM4XXX
  *
  * @retval none
  */
/*
void ltm4xxxLogger( uint8_t idx, uint32_t status  ){
  sLtm4xxxDev * ltm = (sLtm4xxxDev *)i2cDevInfo[idx].srcData;
  uLtm4xxxStatus st;
  uint16_t data;

  // Перебираем все возможные флаги аварий
  while( status ){
    eLogErrId errId;
    eParamId prm;

    st.u32stat = status;
    if( st.vinLow ){
      errId = LOG_ERR_PRM_ON_LOW;
      prm = PRM_VIN;
      st.vinLow = RESET;
      data = toLinear11(ltm->vin.now.param);
    }
    else if( st.vinHi ){
      errId = LOG_ERR_PRM_ON_HI;
      prm = PRM_VIN;
      st.vinHi = RESET;
      data = toLinear11(ltm->vin.now.param);
    }
    else if( st.voutOffLow ){
      errId = LOG_ERR_PRM_OFF_LOW;
      prm = PRM_VOUT;
      st.voutOffLow = RESET;
      data = toLinear11(ltm->vout.now.param);
    }
    else if( st.voutOffHi ){
      errId = LOG_ERR_PRM_OFF_HI;
      prm = PRM_VOUT;
      st.voutOffHi = RESET;
      data = toLinear11(ltm->vout.now.param);
    }
    else if( st.voutOnLow ){
      errId = LOG_ERR_PRM_ON_LOW;
      prm = PRM_VOUT;
      st.voutOnLow = RESET;
      data = toLinear11(ltm->vout.now.param);
    }
    else if( st.voutOnHi ){
      errId = LOG_ERR_PRM_ON_HI;
      prm = PRM_VOUT;
      st.voutOnHi = RESET;
      data = toLinear11(ltm->vout.now.param);
    }
    else if( st.ioutOnLow ){
      errId = LOG_ERR_PRM_ON_LOW;
      prm = PRM_IOUT;
      st.ioutOnLow = RESET;
      data = toLinear11(ltm->iout.now.param);
    }
    else if( st.ioutOnHi ){
      errId = LOG_ERR_PRM_ON_HI;
      prm = PRM_IOUT;
      st.ioutOnHi = RESET;
      data = toLinear11(ltm->iout.now.param);
    }
    else if( st.poutOnLow ){
      errId = LOG_ERR_PRM_ON_LOW;
      prm = PRM_POUT;
      st.poutOnLow = RESET;
      data = toLinear11(ltm->pout.now.param);
    }
    else if( st.poutOnHi ){
      errId = LOG_ERR_PRM_ON_HI;
      prm = PRM_POUT;
      st.poutOnHi = RESET;
      data = toLinear11(ltm->pout.now.param);
    }

    else if( st.tchLow ){
      errId = LOG_ERR_PRM_ON_LOW;
      prm = PRM_TCH;
      st.tchLow = RESET;
      data = toLinear11(ltm->tch.now.param);
    }
    else if( st.tchHi ){
      errId = LOG_ERR_PRM_ON_HI;
      prm = PRM_TCH;
      st.tchHi = RESET;
      data = toLinear11(ltm->tch.now.param);
    }
    else if( st.ticLow ){
      errId = LOG_ERR_PRM_ON_LOW;
      prm = PRM_TIC;
      st.ticLow = RESET;
      data = toLinear11(ltm->tic.now.param);
    }
    else if( st.ticHi ){
      errId = LOG_ERR_PRM_ON_HI;
      prm = PRM_TIC;
      st.ticHi = RESET;
      data = toLinear11(ltm->tic.now.param);
    }
    else if( st.turnOff ){
      errId = LOG_ERR_FLAG;
      prm = PRM_PWR_OTHER;
      data = st.u32stat & LTM4XXX_ALARM_STATUS_MASK;
      st.turnOff = RESET;
    }
    else {
      // Проверили все флаги, которые нужны
      break;
    }

    logger( DEVID_LTM, idx - LTM4XXX_0_IDX, errId, prm, data );
    status = st.u32stat;
  }
}
*/


/**
  * @brief Определяет необходимое действие на выставленные флаги статуса LTM4676.
  *
  * @param[in]  idx Индекс устройства I2C
  * @param[in]  status Статус устройства
  *
  * @retval Требуемое действие
  */
static eStatusAct ltm4xxxStatusAction( sLtm4xxxDev * ltm, uint32_t st ){
  (void)ltm;
  uLtm4xxxStatus status;
  eStatusAct stAct;

 // Реакция на флаг аварии
  status.u16stat1 = 0;
  status.u16stat2 = 0;
  status.u32stat = st;

  if(status.vinHi || status.voutOnHi || status.ioutOnHi || status.poutOnHi || status.turnOff ){
    stAct = ACT_PWR_EMRG_OFF;
  }
  else if( status.tchLow || status.tchHi|| status.ticLow || status.ticHi ){
   stAct = ACT_PWR_FAIL_OFF;
  }
  else if( status.vinLow || status.voutOnLow ){
    stAct = ACT_PWR_OFF;
  }
  else if( status.voutOffLow || status.voutOffHi|| status.poutOnLow ){
    stAct = ACT_HW_LOG;
  }
  else {
    stAct = ACT_NO_ACT;
  }

  return stAct;
}


// Отправка соответствующих сообщений об изменениях параметров
uint8_t ltmAlarmProcess( uLtm4xxxStatus status, volatile sDevState * devstate ) {
  eStatusAct stAct;
  sLtm4xxxDev *ltmtm = NULL;
  FlagStatus alrm = SET;

  // Выполняем функцию-реакцию на аварию
  stAct = ltm4xxxStatusAction( ltmtm, status.u32stat );
  if( stAct ){
    if( alrm ){
      // Выполняем функцию-реакцию на аварию
      pwrAction(stAct);
#if DEBUG_ALARM
      trace_printf("LTM:%s dev: %d, pwrline:%d, errId:4, prm:0x%x \n", alrmMsg[stAct], devstate->devId, devstate->pwrLine, status.u32stat );
#endif
      ledOn( LED_ERR, 0 );
    }

  }
  return stAct;
}


uint32_t tmDataSaveLtm( struct i2c_io_register * sequence, uint32_t seqLen, uint32_t idx ){
  sLtm4xxxDev * ltmDev;
  volatile sDevState * ltm;
  static uLtm4xxxStatus devstatus;
  struct i2c_io_register  *io_register;
  bool powerOn, pwrOn_0;
  bool powerOff;
  float fval;
  eRun runnum;

#if defined(USE_FULL_ASSERT)
  FlagStatus vinFlag = RESET;
#endif // defined(USE_FULL_ASSERT)

  // Находим указатель на блок данных LTM:
  ltmDev = (sLtm4xxxDev *)(i2cDevInfo[idx].srcData);

  ltm = &(i2cDevInfo[idx].devState);

  runnum = ltm->runnum;
//  // Сохраняем нынешний статус
//  devstatus.u32stat = ltm->devStatus;

  // Обнуляем статусы, чтобы обновить
  devstatus.u32stat = LTM4XXX_DATA_NOK_STATUS_BIT | LTM4XXX_PEAK_DATA_NOK_STATUS_BIT;

  if( runnum == RUN_NUM ){
    // Исключена из контроля
    pwrOn_0 = powerOn = RESET;
    powerOff = RESET;
  }
  else {
    pwrOn_0 = (gpioPinRun[runnum].state != Bit_RESET) ;
    powerOn = pwrOn_0 && (relevTick[runnum].revOn <= mTick);    // Питание на данной линии включено
    powerOff = (gpioPinRun[runnum].state == Bit_RESET) && (relevTick[runnum].revOff <= mTick);    // Питание на данной линии включено
  }

  /* Готовность данных определяется в конце последовательности чтения данных из LTM4676. */
  for (size_t i = 0; i < seqLen; i++) {
    io_register = &(sequence[i]);

    if( io_register->parsed == false ){
      continue;
    }
    // Должны быть только считанные данные
    assert_param(io_register->state == ReadingDataI2cDeviceState);

    switch (io_register->code) {
      case PMBUS_REG_PAGE:
//          page = io_register->data;
        break;
      case LTM4XXX_REG_MFR_ADDRESS:
        tmpi2cdata = io_register->u32data;
        break;
      case PMBUS_REG_STATUS_BYTE:
      case PMBUS_REG_STATUS_WORD:
        /* BUSY */
        if (!(io_register->u32data & 0x80)) {
          devstatus.dataNok = RESET;
          // Проверяем - включено ли питание ( Избежать ложного выставления флаг при аварийном выключении)
          if (powerOn && !!(io_register->u32data & 0x40)){
            devstatus.turnOff = SET;
          }
          if (io_register->u32data & 0x20){
            // VOUT_OV
            devstatus.voutOnHi = ltmDev->vout.status.onHiAlrm = SET;
          }
          if (io_register->u32data & 0x10){
            // IOUT_OC
            devstatus.ioutOnHi = ltmDev->iout.status.onHiAlrm = SET;
          }
          if (io_register->u32data & 0x04){
            // TEMP_OT
            devstatus.ticHi = ltmDev->tic.status.onHiAlrm = SET;
          }
          if( ltm->cfgFlag == false ){
            /* Попытка записи конфигурации (проверим CML). */
            ltm->cfgProbe = SET;
          }
        }
        else {
          // Данные не готовы - прерываем обрабатку полученных данных
          i = seqLen;
        }
        break;

      case PMBUS_REG_STATUS_VOUT:
        /* VOUT_OV_FAULT */
        if (io_register->u32data & 0x80){
          devstatus.voutOnHi = ltmDev->vout.status.onHiAlrm = SET;
        }
        /* VOUT_UV_FAULT, TON_MAX_FAULT */
        if (io_register->u32data & 0x14){
          devstatus.voutOnLow = ltmDev->vout.status.onLowAlrm = SET;
        }
        if( pwrOn_0 && ~devstatus.voutOnHi && ~devstatus.voutOnLow ){
          if( ltm->pgood == RESET ){
            ltm->pgood = SET;
//            if(ltm->canAttr.pwrLine <= PWR_LINE_VCC_3){
//            pgFlags[ltm->runnum].pgState = SET;
//              pgFlags[ltm->canAttr.devId - DEV_FPGA_1].pgChange = SET;
//            }
          }
        }
        else {
          if( ltm->pgood == SET ){
            ltm->pgood = RESET;
//            pgFlags[runnum].pgState = RESET;
//            pgFlags[ltm->canAttr.devId - DEV_FPGA_1].pgChange = SET;
          }
        }
        break;
      case PMBUS_REG_STATUS_IOUT:
        /* IOUT_OC_FAULT, IOUT_OC_LV_FAULT */
        if (io_register->u32data & 0xc0){
          devstatus.ioutOnHi = ltmDev->iout.status.onHiAlrm = SET;
        }
        break;
      case PMBUS_REG_STATUS_INPUT:
        /* VIN_OV_FAULT */
        if (io_register->u32data & 0x80){
          devstatus.vinHi = ltmDev->vin.status.onHiAlrm = SET;
        }
        /* Unit Off For Insufficient Input Voltage */
        if (io_register->u32data & 0x08){
          devstatus.vinLow = ltmDev->vin.status.onLowAlrm = SET;
        }
        break;
      case PMBUS_REG_STATUS_TEMPERATURE:
        /* OT_FAULT */
        if (io_register->u32data & 0x80){
          devstatus.tchHi = ltmDev->tch.status.onHiAlrm = SET;
        }
        /* UT_FAULT */
        if (io_register->u32data & 0x10){
          devstatus.tchLow = ltmDev->tch.status.onLowAlrm = SET;
        }
        break;
      case PMBUS_REG_STATUS_CML:
        /* Memory Fault Detected, Processor Fault Detected, Other Memory Or Logic Fault */
        if( io_register->u32data & 0xe2 ){
          if (io_register->u32data & 0x19){
            devstatus.procFault = SET;
          }
        }
        else if( ltm->cfgProbe ){
          // Ошибок нет - конфигурация записана верно
          ltm->cfgFlag = true;
          ltm->cfgProbe = RESET;
          if( i2cDevInfo[idx].cfgSequence == ltm46xxPeakRstSeq ){
            i2cDevInfo[idx].cfgSequence = ltm4678RstSeq;
          }
        }

        break;
      case PMBUS_REG_STATUS_MFR_SPECIFIC:
        /* Internal Temperature Fault Limit Exceeded */
        if (io_register->u32data & 0x80){
          devstatus.ticHi = ltmDev->tic.status.onHiAlrm = SET;
        }
        /* VDD33 UV or OV Fault */
        if (io_register->u32data & 0x02){
          devstatus.vdd33Fault = SET;
        }
        break;
      case PMBUS_REG_VIN_OV_FAULT_LIMIT:
        ltmDev->vin.now.limitOnHi = fromLinear11F(io_register->u32data);
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
/*
        devi->vinLimOn = tmDeviation( ltmDev->vin.prev.limitOnHi, ltmDev->vin.now.limitOnHi, \
                                      ltmDev->vin.now.limitOnHi, PWR_DEVI_CENT );
*/
        break;
      case PMBUS_REG_VOUT_OV_FAULT_LIMIT:
        // !!! Обязательно этот регистр обрабатывается первым, по сравнению с PMBUS_REG_VOUT_UV_FAULT_LIMIT
        ltmDev->vout.now.limitOnHi = io_register->u32data / 4096.0f;
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
/*
        devi->voutLimOn = tmDeviation( ltmDev->vout.prev.limitOnHi, ltmDev->vout.now.limitOnHi, \
                                      ltmDev->vout.now.limitOnHi, PWR_DEVI_CENT );
*/
        break;
      case PMBUS_REG_VOUT_UV_FAULT_LIMIT:
        // !!! Обязательно этот регистр обрабатывается вторым, по сравнению с PMBUS_REG_VOUT_OV_FAULT_LIMIT
        ltmDev->vout.now.limitOnLow = io_register->u32data / 4096.0f;
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
/*
        devi->voutLimOn = devi->voutLimOn || tmDeviation( ltmDev->vout.prev.limitOnLow, ltmDev->vout.now.limitOnLow, \
                                      ltmDev->vout.now.limitOnHi, PWR_DEVI_CENT );
*/
        break;
      case LTM46XX_REG_MFR_READ_IIN:
        ltmDev->iin.now.param = fromLinear11F(io_register->u32data);
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
        tmParamProcF( &(ltmDev->iin), RESET, RESET, RESET);

        assert_param( vinFlag );
        fval = ltmDev->vin.now.param * ltmDev->iin.now.param;
        arch_addPartialPout(fval - ltmDev->pin.now.param, idx);
        ltmDev->pin.now.param = fval;
        tmParamProcF( &ltmDev->pin, RESET, RESET, SET );
        break;
      case PMBUS_REG_READ_VIN:
        ltmDev->vin.now.param = fromLinear11F(io_register->u32data);
        tmParamProcF( &ltmDev->vin, RESET, SET, SET );
        devstatus.vinLow = ltmDev->vin.status.onLowAlrm;
        devstatus.vinHi = ltmDev->vin.status.onHiAlrm;
#ifdef USE_FULL_ASSERT
        vinFlag = SET;
#endif // USE_FULL_ASSERT
        break;
      case PMBUS_REG_READ_IIN:
        ltmDev->iin.now.param = fromLinear11F(io_register->u32data) / 2;
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
        tmParamProcF( &(ltmDev->iin), RESET, RESET, RESET );
        // Calculate Pin
        assert_param( vinFlag );
        fval = ltmDev->vin.now.param * ltmDev->iin.now.param;
        arch_addPartialPout(fval - ltmDev->pin.now.param, idx);
        ltmDev->pin.now.param = fval;
        tmParamProcF( &ltmDev->pin, RESET, RESET, SET );
        break;
      case LTM4XXX_REG_MFR_IOUT_PEAK:
        ltmDev->iout.now.peakMax = fromLinear11F(io_register->u32data);
        break;
      case LTM4XXX_REG_MFR_VOUT_PEAK:
        ltmDev->vout.now.peakMax = io_register->u32data / 4096.0;
        break;
      case PMBUS_REG_READ_VOUT:
        ltmDev->vout.now.param = io_register->u32data / 4096.0f;
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
        tmParamProcF( &ltmDev->vout, powerOff, powerOn, SET );
        devstatus.u16stat1 |= ltmDev->vout.status.u8stat << 5;
        devstatus.dataNok = RESET;
        break;

      case PMBUS_REG_READ_IOUT:
        ltmDev->iout.now.param = fromLinear11F(io_register->u32data);
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
        tmParamProcF( &ltmDev->iout, RESET, SET, SET );
        devstatus.ioutOnLow = ltmDev->iout.status.onLowAlrm;
        devstatus.ioutOnHi = ltmDev->iout.status.onHiAlrm;
        break;
      case PMBUS_REG_READ_TEMPERATURE_1:
        // Переходные процессы устаялись. Если не так - значит, последовательность считывания регистров нарушена
//        assert_param(ltm->is_fully_relevant );

        ltmDev->tch.now.param = fromLinear11F(io_register->u32data);
//        // Обновим значение в структуре TM
        tmParamProcF( &(ltmDev->tch), RESET, RESET, SET );
        devstatus.tchLow = ltmDev->tch.status.onLowAlrm;
        devstatus.tchHi = ltmDev->tch.status.onHiAlrm;
        break;
      case PMBUS_REG_READ_TEMPERATURE_2:
        // Переходные процессы устаялись. Если не так - значит, последовательность считывания регистров нарушена
//        assert_param(ltm->is_fully_relevant );

        ltmDev->tic.now.param = fromLinear11F(io_register->u32data);
        // Обновим значение в структуре TM
        tmParamProcF( &(ltmDev->tic), RESET, powerOn, SET );
        devstatus.ticLow = ltmDev->tic.status.onLowAlrm;
        devstatus.ticHi = ltmDev->tic.status.onHiAlrm;
        break;
      case PMBUS_REG_READ_POUT:
        // Переходные процессы устаялись. Если не так - значит, последовательность считывания регистров нарушена
//        assert_param(ltm->is_fully_relevant );

        ltmDev->pout.now.param = fromLinear11F(io_register->u32data);
        // Проверяем на изменения и выставляем соответствующий флаг, если нужно
        tmParamProcF( &ltmDev->pout, RESET, powerOn, SET );
        devstatus.poutOnLow = ltmDev->pout.status.onLowAlrm;
        devstatus.poutOnHi = ltmDev->pout.status.onHiAlrm;

        break;
      case PMBUS_REG_VOUT_COMMAND:
        dbgPrintf("vout_%d:%d\n", idx, (uint32_t)(io_register->u32data / 4.096f) );
        break;
      default:
        break;
    }
  }


  return devstatus.u32stat;
}

/**
  * @brief  Установка признака целостности канала данных устройств LTM4XXX.
  *
  * @param[in]  chain дескриптор цепочки устройств I2C
  * @param[in]  link_ok признак целостности канала данных
  *
  * @retval none
  */
void _ltm4xxx_set_data_link_ok(sI2cBus *chain, bool link_ok) {
  uLtm4xxxStatus devstatus;
  sI2cDevInfo *dev_info;
  volatile sDevState * devstate;
  uLtm4xxxStatus newAlrm;

  uint8_t idx;
  uint32_t oldstatus;

  idx = (chain->device - i2cDev) + chain->device_idx;

  dev_info = &i2cDevInfo[idx];
  devstate = &(dev_info->devState);

  oldstatus = *(devstate->devStatus);

  /* Индицируем обрыв связи с устройством. */
  if (!link_ok) {
    /* Индицируем обрыв связи с устройством. */
    devstatus.u32stat = *(devstate->devStatus);
    devstatus.linkNok = SET;
    // Ошибка соединения: увеличивем счетчик
    ++(devstate->n_faults);
  }
  else {
    // Сохраняем полученные данные
    devstatus.u32stat = tmDataSaveLtm( chain->io_sequence, chain->io_sequence_length, idx );
    /* Снимаем индикацию обрыва связи с устройством. */
    devstatus.linkNok = RESET;

    /* Выставим готовность максимальных и минимальных значений данных. */
    devstatus.dataPeakNok = devstatus.dataNok;

    // Проверяем на новые Аварии
    newAlrm.u32stat = (devstatus.u32stat & ~oldstatus) & LTM4XXX_ALARM_STATUS_MASK;
    if( newAlrm.u32stat ) {
      ltmAlarmProcess( newAlrm, devstate );
      devstate->n_faults = 0;
    }
    else if( devstate->cfgFlag){
      devstate->n_faults = 0;
    }
    else {
      // Соединение есть, по статус - не готов: увеличивем счетчик
      ++(devstate->n_faults);
    }
  }

  if( devstate->n_faults >= _LTM4XXX_FAULTS_MAX ){
    // Счетчик ошибок соединения превысил лимит
    devstate->n_faults = 0;
    devstate->cfgFlag = false;
    devstate->cfgProbe = RESET;
  }

  /* Обновляем состояние. */
  *(devstate->devStatus) = devstatus.u32stat;
}


/**
  * @brief  Обновление данных телеметрии LTM4xxx.
  *
  * @param[in]  phys_idx  физический индекс устройства I2C
  * @param[out] to    буфер данных телеметрии устройства I2C
  *
  * @retval none
  */
void refreshLtm4xxxTm(size_t phys_idx, sLtm4xxxTm *to){
  sLtm4xxxDev  *ltmtm = i2cDevInfo[phys_idx].srcData;

  assert_param(sizeof(*to) <= sizeof(sLtm4xxxTm));

  MAYBE_BUILD_BUG_ON( sizeof(to->status) != sizeof(uint32_t) );
  MAYBE_BUILD_BUG_ON( sizeof(to->status) != sizeof(ltmtm->status) );

  to->status.u32stat = ltmtm->status.u32stat;

  if ( (ltmtm->status.u16stat1 & LTM4XXX_NOK_STATUS_MASK) == RESET ) {
    to->vin  = toLinear11F(ltmtm->vin.now.param);
    to->vout = toLinear11F(ltmtm->vout.now.param);
    to->iin  = toLinear11F(ltmtm->iin.now.param);
    to->iout = toLinear11F(ltmtm->iout.now.param);
    to->pout = toLinear11F(ltmtm->pout.now.param);
    to->tch  = toLinear11F(ltmtm->tch.now.param);
    to->tic  = toLinear11F(ltmtm->tic.now.param);

    to->vinAlrmLimHi = toLinear11F(ltmtm->vin.now.limitOnHi);

    to->voutAlrmLimOnLow  = toLinear11F(ltmtm->vout.now.limitOnLow);
    to->voutAlrmLimOnHi = toLinear11F(ltmtm->vout.now.limitOnHi);
  }

  to->vinAlrmLimLow = toLinear11F(ltmtm->vin.now.limitOnLow);

  to->voutAlrmLimOffLow  = toLinear11F(ltmtm->vout.now.limitOffLow);
  to->voutAlrmLimOffHi = toLinear11F(ltmtm->vout.now.limitOffHi);

  to->poutAlrmLimOnLow  = toLinear11F(ltmtm->pout.now.limitOnLow);
  to->poutAlrmLimOnHi = toLinear11F(ltmtm->pout.now.limitOnHi);

  if ( (ltmtm->status.u16stat1 & LTM4XXX_PEAK_DATA_NOK_STATUS_BIT) == RESET ) {
    to->vinPeakMin = toLinear11F(ltmtm->vin.now.peakMin);
    to->vinPeakMax = toLinear11F(ltmtm->vin.now.peakMax);

    to->voutPeakMin = toLinear11F(ltmtm->vout.now.peakMin);
    to->voutPeakMax = toLinear11F(ltmtm->vout.now.peakMax);

    to->ioutPeakMin = toLinear11F(ltmtm->iout.now.peakMin);
    to->ioutPeakMax = toLinear11F(ltmtm->iout.now.peakMax);

    to->poutPeakMin = toLinear11F(ltmtm->pout.now.peakMin);
    to->poutPeakMax = toLinear11F(ltmtm->pout.now.peakMax);
  }
}

size_t refreshLtmTm( sLtm4xxxTm *to ){
  size_t tmsize = 0;
  uint8_t idx;

  for ( idx = LTM4XXX_0_IDX;  idx <= LTM4XXX_IDX_MAX; idx++, to++) {
    assert_param( idx <= I2C_IDX_MAX);

    assert_param(to != NULL );

    refreshLtm4xxxTm( idx, to);

    tmsize += sizeof( *to );
  }

  return tmsize;
}


/**
  * @brief  Обновление данных LTM4XXX из блока ТМ.
  *
  * @param[in]  ltmnum порядковый номер LTM4XXX в общем списке всех LTM
  * @param[out] regaddr адрес регистра ТМ в блоке LTM4XXX_TM
  * @param[in]  regdata значение регистра
  *
  * @retval true    обновление прошло успешно
  * @retval false   обнаружены ошибки
  */
bool ltmTmUpdate( uint8_t ltmnum, uint16_t regaddr, uint16_t regdata ) {
  sLtm4xxxDev  *ltvDev = i2cDevInfo[ltmnum + LTM4XXX_0_IDX].srcData;

  switch (regaddr) {
    case offsetof(sLtm4xxxTm, status.u16stat1):
      ltvDev->status.u16stat1 &= ~LTM4XXX_ALARM_STATUS_MASK;
      break;
    case offsetof(sLtm4xxxTm, status.u16stat2):
      ltvDev->status.u16stat2 = 0;
      break;
    case offsetof(sLtm4xxxTm, vinPeakMin):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->vin.now.peakMin = NAN;
      break;
    case offsetof(sLtm4xxxTm, vinPeakMax):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->vin.now.peakMax = NAN;
      break;
    case offsetof(sLtm4xxxTm, voutPeakMin):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->vout.now.peakMin = NAN;
      break;
    case offsetof(sLtm4xxxTm, voutPeakMax):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->vout.now.peakMax = NAN;
      break;
    case offsetof(sLtm4xxxTm, ioutPeakMin):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->iout.now.peakMax = NAN;
      break;
    case offsetof(sLtm4xxxTm, ioutPeakMax):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->iout.now.peakMax = NAN;
      break;
    case offsetof(sLtm4xxxTm, poutPeakMin):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->pout.now.peakMin = NAN;
      break;
    case offsetof(sLtm4xxxTm, poutPeakMax):
      ltvDev->status.dataPeakNok = SET;
      ltvDev->pout.now.peakMax = NAN;
      break;
    case offsetof(sLtm4xxxTm, vinAlrmLimLow):
      ltvDev->vin.now.limitOnLow = fromLinear11F( regdata );
      break;
    case offsetof(sLtm4xxxTm, voutAlrmLimOffLow):
      ltvDev->vout.now.limitOffLow = fromLinear11F( regdata );
      break;
    case offsetof(sLtm4xxxTm, voutAlrmLimOffHi):
      ltvDev->vout.now.limitOffHi = fromLinear11F( regdata );
      break;
    case offsetof(sLtm4xxxTm, poutAlrmLimOnLow):
      ltvDev->pout.now.limitOnLow = fromLinear11F( regdata );
      break;
    case offsetof(sLtm4xxxTm, poutAlrmLimOnHi):
      ltvDev->pout.now.limitOnHi = fromLinear11F( regdata );
      break;
    default:
      // Этот регистр не обрабатывается
      errHandler( ERR_UNKNOWN );
      return false;
      break;
  }

  return true;
}

void ltmDevInit( eI2cDev idx ){
  sI2cDevInfo * i2cinfo = &(i2cDevInfo[idx]);
  sLtm4xxxDev  *ltm = (sLtm4xxxDev *)(i2cinfo->srcData);

  // Установка Лимитов
  const float vinLow = LTM_VIN * (1 - LTM_VIN_CENT);
  const float pinHi = (ltmPmbusReg[idx].ioutMax * (ltmPmbusReg[idx].vout * (1 + LTM_VOUT_CENT)) / 1000.0) / 0.85;    // 0.85 - КПД - 85%

  ltm->vin.now.limitOnLow = vinLow;
  ltm->vin.now.limitOnHi = LTM_VIN * (1 + LTM_VIN_CENT);

  ltm->pin.now.limitOnHi = pinHi;
  ltm->iin.now.limitOnHi = pinHi / vinLow;

  ltm->vout.now.limitOffLow = 0.0;
//  if( (idx == I2C_LTM_DD1_1V8) || (idx == I2C_LTM_DD2_1V8) || (idx == I2C_LTM_DD3_1V8) ){
//    ltm->vout.now.limitOffHi = ltmPmbusReg[idx].vout * 0.0003;
//  }
//  else {
    ltm->vout.now.limitOffHi = 0.2; //ltmPmbusReg[idx].vout * 0.003;
//  }
	ltm->vout.now.limitOnLow = ltmPmbusReg[idx].vout * (1 - LTM_VOUT_CENT) / 1000.0;
	ltm->vout.now.limitOnHi = ltmPmbusReg[idx].vout * (1 + LTM_VOUT_CENT) / 1000.0;
	ltm->iout.now.limitOnLow = ltmPmbusReg[idx].ioutMin;
	ltm->iout.now.limitOnHi = ltmPmbusReg[idx].ioutMax;
	ltm->pout.now.limitOnLow = ltmPmbusReg[idx].ioutMin * ltmPmbusReg[idx].vout / 1000.0;
	ltm->pout.now.limitOnHi = ltmPmbusReg[idx].ioutMax * ltmPmbusReg[idx].vout / 1000.0;
	ltm->tch.now.limitOnLow = -40.0;
	ltm->tch.now.limitOnHi = 125.0;
	ltm->tic.now.limitOnLow = -40.0;
	ltm->tic.now.limitOnHi = 125.0;

#ifdef  USE_FULL_ASSERT
  assert_param( idx < I2C_DEV_NUM);
#else
  (void)idx;
#endif  //USE_FULL_ASSERT

//#if CAN_ENABLE
//  eDevId * devid = (eDevId *)&(i2cinfo->devState.canAttr.devId);
//#endif //CAN_ENABLE

  ltm->status.u32stat = LTM4XXX_DATA_NOK_STATUS_BIT | LTM4XXX_PEAK_DATA_NOK_STATUS_BIT;
#if CAN_ENABLE
  uint32_t sendtout = PRM_PWR_LINE_SEND_TOUT;
  tmPrmInit( &(ltm->vin), RESET, SET, sendtout );
  tmPrmInit( &(ltm->iin), RESET, SET, sendtout );
  tmPrmInit( &(ltm->pin), RESET, SET, sendtout );
  tmPrmInit( &(ltm->vout), SET, RESET, sendtout );
  tmPrmInit( &(ltm->iout), RESET, SET, sendtout );
  tmPrmInit( &(ltm->pout), RESET, SET, sendtout );
  tmPrmInit( &(ltm->tch), RESET, SET, sendtout );
  tmPrmInit( &(ltm->tic), RESET, SET, sendtout );

  i2cinfo->devState.canAttr.minorAddr = DEV_ADDR;
#else
  tmPrmInit( &(ltm->vin), RESET, SET );
  tmPrmInit( &(ltm->iin), RESET, SET );
  tmPrmInit( &(ltm->pin), RESET, SET );
  tmPrmInit( &(ltm->vout), SET, RESET );
  tmPrmInit( &(ltm->iout), RESET, SET );
  tmPrmInit( &(ltm->pout), RESET, SET );
  tmPrmInit( &(ltm->tch), RESET, SET );
  tmPrmInit( &(ltm->tic), RESET, SET );
#endif //CAN_ENABLE

  i2cinfo->devState.devStatus = &(ltm->status.u32stat);
#if LTM_VIN_OFF
  i2cinfo->devState.isOn = RESET;
#else
  i2cinfo->devState.isOn = SET;
#endif // LTM_VIN_OFF

}


