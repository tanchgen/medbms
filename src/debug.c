/*
 * debug.c
 *
 *  Created on: 26 дек. 2018 г.
 *      Author: Gennadiy Tanchin <g.tanchin@yandex.ru>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "tinyalloc.h"

//extern  uint8_t pmlc[];

int32_t m_count = 0;

const char * errMsg[] = {
    "Error not found\r\n",                  /** < Отсутсвие ошибок */
    // ---------------- I2C ERROR ---------------------
    "I2C1 device: Link error.\r\n",         /** < Ошибка связи с устройством I2C1 */
    "I2C2 device: Link error.\r\n",         /** < Ошибка связи с устройством I2C2 */
    "I2C1 device error.\r\n",               /** < Ошибка устройства I2C1 */
    "I2C2 device error.\r\n",               /** < Ошибка устройства I2C2 */
    "I2C device: Data buffer error.\r\n",   /** < Ошибка обращения к буферу данных I2C2 */
    "I2C device: CRC error.\r\n",           /** < Ошибка CRC полученых от устройства I2C2 данных */
    // --------------- CAN_BUS ERROR -------------------
    "CAN Buffer error: ",                   /** < Ошибка записи в буфер отправки CAN */
    "CAN Source ID error",                  /** < Неправильный SrcID */
    "CAN Device ID error",                  /** < Неправильный DevID */
    "CAN Power line error",                 /** < Неправильный PwrLine */
    "CAN Param ID error",                   /** < Неправильный PrmID */
    "CAN Param type error",                 /** < Неправильный PrmType */
    "CAN Param/MAX/MIN/LIMITS error",       /** < Неправильное значения Параметра/MAX/MIN/LIMITS */

    // ---------------- ETH ERROR ---------------------
    "TCP: Max connection quantity",         /** < Превышение максимального количества TCP-клиентов */

    // --------------- SYSTEM ERROR -------------------
    "System ON timeout error: ",             /** < Ошибка Таймаут включения системы */
    "System OFF timeout error: ",            /** < Ошибка Таймаут выключения системы */
    "POWER ALERT! \r\n",
    "POWER ALARM! \r\n",

    "CMOS_Bat ADC error.\r\n",              /** < Ошибка АЦП Bat_CMOS */
    "CMOS_Bat voltage is not reduced.\r\n", /** < Ошибка: не падает напржение Bat_CMOS */
    "CMOS_Bat voltage is not increasing.\r\n",  /** < Ошибка: не растет напржение Bat_CMOS */
    "Heater current ADC error.\r\n",        /** < Ошибка АЦП Hcur (Ток нагревателя) */

    "Fan error.\r\n",                       /** < Ошибка Вентилятора */
    "+3.3V DSW.\r\n",                       /** < Ошибка +3.3В DSW */
    "Intel start timeout error.\r\n",       /** < Ошибка: превышен таймаут запуска INTEL */
    "Intel stop timeout error.\r\n",        /** < Ошибка: превышен таймаут остановки INTEL */
    "CATERR signal from Intel.\r\n",        /** < Ошибка: CatError INTEL */
    "Data not ready.\r\n",                  /** < Ошибка: Общая ошибка - данные не готовы */
    "Unknown Register code.\r\n"            /** < Ошибка: Неизвестный код Регистра SLAVE-модуля */
    "PLL Error: NSS is LOW\r\n",            /** < Ошибка: Низкий уровень линии PLL */
    "USB_Host error\r\n"                    /** < Ошибка: USB host error */

    "UNKNOWN error.\r\n",           /** <  Неизвестная ошибка */
};

const char * alrmMsg[] = {
  "NO_ACT",             // ACT_NO_ACT
  "HW_LOG",             // ACT_HW_LOG
  "PWR_OFF",            // ACT_PWR_OFF
  "PWR_FAIL_OFF",       // ACT_PWR_FAIL_OFF
  "PWR_EMRG_OFF",       // ACT_PWR_ALRT_OFF
  "PWR_FULL_OFF_NOW",   // ACT_PSU_ALRT_OFF
};

const char * pwrStateMsg[PWRSTATE_PWR_ON + 1] = {
  "PWR_OFF",       // 1
  "PWR_START",     // 2
  "I2C_ON",        // 4
  "FAN_ON",        // 5
  "RUN_1",         // 6
  "RUN_2",         // 7
  "RUN_3",         // 8
  "ZYNQ_PL_DONE",  // 9
  "PWR_ON",        // 10
};


void __attribute__((weak)) dbgSend( const char * buf, uint8_t len){
  // TODO: В будущем реализовать вывод на DBG_UART
  (void)buf;
  (void)len;
//    while( dbgUartTxHandle.dma_channel->CNDTR != '\0' )
//    {}
//
//    memcpy( pmlc, (uint8_t*)buf, len );
//
//    while( USART_GetFlagStatus(dbgUartTxHandle.usart, USART_FLAG_TC) == RESET )
//    {}
//    /* Clear the TC bit in the SR register by writing 0 to it. */
//    USART_ClearFlag(dbgUartTxHandle.usart, USART_FLAG_TC);
//
//    /*
//     * In order to reload a new number of data items to be transferred
//     * into the DMA_CNDTRx register, the DMA channel must be disabled.
//     */
//    DMA_Cmd(dbgUartTxHandle.dma_channel, DISABLE);
//
//    dbgUartTxHandle.dma_channel->CMAR = (uint32_t)pmlc;
//    // Укажем число передаваемых байт.
//    dbgUartTxHandle.dma_channel->CNDTR = len;
//
//    /* Activate the channel in the DMA register. */
//    DMA_Cmd(dbgUartTxHandle.dma_channel, ENABLE);
}

// ----------------------------------------------------------------------------

int dbgPrintf(const char* format, ...) {
  int ret;
  va_list ap;

  va_start (ap, format);

  static char buf[255];

  // Print to the local buffer
  ret = vsnprintf (buf, sizeof(buf), format, ap);
  if (ret > 0){
    // Transfer the buffer to the device
    dbgSend( buf, (size_t)ret );
  }

  va_end (ap);
  return ret;
}

void dbgPuts(const char *s) {
  dbgSend( s, strlen(s) );
  dbgSend( "\n", 1);
}

void dbgPutchar(int c) {
  dbgSend( (const char *)&c, 1);
}

void errHandler( eErr err ){
  char errBuf[255];

  MAYBE_BUILD_BUG_ON( ARRAY_SIZE( errMsg ) == ERR_NUM );
  strcpy( errBuf, "ERR: ");
  char * pbuf = errBuf + 5;
  if( err >= ARRAY_SIZE( errMsg ) ){
    strcpy( pbuf, "Unknown error detect.\n");
  }
  else{
    strcpy( pbuf, errMsg[err] );
  }
  dbgSend( errBuf, strlen((char *)errBuf) );
}

void * my_malloc( size_t size ){
  void * p;

  if( (p = ta_alloc( size )) != NULL ){
    m_count++;
  }

  trace_printf( "Malloc p:%x,s:%d\n", p, size);

  return p;
}

void my_free( void * p ){
  trace_printf( "Free p:%x,n:%d\n", p, m_count);
  ta_free( p );
  m_count--;
}
