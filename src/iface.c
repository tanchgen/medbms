#include <stddef.h>

#include "main.h"

void gpioInit( void );
void timersInit( void );
//void adcMainInit( void );
void usbInit( void );
//void measInit( void );
void i2cInit( void );
void bq30z55Init( void );

void gpioEnable( void );
//void adcMainEnable( void );
void i2cEnable( void );

void timersClock( void );
void gpioClock( void );
//void adcProcess( void );
void usbClock();
//void measClock( void );
void i2cClock( void );

void (*funcClock[])( void ) = {
   timersClock,
   gpioClock,
   i2cClock,
//   adcProcess,
//   measClock,
   usbClock,
};

void ifaceEnable( void ){
  gpioEnable();
//  adcMainEnable();
  i2cEnable();
}

void ifaceInit( void ){
  timersInit();
//  measInit();
  gpioInit();
  i2cInit();
  bq30z55Init();
//  adcMainInit();
  usbInit();
}

void ifaceClock( void ){
  for( uint8_t i = 0; i < ARRAY_SIZE(funcClock); i++ ){
    funcClock[i]();
  }
}
