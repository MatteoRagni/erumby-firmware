#include <PWM.h>
#include <Wire.h>

/** 
 * \mainpage
 * \author Davide Piscini, Matteo Cocetti, Matteo Ragni
 * 
 * Firmware for erumby low level controllor.
 * 
 * \todo Documentation shall be completed
 */

#include "types.hpp"
#include "configurations.hpp"

#include "erumby_t.hpp"

volatile erumby_t * erumby;
pulse_t tic, toc;
char debug;

void setup() {
  pinMode(8, OUTPUT);
  debug = 0;
  erumby = erumby_t::create_erumby();
  tic = micros();
  toc = tic;
}

void loop() {
  toc = micros();
  
  if ((pulse_t)(toc - tic) >= (LOOP_TIMING * 1000)) {
    debug ^= 1;
    digitalWrite(8, debug);

    erumby->loop();
    tic = toc;
  }
}
