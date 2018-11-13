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

void setup() {
  erumby = erumby_t::create_erumby();
  tic = millis();
  toc = tic;
}

void loop() {
  toc = millis();
  
  if ((pulse_t)(toc - tic) >= LOOP_TIMING) {
    erumby->loop();
    tic = toc;
  }
}
