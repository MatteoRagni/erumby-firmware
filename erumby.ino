#include <PWM.h>
#include <Wire.h>

#include "types.hpp"
#include "configurations.hpp"
#include "interrupt_manager.hpp"

//void erumby_alarm();
//erumby_mode_t erumby_mode();

#include "erumby_t.hpp"

volatile erumby_t * erumby;
pulse_t tic, toc;

void setup() {
  Serial.println("starting setup");
  Serial.begin(115200);
  erumby = new erumby_t();
  tic = millis();
  toc = tic;
  Serial.println("end setup");
}

pulse_t print_counter;
void loop() {
  toc = millis();
  
  if ((pulse_t)(toc - tic) >= LOOP_TIMING) {
    erumby->loop();

    Serial.print(erumby->omega_l());
    Serial.print(",");
    Serial.print(erumby->omega_r());
    Serial.print(",");
    Serial.print(erumby->mode(), DEC);
    Serial.print(",");
    Serial.print(erumby->traction(), DEC);
    Serial.print(",");
    Serial.print(erumby->steer(), DEC);
    Serial.print('\n');

    tic = toc;
  }

}

ISR(PCINT0_vect, ISR_HOW) {  // Abilito i Pin Change Interrupt della porta B
  pwm_reader_t::portB_isr();
}

ISR(PCINT2_vect, ISR_HOW) {  // Abilito i Pin Change Interrupt della porta B
  pwm_reader_t::portK_isr();
}

void wire_on_request() {
  erumby->comm->send();
}

void wire_on_receive(int size) {
  erumby->comm->receive(size);
}

/*
TODO:

 - se andiamo in modalità Auto il firmware si pianta se la raspvberry non manda il segnale.

*/