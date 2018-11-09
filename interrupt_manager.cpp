#include "interrupt_manager.hpp"

pin_t pwm_reader_t::portB_count = 0;
pin_t pwm_reader_t::portK_count = 0;
pwm_reader_t* pwm_reader_t::portB[8] = {0};
pwm_reader_t* pwm_reader_t::portK[8] = {0};
void pwm_reader_t::init_interrupt(pwm_reader_t* pwm) {
  switch (pwm->pin) {
    case (52):
      init_port_B(pwm);
      break;
    case (53):
      init_port_B(pwm);
      break;
    case (A8):
      init_port_K(pwm);
      break;
    case (A9):
      init_port_K(pwm);
      break;
    default:
      pwm_reader_t::interrupt_error();
      break;
  }
}

void pwm_reader_t::init_port_B(pwm_reader_t* pwm) {
  noInterrupts();
  pinMode(pwm->pin, INPUT_PULLUP);
  PCMSK0 |= pwm->map;
  PCICR |= 0x01;
  pwm_reader_t::portB[pwm_reader_t::portB_count] = pwm;
  pwm_reader_t::portB_count++;
  interrupts();
}

void pwm_reader_t::init_port_K(pwm_reader_t* pwm) {
  noInterrupts();
  pinMode(pwm->pin, INPUT_PULLUP);
  PCMSK2 |= pwm->map;
  PCICR |= 0x02;
  pwm_reader_t::portK[pwm_reader_t::portK_count] = pwm;
  pwm_reader_t::portK_count++;
  interrupts();
}

void pwm_reader_t::portB_isr() {
  for (pin_t i = 0; i < pwm_reader_t::portB_count; i++)
    pwm_reader_t::portB[i]->interrupt_callback();
}

void pwm_reader_t::portK_isr() {
  for (pin_t i = 0; i < pwm_reader_t::portK_count; i++)
    pwm_reader_t::portK[i]->interrupt_callback();
}

pwm_reader_mode_t* pwm_reader_mode_t::self = nullptr;

void pwm_reader_mode_callback() { pwm_reader_mode_t::self->interrupt_callback(); }
