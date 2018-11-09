#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP

#include <Arduino.h>
#include "configurations.hpp"
#include "types.hpp"

#define DUTY_MODE_DELTA 500

class pwm_reader_t {
  pin_t pin;
  pin_t map;
  pulse_t edge_time;
  pin_t read;
  pulse_t pulse;
  counter_t counter;

 public:
  static pwm_reader_t* portB[8];
  static pwm_reader_t* portK[8];
  static pin_t portB_count;
  static pin_t portK_count;

  pwm_reader_t(pin_t pin_) : pin(pin_) {
    map = pwm_reader_t::pin_map(pin);
    pwm_reader_t::init_interrupt(this);
    pulse = 0;
    counter = 0;
    read = 0;
    edge_time = 0;
  }

  void interrupt_callback() {
    pulse_t c_time = micros();
    pin_t c_read = pwm_reader_t::get_read(pin) & map;

    if (c_read ^ read) {
      read = c_read;
      counter++;
      if (read & map)
        edge_time = c_time;
      else
        pulse = c_time - edge_time;
    }
  }

  inline const counter_t get_counter() { return counter; }
  inline void reset_counter() { counter = 0; }
  inline const pulse_t get_pulse() { return pulse; }

  static void portB_isr();
  static void portK_isr();
  static void init_port_B(pwm_reader_t* pwm);
  static void init_port_K(pwm_reader_t* pwm);

 private:
  static pin_t pin_map(pin_t pin) {
    switch (pin) {
      case (52):
        return 0x02;
        break;
      case (53):
        return 0x01;
        break;
      case (A8):
        return 0x01;
        break;
      case (A9):
        return 0x02;
        break;
      default:
        interrupt_error();
        break;
    }
  }

  static void init_interrupt(pwm_reader_t* pwm);

  static uint8_t get_read(pin_t pin) {
    switch (pin) {
      case (52):
        return PINB;
        break;
      case (53):
        return PINB;
        break;
      case (A9):
        return PINK;
        break;
      case (A8):
        return PINK;
        break;
      default:
        pwm_reader_t::interrupt_error();
        break;
    }
  }

  static void interrupt_error() {
    pinMode(ERROR_LED_PORT, OUTPUT);
    digitalWrite(ERROR_LED_PORT, LOW);
    while (1) {
      digitalWrite(ERROR_LED_PORT, HIGH);
      delay(2000);
      digitalWrite(ERROR_LED_PORT, LOW);
      delay(500);
      digitalWrite(ERROR_LED_PORT, HIGH);
      delay(500);
      digitalWrite(ERROR_LED_PORT, LOW);
      delay(500);
      digitalWrite(ERROR_LED_PORT, HIGH);
      delay(500);
      digitalWrite(ERROR_LED_PORT, LOW);
      delay(500);
    }
  }
};

void pwm_reader_mode_callback();

class pwm_reader_mode_t {
  pin_t pin;
  pulse_t pulse;
  pulse_t pulse_real;
  pulse_t edge_time;
  counter_t counter;
  pin_t read;

 public:
  static pwm_reader_mode_t* self;
  pwm_reader_mode_t(pin_t pin_) : pin(pin_) {
    if (self)
      return;
    self = this;
    counter = 0;
    pulse = 0;
    pulse_real = 0;
    edge_time = 0;
    read = 0;

    attachInterrupt(digitalPinToInterrupt(pin), pwm_reader_mode_callback, CHANGE);
  }

  inline counter_t get_counter() { return counter; }
  inline pulse_t get_pulse() { return pulse; }
  inline pulse_t get_pulse_real() { return pulse_real; }

  void interrupt_callback() {
    uint16_t c_time = micros();

    if (digitalRead(pin))
      edge_time = c_time;
    else
      pulse = c_time - edge_time;

    if (abs(pulse_real - pulse) > DUTY_MODE_DELTA)
      counter++;
    else
      counter = 0;

    if (counter == 10) {
      pulse_real = pulse;
      counter = 0;
    }
  }
};

#endif /* INTERRUPT_MANAGER_HPP */