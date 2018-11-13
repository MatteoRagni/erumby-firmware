#include "pwm_reader_t.hpp"

// pwm_reader_t - C++ implementation

pin_t pwm_reader_t::portB_count = 0;
pin_t pwm_reader_t::portK_count = 0;
pwm_reader_t* pwm_reader_t::portB[8] = {0};
pwm_reader_t* pwm_reader_t::portK[8] = {0};

void pwm_reader_t::interrupt_callback() {
  pulse_t c_time = micros();
  pin_t c_read = port_reader() & map;  // probably this can be digitalRead(pin)

  if (c_read ^ read) {
    read = c_read;
    counter++;
    if (read & map)
      edge_time = c_time;
    else
      pulse = c_time - edge_time;
  }
}

void pwm_reader_t::portB_isr() {
  for (pin_t i = 0; i < pwm_reader_t::portB_count; i++)
    pwm_reader_t::portB[i]->interrupt_callback();
}

void pwm_reader_t::portK_isr() {
  for (pin_t i = 0; i < pwm_reader_t::portK_count; i++)
    pwm_reader_t::portK[i]->interrupt_callback();
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

pin_t pwm_reader_t::pin_map(pin_t pin) {
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

pwm_port_reader_t pwm_reader_t::get_port_reader(pin_t pin) {
  switch (pin) {
    case (52):
      return [](void) -> pin_t { return PINB; };
      break;
    case (53):
      return [](void) -> pin_t { return PINB; };
      break;
    case (A9):
      return [](void) -> pin_t { return PINK; };
      break;
    case (A8):
      return [](void) -> pin_t { return PINK; };
      break;
    default:
      pwm_reader_t::interrupt_error();
      break;
  }
  return [](void) -> pin_t { return 0; }; // required
}

void pwm_reader_t::interrupt_error() {
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

ISR(PCINT0_vect, ISR_NOBLOCK) { pwm_reader_t::portB_isr(); }

ISR(PCINT2_vect, ISR_NOBLOCK) { pwm_reader_t::portK_isr(); }

// pwm_reader_attachable_t - C++ implementation

uint8_t pwm_reader_attachable_t::pin_counter = 0;
pwm_reader_attachable_t* pwm_reader_attachable_t::pin_table[6] = {0};

const pwm_attachable_callback_t pwm_reader_attachable_t::callbacks[6] = {
  [](void) -> void { pin_table[0]->interrupt_callback(); }, 
  [](void) -> void { pin_table[1]->interrupt_callback(); },
  [](void) -> void { pin_table[2]->interrupt_callback(); }, 
  [](void) -> void { pin_table[3]->interrupt_callback(); },
  [](void) -> void { pin_table[4]->interrupt_callback(); },
  [](void) -> void { pin_table[5]->interrupt_callback(); }
};

// pwm_reader_mode_t - C++ implementation

// pwm_reader_mode_t* pwm_reader_mode_t::self = nullptr;

// void pwm_reader_mode_callback() { pwm_reader_mode_t::self->interrupt_callback(); }