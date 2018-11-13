#include "radio_t.hpp"

radio_t* radio_t::self = nullptr;

radio_t * radio_t::create_radio(erumby_base_t * m_) {
  if (self)
    return self;
  self = new radio_t(m_);
  return self;
}

radio_t::radio_t(erumby_base_t* m_)
    : m(m_),
      motor(pwm_reader_t(TRACTION)),
      steer(pwm_reader_t(STEERING)),
      mode(pwm_reader_attachable_t(MODE_PIN)),
      curr_mode(Secure) {
#ifndef REMOTE_NOT_WORKING
  cmd_t motor_x[] = REMOTE_MOTOR_LUT_X;
  cmd_t motor_y[] = REMOTE_MOTOR_LUT_Y;
  cmd_t steer_x[] = REMOTE_STEER_LUT_X;
  cmd_t steer_y[] = REMOTE_STEER_LUT_Y;

  motor_lookup = lookup_table_t< cmd_t, REMOTE_MOTOR_LUT_SIZE >(motor_x, motor_y);
  steer_lookup = lookup_table_t< cmd_t, REMOTE_STEER_LUT_SIZE >(steer_x, steer_y);

  if (!motor_lookup.is_valid())
    m->alarm("Motor lookup for radio not valid");
  if (!steer_lookup.is_valid())
    m->alarm("Steer lookup for radio not valid");
  steer_lookup.x_min();
#endif
}

void radio_t::loop() {
  // This static const are initialized only once by the compiler ;)
  static const pulse_t duty_mode_safe_low = DUTY_MODE_SECURE - DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_safe_high = DUTY_MODE_SECURE + DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_auto_low = DUTY_MODE_AUTO - DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_auto_high = DUTY_MODE_AUTO + DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_manual_low = DUTY_MODE_MANUAL - DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_manual_high = DUTY_MODE_MANUAL + DUTY_MODE_OFFSET;

  if ((mode.get_pulse_real() >= duty_mode_safe_low) && (mode.get_pulse_real() <= duty_mode_safe_high)) {
    if (curr_mode != Secure) {
      m->stop();
      curr_mode = erumby_mode_t::Secure;
    }
    return;
  }
  if ((mode.get_pulse_real() >= duty_mode_auto_low) && (mode.get_pulse_real() <= duty_mode_auto_high)) {
    if (curr_mode != Auto) {
      m->stop();
      curr_mode = erumby_mode_t::Auto;
    }
    return;
  }
  if ((mode.get_pulse_real() >= duty_mode_manual_low) && (mode.get_pulse_real() <= duty_mode_manual_high)) {
    if (curr_mode != Manual) {
      m->stop();
      curr_mode = erumby_mode_t::Manual;
    }
#ifndef REMOTE_NOT_WORKING
    if (curr_mode == Manual) {
      m->traction(motor_lookup(motor.get_pulse()));
      m->steer(steer_lookup(steer.get_pulse()));
    }
#endif
    return;
  }

  curr_mode = Secure;
}
