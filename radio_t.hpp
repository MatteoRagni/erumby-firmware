#ifndef RADIO_T_HPP
#define RADIO_T_HPP

#include "configurations.hpp"
#include "interrupt_manager.hpp"
#include "lookup_table_t.hpp"
#include "types.hpp"

void radio_t_mode_callback();

class radio_t {
  pwm_reader_t motor;
  pwm_reader_t steer;
  pwm_reader_mode_t mode;
  erumby_mode_t curr_mode;
  erumby_base_t* m;

#ifndef REMOTE_NOT_WORKING
  lookup_table_t< cmd_t, REMOTE_MOTOR_LUT_SIZE > motor_lookup;
  lookup_table_t< cmd_t, REMOTE_STEER_LUT_SIZE > steer_lookup;
#endif

  static const pulse_t duty_mode_safe_low = DUTY_MODE_SECURE - DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_safe_high = DUTY_MODE_SECURE + DUTY_MODE_OFFSET;

  static const pulse_t duty_mode_auto_low = DUTY_MODE_AUTO - DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_auto_high = DUTY_MODE_AUTO + DUTY_MODE_OFFSET;

  static const pulse_t duty_mode_manual_low = DUTY_MODE_MANUAL - DUTY_MODE_OFFSET;
  static const pulse_t duty_mode_manual_high = DUTY_MODE_MANUAL + DUTY_MODE_OFFSET;

 public:
  radio_t(erumby_base_t* m_)
      : m(m_),
        motor(pwm_reader_t(TRACTION)),
        steer(pwm_reader_t(STEERING)),
        mode(pwm_reader_mode_t(MODE_PIN)),
        curr_mode(Secure) {
#ifndef REMOTE_NOT_WORKING
    cmd_t motor_x[] = REMOTE_MOTOR_LUT_X;
    cmd_t motor_y[] = REMOTE_MOTOR_LUT_Y;
    cmd_t steer_x[] = REMOTE_STEER_LUT_X;
    cmd_t steer_y[] = REMOTE_STEER_LUT_Y;

    motor_lookup.init(motor_x, motor_y, (cmd_t)REMOTE_MOTOR_LUT_SAT);
    steer_lookup.init(steer_x, steer_y, (cmd_t)REMOTE_STEER_LUT_SAT_LOW, (cmd_t)REMOTE_STEER_LUT_SAT_HIGH);
#endif
  }

  erumby_mode_t get_mode() { return curr_mode; }

  void loop() {
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
        m->traction(motor_lookup.eval(motor.get_pulse()));
        m->steer(steer_lookup.eval(steer.get_pulse()));
      }
#endif
      return;
    }

    curr_mode = Secure;
  }
};

#endif /* RADIO_T_HPP */