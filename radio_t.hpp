#ifndef RADIO_T_HPP
#define RADIO_T_HPP

#include <Arduino.h>
#include "configurations.hpp"
#include "pwm_reader_t.hpp"
#include "lookup_table_t.hpp"
#include "types.hpp"

class radio_t {
  static radio_t * self;
  pwm_reader_t motor;
  pwm_reader_t steer;
  pwm_reader_attachable_t mode;
  erumby_mode_t curr_mode;
  erumby_base_t* m;

#ifndef REMOTE_NOT_WORKING
  lookup_table_t< cmd_t, REMOTE_MOTOR_LUT_SIZE > motor_lookup;
  lookup_table_t< cmd_t, REMOTE_STEER_LUT_SIZE > steer_lookup;
#endif

  radio_t(erumby_base_t* m_);
 public:
  static radio_t* create_radio(erumby_base_t * m_);

  void loop();
  erumby_mode_t get_mode() { return curr_mode; }
};

#endif /* RADIO_T_HPP */