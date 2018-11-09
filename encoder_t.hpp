#ifndef ENCODER_T_HPP
#define ENCODER_T_HPP

/**
 * \file encoder_t.hpp
 * \author Matteo Ragni
 *
 * **Class for the Encoder sensors**
 *
 * The class implements the software representation of the
 * Encoder sensor of the car. It takes a
 * PWM as input and calculate the angular velocity of the wheel.
 */

#include "Filters.h"
#include "configurations.hpp"
#include "interrupt_manager.hpp"
#include "types.hpp"

/** /brief Class for the Encoder sensors**
 *
 * The class implements the software representation of the
 * Encoder sensor of the car. It takes a
 * PWM as input and calculate the angular velocity of the wheel.
 */

class encoder_t {
  FilterOnePole* filt; /**< lowpass filter */
  counter_t counter;   /**< incremental counter of the encoder ppr */
  pwm_reader_t pwm;    /**< pwm object for reading the value of signal wave */

 public:
  /** \brief Constructor for the esc object
   *
   * The esc object initializes the value for the ESC PWM
   * and sets the erumby instance pointer. The constructor also
   * initializes the frequency of the PWM pin for the esc.
   * In order to set the frequency, the timers should be initialized
   * by the \p e_rumby::init_timers function, which is called
   * by the constructor.
   *
   * At the end of the constructor the \p alarm is called in order to be sure
   * to write immediately on the PWM the idle values.
   *
   * \param m_ pointers to the unique instance of the erumby machine
   */
  encoder_t(pin_t pin_) : pwm(pwm_reader_t(pin_)), counter(0) { filt = new FilterOnePole(LOWPASS, 5); }

  void interrupt_callback() { pwm.interrupt_callback(); }

  const omega_t get_omega() {
    omega_t omg = (pwm.get_counter() >= 1) ? ((ULONG_PI) / (pwm.get_pulse())) : 0;
    if (pwm.get_counter() == counter) {
      pwm.reset_counter();
    }
    counter = pwm.get_counter();
    return filt->input(omg);
  }

  inline const counter_t get_counter() { return pwm.get_counter(); }
};

#endif /* ENCODER_T_HPP */