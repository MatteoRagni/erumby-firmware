#ifndef ENCODER_T_HPP
#define ENCODER_T_HPP

/**
 * \file encoder_t.hpp
 * \author Matteo Ragni
 *
 * **Class for the Encoder sensors**
 *
 * The class implements the software representation of the
 * Encoder sensor of the car. It takes a PWM as input and calculate the angular 
 * velocity of the wheel.
 */

#include "configurations.hpp"
#include "high_gain_obs_t.hpp"
#include "pwm_reader_t.hpp"
#include "types.hpp"

/** /brief Class for the Encoder sensors**
 *
 * The class implements the software representation of the
 * Encoder sensor of the car. It takes a
 * PWM as input and calculate the angular velocity of the wheel.
 */

class encoder_t {
  counter_t counter;                 /**< incremental counter of the encoder ppr */
  pwm_reader_t pwm;                  /**< pwm object for reading the value of signal wave */
  high_gain_obs_t< LOOP_TIMING > hg; /**< High gain filter for encoder reading */
  float theta;                       /**< Internal position for the wheel (direct read from the sensor) */
  float omega;                       /**< High gain estimation of the wheel speed */

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
  encoder_t(pin_t pin_)
      : pwm(pwm_reader_t(pin_)),
        counter(0),
        hg(high_gain_obs_t< LOOP_TIMING >(HG_L1, HG_L2, HG_L3, HG_EPSILON)),
        theta(0),
        omega(0) {}

  const void loop() {
    theta += (M_PI * float(get_counter()) / 100.0);
    pwm.reset_counter();
    counter = pwm.get_counter();
    omega = hg(theta);
  }

  const float get_omega() const { return omega; }
  const float get_theta() const { return theta; }

  inline const counter_t get_counter() { return pwm.get_counter(); }
  inline const void stop() {
    theta = 0.0;
    omega = 0.0;
    hg.reset();
    pwm.reset_counter();
  }
};

#endif /* ENCODER_T_HPP */