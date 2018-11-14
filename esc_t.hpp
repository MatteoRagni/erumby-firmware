#ifndef ESC_T_HPP
#define ESC_T_HPP

/**
 * \file esc_t.hpp
 * \author Matteo Ragni
 *
 * ** Class for the ESC (Electronic speed controller)**
 *
 * The class implements the software representation of the
 * Electronic Speed Controller of the car. It takes a
 * PWM as input and sets a voltage on the motor input
 * in order to get movement. The ESC class also implements
 * some security features such raising an alarm when the input
 * is out the boundaries.
 */

#include "configurations.hpp"
#include "types.hpp"
#include "lookup_table_t.hpp"

/** \brief Class for the ESC (Electronic speed controller)
 *
 * The class implements the software representation of the
 * Electronic Speed Controller of the car. It takes a
 * PWM as input and sets a voltage on the motor input
 * in order to get movement. The ESC class also implements
 * some security features such raising an alarm when the input
 * is out the boundaries.
 */
class esc_t {
  const pin_t pin;    /**< Pin for the ESC, this is specified in \p configurations.hpp */
  cmd_t value;        /**< PWM value that is currently on PWM */
  cmd_t queued_value; /**< PWM requested by the user */
  erumby_base_t* m;   /**< Pointer to erumby main instance */
  lookup_table_t<float, 2> map; /**< Map from [0, 1] to [idle, max] */

  /** \brief Check the user input. If in remote may raise the alarm
   *
   * The function limits the input between the boundaries of the duty
   * cycle of the esc (\p DUTY_ESC_MIN, \p DUTY_ESC_MAX). If the
   * machine is in remote mode, then the \p erumby_t::alarm will be
   * called for an input out of bounds.
   *
   * \param v the user input to be checked
   * \return the value of PWM that will be set
   */
  inline cmd_t input_check(cmd_t v) {
    if ((v > get_max()) || (v < get_min())) {
      if (m->mode() == Auto) {
        m->alarm("ESC");
      }
      return get_idle();
    }
    return v;
  }

 public:
  /** \brief Constructor for the esc object
   *
   * The esc object initializes the value for the ESC PWM
   * and sets the erumby instance pointer. The constructor also
   * initializes the frequency of the PWM pin for the esc.
   * In order to set the frequency, the timers should be initialized
   * by the \p e_rumby constructor.
   *
   * At the end of the constructor the \p stop is called in order to be sure
   * to write immediately on the PWM the idle values.
   * 
   * \param m_ pointer to the main instance of \p erumby_y
   */
  esc_t(erumby_base_t* m_) : m(m_), value(DUTY_ESC_IDLE), queued_value(DUTY_ESC_IDLE), pin(ESC) {
    const float x[] =  { 0.0, 1.0 };
    const float y[] = { float(DUTY_ESC_IDLE), float(DUTY_ESC_MAX) };
    map = lookup_table_t<float, 2>(x, y);
    SetPinFrequency(pin, PWM_FREQUENCY);
    stop();
  };

  /** \brief Puts a queued value for PWM in the class
   *
   * The method allows the user to request a new value for the
   * PWM. The value will be written to the motor once the esc executes
   * a loop. This means that the writes on ESC is queued to the next
   * iteration.
   * To be clear:
   *  1. the user calls \p set with a value of PWM, the value is in the queue
   *  2. the machine continues with its loop, calling all devices loop
   *  3. When the esc loop is called, if the queued value is different
   *   from the current value, the queued value is written on PWM.
   * To be noted: if the user requires two value **before** at least one
   * loop is executed, the first value will be never exported to the
   * PWM pin (it means that the two request are faster than a single machine loop).
   * This allows to send the command even in an Interrupt Service Routine,
   * or in a communication routine keeping the code execution deterministic.
   *
   * \param v the user value to be queued
   */
  void set(cmd_t v) { queued_value = input_check(v); };

  /** \brief Puts a [0,1] control value in queue
   *
   * The method allows the user to request a new value for the
   * PWM with a float value from 0 to 1. The value will be written to the motor once 
   * the esc executes a loop. This means that the writes on ESC is queued to the next
   * iteration. The function saturates.
   * To be clear:
   *  1. the user calls \p ctrl with a value in [0, 1], the value is in the queue
   *  2. the machine continues with its loop, calling all devices loop
   *  3. When the esc loop is called, if the queued value is different
   *   from the current value, the queued value is written on PWM.
   * To be noted: if the user requires two value **before** at least one
   * loop is executed, the first value will be never exported to the
   * PWM pin (it means that the two request are faster than a single machine loop).
   * This allows to send the command even in an Interrupt Service Routine,
   * or in a communication routine keeping the code execution deterministic.
   * 
   * The input is mapped using a lookup table that is defined as:
   * 
   * | Input Value | Output Value                                     |
   * |-------------|--------------------------------------------------|
   * | < 0         | `DUTY_CYCLE_IDLE`                                |
   * | = 0         | `DUTY_CYCLE_IDLE`                                |
   * | in (0, 1)   | (linear mapping and rounding to nearest integer) |
   * | = 1         | `DUTY_CYCLE_MAX`                                 |
   * | > 1         | `DUTY_CYCLE_MAX`                                 |
   * 
   * 
   * \param v the user value to be queued
   */
  void ctrl(float v) { 
    queued_value = round(map(v)); 
  };

  /** \brief ESC main loop of execution
   *
   * This function must be called inside the \p erumby_t::loop and
   * it is the main loop for the ESC. When value is different than
   * the queued value, the queud value is written to the ESC and stored
   * as the current value.
   *
   * \see set
   */
  void loop() {
    if (value != queued_value) {
      value = queued_value;
      pwmWriteHR(pin, value);
    }
  }

  /** Normal stop
   *
   * The normal stop puts the idle value in the queue in order to
   * arrest motor immediately
   */
  void stop() {
    value = get_idle();
    queued_value = get_idle();
    pwmWriteHR(pin, value);
  }

  /** 
   * \brief Returns the value currently on the PWM pin
   * \return the value that is currently wirtten in pwm 
   */
  inline const cmd_t get() const { return value; }
  /** 
   * \brief Returns minimum PWM value possible 
   * \return the minim value for the ESC that it is possible to write
   * \see DUTY_ESC_MIN 
   */
  inline const cmd_t get_min() const { return DUTY_ESC_MIN; }
  /** 
   * \brief Returns maximum PWM value possible
   * \return the maximum value it is possible to write
   * \see DUTY_ESC_MIN
   */
  inline const cmd_t get_max() const { return DUTY_ESC_MAX; }
  /** 
   * \brief Returns the idle PWM value 
   * \return the PWM value that stops the motor
   * \see DUTY_ESC_IDLE
   */
  inline const cmd_t get_idle() const { return DUTY_ESC_IDLE; }
};

#endif /* ESC_T_HPP */