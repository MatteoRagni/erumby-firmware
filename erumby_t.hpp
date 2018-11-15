#ifndef ERUMBY_T_HPP
#define ERUMBY_T_HPP

/**
 * \file erumby_t.hpp
 * \author Davide Piscini, Matteo Ragni
 *
 * **Class for the ERUMBY robot**
 *
 * The class implements the software of the car. It manage
 * the object in the car. ESC and SERVO for the actuator,
 * ENCODER for the sensing, RADIO for /modality selection
 */

#include <Arduino.h>
#include "communication_t.hpp"
#include "configurations.hpp"
#include "encoder_t.hpp"
#include "esc_t.hpp"
#include "radio_t.hpp"
#include "servo_t.hpp"

#include "controller_t.hpp"

/** \brief Class for the ERUMBY robot
 *
 * The class implements the software of the car. It manage
 * the object in the car. ESC and SERVO for the actuator,
 * ENCODER for the sensing, RADIO for modality selection.
 *
 * Please notice that the erumby_t class is a singleton class,
 * which means no more than a single instance can exist in the 
 * software.
 */
class erumby_t : public erumby_base_t {
  static erumby_t* self;   /**< Pointer to singleton instance */

 public:
  esc_t* esc;              /**< esc pointer to the class */
  servo_t* servo;          /**< servo pointer to the class  */
  radio_t* radio;          /**<  radio pointer to the class */
  encoder_t* enc_l;        /**< left encoder pointer to the class */
  encoder_t* enc_r;        /**< right encoder pointer to the class */
  communication_t* comm;   /**< Communication singleton with Raspberry pi */
  controller_t speed_ctrl; /**< Controller for the wheel speed (ESC) */

  /** \brief Constructor for the erumby object
   *
   * The erumby object call the costructors of the different
   * object: esc, servo, radio, enc_r, enc_l and initialize
   * the timers in order to allow the comunications
   */
  erumby_t();

 public:
  /**
   *
   * The class erumby_t is a singleton class, which means no more than one instance can exists
   * in the software. In order to do so, the constructor is made private and the single existing
   * instance can be created using the static method \p create_erumby.
   * 
   * \return the singleton instance to erumby_t
   */
  static const erumby_t* create_erumby();

  /**
   * \brief Return the current mode (Manual, Auto, Secure)
   * \return the current mode
   */
  erumby_mode_t mode() override { return radio->get_mode(); }

  /** \brief Main loop for erumby
   *
   * In the main loop the mode is read (\see MODE ) and in relations with this
   * a different execution mode is selected.
   */
  void loop();

  /** \brief Secure loop for erumby: in this state the motor are set in the idle
   * state and the sensors are read.
   *
   * In this modality the following command are executed:
   *  - enc_l: update the value of theta position and angular velocity
   *           of the left encoder with an hig gain filter (\p encoder_t main loop).
   *  - enc_r: update the value of theta position and angular velocity
   *           of the right encoder with an hig gain filter (\p encoder_t main loop).
   *  - comm: write in the i2c the value of the angular velocity of the wheel and
   *          the esc current value (\p communication_t::loop_secure)
   *  - radio: read the pwm value of the radio (\p radio_t)
   *  - esc: set the stop mode for the esc (\p esc_t::stop)
   *  - servo: set the stop mode for the servo (\p servo_t::stop)
   */
  void loop_secure();

  /** \brief Auto loop for erumby: in this state the motor are command with set point
   *  and the sensors are read
   *
   * In this modality the following command are executed:
   *  - enc_l: update the value of theta position and angular velocity
   *           of the left encoder with an hig gain filter (\p encoder_t::loop).
   *  - enc_r: update the value of theta position and angular velocity
   *           of the right encoder with an hig gain filter (\p encoder_t::loop).
   *  - comm: write in the i2c the value of the angular velocity of the wheel and
   *          the esc current value. The steering and traction value are read from
   *          the i2c and this are the current set value for the motors (\p communication_t::loop_auto)
   *  - radio: read the pwm value of the radio (\p radio_t)
   *  - esc: it sets as current value the one read by the communication
   *         for the esc (\p esc_t::loop)
   *  - servo: it sets as current value the one read by the communication
   *           for the servo (\p servo_t::loop)
   */
  void loop_auto();

  /** \brief stop loop for erumby: in this state the motor are set in the idle
   * state and the sensors are reset.
   *
   * In this modality the following command are executed:
   *  - enc_l: the value of the state in the high gain filter
   *           are reset(\p encoder_t::stop).
   *  - enc_r: the value of the state in the high gain filter
   *           are reset(\p encoder_t::stop).
   *  - esc: set the stop mode for the esc (\p esc_t::stop)
   *  - servo: set the stop mode for the servo (\p servo_t::stop)
   */
  void stop();

  /** \brief alarm function for error debuging
   *  set the led of port 13 blinking, With serial monitor is possible
   *  to see the alarm error.
   *
   * \param who a string of the module that raised the alarm
   * \param what a string with a description message
   */
  void alarm(const char* who, const char* what) override;

  /** \brief alarm function for error debuging
   *  set the led of port 13 blinking, With serial monitor is possible
   *  to see the alarm error, but the reason is unknown.
   *
   * \param who a string of the module that raised the alarm
   */
  void alarm(const char* who) override { alarm(who, "Unknown reason"); }

  /** \brief The value of the angular velocity of the left encoder
   *
   * \return the angular velocity of the left encoder
   */
  float omega_l() override { return enc_l->get_omega(); }

  /** \brief The value of the angular velocity of the right encoder
   *
   * \return the angular velocity of the right encoder
   */
  float omega_r() override { return enc_r->get_omega(); }

  /** \brief The value of the mean angular velocity of the encoders
   *
   * \return the mean angular velocity of the encoders
   */
  float omega() override { return (omega_l() + omega_r()) / 2.0; }

  /** \brief The value of the pwm value of the esc
   *
   * \return the pwm value of the esc
   */
  const cmd_t traction() const override { return esc->get(); }

  /** \brief set the esc pwm
   * set the pwm value of the esc, this value is saturated in
   * DUTY_ESC_MAX, DUTY_ESC_MIN boundary
   *
   * \param v the value of PWM to write on the ESC
   */
  void traction(cmd_t v) override {
    if ((v <= esc->get_max()) && (v >= esc->get_min()))
      esc->set(v);
  }

  /** \brief Main loop of the controller
   *
   * Evaluates the reference tracking error through the smith predictor (in the
   * block scheme the signal `e`), evaluates the feed forward and closed loop
   * control action and updates the smith predictor.
   *
   * \param v the speed value for the speed controller
   */
  void speed(float v) {
    float u = speed_ctrl(v, omega());
    esc->ctrl(u);
  }

  /** \brief The value of the pwm value of the servo
   *
   * \return the pwm value of the servo
   */
  const cmd_t steer() const override { return servo->get(); }

  /** \brief set the servo pwm
   * set the pwm value of the servo, this value is saturated in
   * DUTY_SERVO_MAX, DUTY_SERVO_MIN boundary
   *
   * \param v the value of PWM to write on the Servo
   */
  void steer(cmd_t v) override {
    if ((v <= servo->get_max()) && (v >= servo->get_min()))
      servo->set(v);
  }
};

#endif /* ERUMBY_T_HPP */