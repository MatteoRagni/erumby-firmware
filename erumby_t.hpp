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
 * ENCODER for the sensing, RADIO for modality selection
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
 */
class erumby_t : public erumby_base_t {
  static erumby_t* self;

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
  static erumby_t* create_erumby();

  /** \brief Main loop for erumby
   * 
   * \callgraph
   */
  void loop() {
    if (mode() == Auto) {
      loop_auto();
      return;
    }
    if (mode() == Manual) {
      loop_secure();
      return;
    }
    loop_secure();
  }

  void loop_secure() {
    enc_l->loop();
    enc_r->loop();
    comm->loop_secure();
    radio->loop();
    esc->stop();
    servo->stop();
  }

  void loop_auto() {
    enc_l->loop();
    enc_r->loop();
    comm->loop_auto();
    radio->loop();
    esc->loop();
    servo->loop();
  }

  erumby_mode_t mode() override { return radio->get_mode(); }

  void stop() {
    esc->stop();
    servo->stop();
  }


  void alarm(const char* who, const char* what) override;
  void alarm(const char* who) override { alarm(who, "Unknown reason"); }

  float omega_l() override { return enc_l->get_omega(); }

  float omega_r() override { return enc_r->get_omega(); }

  float omega() override { return (omega_l() + omega_r()) / 2.0; }

  counter_t counter_l() { return enc_l->get_counter(); }

  counter_t counter_r() { return enc_r->get_counter(); }

  const cmd_t traction() const override { return esc->get(); }

  void traction(cmd_t v) override {
    if ((v <= esc->get_max()) && (v >= esc->get_min()))
      esc->set(v);
  }

  void speed(float v) {
    float u = speed_ctrl(v, omega());
    esc->ctrl(u);
  }

  const cmd_t steer() const override { return servo->get(); }

  void steer(cmd_t v) override {
    if ((v <= servo->get_max()) && (v >= servo->get_min()))
      servo->set(v);
  }
};

#endif /* ERUMBY_T_HPP */