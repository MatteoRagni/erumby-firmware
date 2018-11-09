#ifndef ERUMBY_T_HPP
#define ERUMBY_T_HPP

/**
 * \file erumby_t.hpp
 * \author Davide Piscini
 *
 * ** Class for the ERUMBY robot**
 *
 * The class implements the software of the car. It manage
 * the object in the car. ESC and SERVO for the actuator,
 * ENCODER for the sensing, RADIO for modality selection
 */

#include "communications_t.hpp"
#include "configurations.hpp"
#include "encoder_t.hpp"
#include "esc_t.hpp"
#include "radio_t.hpp"
#include "servo_t.hpp"

/** \brief Class for the ERUMBY robot
 *
 * The class implements the software of the car. It manage
 * the object in the car. ESC and SERVO for the actuator,
 * ENCODER for the sensing, RADIO for modality selection.
 */
class erumby_t : public erumby_base_t {
 public:
  esc_t* esc;       /**< esc pointer to the class */
  servo_t* servo;   /**< servo pointer to the class  */
  radio_t* radio;   /**<  radio pointer to the class */
  encoder_t* enc_l; /**< left encoder pointer to the class */
  encoder_t* enc_r; /**< right encoder pointer to the class */
  communication_t* comm;

  /** \brief Constructor for the erumby object
   *
   * The erumby object call the costructors of the different
   * object: esc, servo, radio, enc_r, enc_l and initialize
   * the timers in order to allow the comunications
   */
  erumby_t() {
    InitTimersSafe();
    Serial.println("creating esc");
    esc = new esc_t(this);
    Serial.println("creating servo");
    servo = new servo_t(this);
    Serial.println("creating radio");
    radio = new radio_t(this);
    Serial.println("creating enc_r");
    enc_r = new encoder_t(R_WHEEL_ENCODER);
    Serial.println("creating end_l");
    enc_l = new encoder_t(L_WHEEL_ENCODER);
    Serial.println("creating comm");
    comm = new communication_t(this);
  }

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
    comm->loop_secure();
    radio->loop();
    esc->stop();
    servo->stop();
  }

  void loop_auto() {
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

  void alarm(const char* who) {
    esc->stop();
    servo->stop();

    Serial.print("ALAAAAAARM!!!  ");
    Serial.println(who);

    Serial.print("ESC CMD: ");
    Serial.println(traction());
    Serial.print("SERVO CMD: ");
    Serial.println(steer());
    Serial.print("I2C ESC ");
    Serial.println(comm->traction());
    Serial.print("I2C SERVO ");
    Serial.println(comm->steer());

    while (1) {
    }
  }

  omega_t omega_l() override { return enc_l->get_omega(); }

  omega_t omega_r() override { return enc_r->get_omega(); }

  counter_t counter_l() { return enc_l->get_counter(); }

  counter_t counter_r() { return enc_r->get_counter(); }

  const cmd_t traction() const override { return esc->get(); }

  void traction(cmd_t v) override {
    if ((v <= esc->get_max()) && (v >= esc->get_min()))
      esc->set(v);
  }

  const cmd_t steer() const override { return servo->get(); }

  void steer(cmd_t v) override {
    if ((v <= servo->get_max()) && (v >= servo->get_min()))
      servo->set(v);
  }
};

#endif /* ERUMBY_T_HPP */