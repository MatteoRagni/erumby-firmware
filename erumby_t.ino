#include "erumby_t.hpp"

erumby_t * erumby_t::self = nullptr;

erumby_t * erumby_t::create_erumby() {
  if (self) return self;
  self = new erumby_t();
  return self;
}

erumby_t::erumby_t()  {
  InitTimersSafe();
  esc = new esc_t(this);
  servo = new servo_t(this);
  enc_r = new encoder_t(R_WHEEL_ENCODER);
  enc_l = new encoder_t(L_WHEEL_ENCODER);
  
  radio = radio_t::create_radio(this);
  comm = communication_t::create_comms(this);
}

void erumby_t::loop() {
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

void erumby_t::loop_secure() {
  enc_l->loop();
  enc_r->loop();
  comm->loop_secure();
  radio->loop();
  esc->stop();
  servo->stop();
}

void erumby_t::loop_auto() {
  enc_l->loop();
  enc_r->loop();
  comm->loop_auto();
  radio->loop();
  esc->loop();
  servo->loop();
}

void erumby_t::stop() {
  enc_l->stop();
  enc_r->stop();
  esc->stop();
  servo->stop();
}

void erumby_t::alarm(const char* who, const char* what) {
  char led = 0;
  esc->stop();
  servo->stop();

  pinMode(ERROR_LED_PORT, OUTPUT);
  Serial.begin(SERIAL_SPEED);
  Serial.flush();
  while (1) {
    led ^= 1;
    digitalWrite(ERROR_LED_PORT, led);

    Serial.println("\n\nALARM MESSAGE");
    Serial.print("origin:   ");
    Serial.println(who);
    Serial.print("reason:   ");
    Serial.println(what);
    Serial.println("\nSET POINTS:");
    Serial.print("esc:      ");
    Serial.println(traction(), DEC);
    Serial.print("servo:    ");
    Serial.println(steer(), DEC);
    Serial.println("");
    Serial.println("\nI2C COMMS:");
    Serial.print("traction: ");
    Serial.println(comm->traction(), DEC);
    Serial.print("steer:    ");
    Serial.println(comm->steer(), DEC);
    delay(5000);
  }
}