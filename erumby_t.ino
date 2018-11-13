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

void erumby_t::alarm(const char* who, const char* what) {
  char led = 0;
  static const byte led_pin = 13;
  esc->stop();
  servo->stop();

  pinMode(led_pin, OUTPUT);
  Serial.begin(115200);
  Serial.flush();
  while (1) {
    led ^= 1;
    digitalWrite(led_pin, led);

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