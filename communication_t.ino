#include "communication_t.hpp"

communication_t * communication_t::self = NULL;

communication_t * communication_t::create_comms(erumby_base_t * m_) {
  if (communication_t::self != NULL)
    return communication_t::self;
  communication_t::self = new communication_t(m_);
  return communication_t::self;
}

const communication_t * communication_t::get_comms() { 
  return communication_t::self; 
}

communication_t::communication_t(erumby_base_t * m_) : m(m_) {
  indata.steering = DUTY_SERVO_MIDDLE;
  indata.traction = DUTY_ESC_IDLE;
  Wire.begin(I2C_ADDR);
  Wire.onRequest([]() -> void { communication_t::get_comms()->send(); });
  Wire.onReceive([](int s) -> void { communication_t::get_comms()->receive(s); });
}

void communication_t::loop_secure() {
  outdata.omega_rr = round(m->omega_r() * 100);
  outdata.omega_rl = round(m->omega_l() * 100);
  outdata.input_esc = m->traction();
}

void communication_t::loop_auto() {
  outdata.omega_rr = round(m->omega_r() * 100);
  outdata.omega_rl = round(m->omega_l() * 100);
  outdata.input_esc = m->traction();

  if (indata.traction > 0) {
    m->speed(float(indata.traction) / 100.0);
  } else {
    m->traction(-indata.traction);
  }
  m->steer(indata.steering);
}

void communication_t::receive(int size) {        
  while (1 < Wire.available()) { 
    input[0] = Wire.read();
    input[1] = Wire.read();
    input[2] = Wire.read();
    indata.traction = input[0];
    indata.traction = indata.traction << 8 | input[1];
  }
  input[3] = Wire.read();

  indata.steering = input[2];
  indata.steering = indata.steering << 8 | input[3]; 
}

void communication_t::send() {
  output[0] = (outdata.omega_rr >> 8) & 0xFF;
  output[1] = outdata.omega_rr & 0xFF;
  output[2] = (outdata.omega_rl >> 8) & 0xFF;
  output[3] = outdata.omega_rl & 0xFF;
  output[4] = (outdata.input_esc >> 8) & 0xFF;
  output[5] = outdata.input_esc & 0xFF;
  Wire.write(output, sizeof(outdata_t)); 
}