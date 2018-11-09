#ifndef COMMUNICATIONS_T_HPP
#define COMMUNICATIONS_T_HPP

/** 
 * \file esc_t.hpp
 * \author Matteo Ragni
 * 
 * ** Class for the i2c communications in the vehicle**
 * 

 */

#include <Wire.h>
#include "configurations.hpp"
#include "types.hpp"

void wire_on_request();
void wire_on_receive(int size);

typedef struct outdata_t {
  cmd_t omega_rr;
  cmd_t omega_rl;
  cmd_t input_esc;
} outdata_t;

typedef struct indata_t {
  int16_t steering;
  int16_t traction;
} indata_t;

/** \brief Class for the i2c communications in the vehicle
 * 
 */
class communication_t {
  indata_t indata;
  outdata_t outdata;
  byte input[sizeof(indata_t)]; 
  byte output[sizeof(outdata_t)];
  erumby_base_t * m;

  public:
  /** \brief Constructor for the communications object
   * 
   */
  communication_t(erumby_base_t * m_) : m(m_) {
    indata.steering = DUTY_SERVO_MIDDLE;
    indata.traction = DUTY_ESC_IDLE;
    Wire.begin(I2C_ADDR);
    Wire.onRequest(wire_on_request);
    Wire.onReceive(wire_on_receive);
  };
  
  /** \brief
   * 
   */
  void loop_secure() {
    outdata.omega_rr = m->omega_r();
    outdata.omega_rl = m->omega_l();
    outdata.input_esc = m->traction();
  }
  void loop_auto() {
    outdata.omega_rr = m->omega_r();
    outdata.omega_rl = m->omega_l();
    outdata.input_esc = m->traction();
    
    m->traction(indata.traction);
    m->steer(indata.steering);
  }

  cmd_t traction() { return indata.traction; }
  cmd_t steer() { return indata.steering; }

  /** 
   * 
   */

  void receive(int size) {        
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

  void send() {           //set the array properly for communication purpose

    output[0] = (outdata.omega_rr >> 8) & 0xFF;
    output[1] = outdata.omega_rr & 0xFF;
    output[2] = (outdata.omega_rl >> 8) & 0xFF;
    output[3] = outdata.omega_rl & 0xFF;
    output[4] = (outdata.input_esc >> 8) & 0xFF;
    output[5] = outdata.input_esc & 0xFF;
    Wire.write(output, sizeof(outdata_t)); 
  }
  
};

#endif /* COMMUNICATIONS_T_HPP */