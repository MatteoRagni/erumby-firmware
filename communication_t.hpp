#ifndef COMMUNICATIONS_T_HPP
#define COMMUNICATIONS_T_HPP

/** 
 * \file esc_t.hpp
 * \author Davide Piscini, Matteo Ragni
 * 
 * Class for the i2c communication between 
 * 

 */

#include <Arduino.h>
#include <Wire.h>
#include "configurations.hpp"
#include "types.hpp"

/** \brief Class for the i2c communications in the vehicle
 * 
 */
class communication_t {
  typedef struct outdata_t {
    cmd_t omega_rr;
    cmd_t omega_rl;
    cmd_t input_esc;
  } outdata_t;

  typedef struct indata_t {
    int16_t steering;
    int16_t traction;
  } indata_t;

  static communication_t * self;
  indata_t indata;
  outdata_t outdata;
  byte input[sizeof(indata_t)]; 
  byte output[sizeof(outdata_t)];
  erumby_base_t * m;

  communication_t(erumby_base_t * m_);
  
  public:
  static communication_t * create_comms(erumby_base_t * m_);
  static const communication_t * get_comms();

  void loop_secure();
  void loop_auto();
  void receive(int size);
  void send();
  
  cmd_t traction() { return indata.traction; }
  cmd_t steer() { return indata.steering; }
};

#endif /* COMMUNICATIONS_T_HPP */