#include <PWM.h>
#include <Wire.h>


/** 
 * \mainpage
 * \author Davide Piscini, Matteo Cocetti, Matteo Ragni
 * 
 * Firmware for erumby low level controllor.
 * 
 * The project of erumby car is an object oriented code for the low level functionaliy of the 
 * car. Several objects have been implemented that correspond to hardware and software functionality
 * with a modular structure. The following is a general explanation of how the code works, for major 
 * datails please see the complete documentations.
 * 
 * @cond                                                                                                
 *                               +-----------+                                                                   
 *                               |           |                                                        
 *                               |   init    |                                                        
 *                               |           |                                                       
 *                               +-----------+                                                       
 *                                     |                                                             
 *                                     |                                                             
 *                                     V                                                             
*                                +-----------+                                                                   
 *                               |           |                                                        
 *                               |   idle    |                                                        
 *                +--------------|           |--------------+                                                  
 *                |              +-----------+              |                                                
 *                |                ^       ^                |                                           
 *                |                |       |                |                                           
 *                |                |       |                |                                           
 *                V                |       |                V                                           
 *          +-----------+          |       |          +-----------+                                           
 *          |           |          |       |          |           |                    
 *          |   auto    |----------+       +----------|  manual   |                    
 *          |           |                             |           |                   
 *          +-----------+                             +-----------+                                                                  
 *                                                                                                  
 *                                                                                                  
 * @endcond                                                                                                 
 * @image html state_machine.png "State Machine Scheme"  
 *                                                                                                
 * At the start the erumby initialize its state and create the various object (encoder, communications, esc, servo),
 * after that the robot goes in the idle state. In this state the encoder, the i2c communications and the radio are read
 * the motors are set in the idle state. At this point if the lateral command of the radio is switched the machine
 * can go in two different state (\p radio_t). If goes in manual mode the the encoder, the i2c communications and the radio 
 * are read and the motors are command via the radio command input.
 * 
 * \warning The remote triggers and wheels does not work, thus the remote input 
 * has been disabled commenting the define \p REMOTE_NOT_WORKING
 * 
 * \warning Due to the non functioning remote the Manual mode has never been 
 * actually tested. **Test it in a safe environment before using it**.  
 * 
 * If the robot goes in auto mode the encoder, the i2c communications and the radio are read. Via i2c the steering command is read
 * and the servo motor is actuated according to this command. For the esc two scenario are possibile:
 * 
 *  - open loop: the input via i2c is a negative uint16 number in the accetable range, this value is basically a pwm target and the 
 *               esc is driven by this signal with the swapped sign ( \p communication_t). 
 * 
 *  - closed loop: the input via i2c is a positive uint16 number, this value is the desire angular velocity of the wheel (\p communication_t)
 *                 this value is the reference of the speed controller and the esc is driven with the controller output (\p controller_t) 
 * 
 * The erumby can return in the idle state at any time                                                                                                                                                                                                -
 */


#include "types.hpp"
#include "configurations.hpp"

#include "erumby_t.hpp"

volatile erumby_t * erumby;
pulse_t tic, toc;
char debug;

void setup() {
  pinMode(8, OUTPUT);
  debug = 0;
  erumby = erumby_t::create_erumby();
  tic = micros();
  toc = tic;
}

void loop() {
  toc = micros();
  
  if ((pulse_t)(toc - tic) >= (LOOP_TIMING * 1000)) {
    debug ^= 1;
    digitalWrite(8, debug);

    erumby->loop();
    tic = toc;
  }
}
