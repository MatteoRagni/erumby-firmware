#include <Wire.h>
#include "PWM.h"
#include "erumby.hpp"
#include "communications.hpp"
#include "configurations.hpp"
#include "motor.hpp"

// State of the machine erumby
volatile erumby_t erumby;
// Pointer that acts as reference to the
// erumby structure (passing around a pointer
// costs less than passing around the entire structure)
// const static erumby_t* erumby_ptr = &erumby;

unsigned long tic, toc;

void setup() {
  erumby.indata.steering = DUTY_SERVO_MIDDLE;
  erumby.indata.traction = 0;
  erumby.cmd.servo = DUTY_SERVO_MIDDLE;
  erumby.cmd.esc = DUTY_ESC_IDLE;

  InitTimersSafe();

  Serial.begin(SERIAL_SPEED);
  Wire.begin(I2C_ADDR);
  Wire.onRequest(wire_request_event);
  Wire.onReceive(wire_receive_event);
  initialize_interrupts();
  initialize_motors();

  tic = millis();
  toc = tic;
}

void loop() {
  toc = millis();
  if ((unsigned long)(toc - tic) >= LOOP_TIMING) {
    read_encoder();
    if (is_secure_mode((const erumby_t *)&erumby))
      secure_mode_loop();
    if (is_remote_ctrl_mode((const erumby_t *)&erumby))
      remote_mode_loop();
    if (is_debug_mode((const erumby_t *)&erumby))
      debug_mode_loop();
    tic = toc;
  }
}

// LOOPS

void secure_mode_loop() {
  erumby.cmd.esc = DUTY_ESC_IDLE;
  erumby.cmd.servo = DUTY_SERVO_MIDDLE;
  pwmWriteHR(ESC, erumby.cmd.esc);
  pwmWriteHR(SERVO, erumby.cmd.servo);
}

void remote_mode_loop() {
  // is possible to write only one statement, i prefer two for mental cleaning and for identify the case of error
  // Condition for avoid dangerous value, on steering
  if ((erumby.indata.steering < DUTY_SERVO_DX) || (erumby.indata.steering > DUTY_SERVO_SX)) {        
    secure_mode_loop();
    return;
  }
  // and on traction
  if ((erumby.indata.traction > DUTY_ESC_MAX) || (erumby.indata.traction < DUTY_ESC_MIN)){
    secure_mode_loop();
    return;
  }

  //servo motor
  erumby.cmd.servo = erumby.indata.steering;
  pwmWriteHR(SERVO, erumby.cmd.servo);
  
  //ESC
  erumby.cmd.esc = erumby.indata.traction;
  erumby.outdata.input_esc = erumby.cmd.esc;
  pwmWriteHR(ESC, erumby.cmd.esc);

}

#ifdef DEBUG_MODE
uint16_t debug_timer = 0;
#endif
void debug_mode_loop() {
  #ifdef DEBUG_MODE
  if (debug_timer >= DEBUG_MODE_STEP_STOP_T) {
    secure_mode_loop();
  } else if (debug_timer >= DEBUG_MODE_STEP_RAISE_T) {
    erumby.cmd.esc = DEBUG_MODE_STEP_RAISE_ESC;
    erumby.cmd.servo = DUTY_SERVO_MIDDLE;
    pwmWriteHR(ESC, erumby.cmd.esc);
    pwmWriteHR(SERVO, erumby.cmd.servo);
  } else {
    secure_mode_loop();
  }
  if ((debug_timer % DEBUG_MODE_SERIAL_DELTA_TIME) == 0) { // Print ever
    Serial.print(millis());
    Serial.print(",");
    Serial.print(erumby.outdata.omega_rr);
    Serial.print(",");
    Serial.print(erumby.outdata.omega_rl);
    Serial.print(",");
    Serial.println(erumby.cmd.esc);
  }
  debug_timer += LOOP_TIMING;
  #endif
}

// Implementations
long omega_old[2] = {0}; //< TODO: To be removed
long omega[2] = {0}; //< TODO: To be removed
int counter_old[2] = {0}; //< TODO: To be removed

void read_encoder() {
  omega[0] = (erumby.enc_rr.counter >= 1) ? ((ULONG_PI)/(erumby.enc_rr.pulseWidth)) : 0;
  omega[1] = (erumby.enc_rl.counter >= 1) ? ((ULONG_PI)/(erumby.enc_rl.pulseWidth)) : 0;

  if (((omega[0] + omega[1]) / 200) >= 200) { // Filtering outliers
    omega[0] = omega_old[0];
    omega[1] = omega_old[1];
  }
  erumby.outdata.omega_rr = omega[0];
  erumby.outdata.omega_rl = omega[1];

  omega_old[0] = omega[0];
  omega_old[1] = omega[1];
  
  if (erumby.enc_rr.counter == counter_old[0]) {
    erumby.enc_rr.counter = 0;
  }
  if (erumby.enc_rl.counter == counter_old[1]) {
    erumby.enc_rl.counter = 0;
  }
  counter_old[0] = erumby.enc_rr.counter; 
  counter_old[1] = erumby.enc_rl.counter;   
}

void intp_read_mode() {
  uint16_t cTime = micros();
  if (digitalRead(MODE) == LOW)
    erumby.mode.pulseWidth = cTime - erumby.mode.edgeTime;
  else
    erumby.mode.edgeTime = cTime;
  if (abs(erumby.mode.pulseWidthReal - erumby.mode.pulseWidth) > 500)
    erumby.mode.counter++;
  else
    erumby.mode.counter = 0;
  if (erumby.mode.counter == 10) {
    erumby.mode.pulseWidthReal = erumby.mode.pulseWidth;
    erumby.mode.counter = 0;
  }
}

void initialize_interrupts() {
  cli();
  // encoders pin
  PCMSK0 = 0;
  pinMode(RLWHEEL, INPUT_PULLUP);
  pinMode(RRWHEEL, INPUT_PULLUP);
  PCMSK0 = 0b00000011;  // maschera del registro B <--> 0, abilito i primi due interupt cambiando i valori logici

  // Pin per la ricevente
  PCMSK2 = 0;
  pinMode(STEERING, INPUT_PULLUP);
  pinMode(TRACTION, INPUT_PULLUP);
  PCMSK2 = 0b00000011;  // maschera del registro K <--> 2, abilito i primi due interupt cambiando i valori logici

  PCICR = 0b00000101;  // Serve per attivare le porte, nel nostro caso la B e la K, cambiando il relativo booleano

  attachInterrupt(0, intp_read_mode, CHANGE);  // abilito l'interupt della porta 2
  sei();
}

void initialize_motors() {
  SetPinFrequency(ESC, 71);
  SetPinFrequency(SERVO, 71);
  erumby.mode.counter = 0;
  erumby.mode.pulseWidthReal = 0;
}

// Interrupt service routines

ISR(PCINT0_vect, ISR_HOW) {  // Abilito i Pin Change Interrupt della porta B
  uint8_t cPINB = PINB;
  uint16_t cTime = micros();
  uint8_t mask = cPINB ^ erumby.PBintLast;      // XOR tra il valore attuale e quello precedente per vedere se sono avvenuti cambiamenti
  erumby.PBintLast = cPINB;  // salva il valore attuale della porta B per le successive invocazioni
  // Lettura Encoder
  if (mask & 0x02) {
    erumby.enc_rl.counter++;
    if (!(cPINB & 0x02))
      erumby.enc_rl.pulseWidth = cTime - erumby.enc_rl.edgeTime;
    else
      erumby.enc_rl.edgeTime = cTime;
  }

  if (mask & 0x01) {
    erumby.enc_rr.counter++;
    if (!(cPINB & 0x01))
      erumby.enc_rr.pulseWidth = cTime - erumby.enc_rr.edgeTime;
    else
      erumby.enc_rr.edgeTime = cTime;
  }
}

ISR(PCINT2_vect, ISR_HOW) {
  uint8_t cPINK = PINK;
  uint16_t cTime = micros();
  uint8_t mask = cPINK ^ erumby.PKintLast;
  erumby.PKintLast = cPINK;
  
  // ACCELERATORE
  if (mask & 0x02) {
    if (!(cPINK & 0x02))
      erumby.motor.pulseWidth = cTime - erumby.motor.edgeTime;
    else
      erumby.motor.edgeTime = cTime;
  }
  // STERZO
  if (mask & 0x01) {
    if (!(cPINK & 0x01))
      erumby.steer.pulseWidth = cTime - erumby.steer.edgeTime;
    else
      erumby.steer.edgeTime = cTime;
  }
}

// Communication

void wire_request_event() {           //set the array properly for communication purpose
  byte output[sizeof(outdata_t)];
   
  output[0] = (erumby.outdata.omega_rr >> 8) & 0xFF;
  output[1] = erumby.outdata.omega_rr & 0xFF;
  output[2] = (erumby.outdata.omega_rl >> 8) & 0xFF;
  output[3] = erumby.outdata.omega_rl & 0xFF;
  output[4] = (erumby.outdata.input_esc >> 8) & 0xFF;
  output[5] = erumby.outdata.input_esc & 0xFF;
  Wire.write(output, sizeof(outdata_t)); 
}

void wire_receive_event(int size) {        // set the reading of 16 bit
  byte a, b, d, e;

  while (1 < Wire.available()) { // loop through all but the last
    a = Wire.read();
    b = Wire.read();
    d = Wire.read();
    erumby.indata.traction = a;
    erumby.indata.traction = erumby.indata.traction << 8 | b;
  }
  e = Wire.read();
 
  erumby.indata.steering = d;
  erumby.indata.steering = erumby.indata.steering << 8 | e; 
}

