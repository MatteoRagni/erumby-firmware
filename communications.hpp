#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "configurations.hpp"

void wire_request_event();
void wire_receive_event(int size);

/*
 * //dai usec ricavo le velocità angolari e invio quelle
 *
 * La formula utilizzata per il calcolo della velocità angolare è 2pi/200  *  1/deltat  se il contatore è > 1
 * ovvero se la ISR si è attivata, altrimenti è zero. Per lavorare solo con interi il valore di velocità angolare 
 * è moltiplicato per 100.
 */
void intp_read_mode();

void initialize_interrupts();
void initialize_motors();


inline bool is_secure_mode(const erumby_t * e) {
  static const uint16_t duty_mode_middle_low = DUTY_MODE_MIDDLE - 75;
  static const uint16_t duty_mode_middle_high = DUTY_MODE_MIDDLE + 75;
  return ((e->mode.pulseWidthReal >= duty_mode_middle_low) && (e->mode.pulseWidthReal <= duty_mode_middle_high)); 
}

inline bool is_remote_ctrl_mode(const erumby_t * e) {
  static const uint16_t duty_mode_remote_low = DUTY_MODE_LOW - 75;
  static const uint16_t duty_mode_remote_high = DUTY_MODE_LOW + 75;
  return ((e->mode.pulseWidthReal >= duty_mode_remote_low) && (e->mode.pulseWidthReal <= duty_mode_remote_high)); 
} 

inline bool is_debug_mode(const erumby_t * e) {
  #ifdef DEBUG_MODE
  static const uint16_t duty_mode_debug_low = DUTY_MODE_HIGH - 75;
  static const uint16_t duty_mode_debug_high = DUTY_MODE_HIGH + 75;
  return ((e->mode.pulseWidthReal >= duty_mode_debug_low) && (e->mode.pulseWidthReal <= duty_mode_debug_high)); 
  #else
  return false;
  #endif
}

#endif /* COMMUNICATIONS_HPP */