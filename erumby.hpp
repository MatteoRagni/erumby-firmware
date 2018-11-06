#ifndef ERUMBY_HPP
#define ERUMBY_HPP

//una sola struttura dati per la lattura di tutti e 7 gli interrupt
typedef struct data_t {
  uint16_t pulseWidth;                          // contiene il dt che ha impiegato una tacca trasparente a passare
  uint16_t pulseWidthReal;                      // serve solo per la modalità per evitare che entri in sicurezza a causa di letture non corrette
  uint16_t edgeTime;                            // contine l'ultimo istante di tempo in cui l'onda è stata alta
  byte counter;                                 // numero di letture tra un invio e l'altro, per la modalità lo usiamo come sicurezza
} data_t;

typedef struct outdata_t {
  uint16_t omega_rr;
  uint16_t omega_rl;
  uint16_t input_esc;
} outdata_t;

typedef struct indata_t {
  int16_t steering;
  int16_t traction;
} indata_t;

typedef struct cmd_t {
  uint16_t esc;
  uint16_t servo;
} cmd_t;

// typedef union outdata_u {
//   outdata_t v;
//   char s[sizeof(outdata_t)];
// } outdata_u;

typedef struct erumby_t {
  // Sensori
  data_t enc_fl;  //< unused
  data_t enc_fr;  //< unused
  data_t enc_rl;
  data_t enc_rr;
  data_t mode;
  data_t steer;   //< unused
  data_t motor;   //< unused
  // Comandi
  indata_t indata;
  cmd_t cmd;
  // Uscite
  outdata_t outdata;
  // Utility
  uint8_t PBintLast;
  uint8_t PKintLast;
} erumby_t;

#endif /* ERUMBY_HPP */