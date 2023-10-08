/*
   Función PID (código no ejecutable)
*/

#define tiempo_ciclo 5000

float PID_I_term_array[3];
float PID_error_ant_array[3];
float PID_error;
float consigna_0, consigna_1, consigna_2;
float variable_medida_0, variable_medida_1, variable_medida_2;
float PID_incl_Y, PID_velo_Y, PID_velo_X;
bool PID_Ki_RST = 1;
uint32_t tiempo_nuevo_ciclo;

void setup() {
}

void loop() {
  while (micros() - tiempo_nuevo_ciclo < tiempo_ciclo);            // Comienzo de un nuevo ciclo
  tiempo_nuevo_ciclo = micros();                                   // Instante de comienzo del ciclo

  PID_incl_Y = PID(consigna_0, variable_medida_0, 3, 0.006, 2, 150, PID_Ki_RST, 0);  // PITCH
  PID_velo_Y = PID(consigna_1, variable_medida_1, 3, 0.006, 2, 150, PID_Ki_RST, 1);  // ROLL
  PID_velo_X = PID(consigna_2, variable_medida_2, 3, 0.006, 2, 150, PID_Ki_RST, 2);  // PITCH
}

float PID(float consigna, float medida, float kp, float ki, float kd, int16_t saturacion, uint8_t RST, uint8_t identificador) {
  /*
    Entradas de la función:
    - consigna: consigna o setpoint del controlador PID
    - medida: medida o variable a controlar
    - kp: constante proporcional
    - ki: constante integral
    - kd : constante derivativa
    - saturacion: saturación tanto de la parte integral como de la salida del PID
    - RST: se utliliza para reiniciar a 0 la parte integral, por ejemplo, al deshabilitar el control de estabilidad.
    - identificador: identificador único de cada PID. Cada controlador PID necesita conocer tanto el error
      como la parte integral del ciclo anterior. Utilizando este identificador se guardan en un array estas dos
      variables de cada PID.

    Salida de la función
     - PID_out: salida del PID en microsegundos
  */

  PID_error  = consigna - medida;                                                                       // Error entre lectura y consigna
  float PID_P_term = kp * PID_error;                                                                    // Parte proporcional
  float PID_I_term = RST * (PID_I_term_array[identificador] + ki * PID_error * (tiempo_ciclo / 1000));  // Parte integral (integral del error en el tiempo)
  if (PID_I_term > saturacion)PID_I_term = saturacion;                                                  // Limitar parte integral (+)
  if (PID_I_term < -saturacion)PID_I_term = -saturacion;                                                // Limitar parte integral (-)
  float PID_D_term = kd * (PID_error - PID_error_ant_array[identificador]) / (tiempo_ciclo / 1000);     // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_error_ant_array[identificador] = PID_error;                                                       // Guardar error del ciclo anterior
  PID_I_term_array[identificador] = PID_I_term;                                                         // Guardar parte integral del ciclo anterior

  float PID_out = PID_P_term + PID_I_term + PID_D_term;                                                 // Salida PID
  if (PID_out > saturacion)PID_out = saturacion;                                                        // Limitar salida del PID (+)
  if (PID_out < -saturacion)PID_out = -saturacion;                                                      // Limitar salida del PID (-)

  return PID_out;
}
