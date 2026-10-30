#pragma once
// Simulacro del bus I2C: en vez de mandar bytes, interpreta lo que se le escribiría al PCA9685.
#include <stdint.h>

struct BusI2C {
  void begin(int sda, int scl) {}
  void setClock(long hz) {}
  void beginTransmission(uint8_t direccion);
  void write(uint8_t byte);
  void endTransmission();
};
extern BusI2C Wire;

// Lo que quedó escrito en el PCA9685 simulado
int registroPca(int registro);          // último valor escrito en un registro de configuración
int cuentaCanal(int canal);             // cuenta en la que termina el pulso de un canal
float pulsoCanal(int canal);            // lo mismo, en µs
