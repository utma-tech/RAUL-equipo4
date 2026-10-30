// Implementación del simulacro de Arduino y Wire.
#include "Arduino.h"
#include "Wire.h"

PuertoSerie Serial;
BusI2C Wire;


// Reloj
static unsigned long tiempoUs = 0;

unsigned long micros() { return tiempoUs; }
unsigned long millis() { return tiempoUs / 1000; }
void avanzarTiempo(unsigned long us) { tiempoUs += us; }
void delay(unsigned long ms) { tiempoUs += ms * 1000; }
void delayMicroseconds(unsigned int us) { tiempoUs += us; }


// Pines
static int pines[64];

void pinMode(int pin, int modo) {}
void digitalWrite(int pin, int valor) { if (pin >= 0) pines[pin] = valor; }
int estadoPin(int pin) { return pines[pin]; }
int digitalRead(int pin) { return HIGH; }


// PCA9685: 256 registros. Los canales empiezan en 0x06 y ocupan 4 registros cada uno.
static int registros[256];
static uint8_t mensaje[16];
static int largo = 0;

void BusI2C::beginTransmission(uint8_t direccion) { largo = 0; }
void BusI2C::write(uint8_t byte) { if (largo < 16) mensaje[largo++] = byte; }

void BusI2C::endTransmission() {
  // El primer byte es el registro; los siguientes se escriben en registros consecutivos
  for (int k = 1; k < largo; k++) registros[(mensaje[0] + k - 1) % 256] = mensaje[k];
}

int registroPca(int registro) { return registros[registro]; }

int cuentaCanal(int canal) {
  int base = 0x06 + 4 * canal;
  return registros[base + 2] + 256 * (registros[base + 3] & 0x0F);
}

float pulsoCanal(int canal) { return cuentaCanal(canal) * 20000.0f / 4096; }
