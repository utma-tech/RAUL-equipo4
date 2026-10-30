#pragma once
// Simulacro mínimo de Arduino para compilar y probar el firmware en la PC.
// El tiempo no corre solo: lo avanza la prueba con avanzarTiempo().
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
using std::max;
using std::min;

static const float PI = 3.14159265f;
static const int LOW = 0, HIGH = 1;
static const int INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2;

unsigned long millis();
unsigned long micros();
void avanzarTiempo(unsigned long us);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

void pinMode(int pin, int modo);
void digitalWrite(int pin, int valor);
int digitalRead(int pin);
int estadoPin(int pin);                 // último valor escrito con digitalWrite

struct PuertoSerie {
  void begin(long) {}
  void println(const char *texto) { printf("    [serie] %s\n", texto); }
};
extern PuertoSerie Serial;
