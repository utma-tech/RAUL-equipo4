#include <Arduino.h>
#include <Wire.h>
#include "movimiento.h"
#include "cinematica.h"
#include "configuracion.h"


// PCA9685
// Cada canal genera un pulso cada 20 ms (50 Hz). El ciclo se divide en 4096 cuentas
// y para cada canal se escribe en qué cuenta termina el pulso.
static const uint8_t PCA_DIRECCION = 0x40;
static const uint8_t REG_MODO1     = 0x00;
static const uint8_t REG_CANAL0    = 0x06;   // cada canal ocupa 4 registros a partir de acá
static const uint8_t REG_TODOS_OFF = 0xFD;
static const uint8_t REG_PRESCALER = 0xFE;
static const float US_POR_CUENTA = 20000.0 / 4096;   // 4,88 µs

static void escribirRegistro(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(PCA_DIRECCION);
  Wire.write(registro);
  Wire.write(valor);
  Wire.endTransmission();
}

static void iniciarPca() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);                    // I2C rápido: menos tiempo ocupado por vuelta
  escribirRegistro(REG_MODO1, 0x10);        // dormido: el prescaler solo se puede cambiar así
  escribirRegistro(REG_PRESCALER, 121);     // 25 MHz / (4096 × 50 Hz) - 1 = 121
  escribirRegistro(REG_MODO1, 0x20);        // despierto, con auto-incremento de registro
  delayMicroseconds(500);                   // el oscilador tarda 500 µs en arrancar
}

static void pulso(int canal, float us) {
  int cuenta = (int)lroundf(us / US_POR_CUENTA);
  Wire.beginTransmission(PCA_DIRECCION);
  Wire.write(REG_CANAL0 + 4 * canal);
  Wire.write(0);                  // el pulso arranca en la cuenta 0...
  Wire.write(0);
  Wire.write(cuenta & 0xFF);      // ...y termina en "cuenta" (byte bajo y byte alto)
  Wire.write(cuenta >> 8);
  Wire.endTransmission();
}


// Estado del movimiento
static float actual[N];                 // ángulos que se están comandando ahora
static float inicio[N];
static float destino[N];
static unsigned long tInicio = 0;       // ms
static unsigned long duracion = 1;      // ms
static bool moviendo = false;
static unsigned long ultimoServo = 0;   // µs
static int turnoServo = 0;

static const float A_RAD = PI / 180;


// Manda la articulación i al ángulo pedido. Si lleva dos servos enfrentados, los mueve
// a los dos: el segundo tiene su propia calibración y gira al revés.
static void moverServo(int i, float angulo) {
  const Motor &m = MOTORES[i];
  pulso(m.canal, m.pulsoCero + m.usPorGrado * angulo);
  if (m.canal2 >= 0)
    pulso(m.canal2, m.pulsoCero2 + m.usPorGrado2 * angulo);
}

// Perfil de 5.° grado: con t de 0 a 1 devuelve de 0 a 1, arrancando y terminando
// con velocidad y aceleración cero. Es lo que hace que el brazo no dé tirones.
static float perfil(float t) {
  return t * t * t * (10 - 15 * t + 6 * t * t);
}

// Codo, muñeca y punta por encima del piso virtual
static bool sobreElPiso(const float q[N]) {
  float brazo     = q[1] * A_RAD;
  float antebrazo = (q[1] + q[2]) * A_RAD;
  float alturaCodo   = D1 + A2 * sinf(brazo);
  float alturaMuneca = alturaCodo + A3 * sinf(antebrazo);
  float alturaPunta  = fk(q).z;
  return alturaCodo >= PISO_VIRTUAL && alturaMuneca >= PISO_VIRTUAL && alturaPunta >= PISO_VIRTUAL;
}

static bool dentroDeLimites(const float q[N]) {
  for (int i = 0; i < N; i++)
    if (q[i] < QMIN[i] || q[i] > QMAX[i]) return false;
  return true;
}


void movIniciar() {
  iniciarPca();

  for (int i = 0; i < N; i++) {
    actual[i] = ARRANQUE[i];
    inicio[i] = ARRANQUE[i];
    destino[i] = ARRANQUE[i];
    moverServo(i, ARRANQUE[i]);
  }
  moviendo = false;
}


bool irA(const float q[N], float segundos) {
  if (!isfinite(segundos) || segundos <= 0) return false;
  for (int i = 0; i < N; i++) {
    if (!isfinite(q[i])) return false;
    if (1.875f * fabsf(q[i] - actual[i]) / segundos > VELOCIDAD_MAX) return false;
  }
  if (!dentroDeLimites(q)) return false;

  // El brazo va en línea recta en el espacio de ángulos. Se revisan 20 puntos del camino:
  // entre dos poses sobre el piso virtual puede haber una que lo atraviese.
  for (int k = 1; k <= 20; k++) {
    float s = k / 20.0;
    float intermedio[N];
    for (int i = 0; i < N; i++) intermedio[i] = actual[i] + (q[i] - actual[i]) * s;
    if (!sobreElPiso(intermedio)) return false;
  }

  for (int i = 0; i < N; i++) {
    inicio[i] = actual[i];
    destino[i] = q[i];
  }
  tInicio = millis();
  duracion = max(1.0f, segundos * 1000);
  moviendo = true;
  return true;
}


void movActualizar() {
  // 1. Ángulos del momento según el perfil suave
  if (moviendo) {
    float t = (millis() - tInicio) / (float)duracion;
    if (t >= 1) {
      t = 1;
      moviendo = false;
    }
    float s = perfil(t);
    for (int i = 0; i < N; i++) actual[i] = inicio[i] + (destino[i] - inicio[i]) * s;
  }

  // 2. Servos: una articulación cada 4 ms, por turno. Con cuatro articulaciones cada una
  //    se renueva cada 16 ms, por debajo de los 20 ms que tarda el PCA9685 en repetir el
  //    pulso, así que ningún servo se queda sin actualizar entre pulso y pulso.
  if (micros() - ultimoServo >= 4000) {
    ultimoServo = micros();
    moverServo(turnoServo, actual[turnoServo]);
    turnoServo = (turnoServo + 1) % N;
  }
}


bool llego() {
  return !moviendo;
}

void frenar() {
  for (int i = 0; i < N; i++) {
    inicio[i] = actual[i];
    destino[i] = actual[i];
  }
  moviendo = false;
}

void angulosActuales(float q[N]) {
  for (int i = 0; i < N; i++) q[i] = actual[i];
}

void abrirPinza()  { pulso(CANAL_PINZA, PINZA_ABIERTA); }
void cerrarPinza() { pulso(CANAL_PINZA, PINZA_CERRADA); }

void apagarMotores() {
  moviendo = false;
  escribirRegistro(REG_TODOS_OFF, 0x10);    // todas las salidas del PCA9685 apagadas
}
