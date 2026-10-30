#pragma once

// Todo lo que hay que ajustar del brazo está en este archivo.
// Las medidas y los límites están repetidos en simulacion/parametros.py y simulacion/brazo.urdf:
// si no coinciden, fallan las pruebas.


// Cuatro articulaciones y la pinza: base, hombro, codo y muñeca. Sin giro de muñeca.
#define N 4
static const char *NOMBRES[N] = {"base", "hombro", "codo", "muneca"};


// Geometría del brazo (mm), medida sobre el CAD del Equipo 3. Los tres ejes están a la
// misma altura y son paralelos, así que el brazo es plano.
static const float D1 = 131;   // cara de montaje de la placa → eje del hombro
static const float A2 = 200;   // hombro → codo
static const float A3 = 170;   // codo → muñeca

// La pinza cuelga 18 mm por debajo del eje de la muñeca. En el CAD se miden 80 mm hacia
// adelante y 18 hacia abajo; de ahí salen D5 y FI.
static const float D5 = 82.0f;      // hypot(80, 18)
static const float FI = 12.6804f;   // atan2(18, 80) en grados

// Alcance: 452 mm del eje del hombro, y 583 mm de altura máxima sobre la placa.


// Límites articulares (grados). Son los 180° que da el servo MG995, no los del mecanismo:
// hay que recortarlos calibrando el brazo real (ver README).
//                          base  hombro  codo  muñeca
static const float QMIN[N] = { -90,    0, -150,   -90};
static const float QMAX[N] = {  90,  180,   30,    90};

// Pose de reposo: brazo plegado (grados)
// Deja la punta a 165 mm del eje de la base y a 166 mm de altura, o sea plegada sobre la
// propia base: sirve como pose para circular por el ITR.
static const float REPOSO[N] = {0, 90, -150, 60};

// Pose de arranque: la que toman los servos al encender (grados).
// Con el codo recto y la pinza hacia arriba, nada queda apoyado contra la plataforma.
static const float ARRANQUE[N] = {0, 0, 0, 90};

// Cuánto tiene que durar como mínimo el primer movimiento, de ARRANQUE al reposo.
// El recorrido más largo es el del codo: 150°. El perfil de 5.° grado pica a 1,875 veces la
// velocidad media, así que en 4 s pide 70 °/s, por debajo de VELOCIDAD_MAX.
// Regla general: ningún movimiento puede durar menos de 1,875 × recorrido / VELOCIDAD_MAX.
static const float SEGUNDOS_AL_REPOSO = 4;

// Piso virtual: el codo, la muñeca y la punta nunca bajan de esta altura sobre la cara inferior de la placa (mm)
static const float PISO_VIRTUAL = 30;


// Motores: ocho servos MG995 por el PCA9685, uno en la base, dos en el hombro, dos en el
// codo, dos en la muñeca y uno en la pinza. Sin paso a paso ni finales de carrera, así que
// tampoco hay homing.
//
// Las articulaciones con dos servos los llevan enfrentados, uno de cada lado del eje, así
// que el segundo gira al revés: su usPorGrado es negativo y su pulsoCero es el complemento
// del otro. canal2 = -1 en las que llevan uno solo.
struct Motor {
  int canal;             // canal del PCA9685 del servo principal
  float pulsoCero;       // pulso (µs) que pone la articulación en 0° del modelo
  float usPorGrado;      // µs por grado

  int canal2;            // segundo servo, enfrentado al primero. -1 si no hay
  float pulsoCero2;
  float usPorGrado2;
};

// Valores PROVISIONALES: calibrar con el brazo real.
// Servos de 180° con pulsos de 500 a 2500 µs → 2000 / 180 = 11,11 µs por grado.
// El pulsoCero de cada uno sale de dónde cae el 0° del modelo dentro de su recorrido:
// el hombro usa de 0° a 180°, así que su 0° es un extremo (500 µs); el codo usa de -150° a
// 30°, así que su centro (1500 µs) queda en -60° y el 0° en 2167 µs.
static const Motor MOTORES[N] = {
  // canal pulsoCero usPorGrado   canal2 pulsoCero2 usPorGrado2
  {   0,     1500,     11.11f,      -1,      0,        0      },   // base, un solo servo
  {   1,      500,     11.11f,       2,   2500,     -11.11f   },   // hombro, dos enfrentados
  {   3,     2167,     11.11f,       4,    833,     -11.11f   },   // codo, dos enfrentados
  {   5,     1500,     11.11f,       6,   1500,     -11.11f   },   // muñeca, dos enfrentados
};

// Pinza (servo del PCA9685)
static const int CANAL_PINZA = 7;
static const float PINZA_ABIERTA = 1000;   // µs
static const float PINZA_CERRADA = 2000;   // µs

// Velocidad máxima que se le pide a un servo (grados por segundo). No es lo que el MG995
// puede dar, que son unos 300 °/s sin carga, sino lo que le pedimos con el brazo cargado.
// Las pruebas avisan si algún tramo se pasa.
static const float VELOCIDAD_MAX = 90;


// Pines del ESP32
static const int PIN_SDA = 21;              // I2C del PCA9685
static const int PIN_SCL = 22;
