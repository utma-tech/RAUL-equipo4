// Prueba la lógica de programa_esp32/movimiento.cpp en la PC, con un simulacro de Arduino
// y del PCA9685 (carpeta simulacro). No prueba los servos reales.
//
// Compilar y correr desde esta carpeta:
//   g++ -Isimulacro -I../programa_esp32 prueba_movimiento.cpp simulacro/simulacro.cpp ../programa_esp32/movimiento.cpp ../programa_esp32/cinematica.cpp -o prueba_mov
//   ./prueba_mov

#include <stdio.h>
#include <math.h>
#include "Arduino.h"
#include "Wire.h"
#include "movimiento.h"
#include "configuracion.h"

static int fallas = 0;

static void revisar(const char *nombre, bool paso) {
  printf("%s %s\n", paso ? "OK   " : "FALLA", nombre);
  if (!paso) fallas++;
}

static bool cerca(float a, float b, float tolerancia) {
  return fabsf(a - b) <= tolerancia;
}

// true si cada servo tiene en el PCA9685 el pulso que corresponde a los ángulos q.
// Las articulaciones con dos servos enfrentados se revisan en los dos canales.
static bool servosEn(const float q[N]) {
  for (int i = 0; i < N; i++) {
    const Motor &m = MOTORES[i];
    float esperado = m.pulsoCero + m.usPorGrado * q[i];
    if (!cerca(pulsoCanal(m.canal), esperado, 2.45f)) return false;        // media cuenta del PCA9685
    if (m.canal2 >= 0) {
      float esperado2 = m.pulsoCero2 + m.usPorGrado2 * q[i];
      if (!cerca(pulsoCanal(m.canal2), esperado2, 2.45f)) return false;
    }
  }
  return true;
}

// Los dos servos de una articulación tienen que empujar para el mismo lado: están
// montados enfrentados, así que cuando uno sube de pulso el otro baja.
static bool enfrentadosOpuestos() {
  for (int i = 0; i < N; i++) {
    const Motor &m = MOTORES[i];
    if (m.canal2 < 0) continue;
    if (m.usPorGrado * m.usPorGrado2 >= 0) return false;
    // Y en el centro del recorrido los dos tienen que pedir el mismo ángulo mecánico
    float medio = (QMIN[i] + QMAX[i]) / 2;
    float uno = m.pulsoCero + m.usPorGrado * medio;
    float dos = m.pulsoCero2 + m.usPorGrado2 * medio;
    if (!cerca(uno + dos, 3000, 1)) return false;   // simétricos respecto de 1500 µs
  }
  return true;
}

// Simula vueltas de loop() de 50 µs hasta que el brazo llega (o pasan "maximo" segundos).
// Devuelve cuánto tardó en segundos.
static float esperarLlegada(float maximo) {
  unsigned long arranque = micros();
  do {
    avanzarTiempo(50);
    movActualizar();
  } while (!llego() && micros() - arranque < maximo * 1000000);
  return (micros() - arranque) / 1000000.0f;
}


int main() {
  // 1. Arranque: PCA9685 configurado a 50 Hz y servos en la pose de arranque
  movIniciar();
  revisar("PCA9685 despierto con auto-incremento", registroPca(0x00) == 0x20);
  revisar("PCA9685 a 50 Hz (prescaler 121)", registroPca(0xFE) == 121);
  revisar("servos en la pose de arranque", servosEn(ARRANQUE));
  revisar("los servos enfrentados giran al reves uno del otro", enfrentadosOpuestos());

  // 2. Órdenes que se tienen que rechazar
  float codoFueraDeLimite[N] = {0, 90, 40, 0};
  revisar("rechaza ángulos fuera de los límites", !irA(codoFueraDeLimite, 2));
  float bajoElPiso[N] = {0, 0, -90, 0};          // antebrazo colgando bajo la plataforma
  revisar("rechaza poses bajo el piso virtual", !irA(bajoElPiso, 2));

  // 3. Ir al reposo: tiempo, posición final, suavidad y que no se pida más de lo que el servo da
  revisar("rechaza tiempo cero", !irA(REPOSO, 0));
  revisar("rechaza exceso de velocidad", !irA(REPOSO, 0.1f));
  float noFinito[N] = {NAN, 90, -150, 60};
  revisar("rechaza angulos no finitos", !irA(noFinito, 4));
  const int MS_AL_REPOSO = (int)(SEGUNDOS_AL_REPOSO * 1000);
  revisar("acepta ir al reposo", irA(REPOSO, SEGUNDOS_AL_REPOSO));

  float anterior[N];
  angulosActuales(anterior);
  float velocidadMax[N] = {0};
  float velocidadInicial = 0;
  float q[N];
  for (int ms = 1; ms <= MS_AL_REPOSO; ms++) {
    for (int k = 0; k < 20; k++) {        // 20 vueltas de loop() de 50 µs = 1 ms
      avanzarTiempo(50);
      movActualizar();
    }
    angulosActuales(q);
    for (int i = 0; i < N; i++) {
      float velocidad = fabsf((q[i] - anterior[i]) * 1000);   // grados por segundo
      if (ms > 1 && velocidad > velocidadMax[i]) velocidadMax[i] = velocidad;
      anterior[i] = q[i];
    }
    if (ms == 10) velocidadInicial = velocidadMax[2];
  }
  float tardo = SEGUNDOS_AL_REPOSO + esperarLlegada(1);
  revisar("llega en el tiempo pedido", cerca(tardo, SEGUNDOS_AL_REPOSO, 0.05));
  revisar("arranca suave (velocidad casi cero al principio)", velocidadInicial < 2);

  // El codo es el que más recorre de ARRANQUE a REPOSO: 150°. El perfil de 5.° grado pica a
  // 1,875 veces la velocidad media, o sea 1,875 × 150 / 4 s = 70 °/s.
  revisar("velocidad máxima del codo como la del perfil (70 °/s)",
          cerca(velocidadMax[2], 70.3, 1.5));

  // Ninguna articulación puede pedir más de lo que estamos dispuestos a pedirle al servo.
  bool dentro = true;
  for (int i = 0; i < N; i++) {
    printf("      %-7s pide %.0f °/s como máximo\n", NOMBRES[i], velocidadMax[i]);
    if (velocidadMax[i] > VELOCIDAD_MAX) dentro = false;
  }
  revisar("ningún servo tiene que ir más rápido de lo permitido", dentro);
  revisar("servos en el reposo", servosEn(REPOSO));

  // 4. Frenar en el medio de un movimiento
  float otraPose[N] = {45, 60, -100, 30};
  revisar("acepta el movimiento que se va a frenar", irA(otraPose, 2));
  for (int k = 0; k < 20000; k++) {      // 1 s en vueltas de 50 µs
    avanzarTiempo(50);
    movActualizar();
  }
  frenar();
  float alFrenar[N];
  angulosActuales(alFrenar);
  esperarLlegada(1);
  angulosActuales(q);
  revisar("frena a mitad de camino y se queda ahí",
          cerca(q[0], alFrenar[0], 0.001) && alFrenar[0] > 1 && alFrenar[0] < 44);

  // 5. Apagado de emergencia
  apagarMotores();
  revisar("apagar: salidas del PCA9685 apagadas", registroPca(0xFD) == 0x10);

  printf("%d fallas\n", fallas);
  return fallas > 0 ? 1 : 0;
}
