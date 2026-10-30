// Programa principal del brazo RAUL-M (ESP32).
// Por ahora es un programa de prueba, hasta tener los módulos de control y rutinas:
// va al reposo y después alterna entre la mesa y el botón del ascensor.
// Las dos poses salen de simulacion/escena.py y valen con el brazo montado sobre el
// alzador propuesto de 233 mm y RAUL arrimado como dice GUIA_EQUIPO.md.

#include "configuracion.h"
#include "cinematica.h"
#include "movimiento.h"

static const Pose MESA  = {305, 0, 197, 0};   // x, y, z (mm), pitch (grados)
static const Pose BOTON = {215, 0, 417, 0};

static bool vamosALaMesa = true;
static bool enMarcha = true;


// Calcula los ángulos con la ik y arranca el movimiento. Avisa por serie si no se puede.
static bool irAPose(Pose p, float segundos) {
  float q[N];
  if (ik(p, q) != IK_OK) {
    Serial.println("Pose imposible: fuera de alcance o de los limites");
    return false;
  }
  if (!irA(q, segundos)) {
    Serial.println("Camino rechazado: atraviesa el piso virtual");
    return false;
  }
  return true;
}


void setup() {
  Serial.begin(115200);

  // Los servos saltan a ARRANQUE apenas se los alimenta: el brazo tiene que estar libre.
  // No hay homing porque no hay paso a paso ni finales de carrera.
  movIniciar();

  irA(REPOSO, SEGUNDOS_AL_REPOSO);   // lento a propósito: el codo recorre 150°
}


void loop() {
  movActualizar();
  if (!enMarcha || !llego()) return;

  if (vamosALaMesa) enMarcha = irAPose(MESA, 3);
  else              enMarcha = irAPose(BOTON, 3);
  vamosALaMesa = !vamosALaMesa;
}
