#pragma once

#include "configuracion.h"

// Mueve las cuatro articulaciones y la pinza, todas con servos MG995 por el PCA9685.
// Las que llevan dos servos enfrentados mueven los dos a la vez. Canales y calibración
// están en configuracion.h.
//
// No hay homing, pero al encender los servos saltan a ARRANQUE: el brazo tiene que estar
// libre antes de alimentarlo.
//
// Cómo se usa desde el programa principal:
//   setup():  movIniciar();  irA(REPOSO, SEGUNDOS_AL_REPOSO);
//   loop():   movActualizar();  en cada vuelta, sin delay()
//             cuando llego() da true, se puede mandar el siguiente irA()

// Arranca el PCA9685 y manda los servos a la pose ARRANQUE de configuracion.h.
void movIniciar();

// Arranca un movimiento suave hasta los ángulos q (grados), que dura los segundos indicados.
// Todas las articulaciones arrancan y llegan juntas.
// Devuelve false y no se mueve si q viola los límites o si el camino baja del piso virtual.
bool irA(const float q[N], float segundos);

// Avanza el movimiento en curso. Llamar en cada vuelta de loop().
void movActualizar();

// true cuando terminó el movimiento.
bool llego();

// Corta el movimiento en curso y deja el brazo donde está.
void frenar();

// Copia en q los ángulos actuales (grados).
void angulosActuales(float q[N]);

void abrirPinza();
void cerrarPinza();

// Emergencia: suelta todos los servos. El brazo queda sin fuerza y puede caer.
// Para volver a usarlo hay que llamar movIniciar().
void apagarMotores();
