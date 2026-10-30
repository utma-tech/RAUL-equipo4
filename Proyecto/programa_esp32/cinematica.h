#pragma once

#include "configuracion.h"   // de acá sale N, la cantidad de articulaciones

// Posición de la punta de las mordazas en mm, respecto a la base del brazo. Ángulos en grados.
//   pitch: inclinación de la pinza respecto a la horizontal (0 = horizontal, -90 = apuntando al piso)
// El brazo tiene cuatro articulaciones y la pinza. No hay giro (roll) de la muñeca.
struct Pose { float x, y, z, pitch; };

enum { IK_OK = 0, FUERA_DE_ALCANCE = -1, FUERA_DE_LIMITES = -2 };

// Ángulos articulares (q[N] = base, hombro, codo, muñeca) → pose de la punta
Pose fk(const float q[N]);

// Pose de la punta → ángulos articulares. Devuelve IK_OK, FUERA_DE_ALCANCE o FUERA_DE_LIMITES.
// Da una sola solución: la de codo arriba.
int ik(Pose p, float q[N]);
