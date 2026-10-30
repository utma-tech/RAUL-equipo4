#include <math.h>
#include "cinematica.h"
#include "configuracion.h"

// Hombro, codo y muñeca giran en un mismo plano vertical; la base orienta ese plano.
// Hombro, codo y muñeca positivos levantan. pitch = hombro + codo + muñeca.
//
// La punta queda a D5 = 82 mm del eje de la muñeca, pero FI = 12,68° por debajo de la
// línea del antebrazo, así que ese tramo se calcula con (pitch − FI) y no con el pitch.

static constexpr float PI = 3.14159265358979323846f;
static const float A_RAD = PI / 180.0f;
static const float A_GRADOS = 180.0f / PI;

Pose fk(const float q[N]) {
  // Ángulo de cada eslabón medido desde la horizontal
  float brazo     = q[1] * A_RAD;
  float antebrazo = (q[1] + q[2]) * A_RAD;
  float pinza     = (q[1] + q[2] + q[3] - FI) * A_RAD;

  // r: distancia horizontal desde el eje de la base hasta la punta
  float r = A2*cosf(brazo) + A3*cosf(antebrazo) + D5*cosf(pinza);
  float z = D1 + A2*sinf(brazo) + A3*sinf(antebrazo) + D5*sinf(pinza);

  float base = q[0] * A_RAD;
  Pose p;
  p.x     = r * cosf(base);
  p.y     = r * sinf(base);
  p.z     = z;
  p.pitch = q[1] + q[2] + q[3];
  return p;
}

int ik(Pose p, float q[N]) {
  // 1. Centro de la muñeca: se retrocede D5 desde la punta, FI grados por debajo del pitch
  float pinza = (p.pitch - FI) * A_RAD;
  float rw = hypotf(p.x, p.y) - D5*cosf(pinza);
  float zw = p.z - D1 - D5*sinf(pinza);

  // 2. Codo: ley de cosenos en el triángulo hombro-codo-muñeca
  float cosCodo = (rw*rw + zw*zw - A2*A2 - A3*A3) / (2*A2*A3);
  if (cosCodo < -1.0001f || cosCodo > 1.0001f) return FUERA_DE_ALCANCE;
  // Con el brazo casi estirado, el redondeo puede dar apenas más que 1: se recorta
  if (cosCodo > 1)  cosCodo = 1;
  if (cosCodo < -1) cosCodo = -1;
  float codo = -acosf(cosCodo);   // negativo = codo arriba

  // 3. Hombro: ángulo hacia la muñeca menos el ángulo que agrega el codo
  float hombro = atan2f(zw, rw) - atan2f(A3*sinf(codo), A2 + A3*cosf(codo));

  // 4. Base y muñeca
  q[0] = atan2f(p.y, p.x) * A_GRADOS;
  q[1] = hombro * A_GRADOS;
  q[2] = codo * A_GRADOS;
  q[3] = p.pitch - q[1] - q[2];

  for (int i = 0; i < N; i++)
    if (q[i] < QMIN[i] || q[i] > QMAX[i]) return FUERA_DE_LIMITES;
  return IK_OK;
}
