// Verifica programa_esp32/cinematica.cpp contra la referencia Python.
// casos.csv lo genera simulacion/pruebas.py:
//   fila 1: QMIN[N], QMAX[N]   límites que usa Python
//   resto:  q[N], pose[N]      ángulos y la pose que calculó Python con fk
// Con N = 4 son ocho números por fila.
//
// Compilar y correr desde esta carpeta:
//   g++ -I../programa_esp32 prueba_cinematica.cpp ../programa_esp32/cinematica.cpp -o prueba
//   ./prueba

#include <stdio.h>
#include <math.h>
#include "cinematica.h"
#include "configuracion.h"

// mm o grados. Holgada porque C usa float (como el ESP32) y pierde precisión
// con el brazo casi estirado.
static const float TOLERANCIA = 0.05;

static const int COLUMNAS = 2 * N;

bool leerFila(FILE *archivo, float v[COLUMNAS]) {
  int leidos = fscanf(archivo, "%f,%f,%f,%f,%f,%f,%f,%f",
                      &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]);
  return leidos == COLUMNAS;
}

// Mayor diferencia entre una pose y los N valores de referencia
float diferencia(Pose p, const float ref[N]) {
  float valores[N] = {p.x, p.y, p.z, p.pitch};
  float maxima = 0;
  for (int i = 0; i < N; i++) maxima = fmaxf(maxima, fabsf(valores[i] - ref[i]));
  return maxima;
}

int main() {
  FILE *archivo = fopen("casos.csv", "r");
  if (!archivo) {
    printf("Falta casos.csv: correr primero simulacion/pruebas.py\n");
    return 1;
  }

  float fila[COLUMNAS];
  int fallas = 0;

  // 1. Los límites de configuracion.h coinciden con los de parametros.py
  leerFila(archivo, fila);
  for (int i = 0; i < N; i++) {
    if (fila[i] != QMIN[i] || fila[N + i] != QMAX[i]) {
      printf("FALLA límites de la articulación %d distintos entre configuracion.h y parametros.py\n", i + 1);
      fallas++;
    }
  }

  // 2. fk de C da lo mismo que fk de Python, y fk(ik(pose)) devuelve la misma pose
  float errorFk = 0, errorIdaVuelta = 0;
  int casos = 0;
  while (leerFila(archivo, fila)) {
    casos++;
    float *q = fila;
    float *poseRef = fila + N;

    errorFk = fmaxf(errorFk, diferencia(fk(q), poseRef));

    Pose objetivo = {poseRef[0], poseRef[1], poseRef[2], poseRef[3]};
    float qIk[N];
    int codigo = ik(objetivo, qIk);
    if (codigo == FUERA_DE_ALCANCE) {
      printf("FALLA ik dice fuera de alcance para una pose alcanzable (caso %d)\n", casos);
      fallas++;
    }
    if (codigo == IK_OK) errorIdaVuelta = fmaxf(errorIdaVuelta, diferencia(fk(qIk), poseRef));
  }
  fclose(archivo);

  if (errorFk > TOLERANCIA) fallas++;
  if (errorIdaVuelta > TOLERANCIA) fallas++;

  printf("%s fk de C vs Python:   error máximo %.4f\n", errorFk <= TOLERANCIA ? "OK   " : "FALLA", errorFk);
  printf("%s fk(ik(pose)) = pose: error máximo %.4f\n", errorIdaVuelta <= TOLERANCIA ? "OK   " : "FALLA", errorIdaVuelta);
  printf("%d casos, %d fallas\n", casos, fallas);
  return fallas > 0 ? 1 : 0;
}
