# Cinemática de referencia del brazo de 4 grados de libertad.
# Posiciones en mm, ángulos en grados.
#   q    = [base, hombro, codo, muñeca]
#   pose = [x, y, z, pitch]   (punta de las mordazas, respecto a la base del brazo)
# fk e ik son las mismas fórmulas que programa_esp32/cinematica.cpp.
from math import sin, cos, atan2, acos, hypot, radians, degrees
import numpy as np
from parametros import D1, A2, A3, D5, FI, QMIN, QMAX, N

OK = 0
FUERA_DE_ALCANCE = -1
FUERA_DE_LIMITES = -2


# ─── Tabla Denavit-Hartenberg (convención estándar) ───────────
#  i |   θ        |  d  |  a  |  α
#  1 | q1         | D1  |  0  | 90°
#  2 | q2         |  0  | A2  |  0
#  3 | q3         |  0  | A3  |  0
#  4 | q4 − φ     |  0  | D5  |  0
#  5 | +φ         |  0  |  0  |  0     ← fija, no es una articulación
#
# Las dos últimas filas son la pinza. En el CAD no va centrada en el eje de la muñeca:
# la punta queda D5 = 82 mm del eje, pero φ = 12,68° por debajo de la línea del antebrazo.
# La fila 4 lleva el origen hasta la punta; la fila 5 gira el eje x de vuelta a la dirección
# en la que apunta la pinza, que es paralela al antebrazo. Así T[:3, 0] es esa dirección
# y T[:3, 3] la posición de la punta.

def matriz_dh(theta, d, a, alfa):
    """Transformación homogénea de una fila de la tabla DH (ángulos en radianes)."""
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alfa), sin(alfa)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])


def fk_dh(q):
    """Transformación base → punta de las mordazas, multiplicando las matrices DH.
    La columna 3 (T[:3, 3]) es la posición; la columna 0 (T[:3, 0]) es la dirección
    en la que apunta la pinza."""
    q1, q2, q3, q4 = [radians(v) for v in q]
    fi = radians(FI)
    tabla = [
        (q1,       D1, 0,  radians(90)),
        (q2,       0,  A2, 0),
        (q3,       0,  A3, 0),
        (q4 - fi,  0,  D5, 0),
        (fi,       0,  0,  0),
    ]
    T = np.eye(4)
    for fila in tabla:
        T = T @ matriz_dh(*fila)
    return T


# ─── Forma cerrada ────────────────────────────────────────────
# Hombro, codo y muñeca giran en un mismo plano vertical; la base orienta ese plano.

def en_el_plano(q):
    """Posición de la punta dentro del plano del brazo: (r, z, pitch).
    r es la distancia horizontal desde el eje de la base hasta la punta.
    pitch es hacia dónde apunta la pinza; la punta en sí cae φ grados por debajo."""
    brazo = radians(q[1])
    antebrazo = radians(q[1] + q[2])
    pinza = radians(q[1] + q[2] + q[3] - FI)

    r = A2 * cos(brazo) + A3 * cos(antebrazo) + D5 * cos(pinza)
    z = D1 + A2 * sin(brazo) + A3 * sin(antebrazo) + D5 * sin(pinza)
    pitch = q[1] + q[2] + q[3]
    return r, z, pitch


def fk(q):
    r, z, pitch = en_el_plano(q)
    base = radians(q[0])
    return [r * cos(base), r * sin(base), z, pitch]


def ik(pose):
    """Devuelve (codigo, q). codigo: OK, FUERA_DE_ALCANCE o FUERA_DE_LIMITES.
    Da una sola solución: la de codo arriba."""
    x, y, z, pitch = pose

    # 1. Centro de la muñeca: se retrocede D5 desde la punta. No se retrocede en la
    #    dirección del pitch sino φ grados por debajo, que es donde está la punta real.
    pinza = radians(pitch - FI)
    rw = hypot(x, y) - D5 * cos(pinza)
    zw = z - D1 - D5 * sin(pinza)

    # 2. Codo: ley de cosenos en el triángulo hombro-codo-muñeca
    cos_codo = (rw**2 + zw**2 - A2**2 - A3**2) / (2 * A2 * A3)
    if cos_codo < -1.0001 or cos_codo > 1.0001:
        return FUERA_DE_ALCANCE, None
    # Con el brazo casi estirado, el redondeo puede dar apenas más que 1: se recorta
    cos_codo = min(max(cos_codo, -1), 1)
    codo = -acos(cos_codo)   # negativo = codo arriba

    # 3. Hombro: ángulo hacia la muñeca menos el ángulo que agrega el codo
    hombro = atan2(zw, rw) - atan2(A3 * sin(codo), A2 + A3 * cos(codo))

    # 4. Base y muñeca
    q = [0.0] * N
    q[0] = degrees(atan2(y, x))
    q[1] = degrees(hombro)
    q[2] = degrees(codo)
    q[3] = pitch - q[1] - q[2]

    for i in range(N):
        if q[i] < QMIN[i] or q[i] > QMAX[i]:
            return FUERA_DE_LIMITES, q
    return OK, q


def punta_adelante(q):
    """True si la punta queda delante del eje de la base.
    Si queda detrás (hombro pasado de la vertical), la ik no reproduce esa configuración:
    gira la base 180° y busca otra. Las pruebas descartan esos casos."""
    r, z, pitch = en_el_plano(q)
    return r > 0
