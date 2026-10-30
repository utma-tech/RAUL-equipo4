# Pruebas de la cinemática y del modelo. Correr desde la carpeta simulacion:  python pruebas.py
#   1. La forma cerrada (fk) coincide con la tabla DH (fk_dh).
#   2. fk e ik son inversas: fk(ik(pose)) devuelve la misma pose.
#   3. brazo.urdf describe el mismo brazo que la tabla DH (comparando contra PyBullet),
#      incluida la caída de 18 mm de la pinza respecto del eje de la muñeca.
#   4. Los límites de brazo.urdf coinciden con los de parametros.py.
#   5. Genera ../pruebas/casos.csv para verificar la versión en C del firmware.
import sys
import random
from pathlib import Path
from math import sin, cos, radians
import numpy as np
import pybullet as p
import escena
from cinematica import fk, fk_dh, ik, punta_adelante, OK, FUERA_DE_ALCANCE
from parametros import QMIN, QMAX, N

CASOS = 2000
TCP = escena.TCP   # índice del eslabón "tcp" (punta de las mordazas) en PyBullet
random.seed(1)   # mismos casos en cada corrida


def angulos_al_azar():
    return [random.uniform(QMIN[i], QMAX[i]) for i in range(N)]


def direccion_pinza(q):
    """Vector unitario hacia donde apunta la pinza, a partir del ángulo de la base y del pitch.
    Es la dirección del antebrazo: la pinza va paralela a él, 18 mm más abajo."""
    base = radians(q[0])
    pitch = radians(q[1] + q[2] + q[3])
    return [cos(pitch) * cos(base), cos(pitch) * sin(base), sin(pitch)]


def informar(nombre, error, tolerancia):
    paso = error < tolerancia
    estado = "OK   " if paso else "FALLA"
    print(f"{estado} {nombre}: error máximo {error:.1e}")
    return paso


def prueba_forma_cerrada_vs_dh():
    error = 0
    for _ in range(CASOS):
        q = angulos_al_azar()
        T = fk_dh(q)
        x, y, z, pitch = fk(q)
        error_posicion = np.abs(T[:3, 3] - [x, y, z]).max()
        error_direccion = np.abs(T[:3, 0] - direccion_pinza(q)).max()
        error = max(error, error_posicion, error_direccion)
    return informar("forma cerrada vs tabla DH", error, 1e-6)


def prueba_ida_y_vuelta():
    error = 0
    for _ in range(CASOS):
        q = angulos_al_azar()
        if not punta_adelante(q):
            continue
        pose = fk(q)
        codigo, q_ik = ik(pose)
        if codigo == FUERA_DE_ALCANCE:
            print(f"FALLA ik dice fuera de alcance para una pose alcanzable: q = {q}")
            return False
        if codigo == OK:
            error = max(error, np.abs(np.array(fk(q_ik)) - pose).max())
    return informar("fk(ik(pose)) = pose", error, 1e-6)


def prueba_urdf_vs_dh():
    error_posicion = 0    # mm
    error_direccion = 0   # componentes de un vector unitario
    for _ in range(CASOS):
        q = angulos_al_azar()
        escena.mover(q)
        estado = p.getLinkState(escena.robot, TCP, computeForwardKinematics=True)
        posicion = np.array(estado[4]) * 1000                     # m → mm
        rotacion = np.array(p.getMatrixFromQuaternion(estado[5])).reshape(3, 3)
        direccion = rotacion[:, 0]                                # en el URDF la pinza apunta sobre x

        T = fk_dh(q)
        error_posicion = max(error_posicion, np.abs(posicion - T[:3, 3]).max())
        error_direccion = max(error_direccion, np.abs(direccion - T[:3, 0]).max())
    posicion_ok = informar("brazo.urdf vs tabla DH, posición [mm]", error_posicion, 0.01)
    direccion_ok = informar("brazo.urdf vs tabla DH, dirección de la pinza", error_direccion, 1e-4)
    return posicion_ok and direccion_ok


def prueba_limites_urdf():
    error = 0
    for i in range(N):
        info = p.getJointInfo(escena.robot, i)
        minimo, maximo = info[8], info[9]
        error = max(error, abs(minimo - radians(QMIN[i])), abs(maximo - radians(QMAX[i])))
    return informar("límites de brazo.urdf vs parametros.py", error, 1e-3)


def generar_casos_para_c():
    # Fila 1: límites de Python (QMIN y QMAX), para que el programa en C los compare con configuracion.h.
    # Resto: q[4] y la pose[4] que calculó Python.
    ruta = Path(__file__).parent.parent / "pruebas" / "casos.csv"
    filas = [QMIN + QMAX]
    while len(filas) < 1000:
        q = angulos_al_azar()
        if punta_adelante(q):
            filas.append(q + fk(q))
    with open(ruta, "w") as archivo:
        for fila in filas:
            archivo.write(",".join(f"{v:.6f}" for v in fila) + "\n")
    print(f"      casos para C guardados en {ruta}")


escena.cargar(con_ventana=False)
resultados = [
    prueba_forma_cerrada_vs_dh(),
    prueba_ida_y_vuelta(),
    prueba_urdf_vs_dh(),
    prueba_limites_urdf(),
]
generar_casos_para_c()

if not all(resultados):
    sys.exit(1)
