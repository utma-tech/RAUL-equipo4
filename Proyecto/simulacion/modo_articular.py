# Simulación en modo articular: un slider por articulación, dentro de sus límites.
# Sirve para explorar a mano hasta dónde puede ir cada articulación sin chocar.
# A diferencia del modo cartesiano, acá el brazo sí se muestra chocando (en rojo),
# para ver con qué pieza choca.
import time
import pybullet as p
import escena
from cinematica import fk
from parametros import QMIN, QMAX

escena.cargar(con_ventana=True)
# Mostrar el panel con los controles manuales.
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

from parametros import N, NOMBRES as nombres
sliders = []
for i in range(N):
    slider = p.addUserDebugParameter(f"{nombres[i]} [grados]", QMIN[i], QMAX[i], 0)
    sliders.append(slider)

while p.isConnected():
    q = [p.readUserDebugParameter(s) for s in sliders]
    escena.mover(q)

    lista = escena.choques()
    if lista:
        escena.mostrar_estado("choque: " + ", ".join(lista), False)
    else:
        x, y, z, pitch = fk(q)
        escena.mostrar_estado(f"ok: punta en ({x:.0f}, {y:.0f}, {z:.0f}) mm, pitch {pitch:.0f}", True)

    time.sleep(1 / 30)
