# Simulación en modo cartesiano: se mueve la punta de la pinza con sliders (x, y, z, pitch)
# y el brazo la sigue usando la ik. La interfaz manual del firmware está pendiente.
# Si la pose es imposible (fuera de alcance, de límites o con choque) el brazo no se mueve
# y el texto se pone rojo.
import time
import pybullet as p
import escena
from cinematica import ik, OK, FUERA_DE_ALCANCE
from parametros import N

escena.cargar(con_ventana=True)
# Mostrar el panel con los controles manuales.
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# Sliders: nombre, mínimo, máximo, valor inicial
# Los rangos siguen el alcance real del brazo: 450 mm desde el hombro.
slider_x     = p.addUserDebugParameter("x [mm]", -500, 500, 300)
slider_y     = p.addUserDebugParameter("y [mm]", -500, 500, 0)
slider_z     = p.addUserDebugParameter("z [mm]", -200, 600, 200)
slider_pitch = p.addUserDebugParameter("pitch [grados]", -90, 90, -45)

# Esfera roja que marca el objetivo
forma_marca = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 0.6])
marca = p.createMultiBody(0, baseVisualShapeIndex=forma_marca)

while p.isConnected():
    x     = p.readUserDebugParameter(slider_x)
    y     = p.readUserDebugParameter(slider_y)
    z     = p.readUserDebugParameter(slider_z)
    pitch = p.readUserDebugParameter(slider_pitch)

    p.resetBasePositionAndOrientation(marca, [escena.mm(x), escena.mm(y), escena.mm(z)], [0, 0, 0, 1])
    codigo, q = ik([x, y, z, pitch])

    if codigo == FUERA_DE_ALCANCE:
        escena.mostrar_estado("fuera de alcance", False)
    elif codigo != OK:
        angulos = "  ".join(f"{v:.0f}" for v in q)
        escena.mostrar_estado(f"fuera de limites: q = [{angulos}]", False)
    else:
        q_anterior = [p.getJointState(escena.robot, i)[0] for i in range(N)]
        escena.mover(q)
        lista = escena.choques()
        if lista:
            # Vuelve a la posición anterior: no se muestra el brazo atravesando nada
            for i in range(N):
                p.resetJointState(escena.robot, i, q_anterior[i])
            escena.mostrar_estado("choque: " + ", ".join(lista), False)
        else:
            angulos = "  ".join(f"{v:.0f}" for v in q)
            escena.mostrar_estado(f"ok: q = [{angulos}]", True)

    time.sleep(1 / 30)
