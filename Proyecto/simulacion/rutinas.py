# Las tres rutinas de manipulación que pide el enunciado:
#   tomar objeto de mesa · depositar en plataforma de carga · presionar botón
#
# Cada rutina es una lista de pasos. Los pasos de movimiento se dan como poses cartesianas
# (x, y, z, pitch), como la pose que recibe ik() en el firmware, y acá se pasan a ángulos con la ik.
# Antes de mover, se revisa TODO el camino con trayectorias.camino_seguro(): si un tramo choca,
# la rutina se detiene ahí y dice contra qué. No busca caminos alternativos, igual que irA()
# en el ESP32, pero el firmware solo revisa su piso virtual; no los obstáculos de PyBullet.
#
# Unidades: mm y grados.
import time
from math import sin, cos, atan2, radians
import escena
import trayectorias
from parametros import QMIN, QMAX, REPOSO

# Pose de transporte, la misma que REPOSO: deja la punta a 165 mm del eje y no choca con
# ningún escenario.
PLEGADO = REPOSO

# Altura de agarre sobre la tapa. Más abajo, el bastidor de la pinza roza la mesa.
ALTURA_AGARRE = 30
PITCH_AGARRE = 0            # pinza horizontal: con la pinza inclinada las puntas tocan la mesa
RETIRO = 60                 # cuánto se aleja la pinza para aproximarse y para levantar

# Punto de soltar: sobre la caja de carga de RAUL
ALTURA_SOLTAR = 80
PITCH_SOLTAR = -45

# Botón del ascensor: se presiona con la punta de los dedos, con la pinza cerrada
PITCH_BOTON = 0
# 120 y no menos: el camino desde el reposo no es recto y la punta se abre a mitad de
# tramo. Con 70 las mordazas rozaban el panel.
RETIRO_BOTON = 120


def pose_para_agarrar(punto, pitch):
    """Pose del TCP para agarrar algo centrado en `punto`.
    La punta va OFFSET_AGARRE más allá, en la dirección en la que apunta la pinza.
    OFFSET_AGARRE es negativo: esta pinza muerde la cara de adelante, no envuelve."""
    x, y, z = punto
    base = atan2(y, x)
    d = escena.OFFSET_AGARRE
    return [x + d * cos(radians(pitch)) * cos(base),
            y + d * cos(radians(pitch)) * sin(base),
            z + d * sin(radians(pitch)), pitch]


def _atras(punto, distancia):
    """El mismo punto, pero `distancia` más cerca del eje de la base (para aproximarse)."""
    x, y, z = punto
    base = atan2(y, x)
    return (x - distancia * cos(base), y - distancia * sin(base), z)


def _arriba(punto, distancia):
    x, y, z = punto
    return (x, y, z + distancia)


# Pasos: ("mover", texto, pose, segundos) · ("agarrar", texto) · ("soltar", texto)
#        ("pinza", texto, abierta) · ("escenario", texto, nombre)

def tomar_de_mesa():
    """Tomar un objeto de la mesa de trabajo."""
    agarre = (escena.OBJETO_EN_MESA[0], escena.OBJETO_EN_MESA[1],
              escena.TAPA_MESA + ALTURA_AGARRE)
    return [
        ("escenario", "poner la mesa y el objeto", "mesa"),
        ("pinza", "abrir la pinza", True),
        # Por arriba y no de frente: el camino recto desde el reposo barre la tapa.
        ("mover", "subir por encima de la mesa",
         pose_para_agarrar(_arriba(_atras(agarre, RETIRO), RETIRO), PITCH_AGARRE), 3),
        ("mover", "bajar frente al objeto",
         pose_para_agarrar(_atras(agarre, RETIRO), PITCH_AGARRE), 1.5),
        ("mover", "entrar sobre el objeto",
         pose_para_agarrar(agarre, PITCH_AGARRE), 1.5),
        ("agarrar", "cerrar la pinza sobre el objeto"),
        ("mover", "levantar el objeto",
         pose_para_agarrar(_arriba(agarre, RETIRO), PITCH_AGARRE), 1.5),
        ("mover", "retirar de la mesa",
         pose_para_agarrar(_arriba(_atras(agarre, RETIRO), RETIRO), PITCH_AGARRE), 2),
    ]


def depositar_en_carga():
    """Depositar en la caja de carga de RAUL. Arranca con el objeto ya agarrado."""
    centro, medidas = escena.CAJA_CARGA
    tapa = centro[2] + medidas[2] / 2
    soltar = (centro[0], centro[1], tapa + ALTURA_SOLTAR)
    return [
        ("mover", "llevar el objeto sobre la caja de carga",
         pose_para_agarrar(_arriba(soltar, RETIRO), PITCH_SOLTAR), 5),
        ("mover", "bajar hasta el punto de soltar",
         pose_para_agarrar(soltar, PITCH_SOLTAR), 2),
        ("soltar", "abrir la pinza y soltar el objeto"),
        ("mover", "retirar la pinza",
         pose_para_agarrar(_arriba(soltar, RETIRO), PITCH_SOLTAR), 2),
    ]


def presionar_boton():
    """Presionar el botón del ascensor. Se presiona con la punta de los dedos cerrados,
    así que acá la pose es la del TCP directamente, sin el offset del agarre."""
    x, y, z = escena.BOTON
    base = atan2(y, x)
    # La punta queda justo delante de la cara del panel; el botón sobresale hacia el brazo.
    punta = (x - escena.MARGEN, y, z)
    atras = _atras(punta, RETIRO_BOTON)
    return [
        ("escenario", "ir al ascensor", "ascensor"),
        ("pinza", "cerrar la pinza para presionar", False),
        ("mover", "aproximar al panel", [atras[0], atras[1], atras[2], PITCH_BOTON], 4),
        ("mover", "presionar el boton", [punta[0], punta[1], punta[2], PITCH_BOTON], 1.5),
        ("mover", "soltar el boton", [atras[0], atras[1], atras[2], PITCH_BOTON], 1.5),
    ]


RUTINAS = {
    "tomar": tomar_de_mesa,
    "depositar": depositar_en_carga,
    "boton": presionar_boton,
}


def poses_tarea():
    """TCP de los pasos efectivos; fuente común para mapas, errores e informe."""
    return {
        "tomar en mesa": next(p[2] for p in tomar_de_mesa()
                              if p[1] == "entrar sobre el objeto"),
        "soltar en caja": next(p[2] for p in depositar_en_carga()
                              if p[1] == "bajar hasta el punto de soltar"),
        "boton ascensor": next(p[2] for p in presionar_boton()
                              if p[1] == "presionar el boton"),
    }


def ejecutar(pasos, q_inicial, animar=False, hz=50):
    """Recorre los pasos desde la configuración q_inicial.
    Devuelve (q_final, informe), donde informe es una lista de (texto, 'ok'|motivo).
    Se detiene en el primer paso que falla."""
    q = list(q_inicial)
    informe = []

    for paso in pasos:
        tipo, texto = paso[0], paso[1]

        if tipo == "escenario":
            escena.poner_escenario(paso[2])
            if paso[2] == "mesa":
                escena.poner_objeto()
            informe.append((texto, "ok"))
            continue

        if tipo == "pinza":
            escena.abrir_pinza() if paso[2] else escena.cerrar_pinza()
            informe.append((texto, "ok"))
            continue

        if tipo == "agarrar":
            ok = escena.agarrar()
            informe.append((texto, "ok" if ok else "no habia objeto que agarrar"))
            if not ok:
                return q, informe
            continue

        if tipo == "soltar":
            ok = escena.soltar()
            informe.append((texto, "ok" if ok else "no habia nada agarrado"))
            if not ok:
                return q, informe
            continue

        pose, segundos = paso[2], paso[3]
        destino, motivo = trayectorias.angulos(pose)
        if destino is None:
            informe.append((texto, motivo))
            return q, informe

        libre, problemas = trayectorias.camino_seguro(q, destino)
        if not libre:
            escena.mover(q)
            informe.append((texto, trayectorias.resumen(problemas)))
            return q, informe

        if animar:
            _animar(q, destino, segundos, hz)
        escena.mover(destino)
        q = destino
        informe.append((texto, "ok"))

    return q, informe


def _animar(q_a, q_b, segundos, hz):
    tiempos, angulos, _, _ = trayectorias.evolucion(q_a, q_b, segundos, hz)
    for t, q in zip(tiempos, angulos):
        escena.mover(q)
        time.sleep(1 / hz)


def ir_a(q_destino, q_inicial, segundos=3, animar=False, hz=50):
    """Mover entre dos configuraciones, revisando el camino. Para ir y volver del reposo."""
    libre, problemas = trayectorias.camino_seguro(q_inicial, q_destino)
    if not libre:
        escena.mover(q_inicial)
        return q_inicial, trayectorias.resumen(problemas)
    if animar:
        _animar(q_inicial, q_destino, segundos, hz)
    escena.mover(q_destino)
    return q_destino, "ok"
