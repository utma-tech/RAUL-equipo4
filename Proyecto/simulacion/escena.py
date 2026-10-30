# Escena de PyBullet: piso, plataforma de RAUL, caja de carga y el brazo.
# Detecta choques del brazo consigo mismo y con el entorno, abre y cierra la pinza,
# y agarra y suelta el objeto.
#
# Medidas en mm, respecto a la base del brazo: z = 0 es la cara inferior de la placa del brazo, sobre el alzador.
# Todas las medidas del entorno son PROVISIONALES: corregirlas cuando se conozca RAUL real.
from pathlib import Path
from math import radians
from parametros import N
import pybullet as p
import pybullet_data

ALTURA_PLATAFORMA = 350   # piso del laboratorio → superficie de la plataforma de RAUL

# Cuánto se levanta la placa del brazo sobre la plataforma de RAUL. 233 mm es la altura propuesta
# para el escenario de demostración; no es una medida del CAD ni un mínimo global (ver GUIA_EQUIPO.md).
ALTURA_MONTAJE = 233

# Piso del laboratorio → cara de montaje de la placa del brazo, que es el z = 0 de todo esto
ALTURA_SUELO = ALTURA_PLATAFORMA + ALTURA_MONTAJE

MARGEN = 10               # distancia mínima permitida entre piezas antes de considerarlo choque

# El objeto agarrado se revisa aparte, sin margen: tiene que poder TOCAR superficies,
# porque se lo levanta de una y se lo apoya en otra. Acá solo interesa si lo estamos
# metiendo dentro de algo. El margen de 10 mm sigue valiendo para todo el brazo.
MARGEN_OBJETO = 0

# Obstáculos como cajas: (centro x, y, z), (largo x, ancho y, alto z)
PLATAFORMA = ((-100, 0, -ALTURA_MONTAJE - ALTURA_PLATAFORMA / 2),
              (500, 400, ALTURA_PLATAFORMA))
PEDESTAL = ((0, 0, -ALTURA_MONTAJE / 2), (160, 160, ALTURA_MONTAJE))

# Al frente y a la izquierda, a 64° del eje x: atrás no llega porque la base gira 180°.
# Alta como el montaje, para que la tapa quede al nivel de la placa del brazo.
CAJA_CARGA = ((80, 165, -ALTURA_MONTAJE / 2), (140, 160, ALTURA_MONTAJE))

# Alturas del entorno, medidas desde el PISO del laboratorio (es como se miden con un metro).
# Para pasarlas a la referencia del brazo hay que restarles ALTURA_SUELO.
ALTURA_MESA = 750         # piso → tapa de la mesa de trabajo
ALTURA_BOTON = 1000       # piso → botón del ascensor


def desde_el_piso(altura):
    """Altura medida desde el piso → altura en la referencia del brazo
    (z = 0 en la cara de montaje de la placa, arriba del pedestal)."""
    return altura - ALTURA_SUELO


# La mesa y el panel están en lugares distintos del ITR, así que no van a la vez: se
# cambia con poner_escenario(). BORDE_MESA y CARA_PANEL son a cuánto se arrima RAUL.
TAPA_MESA = desde_el_piso(ALTURA_MESA)
BORDE_MESA = 270
CARA_PANEL = 230
ESCENARIOS = {
    "mesa": {
        "mesa": ((BORDE_MESA + 300, 0, (TAPA_MESA - ALTURA_SUELO) / 2),
                 (600, 600, TAPA_MESA + ALTURA_SUELO), [0.6, 0.45, 0.3, 1]),
    },
    "ascensor": {
        "panel": ((CARA_PANEL + 20, 0, -ALTURA_SUELO + 675),
                  (40, 800, 1350), [0.55, 0.55, 0.6, 1]),
    },
}

# Objeto a manipular: un cubo de 50 mm apoyado sobre la tapa de la mesa
LADO_OBJETO = 50
OBJETO_EN_MESA = (320, 0, TAPA_MESA + LADO_OBJETO / 2)

# Botón del ascensor: solo una marca para ver a dónde va la punta. No se revisa como obstáculo,
# porque presionarlo es justamente tocarlo.
BOTON = (CARA_PANEL - 5, 0, desde_el_piso(ALTURA_BOTON))

# Índice de cada eslabón en PyBullet (-1 es la raíz del URDF). El orden es q1 base,
# q2 hombro, q3 codo, q4 muñeca, tcp y los dos dedos. "pinza" cuelga de q4 e incluye las
# placas de la muñeca, porque sin giro no hay nada que las separe.
ESLABON = {"base": -1, "torre": 0, "brazo": 1, "antebrazo": 2, "pinza": 3,
           "dedo izq": 5, "dedo der": 6}
TCP = 4

# Apertura de cada mordaza, del centro al eje de la pinza (m, como el URDF). Cerrada es
# como está en el CAD: 14,8 mm de luz. Abierta, 50 mm. Falta confirmar cuánto abre.
DEDO_ABIERTO = 0.0315
DEDO_CERRADO = 0.0139

# Distancia del centro del objeto al TCP. Negativa porque la pinza no envuelve: muerde la
# cara de adelante, y entre la punta y el bastidor hay 12 mm. Al cubo de 50 mm lo agarra
# con un bocado de 10.
OFFSET_AGARRE = -15

# Pares de eslabones del propio brazo que pueden chocar.
# Los eslabones vecinos (torre-brazo, brazo-antebrazo, antebrazo-pinza) se tocan siempre en
# la articulación que los une, por eso no se revisan. Consecuencia: el codo plegado al
# máximo (brazo contra antebrazo) no se detecta acá, lo limita QMIN del codo.
# Los dedos sí se revisan contra el antebrazo: están 50 mm más adelante que la muñeca,
# así que con la muñeca doblada llegan a tocarlo de verdad.
PARES_PROPIOS = [
    ("antebrazo", "torre"),
    ("antebrazo", "base"),
    ("pinza", "torre"),
    ("pinza", "base"),
    ("pinza", "brazo"),
    ("dedo izq", "torre"), ("dedo izq", "base"), ("dedo izq", "brazo"), ("dedo izq", "antebrazo"),
    ("dedo der", "torre"), ("dedo der", "base"), ("dedo der", "brazo"), ("dedo der", "antebrazo"),
]

# Eslabones que se mueven y pueden chocar con el entorno
MOVILES = ["brazo", "antebrazo", "pinza", "dedo izq", "dedo der"]

# Los dedos tienen que poder tocar el objeto: es lo que hacen al agarrarlo.
# El resto del brazo no, y eso sí se revisa (por ejemplo, voltear la caja con el antebrazo).
# La pinza y los dedos quedan afuera del control contra el objeto: tocarlo es lo que
# hacen al agarrarlo. Contra el entorno se revisan con el margen completo.
SIN_REVISAR_CONTRA_OBJETO = ["dedo izq", "dedo der", "pinza"]

robot = None
obstaculos = {}        # nombre → id de PyBullet
escenario_actual = None
objeto = None          # id del cubo, o None si no está puesto
agarrado = False       # True mientras el objeto viaja con la pinza
agarre_local = None    # pose del objeto vista desde la pinza, congelada al agarrarlo
marca_boton = None
texto_estado = -1


def mm(valor):
    return valor / 1000


def crear_caja(centro, medidas, color):
    mitad = [mm(m) / 2 for m in medidas]
    forma = p.createCollisionShape(p.GEOM_BOX, halfExtents=mitad)
    vista = p.createVisualShape(p.GEOM_BOX, halfExtents=mitad, rgbaColor=color)
    posicion = [mm(c) for c in centro]
    return p.createMultiBody(0, forma, vista, posicion)


def cargar(con_ventana, opciones=""):
    """Arma la escena. con_ventana=False para pruebas sin interfaz gráfica."""
    global robot

    if con_ventana:
        p.connect(p.GUI, options=opciones)
        # Encuadre para un brazo de 45 cm: de más lejos no se ve lo que hace la pinza.
        p.resetDebugVisualizerCamera(0.78, 40, -16, [0.18, 0.03, 0.14])
        # Sin los paneles del ExampleBrowser
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # Oculta los paneles de cámara que PyBullet muestra por defecto
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    obstaculos["piso"] = p.loadURDF("plane.urdf", [0, 0, -mm(ALTURA_SUELO)])
    obstaculos["plataforma"] = crear_caja(*PLATAFORMA, color=[0.2, 0.2, 0.2, 1])
    obstaculos["pedestal"] = crear_caja(*PEDESTAL, color=[0.25, 0.25, 0.28, 1])
    obstaculos["caja de carga"] = crear_caja(*CAJA_CARGA, color=[0.8, 0.6, 0.3, 0.8])

    ruta_urdf = Path(__file__).parent / "brazo.urdf"
    robot = p.loadURDF(str(ruta_urdf), useFixedBase=True)
    abrir_pinza()


def poner_escenario(nombre):
    """Cambia la mesa por el panel del ascensor, o al revés. nombre=None deja solo a RAUL."""
    global escenario_actual, marca_boton

    for clave in list(obstaculos):
        if clave not in ("piso", "plataforma", "pedestal", "caja de carga"):
            p.removeBody(obstaculos.pop(clave))
    if marca_boton is not None:
        p.removeBody(marca_boton)
        marca_boton = None

    if nombre:
        for clave, (centro, medidas, color) in ESCENARIOS[nombre].items():
            obstaculos[clave] = crear_caja(centro, medidas, color)
    if nombre == "ascensor":
        forma = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[0.9, 0.9, 0.2, 1])
        marca_boton = p.createMultiBody(0, baseVisualShapeIndex=forma,
                                        basePosition=[mm(v) for v in BOTON])
    escenario_actual = nombre


def poner_objeto(centro=OBJETO_EN_MESA):
    """Pone el cubo a agarrar en esa posición (mm). Lo saca de la pinza si estaba agarrado."""
    global objeto
    soltar()
    if objeto is not None:
        p.removeBody(objeto)
    objeto = crear_caja(centro, (LADO_OBJETO,) * 3, [0.2, 0.7, 0.3, 1])
    return objeto


def mover(q):
    """Pone el brazo en los ángulos q (grados), sin simular dinámica."""
    for i in range(N):
        p.resetJointState(robot, i, radians(q[i]))
    if agarrado:
        _reubicar_objeto()


def _mover_dedos(apertura):
    p.resetJointState(robot, ESLABON["dedo izq"], apertura)
    p.resetJointState(robot, ESLABON["dedo der"], apertura)


def abrir_pinza():
    _mover_dedos(DEDO_ABIERTO)


def cerrar_pinza():
    _mover_dedos(DEDO_CERRADO)


def pinza_abierta():
    return p.getJointState(robot, ESLABON["dedo izq"])[0] > (DEDO_ABIERTO + DEDO_CERRADO) / 2


def _reubicar_objeto():
    """Lleva el objeto a donde está la pinza, manteniendo la pose relativa del agarre.
    Se hace a mano, y no con una restricción de PyBullet, porque acá nunca se llama a
    stepSimulation(): el brazo se posiciona con resetJointState y no hay dinámica,
    así que una restricción no movería nada."""
    estado = p.getLinkState(robot, ESLABON["pinza"], computeForwardKinematics=True)
    posicion, orientacion = p.multiplyTransforms(estado[4], estado[5], *agarre_local)
    p.resetBasePositionAndOrientation(objeto, posicion, orientacion)


def agarrar():
    """Cierra la pinza y engancha el objeto. Devuelve False si no hay objeto o ya estaba agarrado."""
    global agarrado, agarre_local
    if objeto is None or agarrado:
        return False
    cerrar_pinza()
    estado = p.getLinkState(robot, ESLABON["pinza"], computeForwardKinematics=True)
    inversa = p.invertTransform(estado[4], estado[5])
    pose_objeto = p.getBasePositionAndOrientation(objeto)
    agarre_local = p.multiplyTransforms(*inversa, *pose_objeto)
    agarrado = True
    return True


def _objeto_libre(z, posicion, orientacion):
    p.resetBasePositionAndOrientation(objeto, [posicion[0], posicion[1], z], orientacion)
    return not any(p.getClosestPoints(objeto, o, 0) for o in obstaculos.values())


def _apoyar_objeto():
    """Baja el objeto en línea recta hasta que se apoya en algo.
    Hace las veces de gravedad: acá no se llama a stepSimulation(), así que un objeto soltado
    en el aire se quedaría flotando y el brazo lo atravesaría al plegarse.
    Busca por bisección la altura más baja en la que todavía no toca nada."""
    posicion, orientacion = p.getBasePositionAndOrientation(objeto)
    arriba = posicion[2]
    abajo = -mm(ALTURA_SUELO) - 1            # bien debajo del piso: ahí seguro toca
    if _objeto_libre(abajo, posicion, orientacion):
        return                                # no hay nada debajo; se queda donde está
    for _ in range(30):
        medio = (arriba + abajo) / 2
        if _objeto_libre(medio, posicion, orientacion):
            arriba = medio
        else:
            abajo = medio
    p.resetBasePositionAndOrientation(objeto, [posicion[0], posicion[1], arriba], orientacion)


def soltar():
    """Abre la pinza, suelta el objeto y lo deja apoyarse.
    Devuelve False si no había nada agarrado."""
    global agarrado
    if not agarrado:
        return False
    abrir_pinza()
    agarrado = False
    _apoyar_objeto()
    return True


def choques():
    """Lista de choques en la posición actual, por ejemplo ["pinza con plataforma"].
    Lista vacía = posición segura."""
    distancia = mm(MARGEN)
    encontrados = []

    for a, b in PARES_PROPIOS:
        puntos = p.getClosestPoints(robot, robot, distancia, ESLABON[a], ESLABON[b])
        if puntos:
            encontrados.append(f"{a} con {b}")

    for nombre, obstaculo in obstaculos.items():
        for eslabon in MOVILES:
            puntos = p.getClosestPoints(robot, obstaculo, distancia, ESLABON[eslabon])
            if puntos:
                encontrados.append(f"{eslabon} con {nombre}")

    # El objeto, mientras no esté agarrado, es algo que no hay que voltear.
    # Los dedos quedan afuera: tocarlo es lo que tienen que hacer.
    if objeto is not None and not agarrado:
        for eslabon in MOVILES:
            if eslabon in SIN_REVISAR_CONTRA_OBJETO:
                continue
            if p.getClosestPoints(robot, objeto, distancia, ESLABON[eslabon]):
                encontrados.append(f"{eslabon} con objeto")

    # Agarrado, el objeto se mueve con el brazo y puede chocar contra el entorno.
    # Sin margen: rozar la mesa al levantarlo o la caja al apoyarlo es normal, meterse adentro no.
    if agarrado:
        for nombre, obstaculo in obstaculos.items():
            if p.getClosestPoints(objeto, obstaculo, mm(MARGEN_OBJETO)):
                encontrados.append(f"objeto con {nombre}")

    return encontrados


def es_segura(q):
    mover(q)
    return len(choques()) == 0


def mostrar_estado(texto, todo_bien):
    """Escribe un texto sobre el brazo: negro si está todo bien, rojo si no."""
    global texto_estado
    color = [0, 0, 0] if todo_bien else [1, 0, 0]
    texto_estado = p.addUserDebugText(texto, [-0.15, 0, 0.60], textColorRGB=color,
                                      replaceItemUniqueId=texto_estado)
