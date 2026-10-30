# Trayectorias entre dos configuraciones del brazo.
#
# El brazo va en línea recta en el espacio de ÁNGULOS, no en el espacio cartesiano: se interpola
# cada articulación entre su valor inicial y el final. Es lo mismo que hace irA() en el firmware.
#
# La idea que ordena todo este módulo: el perfil de 5.° grado solo cambia EL TIEMPO, no el camino.
# Sea cual sea la duración, el brazo recorre siempre el mismo segmento recto en el espacio de
# ángulos; el perfil decide a qué velocidad lo recorre. Por eso la revisión de choques
# (muestrear / camino_seguro) no necesita saber cuánto va a durar el movimiento, y el resultado
# vale para cualquier velocidad.
#
# Unidades: grados y mm, como en todo el proyecto.
from math import ceil
import escena
from cinematica import ik, OK, FUERA_DE_ALCANCE
from parametros import QMIN, QMAX, N, NOMBRES

PASO_REVISION = 2   # grados de la articulación que más se mueve entre dos revisiones


# Perfil de 5.° grado, el mismo que movimiento.cpp.
# Con t de 0 a 1 devuelve de 0 a 1, arrancando y terminando con velocidad y aceleración cero:
# por eso el brazo no da tirones y no excita vibraciones en la estructura.
def perfil(t):
    return t * t * t * (10 - 15 * t + 6 * t * t)


def perfil_v(t):
    """Derivada del perfil. Vale 0 en los extremos y 1,875 en el medio: ese es el factor
    entre la velocidad máxima y la velocidad media de cualquier movimiento."""
    return 30 * t * t * (1 - t) ** 2


def perfil_a(t):
    """Segunda derivada. Vale 0 en los extremos y cambia de signo en el medio
    (primero acelera, después frena)."""
    return 60 * t * (1 - t) * (1 - 2 * t)


def angulos(pose):
    """Pose cartesiana → ángulos. Devuelve (q, None) o (None, motivo), donde el motivo
    dice qué articulación se pasó, que es lo que hace falta para arreglar la rutina."""
    codigo, q = ik(pose)
    if codigo == OK:
        return q, None
    if codigo == FUERA_DE_ALCANCE:
        return None, "fuera de alcance"
    pasados = [f"{NOMBRES[i]} a {q[i]:.0f}° (limite {QMIN[i]} a {QMAX[i]})"
               for i in range(N) if q[i] < QMIN[i] or q[i] > QMAX[i]]
    return None, "fuera de limites: " + ", ".join(pasados)


def muestrear(q_a, q_b, paso=PASO_REVISION):
    """Puntos del segmento recto entre dos configuraciones, uno cada `paso` grados de la
    articulación que más se mueve. Incluye los dos extremos. Es geometría pura: no sabe de tiempo."""
    recorrido = max(abs(b - a) for a, b in zip(q_a, q_b))
    tramos = max(10, ceil(recorrido / paso))
    return [[a + (b - a) * k / tramos for a, b in zip(q_a, q_b)] for k in range(tramos + 1)]


def camino_seguro(q_a, q_b, paso=PASO_REVISION):
    """Revisa TODO el camino, no solo las puntas.
    Devuelve (True, []) si está libre, o (False, [(fraccion, [choques]), ...]) con los puntos
    del recorrido donde se rompe y contra qué choca. fraccion: 0 = salida, 1 = llegada."""
    puntos = muestrear(q_a, q_b, paso)
    ultimo = len(puntos) - 1
    problemas = []
    # Se arranca en 1, no en 0: el punto de salida es donde el brazo YA está, y si fuera
    # inseguro nunca habría llegado ahí. Es lo mismo que hace irA() en el firmware,
    # que revisa los pasos 1 a 20 del camino y no el 0.
    for k in range(1, ultimo + 1):
        escena.mover(puntos[k])
        lista = escena.choques()
        if lista:
            problemas.append((k / ultimo, lista))
    return not problemas, problemas


def resumen(problemas):
    """Una línea legible con dónde y contra qué se rompe un camino."""
    if not problemas:
        return "libre"
    desde = problemas[0][0]
    hasta = problemas[-1][0]
    contra = sorted({c for _, lista in problemas for c in lista})
    return f"choca entre el {desde:.0%} y el {hasta:.0%} del camino: " + ", ".join(contra)


def velocidades_pico(q_a, q_b, segundos):
    """Velocidad máxima de cada articulación, en °/s, sin tener que muestrear nada:
    el perfil de 5.° grado tiene su pico en la mitad y vale 1,875 veces la velocidad media.
    Sirve para contrastar con VELOCIDAD_MAX de configuracion.h: si un movimiento pide más
    de lo que el motor puede dar, el brazo real se queda atrás y pierde pasos."""
    return [1.875 * abs(b - a) / segundos for a, b in zip(q_a, q_b)]


def tiempo_minimo(q_a, q_b, velocidad_max):
    """Cuánto tiene que durar como mínimo un movimiento para que ninguna articulación
    pase de velocidad_max (°/s)."""
    recorrido = max(abs(b - a) for a, b in zip(q_a, q_b))
    return 1.875 * recorrido / velocidad_max


def evolucion(q_a, q_b, segundos, hz=50):
    """Cómo evoluciona el movimiento en el tiempo, muestreado a `hz` (50 Hz es lo que tarda
    el PCA9685 en repetir el pulso, o sea el ritmo al que el ESP32 renueva los servos).
    Devuelve (tiempos, angulos, velocidades, aceleraciones), cada uno con una fila por instante
    y una columna por articulación. Velocidades en °/s, aceleraciones en °/s²."""
    muestras = max(2, int(segundos * hz) + 1)
    tiempos, angulos_, velocidades, aceleraciones = [], [], [], []
    for k in range(muestras):
        t = segundos * k / (muestras - 1)
        s = t / segundos
        tiempos.append(t)
        angulos_.append([a + (b - a) * perfil(s) for a, b in zip(q_a, q_b)])
        velocidades.append([(b - a) * perfil_v(s) / segundos for a, b in zip(q_a, q_b)])
        aceleraciones.append([(b - a) * perfil_a(s) / segundos ** 2 for a, b in zip(q_a, q_b)])
    return tiempos, angulos_, velocidades, aceleraciones
