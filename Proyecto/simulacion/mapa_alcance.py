# Mapa de alcance del brazo montado sobre RAUL.
# 1. Prueba muchas configuraciones al azar dentro de los límites y marca dónde queda la punta:
#    verde si la posición es segura, rojo si choca. Guarda mapa_alcance.png.
# 2. Revisa los puntos de las tareas: para cada uno busca con qué inclinación de la pinza
#    (pitch) se puede llegar sin chocar.
# Correr desde la carpeta simulacion:  python mapa_alcance.py
from pathlib import Path
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import escena
import rutinas
from cinematica import fk, ik, OK
from parametros import QMIN, QMAX, N

CANTIDAD = 30000   # configuraciones a probar

# Puntos de las tareas (mm, respecto a la base del brazo). PROVISIONALES.
# Se sacan de escena.py para que no queden dos versiones de las mismas medidas:
# si ahí se mueve la caja de carga o cambia la altura de la mesa, esto acompaña.
# OJO: acá se mide el alcance del brazo montado sobre RAUL solo, sin la mesa ni el panel
# del ascensor. Que un punto sea alcanzable no quiere decir que la rutina completa entre:
# eso lo verifica pruebas_rutinas.py, que sí pone el escenario.
PUNTOS_TAREA = {nombre: pose[:3] for nombre, pose in rutinas.poses_tarea().items()}
random.seed(2026)


def pitches_posibles(x, y, z):
    """Inclinaciones de la pinza (cada 5°) con las que la punta llega a (x, y, z) sin chocar."""
    posibles = []
    for pitch in range(-90, 91, 5):
        codigo, q = ik([x, y, z, pitch])
        if codigo == OK and escena.es_segura(q):
            posibles.append(pitch)
    return posibles


def dibujar_obstaculo(eje, centro, medidas, horizontal, vertical):
    """Dibuja una caja de la escena como rectángulo. horizontal / vertical: índice del eje (0=x, 1=y, 2=z)."""
    esquina = (centro[horizontal] - medidas[horizontal] / 2, centro[vertical] - medidas[vertical] / 2)
    eje.add_patch(Rectangle(esquina, medidas[horizontal], medidas[vertical], fill=False, color="black"))


escena.cargar(con_ventana=False)

# ─── 1. Muestreo ──────────────────────────────────────────────
seguras = []
con_choque = []
for _ in range(CANTIDAD):
    q = [random.uniform(QMIN[i], QMAX[i]) for i in range(N)]
    punta = fk(q)
    if escena.es_segura(q):
        seguras.append(punta)
    else:
        con_choque.append(punta)

print(f"Configuraciones probadas: {CANTIDAD}")
print(f"  seguras:    {len(seguras)} ({100 * len(seguras) / CANTIDAD:.0f} %)")
print(f"  con choque: {len(con_choque)}")

# ─── 2. Puntos de las tareas ──────────────────────────────────
print("\nPuntos de las tareas:")
for nombre, (x, y, z) in PUNTOS_TAREA.items():
    posibles = pitches_posibles(x, y, z)
    if posibles:
        print(f"  {nombre:16s} ({x}, {y}, {z:.0f}): alcanzable con pitch entre {min(posibles)}° y {max(posibles)}°")
    else:
        print(f"  {nombre:16s} ({x}, {y}, {z:.0f}): NO alcanzable")

# ─── 3. Gráfico ───────────────────────────────────────────────
fig, (lateral, superior) = plt.subplots(1, 2, figsize=(14, 6))

# Vista lateral: solo las puntas cerca del plano y = 0 (|y| < 80 mm), para que no se mezclen profundidades
for puntos, color, etiqueta in ((con_choque, "red", "con choque"), (seguras, "green", "segura")):
    xs = [pt[0] for pt in puntos if abs(pt[1]) < 80]
    zs = [pt[2] for pt in puntos if abs(pt[1]) < 80]
    lateral.scatter(xs, zs, s=2, c=color, alpha=0.3, label=etiqueta)
dibujar_obstaculo(lateral, *escena.PLATAFORMA, horizontal=0, vertical=2)
dibujar_obstaculo(lateral, *escena.CAJA_CARGA, horizontal=0, vertical=2)
lateral.axhline(-escena.ALTURA_SUELO, color="black")
lateral.set_title("Vista lateral (franja |y| < 80 mm)")
lateral.legend(loc="lower right", markerscale=5)
lateral.set_xlabel("x [mm]")
lateral.set_ylabel("z [mm]")

# Vista superior: todas las puntas seguras
superior.scatter([pt[0] for pt in seguras], [pt[1] for pt in seguras], s=2, c="green", alpha=0.2)
dibujar_obstaculo(superior, *escena.PLATAFORMA, horizontal=0, vertical=1)
dibujar_obstaculo(superior, *escena.CAJA_CARGA, horizontal=0, vertical=1)
superior.set_title("Vista superior (puntas seguras)")
superior.set_xlabel("x [mm]")
superior.set_ylabel("y [mm]")

# Puntos de las tareas en ambas vistas
for nombre, (x, y, z) in PUNTOS_TAREA.items():
    lateral.plot(x, z, "b*", markersize=14)
    lateral.annotate(nombre, (x, z), textcoords="offset points", xytext=(8, 4))
    superior.plot(x, y, "b*", markersize=14)

for eje in (lateral, superior):
    eje.set_aspect("equal")
    eje.grid(True, alpha=0.3)

ruta = Path(__file__).parent.parent / "Entregables" / "figuras" / "mapa_alcance.png"
plt.tight_layout()
plt.savefig(ruta, dpi=120)
print(f"\nGráfico guardado en {ruta}")
