# Gráficos de posición, velocidad y aceleración de un movimiento, para el informe.
# Muestra que con el perfil de 5.° grado las cuatro articulaciones arrancan y frenan con
# velocidad y aceleración cero, y llegan todas juntas: eso es lo que evita los tirones
# y las vibraciones en la estructura.
# Correr desde la carpeta simulacion:  python graficos_trayectoria.py
from pathlib import Path
import matplotlib.pyplot as plt
import escena
import rutinas
import trayectorias

from parametros import N, NOMBRES

# Un color y un tipo de línea por articulación. El tipo de línea no es decoración:
# hace que el gráfico siga siendo legible impreso en blanco y negro y para quien no
# distingue bien los colores.
COLORES = ["#1F6FB2", "#D55E00", "#1B7F5F", "#8E5FA8"]
TRAZOS = ["-", "--", "-.", ":"]


def columna(filas, i):
    return [fila[i] for fila in filas]


escena.cargar(con_ventana=False)

# Movimiento a graficar: el traslado del objeto desde la mesa hasta encima de la caja de carga.
# Es el más largo de las rutinas, así que es el que más exige a los motores.
q = rutinas.PLEGADO
q, informe = rutinas.ejecutar(rutinas.tomar_de_mesa(), q)
if any(estado != "ok" for _, estado in informe):
    raise SystemExit("la rutina de tomar no llegó hasta el final: " + str(informe))

paso = rutinas.depositar_en_carga()[0]
SEGUNDOS = paso[3]            # la duración que usa la rutina de verdad, no un número aparte
destino, motivo = trayectorias.angulos(paso[2])
if destino is None:
    raise SystemExit("no se pudo resolver la pose de destino: " + motivo)

tiempos, angulos, velocidades, aceleraciones = trayectorias.evolucion(q, destino, SEGUNDOS)

figura, ejes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
series = ((angulos, "angulo [grados]"), (velocidades, "velocidad [grados/s]"),
          (aceleraciones, "aceleracion [grados/s2]"))

for eje, (datos, etiqueta) in zip(ejes, series):
    for i in range(N):
        eje.plot(tiempos, columna(datos, i), color=COLORES[i], linestyle=TRAZOS[i],
                 linewidth=1.8, label=NOMBRES[i])
    eje.set_ylabel(etiqueta)
    eje.grid(True, alpha=0.25, linewidth=0.6)
    eje.axhline(0, color="black", linewidth=0.8, alpha=0.4)
    for lado in ("top", "right"):
        eje.spines[lado].set_visible(False)

ejes[0].set_title(f"{paso[1]} — perfil de 5.° grado, {SEGUNDOS:.0f} s")
ejes[0].legend(ncol=5, loc="upper left", frameon=False, fontsize=9)
ejes[2].set_xlabel("tiempo [s]")

ruta = Path(__file__).parent.parent / "Entregables" / "figuras" / "trayectoria.png"
plt.tight_layout()
plt.savefig(ruta, dpi=120)
print(f"Grafico guardado en {ruta}")

# Verificación numérica de lo que se ve en el gráfico
print(f"\nMovimiento de {SEGUNDOS:.0f} s. El perfil de 5.° grado tiene un pico de velocidad")
print("1,875 veces la velocidad media. Si esto no se cumple, el perfil está mal implementado.\n")
print(f"{'articulacion':14s}{'recorrido':>12s}{'vel. media':>12s}{'vel. maxima':>13s}{'factor':>9s}"
      f"{'acel. max':>12s}")
for i in range(N):
    recorrido = destino[i] - q[i]
    media = abs(recorrido) / SEGUNDOS
    maxima = max(abs(v) for v in columna(velocidades, i))
    acel = max(abs(a) for a in columna(aceleraciones, i))
    factor = maxima / media if media > 1e-9 else float("nan")
    print(f"{NOMBRES[i]:14s}{recorrido:>10.1f}°{media:>10.1f}°/s{maxima:>11.1f}°/s"
          f"{factor:>9.3f}{acel:>10.1f}°/s2")

print("\nEn los dos extremos (t = 0 y t = %.0f s):" % SEGUNDOS)
print(f"  velocidad maxima    {max(abs(v) for v in velocidades[0] + velocidades[-1]):.2e} grados/s")
print(f"  aceleracion maxima  {max(abs(a) for a in aceleraciones[0] + aceleraciones[-1]):.2e} grados/s2")
