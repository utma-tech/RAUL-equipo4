# Error de posicionamiento de la punta del gripper (tarea 6 de GUIA_EQUIPO.md).
#
# Estimación geométrica con servos MG995 y PWM del PCA9685.
# Se separan cuantización del mando y error angular de calibración.
# No incluye flexión, holgura, carga, histéresis ni mediciones del prototipo.
# Los máximos corresponden a muestras y vértices del intervalo angular, no a una
# demostración de cota global continua.
import random
from itertools import product
from math import hypot, sqrt
from pathlib import Path
import matplotlib.pyplot as plt
import escena
import rutinas
from cinematica import fk, ik, OK
from parametros import D1, A2, A3, D5, QMIN, QMAX, N, NOMBRES

# Resolución de cada actuador. Son los datos de MOTORES en programa_esp32/configuracion.h
# y de la sección 6 de la guía; pruebas_error.py verifica que sigan coincidiendo.

# Servos por el PCA9685: a 50 Hz el chip reparte 4096 cuentas en los 20 ms del período,
# o sea 4,88 µs por cuenta, y el servo pide 11,11 µs por grado (2000 µs para 180°).
PERIODO_PCA9685 = 20000.0                              # µs
CUENTAS_PCA9685 = 4096
US_POR_CUENTA = PERIODO_PCA9685 / CUENTAS_PCA9685
US_POR_GRADO = 11.11

# Las cuatro articulaciones del CAD se mueven con servos MG995 por el PCA9685, así que
# todas tienen la misma resolución: lo que vale una cuenta del PCA9685 en grados.
RESOLUCION = [US_POR_CUENTA / US_POR_GRADO] * N

# Cuánto se le puede errar al cero de cada articulación después de calibrar a ojo.
# La guía pide probar entre 1° y 2°; se toma 2° como caso de trabajo y 1° como comparación.
ERROR_CALIBRACION = 2.0

# Las cuatro articulaciones mueven la punta: ya no hay giro de muñeca, que era la única
# que no la movía porque el TCP caía justo sobre su eje.
QUE_MUEVEN = tuple(range(N))

ALCANCE = A2 + A3 + D5   # distancia máxima de la punta al hombro, con el brazo estirado

CANTIDAD = 12000         # configuraciones a probar
MUESTRAS = 10            # sorteos de error por configuración, para el caso típico


def distancia(pose_a, pose_b):
    """Separación en mm entre las puntas de dos poses (solo x, y, z)."""
    return sqrt(sum((pose_a[i] - pose_b[i]) ** 2 for i in range(3)))


def distancia_al_hombro(pose):
    """Cuánto brazo hay estirado hasta la punta. El hombro está sobre el eje de la base,
    a D1 de la cara inferior de la placa, así que no depende del ángulo de la base."""
    return hypot(hypot(pose[0], pose[1]), pose[2] - D1)


def cuantizar(q):
    """Los ángulos que el brazo realmente puede tomar: cada uno al múltiplo más cercano de
    la resolución de su actuador. Es lo que pasa cuando el firmware convierte grados a pasos
    o a cuentas del PCA9685 y redondea."""
    ceros = (1500, 500, 2167, 1500)  # servo principal, configuracion.h
    return [(int((ceros[i] + US_POR_GRADO*q[i])/US_POR_CUENTA + 0.5)*US_POR_CUENTA
             - ceros[i])/US_POR_GRADO for i in range(N)]


def amplitudes_resolucion():
    """Cuánto se puede desviar cada ángulo solo por resolución: media cuenta o medio paso,
    porque el firmware redondea al más cercano y no trunca."""
    return [paso / 2 for paso in RESOLUCION]


def amplitudes_calibracion(error):
    return [error] * N


def amplitudes_total(error):
    """Los dos errores juntos. Se suman porque son independientes: el cero puede estar corrido
    y además el ángulo pedido cae entre dos cuentas."""
    return [paso / 2 + error for paso in RESOLUCION]


def error_peor_caso(q, pose, amplitudes):
    """Cuánto es lo máximo que se puede correr la punta si cada ángulo se desvía hasta ±amplitud.
    Se prueban las 16 combinaciones de signos de las cuatro articulaciones que mueven la punta:
    para desviaciones de pocos grados el desplazamiento crece casi lineal con el ángulo, así que
    se estima el peor caso evaluando las esquinas; no es una cota global probada."""
    peor = 0.0
    for signos in product((-1, 1), repeat=len(QUE_MUEVEN)):
        desviado = list(q)
        for signo, i in zip(signos, QUE_MUEVEN):
            desviado[i] += signo * amplitudes[i]
        peor = max(peor, distancia(pose, fk(desviado)))
    return peor


def error_tipico(q, pose, amplitudes, azar, muestras=MUESTRAS):
    """Cuánto se corre la punta en promedio con errores sorteados dentro de ±amplitud.
    Es lo que se espera en una corrida cualquiera: que los cuatro errores se den a la vez
    en el peor sentido es raro."""
    suma = 0.0
    for _ in range(muestras):
        desviado = [q[i] + azar.uniform(-amplitudes[i], amplitudes[i]) for i in range(N)]
        suma += distancia(pose, fk(desviado))
    return suma / muestras


def error_de_cuantizacion(q):
    """El error de resolución medido como lo pide la guía: redondeando los ángulos a lo que
    el actuador puede dar. Depende de dónde caiga q entre dos cuentas, así que es un caso
    concreto, no el peor."""
    return distancia(fk(q), fk(cuantizar(q)))


def recorrer(cantidad=CANTIDAD, error_calibracion=ERROR_CALIBRACION, semilla=1):
    """Prueba muchas configuraciones al azar dentro de los límites articulares y devuelve,
    para cada una, dónde quedó la punta y cuánto se corre con cada fuente de error.

    No se descartan las configuraciones que chocan contra la escena: el error es geométrico y
    no depende de los obstáculos, y así el mapa cubre todo el volumen que el brazo puede barrer.
    Tampoco importa el ángulo de la base para el tamaño del error, solo para hacia dónde apunta;
    por eso el gráfico usa la distancia al eje (r) y no x e y por separado."""
    azar = random.Random(semilla)
    resolucion = amplitudes_resolucion()
    calibracion = amplitudes_calibracion(error_calibracion)
    total = amplitudes_total(error_calibracion)

    datos = {clave: [] for clave in ("q", "r", "z", "hombro", "cuantizacion",
                                     "res_peor", "cal_peor", "total_peor",
                                     "res_tipico", "cal_tipico", "total_tipico")}
    for _ in range(cantidad):
        q = [azar.uniform(QMIN[i], QMAX[i]) for i in range(N)]
        pose = fk(q)
        datos["q"].append(q)
        datos["r"].append(hypot(pose[0], pose[1]))
        datos["z"].append(pose[2])
        datos["hombro"].append(distancia_al_hombro(pose))
        datos["cuantizacion"].append(error_de_cuantizacion(q))
        datos["res_peor"].append(error_peor_caso(q, pose, resolucion))
        datos["cal_peor"].append(error_peor_caso(q, pose, calibracion))
        datos["total_peor"].append(error_peor_caso(q, pose, total))
        datos["res_tipico"].append(error_tipico(q, pose, resolucion, azar))
        datos["cal_tipico"].append(error_tipico(q, pose, calibracion, azar))
        datos["total_tipico"].append(error_tipico(q, pose, total, azar))
    return datos


def promedio(valores):
    return sum(valores) / len(valores)


def donde_esta_el_maximo(datos, clave):
    """Índice de la configuración con más error, para poder contar dónde pasa."""
    return max(range(len(datos[clave])), key=lambda i: datos[clave][i])


def error_en_un_punto(x, y, z, pitch, error_calibracion=ERROR_CALIBRACION):
    """Error de peor caso en un punto concreto del espacio, resolviendo la ik para llegar.
    Devuelve None si el punto no se alcanza con esa inclinación de pinza."""
    codigo, q = ik([x, y, z, pitch])
    if codigo != OK:
        return None
    pose = fk(q)
    return error_peor_caso(q, pose, amplitudes_total(error_calibracion))


if __name__ == "__main__":
    datos = recorrer()
    cantidad = len(datos["q"])

    print(f"Resolución de cada actuador (grados por paso o por cuenta):")
    for i, nombre in enumerate(NOMBRES):
        print(f"  {nombre:7s} {RESOLUCION[i]:.4f}")
    print(f"Error de calibración supuesto: ±{ERROR_CALIBRACION}° por articulación")
    print(f"Configuraciones probadas: {cantidad}")

    print("\nError de la punta [mm], sobre todo el espacio de trabajo:")
    print(f"  {'fuente':24s} {'medio':>8s} {'máximo':>8s}")
    for clave, titulo in (("res_tipico", "resolución, típico"),
                          ("res_peor", "resolución, peor caso"),
                          ("cal_tipico", "calibración, típico"),
                          ("cal_peor", "calibración, peor caso"),
                          ("total_tipico", "los dos, típico"),
                          ("total_peor", "los dos, peor caso")):
        print(f"  {titulo:24s} {promedio(datos[clave]):8.1f} {max(datos[clave]):8.1f}")

    # La cuantización real tiene que quedar siempre por debajo del peor caso de media cuenta:
    # es la misma cuenta mirada de dos maneras y sirve de control cruzado.
    print(f"  {'redondeo real':24s} {promedio(datos['cuantizacion']):8.1f} "
          f"{max(datos['cuantizacion']):8.1f}")

    veces = promedio(datos["cal_peor"]) / promedio(datos["res_peor"])
    print(f"\nLa calibración pesa {veces:.0f} veces más que la resolución.")
    print("Afinar la calibración (medir mejor el brazo y los ceros de los servos) baja el error")
    print("sin cambiar la resolución nominal del PWM.")

    # Dónde se da el máximo. Con el brazo estirado cualquier error angular se amplifica más,
    # así que se espera encontrarlo cerca del alcance máximo (A2 + A3 + D5).
    peor = donde_esta_el_maximo(datos, "total_peor")
    q_peor = datos["q"][peor]
    pose_peor = fk(q_peor)
    print(f"\nMáximo error (peor caso, los dos errores juntos): {datos['total_peor'][peor]:.1f} mm")
    print(f"  punta en x = {pose_peor[0]:.0f}, y = {pose_peor[1]:.0f}, z = {pose_peor[2]:.0f} mm")
    print(f"  ángulos " + ", ".join(f"{NOMBRES[i]} {q_peor[i]:.0f}°" for i in range(N)))
    print(f"  la punta está a {datos['hombro'][peor]:.0f} mm del hombro, "
          f"sobre {ALCANCE} mm de alcance máximo")

    # El error crece con el brazo estirado porque cada grado de error se multiplica por la
    # distancia. Se mide cuántos mm de error hay por cada 100 mm de brazo estirado.
    por_cien = [1000 * datos["total_peor"][i] / datos["hombro"][i]
                for i in range(cantidad) if datos["hombro"][i] > 100]
    print(f"\nError por cada 100 mm de distancia al hombro: entre {min(por_cien) / 10:.1f} "
          f"y {max(por_cien) / 10:.1f} mm (medio {promedio(por_cien) / 10:.1f})")
    print("O sea que el error es casi proporcional a cuánto brazo hay estirado: el brazo plegado")
    print("es preciso y el brazo estirado es el peor caso.")

    # Lo mismo con ±1° de calibración, para ver cuánto se gana calibrando mejor
    datos_uno = recorrer(cantidad=2000, error_calibracion=1.0)
    print(f"\nCon ±1° en vez de ±2°: error medio {promedio(datos_uno['total_peor']):.1f} mm, "
          f"máximo {max(datos_uno['total_peor']):.1f} mm (peor caso)")

    # Los tres puntos de las tareas, para tenerlos en el informe
    puntos = {nombre: (pose[:3], pose[3])
              for nombre, pose in rutinas.poses_tarea().items()}

    print("\nError de peor caso en los puntos de las tareas:")
    for nombre, ((x, y, z), pitch) in puntos.items():
        error = error_en_un_punto(x, y, z, pitch)
        if error is None:
            print(f"  {nombre:16s} no se alcanza con pitch {pitch}°")
        else:
            distancia_hombro = distancia_al_hombro([x, y, z])
            print(f"  {nombre:16s} ({x:.0f}, {y:.0f}, {z:.0f}) con pitch {pitch}°: "
                  f"{error:.0f} mm, punta a {distancia_hombro:.0f} mm del hombro")

    # Gráfico. Mismo estilo que mapa_alcance.py: una vista del espacio de trabajo a la
    # izquierda y lo que se quiere mostrar a la derecha.
    fig, (mapa, contra_distancia) = plt.subplots(1, 2, figsize=(14, 6))

    # Vista lateral del espacio de trabajo. Se usa la distancia al eje de la base (r) en vez de
    # x porque el error no cambia al girar la base: un solo corte vale para todo el volumen.
    dibujo = mapa.scatter(datos["r"], datos["z"], s=6, c=datos["total_peor"],
                          cmap="viridis", alpha=0.7)
    fig.colorbar(dibujo, ax=mapa, label="error de la punta [mm]")
    mapa.plot(0, D1, "r.", markersize=12)
    mapa.annotate("hombro", (0, D1), textcoords="offset points", xytext=(8, 4), color="red")
    mapa.set_title(f"Error de posicionamiento, peor caso\n"
                   f"(resolución de los motores + ±{ERROR_CALIBRACION}° de calibración)")
    mapa.set_xlabel("distancia al eje de la base [mm]")
    mapa.set_ylabel("z [mm]")
    mapa.set_aspect("equal")

    # El error contra cuánto brazo hay estirado: es lo que explica el mapa de la izquierda
    for clave, color, etiqueta in (("total_peor", "tab:red", "los dos, peor caso"),
                                   ("cal_peor", "tab:orange", "solo calibración, peor caso"),
                                   ("total_tipico", "tab:blue", "los dos, típico"),
                                   ("res_peor", "tab:green", "solo resolución, peor caso")):
        contra_distancia.scatter(datos["hombro"], datos[clave], s=4, alpha=0.3,
                                 c=color, label=etiqueta)
    contra_distancia.axvline(ALCANCE, color="black", linestyle="--")
    contra_distancia.annotate(f"brazo estirado ({ALCANCE} mm)", (ALCANCE, max(datos["total_peor"])),
                              textcoords="offset points", xytext=(-170, -14))
    contra_distancia.set_title("El error crece con el brazo estirado")
    contra_distancia.set_xlabel("distancia de la punta al hombro [mm]")
    contra_distancia.set_ylabel("error de la punta [mm]")
    contra_distancia.legend(loc="upper left", markerscale=4)

    for eje in (mapa, contra_distancia):
        eje.grid(True, alpha=0.3)

    ruta = Path(__file__).parent.parent / "Entregables" / "figuras" / "error_posicionamiento.png"
    plt.tight_layout()
    plt.savefig(ruta, dpi=120)
    print(f"\nGráfico guardado en {ruta}")
