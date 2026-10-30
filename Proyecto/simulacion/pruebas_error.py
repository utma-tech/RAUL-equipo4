# Pruebas del error de posicionamiento (tarea 6 de GUIA_EQUIPO.md).
# Correr desde la carpeta simulacion:  python pruebas_error.py
#   1. Las resoluciones de error_posicionamiento.py son las que dicen configuracion.h y movimiento.cpp.
#   2. La pinza cuelga 18 mm del eje de la muñeca, como en el CAD.
#   3. Redondear los ángulos de verdad nunca da más error que el peor caso de media cuenta.
#   4. El error de calibración le gana por lejos al de resolución.
#   5. El error más grande aparece con el brazo estirado.
#   6. El error crece casi proporcional a la distancia de la punta al hombro.
#   7. Los números siguen dentro de las cotas del informe.
#
# Las cotas de la prueba 7 son generosas a propósito: no están para validar el brazo real, que
# no se ha medido en este ensayo, sino para que cambiar una medida o una calibración y duplicar el error no
# pase desapercibido.
import sys
import re
from pathlib import Path
import error_posicionamiento as ep
from cinematica import fk
from parametros import QMIN, QMAX, N, D1, A2, A3, LARGO_PINZA, CAIDA_PINZA

# Cotas esperadas (mm). Modelo V4 de parametros.py y ±2° de
# calibración: media típica cercana a 9,6 mm y máximo muestreado cercano a 35 mm.
COTA_TIPICO_MEDIO = 12
COTA_PEOR_MAXIMO = 40
VECES_MINIMAS = 5          # cuántas veces, por lo menos, la calibración le gana a la resolución
MM_POR_CIEN_MM = (4, 25)   # mm de error de peor caso por cada 100 mm de brazo estirado

CARPETA_FIRMWARE = Path(__file__).parent.parent / "programa_esp32"


def informar(nombre, paso, detalle):
    print(f"{'OK   ' if paso else 'FALLA'} {nombre}: {detalle}")
    return paso


def numero(texto):
    """Evalúa una cuenta escrita en el firmware, como '200 * 16 * 5 / 360.0'.
    Se acepta solo aritmética con números para no ejecutar nada más del archivo."""
    if not re.fullmatch(r"[\d\s.*/+\-()]+", texto):
        raise ValueError(f"no parece una cuenta: {texto}")
    return eval(texto)


def motores_del_firmware():
    """Las filas de MOTORES en configuracion.h, como diccionarios con lo que acá interesa."""
    texto = (CARPETA_FIRMWARE / "configuracion.h").read_text(encoding="utf-8")
    bloque = texto.split("MOTORES[N] = {")[1].split("\n};")[0]
    motores = []
    for linea in bloque.splitlines():
        linea = linea.split("//")[0].strip()
        if not linea.startswith("{"):
            continue
        campos = [campo.strip() for campo in linea.strip("{} ,").split(",")]
        # canal, pulsoCero, usPorGrado, canal2, pulsoCero2, usPorGrado2
        motores.append({
            "canal": numero(campos[0]),
            "usPorGrado": numero(campos[2].rstrip("f")),
            "canal2": numero(campos[3]),
            "usPorGrado2": numero(campos[5].rstrip("f")),
        })
    return motores


def prueba_resoluciones_vs_firmware():
    """La resolución de cada articulación tiene que salir de los mismos números que usa el ESP32.
    Si alguien cambia una reducción o un servo en configuracion.h, esta prueba lo avisa."""
    motores = motores_del_firmware()
    if len(motores) != N:
        return informar("resoluciones vs configuracion.h", False,
                        f"se leyeron {len(motores)} motores en vez de {N}")

    peor = 0.0
    for i, motor in enumerate(motores):
        # Son todos servos del PCA9685, que solo entiende cuentas enteras:
        # la resolución es µs por cuenta / µs por grado
        esperada = ep.US_POR_CUENTA / abs(motor["usPorGrado"])
        peor = max(peor, abs(esperada - ep.RESOLUCION[i]))
        if motor["canal2"] >= 0 and motor["usPorGrado"] * motor["usPorGrado2"] >= 0:
            peor = float("inf")   # los dos servos enfrentados tienen que girar al revés
    resoluciones_ok = informar("resoluciones vs configuracion.h", peor < 1e-9,
                               f"diferencia máxima {peor:.1e}°")

    # El firmware calcula los µs por cuenta del PCA9685 en movimiento.cpp: tiene que dar lo mismo
    texto = (CARPETA_FIRMWARE / "movimiento.cpp").read_text(encoding="utf-8")
    cuenta = re.search(r"US_POR_CUENTA\s*=\s*([^;]+);", texto).group(1)
    diferencia = abs(numero(cuenta) - ep.US_POR_CUENTA)
    cuenta_ok = informar("µs por cuenta vs movimiento.cpp", diferencia < 1e-9,
                         f"diferencia {diferencia:.1e} µs")
    return resoluciones_ok and cuenta_ok


def prueba_la_pinza_cuelga():
    """La pinza no va centrada en el eje de la muñeca: en el CAD cuelga 18 mm por debajo.
    Con el brazo estirado y horizontal (q = 0) la punta tiene que quedar a LARGO_PINZA del eje
    de la muñeca y CAIDA_PINZA por debajo de la línea de los tres ejes. Si esto falla, se
    perdió la caída de la pinza en alguna de las dos fórmulas."""
    x, y, z, pitch = fk([0] * N)
    peor = max(abs(x - (A2 + A3 + LARGO_PINZA)), abs(y),
               abs(z - (D1 - CAIDA_PINZA)), abs(pitch))
    return informar("la pinza cuelga 18 mm del eje de la muñeca", peor < 1e-9,
                    f"diferencia máxima {peor:.1e}")


def prueba_redondeo_dentro_del_peor_caso(datos):
    """Control cruzado: el error de redondear los ángulos de verdad (lo que pide la guía) no puede
    superar al peor caso de media cuenta. Son la misma cuenta mirada de dos maneras."""
    excesos = [i for i in range(len(datos["q"]))
               if datos["cuantizacion"][i] > datos["res_peor"][i] + 1e-9]
    return informar("redondeo real dentro del peor caso de resolución", not excesos,
                    f"{len(excesos)} configuraciones lo superan")


def prueba_domina_la_calibracion(datos):
    """Cuál de los dos errores manda. Decide dónde conviene gastar el esfuerzo: medir mejor
    el brazo, o comprar actuadores con más resolución."""
    veces = ep.promedio(datos["cal_peor"]) / ep.promedio(datos["res_peor"])
    return informar("la calibración domina sobre la resolución", veces > VECES_MINIMAS,
                    f"pesa {veces:.0f} veces más (mínimo esperado {VECES_MINIMAS})")


def prueba_maximo_con_el_brazo_estirado(datos):
    """Cada grado de error se multiplica por el brazo que haya estirado, así que el máximo tiene
    que caer cerca del alcance máximo. Si apareciera plegado, algo raro hay en el modelo."""
    peor = ep.donde_esta_el_maximo(datos, "total_peor")
    fraccion = datos["hombro"][peor] / ep.ALCANCE
    return informar("el máximo error ocurre con el brazo estirado", fraccion > 0.95,
                    f"la punta está al {100 * fraccion:.0f} % del alcance "
                    f"({datos['hombro'][peor]:.0f} de {ep.ALCANCE} mm)")


def prueba_crece_con_la_distancia(datos):
    """El error de peor caso por cada 100 mm de brazo estirado se tiene que mantener en una
    franja estrecha: es lo que quiere decir "el error es casi proporcional a la distancia"."""
    razones = [100 * datos["total_peor"][i] / datos["hombro"][i]
               for i in range(len(datos["q"])) if datos["hombro"][i] > 100]
    minimo, maximo = MM_POR_CIEN_MM
    dentro = min(razones) > minimo and max(razones) < maximo
    return informar("el error crece con la distancia al hombro", dentro,
                    f"entre {min(razones):.1f} y {max(razones):.1f} mm por cada 100 mm "
                    f"(esperado entre {minimo} y {maximo})")


def prueba_cotas(datos):
    tipico = ep.promedio(datos["total_tipico"])
    peor = max(datos["total_peor"])
    tipico_ok = informar("error típico medio bajo la cota", tipico < COTA_TIPICO_MEDIO,
                         f"{tipico:.1f} mm (cota {COTA_TIPICO_MEDIO} mm)")
    peor_ok = informar("error de peor caso bajo la cota", peor < COTA_PEOR_MAXIMO,
                       f"{peor:.1f} mm (cota {COTA_PEOR_MAXIMO} mm)")
    return tipico_ok and peor_ok


# Menos configuraciones que en el informe: alcanzan para que los números se estabilicen
datos = ep.recorrer(cantidad=3000)

resultados = [
    prueba_resoluciones_vs_firmware(),
    prueba_la_pinza_cuelga(),
    prueba_redondeo_dentro_del_peor_caso(datos),
    prueba_domina_la_calibracion(datos),
    prueba_maximo_con_el_brazo_estirado(datos),
    prueba_crece_con_la_distancia(datos),
    prueba_cotas(datos),
]

print()
print("TODO OK" if all(resultados) else f"{resultados.count(False)} FALLAS")
sys.exit(0 if all(resultados) else 1)
