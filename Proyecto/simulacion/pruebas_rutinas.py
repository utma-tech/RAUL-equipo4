# Corre las tres rutinas sin ventana y falla si alguna choca o no se puede resolver.
# Es la verificación de la tarea 5 de GUIA_EQUIPO.md: "las tres rutinas corren en PyBullet
# sin choques". Correr desde la carpeta simulacion:  python pruebas_rutinas.py
import re
import sys
from pathlib import Path
import escena
import rutinas
import trayectorias
from cinematica import fk
import parametros

# VELOCIDAD_MAX de programa_esp32/configuracion.h. Si un tramo pide más que esto,
# el servo no llega: el brazo real se queda atrás del perfil.
VELOCIDAD_MAX = 90   # grados por segundo
from parametros import NOMBRES

escena.cargar(con_ventana=False)
fallas = []
rapidos = []


def revisar_reposo_contra_firmware():
    """REPOSO está escrito en parametros.py y en configuracion.h. Si se cambia uno y no el otro,
    la simulación valida una pose que el brazo real nunca va a tomar."""
    archivo = Path(__file__).parent.parent / "programa_esp32" / "configuracion.h"
    texto = archivo.read_text(encoding="utf-8")
    encontrado = re.search(r"REPOSO\[N\]\s*=\s*\{([^}]*)\}", texto)
    if not encontrado:
        print("FALLA no se encontró REPOSO en configuracion.h")
        fallas.append("REPOSO no está en configuracion.h")
        return
    del_firmware = [float(v) for v in encontrado.group(1).split(",")]
    if del_firmware != [float(v) for v in parametros.REPOSO]:
        print(f"FALLA REPOSO distinto entre los dos archivos:\n"
              f"          configuracion.h {del_firmware}\n"
              f"          parametros.py   {parametros.REPOSO}")
        fallas.append("REPOSO distinto entre configuracion.h y parametros.py")
    else:
        print(f"OK    REPOSO igual en configuracion.h y parametros.py: {parametros.REPOSO}")


def avisar_si_reposo_no_sirve_para_circular():
    """REPOSO es la pose del firmware y no se toca acá. Pero si con los escenarios puestos
    el brazo quieto ya está chocando, conviene saberlo: quiere decir que RAUL no se puede
    arrimar a esa estación de trabajo con el brazo en reposo. Es un aviso, no una falla."""
    for escenario in ("mesa", "ascensor"):
        escena.poner_escenario(escenario)
        assert "pedestal" in escena.obstaculos, "se perdió el pedestal al cambiar escenario"
        escena.mover(parametros.REPOSO)
        lista = escena.choques()
        if lista:
            fallas.append(f"reposo con {escenario}")
            print(f"FALLA REPOSO choca con el escenario '{escenario}': {', '.join(lista)}")
    escena.poner_escenario(None)
    x, y, z, _ = fk(parametros.REPOSO)
    plegado = fk(rutinas.PLEGADO)
    print(f"      REPOSO deja la punta a {(x*x + y*y) ** 0.5:.0f} mm del eje de la base; "
          f"las rutinas circulan en PLEGADO, que la deja a "
          f"{(plegado[0] ** 2 + plegado[1] ** 2) ** 0.5:.0f} mm")


revisar_reposo_contra_firmware()
avisar_si_reposo_no_sirve_para_circular()


def revisar_velocidad(nombre, texto, q_a, q_b, segundos):
    picos = trayectorias.velocidades_pico(q_a, q_b, segundos)
    for i, pico in enumerate(picos):
        if pico > VELOCIDAD_MAX:
            minimo = trayectorias.tiempo_minimo(q_a, q_b, VELOCIDAD_MAX)
            rapidos.append(f"{nombre} / {texto}: {NOMBRES[i]} llega a {pico:.0f} grados/s "
                           f"(maximo {VELOCIDAD_MAX}); el tramo tendria que durar {minimo:.1f} s "
                           f"en vez de {segundos}")
    return max(picos)


def correr(nombre, pasos, q):
    """Recorre la rutina paso por paso, para poder medir también la velocidad de cada tramo."""
    print(f"\n{nombre}")
    for paso in pasos:
        texto = paso[1]
        antes = q
        q, informe = rutinas.ejecutar([paso], q)
        estado = informe[-1][1]
        extra = ""
        if paso[0] == "mover" and estado == "ok":
            pico = revisar_velocidad(nombre, texto, antes, q, paso[3])
            extra = f"   ({paso[3]} s, pico {pico:.0f} grados/s)"
        print(f"  {'OK   ' if estado == 'ok' else 'FALLA'} {texto}{extra}"
              + ("" if estado == "ok" else f"\n          {estado}"))
        if estado != "ok":
            fallas.append(f"{nombre}: {texto}")
            return q
    return q


# El brazo arranca plegado, como cuando RAUL se acerca a la mesa
q = rutinas.PLEGADO
escena.mover(q)
if escena.choques():
    print("FALLA la pose de transporte ya choca:", escena.choques())
    fallas.append("plegado")

q = correr("tomar objeto de mesa", rutinas.tomar_de_mesa(), q)
q = correr("depositar en plataforma de carga", rutinas.depositar_en_carga(), q)

print("\nvolver al reposo")
q, estado = rutinas.ir_a(rutinas.PLEGADO, q)
print(f"  {'OK   ' if estado == 'ok' else 'FALLA'} plegado"
      + ("" if estado == "ok" else f"\n          {estado}"))
if estado != "ok":
    fallas.append("volver al plegado")

q = correr("presionar boton del ascensor", rutinas.presionar_boton(), q)

if rapidos:
    print("\nAVISO: tramos que piden mas velocidad de la que da el motor")
    print("(no es un choque, pero el brazo real no va a seguir el perfil)")
    for linea in rapidos:
        print("  " + linea)

print()
print("TODO OK" if not fallas else f"{len(fallas)} FALLAS:\n  " + "\n  ".join(fallas))
sys.exit(1 if fallas or rapidos else 0)
