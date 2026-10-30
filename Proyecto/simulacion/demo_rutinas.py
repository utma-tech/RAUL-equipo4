# Demostración de las tres rutinas en PyBullet, con ventana.
# Es la demostración de agarre y posicionamiento que pide el enunciado.
# Correr desde la carpeta simulacion:  python demo_rutinas.py
#
# Hace, en orden: tomar el objeto de la mesa, depositarlo en la caja de carga de RAUL,
# volver a la pose plegada, ir al ascensor y presionar el botón. Después repite.
# Si algún tramo chocara, se detiene y lo escribe en rojo sobre el brazo.
import time
import pybullet as p
import escena
import rutinas

PAUSA = 0.8   # segundos entre pasos, para que se pueda seguir a ojo

escena.cargar(con_ventana=True)
escena.mostrar_estado("preparando...", True)


def correr(nombre, pasos, q):
    for paso in pasos:
        texto = paso[1]
        escena.mostrar_estado(f"{nombre}: {texto}", True)
        q, informe = rutinas.ejecutar([paso], q, animar=True)
        estado = informe[-1][1]
        if estado != "ok":
            escena.mostrar_estado(f"{nombre}: {texto} -> {estado}", False)
            return q, False
        time.sleep(PAUSA)
    return q, True


q = rutinas.PLEGADO
escena.mover(q)

while p.isConnected():
    for nombre, construir in (("tomar de la mesa", rutinas.tomar_de_mesa),
                              ("depositar en la carga", rutinas.depositar_en_carga),
                              ("presionar el boton", rutinas.presionar_boton)):
        if not p.isConnected():
            break
        q, bien = correr(nombre, construir(), q)
        if not bien:
            time.sleep(5)
            break
        if nombre == "depositar en la carga":
            escena.mostrar_estado("volver a la pose plegada", True)
            q, estado = rutinas.ir_a(rutinas.PLEGADO, q, segundos=3, animar=True)
            if estado != "ok":
                escena.mostrar_estado(f"volver al plegado -> {estado}", False)
                time.sleep(5)
                break
    else:
        escena.mostrar_estado("listo: vuelve a empezar", True)
        q, _ = rutinas.ir_a(rutinas.PLEGADO, q, segundos=3, animar=True)
        time.sleep(1.5)
