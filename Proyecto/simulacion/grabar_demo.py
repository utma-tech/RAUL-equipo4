# Graba en video las tres rutinas, sin abrir ventana: es la demostración de agarre y
# posicionamiento que pide el enunciado.
#
# Correr desde la carpeta simulacion:  python grabar_demo.py
# Deja el video en ../Entregables/demostracion_rutinas.gif
#
# Arma cada cuadro con getCameraImage(), le pega el cartel con PIL y lo guarda como GIF con Pillow. Al no depender de un capturador de pantalla, sale igual en cualquier máquina.
import shutil
import tempfile
import sys
from pathlib import Path

import pybullet as p
from PIL import Image, ImageDraw, ImageFont

import escena
import rutinas
import trayectorias

ANCHO, ALTO = 720, 405
FPS = 12
PAUSA = 0.4                  # segundos que se queda quieto entre pasos
CARPETA = Path(__file__).parent.parent / "Entregables"
SALIDA = CARPETA / "demostracion_rutinas.gif"
CUADROS = Path(tempfile.mkdtemp(prefix="raul_cuadros_"))

# Una cámara por escenario: el panel del ascensor mide 80 cm de ancho y con el encuadre
# de la mesa se mete entre la cámara y el brazo, así que esa escena se mira casi de perfil.
# (centro en metros, distancia, giro, inclinación)
CAMARAS = {
    None:       ([0.20, 0.03, 0.15], 1.05,   40, -18),
    "mesa":     ([0.20, 0.03, 0.15], 1.05,   40, -18),
    "ascensor": ([0.15, 0.00, 0.30], 1.20, -150, -12),
}

# La rutina de depositar no cambia de escenario, así que la mesa queda entre la cámara y
# la caja de carga. Para esa etapa se mira desde la izquierda.
CAMARA_CARGA = ([0.10, 0.09, 0.10], 0.95, 10, -20)

# 720x405: sin ventana PyBullet renderiza por software y cada cuadro cuesta
# medio segundo.
_proyeccion = p.computeProjectionMatrixFOV(60, ANCHO / ALTO, 0.05, 10)


def _vista_de(escenario):
    centro, distancia, giro, inclinacion = CAMARAS.get(escenario, CAMARAS[None])
    return p.computeViewMatrixFromYawPitchRoll(centro, distancia, giro, inclinacion, 0, 2)


_vista = _vista_de(None)

try:
    _letra = ImageFont.truetype("arial.ttf", 22)
    _letra_chica = ImageFont.truetype("arial.ttf", 16)
except OSError:
    _letra = _letra_chica = ImageFont.load_default()

_n = 0


def _cuadro(titulo, detalle):
    """Renderiza un cuadro y lo guarda numerado."""
    global _n
    _, _, pixeles, _, _ = p.getCameraImage(ANCHO, ALTO, _vista, _proyeccion,
                                           renderer=p.ER_TINY_RENDERER)
    imagen = Image.frombytes("RGBA", (ANCHO, ALTO), bytes(pixeles)).convert("RGB")

    dibujo = ImageDraw.Draw(imagen, "RGBA")
    dibujo.rectangle([0, 0, ANCHO, 72], fill=(16, 20, 28, 210))
    dibujo.text((22, 12), titulo, font=_letra, fill=(255, 255, 255))
    dibujo.text((22, 43), detalle, font=_letra_chica, fill=(150, 205, 255))
    dibujo.text((ANCHO - 230, 43), "RAUL-M · Equipo 4", font=_letra_chica, fill=(130, 140, 150))

    imagen.save(CUADROS / f"{_n:05d}.png")
    _n += 1


def _quieto(titulo, detalle, segundos=PAUSA):
    for _ in range(max(1, int(segundos * FPS))):
        _cuadro(titulo, detalle)


def _animar(titulo, detalle, q_a, q_b, segundos):
    _, angulos, _, _ = trayectorias.evolucion(q_a, q_b, segundos, FPS)
    for q in angulos:
        escena.mover(q)
        _cuadro(titulo, detalle)


def correr(titulo, pasos, q):
    """Igual que rutinas.ejecutar, pero grabando cada cuadro."""
    global _vista
    if titulo.startswith("2"):
        _vista = p.computeViewMatrixFromYawPitchRoll(*CAMARA_CARGA[:1], *CAMARA_CARGA[1:], 0, 2)
    for paso in pasos:
        tipo, texto = paso[0], paso[1]

        if tipo == "mover":
            destino, motivo = trayectorias.angulos(paso[2])
            if destino is None:
                _quieto(titulo, f"{texto}: {motivo}", 2)
                return q, False
            libre, problemas = trayectorias.camino_seguro(q, destino)
            if not libre:
                escena.mover(q)
                _quieto(titulo, f"{texto}: {trayectorias.resumen(problemas)}", 2)
                return q, False
            _animar(titulo, texto, q, destino, paso[3])
            escena.mover(destino)
            q = destino
        else:
            q, informe = rutinas.ejecutar([paso], q)
            if informe[-1][1] != "ok":
                _quieto(titulo, f"{texto}: {informe[-1][1]}", 2)
                return q, False
            if tipo == "escenario":
                _vista = _vista_de(paso[2])
            escena.mover(q)
        _quieto(titulo, texto)
    return q, True


def main():
    CARPETA.mkdir(exist_ok=True)

    escena.cargar(con_ventana=False)
    q = rutinas.PLEGADO
    escena.mover(q)
    _quieto("Brazo RAUL-M", "pose de reposo, con la que RAUL circula", 1.5)

    etapas = (("1 · Tomar objeto de la mesa", rutinas.tomar_de_mesa),
              ("2 · Depositar en la caja de carga", rutinas.depositar_en_carga),
              ("3 · Presionar el boton del ascensor", rutinas.presionar_boton))

    for titulo, construir in etapas:
        q, bien = correr(titulo, construir(), q)
        if not bien:
            raise RuntimeError(f"FALLA en {titulo}: no se publica una demostración incompleta")
        if titulo.startswith("2"):
            destino = rutinas.PLEGADO
            libre, _ = trayectorias.camino_seguro(q, destino)
            if libre:
                _animar(titulo, "volver a la pose de reposo", q, destino, 3)
                q = destino
            else:
                raise RuntimeError("No se pudo volver al reposo")
            _quieto(titulo, "volver a la pose de reposo")

    _quieto("Las tres rutinas, sin choques", "verificado cada 2 grados del camino", 2)
    p.disconnect()

    print(f"{_n} cuadros, {_n/FPS:.1f} s de simulación. Codificando GIF...")
    cuadros=[]
    for archivo in sorted(CUADROS.glob('*.png')):
        with Image.open(archivo) as im:
            cuadros.append(im.convert('P', palette=Image.Palette.ADAPTIVE, colors=128))
    cuadros[0].save(SALIDA, save_all=True, append_images=cuadros[1:],
                    duration=round(1000/FPS), loop=0, optimize=False, disposal=2)
    # Solo se borra el directorio temporal creado por este proceso.
    assert CUADROS.resolve().parent == Path(tempfile.gettempdir()).resolve()
    shutil.rmtree(CUADROS)
    print(f"Demostración guardada en {SALIDA}")

if __name__ == "__main__":
    main()
