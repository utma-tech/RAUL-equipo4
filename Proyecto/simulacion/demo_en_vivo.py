"""Ejecución visible y trazable de un ciclo. La captura de ventana se realiza con OBS.

python simulacion/demo_en_vivo.py
Opcional: --senal ruta para esperar a que el grabador cree ese archivo.
La ventana permanece abierta al terminar; no reinicia una secuencia fallida.
"""
import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
import pybullet as p
import escena
import rutinas

ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--senal', type=Path)
    args = parser.parse_args()
    salida = ROOT / 'Entregables/registro_ejecucion_vivo.json'
    eventos = []
    inicio = time.perf_counter()

    def anotar(etapa, paso, resultado):
        eventos.append(dict(segundos=round(time.perf_counter()-inicio, 3),
                            fecha_utc=datetime.now(timezone.utc).isoformat(),
                            etapa=etapa, paso=paso, resultado=resultado))
        salida.write_text(json.dumps(eventos, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'{etapa}: {paso} -> {resultado}', flush=True)

    def camara(etapa):
        vistas = {
            'mesa': (1.00, 40, -18, [0.18, 0.03, 0.14]),
            'carga': (0.95, 10, -20, [0.10, 0.09, 0.10]),
            'boton': (1.10, -150, -12, [0.15, 0.00, 0.29]),
        }
        p.resetDebugVisualizerCamera(*vistas[etapa])

    escena.cargar(True, '--width=1280 --height=720')
    camara('mesa')
    q = list(rutinas.PLEGADO)
    escena.mover(q)
    escena.mostrar_estado('RAUL-M V4 - Preparado para grabar', True)
    anotar('preparacion', 'ventana abierta; brazo en reposo', 'ok')
    if args.senal:
        while p.isConnected() and not args.senal.exists():
            time.sleep(0.1)
    else:
        time.sleep(5)
    if not p.isConnected():
        return 1
    inicio = time.perf_counter()
    anotar('inicio', 'ejecucion visible', 'ok')

    def ejecutar(etapa, pasos):
        nonlocal q
        for paso in pasos:
            if not p.isConnected():
                raise RuntimeError('Ventana cerrada durante la ejecución')
            escena.mostrar_estado(etapa + ': ' + paso[1], True)
            anotar(etapa, paso[1], 'inicio')
            q, informe = rutinas.ejecutar([paso], q, animar=True)
            estado = informe[-1][1]
            anotar(etapa, paso[1], estado)
            if estado != 'ok':
                raise RuntimeError(estado)
            time.sleep(0.8)

    try:
        time.sleep(2)
        camara('mesa')
        ejecutar('1 Tomar en mesa', rutinas.tomar_de_mesa())
        camara('carga')
        ejecutar('2 Depositar en carga', rutinas.depositar_en_carga())
        escena.mostrar_estado('3 Regresar al reposo', True)
        anotar('3 Regreso', 'volver al reposo', 'inicio')
        q, estado = rutinas.ir_a(rutinas.PLEGADO, q, segundos=3, animar=True)
        anotar('3 Regreso', 'volver al reposo', estado)
        if estado != 'ok':
            raise RuntimeError(estado)
        time.sleep(2)
        camara('boton')
        ejecutar('4 Aproximar al boton', rutinas.presionar_boton())
        escena.mostrar_estado('FINALIZADO - Todas las etapas verificadas', True)
        anotar('fin', 'ciclo completo', 'ok')
    except Exception as error:
        anotar('fin', str(error), 'FALLA')
        if p.isConnected():
            escena.mostrar_estado('FALLA: ' + str(error), False)
    while p.isConnected():
        time.sleep(0.2)
    return 0 if eventos[-1]['resultado'] == 'ok' else 1


if __name__ == '__main__':
    raise SystemExit(main())
