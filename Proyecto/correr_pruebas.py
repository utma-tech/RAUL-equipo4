"""Ejecuta las seis suites y guarda evidencia de esta revisión del código.
Desde la raíz: python correr_pruebas.py. Requiere g++ y environment.yml.
"""
import hashlib, importlib.metadata, os, platform, shutil, subprocess, sys, tempfile
from datetime import datetime, timezone
from pathlib import Path
AQUI=Path(__file__).resolve().parent
SALIDA=AQUI/'Entregables/evidencia_pruebas.txt'

def main():
    lineas=[]
    def escribir(s):
        print(s,flush=True); lineas.append(s)
    escribir('EVIDENCIA DE PRUEBAS RAUL-M — Equipo 4')
    escribir('Fecha UTC: '+datetime.now(timezone.utc).isoformat(timespec='seconds'))
    escribir('Plataforma: '+platform.platform())
    escribir('Python: '+platform.python_version())
    for mod in ('numpy','pybullet-arm64','matplotlib'):
        try: version=importlib.metadata.version(mod)
        except importlib.metadata.PackageNotFoundError: version='NO INSTALADO'
        escribir(mod+': '+version)
    # Huella del código y de los datos fuente: identifica cambios aun sin commit.
    h=hashlib.sha256()
    for f in sorted(AQUI.rglob('*')):
        if f.is_file() and f.suffix in ('.py','.cpp','.h','.ino','.urdf','.yml') and '.git' not in f.parts:
            h.update(f.relative_to(AQUI).as_posix().encode());h.update(f.read_bytes())
    escribir('SHA256 código/modelo: '+h.hexdigest())
    env=dict(os.environ,PYTHONIOENCODING='utf-8',MPLBACKEND='Agg')
    gpp=shutil.which('g++')
    if not gpp and Path(r'C:\msys64\ucrt64\bin\g++.exe').exists():
        gpp=r'C:\msys64\ucrt64\bin\g++.exe'
    if gpp: env['PATH']=str(Path(gpp).parent)+os.pathsep+env.get('PATH','')
    fallas=[]
    def run(nombre,cmd,cwd):
        escribir('\n'+nombre)
        try:
            r=subprocess.run(cmd,cwd=cwd,env=env,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=900)
            escribir((r.stdout+r.stderr).strip());ok=r.returncode==0
        except (OSError,subprocess.TimeoutExpired) as e:
            escribir(str(e));ok=False
        escribir('RESULTADO: '+('OK' if ok else 'FALLA'))
        if not ok: fallas.append(nombre)
        return ok
    sim=AQUI/'simulacion';pr=AQUI/'pruebas';fw=AQUI/'programa_esp32'
    casos_ok=run('Cinemática Python y URDF',[sys.executable,'pruebas.py'],sim)
    if not gpp:
        fallas.append('C++ no verificado: falta g++');escribir(fallas[-1])
    else:
        with tempfile.TemporaryDirectory(prefix='raul_pruebas_') as temp:
            for nombre,fuentes in [('cinematica',[pr/'prueba_cinematica.cpp',fw/'cinematica.cpp']),('movimiento',[pr/'prueba_movimiento.cpp',pr/'simulacro/simulacro.cpp',fw/'movimiento.cpp',fw/'cinematica.cpp'])]:
                exe=Path(temp)/(nombre+('.exe' if os.name=='nt' else ''))
                cmd=[gpp,'-I'+str(pr/'simulacro'),'-I'+str(fw),'-o',str(exe),*map(str,fuentes)]
                if run('Compilación C++ '+nombre,cmd,pr) and (casos_ok or nombre=='movimiento'):
                    run('Prueba C++ '+nombre,[str(exe)],pr)
    for script in ('pruebas_rutinas.py','pruebas_error.py','pruebas_modelo.py'):
        run(script,[sys.executable,script],sim)
    escribir('\n'+('VERIFICACIÓN COMPLETA: seis suites aprobadas y dos compilaciones C++ correctas.' if not fallas else 'VERIFICACIÓN INCOMPLETA: '+', '.join(fallas)))
    escribir('Alcance: simulación cinemática y C++ de escritorio; no prueba física ni compilación para ESP32.')
    SALIDA.parent.mkdir(exist_ok=True);SALIDA.write_text('\n'.join(lineas)+'\n',encoding='utf-8')
    return bool(fallas)
if __name__=='__main__':sys.exit(main())
