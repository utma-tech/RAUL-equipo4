from pathlib import Path
import random, re, math
import pybullet as p
import escena, rutinas, parametros
from torques import torques

escena.cargar(False)
p.setGravity(0,0,-9.81)
# V4 ya incluye todos los componentes del conjunto de muñeca y pinza.
masa=sum(p.getDynamicsInfo(escena.robot,i)[0] for i in range(p.getNumJoints(escena.robot)))
assert abs(masa-(.363+.266+.281+.186)) < 1e-9, masa
azar=random.Random(4)
max_error=0
for _ in range(100):
    q=[azar.uniform(a,b) for a,b in zip(parametros.QMIN,parametros.QMAX)]
    tau=p.calculateInverseDynamics(escena.robot,[math.radians(v) for v in q]+[.0139,.0139],[0.]*6,[0.]*6)
    max_error=max(max_error,max(abs(a-b) for a,b in zip(tau[:4],torques(q))))
assert max_error < 1e-8,max_error
print(f'OK masas V4 sin doble conteo; estática analítica vs URDF: {max_error:.2e} N*m')
for nombre in ('mesa','ascensor',None):
    escena.poner_escenario(nombre)
    assert 'pedestal' in escena.obstaculos
print('OK pedestal conservado en todos los escenarios')
f=(Path(__file__).parent.parent/'programa_esp32/programa_esp32.ino').read_text(encoding='utf-8')
for nombre,pose in [('MESA',rutinas.poses_tarea()['tomar en mesa']),('BOTON',rutinas.poses_tarea()['boton ascensor'])]:
    valores=[float(v) for v in re.search(r'Pose '+nombre+r'\s*=\s*\{([^}]+)',f)[1].split(',')]
    assert max(abs(a-b) for a,b in zip(valores,pose))<1e-6,(valores,pose)
print('OK poses de ejemplo ESP32 iguales a los TCP de las rutinas')
p.disconnect()
