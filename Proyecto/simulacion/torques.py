"""Estática V4: masas agregadas, sin volver a sumar servos ni mordazas.
La derivada de la energía potencial da el par de sostén, en N*m.
Los valores MG995 son de bloqueo según la ficha del Equipo 3, no continuos.
"""
from math import sin, cos, radians
from itertools import product
from parametros import A2, A3, LARGO_PINZA, CAIDA_PINZA
G = 9.81
CARGA = 0.2
MASAS = (0.266, 0.281, 0.186)
COM = ((0.1386, 0), (0.125, 0), (0.0294, -0.0268))
BLOQUEO = {"4,8 V": 1.66, "6 V": 1.96}

def torques(q, carga=0.0):
    a,b,c = map(radians, (q[1],q[1]+q[2],q[1]+q[2]+q[3]))
    l1,l2 = A2/1000,A3/1000
    m1,m2,m3 = MASAS
    h1 = COM[0][0]*cos(a)-COM[0][1]*sin(a)
    h2 = COM[1][0]*cos(b)-COM[1][1]*sin(b)
    h3 = COM[2][0]*cos(c)-COM[2][1]*sin(c)
    tcp = LARGO_PINZA/1000*cos(c)+CAIDA_PINZA/1000*sin(c)
    muneca = G*(m3*h3+carga*tcp)
    codo = G*(m2*h2+(m3+carga)*l2*cos(b))+muneca
    hombro = G*(m1*h1+(m2+m3+carga)*l1*cos(a))+codo
    return [0.0,hombro,codo,muneca]

def resumen():
    maximos = [0.0]*4
    for a,b,c in product(range(0,181,10),range(-150,31,10),range(-90,91,10)):
        t = torques([0,a,b,c],CARGA)
        maximos = [max(v,abs(w)) for v,w in zip(maximos,t)]
    return {"horizontal_sin_carga":torques([0]*4),
            "horizontal_con_carga":torques([0]*4,CARGA),
            "maximo_muestreado_con_carga":maximos}

if __name__ == '__main__':
    datos=resumen()
    print('Modelo V4; carga 0,2 kg; g = 9,81 m/s²')
    print('Junta       Horizontal vacío  Horizontal con carga  Máximo malla 10°')
    for i,n in enumerate(('base','hombro','codo','muñeca')):
        if i:
            print(f"{n:10s} {datos['horizontal_sin_carga'][i]:14.4f} {datos['horizontal_con_carga'][i]:21.4f} {datos['maximo_muestreado_con_carga'][i]:16.4f} N*m")
    print('Dos MG995: bloqueo ideal sumado 1,66 N*m a 4,8 V; 1,96 N*m a 6 V.')
    print('El hombro excede el bloqueo incluso vacío: reducir la carga a cero no resuelve')
    print('el requisito. Requiere redimensionar actuadores/transmisión o compensar peso.')
    print('Estática solamente: no certifica aceleración, rozamiento ni torque continuo.')
