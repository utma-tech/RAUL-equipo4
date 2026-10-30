# Geometría y límites del brazo. Los mismos valores están en configuracion.h y en
# brazo.urdf; si no coinciden, fallan pruebas.py o prueba_cinematica.cpp.

from math import hypot, atan2, degrees

# Cuatro articulaciones y la pinza: base, hombro, codo y muñeca. Sin giro de muñeca.
NOMBRES = ("base", "hombro", "codo", "muneca")
N = 4

# Longitudes (mm), medidas sobre el CAD del Equipo 3. Los tres ejes están a la misma
# altura y son paralelos, así que el brazo es plano.
D1 = 131   # cara de montaje de la placa → eje del hombro
A2 = 200   # hombro → codo
A3 = 170   # codo → muñeca

# La pinza cuelga 18 mm por debajo del eje de la muñeca. Las dos componentes van por
# separado porque así se miden en el CAD.
LARGO_PINZA = 80   # muñeca → punta, a lo largo del eje del antebrazo
CAIDA_PINZA = 18   # cuánto cuelga la punta por debajo de ese eje

# Lo que usa la cinemática: distancia muñeca → punta y el ángulo con que cae.
D5 = hypot(LARGO_PINZA, CAIDA_PINZA)        # 82,0 mm
FI = degrees(atan2(CAIDA_PINZA, LARGO_PINZA))   # 12,68°

# Límites articulares (grados). Son los 180° que da el servo MG995. Hay que recortarlos
# calibrando el brazo real.
#        base  hombro  codo  muñeca
QMIN = [ -90,     0,   -150,   -90]
QMAX = [  90,   180,     30,    90]

# Pose de reposo (grados). Igual que REPOSO en configuracion.h. Deja la punta a 165 mm
# del eje, plegada sobre la base.
REPOSO = [0, 90, -150, 60]
