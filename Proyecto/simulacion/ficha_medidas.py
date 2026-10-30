"""Ficha geométrica generada con los mismos parámetros de la cinemática."""
from pathlib import Path
import matplotlib.pyplot as plt
from parametros import D1,A2,A3,LARGO_PINZA,CAIDA_PINZA
f,ax=plt.subplots(figsize=(10,4.4))
x=[0,0,A2,A2+A3,A2+A3+LARGO_PINZA]
z=[0,D1,D1,D1,D1-CAIDA_PINZA]
ax.plot(x,z,'o-',color='#244f73',lw=5,markersize=8)
ax.plot([-80,80],[0,0],color='#444444',lw=5)
for px,pz,txt in [(0,D1,'Hombro'),(A2,D1,'Codo'),(A2+A3,D1,'Muñeca'),(x[-1],z[-1],'TCP')]:
    ax.annotate(txt,(px,pz),xytext=(0,14),textcoords='offset points',ha='center',fontsize=10)
for a,b,label in [(0,A2,'200 mm'),(A2,A2+A3,'170 mm'),(A2+A3,x[-1],'80 mm')]:
    ax.annotate('',xy=(a,75),xytext=(b,75),arrowprops={'arrowstyle':'<->','color':'#555'})
    ax.text((a+b)/2,60,label,ha='center',fontsize=11)
ax.annotate('',xy=(-35,0),xytext=(-35,D1),arrowprops={'arrowstyle':'<->'})
ax.text(-45,D1/2,'131 mm',rotation=90,ha='center',va='center')
ax.annotate('Caída 18 mm',xy=(450,113),xytext=(420,15),arrowprops={'arrowstyle':'->'},ha='center')
ax.text(175,15,'Pose q = [0, 0, 0, 0]  |  TCP = (450, 0, 113) mm',ha='center',fontsize=10)
ax.set(xlim=(-85,510),ylim=(-15,190),xlabel='x [mm]',ylabel='z sobre placa [mm]',title='RAUL-M V4 · Geometría en el plano del brazo')
ax.set_aspect('equal');ax.grid(alpha=.15);f.tight_layout()
out=Path(__file__).parent.parent/'Entregables/figuras/ficha_medidas.png'
out.parent.mkdir(exist_ok=True,parents=True);f.savefig(out,dpi=160);print(out)
