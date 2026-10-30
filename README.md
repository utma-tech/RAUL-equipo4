RAUL-M — Equipo 4
Proyecto 2 de Tecnología de Control y Robótica — UTEC ITR Oeste, 2026.

El proyecto utiliza el modelo RAUL_M.step y las medidas de la ficha RAUL_M_V4 del Equipo 3.

Contenido del proyecto
Entregables/: informe en Word y PDF, GIF de la simulación, figuras y resultados de las pruebas.
simulacion/: modelo URDF, cinemática, trayectorias y rutinas en PyBullet.
programa_esp32/: código de control para ESP32.
pruebas/: pruebas de funcionamiento.
GUIA_EQUIPO.md: información técnica y resultados del proyecto.
RAUL_M.step: modelo del brazo proporcionado por el Equipo 3.
Cómo ejecutar
Desde Miniforge Prompt, ubicarse en la carpeta del proyecto y ejecutar:

conda activate raul_sim
python correr_pruebas.py
python simulacion/demo_en_vivo.py
Para controlar el brazo manualmente:

python simulacion/modo_articular.py
python simulacion/modo_cartesiano.py
Instalación en Windows
Se utilizó Python 3.11.18 con PyBullet en el entorno raul_sim.

Si el entorno ya está instalado y funciona, no hace falta volver a configurarlo.

Para instalarlo desde cero, ejecutar los siguientes comandos por separado:

conda create -n raul_sim python=3.11 pip -y
conda activate raul_sim
python -m pip install pybullet-arm64==3.2.8 numpy matplotlib pillow
Comprobar la instalación:

python -c "import pybullet; print('PyBullet instalado correctamente')"
Generar las pruebas y resultados
Para actualizar los gráficos, la simulación y la evidencia:

python simulacion/mapa_alcance.py
python simulacion/graficos_trayectoria.py
python simulacion/error_posicionamiento.py
python simulacion/torques.py
python simulacion/ficha_medidas.py
python simulacion/grabar_demo.py
python correr_pruebas.py
Los gráficos se guardan en Entregables/figuras/.

Control con ESP32
El archivo programa_esp32.ino se puede abrir en Arduino IDE.

El programa permite controlar los servomotores mediante el PCA9685. Todavía no incluye las tres rutinas autónomas completas ni se realizaron pruebas físicas con el prototipo.

Observaciones
Las medidas del brazo corresponden a la ficha V4 del Equipo 3.
La plataforma de 350 mm y el alzador de 233 mm son medidas propuestas para la simulación.
Se comprobaron las trayectorias y posibles colisiones en PyBullet.
Los dos servomotores MG995 del hombro no alcanzan el torque necesario para levantar 0,2 kg con el brazo completamente extendido.
La simulación incluye las tres rutinas: tomar un objeto, depositarlo en la plataforma y presionar un botón.
Queda pendiente grabar una demostración MP4 en vivo y realizar las pruebas con el prototipo físico.
Cambios futuros
Si se modifican las medidas o los límites del brazo, hay que actualizar los archivos parametros.py, configuracion.h y brazo.urdf.

Después se deben ejecutar nuevamente las pruebas y generar los gráficos y el informe actualizado.
