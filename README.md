# Manual para preparar clase práctica
Todos los archivos necesarios deben encontrarse en el mismo directorio que este manual.

### Generar imagen con archivos modificados
Primero hay que generar un archivo `Dockerfile` con estructura:
```docker
FROM duckietown/rpi-duckiebot-base:master19
#

# Install all the required dependencies
# This part is taken from "Breandan Considine breandan.considine@umontreal.ca" Dockerfile
# RUN [ "cross-build-start" ]
COPY requirements.txt /requirements.txt
# otherwise installation of Picamera fails https://github.com/resin-io-projects/resin-rpi-python-picamera/issues/8
ENV READTHEDOCS True
RUN pip install -r /requirements.txt
RUN bash -c "source /home/software/docker/env.sh && python -c 'import duckietown_utils'"

# Only need to install the dependencies, leave building the ROS workspace for now
#RUN mkdir /home/software
#COPY . /home/software/
#ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
#RUN /bin/bash -c "cd /home/software/ && source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"
#RUN echo "source /home/software/docker/env.sh" >> ~/.bashrc
# RUN [ "cross-build-end" ]

# Identify the maintainer of an image
LABEL maintainer="Juan Miguel Serrano Rodríguez (juan11iguel@gmail.com)"

# Copy seed file to calibrations folder 
COPY duckiebot_random_seed.yaml /data/config/calibrations/

# Copy modified inverse_kinematics_node program
RUN rm /home/software/catkin_ws/src/06-kinematics/dagu_car/src/inverse_kinematics_node.py
COPY inverse_kinematics_node.py /home/software/catkin_ws/src/06-kinematics/dagu_car/src/

# Copy modified lane_controller_node program
RUN rm /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/lane_controller_node.py
COPY lane_controller_node.py /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/

# # Upgrade pip
# FROM python:2.7.13
# RUN pip install --upgrade pip

# Update and install some packages
ARG CACHEBUST=1 
RUN apt-get update
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
# RUN sudo dpkg --configure -a
RUN sudo apt-get install -y -q
RUN apt-get install dialog apt-utils -y
RUN apt-get install nano -y
RUN apt-get install curl -y
RUN curl https://getmic.ro | bash
```

~~Y construir imagen con:~~
```shell
docker build -t imagen_prueba https://github.com/Juasmis/duckietown_clase_practica.git#main
```

o localmente, preparar para construir imagen ARM:
```docker
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes 
docker buildx rm builder
docker buildx create --name builder --driver docker-container --use  
docker buildx inspect --bootstrap
```
Construir imagen:
```shell
 docker buildx build --platform linux/arm --push --tag patomareao/duckietown_ual:practicaLab_seed5 ~/duckietown_clase_practica
```
Se especifica la arquitectura que es *arm64* para la RPi, con *buildx* es mucho más rápido.

**NOTA**: Si de un día para otro da error, volver a paso de construir imagen ARM y ejecutar de nuevo.

Sincronizar en [dockerHub](https://hub.docker.com/) (si no se está ya logeado):
```shell
docker login
```
~~Etiquetar imagen:~~
```shell
docker tag imagen_prueba patomareao/duckietown_ual:practicaLab_seed3
```
~~Y finalmente subir en repositorio:~~
```shell
docker push patomareao/duckietown_ual:practicaLab_seed3
```
Con buildx no es necesario, directamente se sube.

### Generar contenedor a partir de imagen modificada
Descargar imagen modificada en RPi con:
```bash
docker -H DUCKIEBOT_NAME.local pull patomareao/duckietown_ual:practicaLab_seed5
```

o directamente con:
```docker
docker -H duckiebot1.local run -it --net host --memory="800m" --memory-swap="2.8g" --privileged --name clase_practica_1 -t -i patomareao/duckietown_ual:practicaLab_seed5
```
Visto en documentación [versión 2018](https://docs.duckietown.org/DT18/opmanual_duckiebot/out/demo_lane_following.html).

### Modificaciones a programas para práctica
Común para ambas tareas:
- Archivo `duckiebot_random_seed.yaml`  en `/data/config/calibrations/` con contenido:
```yaml
# seed debe ser un número comprendido entre 1 y 7, asigna un seed distinto a cada imagen practica_laboratorio
# de manera que haya siete imágenes practica_laboratorio_1 cada una con un seed distinto
seed: 3
```

#### Tarea 1.1 (cinemática)
Modificación de `inverse_kinematics_node.py`
Aunque se manipulen los valores del archivo de calibración de cinemática inversa  `duckiebot.yaml`, es fácil darse cuenta de que los valores adecuados van a ser 1 para *gain* y 0 o ligeramente distinto para `trim`. Por ello en el *script* `inverse_kinematic_node.py` se introduce la siguiente modificación (línea ~190):
```python
import yaml
import time
import os.path
from duckietown_utils import get_duckiefleet_root
import random
									.
									.
									.
# adjusting k by gain and trim
# Modificado para que el valor adecuado no sea el valor por defecto 
# de gain = 1 y trim = 0 
file_name = get_duckiefleet_root() + '/calibrations/' + 'duckiebot_random_seed' + ".yaml"

# Open configuration
with open(file_name, 'r') as archivo:
	config = yaml.safe_load(archivo)

random.seed(config['seed'])

opciones = [-15, -12, -10, -8, 8, 10, 12, 15]
offset_introducido = random.choice(opciones)
# offset_introducido = minimo + (value * (maximo - minimo))

# assuming same motor constants k for both motors
k_r = self.k + offset_introducido
k_l = self.k - offset_introducido
```
Que provoca que en base a la semilla del archivo de configuración se genere un offset aleatorio a ambos parámetros único para cada imagen.

#### Tarea 1.2 (controlador)
Siguiendo la misma estrategia anterior se modifica la ganancia proporcional e integral del controlador para controlar el desplazamiento respecto al centro del carril aplicando un offset a a las ganancias en base a un número aleatorio:
```python
import yaml
import os
import random
from duckietown_utils import get_duckiefleet_root

										.
										.
										.

# Ganancia proporcional e integral del control de desplazamiento respecto a centro de carril
self.k_d = self.setupParameter("~k_d",k_d_fallback)             # P gain for d
self.k_Id = self.setupParameter("~k_Id", k_Id_fallback)         # gain for integrator of d
# MODIFICADO para clase práctica, se añade offset a Kp y Ki aleatoriamente en base a semilla
file_name = get_duckiefleet_root() + '/calibrations/' + 'duckiebot_random_seed' + ".yaml"

# Open configuration
with open(file_name, 'r') as archivo:
	config = yaml.safe_load(archivo)

random.seed(config['seed'])

opciones = [-5, -6, -7, -4, 2, 3, 3.4]
offset_Kp = random.choice(opciones)

opciones = [-0.9, -0.5, 5, 6, 7, 10]
offset_Ki = random.choice(opciones)
# offset_Kp = min + (value[0] * (max - min))

# min = 0; max = 10
# offset_Ki = min + (value[1] * (max - min))

self.k_d  = self.k_d  + offset_Kp
self.k_Id = self.k_Id + offset_Ki
```

