# Manual para preparar clase práctica
Todos los archivos necesarios deben encontrarse en el mismo directorio que este manual.

### Generar imagen con archivos modificados
Primero hay que generar un archivo `Dockerfile` con estructura:
```docker
FROM duckietown/rpi-duckiebot-base:master19
#

# Install all the required dependencies
# This part is taken from "Breandan Considine breandan.considine@umontreal.ca" Dockerfile
RUN [ "cross-build-start" ]
COPY requirements.txt /requirements.txt
# otherwise installation of Picamera fails https://github.com/resin-io-projects/resin-rpi-python-picamera/issues/8
ENV READTHEDOCS True
RUN pip install -r /requirements.txt
# Only need to install the dependencies, leave building the ROS workspace for now
#RUN mkdir /home/software
#COPY . /home/software/
#ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
#RUN /bin/bash -c "cd /home/software/ && source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"
#RUN echo "source /home/software/docker/env.sh" >> ~/.bashrc

# Identify the maintainer of an image
LABEL maintainer="Juan Miguel Serrano Rodríguez (juan11iguel@gmail.com)"

# Copy seed file to calibrations folder 
ADD https://github.com/Juasmis/duckietown_clase_practica/blob/main/duckiebot_random_seed.yaml /data/config/calibrations/

# Copy modified inverse_kinematics_node program
ADD https://github.com/Juasmis/duckietown_clase_practica/blob/main/inverse_kinematics_node.py /home/software/catkin_ws/src/06-kinematics/dagu_car/src/ 

# Copy modified lane_controller_node program
ADD https://github.com/Juasmis/duckietown_clase_practica/blob/main/lane_controller_node.py /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/

# Upgrade pip
FROM python:2.7.13
RUN pip install --upgrade pip

# Update and install some packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install nano
RUN apt-get install curl
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
docker -H DUCKIEBOT_NAME.local pull duckietown_ual/clase_practica_X
```

o directamente con:
```docker
docker -H duckiebot1.local run -it --net host --memory="800m" --memory-swap="2.8g" --privileged --name clase_practica_1 -a -t -i -e "DUCKIEBOT_IP=192.168.226.182" patomareao/duckietown_ual:practicaLab_seed5
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
# adjusting k by gain and trim
# Modificado para que el valor adecuado no sea el valor por defecto 
# de gain = 1 y trim = 0 
file_name = get_duckiefleet_root() + '/calibrations/kinematics/' + 'duckiebot_random_seed' + ".yaml"

# Open configuration
with open(file_name, 'r') as archivo:
	config = yaml.safe_load(archivo)

seed(config['seed'])

value = random()
min = -5; max = 5
offset_trim = min + (value * (max - min))

value = random()
min = 0.3; max = 5
offset_gain = min + (value * (max - min))

k_r_inv = (self.gain+offset_gain + self.trim+offset_trim) / k_r
k_l_inv = (self.gain+offset_gain - self.trim+offset_trim) / k_l
```
Que provoca que en base a la semilla del archivo de configuración se genere un offset aleatorio a ambos parámetros único para cada imagen.

#### Tarea 1.2 (controlador)
Siguiendo la misma estrategia anterior se modifica la ganancia proporcional e integral del controlador para controlar el desplazamiento respecto al centro del carril aplicando un offset a a las ganancias en base a un número aleatorio:
```python
# Modificado por JM
import yaml
import os
from random import seed, random
from duckietown_utils import get_duckiefleet_root

#...

# linea 150~
# Ganancia proporcional e integral del control de desplazamiento respecto a centro de carril
self.k_d = self.setupParameter("~k_d",k_d_fallback)             # P gain for d
self.k_Id = self.setupParameter("~k_Id", k_Id_fallback)         # gain for integrator of d
# MODIFICADO para clase práctica, se añade offset a Kp y Ki aleatoriamente en base a semilla
file_name = get_duckiefleet_root() + '/calibrations/' + 'duckiebot_random_seed' + ".yaml"

# Open configuration
with open(file_name, 'r') as archivo:
	config = yaml.safe_load(archivo)

seed(config['seed'])

value = random()
min = -2; max = 10
offset_Kp = min + (value * (max - min))

value = random()
min = 0; max = 10
offset_Ki = min + (value * (max - min))

self.k_d  = self.k_d  + offset_Kp
self.k_Id = self.k_Id + offset_Ki

```

