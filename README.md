# Manual para preparar clase práctica
Todos los archivos necesarios deben encontrarse en el mismo directorio que este manual.

### Generar imagen con archivos modificados
Primero hay que generar un archivo `Dockerfile` con estructura:
```
FROM duckietown/rpi-duckiebot-base
#
# Identify the maintainer of an image
LABEL maintainer="Juan Miguel Serrano Rodríguez (juan11iguel@gmail.com)"
#
# Copy seed file to calibrations folder 
ADD https://github.com/Juasmis/duckietown_clase_practica/blob/main/duckiebot_random_seed.yaml /data/config/calibrations/
#
# Copy modified inverse_kinematics_node program
ADD https://github.com/Juasmis/duckietown_clase_practica/blob/main/inverse_kinematics_node.py /home/software/catkin_ws/src/06-kinematics/dagu_car/src/ 
#
# Copy modified lane_controller_node program
ADD https://github.com/Juasmis/duckietown_clase_practica/blob/main/lane_controller_node.py /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/
```

Y construir imagen con:
```shell
docker build -t imagen_prueba https://github.com/Juasmis/duckietown_clase_practica.git#main
```

Sincronizar en [dockerHub](https://hub.docker.com/):
```shell
docker login
```
(Si no se está ya logeado)
```shell
docker tag imagen_prueba patomareao/duckietown_ual:practicaLab_seed3
```
```shell
docker push patomareao/duckietown_ual:practicaLab_seed3
```
### Generar contenedor a partir de imagen modificada
Descargar imagen modificada en RPi con:
```bash
docker -H DUCKIEBOT_NAME.local pull duckietown_ual/clase_practica_X
```
(X del 1 al 7)

Generar contenedor con parámetros adecuados con (esto todavía lo tengo que comprobar mirando `/var/local/DT18_05_duckiebot_base.yaml`).
Hay que generar un archivo (`practica.yaml`) con esta estructura (de nuevo, mirar `duckiebot_base.yaml`):
```yaml
version: '3'
services:

  portainer:
    image: portainer/portainer:linux-arm
    command: ["--host=unix:///var/run/docker.sock", "--no-auth"]
    restart: always
    network_mode: "host"
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock
```
Y después generar el contendor con:
```bash
docker-compose -H DUCKIEBOT_NAME.local -f practica.yaml up
```
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

