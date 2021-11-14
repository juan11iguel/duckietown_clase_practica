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