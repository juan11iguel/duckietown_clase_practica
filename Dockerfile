FROM duckietown/rpi-duckiebot-base
#

# This part is taken from "Breandan Considine breandan.considine@umontreal.ca" Dockerfile
COPY requirements.txt /requirements.txt
# otherwise installation of Picamera fails https://github.com/resin-io-projects/resin-rpi-python-picamera/issues/8
ENV READTHEDOCS True
# Upgrade pip
RUN python -m pip install --upgrade pip   
# Install all the required dependencies
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

# Update packages
RUN apt-get update && apt-get upgrade -y