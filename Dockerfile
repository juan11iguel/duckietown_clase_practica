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