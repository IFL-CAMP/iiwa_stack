FROM ros:kinetic-ros-base

# use bash to be able to source the ros files
SHELL ["/bin/bash", "-c"]

# upgrade ros packages
RUN apt-get update && \
    apt-get install --only-upgrade ros-kinetic-* -y && \
    apt-get install python-wstool -y

# setup catkin_ws for java 6 message generation  
RUN mkdir -p /catkin_ws
WORKDIR /catkin_ws
RUN source /ros_entrypoint.sh && \
    wstool init src && \
    wstool set -y -t src genjava --git https://github.com/Tuebel/genjava.git --version=kinetic_java6 && \
    wstool update -t src && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make

# add and generate the message.
ADD . /catkin_ws/src/iiwa_msgs

# disbale message generation for all but java if using catkin_make.
# note that the jars generated via catkin_make seem to be empty
# ENV ROS_LANG_DISABLE "gencpp;geneus;genlisp;gennodejs;genpy"
