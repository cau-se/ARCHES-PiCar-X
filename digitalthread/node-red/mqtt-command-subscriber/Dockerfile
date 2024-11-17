ARG TAG=latest

FROM ghcr.io/cau-se/arches-picar-x/arches-mqtt-bridge:${TAG:-latest}

RUN pip3 install paho-mqtt 

COPY . ./src/digitalthread/node-red/command-subscriber-mqtt

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]
