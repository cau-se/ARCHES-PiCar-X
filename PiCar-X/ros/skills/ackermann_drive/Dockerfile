ARG TAG=latest

FROM ghcr.io/cau-se/arches-picar-x/picarx:${TAG}

COPY . ./src/skills/ackermann_drive

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]