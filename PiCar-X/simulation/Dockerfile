ARG TAG=latest
FROM osrf/ros:noetic-desktop-full-focal AS gazebo-full
ARG TAG
WORKDIR /root/catkin_ws
RUN apt-get update \
    && apt-get install -y python3-pip python3-setuptools python3-yaml ros-noetic-ros-control ros-noetic-ros-controllers python3-catkin-tools curl \
    && apt-get clean \
    && update-ca-certificates -f

FROM gazebo:libgazebo9-bionic AS gazebo-only 
ARG TAG
WORKDIR /root/catkin_ws
RUN apt-get update \
    && apt-get install -y python3-pip python3-setuptools python3-yaml ros-noetic-ros-control ros-noetic-ros-controllers python3-catkin-tools curl \
    && apt-get clean \
    && update-ca-certificates -f

ARG TAG
FROM ghcr.io/cau-se/arches-picar-x/picarx:${TAG} AS picarx-core

FROM gazebo-full AS desktop
COPY --from=picarx-core /root/catkin_ws .
COPY . ./src/simulation
RUN ls .
RUN /bin/bash -c "source /opt/ros/noetic/setup.sh && catkin build" \
    && touch /root/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source ./install/setup.bash" >> /root/.bashrc
ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]

FROM gazebo-only AS tests
COPY --from=picarx-core /root/catkin_ws .
COPY . ./src/simulation
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build" \
    && touch /root/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source ./install/setup.bash" >> /root/.bashrc
ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]