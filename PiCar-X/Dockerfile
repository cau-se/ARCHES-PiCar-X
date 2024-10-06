ARG TAG=latest
ARG ARCH=''

FROM ${ARCH:+${ARCH}/}ros:noetic AS rosnoetic
WORKDIR /root/catkin_ws/
RUN apt update -y \
    && apt-get install -y python3-pip python3-setuptools python3-catkin-tools wget unzip curl nano \
    && apt-get clean \
    && update-ca-certificates -f \
    && pip3 install pytest pytest-cov avro

FROM rosnoetic AS adtf
RUN mkdir -p ./src/arches \
    && wget -c https://git.geomar.de/open-source/arches/arches_msgs/-/archive/master/arches_msgs-master.tar.gz -O - | tar -xz \
    && mv arches_msgs-master ./src/arches/arches_msgs \
    && wget -c https://git.geomar.de/open-source/arches/arches_core/-/archive/main/arches_core-main.tar.gz -O - | tar -xz \
    && mv arches_core-main ./src/arches/arches_core 
COPY ./core ./src/core
COPY ./ros/picarx_msgs ./src/picarx_msgs
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin config --isolate-devel --install && catkin build" \
    && pip3 install -e ./src/core/picarx \
    && touch /root/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source ./install/setup.bash" >> /root/.bashrc
ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]
