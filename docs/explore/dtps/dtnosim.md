---
title: Option 1
has_children: false
nav_order: 1
parent: Replace the Physical Twin with a Digital Twin Prototype
---

# Only the Digital Twin Prototype starts a simulation
If you want to develop new features for the PiCar-X and monitor the communication between the PiCar-X's digital twin prototype and its digital twin, you can start the digital twin prototype and a reduced digital twin, which does not include the GAZEBO simulation. In that case, you can just use inside a Docker container of the digital twin the *echo* function provided by ROS on the topic you want to monitor.

## Adjust the environment files

There are three environment files with relevant variables:
- picarx.env (for the DTP)
- picarx-dt.env
- simulation.env
- simulation-dt.env

## Start the Digital Twin without GAZEBO
```console
# Build Docker Containers if not previously done
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dt.yml build 

# Start the DT
docker compose -f docker-compose-dt.yml up
```

## Start the Digital Twin Prototype with GAZEBO
```console
# Start the DTP
docker compose -f docker-compose-dtp.yml up
```

## Print all Incoming Status Messages on the Digital Twin

To subscribe to any topic and get the content, you need to switch into one of the Docker containers:

```console
docker exec -it picar-x-ackermann_skill-dt-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash
```

There you can list all topics via:

```console
rostopic list
```

Afterwards, you can subscribe to one of the topics, e.g. the status topic that receives all status changes from the PiCar-X:

```console
rostopic echo /drive/status

# The console should now print all incoming messages with the corresponding status from the DTP.
```

## Operate the Digital twin Prototype

To operate the digital twin prototype, you need to switch into the container:

```console
docker exec -it picar-x-ackermann_skill-dtp-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash
```

Inside the container you can publish a command message:


```console
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The status should be sent automatically to the DT and the DT prints all incoming messages.
```