---
title: Option 2
has_children: false
nav_order: 2
parent: Replace the Physical Twin with a Digital Twin Prototype
---


# Only the Digital Twin starts a simulation
If you want to develop new features for the PiCar-X's digital twin and monitor the communication between the PiCar-X's digital twin and the digital twin prototype, you can start the digital twin and a reduced digital twin prototype, which does not include the GAZEBO simulation. In that case, you can just use inside a Docker container of the digital twin prototype the *echo* function provided by ROS on the topic you want to monitor.

## Adjust the environemnt files

There are three environment files with relevant variables:
- picarx.env (for the DTP)
- picarx-dt.env
- simulation.env
- simulation-dt.env


## Start the Digital Twin with GAZEBO

```console
# Build Docker Containers if not previously done
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dtsim.yml build 

# Start the DT
docker compose -f docker-compose-dtsim.yml up
```

## Start the Digital Twin Prototype without GAZEBO

```console
docker compose -f docker-compose-dtp-no-gazebo.yml up
```

## Print all Incoming Status Messages on the Digital Twin Prototype

To subscribe to any topic and get the content, you need to switch into one of the digital twin prototype's Docker containers:

```console
docker exec -it picar-x-ackermann_skill-dtp-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash
```

There you can list all topics via:

```console
rostopic list
```

Afterwards, you can subscribe to one of the topics, e.g. the command topic that receives all commands from the digital twin:

```console
rostopic echo /picarx/drive/command

# The console should now print all incoming messages with the corresponding command from the DT.
```

## Operate the Digital Twin Prototype from the Digital Twin

To operate the digital twin prototype, you need to switch into the container:

```console
docker exec -it picar-x-ackermann_skill-dt-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash
```

Inside the container you can publish a command message:


```console
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The DT sends a command to the DTP and there the command is printed.
```