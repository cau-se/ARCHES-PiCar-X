---
title: Connect Physical Twin and Digital Shadow
has_children: false
nav_order: 2
parent: Explore
---

<link rel="stylesheet" href="{{ site.baseurl }}{% link assets/css/tabs.css %}">
<script src="{{ site.baseurl }}{% link assets/js/tabs.js %}"> </script>

# Connecting the Physical Twin with a Digital Shadow
In this example, we demonstrate how the PiCar-X can be connected to a digital shadow. For this purpose, we created a special Docker container that contains a ROS node, which does not send any commands to the physical twin. The communication between both is established via MQTT. The MQTT server is part of the digital shadow and you have to allow connections to port 1883 on the system where the MQTT server runs. This may requires adjustments to your firewall.

## Adjust the environemnt files

There are three environment files with relevant variables:
- picarx.env (on the Picar-X)
- picarx-ds.env

In this files you have to adjust the I2C devices (see [Getting Started]({% link getting_started.md %})) and the MQTT server IP.

## Start the Digital Shadow
We assume at this point that the digital shadow runs on a X64 Linux system (e.g. a server).

```console
# Build and execute the Docker Containers
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-ds.yml build 
docker compose -f docker-compose-ds.yml up
```


## Start the Physical Twin
<div class="tab-container" id="activaterpi">
  <ul class="tab-list">
<li class="tab active" data-tab="tab3-1">arm32v7 (RPi3)</li>
<li class="tab" data-tab="tab3-2">arm64v8 (RPi4)</li>
  </ul>
  <div class="tab-content active" id="tab3-1">
  {% highlight console %}
# First copy all content to the RaspberryPi4
scp -r ./ <user>@<picarx-ip>:~/

# Login to the RaspberryPi via ssh
ssh <user>@<picarx-ip>

# Build and execute the Docker Containers
TAG=latest ARCH=arm32v7 docker compose -f docker-compose-core.yml build 
TAG=latest ARCH=arm32v7 docker compose -f docker-compose-pt.yml build 
TAG=latest ARCH=arm32v7 docker compose -f docker-compose-pt.yml up  {% endhighlight %}
  </div>
  <div class="tab-content" id="tab3-2">
  {% highlight console %}
# First copy all content to the RaspberryPi4
scp -r ./ <user>@<picarx-ip>:~/

# Login to the RaspberryPi via ssh
ssh <user>@<picarx-ip>

# Build and execute the Docker Containers
TAG=latest ARCH=arm64v8 docker compose -f docker-compose-core.yml build 
TAG=latest ARCH=arm64v8 docker compose -f docker-compose-pt.yml build 
TAG=latest ARCH=arm64v8 docker compose -f docker-compose-pt.yml up  {% endhighlight %}  
  </div>
</div>

# Execute Commands on the Physical Twin
The digital shadow can only receive data. To let the PiCar-X move and monitor the behavior in the GAZEBO simulation, you have to execute on the PiCar-X:

```console
# Connect to the RaspberryPi
ssh <user>@<picarx-ip>

# SWITCH INTO THE CONTAINER
docker exec -it picar-x-ackermann_skill-pt-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

# PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"
```

The model in the simulation should turn right and drive in circles until you send a different command.