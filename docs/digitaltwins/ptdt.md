---
title: Conncet Physical and Digital Twin
has_children: true
nav_order: 1
parent: Explore
---

<link rel="stylesheet" href="{{ site.baseurl }}{% link assets/css/tabs.css %}">
<script src="{{ site.baseurl }}{% link assets/js/tabs.js %}"> </script>

# Connecting the Physical Twin with a Digital Twin
This example can be used to connect the PiCar-X's phyiscal twin with a digital twin. The communication between both is established via MQTT. The MQTT server is part of the digital twin.

## Start the Digital Twin
We assume at this point that the digital twin runs on a X64 Linux system (e.g. a server).

```console
    # Build and execute the Docker Containers
    docker compose -f docker-compose-core.yml build 
    docker compose -f docker-compose-dtsim.yml build 
    docker compose -f docker-compose-dtsim.yml up
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

## Send Commands from the Digital Twin to the Physical Twin
```console
    # SWITCH INTO THE CONTAINER
    docker exec -it picar-x-ackermann_skill-dt-1 /bin/bash

    # INSIDE CONTAINER
    source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

    # PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
    rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"
```