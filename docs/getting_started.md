---
title: Getting Started
has_children: true
nav_order: 2
---

<link rel="stylesheet" href="{{ site.baseurl }}{% link assets/css/tabs.css %}">
<script src="{{ site.baseurl }}{% link assets/js/tabs.js %}"> </script>

# Getting Started
This project based on ROS and Docker. Due to the used interfaces on the RPi, we have to use Linux Kernel functions for GPIO and I2C. Before you can start this project you have to activate GPIO and I2C. If you already activated these modules, you can proceed with ##. Otherwise you have to build these modules first.

| The PiCar-X by Sunfounder                                            | The digital twin prototype in GAZEBO                                                               |
| -------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| ![Physical Twin](./images/picarx-pt.jpg "The PiCar-x by Sunfounder") | ![Digital Twin Prototype](./images/picarx-gazebo.gif "The digital twin prototype of the PiCar-X") |


## Clone Repository
```console 
    git clone https://github.com/cau-se/ARCHES-PiCar-X.git
    cd ./ARCHES-Picar-X/PiCar-X
```

## Check if required modules are installed
```console
modinfo i2c-stub
modinfo i2c-dev
modinfo gpio-mockup
```

**If one of the modules is missing follow the installation guides**

- [Install on Ubuntu 20.04]({{ site.baseurl }}{% link installation/installation_linux.md %})
- [Install on Windows (with WSL2)]({{ site.baseurl }}{% link installation/installation_windows.md %})

## Activate GPIO and I2C on Your System
If you already have built the modules, you need to activate them via:

<div class="tab-container" id="activateinterfaces">
  <ul class="tab-list">
    <li class="tab active" data-tab="tab1-1">X64</li>
    <li class="tab" data-tab="tab1-2">RaspberryPi 3/4</li>
  </ul>
  <div class="tab-content active" id="tab1-1">
  {% highlight console %}
    sudo modprobe gpio-mockup gpio_mockup_ranges=1,41
    sudo modprobe i2c-dev
    sudo modprobe i2c-stub chip_addr=0x14 {% endhighlight %}
  </div>
  <div class="tab-content" id="tab1-2">
  {% highlight console %}
    sudo modprobe i2c-stub chip_addr=0x14 {% endhighlight %}  
  </div>

</div>

## Build and Start Docker Containers from Scratch

<div class="tab-container" id="startdocker">
  <ul class="tab-list">
    <li class="tab active" data-tab="tab2-1">X64</li>
    <li class="tab" data-tab="tab2-2">arm32v7 (RPi3)</li>
    <li class="tab" data-tab="tab2-3">arm64v8 (RPi4)</li>
  </ul>
  <div class="tab-content active" id="tab2-1">
  {% highlight console %}
    docker compose -f docker-compose-core.yml build 
    docker compose -f docker-compose-dtp.yml build 
    docker compose -f docker-compose-dtp.yml up {% endhighlight %} 
  </div>
  <div class="tab-content" id="tab2-2">
  {% highlight console %}
    # First copy all content to the RaspberryPi3
    scp -r ./ <user>@<picarx-ip>:~/

    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-core.yml build 
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-dtp-no-gazebo.yml build 
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-dtp-no-gazebo.yml up  {% endhighlight %} 
  </div>
  <div class="tab-content" id="tab2-3">
  {% highlight console %}
    # First copy all content to the RaspberryPi4
    scp -r ./ <user>@<picarx-ip>:~/

    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-core.yml build 
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-dtp-no-gazebo.yml build 
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-dtp-no-gazebo.yml up   {% endhighlight %} 
  </div>

</div>


## Build and Start the Physical Twin on a RPi 3/4

<div class="tab-container" id="activaterpi">
  <ul class="tab-list">
    <li class="tab active" data-tab="tab3-1">arm32v7 (RPi3)</li>
    <li class="tab" data-tab="tab3-2">arm64v8 (RPi4)</li>
  </ul>
  <div class="tab-content active" id="tab3-1">
  {% highlight console %}
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-core.yml build 
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-pt.yml build 
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-pt.yml up  {% endhighlight %}
  </div>
  <div class="tab-content" id="tab3-2">
  {% highlight console %}
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-core.yml build 
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-pt.yml build 
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-pt.yml up  {% endhighlight %}  
  </div>

</div>


## Let the DTP Drive

After you start all Docker containers, you can switch into one of the containers and publish a picarx_msgs/Drive message, which will move the DTP with a certain speed and steering angle.


<div class="tab-container" id="activaterpi">
  <ul class="tab-list">
    <li class="tab active" data-tab="tab4-1">Start the Digital Twin Prototype</li>
    <li class="tab" data-tab="tab4-2">Start the Physical Twin</li>
  </ul>
  <div class="tab-content" id="tab4-1">
  {% highlight console %}
    # SWITCH INTO THE CONTAINER
    docker exec -it picar-x-ackermann_skill-dtp-1 /bin/bash

    # INSIDE CONTAINER
    source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

    # PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
    rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

    # PUBLISH A MESSAGE TO TURN LEFT WITH 80 percent motor speed
    rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 80, angle: -20}"

    # PUBLISH A MESSAGE TO TURN THE WHEELS LEFT BUT DRIVE BACKWARD WITH 80 percent motor speed
    rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: -80, angle: -20}"  {% endhighlight %}  
  </div>
  <div class="tab-content active" id="tab4-2">
  {% highlight console %}
    # SWITCH INTO THE CONTAINER
    docker exec -it picar-x-ackermann_skill-pt-1 /bin/bash

    # INSIDE CONTAINER
    source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

    # PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
    rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

    # PUBLISH A MESSAGE TO TURN LEFT WITH 80 percent motor speed
    rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 80, angle: -20}"

    # PUBLISH A MESSAGE TO TURN THE WHEELS LEFT BUT DRIVE BACKWARD WITH 80 percent motor speed
    rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: -80, angle: -20}"  {% endhighlight %}
  </div>

</div>