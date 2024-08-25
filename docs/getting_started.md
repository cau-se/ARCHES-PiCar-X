---
title: Getting Started
has_children: true
nav_order: 2
---

<link rel="stylesheet" href="{{ site.baseurl }}{% link tabs.css %}">
<script src="{{ site.baseurl }}{% link tabs.js %}"> </script>


# Getting Started
This project based on ROS and Docker. Due to the used interfaces on the RPi, we have to use Linux Kernel functions for GPIO and I2C. Before you can start this project you have to activate GPIO and I2C. If you already activated these modules, you can proceed with ##. Otherwise you have to build these modules first.

## Activate GPIO and I2C on Your System

- [Install on Ubuntu 20.04](#build-new-linux-kernel)
- [Install on Windows (with WSL2)](#build-new-windows-wsl2-kernel)

**If you are using this project with WSL2, you also have to [install and start the XLaunch](#xlaunch-for-windows). Otherwise you can start Gazebo with Docker.**

If you already have built the modules, you can activate them via:
```console 
    sudo modprobe gpio-mockup gpio_mockup_ranges=1,41
    sudo modprobe i2c-dev
    sudo modprobe i2c-stub chip_addr=0x14
```

If you want to run the Digital Twin Prototype without the Gazebo Simulation on a RaspberryPi, you only need to active the I2C stub:
```console 
    sudo modprobe i2c-stub chip_addr=0x14
```

## Clone Repository
```console 
    git clone https://github.com/cau-se/ARCHES-PiCar-X.git
    cd ./ARCHES-Picar-X/PiCar-X
```

## Start Digital Twin Prototype
```console 
    docker compose -f docker-compose-dtp.yml up 
```

## Build and Start Docker Containers from Scratch

{% tabs docker %}

{% tab docker x64 %}
```console 
    docker compose -f docker-compose-core.yml build 
    docker compose -f docker-compose-dtp.yml build 
    docker compose -f docker-compose-dtp.yml up 
```
{% endtab %}

{% tab docker arm32v7 %}
```console 
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-core.yml build 
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-dtp-no-gazebo.yml build 
    TAG=latest ARCH=arm32v7 docker compose -f docker-compose-dtp-no-gazebo.yml up 
```
{% endtab %}

{% tab docker arm64v8 %}
```console 
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-core.yml build 
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-dtp-no-gazebo.yml build 
    TAG=latest ARCH=arm64v8 docker compose -f docker-compose-dtp-no-gazebo.yml up 
```
{% endtab %}

{% endtabs %}


## Build and start the Physical Twin on a RPI
```console 
    TAG=latest ARCH=<SELECT ARM VERSION> docker compose -f docker-compose-core.yml build
    TAG=latest ARCH=<SELECT ARM VERSION> docker compose -f docker-compose-pt.yml build
    TAG=latest ARCH=<SELECT ARM VERSION> docker compose -f docker-compose-pt.yml up -d
```
