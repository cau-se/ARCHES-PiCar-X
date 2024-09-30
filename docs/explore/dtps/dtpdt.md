---
title: Option 3
has_children: false
nav_order: 3
parent: Replace the Physical Twin with a Digital Twin Prototype
---


# Start Digital Twin Prototype and Digital Twin with Separate Gazebo Silumation
In general it is always possible to start the digital twin prototype and digital twin on two different systems. Just make sure that the MQTT port 1883 is open on the digital twin side and that the MQTT clients have the IP of the MQTT server configured.
In the following, we assume that you want to start the digital twin prototype and the digital twin on the same system with separate GAZEBO simulations for both. We further assume, that you can only create one I2C-stub chip.

In the [Getting Started Guide]({{ site.baseurl }}{% link getting_started.md %}), we create the I2C-stub with only one active address space (0x14). For this example, we have to register an additional address space (0x15). If you have already explored one of the other options, restart your system and execute:

## Adjust the environemnt files

There are three environment files with relevant variables:
- picarx.env (for the DTP)
- picarx-dt.env
- simulation.env
- simulation-dt.env


```console
sudo modprobe gpio-mockup gpio_mockup_ranges=1,41
sudo modprobe i2c-dev
sudo modprobe i2c-stub chip_addr=[0x14,0x15]
```

With the `i2c-tools` installed, you can check whether both addresses are allocated now:

```console
i2cdetect -y 0      # the 0 represens /dev/i2c-0, insert the number of the stub here

# The result should be something like -- 0x14 0x15 --
```

Ofcourse the idea of the digital twin prototype is that you use the same configuration as the original system, however, since all containers share the kernel modules of the system this example is executed and in most cases you cannot create two I2C-stub devices, they need to share the I2C device. In examples with seriel or network protocols, this would be different.

The digital twin will use the address space `0x15` and two other GPIO pins.

## Start the Digital Twin with GAZEBO
```console
# Build Docker Containers if not previously done
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dtsim-same-system.yml build 

# Start the container

docker compose -f docker-compose-dtsim-same-system.yml up
```

## Start the Digital Twin Prototype with GAZEBO
```console
docker compose -f docker-compose-dtp.yml up
```

## Operate the Digital twin Prototype

To operate the digital twin prototype, you need to switch into one of the digital twin's container:

```console
docker exec -it picar-x-ackermann_skill-dt-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash
```

Inside the container you can publish a command message:


```console
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The status should be sent automatically to the DTP and the PiCar-X moves in GAZEBO
```
