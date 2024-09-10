---
title: Replace the Physical Twin with a Digital Twin Prototype
has_children: true
nav_order: 3
parent: Explore
---

<link rel="stylesheet" href="{{ site.baseurl }}{% link assets/css/tabs.css %}">
<script src="{{ site.baseurl }}{% link assets/js/tabs.js %}"> </script>

# Replace the Physical Twin with a Digital Twin Prototype and Connect it to a Digital Twin
The digital twin prototype can be used to replace the physical twin during development, including its sensors and actuators. The data from these components are provided through a virtual context. This shift from hardware-in-the-loop to software-in-the-loop development enables faster system development and allows for greater collaboration among developers, as it eliminates the need for each person to have their own PiCar-X and avoids the need to take turns using a physical device.

We have three options at this point:

* [The digital twin prototype and the digital twin each start separate Gazebo simulations.](#option1)
* [The digital twin prototype includes the Gazebo simulation, while the digital twin is launched without a simulation, displaying all data in the console or a log.](#option2)
* [The digital twin prototype is launched without a simulation, while the digital twin uses a Gazebo simulation to display the model's behavior based on the incoming data.](#option3)

# Option 1: Start Digital Twin Prototype and Digital Twin with Separate Gazebo Silumation {#option1}
**You need to change the ROS Master Ports on one of the docker compose files, else they will conflict.**

## Start the Digital Twin with GAZEBO
```console
# Build and execute the Docker Containers
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dtsim.yml build 
docker compose -f docker-compose-dtsim.yml up
```

## Start the Digital Twin Prototype with GAZEBO
```console
# Build and execute the Docker Containers
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dtp.yml build 
docker compose -f docker-compose-dtp.yml up
```

## Send Commands from both directions
<div class="tab-container" id="dtpdtsims">
  <ul class="tab-list">
<li class="tab active" data-tab="tab1-1">Commands on the DTP</li>
<li class="tab" data-tab="tab1-2">Commands from the DT</li>
  </ul>
  <div class="tab-content active" id="tab1-1">
{% highlight console %}
# SWITCH INTO THE CONTAINER
docker exec -it picar-x-ackermann_skill-dtp-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

# PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The status should be sent automatically to the DT and the DT also start to move. {% endhighlight %}
  </div>
  <div class="tab-content" id="tab1-2">
{% highlight console %}
# SWITCH INTO THE CONTAINER
docker exec -it picar-x-ackermann_skill-dt-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

# PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The command should be sent automatically to the DTP and the DTP also start to move. {% endhighlight %}
  </div>
</div>


# Option 2: Only the Digital Twin Prototype starts a simulation {#option2}
## Start the Digital Twin without GAZEBO
```console
# Build and execute the Docker Containers
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dt.yml build 
docker compose -f docker-compose-dt.yml up
```

## Start the Digital Twin Prototype with GAZEBO
```console
# Build and execute the Docker Containers
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dtp.yml build 
docker compose -f docker-compose-dtp.yml up
```

## Print all Incoming Commands on the Digital Twin
<div class="tab-container" id="dtpsim">
  <ul class="tab-list">
<li class="tab active" data-tab="tab2-1">Subscribe to Status on DT</li>
<li class="tab" data-tab="tab2-2">Commands from the DTP</li>
  </ul>
  <div class="tab-content active" id="tab2-1">
{% highlight console %}
# SWITCH INTO THE CONTAINER
docker exec -it picar-x-ackermann_skill-dt-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

# SUBSCRIBE TO THE STATUS TOPIC
rostopic echo /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The console should now print all incoming messages with the corresponding status from the DTP. {% endhighlight %}
  </div>
  <div class="tab-content" id="tab2-2">
{% highlight console %}
# SWITCH INTO THE CONTAINER
docker exec -it picar-x-ackermann_skill-dtp-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

# PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The status should be sent automatically to the DT and the DT prints all incoming messages. {% endhighlight %}
  </div>
</div>

# Option 3: Only the Digital Twin starts a simulation {#option3}
## Start the Digital Twin with GAZEBO
```console
# Build and execute the Docker Containers
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dtsim.yml build 
docker compose -f docker-compose-dtsim.yml up
```

## Start the Digital Twin Prototype without GAZEBO
```console
# Build and execute the Docker Containers
docker compose -f docker-compose-core.yml build 
docker compose -f docker-compose-dtp-no-gazebo.yml build 
docker compose -f docker-compose-dtp-no-gazebo.yml up
```

## Print all Incoming Commands on the Digital Twin Prototype
<div class="tab-container" id="dtpsim">
  <ul class="tab-list">
<li class="tab active" data-tab="tab2-1">Subscribe to Commands on the DTP</li>
<li class="tab" data-tab="tab2-2">Commands from the DT</li>
  </ul>
  <div class="tab-content active" id="tab2-1">
{% highlight console %}
# SWITCH INTO THE CONTAINER
docker exec -it picar-x-ackermann_skill-dtp-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

# SUBSCRIBE TO THE COMMAND TOPIC
rostopic echo /picarx/drive/command

# The console should now print all incoming messages with the corresponding command from the DT. {% endhighlight %}
  </div>
  <div class="tab-content" id="tab2-2">
{% highlight console %}
# SWITCH INTO THE CONTAINER
docker exec -it picar-x-ackermann_skill-dt-1 /bin/bash

# INSIDE CONTAINER
source /root/catkin_ws/devel/picarx_ackermann_drive/setup.bash

# PUBLISH A MESSAGE TO TURN RIGHT WITH 50 percent motor speed
rostopic pub /picarx/drive/command picarx_msgs/Drive "{speed: 50, angle: 20}"

# The DT sends a command to the DTP and there the command is printed. {% endhighlight %}
  </div>
</div>