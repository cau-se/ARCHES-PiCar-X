---
title: Replace the Physical Twin with a Digital Twin Prototype
has_children: true
nav_order: 1
parent: Explore
---

<link rel="stylesheet" href="{{ site.baseurl }}{% link assets/css/tabs.css %}">
<script src="{{ site.baseurl }}{% link assets/js/tabs.js %}"> </script>

# Replace the Physical Twin with a Digital Twin Prototype and Connect it to a Digital Twin
The digital twin prototype can be used to replace the physical twin during development, including its sensors and actuators. The data from these components are provided through a virtual context. This shift from hardware-in-the-loop to software-in-the-loop development enables faster system development and allows for greater collaboration among developers, as it eliminates the need for each person to have their own PiCar-X and avoids the need to take turns using a physical device.

We have three options at this point:

* [The digital twin prototype includes the Gazebo simulation, while the digital twin is launched without a simulation, displaying all data in the console or a log.]({{ site.baseurl }}{% link explore/dtps/dtnosim.md %})
* [The digital twin prototype is launched without a simulation, while the digital twin uses a Gazebo simulation to display the model's behavior based on the incoming data.]({{ site.baseurl }}{% link explore/dtps/dtpnosim.md %})
* [The digital twin prototype and the digital twin each start separate Gazebo simulations.]({{ site.baseurl }}{% link explore/dtps/dtpdt.md %})