---
title: The Digital Thread
has_children: false
nav_order: 4
parent: Digital Twin Concept
---

**This section is based on the publications:**
>[1] Barbie, A., & Hasselbring, W. (2024). From Digital Twins to Digital Twin Prototypes: Concepts, Formalization, and Applications. IEEE Access. [https://doi.org/10.1109/access.2024.3406510](https://doi.org/10.1109/access.2024.3406510)

>[2] Barbie, A., & Hasselbring, W. (2024). Toward Reproducibility of Digital Twin Research: Exemplified with the PiCar-X. arXiv preprint arXiv:2408.13866. [https://doi.org/10.48550/ARXIV.2408.13866](https://doi.org/10.48550/ARXIV.2408.13866)

# Context
The concept of the **digital thread** remains rather unspecified, leaving much room for interpretation. In our formalization in [1], we have attempted to represent it mathematically using processes and threads responsible for data processing and communication between the physical twin and the digital model, shadow, or twin. For example, we developed the [ARCHES Digital Twin Framework]({{ site.baseurl }}{% link explore/adtf.md %}). as a support tool to uniformly map the digital thread between underwater ocean observation systems and their digital twins on a research vessel. We then applied this concept to the PiCar-X example.

# Definition

The **digital thread** refers to the communication framework that allows a connected data flow and integrated view of the physical twin’s data and operations throughout its lifecycle.

## Key Points:
- Ensures continuous data flow between physical and digital components, but is not limited to this communication.
- Enables (near) real-time monitoring of the physical twin's status on the digital twin.
- In the PiCar-X example, the **digital thread** uses ROS (Robot Operating System) and the MQTT protocol to connect the physical PiCar-X to its digital model and synchronize data.

---

# The Digital Thread for the PiCar-X

The digital thread for the PiCar-X is realized using the functions from the [ARCHES Digital Twin Framework]({{ site.baseurl }}{% link explore/adtf.md %}). The digital model already contains parts of the digital thread, in form of the GAZEBO simulation connected to ROS software. Other tools that are connected to the digital model/shadow/twin to enhance monitoring capabilities, data analysis, or operation, would be also part of the digital thread. A good example would be a database that stores all messages, e.g., a [MongoDB](https://www.mongodb.com/) or [CrateDB](https://cratedb.com/). 

Continue reading with [digital shadow]({{ site.baseurl }}{% link digitaltwins/ds.md %})
