---
title: The Digital Twin
has_children: false
nav_order: 7
parent: Digital Twin Concept
---

**This section is based on the publications:**
>[1] Barbie, A., & Hasselbring, W. (2024). From Digital Twins to Digital Twin Prototypes: Concepts, Formalization, and Applications. IEEE Access. [https://doi.org/10.1109/access.2024.3406510](https://doi.org/10.1109/access.2024.3406510)

>[2] Barbie, A., & Hasselbring, W. (2024). Toward Reproducibility of Digital Twin Research: Exemplified with the PiCar-X. arXiv preprint arXiv:2408.13866. [https://doi.org/10.48550/ARXIV.2408.13866](https://doi.org/10.48550/ARXIV.2408.13866)

# Context


# Definition

A **Digital Twin Prototype (DTP)** is the software prototype of a physical twin. The configurations are equal, yet the connected sensors/actuators are emulated. To simulate the behavior of the physical twin, the emulators use existing recordings of sensors and actuators. For continuous integration testing, the DTP can be connected to its corresponding digital twin, without the availability of the physical twin.

## Key Points:
- Replaces hardware components with emulators or simulations.
- Useful for continuous integration (CI) pipelines and automated testing.
- The **digital twin prototype** of the PiCar-X emulates the car's motors and sensors, enabling testing in software development environments without requiring the actual car.
- The PiCar-X DTP uses a GAZEBO simulation for virtual context.

---

# Digital Digital Prototype of the PiCar-X


