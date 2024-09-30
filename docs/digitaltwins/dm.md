---
title: The PiCar-X's Digital Model
has_children: false
nav_order: 2
parent: Digital Twin Concept
---

**This section is based on the publications and we recommend to read both:**
>[1] Barbie, A., & Hasselbring, W. (2024). From Digital Twins to Digital Twin Prototypes: Concepts, Formalization, and Applications. IEEE Access. [https://doi.org/10.1109/access.2024.3406510](https://doi.org/10.1109/access.2024.3406510)

>[2] Barbie, A., & Hasselbring, W. (2024). Toward Reproducibility of Digital Twin Research: Exemplified with the PiCar-X. arXiv preprint arXiv:2408.13866. [https://doi.org/10.48550/ARXIV.2408.13866](https://doi.org/10.48550/ARXIV.2408.13866)

# Context

In our formalization [1], the digital model is represented by a software state machine model. This PiCar-X example demonstrates the approach with a CAD model in a [GAZEBO](https://gazebosim.org) simulation, an open-source tool integrated with ROS, as digital model.
We define a **digital model** as follows [1]: 


# Definition

A **digital model** describes an object, a process, or a complex aggregation. The description is either a mathematical or a computer-aided design (CAD).

## Key Points:
- Provides a virtual, often simplified, version of the physical twin.
- Can be used for simulations and testing without interacting with the actual hardware.
- For the PiCar-X, a CAD model is used in a GAZEBO simulation, representing the car's dimensions and driving behavior.

---

# Physical Twin of the PiCar-X

Lacking official CAD files for the PiCar-X, we utilized a [simplified CAD model of an older SunFounder PiCar version, available under Apache 2.0 license on GitHub](https://github.com/Theosakamg/PiCar_Hardware). This model, consisting of just the frame and wheels, closely mirrors the original PiCar-X's key dimensions like wheelbase and track, crucial for an accurate Ackermann steering simulation. However, the original PiCar-X's steering mechanism, operated by a steering bar to achieve Ackermann angles, could not be replicated in [GAZEBO](https://gazebosim.org). Instead, we approximate the Ackermann steering angles for both front wheels based on established methodologies [2].


![Digital Twin Prototype](./assets/images/picarx-gazebo.gif "The digital twin prototype of the PiCar-X")
