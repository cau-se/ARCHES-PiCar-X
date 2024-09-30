---
title: The PiCar-X's Digital Thread'
has_children: false
nav_order: 4
parent: Digital Twin Concept
---

**This section is based on the publications:**
>[1] Barbie, A., & Hasselbring, W. (2024). From Digital Twins to Digital Twin Prototypes: Concepts, Formalization, and Applications. IEEE Access. [https://doi.org/10.1109/access.2024.3406510](https://doi.org/10.1109/access.2024.3406510)

>[2] Barbie, A., & Hasselbring, W. (2024). Toward Reproducibility of Digital Twin Research: Exemplified with the PiCar-X. arXiv preprint arXiv:2408.13866. [https://doi.org/10.48550/ARXIV.2408.13866](https://doi.org/10.48550/ARXIV.2408.13866)

# Context


# Definition

A **digital template** serves as a framework that can be tailored or populated with specific information to generate the physical twin. It encompasses the software operating the physical twin, its digital model,
and all the essential information needed for constructing and sustaining the physical twin, such as blueprints, bills of materials, technical manuals, and similar documentation.

## Key Points:
- Ensures continuous data flow between physical and digital components, but is not limited to this communication.
- Enables (near) real-time monitoring of the physical twin's status on the digital twin.
- In the PiCar-X example, the **digital thread** uses ROS (Robot Operating System) and the MQTT protocol to connect the physical PiCar-X to its digital model and synchronize data.

---

# The Digital Thread for the PiCar-X
