---
title: Explore
has_children: true
nav_order: 4
---

# Digital Twin Concepts
The ARCHES PiCar-X was specifically developed to help scientists and engineers understand and experiment with the concept of Digital Twins. In our paper "[From Digital Twins to Digital Twin Prototypes: Concepts, Formalization, and Applications](https://doi.org/10.1109/access.2024.3406510)", we formalized the Digital Twin concept and its sub-concepts physical twin, digital model, digital template, digital thread, digital shadow, digital twin, and digital twin prototype using Object-Z. In "[Toward Reproducibility of Digital Twin Research: Exemplified with the PiCar-X.](https://doi.org/10.48550/ARXIV.2408.13866)", we then demonstrated how these concepts were implemented using the ARCHES PiCar-X. This project contains the source code for that implementation.

## Inhaltsverzeichnis
1. [Physical Twin](#1-physical-twin)
2. [Digital Model](#2-digital-model)
3. [Digital Template](#3-digital-template)
4. [Digital Thread](#4-digital-thread)
5. [Digital Shadow](#5-digital-shadow)
6. [Digital Twin](#6-digital-twin)
7. [Digital Twin Prototype](#7-digital-twin-prototype)
8. [Explore](#8-explore)

## 1. Physical Twin

### Definition
A **Physical Twin** is the real-world physical system or product that a digital twin is based on. It consists of the physical object itself, which is equipped with sensors or actuators and controlled by embedded software.

### Key Points:
- A real object, such as a vehicle or machine.
- Equipped with sensors and actuators for interaction with its environment.
- In the context of the PiCar-X, the **Physical Twin** is the PiCar-X itself, with all its hardware components, including motors, sensors, and embedded software.

---

## 2. Digital Model

### Definition
A **Digital Model** is a virtual representation of the physical object, process, or system. It can be a mathematical model or a computer-aided design (CAD) used to simulate the behavior of the physical twin.

### Key Points:
- Provides a virtual, often simplified, version of the physical twin.
- Can be used for simulations and testing without interacting with the actual hardware.
- For the PiCar-X, a **CAD model** is used in a GAZEBO simulation, representing the car's dimensions and driving behavior.

---

## 3. Digital Template

### Definition
A **Digital Template** serves as a framework that can be populated with specific data to generate a digital or physical twin. It includes the necessary software, models, and documentation for replicating or understanding the system.

### Key Points:
- Combines the physical twin’s software, digital models, blueprints, and documentation.
- Provides the structure and data needed to recreate or understand the physical twin.
- In the case of the PiCar-X, the **Digital Template** includes software, a CAD model, and documentation on materials and schematics.

---

## 4. Digital Thread

### Definition
A **Digital Thread** refers to the communication framework that enables the flow of data between the physical twin and its digital representations throughout the product's lifecycle.

### Key Points:
- Ensures continuous data flow between physical and digital components.
- Provides real-time data on the physical twin's status to the digital twin.
- In the PiCar-X example, the **Digital Thread** uses ROS (Robot Operating System) and the MQTT protocol to connect the physical PiCar-X to its digital model and synchronize data.

---

## 5. Digital Shadow

### Definition
A **Digital Shadow** is a passive digital representation that mirrors the data and state of the physical twin. It reflects any changes in the physical twin but does not directly control it.

### Key Points:
- Automatically updates based on the physical twin’s data.
- One-way communication: physical twin data is reflected in the digital shadow.
- In the PiCar-X setup, the **Digital Shadow** monitors the car's data, such as motor speed and steering angles, but does not alter its behavior.

---

## 6. Digital Twin

### Definition
A **Digital Twin** is a fully integrated virtual counterpart of the physical twin, with bidirectional communication. Any changes made to the digital twin will affect the physical twin and vice versa.

### Key Points:
- Supports real-time, two-way interaction between the digital and physical twins.
- Changes in the digital model can impact the physical twin, and changes in the physical twin are reflected in the digital twin.
- For the PiCar-X, commands issued to the **Digital Twin** are simultaneously sent to the physical car, influencing its behavior.

---

## 7. Digital Twin Prototype

### Definition
A **Digital Twin Prototype** is a software prototype of the physical twin, emulating its hardware interactions. It allows for testing and development without needing access to the actual physical twin.

### Key Points:
- Replaces hardware components with emulators or simulations.
- Useful for continuous integration (CI) pipelines and automated testing.
- The **Digital Twin Prototype** of the PiCar-X emulates the car's motors and sensors, enabling testing in software development environments without requiring the actual car.

# Explore

* [Example for connecting physical twin and digital twin]({{ site.baseurl }}{% link digitaltwins/ptdt.md %})
* [Example for connceting physical twin and digital shadow]({{ site.baseurl }}{% link digitaltwins/ptds.md %})
* [Example for replacing the physical twin with a digital twin prototype]({{ site.baseurl }}{% link digitaltwins/dtp.md %})