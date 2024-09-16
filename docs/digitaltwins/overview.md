---
title: Explore
has_children: true
nav_order: 4
---

# Digital Twin Concepts
The ARCHES PiCar-X was specifically developed to help scientists and engineers understand and experiment with the concept of Digital Twins. In our paper "[From Digital Twins to Digital Twin Prototypes: Concepts, Formalization, and Applications](https://doi.org/10.1109/access.2024.3406510)", we formalized the Digital Twin concept and its sub-concepts physical twin, digital model, digital template, digital thread, digital shadow, digital twin, and digital twin prototype using Object-Z. In "[Toward Reproducibility of Digital Twin Research: Exemplified with the PiCar-X.](https://doi.org/10.48550/ARXIV.2408.13866)", we then demonstrated how these concepts were implemented using the ARCHES PiCar-X. This project contains the source code for that implementation.

## Table of Contents
1. [Physical Twin](#1-physical-twin)
2. [Digital Model](#2-digital-model)
3. [Digital Template](#3-digital-template)
4. [Digital Thread](#4-digital-thread)
5. [Digital Shadow](#5-digital-shadow)
6. [Digital Twin](#6-digital-twin)
7. [Digital Twin Prototype](#7-digital-twin-prototype)
8. [Explore](#explore)

## 1. Physical Twin

### Definition
A **physical twin** is a real-world physical System-of-Systems or product. It comprises sensing or actuation capabilities driven by embedded software.

### Key Points:
- A real object, such as a vehicle or machine.
- Equipped with sensors and actuators for interaction with its environment.
- In the context of the PiCar-X, the **physical twin** is the PiCar-X itself, with all its hardware components, including motors, sensors, and embedded software.

---

## 2. Digital Model

### Definition
A **digital model** describes an object, a process, or a complex aggregation. The description is either a mathematical or a computeraided design (CAD).

### Key Points:
- Provides a virtual, often simplified, version of the physical twin.
- Can be used for simulations and testing without interacting with the actual hardware.
- For the PiCar-X, a CAD model is used in a GAZEBO simulation, representing the car's dimensions and driving behavior.

---

## 3. Digital Template

### Definition
A **digital template** serves as a framework that can be tailored or populated with specific information to generate the physical twin. It encompasses the software operating the physical twin, its digital model,
and all the essential information needed for constructing and sustaining the physical twin, such as blueprints, bills of materials, technical manuals, and similar documentation.

### Key Points:
- Combines the physical twin’s software, digital models, blueprints, and documentation.
- Provides the structure and data needed to recreate or understand the physical twin.
- In the case of the PiCar-X, the **digital template** includes software, a CAD model, and documentation on materials and schematics.

---

## 4. Digital Thread

### Definition
The **digital thread** refers to the communication framework that allows a connected data flow and integrated view of the physical twin’s data and operations throughout its lifecycle.

### Key Points:
- Ensures continuous data flow between physical and digital components, but is not limited to this communication.
- Enables (near) real-time monitoring of the physical twin's status on the digital twin.
- In the PiCar-X example, the **digital thread** uses ROS (Robot Operating System) and the MQTT protocol to connect the physical PiCar-X to its digital model and synchronize data.

---

## 5. Digital Shadow

### Definition
A **digital shadow** is the sum of all the data that are gathered by an embedded system from sensing, processing, or actuating. The connection from a physical twin to its digital shadow is automated. Changes on the physical twin are reflected to the digital shadow automatically. Vice versa, the digital shadow does not change the state of the physical twinö

### Key Points:
- Automatically updates based on the physical twin’s data.
- One-way communication: physical twin data is reflected in the digital shadow.
- In the PiCar-X setup, the **digital shadow** monitors the car's data, such as motor speed and steering angles, but does not alter its behavior.

---

## 6. Digital Twin

### Definition
A **digital twin** is a digital model of a real entity, the physical twin. It is both a digital shadow reflecting the status/operation of its physical twin, and a digital thread, recording the evolution of the physical twin over time. The digital twin is connected to the physical twin over the entire life cycle for automated bidirectional data exchange, i.e. changes made to the digital twin lead to adapted behavior of the physical twin and vice-versa.

### Key Points:
- Supports real-time, two-way interaction between the digital and physical twins.
- Changes in the digital model can impact the physical twin, and changes in the physical twin are reflected in the digital twin.
- For the PiCar-X, commands issued to the **digital twin** are simultaneously sent to the physical car, influencing its behavior.

---

## 7. Digital Twin Prototype

### Definition
A **Digital Twin Prototype (DTP)** is the software prototype of a physical twin. The configurations are equal, yet the connected sensors/actuators are emulated. To simulate the behavior of the physical twin, the emulators use existing recordings of sensors and actuators. For continuous integration testing, the DTP can be connected to its corresponding digital twin, without the availability of the physical twin.

### Key Points:
- Replaces hardware components with emulators or simulations.
- Useful for continuous integration (CI) pipelines and automated testing.
- The **digital twin prototype** of the PiCar-X emulates the car's motors and sensors, enabling testing in software development environments without requiring the actual car.
- The PiCar-X DTP uses a GAZEBO simulation for virtual context.

# Explore

* [Example for connecting physical twin and digital twin]({{ site.baseurl }}{% link digitaltwins/ptdt.md %})
* [Example for connceting physical twin and digital shadow]({{ site.baseurl }}{% link digitaltwins/ptds.md %})
* [Example for replacing the physical twin with a digital twin prototype]({{ site.baseurl }}{% link digitaltwins/dtp.md %})