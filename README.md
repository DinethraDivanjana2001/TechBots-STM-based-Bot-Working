# TechBots-Bot-Working

### Introduction

Our robot was designed to compete in the Sri Lankan Robotics Challenge 2024. The robot utilizes a Raspberry Pi 4B for color detection and alignment, while the main control logic is handled by an STM32 microcontroller. The project combines various hardware and software components to achieve precise control and functionality.

The repository contains the codes used for the microcontroller, the Raspberry Pi, schematics, PCB Design and 3D enclosure design.

Following are the tasks capable by the robot:
* Line detection and following
* Wall colour detection
* Floor colour detection
* Object detection between a cube and a cylinder
* Shooting a ball into a target

The robot is entirely autonomous.

### Codes

The code for the microcontroller is found under STM32 firmware and firmware. There are 2 versions: one done with an STM32 microcontroller and the other with an ESP32. This codes is responsible for getting readings from the sensors and moving the robot by controlling the motors, The Raspberry Pi firmware is found under Raspberry Pi firmware directory. This includes codes regarding the colour detection and aligining with the target.

### PCB design

The schematics and PCB design of the robot are found under the PCB design directory. Schematics are included for subcircuits. A 4-layer PCB was designed for the robot.

### 3D Models

The 3D models and 3D renders of the robot are found under the 3D designs directory. The assembly is also included.
