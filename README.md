# Hydraulic and Pneumatic System Simulation

This repository contains the implementation and simulation of a hydraulic and pneumatic system, as described in the project for the course "Hydraulic and Pneumatic Systems" at the Faculty of Electrical Engineering. The project focuses on modeling a mechanical system controlled by a hydraulic cylinder using a state-space representation and a PDI (Proportional-Derivative-Integral) controller.

## Project Overview

### System Description
The mechanical system is driven by a hydraulic cylinder, which is controlled via a single-stage servo valve. The control signal is the voltage applied to the torque motor, and the system follows a trapezoidal velocity profile for movement. The project also includes the selection of appropriate hydraulic components based on design requirements such as maximum force, stroke length, and cylinder speed.

### Key Features
- **State-Space Modeling:** The system is modeled in the state-space form, allowing for an efficient representation and control.
- **Trapezoidal Velocity Profile:** The system moves according to a trapezoidal velocity profile, with parameters such as total distance (L), total time (T), and acceleration/deceleration time (0.25T).
- **PDI Controller:** A Proportional-Derivative-Integral (PDI) control algorithm is implemented to regulate the system.
- **Component Selection:** Based on design requirements, hydraulic components such as the cylinder dimensions, pump displacement, and power are selected.

### Hydraulic System Design
- Cylinder dimensions: **D = 70mm**, **d = 36mm**
- Maximum force: **Fmax = 53555 N**
- Maximum speed: **vmax = 0.2142 m/s**
- Stroke length: **h = 0.4m**
- Pump displacement: **V = 30 cm³**

### Control Design
The PDI controller was designed using the Ziegler-Nichols tuning method:
- **P:** 84
- **I:** 4800
- **D:** 0.3675

### Simulation Results
The system's behavior was simulated using MATLAB Simulink. Time-domain responses such as position, velocity, acceleration, and differential pressure were plotted and analyzed. The system achieved the desired performance with minimal overshoot and adequate stability.

### Code Overview
The repository includes:
- MATLAB code for hydraulic system simulation and control design
- Simulink models for system simulation
- Calculation scripts for component selection
