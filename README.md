# 🦅 F-16 Flight Control System (FCS) & Simulink-FlightGear Integration


---

## Project Overview

This project implements a non-linear **6-DOF (Six Degrees of Freedom)** simulation of an F-16 Fighting Falcon. It interfaces a highly accurate mathematical model in MATLAB/Simulink with the FlightGear graphics engine for real-time visualization via hardware input (Joystick/HOTAS).

The F-16 is renowned for being the first fighter aircraft designed with **Relaxed Static Stability**. Its center of gravity is artificially shifted aft to maximize combat agility, making the "Bare Model" inherently un-flyable by a human without computer assistance. The goal of this project is to design, test, and validate the Flight Control Systems (FCS) required to stabilize the aircraft and enable piloting.

---

## F-16 MIMO Analysis

The F-16 cannot be treated as a collection of isolated systems; every input influences multiple outputs (e.g., roll induces adverse yaw). The highly coupled nature of its dynamics requires a robust MIMO (Multiple-Input Multiple-Output) approach.

### System Definitions

| Input Vector (Commands) | Description |
| :--- | :--- |
| **Thrust (T)** | Engine thrust ranging from 1,000 to 60,000 lbf |
| **Elevator (δe)** | Pitch control (max deflection ≈ ±25° or 0.4 rad) |
| **Aileron (δa)** | Roll control |
| **Rudder (δr)** | Yaw control |

| Output/State Vector (Sensors) | Variables |
| :--- | :--- |
| **Velocities** | V, U, W |
| **Angular Velocities** | p, q, r |
| **Euler Angles** | ϕ, θ, ψ |
| **Aerodynamic Angles** | Angle of attack (α) and sideslip angle (β) |
| **Position** | Altitude (h) |

---

##Control Strategies (FCS)

To tame the instability, the control architecture is divided into two main channels. We utilize different modern control algorithms for research and performance comparison:

### 1. Longitudinal Control (Pitch)
The longitudinal channel is the most critical due to the positive real part pole (instability) of the open-loop model. The primary goal is to strictly track the angle of attack (α) or the pitch rate (q), preventing a deep stall scenario.

### 2. Lateral-Directional Control (Roll & Yaw)
This channel combines aileron and rudder control to coordinate turns. It focuses on minimizing the sideslip angle (β) and managing kinematic coupling effects like the Dutch Roll.

### Implemented Algorithms

* **LQR (Linear Quadratic Regulator):** An optimal state feedback controller that perfectly manages the MIMO nature of the F-16. It balances aircraft responsiveness with actuator energy consumption by calculating a gain matrix K on a linearized model around a specific trim point (e.g., Mach 0.8 at 50,000 ft).
* **MPC (Model Predictive Control):** The state-of-the-art approach. It evaluates a future time horizon to optimize the trajectory while natively managing physical constraints. It mathematically prevents commands from exceeding the maximum deflection of the control surfaces or the maximum engine thrust, actively preventing numerical crashes and aerodynamic stalls caused by abrupt pilot inputs.

---

## Hardware / Software Interface Setup

The real-time simulation (simulated "Hardware-In-The-Loop") requires precise signal routing between the HOTAS, Simulink, and FlightGear.

### Crucial Signal Multiplexing
The Simulink model expects input signals to be strictly vectorized in the following order before entering the Plant:
1. Thrust
2. Elevator (radians)
3. Aileron (radians)
4. Rudder (radians)

### FlightGear Network Configuration
To ensure FlightGear acts purely as a visual renderer without interfering with Simulink's sophisticated physics, its internal aerodynamic engine (FDM) must be disabled. Insert your network configuration string into the Additional Options (Plaintext) in the FlightGear launcher.

---


Press PLAY on Simulink. The system will begin calculating the state equations and transmitting coordinates over the network to FlightGear.
