🦅 F-16 Flight Control System (FCS) & Simulink-FlightGear Integration
This project implements a non-linear 6-DOF (Six Degrees of Freedom) simulation of an F-16 Fighting Falcon fighter jet, interfacing a mathematical model in MATLAB/Simulink with the FlightGear graphics engine for real-time visualization via hardware input (Joystick/HOTAS).

🚀 Project Overview

The F-16 is renowned for being the first fighter aircraft designed with Relaxed Static Stability. Its center of gravity is artificially shifted aft to maximize combat agility. This makes the aircraft (the "Plant" or "Bare Model") inherently and physically un-flyable by a human without the assistance of an onboard computer.

The goal of this project is to design, test, and validate the Flight Control Systems (FCS) required to stabilize the aircraft and enable piloting, addressing the highly coupled nature of its dynamics through a MIMO (Multiple-Input Multiple-Output) Analysis.

📐 F-16 MIMO Analysis

The F-16 cannot be treated as a collection of isolated systems. Every input influences multiple outputs (e.g., roll induces adverse yaw). The system is defined by:

Input Vector (Commands):

Thrust (T): Engine thrust (from 1000 to 60000 lbf).

Elevator (δe): Elevator for pitch control (max ≈ ±25° or 0.4 rad).

Aileron (δa): Ailerons for roll control.

Rudder (δr): Rudder for yaw control.

Output/State Vector (Sensors):

Velocities (V, U, W)

Angular velocities (p, q, r)

Euler Angles (ϕ, θ, ψ)

Angle of attack and sideslip angle (α, β)

Altitude (h)

🧠 Control Strategies (FCS)

To tame the instability, the project divides the control into two main channels, utilizing three different control architectures for research and comparison purposes:

1. Longitudinal Control (Pitch)
The longitudinal channel is the most critical due to the positive real part pole (instability) of the open-loop model. The goal is to track the angle of attack (α) or the pitch rate (q), preventing a deep stall.

2. Lateral-Directional Control (Roll & Yaw)
It combines aileron and rudder control to coordinate turns, minimizing the sideslip angle (β) and managing kinematic coupling (Dutch Roll).

Implemented Algorithms:

LQR (Linear Quadratic Regulator): An optimal state feedback controller. It perfectly manages the MIMO nature of the F-16, elegantly balancing aircraft responsiveness with actuator energy consumption by calculating a gain matrix K on a linearized model around a trim point (e.g., Mach 0.8 at 5000 ft).

MPC (Model Predictive Control): The state of the art. It looks at a future time horizon to optimize the trajectory and, above all, natively manages physical constraints. It mathematically prevents commands from exceeding the maximum deflection of the control surfaces (e.g., saturates at 0.4 rad) or the maximum engine thrust, preventing numerical crashes and aerodynamic stalls caused by abrupt pilot inputs.

🔌 Hardware / Software Interface Setup

The real-time simulation (simulated "Hardware-In-The-Loop") requires the correct routing of signals between the Joystick, Simulink, and FlightGear.

Signal Multiplexing (Crucial)
The Simulink model (the flight block) expects the input signals to be strictly vectorized in this order before entering the Plant:

Thrust

Elevator (in radians)

Aileron (in radians)

Rudder (in radians)

Connection with FlightGear
To ensure that FlightGear acts purely as a visual "screen" without interfering with the sophisticated physics calculated by Simulink, its internal aerodynamic engine (FDM) must be disabled.

Startup string for FlightGear:
Insert into the Additional Options (Plaintext):

🛠️ How to Start the Simulation

Initialization: Run the main MATLAB script (e.g., init.m or setup.m) to load the constants, geometry, aerodynamic coefficients, and calculate the Trim point (initial altitude and velocity in feet/second).

Start FlightGear: Launch FlightGear with the network options configured as above. Wait for the scenario to load.

Hardware Check: Ensure the HOTAS is connected and centered (an asymmetrical input at time T=0 on an unstable aircraft causes instantaneous numerical errors).

Start Simulink: Press PLAY on Simulink. The system will begin calculating the state equations and transmitting the coordinates to FlightGear.
