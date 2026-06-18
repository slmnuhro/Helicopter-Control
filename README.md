# Helicopter Pitch Control

Final project for the course **Integration Project Systems and Control (SC42035)** at TU Delft.

This project covers the full workflow of modelling, identifying, and controlling the pitch angle of a one degree-of-freedom (1-DoF) helicopter setup. A grey-box model is identified from experimental data, a Kalman filter is designed to estimate the unmeasured state, and two state-feedback controllers are developed and compared. The goal is to make the helicopter hold a stationary pitch angle, reject disturbances, and track a pitch reference. Everything is implemented in MATLAB and Simulink.

The project explores:

- Separate positive- and negative-input models for the asymmetric dynamics
- Grey-box system identification (`idgrey` / `greyest`)
- Kalman filter state estimation
- Linear-Quadratic-Integral (LQI) control
- Model Predictive Control (MPC)

## Project Overview

The helicopter is described by three states: pitch angle, pitch angular velocity, and motor angular velocity. Only the pitch angle and motor angular velocity can be measured, so the pitch angular velocity is reconstructed with a state estimator. The work is split into four sequential stages, mirrored by the folder layout:

- **System Identification** — Step and chirp responses are collected, filtered, and used in a grey-box estimation to obtain linearised state-space models. Because the response differs for positive and negative pitch, two separate models (Positive Input and Negative Input) are identified and validated against a held-out dataset using NRMSE.

- **State Estimation** — A Kalman filter is designed from the identified models to estimate the unmeasured pitch angular velocity, with the gain computed offline and implemented in Simulink.

- **LQI Control** — A Linear-Quadratic Regulator with an added integrator removes steady-state error and rejects disturbances around a setpoint.

- **MPC** — A Model Predictive Controller handles input and output constraints, using the stabilizing Riccati solution as the terminal cost. Both an interior-point (`quadprog`) and an active-set (`MPCActiveSetSolver`) solver are used.

The repository contains the MATLAB scripts and Simulink models for each stage, together with the [final report](Helicopter_Control_Report.pdf) containing the modelling, identification, estimator design, controller design, and results.

## Repository Structure

```
Helicopter-Control/
│
├── Code/
│   ├── 01_Helicopter_SystemID/
│   │   ├── id_data/                      % identification datasets
│   │   ├── validation_data/              % validation datasets
│   │   ├── helicopter_sysID.m            % grey-box identification script
│   │   ├── helicopter_sysID_sim.slx
│   │   ├── helicopter_sysID_sim_21b.slx
│   │   └── sysID_output.mat              % identified models
│   │
│   ├── 02_Helicopter_StateEstimation/
│   │   ├── helicopter_StateEst_21b.slx
│   │   └── hwinit.m
│   │
│   ├── 03_Helicopter_LQI/
│   │   ├── helicopter_LQR_21b.slx
│   │   └── hwinit.m
│   │
│   └── 04_Helicopter_MPC/
│       ├── helicopter_MPC_21b.slx
│       └── hwinit.m
│
├── Helicopter_Control_Report.pdf
│
└── README.md
```


## Requirements

- MATLAB (R2021b or later)
- Simulink
- System Identification Toolbox (`idgrey`, `greyest`)
- Control System Toolbox (`lqi`, `kalman`, `dlqr`)
- Model Predictive Control Toolbox / Optimization Toolbox (`quadprog`, `MPCActiveSetSolver`)

## A Note on Scope

This repository does not contain code used to power the hardware, initialize data I/O connections, or run sensor calibrations.

The contents are the Simulink diagrams we built ourselves and the scripts that we either wrote in full or contributed to substantially.

---

## Collaborators

- Sven Rutgers
- Melis Orhun
