# AMR-4WS-Path-Tracking

## Overview

This project implements a complete path planning and tracking pipeline for a
four-wheel-steering (4WS) autonomous mobile robot. The main components are:

- **Path Planning**
  - Sampling-based planning using a Rapidly-exploring Random Tree (RRT)
  - Obstacle inflation to provide safety margins during planning

- **Path Representation and Reference Generation**
  - Path resampling based on arc length for smooth tracking
  - Forward-looking reference point selection for Pure Pursuit control
  - Dedicated utilities for both open paths and closed-loop tracks

- **Closed-loop Path Tracking**
  - Pure Pursuit controller as the outer-loop path-following strategy
  - 4WS kinematic allocation to generate wheel steering angles and speed commands

- **Stability and Robustness Evaluation**
  - Monte Carlo simulations under random initial conditions (Test A)
  - Closed-loop looped-path tracking with external disturbances (Test B)

This structure allows the planning, control, and evaluation components to be
clearly separated while maintaining a fully integrated simulation framework.

---

## How to run

Open MATLAB and set the **Current Folder** to the repository root directory, then run:

1. **Main integrated demo (planning + tracking)**
   - `main_project_demo.m`

2. **Test A: Monte Carlo stability analysis**
   - `tests/testA_montecarlo_stability.m`

3. **Test B: Closed-loop looped-path tracking with disturbances**
   - `tests/testB_loop_disturbance.m`

All required paths are added automatically inside each script.

---

## Repository structure

```text
AMR-4WS-Path-Tracking/
├── main_project_demo.m
├── tests/
│   ├── testA_montecarlo_stability.m
│   └── testB_loop_disturbance.m
├── src/
│   ├── rrt/        % RRT path planning utilities
│   ├── path/       % path generation, reference lookup, error computation
│   ├── loop/       % closed-loop looped-path utilities (arc-length based)
│   └── utils/      % helper functions
└── README.md
```

---

## Notes

- Random seeds are fixed (`rng(11)`) to ensure reproducible results.
- Controller and vehicle parameters are selected to demonstrate stable closed-loop behavior
  under random initial conditions and external disturbances.

---

## Environment

- Tested on **MATLAB R2025a**
- Expected to be compatible with MATLAB **R2021a or later**
