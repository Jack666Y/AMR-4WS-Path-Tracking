# AMR-4WS-Path-Tracking

This repository implements closed-loop path tracking for a 4-wheel-steering (4WS) mobile robot.
A Pure Pursuit controller is used as the outer-loop path follower, combined with a simplified
vehicle model, wheel-speed PI control, and a slip-speed tire model.

System performance is evaluated through:
- Monte Carlo simulations under random initial conditions (stability evidence)
- Closed-loop looped-path tracking under external disturbances (robustness evidence)

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


---

## Notes

- Random seeds are fixed (`rng(11)`) to ensure reproducible results.
- Controller and vehicle parameters are selected to demonstrate stable closed-loop behavior
  under random initial conditions and external disturbances.

---

## Environment

- Tested on **MATLAB R2025a**
- Expected to be compatible with MATLAB **R2021a or later**
