# Adaptive and Nonlinear Control - Politecnico di Milano

This repository contains two projects done for the "Adaptive and Nonlinear Control" course at Politecnico di Milano. The first one is a practical Simulink project focused on UAV control, and the second one is a short research presentation on autonomous path-planning algorithms.

### Project 1: Hexarotor UAV Adaptive Control
* **UAV Modeling:** We built a 6-DOF nonlinear dynamic model of a hexarotor in Simulink. We used SVD (Singular Value Decomposition) to properly allocate the commanded forces and moments to the six propellers.
* **Baseline Controller:** We designed a cascaded control architecture (Position -> Velocity -> Attitude) and mathematically proved its stability using Lyapunov functions.
* **Adaptive Control (PBMRAC):** To handle unknown changes in mass and CG (for example, when the drone carries an unpredictable delivery package), we upgraded the system with a Predictor-Based Model Reference Adaptive Controller. We also implemented a projector and *e-modification* to prevent the adaptive gains from drifting.
* **Robustness Testing:** We tested the UAV under tough conditions, such as strong crosswinds and a sudden 20% effectiveness loss in one of the propellers. The adaptive controller successfully managed these disturbances, maintaining a very low tracking error during the delivery route.

### Project 2: Autonomous Path Planning
* **Overview:** A review of the state of the art in path-planning algorithms for autonomous systems, specifically focusing on how they react to dynamic environments.
* **D* Lite:** We analyzed this algorithm, which is much more computationally efficient than traditional A*. Instead of recalculating the whole map when a new obstacle appears, it only updates the affected nodes.
* **Risk-Aware Navigation (D+*):** We explored how to make UAV navigation safer by assigning "Risk Costs" to unknown areas or paths that get too close to walls, keeping a safe buffer.
* **Neural Path Planning:** Finally, we looked into the integration of Deep Learning (like Neural A*/D*). Using Convolutional Neural Networks (CNNs) to guide the algorithm can reduce the amount of explored nodes by up to 90%, saving a lot of CPU power.
