# Model Identification Algorithms Guide

This module provides a comprehensive suite of system identification algorithms for modeling, analyzing, and controlling mechanical and electromechanical systems.

## Selecting the Right Algorithm

When selecting a system identification algorithm, consider the type of system you are modeling, the available data, the required accuracy, and the expected computational resources.

### 1. Step Response Analysis (`StepResponseIdentifier`)
* **Best For:** Quick, initial characterizations of simple linear systems (e.g., thermal systems, simple motors).
* **Advantages:** Extremely simple, requires minimal computation, provides intuitive metrics (rise time, settling time, overshoot).
* **Disadvantages:** Highly sensitive to noise. Cannot identify complex dynamics, resonances, or nonlinearities.

### 2. Frequency Response Analysis (`FrequencyIdentification`)
* **Best For:** Discovering structural resonances, designing classical filters, and checking the system bandwidth.
* **Advantages:** Excellent for identifying resonant frequencies and assessing robustness via Bode plots. 
* **Disadvantages:** Typically requires long experiments (e.g., Chirp or PRBS) to get clean spectral resolution. Phase unwrapping and noise can complicate interpretation.

### 3. Least Squares (`RecursiveLeastSquares`, `BatchLeastSquares`)
* **Best For:** Estimating parameters of known linear models (ARX) where you want a fast, direct solution.
* **Advantages:** Convex optimization problem (no local minima), mathematically simple, fast to compute. RLS is ideal for online parameter tracking.
* **Disadvantages:** Biased estimates if the noise is not white (colored noise).

### 4. Polynomial Models (`PolynomialIdentifier` - ARX, ARMAX, OE, Box-Jenkins)
* **Best For:** Systems where disturbance/noise modeling is as important as the system dynamics.
* **Advantages:** 
  - **ARX:** Simplest, solvable via Least Squares.
  - **ARMAX:** Better handling of colored noise.
  - **OE (Output Error):** Focuses solely on system dynamics, good for simulation.
  - **Box-Jenkins:** Complete separation of system and noise models.
* **Disadvantages:** Highly sensitive to the chosen model order. Can suffer from local minima during optimization (except ARX).

### 5. Subspace Identification (`SubspaceIdentifier` - N4SID, MOESP, CVA)
* **Best For:** Multi-Input Multi-Output (MIMO) systems and high-order state-space models.
* **Advantages:** Numerically robust (relies on SVD/QR decompositions), estimates state-space matrices directly without iterative optimization. Do not suffer from local minima.
* **Disadvantages:** Computationally heavy for large datasets. Less intuitive to tune the hyper-parameters (block rows).
  - **N4SID:** Good general-purpose algorithm.
  - **MOESP:** Better for systems with deterministic inputs.
  - **CVA:** Maximizes correlation between past and future, theoretically optimal for stochastic systems.

### 6. Rigid Body Identification (`RigidBodyIdentifier`)
* **Best For:** Identifying fundamental mechanical parameters: Inertia, Viscous Friction, Coulomb Friction.
* **Advantages:** Provides physically meaningful parameters directly usable in feedforward controllers.
* **Disadvantages:** Assumes the system is perfectly rigid. Will fail or give biased results if there is significant flexibility or backlash.

### 7. Friction Identification (`AdvancedFrictionModels`)
* **Best For:** High-precision motion control where static friction, Stribeck effect, and pre-sliding behavior degrade performance.
* **Advantages:**
  - **LuGre / Dahl:** Captures dynamic pre-sliding displacement and hysteresis.
  - **Bouc-Wen:** Excellent for generic hysteresis.
* **Disadvantages:** Highly non-linear, parameters are difficult to identify accurately without specialized test signals (e.g., slow oscillating trajectories).

### 8. Adaptive Observers (`ExtendedKalmanFilter`, `UnscentedKalmanFilter`)
* **Best For:** Online estimation of unmeasurable states and time-varying parameters.
* **Advantages:** 
  - **EKF:** Standard, computationally efficient for mildly non-linear systems.
  - **UKF:** Better accuracy for highly non-linear systems without needing Jacobians.
* **Disadvantages:** Computationally demanding to run in real-time. Sensitive to process and measurement noise covariance tuning.

## Typical Identification Workflow

1. **Test Signal Generation:** Use a Pseudo-Random Binary Sequence (PRBS) or a Chirp signal to excite all relevant dynamics.
2. **Pre-processing:** Remove means, detrend data, and apply anti-aliasing filters.
3. **Non-parametric Identification:** Use ETFE or Step Response to get a rough idea of the system complexity and delays.
4. **Parametric Identification:** If a simple model is sufficient, use ARX or Least Squares. For complex/MIMO systems, use Subspace methods. 
5. **Physical Parameter Extraction:** If physical parameters are needed for feedforward (Inertia, Friction), use the Rigid Body Identifier or Advanced Friction Models with targeted trajectories (e.g., constant velocity runs for friction mapping).
6. **Validation:** Simulate the identified model with a *fresh* validation dataset and compare the simulated output against the measured output.