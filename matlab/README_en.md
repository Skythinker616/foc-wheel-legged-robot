# Model Building & Algorithm Simulation

**[简体中文](README.md) | English**

This project uses MATLAB(R2022b) for the design and simulation of balance algorithms. The main tasks include:

- Leg link posture calculation
- Establishment of the system state space equation
- LQR feedback matrix computation and fitting
- Simscape Multibody physical model construction
- Simulink control algorithm simulation

---

## Control Algorithm

The balance algorithm mainly refers to this [column article](https://zhuanlan.zhihu.com/p/563048952) with only a few differences, hence not elaborated here. It only provides a brief overview of the process, which includes the following steps:

> Note: The variable names in the code remain mostly consistent with those described in the article, and readers can refer to it for comparison.

1. Simplify the leg structure and obtain the mapping relationship between symbols before and after simplification, allowing the calculation of the leg posture using motor data.
2. Determine the mapping relationship between the simplified virtual leg force and the motor torque, enabling control of the simplified leg using the VMC (Virtual Model Control) concept.
3. Conduct classical mechanics analysis and symbolic simplification to obtain the system state space equation.
4. Set the weight coefficients for LQR, calculate the feedback matrix K, and fit the K values for different leg lengths.
5. Implement state feedback control.

---

## File Introduction

- `leg_func_calc.m`: Performs leg kinematics calculations and VMC mapping, which includes steps 1-2 mentioned above, and exports the following three M functions:
	- `leg_pos.m`: Obtains leg posture from joint motor angles.
	- `leg_spd.m`: Obtains leg motion speed from joint motor angles and speeds.
	- `leg_conv.m`: Calculates the torque required by motors from the target torques and thrusts of virtual legs.
- `sys_calc.m`: Computes the system state equation and feedback matrix, which includes steps 3-4 mentioned above, and exports the following M function:
	- `lqr_k.m`: Returns the feedback matrix K corresponding to a specific leg length L0.
- `leg_sim.slx`: Simulink model for leg simulation, including the leg Multibody physical model for verifying the leg VMC algorithm.
	- `leg_sim_2020a.slx`: Exported old version model, suitable for R2020a and above.
- `sys_sim.slx`: Simulink model for the overall balance simulation, including the full robot Multibody physical model for verifying the VMC+LQR overall algorithm.
	- `sys_sim_2020a.slx`: Exported old version model, suitable for R2020a and above.

> Note: The exported M functions are directly used in the simulation model and can be converted to C language using the MATLAB Coder toolbox for direct use in esp32 code. You can find the same-named files in the main control code of this project.
