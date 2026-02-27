# Multiplicative Extended Kalman Filter (MEKF) for Nanosatellite ADCS

## Overview
This repository contains a C implementation of a Multiplicative Extended Kalman Filter (MEKF). [cite_start]It was specifically designed for an Attitude Determination and Control System (ADCS) of a CubeSat built on the STM32 architecture. [cite_start]The library performs robust sensor data fusion to accurately estimate the spatial orientation of the nanosatellite using quaternions. My implementation uses data from gyroscope, magnetometer and sun sensor, in fact input data are any two noncolinear vectors so other sensor can be use. For example you can use accellometer and magnetometer.

## Key Features
* [cite_start]**Quaternion-Based Kinematics:** Avoids gimbal lock issues and provides smooth attitude representation.
* [cite_start]**Sensor Fusion:** Fuses data from IMU sensors (Gyroscopes, Accelerometers, Magnetometers).
* [cite_start]**Embedded-Optimized:** Written in standard C for stm32 architectures, arm_math library is used for matrix math operations

## Mathematical Model
The core of the MEKF relies on estimating the error state to update the global attitude quaternion. The state vector $x$ consists of the attitude quaternion $\mathbf{q}$ and the angular velocity $\mathbf{\omega}$:

$$x = \begin{bmatrix} \mathbf{q} \\ \mathbf{\omega} \end{bmatrix}$$


## Usage Example
Integration into an existing STM32 project is straightforward. Filter loop works in two steps prediction and correction. Prediction step should be triggered in dependece of time constant dt you choose. Then the prediction step could be trigered less often and should be tunned to your aplication. Here is basic structure filter loop 

```c
#include "KalmanFilter.h"

// 1. Create instance
KalmanState KF;

int main(void){

// 2. Initialize the filter structures
kalman_init(&KF, dt);

}

// Create event for predict step
kalman_predict_step(&KF, omega, dt);


// Create loop for correction step
kalman_correction_step(&KF, vector1, vector2);
