# Multiplicative Extended Kalman Filter (MEKF) for Nanosatellite ADCS

## Overview
This repository contains a C implementation of a Multiplicative Extended Kalman Filter (MEKF). [cite_start]It was specifically designed for an Attitude Determination and Control System (ADCS) of a CubeSat built on the STM32 architecture. [cite_start]The library performs robust sensor data fusion to accurately estimate the spatial orientation of the nanosatellite using quaternions.

## Key Features
* [cite_start]**Quaternion-Based Kinematics:** Avoids gimbal lock issues and provides smooth attitude representation.
* [cite_start]**Sensor Fusion:** Fuses data from IMU sensors (Gyroscopes, Accelerometers, Magnetometers).
* [cite_start]**Embedded-Optimized:** Written in standard C, targeted and tested on STM32 microcontrollers (via STM32CubeIDE).

## Mathematical Model
The core of the MEKF relies on estimating the error state to update the global attitude quaternion. The state vector $x$ consists of the attitude quaternion $\mathbf{q}$ and the angular velocity $\mathbf{\omega}$:

$$x = \begin{bmatrix} \mathbf{q} \\ \mathbf{\omega} \end{bmatrix}$$

*(Optional: Add 1-2 sentences here explaining your specific prediction and correction steps, or the sensors used for the measurement model).*

## Usage Example
Integration into an existing STM32 project is straightforward. Here is a basic example of the filter loop:

```c
#include "mekf.h"

// 1. Initialize the filter structures
MEKF_State filter_state;
MEKF_Init(&filter_state);

// Main control loop
while(1) {
    // 2. Read raw data from sensors (e.g., via I2C/SPI)
    Vector3f gyro_data = read_gyro();
    Vector3f accel_data = read_accel();
    Vector3f mag_data = read_mag();

    // 3. Perform prediction and update steps
    // dt represents the time step since the last update
    MEKF_Update(&filter_state, gyro_data, accel_data, mag_data, dt);

    // 4. Retrieve the fused quaternion for the ADCS control logic
    Quaternion current_attitude = filter_state.q;
}
