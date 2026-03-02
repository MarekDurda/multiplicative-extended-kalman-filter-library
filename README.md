# Multiplicative Extended Kalman Filter (MEKF) for Nanosatellite ADCS

## Overview
This repository contains a C implementation of a Multiplicative Extended Kalman Filter (MEKF). It was specifically designed for an Attitude Determination and Control System (ADCS) of a CubeSat built on the STM32 architecture. The library performs robust sensor data fusion to accurately estimate the spatial orientation of the nanosatellite using quaternions. 

This implementation uses data from a gyroscope, magnetometer, and a sun sensor. In fact, the input data can be any two non-collinear vectors, meaning other sensors can be easily integrated (for example, you can use an accelerometer instead of a sun sensor).

## Key Features
* **Quaternion-Based Kinematics:** Avoids gimbal lock issues and provides smooth attitude representation.
* **Sensor Fusion:** Fuses data from IMU sensors (Gyroscopes, Accelerometers/Sun Sensors, Magnetometers).
* **Embedded-Optimized:** Written in standard C for STM32 architectures, utilizing the `arm_math` library for highly efficient matrix operations.

## Usage Example
Integration into an existing STM32 project is straightforward. The filter loop works in two distinct steps: prediction and correction. The prediction step should be triggered depending on the time constant `dt` you choose. The correction step can be triggered less frequently and should be tuned to your specific application. 

Here is the basic structure of the filter loop:

```c
#include "KalmanFilter.h"

// 1. Create filter instance
KalmanState KF;

int main(void) {
    // 2. Initialize the filter structures
    kalman_init(&KF, dt);

    while(1) {
        // Main loop
    }
}

// 3. Timer Interrupt / Task for the Prediction step
void TimerEvent1_Callback(void) {
    kalman_predict_step(&KF, omega, dt);
}

// 4. Timer Interrupt / Task for the Correction step
void TimerEvent2_Callback(void) {
    kalman_correction_step(&KF, vector1, vector2);
}
