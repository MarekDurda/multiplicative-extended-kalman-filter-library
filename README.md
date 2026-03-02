# Multiplicative Extended Kalman Filter (MEKF) for Nanosatellite ADCS

## Overview
This repository contains a C implementation of a Multiplicative Extended Kalman Filter (MEKF). It was specifically designed for an Attitude Determination and Control System (ADCS) of a CubeSat built on the STM32 architecture. The library performs robust sensor data fusion to accurately estimate the spatial orientation of the nanosatellite using quaternions. This implementation uses data from gyroscope, magnetometer and sun sensor, in fact input data are any two noncolinear vectors so other sensor can be used. For example you can use accelerometer insted of sun sensor.

## Key Features
* **Quaternion-Based Kinematics:** Avoids gimbal lock issues and provides smooth attitude representation.
* **Sensor Fusion:** Fuses data from IMU sensors (Gyroscopes, Accelerometers, Magnetometers).
* **Embedded-Optimized:** Written in standard C for stm32 architectures, arm_math library is used for matrix math operations


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

// 3. Create event for predict step with given frequency
TimerEvent1{
  kalman_predict_step(&KF, omega, dt);
}

// 4. Create event for correction step with given frequency
TimerEvent2{
  kalman_correction_step(&KF, vector1, vector2);
}
