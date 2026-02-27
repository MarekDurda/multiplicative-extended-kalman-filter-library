/*
 * KalmanFilter.h
 *
 *  Created on: Nov 26, 2025
 *      Author: Spongebob
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_


#include "Quaternion.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>


#define STATE_DIM 6 // (3 pro chybu orientace, 3 pro chybu biasu)
#define MEAS_DIM 6  // (3 pro magnetometr, 3 pro slunce)
#define Q_ANGLE 0.01
#define Q_BIAS 0.000000001
#define R_MAG  0.0001
#define R_SUN  0.00001
#define MAT_ELEMENT(mat, rowIndex, colIndex) (mat.pData[(colIndex) + (uint32_t)mat.numCols*(rowIndex)])


typedef struct{
	Quaternion q0;
    float bias[3];   // Bias gyroskopu [x, y, z]
    float32_t P_data[STATE_DIM*STATE_DIM];
    float32_t H_data[STATE_DIM*STATE_DIM];
    float32_t R_data[STATE_DIM*STATE_DIM];
    float32_t K_data[STATE_DIM*STATE_DIM];
    float32_t S_data[STATE_DIM*STATE_DIM];
    float32_t I_data[STATE_DIM*STATE_DIM];
    float32_t invS_data[STATE_DIM*STATE_DIM];
    float32_t H_T_data[STATE_DIM*STATE_DIM];
    float32_t F_data[STATE_DIM*STATE_DIM];
    float32_t F_T_data[STATE_DIM*STATE_DIM];
    float32_t Q_data[STATE_DIM*STATE_DIM];
    float32_t z_data[STATE_DIM];
    float32_t error_state_data[STATE_DIM];
    float32_t Temp_Mat1_data[STATE_DIM*STATE_DIM], Temp_Mat2_data[STATE_DIM*STATE_DIM];

    arm_matrix_instance_f32 P;			// Kovarianční matice chyb
    arm_matrix_instance_f32 R;			// Matice mericiho sumu
    arm_matrix_instance_f32 K;			// Matice kalmanova zisku
    arm_matrix_instance_f32 S;  		// Evidencni matice
    arm_matrix_instance_f32 invS;  		// Inverzni evidencni matice
    arm_matrix_instance_f32 H;			// Merici matice
    arm_matrix_instance_f32 H_T_;		// Transponovana matice H
    arm_matrix_instance_f32 F;			// Prechodova matice
    arm_matrix_instance_f32 F_T_;		// Transponovana matice F
    arm_matrix_instance_f32 Q;			// Matice procesniho sumu
    arm_matrix_instance_f32 I;			// Jednotkova matice
    arm_matrix_instance_f32 z;			//vektor residui
    arm_matrix_instance_f32 error_state;		//stavovy vektor
    arm_matrix_instance_f32 Temp_Mat1, Temp_Mat2;	// Pomocne vypocetni matice

    double mag_pred[3];
    double sun_pred[3];


}KalmanState;

// Inicializace
void kalman_init(KalmanState *state, float32_t dt);
// Predikce
void kalman_predict_step(KalmanState *state, float gyro_meas[3], float dt);
// Korekce
void kalman_correction_step(KalmanState *state, float mag_meas[3], float sun_meas[3]);

#endif /* INC_KALMANFILTER_H_ */
