#include "KalmanFilter.h"

void kalman_init(KalmanState *state, float32_t dt) {
    // 1. Inicializace stavu (quaternionu)
    // Jednotkový quaternion [1, 0, 0, 0] vyjadřuje nulovou rotaci
	Quaternion_set(1.0f, 0.0f, 0.0f, 0.0f, &state->q0);
    // 2. Inicializace biasu
    // Předpokládáme nulu
    state->bias[0] = 0.0f;
    state->bias[1] = 0.0f;
    state->bias[2] = 0.0f;

    //vynulovani nekterych matic
    for(uint8_t inc=0; inc<STATE_DIM*STATE_DIM; inc++){
    	state->P_data[inc] = 0.0f;
    	state->R_data[inc] = 0.0f;
    	state->H_data[inc] = 0.0f;
    	state->Q_data[inc] = 0.0f;
    	state->I_data[inc] = 0.0f;
    }
    for(uint8_t inc=0; inc<STATE_DIM; inc++)
    	state->I_data[inc*STATE_DIM+inc] = 1.0f;

    //astavíme diagonálu pro ORIENTACI (velká nejistota)
    //zrad^2 říká "věřím měření, ne predikci"
    state->P_data[0] = Q_ANGLE;
    state->P_data[7] = Q_ANGLE;
    state->P_data[14] = Q_ANGLE;

    // Nastavíme diagonálu pro BIAS (menší nejistota)
    // Odhad např. (0.1 rad/s)^2 = 0.01
    state->P_data[21] = Q_BIAS;
	state->P_data[28] = Q_BIAS;
	state->P_data[35] = Q_BIAS;

    // Horni cast diagonaly je merici sum magnetometru
    state->R_data[0] = R_MAG;
    state->R_data[7] = R_MAG;
    state->R_data[14] = R_MAG;

    // Dolni cast diagonaly je merici sum slunecniho senzoru
    state->R_data[21] = R_SUN;
	state->R_data[28] = R_SUN;
	state->R_data[35] = R_SUN;

	// Horni cast diagonaly je procesni sum chyby uhlu
	state->Q_data[0] = Q_ANGLE;
	state->Q_data[7] = Q_ANGLE;
	state->Q_data[14] = Q_ANGLE;

	// Dolni cast diagonaly je procesni sum chyby biasu
	state->Q_data[21] = Q_BIAS;
	state->Q_data[28] = Q_BIAS;
	state->Q_data[35] = Q_BIAS;

	//inicializace matice P
	arm_mat_init_f32(&state->P, STATE_DIM, STATE_DIM, state->P_data);
	//inicializace matice R
	arm_mat_init_f32(&state->R, STATE_DIM, STATE_DIM, state->R_data);
	//inicializaci evidencni matice S
	arm_mat_init_f32(&state->S, STATE_DIM, STATE_DIM, state->S_data);
    //inicializace merici matice H Transponovane
    arm_mat_init_f32(&state->H_T_, STATE_DIM, STATE_DIM, state->H_T_data);
    //inicializace merici matice H Transponovane
    arm_mat_init_f32(&state->K, STATE_DIM, STATE_DIM, state->K_data);
    //inicializace merici matice H Transponovane
    arm_mat_init_f32(&state->invS, STATE_DIM, STATE_DIM, state->invS_data);
    //inicializace matice H
    arm_mat_init_f32(&state->H, STATE_DIM, STATE_DIM, state->H_data);
    //inicializace jednotkove matice
	arm_mat_init_f32(&state->I, STATE_DIM, STATE_DIM, state->I_data);
    //inicializace residua z
    arm_mat_init_f32(&state->z, STATE_DIM, 1, state->z_data);
    //inicializace chyboveho vektoru error state
	arm_mat_init_f32(&state->error_state, STATE_DIM, 1, state->error_state_data);
	//inicializace Prechodove matice F
	arm_mat_init_f32(&state->F, STATE_DIM, STATE_DIM, state->F_data);
	//inicializace Transponovane Prechodove matice
    //F=[I3x3​0 I3x3​​−Δt]⋅
    //  [03x3​   I3x3​ ]
    memcpy(state->F_data, state->I_data, 36*sizeof(float32_t));
    MAT_ELEMENT(state->F, 0, 3) = -dt;
    MAT_ELEMENT(state->F, 1, 4) = -dt;
    MAT_ELEMENT(state->F, 2, 5) = -dt;
	arm_mat_init_f32(&state->F_T_, STATE_DIM, STATE_DIM, state->F_T_data);
	arm_mat_trans_f32(&state->F, &state->F_T_);
	//inicializace matice procesniho sumu
	arm_mat_init_f32(&state->Q, STATE_DIM, STATE_DIM, state->Q_data);
	//inicializace pomocnych matici
	arm_mat_init_f32(&state->Temp_Mat1, STATE_DIM, STATE_DIM, state->Temp_Mat1_data);
	arm_mat_init_f32(&state->Temp_Mat2, STATE_DIM, STATE_DIM, state->Temp_Mat2_data);

}

void kalman_predict_step(KalmanState *state, float gyro_meas[3], float dt) {

    // --- 1. PREDIKCE STAVU ---
    // Ziskame "cistou" rychlost odectenim odhaovaneho zkresleni
    float omega[3];

    omega[0] = gyro_meas[0] - state->bias[0];
    omega[1] = gyro_meas[1] - state->bias[1];
    omega[2] = gyro_meas[2] - state->bias[2];

    //Prevod uhlove rychlosti na quaternion
    Quaternion_normalize(&state->q0, &state->q0);
    Quaternion_from_Omega(dt, omega, &state->q0, &state->q0);

    // --- 2. PREDIKCE KOVARIANCE P (Chybovy stav) ---
    arm_mat_mult_f32(&state->F, &state->P, &state->Temp_Mat1);				//|
    arm_mat_mult_f32(&state->Temp_Mat1, &state->F_T_, &state->Temp_Mat2);	//| Vzorec: P_new = F * P_old * F_T + Q
    arm_mat_add_f32(&state->Temp_Mat2, &state->Q, &state->P);				//|
}

void kalman_correction_step(KalmanState *state, float mag_meas[3], float sun_meas[3]) {
    // 1. Predikce mereni (rotace referenčních vektoru aktualnim quaternionem)
    // Reference: Mag = [1,0,0], Sun = [0,1,0]
	double mag_ref[3] = {1.0f,0.0f,0.0f};
	double sun_ref[3] = {0.0f,0.0f,1.0f};
    // Zde použijeme ten efektivní vzorec s vektorovým součinem

    // ... Rotace ...
    Quaternion_rotate(&state->q0, mag_ref, state->mag_pred);
    Quaternion_rotate(&state->q0, sun_ref, state->sun_pred);

    // Matice H: jak chyba rotace vektoru zavisi na chybe uhlu (derivace rotace vektoru podle uhlu)
    // Používáme mag_pred složky!
    /*{0, -magz, magy, 0, 0, 0},
      {magz,0,-magx, 0, 0, 0},
      {-magy,magx,0, 0, 0, 0},
      {0, -sunz, suny, 0, 0, 0},
      {sunz,0,-sunx, 0, 0, 0},
      {-suny,sunx,0, 0, 0, 0}*/

    // levy horni blok, antisymetricka 3x3 matice z magnetometru
    MAT_ELEMENT(state->H, 0, 1) = -(float32_t)state->mag_pred[2];  // -magz
    MAT_ELEMENT(state->H, 0, 2) =  (float32_t)state->mag_pred[1];   // magy
    MAT_ELEMENT(state->H, 1, 0) =  (float32_t)state->mag_pred[2];   // magz
    MAT_ELEMENT(state->H, 1, 2) = -(float32_t)state->mag_pred[0];  // -magx
    MAT_ELEMENT(state->H, 2, 0) = -(float32_t)state->mag_pred[1];  // -magy
    MAT_ELEMENT(state->H, 2, 1) =  (float32_t)state->mag_pred[0];   // magx

    // levy spodni blok, antisymetricka 3x3 matice ze slunce
    MAT_ELEMENT(state->H, 3, 1) = -(float32_t)state->sun_pred[2];  // -sunz
    MAT_ELEMENT(state->H, 3, 2) =  (float32_t)state->sun_pred[1];   // suny
    MAT_ELEMENT(state->H, 4, 0) =  (float32_t)state->sun_pred[2];   // sunz
    MAT_ELEMENT(state->H, 4, 2) = -(float32_t)state->sun_pred[0];  // -sunx
    MAT_ELEMENT(state->H, 5, 0) = -(float32_t)state->sun_pred[1];  // -suny
    MAT_ELEMENT(state->H, 5, 1) =  (float32_t)state->sun_pred[0];   // sunx

    // 3. Vypocet residua (chyby) z = meas - pred
    for(int i=0; i<3; i++){
    	state->z_data[i]   = (float32_t)mag_meas[i] - (float32_t)state->mag_pred[i];
    	state->z_data[i+3] = (float32_t)sun_meas[i] - (float32_t)state->sun_pred[i];
    }
    // 4. Kalmanův update (Standardní maticová algebra)
    // S = H * P * H_T + R
    // K = P * H_T * inv(S)
    // error_state = K * z
    // P_new = (I - K * H) * P

    arm_mat_trans_f32(&state->H, &state->H_T_);		    //Transpozice matice H
    arm_mat_mult_f32(&state->H, &state->P, &state->Temp_Mat1);  		//|
    arm_mat_mult_f32(&state->Temp_Mat1, &state->H_T_, &state->Temp_Mat2);	    //|S = H * P * H_T + R
    arm_mat_add_f32(&state->Temp_Mat2, &state->R, &state->S);	   		//|
    arm_mat_inverse_f32(&state->S, &state->invS);		//Inverze matice S
    arm_mat_mult_f32(&state->P, &state->H_T_, &state->Temp_Mat1);		//| K = P * H_T * inv(S)
    arm_mat_mult_f32(&state->Temp_Mat1, &state->invS, &state->K);		//|
    arm_mat_mult_f32(&state->K, &state->z, &state->error_state);	// error_state = K * z
    arm_mat_mult_f32(&state->K, &state->H, &state->Temp_Mat1);	//| Korekce kovariace
    arm_mat_sub_f32(&state->I, &state->Temp_Mat1, &state->Temp_Mat2);	//| P_new = (I - K * H) * P
    arm_mat_mult_f32(&state->Temp_Mat2, &state->P, &state->Temp_Mat1);	//|
    memcpy(state->P_data, state->Temp_Mat1_data, 36*sizeof(float32_t));


    // 5. INJEKCE CHYBY (Klíčový moment!)

    // A) Oprava Biasu
    for(int i=0; i<3; i++)
        state->bias[i] += (float)state->error_state_data[i+3];

    // B) Oprava Quaternionu (Multiplikativní)
    // error_state[0..2] je vektor rotace. Pro malé úhly vytvoříme "chybový quaternion".
    // dq = [1, error_x/2, error_y/2, error_z/2] (přibližně pro malé úhly)

    Quaternion d_quat;
    // Nový quaternion = Starý_quaternion * chybový_quaternion
    // q_new = quat_multiply(state->q, dq);
    // Reprezentujeme quaternion jako cos(phi/2) + sin(phi/2)*(i + j + k) jelikoz predpokladame ze chybovy uhel je velmi maly
    // pak muzem provest zjednoduseni cos(phi/2) =1 a sin(phi/2) = phi/2
    Quaternion_set(1, -0.5f * (double)state->error_state_data[0],
    				  -0.5f * (double)state->error_state_data[1],
		  			  -0.5f * (double)state->error_state_data[2], &d_quat); //inicializace chybového quaternionu
    Quaternion_normalize(&d_quat, &d_quat);
    Quaternion_multiply(&state->q0, &d_quat, &state->q0);
    Quaternion_normalize(&state->q0, &state->q0);

}


