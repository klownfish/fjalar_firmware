// #include <alloca.h>
#include <string.h>
#include <math.h>
#include <zsl/matrices.h>
// #include <pla.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
#include "filter.h"

LOG_MODULE_REGISTER(filter, CONFIG_APP_FILTER_LOG_LEVEL);

void window_init(window_t *window, float *values, int size) {
    window->values = values;
    window->size = size;
    window->index = -1;
}

void window_insert(window_t *filter, float value) {
    if (filter->index == -1) {
        for (int i = 0; i < filter->size; i++) {
            filter->values[i] = value;
        }
    }
    filter->values[filter->index] = value;
    filter->index = (filter->index + 1) % filter->size;
}

float window_get_median(window_t *filter) {
    // float *tmp = alloca(sizeof(float) * filter->size);
    float tmp[filter->size];
    memcpy(tmp, filter->values, sizeof(float) * filter->size);
    for (int i = 1; i < filter->size; i++) {
        float x = tmp[i];
        int j;
        for (j = i - 1; j >= 0 && tmp[j] > x; j--) {
            tmp[j + 1] = tmp[j];
        }
        tmp[j + 1] = x;
    }

    return tmp[filter->size / 2];
}

void window_get_min_max(window_t *filter, float *min, float *max) {
    *min = 1e99;
    *max = -1e99;
    for (int i = 1; i < filter->size; i++) {
        float val = filter->values[i];
        *min = val < *min ? val : *min;
        *max = val > *max ? val : *max;
    }
}

float window_get_oldest(window_t *filter) {
    return filter->values[filter->index];
}

float window_get_newest(window_t *filter) {
    int index = filter->index - 1 >= 0 ? filter->index - 1 : filter->size - 1;
    return filter->values[index];
}

void altitude_filter_init(altitude_filter_t *kf) {
    float process_noise = 0.01;
    float measurement_noise = 10;
    float accel_noise = 0.001;

    float P_init[9] = {
        process_noise, 0, 0,
        0, process_noise, 0,
        0, 0, process_noise
    };
    kf->P.data = kf->P_data;
    kf->P.sz_rows = 3;
    kf->P.sz_cols = 3;
    memcpy(kf->P_data, P_init, sizeof(P_init));

    float X_init[3] = {
        0,
        0,
        0
    };
    kf->X.data = kf->X_data;
    kf->X.sz_rows = 3;
    kf->X.sz_cols = 1;
    memcpy(kf->X_data, X_init, sizeof(X_init));

    float Q_init[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    kf->Q.data = kf->Q_data;
    kf->Q.sz_rows = 3;
    kf->Q.sz_cols = 3;
    memcpy(kf->Q_data, Q_init, sizeof(Q_init));

    float K_init[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    kf->K.data = kf->K_data;
    kf->K.sz_rows = 3;
    kf->K.sz_cols = 3;
    memcpy(kf->K_data, K_init, sizeof(K_init));

    float R_init[9] = {
        measurement_noise, 0, 0,
        0, 0, 0,
        0, 0, accel_noise
    };
    kf->R.data = kf->R_data;
    kf->R.sz_rows = 3;
    kf->R.sz_cols = 3;
    memcpy(kf->R_data, R_init, sizeof(R_init));
    kf->seeded = false;
}

void altitude_filter_update_accel(altitude_filter_t *kf, float value, float acceleration, uint32_t time) {
    if (!kf->seeded) {
        kf->X_data[0] = value;
        kf->previous_update = time;
        kf->seeded = true;
        return;
    }

    float dt = (time - kf->previous_update) / 1000.0;
    kf->previous_update = time;

    zsl_real_t A_data[9] = {
        1, dt, dt * dt / 2,
        0, 1, dt,
        0, 0, 1
    };
    struct zsl_mtx A = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = A_data,
    };

    zsl_real_t At_data[9] = {
        1, 0, 0,
        dt, 1, 0,
        dt * dt / 2, dt, 1
    };
    struct zsl_mtx At = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = At_data,
    };

    zsl_real_t Z_data[3] = {
        value, 0, acceleration,
    };
    struct zsl_mtx Z = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = Z_data
    };

    zsl_real_t H_data[9] = {
        1, 0, 0,
        0, 0, 0,
        0, 0, 1
    };
    struct zsl_mtx H = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = H_data,
    };
    struct zsl_mtx Ht = H;

    zsl_real_t Pp_data[9] = {
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    };
    struct zsl_mtx Pp = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = Pp_data,
    };

    zsl_real_t Xp_data[3];
    struct zsl_mtx Xp = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = Xp_data,
    };

    zsl_real_t I_data[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    struct zsl_mtx I = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = I_data,
    };

    // predict state
    zsl_mtx_mult(&A, &kf->X, &Xp);

    // predict covariance
    ZSL_MATRIX_DEF(P_At, 3, 3);
    zsl_mtx_mult(&kf->P, &At, &P_At);
    ZSL_MATRIX_DEF(A_P_At, 3, 3);
    zsl_mtx_mult(&A, &P_At, &A_P_At);
    zsl_mtx_add(&A_P_At, &kf->Q, &Pp);

    // update kalman gain (should be constant so can move to init)
    ZSL_MATRIX_DEF(Pp_Ht, 3, 3);
    zsl_mtx_mult(&Pp, &Ht, &Pp_Ht);
    ZSL_MATRIX_DEF(H_Pp_Ht, 3, 3);
    zsl_mtx_mult(&H, &Pp_Ht, &H_Pp_Ht);
    ZSL_MATRIX_DEF(H_Pp_Ht_ADD_R, 3, 3);
    zsl_mtx_add(&H_Pp_Ht, &kf->R, &H_Pp_Ht_ADD_R);
    ZSL_MATRIX_DEF(H_Pp_Ht_ADD_R_INV, 3, 3);
    zsl_mtx_inv_3x3(&H_Pp_Ht_ADD_R, &H_Pp_Ht_ADD_R_INV);
    ZSL_MATRIX_DEF(Ht_H_Pp_Ht_ADD_R_INV, 3, 3);
    zsl_mtx_mult(&Ht, &H_Pp_Ht_ADD_R_INV, &Ht_H_Pp_Ht_ADD_R_INV);
    zsl_mtx_mult(&Pp, &Ht_H_Pp_Ht_ADD_R_INV, &kf->K);
    // LOG_INF("K %f %f %f %f %f %f %f %f %f", kf->K_data[0],kf->K_data[1],kf->K_data[2],kf->K_data[3],kf->K_data[4],kf->K_data[5],kf->K_data[6],kf->K_data[7],kf->K_data[8]);
    // update state
    ZSL_MATRIX_DEF(H_Xp, 3, 1);
    zsl_mtx_mult(&H, &Xp, &H_Xp);
    ZSL_MATRIX_DEF(Z_SUB_H_Xp, 3, 1);
    zsl_mtx_sub(&Z, &H_Xp, &Z_SUB_H_Xp);
    ZSL_MATRIX_DEF(K_Z_SUB_H_Xp, 3, 1);
    zsl_mtx_mult(&kf->K, &Z_SUB_H_Xp, &K_Z_SUB_H_Xp);
    zsl_mtx_add(&Xp, &K_Z_SUB_H_Xp, &kf->X);

    // update covariance
    ZSL_MATRIX_DEF(K_H, 3, 3);
    zsl_mtx_mult(&kf->K, &H, &K_H);
    ZSL_MATRIX_DEF(I_SUB_K_H, 3, 3);
    zsl_mtx_sub(&I, &K_H, &I_SUB_K_H);
    zsl_mtx_mult(&I_SUB_K_H, &Pp, &kf->P);
}

void altitude_filter_update(altitude_filter_t *kf, float value, uint32_t time) {
    if (!kf->seeded) {
        kf->X_data[0] = value;
        kf->previous_update = time;
        kf->seeded = true;
        return;
    }

    float dt = (time - kf->previous_update) / 1000.0;
    kf->previous_update = time;

    zsl_real_t A_data[9] = {
        1, dt, dt * dt / 2,
        0, 1, dt,
        0, 0, 1
    };
    struct zsl_mtx A = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = A_data,
    };

    zsl_real_t At_data[9] = {
        1, 0, 0,
        dt, 1, 0,
        dt * dt / 2, dt, 1
    };
    struct zsl_mtx At = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = At_data,
    };

    zsl_real_t Z_data[3] = {
        value, 0, 0,
    };
    struct zsl_mtx Z = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = Z_data
    };

    zsl_real_t H_data[9] = {
        1, 0, 0,
        0, 0, 0,
        0, 0, 0
    };
    struct zsl_mtx H = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = H_data,
    };
    struct zsl_mtx Ht = H;

    zsl_real_t Pp_data[9] = {
        0, 0, 0,
        0, 0, 0,
        0, 0, 0
    };
    struct zsl_mtx Pp = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = Pp_data,
    };

    zsl_real_t Xp_data[3];
    struct zsl_mtx Xp = {
        .sz_rows = 3,
        .sz_cols = 1,
        .data = Xp_data,
    };

    zsl_real_t I_data[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
    struct zsl_mtx I = {
        .sz_rows = 3,
        .sz_cols = 3,
        .data = I_data,
    };


    // predict state
    zsl_mtx_mult(&A, &kf->X, &Xp);

    // predict covariance
    ZSL_MATRIX_DEF(P_At, 3, 3);
    zsl_mtx_mult(&kf->P, &At, &P_At);
    ZSL_MATRIX_DEF(A_P_At, 3, 3);
    zsl_mtx_mult(&A, &P_At, &A_P_At);
    zsl_mtx_add(&A_P_At, &kf->Q, &Pp);

    // update kalman gain (should be constant so can move to init)
    ZSL_MATRIX_DEF(Pp_Ht, 3, 3);
    zsl_mtx_mult(&Pp, &Ht, &Pp_Ht);
    ZSL_MATRIX_DEF(H_Pp_Ht, 3, 3);
    zsl_mtx_mult(&H, &Pp_Ht, &H_Pp_Ht);
    ZSL_MATRIX_DEF(H_Pp_Ht_ADD_R, 3, 3);
    zsl_mtx_add(&H_Pp_Ht, &kf->R, &H_Pp_Ht_ADD_R);
    ZSL_MATRIX_DEF(H_Pp_Ht_ADD_R_INV, 3, 3);
    zsl_mtx_inv_3x3(&H_Pp_Ht_ADD_R, &H_Pp_Ht_ADD_R_INV);
    ZSL_MATRIX_DEF(Ht_H_Pp_Ht_ADD_R_INV, 3, 3);
    zsl_mtx_mult(&Ht, &H_Pp_Ht_ADD_R_INV, &Ht_H_Pp_Ht_ADD_R_INV);
    zsl_mtx_mult(&Pp, &Ht_H_Pp_Ht_ADD_R_INV, &kf->K);
    // LOG_INF("K %f %f %f %f %f %f %f %f %f", kf->K_data[0],kf->K_data[1],kf->K_data[2],kf->K_data[3],kf->K_data[4],kf->K_data[5],kf->K_data[6],kf->K_data[7],kf->K_data[8]);
    // update state
    ZSL_MATRIX_DEF(H_Xp, 3, 1);
    zsl_mtx_mult(&H, &Xp, &H_Xp);
    ZSL_MATRIX_DEF(Z_SUB_H_Xp, 3, 1);
    zsl_mtx_sub(&Z, &H_Xp, &Z_SUB_H_Xp);
    ZSL_MATRIX_DEF(K_Z_SUB_H_Xp, 3, 1);
    zsl_mtx_mult(&kf->K, &Z_SUB_H_Xp, &K_Z_SUB_H_Xp);
    zsl_mtx_add(&Xp, &K_Z_SUB_H_Xp, &kf->X);

    // update covariance
    ZSL_MATRIX_DEF(K_H, 3, 3);
    zsl_mtx_mult(&kf->K, &H, &K_H);
    ZSL_MATRIX_DEF(I_SUB_K_H, 3, 3);
    zsl_mtx_sub(&I, &K_H, &I_SUB_K_H);
    zsl_mtx_mult(&I_SUB_K_H, &Pp, &kf->P);
}

float altitude_filter_get_altitude(altitude_filter_t *kf) {
    return kf->X_data[0];
}

float altitude_filter_get_velocity(altitude_filter_t *kf) {
    return kf->X_data[1];
}