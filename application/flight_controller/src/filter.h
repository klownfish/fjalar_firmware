#pragma once

#include <zsl/matrices.h>

typedef struct {
    float *values;
    int size;
    int index;
} window_t;
void window_init(window_t *window, float *values, int size);
void window_insert(window_t * filter, float value);
float window_get_median(window_t * filter);
void window_get_min_max(window_t * filter, float *min, float *max);
float window_get_newest(window_t *filter);
float window_get_oldest(window_t *filter);

typedef struct altitude_filter {
    zsl_real_t P_data[9];
    struct zsl_mtx P;
    zsl_real_t X_data[3];
    struct zsl_mtx X;
    zsl_real_t Q_data[9];
    struct zsl_mtx Q;
    zsl_real_t K_data[9];
    struct zsl_mtx K;
    zsl_real_t R_data[9];
    struct zsl_mtx R;
    uint32_t previous_update;
    bool seeded;
} altitude_filter_t;

void altitude_filter_init(altitude_filter_t *kf);

void altitude_filter_update(altitude_filter_t *kf, float value, uint32_t time);
void altitude_filter_update_accel(altitude_filter_t *kf, float value, float accel, uint32_t time);

float altitude_filter_get_altitude(altitude_filter_t *kf);

float altitude_filter_get_velocity(altitude_filter_t *kf);