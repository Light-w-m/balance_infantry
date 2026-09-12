#ifndef ADRC_H
#define ADRC_H

#define X_DIM  6
#define U_DIM  2
#define XE_DIM 8

typedef struct {
    float x_hat[X_DIM];      // 估计的6个原状态
    float d_hat[U_DIM];      // 扩张出来的2个输入扰动: dT, dTp

    float A[X_DIM][X_DIM];
    float B[X_DIM][U_DIM];

    // 连续型观测器增益: 8x6
    float L[XE_DIM][X_DIM];

    float dt;
    float d_limit[U_DIM];
} LESO_t;

// typedef struct {
//     float z1;
//     float z2;
//     float z3;

//     float b0;
//     float beta1;
//     float beta2;
//     float beta3;
//     float dt;

//     float d_limit;
// } LESO_t;

// static inline float limit_f(float x, float min_v, float max_v)
// {
//     if (x > max_v) return max_v;
//     if (x < min_v) return min_v;
//     return x;
// }

// void LESO_Init(LESO_t *o, float wo, float b0, float dt, float y0)
// {
//     o->z1 = y0;
//     o->z2 = 0.0f;
//     o->z3 = 0.0f;

//     o->b0 = b0;
//     o->dt = dt;

//     o->beta1 = 3.0f * wo;
//     o->beta2 = 3.0f * wo * wo;
//     o->beta3 = wo * wo * wo;

//     o->d_limit = 1000.0f;
// }

// float LESO_Update(LESO_t *o, float y, float u_last)
// {
//     float e = y - o->z1;

//     float z1_dot = o->z2 + o->beta1 * e;
//     float z2_dot = o->z3 + o->b0 * u_last + o->beta2 * e;
//     float z3_dot = o->beta3 * e;

//     o->z1 += o->dt * z1_dot;
//     o->z2 += o->dt * z2_dot;
//     o->z3 += o->dt * z3_dot;

//     o->z3 = limit_f(o->z3, -o->d_limit, o->d_limit);

//     return o->z3;
// }

void LESO_Init(
    LESO_t *o,
    const float A[6][6],
    const float B[6][2],
    const float L[8][6],
    const float x0[6],
    float dt
);
void LESO_Update(LESO_t *o, const float y[6], const float u_last[2]);

#endif // ADRC_H