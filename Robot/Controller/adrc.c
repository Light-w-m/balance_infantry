#include "adrc.h"
#include "string.h"

static inline float limit_f(float x, float min_v, float max_v);

void LESO_Init(
    LESO_t *o,
    const float A[6][6],
    const float B[6][2],
    const float L[8][6],
    const float x0[6],
    float dt
)
{
    memset(o, 0, sizeof(LESO_t));

    for (int i = 0; i < 6; i++) {
        o->x_hat[i] = x0[i];

        for (int j = 0; j < 6; j++) {
            o->A[i][j] = A[i][j];
        }

        for (int j = 0; j < 2; j++) {
            o->B[i][j] = B[i][j];
        }
    }

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
            o->L[i][j] = L[i][j];
        }
    }

    o->d_hat[0] = 0.0f;
    o->d_hat[1] = 0.0f;

    o->dt = dt;
    o->d_limit[0] = 2.0f;   // 对应 T 扰动限幅
    o->d_limit[1] = 10.0f;  // 对应 Tp 扰动限幅
}

void LESO_Update(LESO_t *o, const float y[6], const float u_last[2])
{
    float e[6];
    float dx[8] = {0};

    for (int i = 0; i < 6; i++) {
        e[i] = y[i] - o->x_hat[i];
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++)
            dx[i] += o->A[i][j] * o->x_hat[j];

        for (int j = 0; j < 2; j++)
            dx[i] += o->B[i][j] * (u_last[j] + o->d_hat[j]);
    }

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++)
            dx[i] += o->L[i][j] * e[j];
    }

    for (int i = 0; i < 6; i++)
        o->x_hat[i] += o->dt * dx[i];

    for (int i = 0; i < 2; i++) {
        o->d_hat[i] += o->dt * dx[6 + i];
        o->d_hat[i] = limit_f(o->d_hat[i], -o->d_limit[i], o->d_limit[i]);
    }
}

static inline float limit_f(float x, float min_v, float max_v)
{
    if (x > max_v) return max_v;
    if (x < min_v) return min_v;
    return x;
}
