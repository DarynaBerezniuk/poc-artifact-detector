#include "madgwick.h"
#include <math.h>

void madgwick_init(madgwick_t *f, float beta) {
    f->q0 = 1.0f;
    f->q1 = f->q2 = f->q3 = 0.0f;
    f->beta = beta;
}

static void normalise_q(madgwick_t *f) {
    float n = sqrtf(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    if (n < 1e-9f) {
        f->q0 = 1.0f;
        f->q1 = f->q2 = f->q3 = 0.0f;
        return;
    }
    n = 1.0f / n;
    f->q0 *= n;
    f->q1 *= n;
    f->q2 *= n;
    f->q3 *= n;
}

void madgwick_update(madgwick_t *f, const float gyr[3], const float acc[3], const float mag[3], bool use_mag, float dt) {
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;
    float gx = gyr[0], gy = gyr[1], gz = gyr[2];

    float qd0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qd1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qd2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qd3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (an > 1e-6f) {
        float ax = acc[0] / an, ay = acc[1] / an, az = acc[2] / an;
        float s0, s1, s2, s3;

        float mn = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
        if (use_mag && mn > 1e-9f) {
            float mx = mag[0] / mn, my = mag[1] / mn, mz = mag[2] / mn;
            float _2q0mx = 2.0f * q0 * mx;
            float _2q0my = 2.0f * q0 * my;
            float _2q0mz = 2.0f * q0 * mz;
            float _2q1mx = 2.0f * q1 * mx;
            float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
            float _2q0q2 = 2.0f * q0 * q2;
            float _2q2q3 = 2.0f * q2 * q3;
            float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
            float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
            float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

            float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            float _2bx = sqrtf(hx * hx + hy * hy);
            float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            float _4bx = 2.0f * _2bx;
            float _4bz = 2.0f * _2bz;

            float ea = 2.0f * q1q3 - _2q0q2 - ax;
            float eb = 2.0f * q0q1 + _2q2q3 - ay;
            float ex = _2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx;
            float ey = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my;
            float ez = _2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz;

            s0 = -_2q2 * ea + _2q1 * eb - _2bz * q2 * ex + (-_2bx * q3 + _2bz * q1) * ey + _2bx * q2 * ez;
            s1 = _2q3 * ea + _2q0 * eb - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * ex + (_2bx * q2 + _2bz * q0) * ey + (_2bx * q3 - _4bz * q1) * ez;
            s2 = -_2q0 * ea + _2q3 * eb - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * ex + (_2bx * q1 + _2bz * q3) * ey + (_2bx * q0 - _4bz * q2) * ez;
            s3 = _2q1 * ea + _2q2 * eb + (-_4bx * q3 + _2bz * q1) * ex + (-_2bx * q0 + _2bz * q2) * ey + _2bx * q1 * ez;
        } else {
            float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
            float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
            float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
            float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        }

        float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (sn > 1e-9f) {
            sn = 1.0f / sn;
            qd0 -= f->beta * s0 * sn;
            qd1 -= f->beta * s1 * sn;
            qd2 -= f->beta * s2 * sn;
            qd3 -= f->beta * s3 * sn;
        }
    }

    f->q0 = q0 + qd0 * dt;
    f->q1 = q1 + qd1 * dt;
    f->q2 = q2 + qd2 * dt;
    f->q3 = q3 + qd3 * dt;
    normalise_q(f);
}

float madgwick_heading_rad(const madgwick_t *f) {
    float yaw_ccw = atan2f(2.0f * (f->q0 * f->q3 + f->q1 * f->q2), 1.0f - 2.0f * (f->q2 * f->q2 + f->q3 * f->q3));
    float h = -yaw_ccw;
    if (h < 0.0f) h += 2.0f * (float)M_PI;
    return h;
}

void madgwick_tilt_rad(const madgwick_t *f, float *roll, float *pitch) {
    float sinp = 2.0f * (f->q0 * f->q2 - f->q3 * f->q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp);
    *roll = atan2f(2.0f * (f->q0 * f->q1 + f->q2 * f->q3), 1.0f - 2.0f * (f->q1 * f->q1 + f->q2 * f->q2));
}
