#include "km_math.h"

float wrap360(float angle, float unit_mod) {
    const float ang_360 = 360.f * unit_mod;
    float res = fmodf(angle, ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}

float wrap180(float angle, float unit_mod) {
    float res = wrap360(angle, unit_mod);
    if (res > 180.f * unit_mod) {
        res -= 360.f * unit_mod;
    }
    return res;
}

float wrap_2PI(float radian)
{
    float res = fmodf(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

float wrap_PI(float radian)
{
    float res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}
