/*
 * km_math.h
 *
 *  Created on: 2017 kov. 25
 *      Author: matas
 */

#ifndef SRC_KM_MATH_H_
#define SRC_KM_MATH_H_

#define M_PI 3.14159265359f
#define M_2PI 2.0f*M_PI
#define DEG_TO_RAD M_PI/180.0f
#define RAD_TO_DEG 180.0f/M_PI

float wrap360(float angle, float unit_mod);
float wrap180(float angle, float unit_mod);
float wrap_2PI(float radian);
float wrap_PI(float radian);

#endif /* SRC_KM_MATH_H_ */
