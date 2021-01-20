#pragma once

#include <math.h>
#include "state.h"

// #ifdef __cplusplus  
// extern "C" { 
// #endif 

#define R2D (180.0 / 3.142)
#define D2R (3.142 / 180.0)

#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }
#define BoundAbs(_x, _max) Bound(_x, -(_max), (_max))

// bound a value to a range [min,max]
inline float bound_f(float val, float min, float max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}

/* when using attitude from optitrack from pprz_float_math **/
void float_eulers_of_quat(att3f_t *e, struct FloatQuat *q) {
	const float qx2  = q->qx * q->qx;
	const float qy2  = q->qy * q->qy;
	const float qz2  = q->qz * q->qz;
	const float qi2  = q->qi * q->qi;
	const float qiqx = q->qi * q->qx;
	const float qiqy = q->qi * q->qy;
	const float qiqz = q->qi * q->qz;
	const float qxqy = q->qx * q->qy;
	const float qxqz = q->qx * q->qz;
	const float qyqz = q->qy * q->qz;
	const float r11  = -2 * (qxqy - qiqz);
	const float r12  = qi2 - qx2 + qy2 - qz2;
	float r21  =  2 * (qyqz + qiqx);
	const float r31  = -2 * (qxqz - qiqy);
	const float r32  = qi2 - qx2 - qy2 + qz2;

	// asinf does not exist outside [-1,1]
	// BoundAbs(r21, 1.0);
	bound_f(r21, -1.0, 1.0);

	e->yaw = -1 * atan2f(r11, r12);
	e->pitch = -1 * asinf(r21);
	e->roll = atan2f(r31, r32);

	// TODO: verify reference frames
	/** According to pprz;
	// e->yaw = atan2f(r11, r12);
	// e->roll = asinf(r21);
	// e->pitch = atan2f(r31, r32);
	 **/
}


//fonts color
#define COLOR_FBLACK      "\033[30;"
#define COLOR_FRED        "\033[31;"
#define COLOR_FGREEN      "\033[32;"
#define COLOR_FYELLOW     "\033[33;"
#define COLOR_FBLUE       "\033[34;"
#define COLOR_FPURPLE     "\033[35;"
#define COLOR_D_FGREEN    "\033[6;"
#define COLOR_FWHITE      "\033[7;"
#define COLOR_FCYAN       "\x1b[36m"

//background color
#define COLOR_BBLACK      "40m"
#define COLOR_BRED        "41m"
#define COLOR_BGREEN      "42m"
#define COLOR_BYELLOW     "43m"
#define COLOR_BBLUE       "44m"
#define COLOR_BPURPLE     "45m"
#define COLOR_D_BGREEN    "46m"
#define COLOR_BWHITE      "47m"

//end color
#define COLOR_NONE        "\033[0m"


// #ifdef __cplusplus 
// } 
// #endif 