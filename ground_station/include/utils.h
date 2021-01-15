#pragma once

// #include <math.h>
// #include <stdint.h>

#define R2D (180.0 / 3.142)
#define D2R (3.142 / 180.0)

// bound a value to a range [min,max]
inline float bound_f(float val, float min, float max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}