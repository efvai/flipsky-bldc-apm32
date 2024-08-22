#ifndef UTILS_MATH_H_
#define UTILS_MATH_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

void utils_fast_sincos_better(float angle, float *sin, float *cos);
float utils_min_abs(float va, float vb);
float utils_max_abs(float va, float vb);

// Constants
#define ONE_BY_SQRT3			(0.57735026919)
#define TWO_BY_SQRT3			(2.0f * 0.57735026919)
#define SQRT3_BY_2				(0.86602540378)
#define COS_30_DEG				(0.86602540378)
#define SIN_30_DEG				(0.5)
#define COS_MINUS_30_DEG		(0.86602540378)
#define SIN_MINUS_30_DEG		(-0.5)
#define ONE_BY_SQRT2			(0.7071067811865475)

// Return the sign of the argument. -1.0 if negative, 1.0 if zero or positive.
#define SIGN(x)				(((x) < 0.0) ? -1.0 : 1.0)

// Squared
#define SQ(x)				((x) * (x))

// Two-norm of 2D vector
//#define NORM2(x,y)		(sqrt(SQ(x) + SQ(y)))
#define NORM2_f(x,y)		(sqrtf(SQ(x) + SQ(y)))

// nan and infinity check for floats
#define UTILS_IS_INF(x)		((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0 : x)

// Handy conversions for radians/degrees and RPM/radians-per-second
#define DEG2RAD_f(deg) ((deg) * (float)(M_PI / 180.0))
#define RAD2DEG_f(rad) ((rad) * (float)(180.0 / M_PI))
#define RPM2RADPS_f(rpm) ((rpm) * (float)((2.0 * M_PI) / 60.0))
#define RADPS2RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0 / (2.0 * M_PI)))

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

// For double precision literals
#define D(x) 				((double)x##L)

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

static inline void utils_truncate_number_abs(float *number, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < -max) {
		*number = -max;
	}
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
static inline void utils_norm_angle(float *angle) {
	*angle = fmodf(*angle, 360.0);

	if (*angle < 0.0) {
		*angle += 360.0;
	}
}

static inline void utils_truncate_number(float *number, float min, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

static inline void utils_truncate_number_int(int *number, int min, int max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

static inline void utils_truncate_number_uint32(uint32_t *number, uint32_t min, uint32_t max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

static inline float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Truncate the magnitude of a vector.
 *
 * @param x
 * The first component.
 *
 * @param y
 * The second component.
 *
 * @param max
 * The maximum magnitude.
 *
 * @return
 * True if saturation happened, false otherwise
 */
static inline bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag = NORM2_f(*x, *y);
	max = fabsf(max);

	if (mag < 1e-10) {
		mag = 1e-10;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

#endif