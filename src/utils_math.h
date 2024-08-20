#ifndef UTILS_MATH_H_
#define UTILS_MATH_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

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

#endif