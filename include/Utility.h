/*
* Utility.h
*
*  Created on: Feb 10, 2014
*      Author: johan
*/

#ifndef UTILITY_H_
#define UTILITY_H_

#include <Definition.h>
#include <cmath>
#include <vector>

#ifdef __GNUC__

#ifndef likely
#define likely(cond)   (__builtin_expect(!!(cond), 1))
#endif /* N likely */

#ifndef unlikely
#define unlikely(cond) (__builtin_expect(!!(cond), 0))
#endif /* N unlikely */

#else
#define likely(cond) cond
#define unlikely(cond) cond

#endif /* N __GNUC__ */

/*
* Missing useful math methods.
*/

template<typename T>
int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

template<typename T>
int sgn_nozero(T val) {
	return (T(0) <= val) - (val < T(0));
}

template<typename T>
T toRadians(T degrees) {
	return T(degrees * M_PI / 180);
}

template<typename T>
T toDegrees(T radians) {
	return T(radians * 180 / M_PI);
}

static const physx::PxVec3 kPosXAxis(1, 0, 0);
static const physx::PxVec3 kPosYAxis(0, 1, 0);
static const physx::PxVec3 kPosZAxis(0, 0, 1);

static const physx::PxVec3 kNegXAxis(-1, 0, 0);
static const physx::PxVec3 kNegYAxis(0, -1, 0);
static const physx::PxVec3 kNegZAxis(0, 0, -1);

inline auto signedAngle(const physx::PxVec3 & from, const physx::PxVec3 & to, const physx::PxVec3 & normal)->decltype(from.dot(to)) {
	const int sign = sgn_nozero((normal.cross(from)).dot(to));
	return sign * std::acos(from.getNormalized().dot(to.getNormalized()));
}

#endif /* UTILITY_H_ */
