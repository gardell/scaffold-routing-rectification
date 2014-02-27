/*
* DNA.h
*
*  Created on: Feb 13, 2014
*      Author: johan
*/

#ifndef DNA_H_
#define DNA_H_

namespace DNA {
	const double PITCH = 720.0 / 21.0,														// degrees
		STEP = 0.334,
		RADIUS = 1.0,
		SPHERE_RADIUS = 0.13,																// not dna properties, but visual
		OPPOSITE_ROTATION = 155.0;

	const double HALF_TURN_LENGTH = 180.0 / PITCH * STEP;
	const double FULL_TURN_LENGTH = 360.0 / PITCH * STEP;
	const int APPROXIMATION_RADIUS_MULTIPLIER = 4;											// when approximating the cylinder in PhysX, create these exaggerated end bases to compensate for the capsule shape, where their radius is this multiplier times the base RADIUS.

	/*
	* Returns the rounded number of bases required to occupy the given distance.
	*/
	template<typename T>
	int DistanceToBaseCount(T distance) {
		return int(floor(distance / T(STEP) + T(0.5)));
	}
	/*
	* Returns the actual length a helix created to match the distance will actually have.
	*/
	template<typename T>
	T HelixLength(T distance) {
		return T(DistanceToBaseCount(distance) * STEP);
	}

	/*
	* Returns the total rotation done by a with the target length.
	*/
	template<typename T>
	T HelixRotation(T distance) {
		return T(DistanceToBaseCount(distance) * PITCH);
	}
}

#endif /* DNA_H_ */
