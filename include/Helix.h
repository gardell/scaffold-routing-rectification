#ifndef _HELIX_H_
#define _HELIX_H_

#include <Utility.h>
#include <PxPhysicsAPI.h>

#include <array>
#include <utility>

class Helix {
public:
	const static physx::PxReal kDensity;
	const static physx::PxReal kStaticFriction, kDynamicFriction, kRestitution;

	enum AttachmentPoint {
		kForwardThreePrime = 0,
		kForwardFivePrime = 1,
		kBackwardThreePrime = 2,
		kBackwardFivePrime = 3,
		kNoAttachmentPoint = 4
	};

	Helix(physx::PxPhysics & physics, physx::PxScene & scene, int bases, const physx::PxTransform & transform);

	inline physx::PxTransform getTransform() const {
		return rigidBody->getGlobalPose();
	}

	void attach(physx::PxPhysics & physics, Helix & other, AttachmentPoint thisPoint, AttachmentPoint otherPoint);
	bool detach(AttachmentPoint point);

	physx::PxReal getSeparation(AttachmentPoint atPoint);

	inline unsigned int getBaseCount() const {
		return bases;
	}

private:

	AttachmentPoint otherPoint(AttachmentPoint point, const Helix & other) const;

	physx::PxRigidDynamic *rigidBody = NULL;

	struct Connection {
		physx::PxJoint *joint;
		Helix *helix;

		inline Connection(Helix & helix, physx::PxJoint & joint) : helix(&helix), joint(&joint) {}
		inline Connection() : helix(NULL), joint(NULL) {}
	};
	std::array<Connection, 4> joints{ { Connection(), Connection(), Connection(), Connection() } }; // Index by AttachmentPoint.

	const unsigned int bases;

	static physx::PxMaterial *material;
};

#endif /* _HELIX_H_ */
