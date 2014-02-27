#include <DNA.h>
#include <Helix.h>

#include <cassert>
#include <initializer_list>

const physx::PxReal Helix::kDensity(10);
const physx::PxReal Helix::kStaticFriction(physx::PxReal(0.5)), Helix::kDynamicFriction(physx::PxReal(0.5)), Helix::kRestitution(physx::PxReal(0.6));
physx::PxMaterial *Helix::material(NULL);

physx::PxVec3 localFrame(Helix::AttachmentPoint point, int bases) {
	switch (point) {
	case Helix::kForwardThreePrime:
		return physx::PxQuat(physx::PxReal(toRadians(DNA::PITCH * bases)), physx::PxVec3(0, 0, 1)).rotate(physx::PxVec3(0, 0, 1));
	case Helix::kForwardFivePrime:
		return physx::PxVec3(0, 0, 0);
	case Helix::kBackwardThreePrime:
		return physx::PxQuat(physx::PxReal(toRadians(DNA::OPPOSITE_ROTATION)), physx::PxVec3(0, 0, 1)).rotate(physx::PxVec3(0, 0, 1));
	case Helix::kBackwardFivePrime:
		return physx::PxQuat(physx::PxReal(toRadians(DNA::PITCH * bases + DNA::OPPOSITE_ROTATION)), physx::PxVec3(0, 0, 1)).rotate(physx::PxVec3(0, 0, 1));
	default:
		assert(0);
		return physx::PxVec3();
	}
}

std::initializer_list<Helix::AttachmentPoint> opposites(Helix::AttachmentPoint point) {
	switch (point) {
	case Helix::kForwardThreePrime:
	case Helix::kBackwardThreePrime:
		return{ Helix::kForwardFivePrime, Helix::kBackwardFivePrime };
	case Helix::kForwardFivePrime:
	case Helix::kBackwardFivePrime:
		return { Helix::kForwardThreePrime, Helix::kBackwardThreePrime };
	default:
		assert(0);
		return{ Helix::kNoAttachmentPoint, Helix::kNoAttachmentPoint };
	}
}

Helix::Helix(physx::PxPhysics & physics, physx::PxScene & scene, int bases, const physx::PxTransform & transform) : bases(bases) {
	physx::PxReal length(physx::PxReal(bases * DNA::STEP));
	assert(length > DNA::RADIUS * 2);

	if (!material) {
		material = physics.createMaterial(kStaticFriction, kDynamicFriction, kRestitution);
	}

	rigidBody = physx::PxCreateDynamic(physics, transform, physx::PxCapsuleGeometry(physx::PxReal(DNA::RADIUS + DNA::SPHERE_RADIUS), length / 2 - physx::PxReal(DNA::RADIUS + DNA::SPHERE_RADIUS)), *material, kDensity, physx::PxTransform(physx::PxQuat(physx::PxReal(M_PI_2), physx::PxVec3(0, -1, 0))));

	const physx::PxReal radius(physx::PxReal(DNA::SPHERE_RADIUS * DNA::APPROXIMATION_RADIUS_MULTIPLIER));
	const physx::PxReal offset(physx::PxReal(DNA::RADIUS - radius + DNA::SPHERE_RADIUS));
	rigidBody->createShape(physx::PxSphereGeometry(radius), *material)->setLocalPose(physx::PxTransform(physx::PxVec3(-length / 2 + radius, 0, offset)));
	rigidBody->createShape(physx::PxSphereGeometry(radius), *material)->setLocalPose(physx::PxTransform(physx::PxQuat(physx::PxReal(toRadians(DNA::OPPOSITE_ROTATION)), physx::PxVec3(1, 0, 0)).rotate(physx::PxVec3(-length / 2 + radius, 0, offset))));

	rigidBody->createShape(physx::PxSphereGeometry(radius), *material)->setLocalPose(physx::PxTransform(physx::PxQuat(physx::PxReal(toRadians(DNA::PITCH * bases)), physx::PxVec3(1, 0, 0)).rotate(physx::PxVec3(physx::PxReal(bases * DNA::STEP) - length / 2 - radius, 0, offset))));
	rigidBody->createShape(physx::PxSphereGeometry(radius), *material)->setLocalPose(physx::PxTransform(physx::PxQuat(physx::PxReal(toRadians(DNA::PITCH * bases + DNA::OPPOSITE_ROTATION)), physx::PxVec3(1, 0, 0)).rotate(physx::PxVec3(physx::PxReal(bases * DNA::STEP) - length / 2 - radius, 0, offset))));

	scene.addActor(*rigidBody);
}

void Helix::attach(physx::PxPhysics & physics, Helix & other, AttachmentPoint thisPoint, AttachmentPoint otherPoint) {
	assert(rigidBody);

	detach(thisPoint);
	other.detach(otherPoint);

	physx::PxSphericalJoint *joint(physx::PxSphericalJointCreate(physics, rigidBody, physx::PxTransform(localFrame(thisPoint, bases)), other.rigidBody, physx::PxTransform(localFrame(otherPoint, bases))));
	joints[thisPoint].helix = &other;
	joints[thisPoint].joint = joint;
	other.joints[otherPoint].helix = this;
	other.joints[otherPoint].joint = joint;
}

bool Helix::detach(AttachmentPoint point) {
	if (joints[point].helix && joints[point].joint) {
		PRINT("DEBUG: Don't know if this is enough to delete a joint.");
		joints[point].joint->release(); // Is this enough to delete it?
		for (Helix::Connection & connection : joints[point].helix->joints) {
			if (connection.helix == this) {
				connection = Connection();
				return true;
			}
		}

		assert(0);
	}

	return false;
}

physx::PxReal Helix::getSeparation(AttachmentPoint atPoint) {
	const physx::PxVec3 attachmentPoint0(physx::PxMat44(rigidBody->getGlobalPose()).transform(localFrame(atPoint, bases)));
	const Helix & other(*joints[atPoint].helix);
	const physx::PxVec3 attachmentPoint1(physx::PxMat44(other.rigidBody->getGlobalPose()).transform(localFrame(otherPoint(atPoint, other), other.bases)));
	return (attachmentPoint1 - attachmentPoint0).magnitude();
}

Helix::AttachmentPoint Helix::otherPoint(Helix::AttachmentPoint point, const Helix & other) const {
	for (Helix::AttachmentPoint otherPoint : opposites(point)) {
		if (other.joints[otherPoint].helix == this)
			return otherPoint;
	}

	assert(0);
	return kNoAttachmentPoint;
}