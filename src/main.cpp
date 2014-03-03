#include <DNA.h>
#include <Utility.h>

#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#ifdef _WINDOWS
#include <windows.h>
#else
#include <sys/utime.h>
#endif /* N _WINDOWS */

#include <PxPhysicsAPI.h>

#define PVD_HOST "127.0.0.1"

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics = NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene = NULL;

PxMaterial*				gMaterial = NULL;

PxVisualDebuggerConnection*gConnection = NULL;

volatile bool running = true;

inline void sleepms(unsigned int ms) {
#ifdef _WINDOWS
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif /* N _WINDOWS */
}

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 1.0f);
	//dynamic->setAngularDamping(0.5f);
	//dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

PxRigidDynamic *createHelix(const PxTransform & t, int bases) {
	PxReal length(bases * DNA::STEP);
	assert(length > DNA::RADIUS * 2);
	PxRigidDynamic *dynamic = PxCreateDynamic(*gPhysics, t, PxCapsuleGeometry(PxReal(DNA::RADIUS + DNA::SPHERE_RADIUS), length / 2 - PxReal(DNA::RADIUS + DNA::SPHERE_RADIUS)), *gMaterial, 10.0f);
	/*for (int i = 0; i < bases; ++i) {
		dynamic->createShape(PxSphereGeometry(DNA::SPHERE_RADIUS), *gMaterial)->setLocalPose(PxTransform(PxQuat(toRadians(DNA::PITCH * i), PxVec3(1, 0, 0)).rotate(PxVec3(PxReal(i * DNA::STEP) - length / 2, 0, DNA::RADIUS))));
	}*/

	const PxReal radius(DNA::SPHERE_RADIUS * 4);
	const PxReal offset(DNA::RADIUS - radius + DNA::SPHERE_RADIUS);
	dynamic->createShape(PxSphereGeometry(radius), *gMaterial)->setLocalPose(PxTransform(PxVec3(-length / 2 + radius, 0, offset)));
	dynamic->createShape(PxSphereGeometry(radius), *gMaterial)->setLocalPose(PxTransform(PxQuat(toRadians(DNA::OPPOSITE_ROTATION), PxVec3(1, 0, 0)).rotate(PxVec3(-length / 2 + radius, 0, offset))));

	dynamic->createShape(PxSphereGeometry(radius), *gMaterial)->setLocalPose(PxTransform(PxQuat(toRadians(DNA::PITCH * bases), PxVec3(1, 0, 0)).rotate(PxVec3(PxReal(bases * DNA::STEP) - length / 2 - radius, 0, offset))));
	dynamic->createShape(PxSphereGeometry(radius), *gMaterial)->setLocalPose(PxTransform(PxQuat(toRadians(DNA::PITCH * bases + DNA::OPPOSITE_ROTATION), PxVec3(1, 0, 0)).rotate(PxVec3(PxReal(bases * DNA::STEP) - length / 2 - radius, 0, offset))));

	gScene->addActor(*dynamic);
	return dynamic; // TODO add shapes.
}

#ifdef _WINDOWS
BOOL WINAPI HandlerRoutine(_In_  DWORD dwCtrlType) {
	if (dwCtrlType == CTRL_C_EVENT) {
		std::cerr << "Aborting simulation" << std::endl;
		running = false;
	}
	return TRUE;
}
#else
void handle_exit(int s) {
	running = false;
}
#endif /* N _WINDOWS */


int main(int argc, const char **argv) {
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = &PxProfileZoneManager::createProfileZoneManager(gFoundation);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, profileZoneManager);

	if (gPhysics->getPvdConnectionManager())
	{
		gPhysics->getVisualDebugger()->setVisualizeConstraints(true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_SCENEQUERIES, true);
		gConnection = PxVisualDebuggerExt::createConnection(gPhysics->getPvdConnectionManager(), PVD_HOST, 5425, PxVisualDebuggerConnectionFlag::eDEBUG);
	}

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	std::vector<PxRigidDynamic *> balls;
	for (int i = 0; i < 10; ++i) {
		//PxRigidDynamic *ball(createDynamic(PxTransform(PxVec3(0, PxReal(2 * i + 1), 0)), PxSphereGeometry(1)));
		//PxRigidDynamic *ball(createDynamic(PxTransform(PxVec3(0, PxReal(2 * i + 1), PxReal(i) * PxReal(0.2))), PxCapsuleGeometry(1, 2)));
		PxRigidDynamic *ball(createHelix(PxTransform(PxVec3(0, PxReal((DNA::RADIUS + DNA::SPHERE_RADIUS) * 2 * i + 1), PxReal(i) * PxReal(0.2))), 21));
		balls.push_back(ball);
	}

	std::vector<PxRigidDynamic *>::iterator it(balls.begin());
	std::vector<PxRigidDynamic *>::iterator prev_it(it++);
	for (; it != balls.end(); ++it, ++prev_it) {
		//PxSphericalJointCreate(*gPhysics, *prev_it, PxTransform(1, DNA::RADIUS, 0), *it, PxTransform(1, -DNA::RADIUS, 0));
		//PxSphericalJointCreate(*gPhysics, *prev_it, PxTransform(-1, DNA::RADIUS, 0), *it, PxTransform(-1, -DNA::RADIUS, 0));
	}

	{
#ifdef _WINDOWS
		SetConsoleCtrlHandler(HandlerRoutine, TRUE);
#else
		struct sigaction sigint_handler;
		sigint_handler.sa_handler = handle_exit;
		sigemptyset(&sigint_handler.sa_mask);
		sigint_handler.sa_flags = 0;

		sigaction(SIGINT, &sigint_handler, NULL);
#endif
	}

	std::cerr << "Running simulation" << std::endl;
	while (running) {
		gScene->simulate(1.0f / 60.0f);
		gScene->fetchResults(true);
		//std::cerr << "Ball transform: " << ball->getGlobalPose().p.x << ", " << ball->getGlobalPose().p.y << ", " << ball->getGlobalPose().p.z << std::endl;
		sleepms(16);
	}

	gScene->release();
	gDispatcher->release();
	if (gConnection != NULL)
		gConnection->release();
	gPhysics->release();
	profileZoneManager->release();
	gFoundation->release();

	return 0;
}
