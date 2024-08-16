// PhysXJump.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <PxPhysicsAPI.h>

using namespace physx;

int main()
{
	PxDefaultAllocator allocator;
	PxDefaultErrorCallback errorCallback;
	PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
	PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true);
	PxSceneDesc sceneDesc(physics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(0);
	sceneDesc.cpuDispatcher = dispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	PxScene* scene = physics->createScene(sceneDesc);
	PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.9f);
	
	PxRigidStatic* ground = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);
	scene->addActor(*ground);

	float radius = 0.5f;
	PxSphereGeometry sphere(radius);
	PxRigidDynamic* ball = PxCreateDynamic(*physics, PxTransform(PxVec3(0, radius, 10)), sphere, *material, 10.0f);
	ball->setAngularDamping(0.5f);
	ball->setLinearVelocity(PxVec3(0, 0, 0));
	scene->addActor(*ball);

	float dt = 1.0f / 15.0f;
	float target = 1.0f;
	PxVec3 v(0, sqrtf(-2.0f * sceneDesc.gravity.y * target), 0);
	v += -0.5f * sceneDesc.gravity * dt;
	ball->addForce(v, PxForceMode::eVELOCITY_CHANGE);
	float h = 0.0f;
	float peak = 0.0f;
	
	do
	{
		scene->simulate(dt);
		scene->fetchResults(true);
		h = ball->getGlobalPose().p.y - radius;
		peak = std::max(h, peak);
	}
	while (ball->getLinearVelocity().y >= 0.0f);

	std::cout.precision(2);
	std::cout << "The ball reached a height of " << peak << " meters" << std::endl;

	PX_RELEASE(scene);
	PX_RELEASE(physics);
	PX_RELEASE(foundation);
}
