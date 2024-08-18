// PhysXJump.cpp

#include <iostream>
#include <PxPhysicsAPI.h>

using namespace physx;

class PhysicsTest
{
	PxDefaultAllocator allocator;
	PxDefaultErrorCallback errorCallback;
	PxFoundation* foundation;
	PxPhysics* physics;
	PxScene* scene;
	PxRigidDynamic* ball = NULL;
	PxMaterial* material = NULL;
	PxVec3 gravity;
	float dt;

public:
	PhysicsTest(float dt, PxVec3 gravity) : dt(dt), gravity(gravity)
	{
		foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
		physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true);
		PxSceneDesc sceneDesc(physics->getTolerancesScale());
		sceneDesc.gravity = gravity;
		PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(0);
		sceneDesc.cpuDispatcher = dispatcher;
		sceneDesc.filterShader = PxDefaultSimulationFilterShader;
		scene = physics->createScene(sceneDesc);
	}

	float MechanicalEnergy(PxVec3 velocity, float height, float mass)
	{
		float Pe = mass * gravity.magnitude() * height;
		float Ke = 0.5f * mass * velocity.magnitudeSquared();
		return Pe + Ke;
	}

	void CreateSurface()
	{
		material = physics->createMaterial(0.5f, 0.5f, 0.6f);
		PxRigidStatic* ground = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);
		scene->addActor(*ground);
	}

	void CreateBall(float radius, float mass, PxVec3 u)
	{
		PxSphereGeometry sphere(radius);
		ball = PxCreateDynamic(*physics, PxTransform(PxVec3(0, radius, 10)), sphere, *material, 10.0f);
		ball->setAngularDamping(0.5f);
		ball->setMass(mass);
		ball->setLinearVelocity(u);
		scene->addActor(*ball);
	}

	void HitBall(PxVec3 v)
	{
		ball->addForce(v * ball->getMass(), PxForceMode::eIMPULSE);
	}

	float SimulateToFindPeak()
	{
		float h = 0.0f;
		float peak = 0.0f;
		float baseline = ball->getGlobalPose().p.y;
		PxVec3 v = ball->getLinearVelocity();
		do
		{
			scene->simulate(dt);
			scene->fetchResults(true);
			h = ball->getGlobalPose().p.y - baseline;
			v = ball->getLinearVelocity();
			peak = std::max(h, peak);
		}
		while (v.y > 0.0f);
		return peak;
	}

	~PhysicsTest()
	{
		PX_RELEASE(scene);
		PX_RELEASE(physics);
		PX_RELEASE(foundation);
	}
};


void Experiment(bool correctForLag, bool impulse)
{
	float tennisBallRadius = 0.068f; // 6.8cm
	float tennisBallMass = 0.057f; // 57g
	float netHeight = 0.941f; // 94.1cm
	int fps = 50;
	float dt = 1.f / fps;
	PxVec3 gravity = PxVec3(0, -1, 0) * 9.81f;
	PhysicsTest test(dt, gravity);
	test.CreateSurface();
	PxVec3 u(0, sqrtf(-2.0f * gravity.y * netHeight), 0);
	float Me = test.MechanicalEnergy(u, 0.0f, tennisBallMass);
	std::cout.precision(2);

	PxVec3 Eu(0);
	if (correctForLag)
	{
		// calculate the required adjustment to the velocity to get it
		// to what it would have needed to be, half a time step ago,
		// so that it is correct velocity when applied by updateForces()
		Eu = -0.5f * gravity * dt;
	}

	if (impulse)
	{
		test.CreateBall(tennisBallRadius, tennisBallMass, PxVec3(0));
		test.HitBall(u + Eu);
		std::cout << "Impulse energy is " << 0.5f * u.magnitudeSquared() * tennisBallMass << " J" << std::endl;
	}
	else
	{
		std::cout << "Initial energy is " << 0.5f * u.magnitudeSquared() * tennisBallMass << " J" << std::endl;
		test.CreateBall(tennisBallRadius, tennisBallMass, u + Eu);
	}

	float peak = test.SimulateToFindPeak();
	Me = test.MechanicalEnergy(PxVec3(0), peak, tennisBallMass);
	std::cout << "Ball mechanical energy " << Me << " J at peak height of " << peak << " m" << std::endl;
	std::cout << std::endl;
}

int main()
{
	bool correctForLag = true;
	bool impulse = true;
	std::cout << "Test with initial velocity..." << std::endl;
	Experiment(!correctForLag, !impulse);
	std::cout << "Test with impulse..." << std::endl;
	Experiment(!correctForLag, impulse);
	std::cout << "Apply correction to initial velocity..." << std::endl;
	Experiment(correctForLag, !impulse);
	std::cout << "Apply correction to impulse..." << std::endl;
	Experiment(correctForLag, impulse);
	return 0;
}
