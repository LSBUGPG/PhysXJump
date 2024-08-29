// PhysXJump.cpp

#include <iostream>
#include <utility>
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

public:
	PxVec3 gravity;
	float dt;

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
		material = physics->createMaterial(0.5f, 0.5f, 0.6f);
	}

	void CreateSurface()
	{
		PxRigidStatic* ground = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);
		scene->addActor(*ground);
	}

	PxRigidDynamic* CreateBall(float radius, float mass, PxVec3 position, PxVec3 u)
	{
		PxSphereGeometry sphere(radius);
		ball = PxCreateDynamic(*physics, PxTransform(position), sphere, *material, 10.0f);
		ball->setAngularDamping(0.5f);
		ball->setMass(mass);
		ball->setLinearVelocity(u);
		scene->addActor(*ball);
		return ball;
	}

	void HitBall(PxVec3 v)
	{
		ball->addForce(v * ball->getMass(), PxForceMode::eIMPULSE);
	}

	void Update()
	{
		scene->simulate(dt);
		scene->fetchResults(true);
	}

	~PhysicsTest()
	{
		PX_RELEASE(scene);
		PX_RELEASE(physics);
		PX_RELEASE(foundation);
	}
};

float MechanicalEnergy(PxVec3 gravity, PxVec3 velocity, float height, float mass)
{
	float Pe = mass * gravity.magnitude() * height;
	float Ke = 0.5f * mass * velocity.magnitudeSquared();
	return Pe + Ke;
}

float GetHeightAboveBaseline(PxRigidDynamic* object, float baseline)
{
	return object->getGlobalPose().p.y - baseline;
}

float SimulateToFindPeak(PxRigidDynamic* ball, PhysicsTest& test, float& Me, PxVec3 gravity)
{
	float h = 0.0f;
	float peak = 0.0f;
	float baseline = GetHeightAboveBaseline(ball, 0.0f);
	PxVec3 v;
	do
	{
		if (gravity != PxVec3(0))
		{
			ball->addForce(gravity * ball->getMass(), PxForceMode::eFORCE);
		}
		test.Update();
		h = GetHeightAboveBaseline(ball, baseline);
		v = ball->getLinearVelocity();
		peak = std::max(h, peak);
	} while (v.y > 0.0f);
	Me = MechanicalEnergy((gravity != PxVec3(0))? gravity : test.gravity, v, peak, ball->getMass());
	return peak;
}

const char* GravityMethod(bool manual)
{
	return manual ? "manual" : "built in";
}

void ImpulseJumpHeightExperiment(bool manualGravity)
{
	std::cout << "Test impulse jump with " << GravityMethod(manualGravity) << " gravity..." << std::endl;
	float tennisBallRadius = 0.068f; // 6.8cm
	float tennisBallMass = 0.057f; // 57g
	float netHeight = 0.941f; // 94.1cm
	int fps = 50;
	float dt = 1.f / fps;
	PxVec3 gravity = PxVec3(0, -1, 0) * 9.81f;
	PhysicsTest test(dt, manualGravity ? PxVec3(0) : gravity);
	test.CreateSurface();
	PxVec3 u(0, sqrtf(-2.0f * -gravity.magnitude() * netHeight), 0);
	float Me = MechanicalEnergy(gravity, u, 0.0f, tennisBallMass);
	std::cout.precision(2);

	PxRigidDynamic* ball = test.CreateBall(tennisBallRadius, tennisBallMass, PxVec3(0, tennisBallRadius, 0), PxVec3(0));
	// wait for ball to sleep
	do
	{
		test.Update();
	}
	while (!ball->isSleeping());

	ball->addForce(u * tennisBallMass, PxForceMode::eIMPULSE);
	std::cout << "Impulse energy is " << Me << " J" << std::endl;

	float peak = SimulateToFindPeak(ball, test, Me, manualGravity ? gravity : PxVec3(0));
	std::cout << "Ball mechanical energy " << Me << " J at peak height of " << peak << " m" << std::endl;
	std::cout << std::endl;
}


void InitialVelocityJumpHeightExperiment(bool manualGravity)
{
	std::cout << "Test initial velocity jump with " << GravityMethod(manualGravity) << " gravity..." << std::endl;
	float tennisBallRadius = 0.068f; // 6.8cm
	float tennisBallMass = 0.057f; // 57g
	float netHeight = 0.941f; // 94.1cm
	int fps = 50;
	float dt = 1.f / fps;
	PxVec3 gravity = PxVec3(0, -1, 0) * 9.81f;
	PhysicsTest test(dt, manualGravity? PxVec3(0) : gravity);
	test.CreateSurface();
	PxVec3 u(0, sqrtf(-2.0f * -gravity.magnitude() * netHeight), 0);
	float Me = MechanicalEnergy(gravity, u, 0.0f, tennisBallMass);
	std::cout.precision(2);

	std::cout << "Initial energy is " << Me << " J" << std::endl;
	PxRigidDynamic* ball = test.CreateBall(tennisBallRadius, tennisBallMass, PxVec3(0, tennisBallRadius, 0), u);

	float peak = SimulateToFindPeak(ball, test, Me, manualGravity? gravity : PxVec3(0));
	std::cout << "Ball mechanical energy " << Me << " J at peak height of " << peak << " m" << std::endl;
	std::cout << std::endl;
}

PxVec3 VectorFrom(PxRigidDynamic* a, PxRigidDynamic* b)
{
	return a->getGlobalPose().p - b->getGlobalPose().p;
}

float MeasureAphelionDrift(PxRigidDynamic* Earth, PxRigidDynamic* Sun, PhysicsTest& test, float orbits)
{
	// r in AU
	float r = VectorFrom(Earth, Sun).magnitude();
	float aphelion = r;
	do
	{
		// G in AU^3 / (SolarMasses * year^2)
		float G = 39.478716f;
		float F = G * Sun->getMass() * Earth->getMass() / (r * r);
		Earth->addForce(F * VectorFrom(Earth, Sun));
		Sun->addForce(F * VectorFrom(Sun, Earth));
		test.Update();
		orbits -= test.dt;
		r = VectorFrom(Earth, Sun).magnitude();
		aphelion = std::max(aphelion, r);
	} while (orbits > 0);

	return aphelion;
}

void OrbitExperiment()
{
	std::cout << "Test orbit..." << std::endl;
	std::cout.precision(2);
	int fpy = 50;
	float dt = 1.f / fpy;
	PhysicsTest test(dt, PxVec3(0));
	// mass in solar masses
	// distances in AU
	// velocities in AU / year
	PxRigidDynamic* Earth = test.CreateBall(4.25875e-05f, 3.003353e-06f, PxVec3(0.9832924f, 0, 0), PxVec3(0, 0, 6.38966f));
	// The sun gets a small initial velocity to counter the Earth's initial momentum
	PxRigidDynamic* Sun = test.CreateBall(0.004650467f, 1.f, PxVec3(0, 0, 0), PxVec3(0, 0, -1.91904e-5f));

	int orbits = 100;
	float aphelion = 1.0167f;
	float aphelionError = std::fabsf(MeasureAphelionDrift(Earth, Sun, test, (float)orbits) - aphelion);
	std::cout << "Aphelion error is " << aphelionError << " AU after " << orbits << " orbits" << std::endl;
	std::cout << std::endl;
}

int main()
{
	bool manualGravity = true;
	InitialVelocityJumpHeightExperiment(!manualGravity);
	ImpulseJumpHeightExperiment(!manualGravity);
	InitialVelocityJumpHeightExperiment(manualGravity);
	ImpulseJumpHeightExperiment(manualGravity);
	OrbitExperiment();
	return 0;
}
