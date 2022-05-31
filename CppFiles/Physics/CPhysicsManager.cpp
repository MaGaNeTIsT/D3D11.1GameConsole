#include "../Headers/Physics/CPhysicsManager.h"


void CPhysicsManager::Init()
{
	using namespace PhysicsManager;

	Trace =TraceImpl;

	// Create a factory
	Factory::sInstance = new Factory();

	// Register all Jolt physics types
	RegisterTypes();

	//pre-allocated memory for simulation.
	TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
	m_TempAllocator = &temp_allocator;

	//an example implementation for jobsystem
	//used for multiple threads
	JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);
	m_JobSystem = &job_system;

	const uint cMaxBodies				= 65535;	//Max amount of rigid bodies.
	const uint cNumBodyMutexes			= 0;		//Mutexes count,0 to default setting.
	const uint cMaxBodyPairs			= 65535;	//Max amount of body pairs that can be queued at any time.
	const uint cMaxContactConstraints	= 65535;	//This is the maximum size of the contact constraint buffer.

	// Create mapping table from object layer to broadphase layer
	CBPLayerInterfaceImpl broad_phase_layer_interface;

	PhysicsSystem physics_system;
	physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface, BroadPhaseCanCollide, ObjectCanCollide);

	CBodyActivationListener body_activation_listener;
	physics_system.SetBodyActivationListener(&body_activation_listener);

	CContactListener contact_listener;
	physics_system.SetContactListener(&contact_listener);

	m_BodyInterface = &physics_system.GetBodyInterface();
	m_PhysicsSystem = &physics_system;
}

void CPhysicsManager::Tick(const float cDeltaTime)
{
	// Do n collision step per cDeltaTime
	const int cCollisionSteps = 1;
	// If you want more accurate step results you can do multiple sub steps within a collision step. Usually you would set this to 1.
	const int cIntegrationSubSteps = 1;

	m_PhysicsSystem->Update(cDeltaTime, cCollisionSteps, cIntegrationSubSteps, m_TempAllocator, m_JobSystem);
}

CPhysicsManager::~CPhysicsManager()
{
	//TODO:Remove BodyIDs;
	
	delete m_PhysicsSystem;
	m_PhysicsSystem = nullptr;
	delete m_BodyInterface;
	m_BodyInterface = nullptr;
	delete m_TempAllocator;
	m_TempAllocator = nullptr;
	delete m_JobSystem;
	m_JobSystem = nullptr;

	// Destroy the factory
	delete Factory::sInstance;
	Factory::sInstance = nullptr;

}