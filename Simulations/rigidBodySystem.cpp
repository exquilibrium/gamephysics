#include "rigidBodySystem.hpp"
using namespace GamePhysics;
RigidBodySystem::RigidBodySystem() :
	m_rigidBodies()
{
}

RigidBodySystem::~RigidBodySystem()
{
}

void RigidBodySystem::SceneSetup(int sceneflag)
{
	m_rigidBodies.clear();
	if (sceneflag == 0 || sceneflag == 1)
	{// basic test
		m_rigidBodies.emplace_back(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		Quat rotation(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f);
		m_rigidBodies.back().setRotation(rotation);
		//m_rigidBodies.back().setVelocity(XMVectorSet(-0.3f, -0.5f, -0.25f, 0.0f));
		m_rigidBodies.back().update(0.0f);// set up obj to World and World to obj Matrix!
		
		Vec3 force = Vec3(1.0f, 1.0f, 0.0f);
		Vec3 fwhere = Vec3(0.3f, 0.5f, 0.25f);
		
		m_rigidBodies.back().addForceWorld(force, fwhere);

		if (sceneflag == 1){
			update(0.01); // in the update, the extra force will be cleared
		}
		
		if (sceneflag == 0){
			update(2.0f);
			Vec3 tmpfloat =  m_rigidBodies.back().getCenter();
			std::cout << "new Position, x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			tmpfloat =  m_rigidBodies.back().getVelocity();
			std::cout << "new Velocity, x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			tmpfloat =  m_rigidBodies.back().getAngularV();
			std::cout << "new Angular V, x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			Vec3 xa_world = Vec3(-0.3f, -0.5f, -0.25f) - m_rigidBodies.back().getCenter();
			Vec3 velocityA = m_rigidBodies.back().getVelocity() + cross(m_rigidBodies.back().getAngularV(), xa_world);

			tmpfloat =  velocityA;
			std::cout << "vel at P(-0.3, -0.5, -0.25), x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			testCheckCollision(1);
			testCheckCollision(2);
			testCheckCollision(3);
		}
	}
	else if (sceneflag == 2)
	{
		m_rigidBodies.emplace_back(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		m_rigidBodies.back().update(0.0f);
		m_rigidBodies.emplace_back(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		// Quat normal axis not used here
		m_rigidBodies.back().setRotation(Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI)*0.25f));
		m_rigidBodies.back().setVelocity(Vec3(0.0f, -0.1f, 0.05f));
		m_rigidBodies.back().update(0.0f);
	}
}

void RigidBodySystem::dragTogether()
{
	for (int i = 0 ; i < m_rigidBodies.size() - 1; ++i)
	{
		Vec3 vel = m_rigidBodies[i+1].getCenter() - m_rigidBodies[i].getCenter();
		m_rigidBodies[i].setVelocity(vel * 0.1f);
		m_rigidBodies[i+1].setVelocity(vel * -0.1f);
	}
}

void RigidBodySystem::addGlobalFrameForce(const Vec3 force)
{
	for (RigidBody& rigidBody : m_rigidBodies)
	{
		rigidBody.addForceWorld(force, XMVectorZero());
	}
}

void RigidBodySystem::update(float deltaTime)
{

	for (RigidBody& rigidBody : m_rigidBodies)
	{
		rigidBody.update(deltaTime);
	}
	for (size_t i = 0; i < m_rigidBodies.size(); ++i)
	{
		for (size_t j = i + 1; j < m_rigidBodies.size(); ++j)
		{
			if (RigidBody::collide(m_rigidBodies[i], m_rigidBodies[j]))
			{
			}
		}
	}
}