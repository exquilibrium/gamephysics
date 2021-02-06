#include "fbcSystem.h"
using namespace GamePhysics;

FBCSystem::~FBCSystem()
{
}

// TODO: Not sure if this works?!!!
void FBCSystem::SceneSetup(int sceneflag)
{
	m_rigidBodies.clear();
	m_springs.clear();
	m_stiffness = 40.0f;
	m_damping = 0.0f;
	if (sceneflag == 0)
	{// basic test
		m_rigidBodies.emplace_back(Vec3(0.0f, 0.5f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 1.0f);
		m_rigidBodies.emplace_back(Vec3(0.0f, -0.5f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 1.0f);
		m_rigidBodies.emplace_back(Vec3(0.5f, 0.5f, 0.0f), Vec3(0.25f, 0.25f, 0.25f), 1.0f);

		m_rigidBodies.emplace_back(Vec3(0.0f, -0.75f, 0.0f), Vec3(10000, 0.05f, 10000), 10000);
		m_rigidBodies[3].update(0);
		m_rigidBodies[3].fixed = true;
		AddSpring(0,1);

		m_rigidBodies[0].addForceWorld(Vec3(0,200,0), Vec3(0,0.5,0));
		m_rigidBodies[1].addForceWorld(Vec3(0, -200, 0), Vec3(0, -0.5, 0));
		m_rigidBodies[2].addForce(Vec3(-200,0, 0), Vec3(0, 0, 0));
	}
}

void FBCSystem::dragTogether()
{
	for (int i = 0; i < m_rigidBodies.size() - 1; ++i)
	{
		Vec3 vel = m_rigidBodies[i + 1].getCenter() - m_rigidBodies[i].getCenter();
		m_rigidBodies[i].setVelocity(vel * 0.1f);
		m_rigidBodies[i + 1].setVelocity(vel * -0.1f);
	}
}

void FBCSystem::addGlobalFrameForce(const Vec3 force)
{
	for (RigidBody& rigidBody : m_rigidBodies)
	{
		rigidBody.addForceWorld(force, XMVectorZero());
	}
}

void FBCSystem::update(float deltaTime)
{

	for (RigidBody& rigidBody : m_rigidBodies)
	{
		if (!rigidBody.fixed) {
			rigidBody.update(deltaTime);
		}
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

void FBCSystem::ComputeSpringForces()
{
	for (size_t i = 0; i < m_springs.size(); i++)
	{
		// Spring forces
		RigidBody* body1 = m_springs[i].body1;
		RigidBody* body2 = m_springs[i].body2;
		Vec3 pos1 = body1->getCenter();
		Vec3 pos2 = body2->getCenter();

		Vec3 d = pos1 - pos2;
		float l = norm(d);
		float L = m_springs[i].initialLength;
		float k = m_stiffness;

		Vec3 f = d * (-k * (l - L) / l);

		body1->addForceWorld(f + (body1->getVelocity() * -m_damping), pos1);
		body2->addForceWorld((-f) + (body2->getVelocity() * -m_damping), pos1);
	}
}

/*void FBCSystem::BoundingBoxCheck(float times)
{
	for (size_t i = 0; i < m_points.size(); i++)
	{
		if (!m_points[i].fixed)
		{
			Vec3 pos = m_points[i].pos;
			Vec3 vel = m_points[i].vel;

			for (int f = 0; f < 6; f++)
			{
				float sign = (f % 2 == 0) ? -1.0f : 1.0f;
				if (sign * pos.value[f / 2] < -0.5f * times)
				{
					pos.value[f / 2] = sign * -0.5f * times;
					vel.value[f / 2] = 0;
				}
			}

			m_points[i].pos = pos;
			m_points[i].vel = vel;
		}
	}
}*/