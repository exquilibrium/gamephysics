#include "fbcSystem.h"
using namespace GamePhysics;

FBCSystem::~FBCSystem()
{
}

// TODO: Not sure if this works?!!!
void FBCSystem::SceneSetup(int sceneflag)
{
	m_rigidBodies.clear();
	m_points.clear();
	m_springs.clear();
	m_leapfrogFirst = true;
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

		if (sceneflag == 1) {
			update(0.01); // in the update, the extra force will be cleared

			m_gravity = Vec3(0, 0, 0);
			m_mass = 10.0f;
			m_stiffness = 40.0f;
			m_damping = 0.0f;

			int p0 = AddPoint(Vec3(0.0, 0.0f, 0), false);
			int p1 = AddPoint(Vec3(0.0, 2.0f, 0), false);
			SetPointVelocity(p0, Vec3(-1.0, 0.0f, 0));
			SetPointVelocity(p1, Vec3(1.0, 0.0f, 0));
			AddSpring(p0, p1, 1.0);
		}

		if (sceneflag == 0) {
			update(2.0f);
			Vec3 tmpfloat = m_rigidBodies.back().getCenter();
			std::cout << "new Position, x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			tmpfloat = m_rigidBodies.back().getVelocity();
			std::cout << "new Velocity, x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			tmpfloat = m_rigidBodies.back().getAngularV();
			std::cout << "new Angular V, x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			Vec3 xa_world = Vec3(-0.3f, -0.5f, -0.25f) - m_rigidBodies.back().getCenter();
			Vec3 velocityA = m_rigidBodies.back().getVelocity() + cross(m_rigidBodies.back().getAngularV(), xa_world);

			tmpfloat = velocityA;
			std::cout << "vel at P(-0.3, -0.5, -0.25), x, " << tmpfloat.x << ", y, " << tmpfloat.y << ", z, " << tmpfloat.z << "\n";

			testCheckCollision(1);
			testCheckCollision(2);
			testCheckCollision(3);

			m_gravity = Vec3(0, 0, 0);
			m_mass = 10.0f;
			m_stiffness = 40.0f;
			m_damping = 0.0f;

			int p0 = AddPoint(Vec3(0.0, 0.0f, 0), false);
			int p1 = AddPoint(Vec3(0.0, 2.0f, 0), false);
			SetPointVelocity(p0, Vec3(-1.0, 0.0f, 0));
			SetPointVelocity(p1, Vec3(1.0, 0.0f, 0));
			AddSpring(p0, p1, 1.0);


			cout << "Test case Begin with:\n";
			for (size_t i = 0; i < m_points.size(); i++)
				PrintPoint((int)i);

			AdvanceEuler(0.1f);

			cout << "After an Euler step:\n";
			for (size_t i = 0; i < m_points.size(); i++)
				PrintPoint(i);

			// reset
			SetPointVelocity(p0, Vec3(-1.0, 0.0f, 0));
			SetPointVelocity(p1, Vec3(1.0, 0.0f, 0));
			SetPointPosition(p0, Vec3(0.0, 0.0f, 0));
			SetPointPosition(p1, Vec3(0.0, 2.0f, 0));
			AdvanceLeapFrog(0.1f);
			cout << "After a LeapFrog step:\n";
			for (size_t i = 0; i < m_points.size(); i++)
				PrintPoint(i);

			// reset
			SetPointVelocity(p0, Vec3(-1.0, 0.0f, 0));
			SetPointVelocity(p1, Vec3(1.0, 0.0f, 0));
			SetPointPosition(p0, Vec3(0.0, 0.0f, 0));
			SetPointPosition(p1, Vec3(0.0, 2.0f, 0));
			AdvanceMidPoint(0.1f);
			cout << "After a midpoint step:\n";
			for (size_t i = 0; i < m_points.size(); i++)
				PrintPoint(i);
		}
	}
	else if (sceneflag == 2)
	{
		m_rigidBodies.emplace_back(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		m_rigidBodies.back().update(0.0f);
		m_rigidBodies.emplace_back(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		// Quat normal axis not used here
		m_rigidBodies.back().setRotation(Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
		m_rigidBodies.back().setVelocity(Vec3(0.0f, -0.1f, 0.05f));
		m_rigidBodies.back().update(0.0f);

		m_gravity = Vec3(0, 0, 0);
		int p1 = AddPoint(Vec3(0.25, 0.5, 0), true);
		int p2 = AddPoint(Vec3(0.25, 0.2, 0), false);
		int p3 = AddPoint(Vec3(0.45, 0.2, 0), false);
		AddSpring(p1, p2);
		AddSpring(p2, p3);

		int p4 = AddPoint(Vec3(-0.25f, 0.5f, 0), true);
		int p5 = AddPoint(Vec3(-0.25f, 0.2f, 0), false);
		AddSpring(p4, p5);

		{
			static const float  t = 1.618033988749894848205f; // (1 + sqrt(5)) / 2
			static const float t2 = 1.519544995837552493271f; // sqrt( 1 + sqr( (1 + sqrt(5)) / 2 ) )

			static const Vec3 verts[13] = {
				Vec3(t / t2, 1.f / t2, 0),
				Vec3(-t / t2, 1.f / t2, 0),
				Vec3(t / t2, -1.f / t2, 0),
				Vec3(-t / t2, -1.f / t2, 0),
				Vec3(1.f / t2, 0, t / t2),
				Vec3(1.f / t2, 0, -t / t2),
				Vec3(-1.f / t2, 0, t / t2),
				Vec3(-1.f / t2, 0, -t / t2),
				Vec3(0, t / t2, 1.f / t2),
				Vec3(0, -t / t2, 1.f / t2),
				Vec3(0, t / t2, -1.f / t2),
				Vec3(0, -t / t2, -1.f / t2),
				Vec3(0, 0, 0)
			};

			static const int faces[20 * 3] =
			{
				0, 8, 4,
				0, 5, 10,
				2, 4, 9,
				2, 11, 5,
				1, 6, 8,
				1, 10, 7,
				3, 9, 6,
				3, 7, 11,
				0, 10, 8,
				1, 8, 10,
				2, 9, 11,
				3, 11, 9,
				4, 2, 0,
				5, 0, 2,
				6, 1, 3,
				7, 3, 1,
				8, 6, 4,
				9, 4, 6,
				10, 5, 7,
				11, 7, 5
			};

			float size = 0.1f;

			int pi[13];
			for (int i = 0; i < 13; i++)
			{
				Vec3 pos = verts[i] * size;
				pi[i] = AddPoint(pos, false);
			}

			for (int i = 0; i < 12; i++)
			{
				AddSpring(pi[i], pi[12]);
			}

			for (int i = 0; i < 20; i++)
			{
				if (faces[i * 3 + 0] < faces[i * 3 + 1]) AddSpring(pi[faces[i * 3 + 0]], pi[faces[i * 3 + 1]]);
				if (faces[i * 3 + 1] < faces[i * 3 + 2]) AddSpring(pi[faces[i * 3 + 1]], pi[faces[i * 3 + 2]]);
				if (faces[i * 3 + 2] < faces[i * 3 + 0]) AddSpring(pi[faces[i * 3 + 2]], pi[faces[i * 3 + 0]]);
			}

			//int p6 = AddPoint(XMFLOAT3(0,0.5,0), true);
			//AddSpring(pi[0], p6, 0.25f);
		}

		if (m_cube > 1)
		{
			std::vector<int> pi;

			for (int z = 0; z < m_cube; z++)
				for (int y = 0; y < m_cube; y++)
					for (int x = 0; x < m_cube; x++)
					{
						float width = 0.4f;
						Vec3 pos(-width / 2 + (float)x / (m_cube - 1) * width,
							-0.5f + (float)y / (m_cube - 1) * width,
							-width / 2 + (float)z / (m_cube - 1) * width);
						pi.push_back(AddPoint(pos, false));
					}

			for (int z = 0; z < m_cube; z++)
				for (int y = 0; y < m_cube; y++)
					for (int x = 0; x < m_cube; x++)
					{
						if (x < m_cube - 1) AddSpring(pi[(z * m_cube + y) * m_cube + x], pi[(z * m_cube + y) * m_cube + (x + 1)]);
						if (y < m_cube - 1) AddSpring(pi[(z * m_cube + y) * m_cube + x], pi[(z * m_cube + (y + 1)) * m_cube + x]);
						if (z < m_cube - 1) AddSpring(pi[(z * m_cube + y) * m_cube + x], pi[((z + 1) * m_cube + y) * m_cube + x]);

						if (x < m_cube - 1 && y < m_cube - 1) AddSpring(pi[(z * m_cube + y) * m_cube + x], pi[(z * m_cube + (y + 1)) * m_cube + (x + 1)]);
						if (y < m_cube - 1 && z < m_cube - 1) AddSpring(pi[(z * m_cube + y) * m_cube + x], pi[((z + 1) * m_cube + (y + 1)) * m_cube + x]);
						if (x < m_cube - 1 && z < m_cube - 1) AddSpring(pi[(z * m_cube + y) * m_cube + x], pi[((z + 1) * m_cube + y) * m_cube + (x + 1)]);

						if (x < m_cube - 1 && y < m_cube - 1 && z < m_cube - 1) AddSpring(pi[(z * m_cube + y) * m_cube + x], pi[((z + 1) * m_cube + (y + 1)) * m_cube + (x + 1)]);
					}
		}
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

void FBCSystem::AdvanceEuler(float dt)
{
	m_leapfrogFirst = true;

	std::vector<Vec3> forces = ComputeForces();

	UpdatePositions(dt);

	UpdateVelocities(dt, forces);
}

void FBCSystem::AdvanceLeapFrog(float dt)
{
	std::vector<Vec3> forces = ComputeForces(); // a (t)

	if (m_leapfrogFirst) {
		UpdateVelocities(dt * 0.5, forces);
	}
	else {
		UpdateVelocities(dt, forces);
	}
	m_leapfrogFirst = false;

	UpdatePositions(dt);
}

void FBCSystem::AdvanceMidPoint(float dt)
{
	m_leapfrogFirst = true;

	std::vector<Point> ori_points = m_points;// save x_old, v_old

	std::vector<Vec3> forces = ComputeForces();// force = a( x_old, v_old), force_old
	UpdatePositions(dt / 2.0f); // x = x_tmp, using v_old
	UpdateVelocities(dt / 2.0f, forces); // v = v_tmp, using force_old

	forces = ComputeForces();// force = a ( x_tmp, v_tmp )

	for (size_t i = 0; i < m_points.size(); i++)//restore x_old
	{
		m_points[i].pos = ori_points[i].pos;
	}
	UpdatePositions(dt);// x = x ( vtmp )
	for (size_t i = 0; i < m_points.size(); i++)//restore v_old
	{
		m_points[i].vel = ori_points[i].vel;
	}
	UpdateVelocities(dt, forces);// v = v( a (xtmp, vtmp) )
}


std::vector<Vec3> FBCSystem::ComputeForces()
{
	// Gravity forces

	std::vector<Vec3> forces(m_points.size(), m_gravity * m_mass);

	for (size_t i = 0; i < m_springs.size(); i++)
	{
		// Spring forces
		int p1 = m_springs[i].point1;
		int p2 = m_springs[i].point2;
		Vec3 pos1 = m_points[p1].pos;
		Vec3 pos2 = m_points[p2].pos;

		Vec3 d = pos1 - pos2;
		float l = norm(d);
		float L = m_springs[i].initialLength;
		float k = m_stiffness;

		Vec3 f = d * (-k * (l - L) / l);

		forces[p1] += f;
		forces[p2] -= f;
	}

	// Damping forces
	for (size_t i = 0; i < m_points.size(); i++)
	{
		Vec3 vel = m_points[i].vel;

		forces[i] += vel * -m_damping;
	}
	return forces;
}

void FBCSystem::UpdatePositions(float dt)
{
	for (size_t i = 0; i < m_points.size(); i++)
	{
		if (!m_points[i].fixed)
		{
			Vec3 pos = m_points[i].pos;
			Vec3 vel = m_points[i].vel;

			pos += vel * dt;
			m_points[i].pos = pos;
		}
	}
}

void FBCSystem::UpdateVelocities(float dt, const std::vector<Vec3>& forces)
{
	for (size_t i = 0; i < m_points.size(); i++)
	{
		if (!m_points[i].fixed)
		{
			Vec3 vel = m_points[i].vel;
			float m = m_mass;

			vel += forces[i] * (dt / m);

			m_points[i].vel = vel;
		}
		else
		{
			m_points[i].vel = Vec3(0, 0, 0);
		}
	}
}

void FBCSystem::BoundingBoxCheck(float times)
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
}