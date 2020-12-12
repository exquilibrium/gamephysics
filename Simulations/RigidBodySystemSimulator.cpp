#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
}

const char * RigidBodySystemSimulator::getTestCasesStr() {
	return " simple_single_body_simulation,Two-rigid-body_collision scene,Complex_simulation";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:
		break;
	case 2:break;
	default:break;
	}
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	bodies.clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for(auto& r : bodies)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawRigidBody(r.transform*r.r.getRotMat());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	switch (m_iTestCase)
	{
	case 0:
		cout << "simple_single_body_simulation!\n";
		addRigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), 10);
		applyForceOnBody(0, Vec3(0.25, 0.25, 0), Vec3(0, -10000, 0));
		setOrientationOf(0,Quat(1,0,0));
		/*addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		applyForceOnBody(0, Vec3(0.0, 0.0f, 0.0), Vec3(0, 0, 200));*/
		grav = false;
		break;
	case 1:
		cout << "Two-rigid-body_collision scene!\n";
		break;
	case 2:
		cout << "Complex_simulation!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	for(auto& r : bodies)
	{
		if (grav) {
			r.F.push_back(pair<Vec3, Vec3>(Vec3(0, 0, 0), Vec3(0, -9.81, 0)));
		}
		m_externalForce = Vec3();
		for(auto& f : r.F)
		{
			m_externalForce += f.second;
		}
		Vec3 translate = timeElapsed * r.vcm;
		Mat4 t = Mat4();
		t.initTranslation(translate.x, translate.y, translate.z);
		r.transform *= t;

		r.vcm += timeElapsed * (m_externalForce / r.mass);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	externalForcesCalculations(timeStep);

	for(auto& r : bodies)
	{
		Vec3 q = Vec3();
		for(auto& f : r.F)
		{
			q += cross( f.first, f.second);
		}
		r.F.clear();

		r.r += ((timeStep / 2) * Quat(0, r.w.x, r.w.y, r.w.z)*(r.r));

		r.L += timeStep * q;

		Mat4 rt = Mat4(r.r.getRotMat());
		rt.transpose();
		Mat4 I = r.r.getRotMat() * r.I * rt;

		r.w = I * r.L;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	Vec3 res;
	bodies[i].transform.decompose(res, Vec3(), Vec3(), Vec3());
	return res;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return bodies[i].vcm;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return bodies[i].w;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	bodies[i].F.push_back(pair<Vec3, Vec3>(loc,force));
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	Rigidbody b = {};
	b.transform.initTranslation(position.x, position.y, position.z);
	b.transform.value[0][0] = size.x;
	b.transform.value[1][1] = size.y;
	b.transform.value[2][2] = size.z;
	b.mass = mass;


	b.I.value[0][0] = (1/12.0f) * b.mass *(size.y*size.y + size.z * size.z);
	b.I.value[1][1] = (1 / 12.0f) * b.mass * (size.x * size.x + size.z * size.z);
	b.I.value[2][2] = (1 / 12.0f) * b.mass * (size.x * size.x + size.y * size.y);
	b.I.value[3][3] = 1;

	b.I.inverse();

	bodies.push_back(b);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	bodies[i].r = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	bodies[i].vcm = velocity;
}
