#include "RigidBodySystemSimulator.h"
#include <unordered_set>

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	m_iTestCase = 0;
	collisions = set<pair<int, int>>();
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
	collisions.clear();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for(auto& r : bodies)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawRigidBody(r.scMat * r.r.getRotMat() * r.transMat);
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	Mat4 m;
	switch (m_iTestCase)
	{
	case 0:
		cout << "simple_single_body_simulation!\n";
		addRigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), 10);
		applyForceOnBody(0, Vec3(0.25, 0.25, 0), Vec3(100, -100, 0));
		/*addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		applyForceOnBody(0, Vec3(0.0, 0.0f, 0.0), Vec3(0, 0, 200));*/
		grav = false;
		break;
	case 1:
		cout << "Two-rigid-body_collision scene!\n";
		addRigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), 10);
		applyForceOnBody(0, Vec3(0.0, 0.0, 0), Vec3(0, -100, 0));
		addRigidBody(Vec3(0.0f, -0.2f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 10.0f);
		applyForceOnBody(1, Vec3(0.0, 0.0f, 0.0), Vec3(0, 100, 0));
		grav = false;
		break;
	case 2:
		cout << "Complex_simulation!\n";
		addRigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), 10);
		applyForceOnBody(0, Vec3(0.25, 0.25, 0), Vec3(-100, -100, 0));
		addRigidBody(Vec3(0.0f, -0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 10.0f);
		addRigidBody(Vec3(0.0f, -0.2f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 10.0f);
		//applyForceOnBody(1, Vec3(0.0, 0.0f, 0.0), Vec3(100, 100, 0));
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
		r.transMat *= t;

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

		r.r += ((timeStep / 2) * Quat(r.w.x, r.w.y, r.w.z,0)*(r.r));

		r.L += timeStep * q;

		Mat4 rt = Mat4(r.r.getRotMat());
		rt.transpose();
		Mat4 I = r.r.getRotMat() * r.I * rt;

		r.w = I * r.L;
	}

	for (int i = 0; i < bodies.size(); i++) {
		auto b1 = bodies[i];
		auto mat1 = b1.scMat * b1.r.getRotMat() * b1.transMat;
		auto mat1_inv = Mat4(mat1);
		mat1_inv.inverse();
		for (int j = i + 1; j < bodies.size(); j++) {
			auto b2 = bodies[j];
			auto mat2 = b2.scMat * b2.r.getRotMat() * b2.transMat;
			auto mat2_inv = Mat4(mat2);
			mat2_inv.inverse();

			auto ci = checkCollisionSAT(mat1, mat2);
			
			
			if (!ci.isValid) {
				collisions.erase(minmax(i, j));
				continue;
			}

			//if (collisions.count(minmax(i, j)))
				//continue;

			auto n = ci.normalWorld;
			n = getNormalized(n);
			auto cp_world = ci.collisionPointWorld;
			cout << cp_world << endl;
			auto cp1_local = mat1_inv.transformVector(cp_world);
			auto cp2_local = mat2_inv.transformVector(cp_world);

			auto v1_world = b1.vcm + cross(b1.w, cp1_local);
			auto v2_world = b2.vcm + cross(b2.w, cp2_local);
			auto v_rel =  v2_world - v1_world;

			if (dot(v_rel, n) > 0.5) {
				continue;
			}
			auto J_nom = -(1 + c) * v_rel * n;
			auto J_denom = 1.0 / b1.mass + 1.0 / b2.mass + (cross(b1.I * cross(cp1_local, n), cp1_local) + cross(b2.I * cross(cp2_local, n), cp2_local)) * n;
			auto J = J_nom / J_denom;

			bodies[j].vcm += J * n / b1.mass;
			bodies[j].L += cross(cp1_local, J * n);
			bodies[i].vcm -= J * n / b2.mass;
			bodies[i].L -= cross(cp2_local, J * n);

			collisions.insert(minmax(i, j));
		}
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
	bodies[i].transMat.decompose(res, Vec3(), Vec3(), Vec3());
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
	b.transMat.initTranslation(position.x, position.y, position.z);
	b.scMat.initScaling(size.x, size.y, size.z);
	/*b.transform.value[0][0] = size.x;
	b.transform.value[1][1] = size.y;
	b.transform.value[2][2] = size.z;*/
	b.mass = mass;

	b.I.value[0][0] = (1/12.0f) * b.mass *(size.y*size.y + size.z * size.z);
	b.I.value[1][1] = (1 / 12.0f) * b.mass * (size.x * size.x + size.z * size.z);
	b.I.value[2][2] = (1 / 12.0f) * b.mass * (size.x * size.x + size.y * size.y);
	b.I.value[3][3] = 1;

	b.I = b.I.inverse();

	Mat4 id = Mat4();
	id.initId();
	b.r = Quat(id);
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
