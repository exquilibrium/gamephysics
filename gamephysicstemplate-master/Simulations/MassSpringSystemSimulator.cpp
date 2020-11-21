#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator(){
	m_iTestCase = 0;
	
	initial_length = 1.0f;

	m_fDamping = 0;
	m_fMass = 10;
	m_fStiffness = 40;

	m_iIntegrator = MIDPOINT;

	m_fSphereSize = 0.05f;

	gravity = false;
}

void MassSpringSystemSimulator::addTwoSpringSetup() {
	reset();
	points.clear();
	springs.clear();
	points.push_back(Point(Vec3(0, 0, 0), Vec3(-1, 0, 0), 10, 0, false));
	points.push_back(Point(Vec3(0, 2, 0), Vec3(1, 0, 0), 10, 0, false));
	springs.push_back(Spring(&points[0], &points[1], 40, 1));
}

void MassSpringSystemSimulator::addSixSpringSetup() {
	reset();
	points.clear();
	springs.clear();

	addMassPoint(Vec3(0.5, 1.5, 0), Vec3(0, 0, 0), true);
	addMassPoint(Vec3(0.5, 0.85, 0), Vec3(0, 0, 0), true);

	addMassPoint(Vec3(0, 1.2, 0.5), Vec3(-0.5, 0.5, 0.5), false);
	addMassPoint(Vec3(0, 1.2, -0.5), Vec3(-0.5, 0.5, -0.5), false);
	addMassPoint(Vec3(1, 1.2, -0.5), Vec3(0.5, 0.5, -0.5), false);
	addMassPoint(Vec3(1, 1.2, 0.5), Vec3(0.5, 0.5, 0.5), false);

	addSpring(0, 2, 0.046);
	addSpring(0, 3, 0.046);
	addSpring(0, 4, 0.046);
	addSpring(0, 5, 0.046);

	addSpring(1, 2, 0.08);
	addSpring(1, 3, 0.08);
	addSpring(1, 4, 0.08);
	addSpring(1, 5, 0.08);

	addSpring(2, 3, 1.2);
	addSpring(3, 4, 1.2);
	addSpring(4, 5, 1.2);
	addSpring(5, 2, 1.2);
}

void MassSpringSystemSimulator::addTenSpringSetup() {
	reset();
	points.clear();
	springs.clear();

	addMassPoint(Vec3(0.5,1.5,0), Vec3(0,0,0), true);
	addMassPoint(Vec3(0.5, 0.85, 0), Vec3(0, 0, 0), true);

	addMassPoint(Vec3(0, 1.2, 0.5), Vec3(-0.5, 0.5, 0.5), false);
	addMassPoint(Vec3(0, 1.2, -0.5), Vec3(-0.5, 0.5, -0.5), false);
	addMassPoint(Vec3(1, 1.2, -0.5), Vec3(0.5, 0.5, -0.5), false);
	addMassPoint(Vec3(1, 1.2, 0.5), Vec3(0.5, 0.5, 0.5), false);

	addMassPoint(Vec3(0, 0.5, 0.5), Vec3(-0.5, -0.5, 0.5), false);
	addMassPoint(Vec3(0, 0.5, -0.5), Vec3(-0.5, -0.5, -0.5), false);
	addMassPoint(Vec3(1, 0.5, -0.5), Vec3(0.5, -0.5, -0.5), false);
	addMassPoint(Vec3(1, 0.5, 0.5), Vec3(0.5, -0.5, 0.5), false);

	addSpring(0, 2, 0.046);
	addSpring(0, 3, 0.046);
	addSpring(0, 4, 0.046);
	addSpring(0, 5, 0.046);

	addSpring(1, 2, 0.08);
	addSpring(1, 3, 0.08);
	addSpring(1, 4, 0.08);
	addSpring(1, 5, 0.08);
	addSpring(1, 6, 0.08);
	addSpring(1, 7, 0.08);
	addSpring(1, 8, 0.08);
	addSpring(1, 9, 0.08);

	addSpring(2, 3, 1.2);
	addSpring(3, 4, 1.2);
	addSpring(4, 5, 1.2);
	addSpring(5, 2, 1.2);
	addSpring(6, 7, 1.2);
	addSpring(7, 8, 1.2);
	addSpring(8, 9, 1.2);
	addSpring(6, 9, 1.2);

	addSpring(2, 6, 1.2);
	addSpring(3, 7, 1.2);
	addSpring(4, 8, 1.2);
	addSpring(5, 9, 1.2);
}


const char* MassSpringSystemSimulator::getTestCasesStr() {
	return " Euler 2-point mass-spring-setup, Euler 10-point mass-spring-setup, Midpoint 2-point mass-spring-setup, Midpoint 10-point mass-spring-setup, Euler6, Midpoint6";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Use gravity", TW_TYPE_BOOLCPP, &gravity, "");
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	switch (testCase)
	{
	case 0:
		m_iIntegrator = EULER;
		addTwoSpringSetup();
		break;
	case 1:
		m_iIntegrator = EULER;
		addTenSpringSetup();
		break;
	case 2:
		m_iIntegrator = MIDPOINT;
		addTwoSpringSetup();
		break;
	case 3:
		m_iIntegrator = MIDPOINT;
		addTenSpringSetup();
		break;

	case 4:
		m_iIntegrator = EULER;
		addSixSpringSetup();
		break;
	case 5:
		m_iIntegrator = MIDPOINT;
		addSixSpringSetup();
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	if (points.size() < 1)
		return;
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for (int i = 0; i < points.size(); i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(points[i].pos, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
	}

	if (springs.size() < 1)
		return;

	DUC->beginLine();
	for (Spring s : springs) {
		DUC->drawLine(s.p1->pos, Vec3(1,1,1), s.p2->pos, Vec3(1,1,1));
	}
	DUC->endLine();
}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	m_externalForce = Vec3(0, 0, 0);
}
void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	applyExternalForce(m_externalForce);
	for (auto &s : springs) {
		float l = sqrt(s.p1->pos.squaredDistanceTo(s.p2->pos));
		Vec3 force = -s.stiffnes * (l - s.initLength) * ((s.p1->pos - s.p2->pos) / l);
		s.p1->force += force;
		s.p2->force -= force;
  	}

	switch (m_iIntegrator) {
		case EULER:
			eulerIntegrate(timeStep); break;

		case MIDPOINT:
			midpointIntegrate(timeStep); break;
	}
}
void MassSpringSystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Specific Functions
void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}
void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}
void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	points.push_back(Point(position, Velocity,m_fMass, m_fDamping, isFixed));
	return points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	springs.push_back(Spring(&points[masspoint1], &points[masspoint2], m_fStiffness, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return points[index].pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return points[index].vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	for (auto& p : points) {
		if (gravity) {
			force += Vec3(0, -9.81 * p.mass, 0);
		}

		p.clearForce();
		p.force += force;
	}
}

void MassSpringSystemSimulator::eulerIntegrate(float timeStep) {
	int i = 0;
	for (auto &p : points) {
		if (p.fixed) {
			continue;
		}
		Vec3 a = p.force / p.mass;
		Vec3 v = p.vel + timeStep * a;
		p.pos = p.pos + timeStep * p.vel;
		if (p.pos.y < -1) {
			p.pos.y = -1;
		}
		p.vel = v;

		cout << "EULER Point " + to_string(i);
		cout << ": Position = " + p.pos.toString() + ", Velocity = " + p.vel.toString() + "\n";
		i++;
	}
}

void MassSpringSystemSimulator::midpointIntegrate(float timeStep) {
	for (auto &p : points) {
		if (p.fixed) {
			continue;
		}
		Vec3 a0 = p.force / p.mass;
		p.tempPos = p.pos + (timeStep/2) * p.vel;
		p.tempVel = p.vel + (timeStep/2) * a0;

		p.pos = p.pos + timeStep * p.tempVel;

		if (p.pos.y < -1) {
			p.pos.y = -1;
		}
	}

	// applyExternalForce(m_externalForce);

	for (auto &s : springs) { 
		float l = sqrt(s.p1->pos.squaredDistanceTo(s.p2->pos));
		Vec3 force = -s.stiffnes * (l - s.initLength) * ((s.p1->pos - s.p2->pos) / l);
		s.p1->force += force;
		s.p2->force -= force;
	}

	int i = 0;
	for (auto &p : points) {
		p.vel = p.vel + timeStep * (p.force / p.mass);

		cout << "MIDPOINT Point " + to_string(i);
		cout << ": Position = " + p.pos.toString() + ", Velocity = " + p.vel.toString() + "\n";
		i++;
	}
}



