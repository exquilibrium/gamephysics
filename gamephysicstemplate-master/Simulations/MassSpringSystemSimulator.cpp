#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator(){
	m_iTestCase = 0;
	
	initial_length = 1.0f;

	m_fDamping = 0;
	m_fMass = 10;
	m_fStiffness = 40;

	m_iIntegrator = EULER;

	m_fSphereSize = 0.05f;

	gravity = true;

	points.push_back(Point(Vec3(0,0,0), Vec3(-1,0,0), 10, 0, false));
	points.push_back(Point(Vec3(0, 2, 0), Vec3(1, 0, 0), 10, 0, false));
	springs.push_back(Spring(&points[0], &points[1], 40, 1));
}


const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "2-point mass-spring-setup, 10-point mass-spring-setup";
}

void MassSpringSystemSimulator::reset() {
	//TODO probably has to do more
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	//TODO: put UI for time step, method, etc
	this->DUC = DUC;
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	//TODO: Add other cases
	switch (testCase)
	{
	case 0:
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
}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	for (auto &p : points) {
		p.clearForce();
		if (gravity) {
			p.force += Vec3(0, -9.81*p.mass, 0);
		}

		//?
	}
}
void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
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
	springs.push_back(Spring(&points[masspoint2], &points[masspoint2], m_fStiffness, initialLength));
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
	
}

void MassSpringSystemSimulator::eulerIntegrate(float timeStep) {
	for (auto &p : points) {
		Vec3 a = p.force / p.mass;
		Vec3 v = p.vel + timeStep * a;
		p.pos = p.pos + timeStep * p.vel;
		p.vel = v;
	}
}

void MassSpringSystemSimulator::midpointIntegrate(float timeStep) {	
	for (auto &p : points) {
		Vec3 a0 = p.force / p.mass;
		p.tempPos = p.pos + timeStep/2 * p.vel;
		Vec3 v1 = p.vel + timeStep/2 * a0;

		p.pos = p.pos + timeStep * v1;
	}

	externalForcesCalculations(timeStep);
	for (auto  &s : springs) { 

		float l = sqrt(s.p1->tempPos.squaredDistanceTo(s.p2->tempPos));
		Vec3 force = -s.stiffnes * (l - s.initLength) * ((s.p1->tempPos - s.p2->tempPos) / l);
		s.p1->force += force;
		s.p2->force -= force;
	}

	for (auto &p : points) {
		Vec3 a1 = p.force / p.mass;
		p.vel = p.vel + timeStep * a1;
	}
}



