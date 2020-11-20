#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator(){
	m_iTestCase = 0;
	
	initial_length = 1.0f;

	m_fDamping = 0;
	m_fMass = 10;
	m_fStiffness = 40;

	m_iIntegrator = EULER;

	m_fSphereSize = 0.05f;

	changeConfig(init_case0);
}

void MassSpringSystemSimulator::changeConfig(InitValues values) {
	num_mp = values.get_num_mp();
	positions = values.get_positions();
	velocities = values.get_velocities();

	num_springs = values.get_num_spring();
	springs = values.get_springs();

	spring_lengths = vector<float>(num_springs);
	for (int i = 0; i < num_springs; i++) {
		spring_lengths[i] = initial_length;
	}
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
	if (num_mp < 1)
		return;
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for (int i = 0; i < num_mp; i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(positions[i], Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
	}
}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	//TODO
}
void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	//TODO
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
	//TODO
	return 0;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	//TODO
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return num_mp;
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return num_springs;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return positions[index];
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return velocities[index];
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	//TODO
}


