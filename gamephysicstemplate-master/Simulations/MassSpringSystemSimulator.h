#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	class InitValues {
	private:
		const int num_mp;
		const vector<Vec3> positions;
		const vector<Vec3> velocities;

		const int num_spring;
		const vector<pair<int, int>> springs;

	public:
		InitValues(const int num_mp, const vector<Vec3> positions, const vector<Vec3> velocities, const int num_spring, const vector<pair<int, int>> springs) : num_mp(num_mp), positions(positions), velocities(velocities), 
			num_spring(num_spring), springs(springs) {

		}

		int get_num_mp() {
			return num_mp;
		}

		vector<Vec3> get_positions() {
			vector<Vec3> result = positions;
			return result;
		}

		vector<Vec3> get_velocities() {
			vector<Vec3> result = velocities;
			return result;
		}

		int get_num_spring() {
			return num_spring;
		}

		vector<pair<int, int>> get_springs() {
			vector<pair<int, int>> result = springs;
			return result;
		}
	};

	// Data Attributes
	InitValues init_case0 = InitValues(
		2, vector<Vec3>{ Vec3(0, 0, 0), Vec3(0, 2, 0) }, vector<Vec3>{Vec3(0, 1, 0), Vec3(0, -1, 0)},
		1, vector<pair<int, int>> {make_pair(0, 1)}
	);

	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	float gravity;
	float initial_length;

	int num_mp;
	vector<Vec3> positions;
	vector<Vec3> velocities;
	int num_springs;
	vector<pair<int, int>> springs;
	vector<float> spring_lengths;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_fSphereSize;

	void changeConfig(InitValues values);
};
#endif