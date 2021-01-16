#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(size_t m, size_t n) : n(n), m(m){
		grid = new float[n * m];
		memset(grid, 0.0f, n * m);
	}

	float get(int x, int y) {
		return grid[x + y * n];
	}

	void set(float val,  int x, int y) {
		grid[x + y * n] = val;
	}

	size_t getN() {
		return n;
	}

	size_t getM() {
		return m;
	}

private:
	// Attributes
	float* grid;
	const size_t m, n;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(float timestep);
	void diffuseTemperatureImplicit(float timestep);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float alpha, delta_t;
	size_t m, n;
	Grid *T; //save results of every time step
};

#endif