#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	alpha = 1.0f;
	delta_t = 0.1f;
	m = 25;
	n = 25;
	T = new Grid(m, n);
	// to be implemented
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
		for (int y = 1; y < m - 1; y++) {
			for (int x = 1; x < n - 1; x++) {
				T->set(x, y, 0.5f);
			}
		}
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "n", TW_TYPE_INT32, &n, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "m", TW_TYPE_INT32, &m, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "alpha", TW_TYPE_FLOAT, &alpha, "min=0");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		reset();
		break;
	case 1:
		cout << "Implicit solver!\n";
		reset();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timestep) {//add your own parameters
	size_t m = T->getM();
	size_t n = T->getN();
	Grid* newT = new Grid(m, n);
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	
	for (int y = 1; y < m - 1; y++) {
		for (int x = 1; x < n - 1; x++) {
			float F = alpha * timestep / 4.0f;
			float newVal = T->get(x, y) + F * (T->get(x + 1, y + 1) - T->get(x - 1, y + 1) - T->get(x + 1, y - 1) + T->get(x - 1, y - 1));
			newT->set(newVal, x, y);
		}
	}

	return newT;
}

void setupB(std::vector<Real>& b, size_t m, size_t n, float f, Grid *old_t) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int y = 0; y < m; y++) {
		for (int x = 0; x < n; x++) {
			b.at(x + y * n) = f * old_t->get(x, y);
		}
	}
}

void fillT(size_t m, size_t n, Grid* t, vector<Real> x_result) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	for (int y = 1; y < m - 1; y++) {
		for (int x = 1; x < n - 1; x++) {
			t->set(x_result[x + y * n], x, y);
		}
	}
}

void setupA(SparseMatrix<Real>& A, double factor, size_t m, size_t n, float f) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < m * n; i++) {
		if (i % n == 0 || i % n == n - 1 || i < n || i > (m - 1) * n)// set diagonal
			A.set_element(i, i, 1.0);
	}

	for (int y = 1; y < m - 1; y++) {
		for (int x = 1; x < n - 1; x++) {
			A.set_element(x + y * n, x + y * n, f);
			A.set_element(x + y * n, x - 1 + (y - 1) * n, 1);
			A.set_element(x + y * n, x + 1 + (y - 1) * n, -1);
			A.set_element(x + y * n, x - 1 + (y + 1) * n, -1);
			A.set_element(x + y * n, x + 1 + (y + 1) * n, 1);
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timestep) {//add your own parameters
	// solve A T = b
	// to be implemented
	size_t m = T->getM();
	size_t n = T->getN();
	float F = 4.0f / (alpha * timestep);
	const int N = T->getM() * T->getN();//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);
		
	setupA(*A, 0.1, m, n, F);
	setupB(*b, m, n, F, T);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(m, n, T, x);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	int min = INT_MAX;
	int max = INT_MIN;
	for (int y = 0; y < m; y++) {
		for (int x = 0; x < n; x++) {
			float val = T->get(x, y);
			if (val < min)
				min = val;
			if (val > max)
				max = val;
		}
	}
	for (int y = 0; y < m; y++) {
		for (int x = 0; x < n; x++) {
			float val = T->get(x, y);
			float frac = (val - min) / (max - min);
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(frac, 0, 1 - frac));
			DUC->drawSphere(Vec3(0.1 * x, 0.1 * y, 0), Vec3(0.1, 0.1, 0.1));
		}
	}
	}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
