#include "FBCSystemSimulator.h"
#define DebugRigidBody
FBCSystemSimulator::FBCSystemSimulator()
{
	m_fMass      = 0.01f;
	m_fStiffness = 25.0f;
	m_fDamping   = 0.01f;
	m_iIntegrator = 1;
	m_iCube = 1;
	m_iTestCase = 1;
	m_externalForce = Vec3(0,0,0); // Not initialized in Rigidbody
	m_pFBCSystem = new FBCSystem();
}


const char * FBCSystemSimulator::getTestCasesStr()
{
	return "BasicTest";
}

void FBCSystemSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void FBCSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC	 =	DUC;
    switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
		{
			TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,LeapFrog,Midpoint");
			TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		}
		break;
	case 2:
		{
			TwAddVarRW(DUC->g_pTweakBar, "SpringCube", TW_TYPE_INT32, &m_iCube, "min = 1");
			TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,LeapFrog,Midpoint");
			TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
			TwAddVarRW(DUC->g_pTweakBar, "Mass",        TW_TYPE_FLOAT, &m_fMass,       "step=0.001  min=0.001");
			TwAddVarRW(DUC->g_pTweakBar, "Stiffness",   TW_TYPE_FLOAT, &m_fStiffness,  "step=0.001  min=0.001");
			TwAddVarRW(DUC->g_pTweakBar, "Damping",     TW_TYPE_FLOAT, &m_fDamping,    "step=0.001  min=0");

		}
		break;
	default:
		break;
	}
}

void FBCSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_pFBCSystem->SetCube(m_iCube);
	m_pFBCSystem->SceneSetup(m_iTestCase);
}


void FBCSystemSimulator::externalForcesCalculations(float elapsedTime)
{
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.2f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;
	
	m_externalForce = pullforce + Vec3(0,-9.81,0);
	
}
void FBCSystemSimulator::simulateTimestep(float timeStep)
{
	// one fixed time step
	switch (m_iTestCase)
	{
	case 0: // case 0 do nothing
		m_pFBCSystem->addGlobalFrameForce(m_externalForce);
		m_pFBCSystem->ComputeSpringForces();
		m_pFBCSystem->update(timeStep);
		break;
	case 1:
	{
		m_pFBCSystem->addGlobalFrameForce(m_externalForce);
		m_pFBCSystem->ComputeSpringForces();
		m_pFBCSystem->update(timeStep);
	}
		break;
	case 2:
	{
		if (DXUTIsKeyDown(VK_LBUTTON))
			m_pFBCSystem->dragTogether();
		m_pFBCSystem->addGlobalFrameForce(m_externalForce);
		m_pFBCSystem->ComputeSpringForces();
		m_pFBCSystem->update(timeStep);

        m_pFBCSystem->SetGravity(m_externalForce);
        m_pFBCSystem->SetMass(m_fMass);
        m_pFBCSystem->SetStiffness(m_fStiffness);
        m_pFBCSystem->SetDamping(m_fDamping);
	}
	break;		
	default:
		break;
	}
}
void FBCSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
		Vec3 colors[3] = {Vec3(0.9,0.97,1), Vec3(0.5,0.5,1), Vec3(1,1,0)};
		int i = 0;
		for (RigidBody& rigidBody : m_pFBCSystem->m_rigidBodies)
		{
			DUC->setUpLighting(Vec3(0,0,0),0.4*Vec3(1,1,1),2000.0,colors[i%3]);
			DUC->drawRigidBody(rigidBody.getObj2World());
			++i;
#ifdef DebugRigidBody
			std::mt19937 eng;
			std::uniform_real_distribution<float> randCol( 0.0f, 1.0f);
			std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
			DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(randCol(eng),randCol(eng), randCol(eng)));
			//DUC->drawSphere(rigidBody.collisonPoint,Vec3(0.05f, 0.05f, 0.05f));
			DUC->beginLine();
			Vec3 velocity = rigidBody.totalVelocity;
			DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(),Colors::DeepPink,rigidBody.collisonPoint.toDirectXVector()+rigidBody.relVelocity.toDirectXVector(),Colors::DeepPink);

			if(i==1){
				DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(),Colors::DarkGreen,rigidBody.collisonPoint.toDirectXVector()+velocity.toDirectXVector(),Colors::DarkGreen);
				DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(),Colors::Yellow,rigidBody.collisonPoint.toDirectXVector()+rigidBody.collisioNormal.toDirectXVector(),Colors::Orange);
			}
			else{ 
				DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(),Colors::DarkRed,rigidBody.collisonPoint.toDirectXVector()+velocity.toDirectXVector(),Colors::DarkRed);
				DUC->drawLine(rigidBody.collisonPoint.toDirectXVector(),Colors::DarkCyan,rigidBody.collisonPoint.toDirectXVector()+rigidBody.collisioNormal.toDirectXVector(),Colors::DarkViolet);

			}
			DUC->endLine();

/*
			std::vector<XMVECTOR> corners = rigidBody.getCorners();
			Vec3 colors[8] = {Vec3(0,0,0),Vec3(0,0,1),Vec3(0,1,0),Vec3(0,1,1),Vec3(1,0,0),Vec3(1,0,1),Vec3(1,1,0),Vec3(1,1,1)};
			for(int j = 0; j< 8;j++)
			{
				DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,colors[j]);
				DUC->drawSphere(corners[j],Vec3(0.02f, 0.02f, 0.02f));
			
			}
			*/
#endif
		}

		DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*Vec3(0.83,0.36,0.36));
		DUC->beginLine();
		auto& springs = m_pFBCSystem->GetSprings();
		for(size_t i=0; i<springs.size(); i++)
		{
			Vec3 color = Vec3(0,0.4,0);
			DUC->drawLine(springs[i].body1->getCenter(),color,springs[i].body2->getCenter(),color);	
		}
		DUC->endLine();
}

int FBCSystemSimulator::getNumberOfRigidBodies()
{
	return m_pFBCSystem->m_rigidBodies.size();
}

Vec3 FBCSystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pFBCSystem->m_rigidBodies[i].getCenter();
}

Vec3 FBCSystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pFBCSystem->m_rigidBodies[i].getVelocity();
}

Vec3 FBCSystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pFBCSystem->m_rigidBodies[i].getAngularV();
}

void FBCSystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pFBCSystem->m_rigidBodies[i].addForceWorld(force,loc);
}

void FBCSystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pFBCSystem->m_rigidBodies.emplace_back(position, size, mass);
	m_pFBCSystem->m_rigidBodies.back().update(0.0f);

}

void FBCSystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pFBCSystem->m_rigidBodies[i].setRotation(orientation);

}

void FBCSystemSimulator::setVelocityOf(int i,Vec3 velocity)
{
	m_pFBCSystem->m_rigidBodies[i].setVelocity(velocity);
}

// FBCSystem
void FBCSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	m_pFBCSystem->SetStiffness(m_fStiffness);
}

void FBCSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
	m_pFBCSystem->SetMass(m_fMass);
}

void FBCSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
	m_pFBCSystem->SetDamping(m_fDamping);
}

void FBCSystemSimulator::addSpring(int index1, int index2, float initialLength)
{
	 m_pFBCSystem->AddSpring(index1,index2,initialLength);
}

int FBCSystemSimulator::getNumberOfMassPoints()
{
	return m_pFBCSystem->m_rigidBodies.size();
}

int FBCSystemSimulator::getNumberOfSprings()
{
	return m_pFBCSystem->GetSprings().size();
}

Vec3 FBCSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_pFBCSystem->m_rigidBodies[index].getCenter();
}

Vec3 FBCSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_pFBCSystem->m_rigidBodies[index].getVelocity();
}

void FBCSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
	m_pFBCSystem->SetGravity(m_externalForce.toDirectXVector());
}

void FBCSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void FBCSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}