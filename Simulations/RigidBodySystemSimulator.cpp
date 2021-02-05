#include "RigidBodySystemSimulator.h"
#define DebugRigidBody
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 1;
	m_pRigidBodySystem = new RigidBodySystem();
}


const char * RigidBodySystemSimulator::getTestCasesStr()
{
	return "BasicTest,Setup1,Setup2";
}

void RigidBodySystemSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC	 =	DUC;
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_pRigidBodySystem->SceneSetup(m_iTestCase);
}


void RigidBodySystemSimulator::externalForcesCalculations(float elapsedTime)
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
	
	m_externalForce = pullforce;
	
}
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	// one fixed time step
	switch (m_iTestCase)
	{
	case 0: // case 0 do nothing
		break;
	case 1:
	{
		m_pRigidBodySystem->addGlobalFrameForce(m_externalForce);
		m_pRigidBodySystem->update(timeStep);
	}
		break;
	case 2:
	{
		if (DXUTIsKeyDown(VK_LBUTTON))
			m_pRigidBodySystem->dragTogether();
		m_pRigidBodySystem->addGlobalFrameForce(m_externalForce);
		m_pRigidBodySystem->update(timeStep);
			
	}
	break;		
	default:
		break;
	}
}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
		Vec3 colors[3] = {Vec3(0.9,0.97,1), Vec3(0.5,0.5,1), Vec3(1,1,0)};
		int i = 0;
		for (RigidBody& rigidBody : m_pRigidBodySystem->m_rigidBodies)
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
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->m_rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigidBodies[i].getCenter();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigidBodies[i].getVelocity();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_rigidBodies[i].getAngularV();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem->m_rigidBodies[i].addForceWorld(force,loc);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pRigidBodySystem->m_rigidBodies.emplace_back(position, size, mass);
	m_pRigidBodySystem->m_rigidBodies.back().update(0.0f);

}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->m_rigidBodies[i].setRotation(orientation);

}

void RigidBodySystemSimulator::setVelocityOf(int i,Vec3 velocity)
{
	m_pRigidBodySystem->m_rigidBodies[i].setVelocity(velocity);
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