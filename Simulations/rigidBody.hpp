
#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include <GeometricPrimitive.h>
#include <Effects.h>
#include <PrimitiveBatch.h>
#include <VertexTypes.h>
#include <iostream>
#include "util/vectorbase.h"
#include "util/matrixbase.h"
#include "util/quaternion.h"
#include "collisionDetect.h"
//using namespace DirectX;


#define MYPRF(name, p) { std::cout << name << " " << XMVectorGetX(p) <<" "<< XMVectorGetY(p) <<" "<< XMVectorGetZ(p) <<" "<< XMVectorGetW(p) <<"\n"; }
#define MYPRM(name , m) { MYPRF(name, m.r[0]); MYPRF("    ", m.r[1]);  MYPRF("    ", m.r[2]);  MYPRF("    ", m.r[3]);  }

extern DirectX::BasicEffect* g_pEffectPositionNormal;
extern DirectX::PrimitiveBatch<DirectX::VertexPositionNormal>* g_pPrimitiveBatchPositionNormal;

class RigidBody
{
public:
	static bool collide(RigidBody& body1, RigidBody& body2);

	RigidBody();
	RigidBody(const GamePhysics::Vec3 center, const GamePhysics::Vec3 size, float mass);
	virtual ~RigidBody();

	void addForce(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where);
	void addForceWorld(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where);
	void update(float deltaTime);

	void setCenter(const GamePhysics::Vec3  center);
	void setVelocity(const GamePhysics::Vec3  velocity);
	void setRotation(const GamePhysics::Quat  rotation);

	GamePhysics::Vec3 collisonPoint;
	GamePhysics::Vec3 collisioNormal;
	GamePhysics::Vec3 totalVelocity;
	GamePhysics::Vec3 relVelocity;

	std::vector<XMVECTOR> getCorners();
	const GamePhysics::Vec3 getCenter() const;

	const GamePhysics::Vec3 getVelocity() const
	{
		return m_velocity;
	}

	const GamePhysics::Vec3 getAngularV() const
	{
		return m_angularV;
	}

	const GamePhysics::Mat4 getWorld2Obj() const
	{
		return m_worldToObj;
	}

	const GamePhysics::Mat4 getObj2World() const
	{
		return m_objToWorld;
	}

//sprotected:
	//struct CollisionInfo{ // the return structure, with these values, you should be able to calculate the impulse
	//	DirectX::XMVECTOR collisionPointWorld; // the position of the collision point in world space
	//	DirectX::XMVECTOR normalWorld;             // the direction of the impulse to A, negative of the collision face of A
	//};
	
	static GamePhysics::Mat4 computeInertiaTensorInverse(const GamePhysics::Vec3 size, float massInverse);

	static const GamePhysics::Vec3 s_corners[8];

	//CollisionInfo checkCollisionChild(const XMMATRIX obj2World_A, const XMMATRIX obj2World_B, 
	//float xlen_A, float ylen_A, float zlen_A, float xlen_B, float ylen_B, float zlen_B) const;

	CollisionInfo RBcheckCollision(const RigidBody& other) const;

	GamePhysics::Vec3 m_center;
	GamePhysics::Quat m_rotation;
	GamePhysics::Vec3 m_scale;
	GamePhysics::Vec3 m_angularV;
	
	GamePhysics::Vec3 m_velocity;
	GamePhysics::Vec3 m_momentum;
	float m_massInverse;
	GamePhysics::Mat4 m_inertiaTensorInverse;

	GamePhysics::Mat4 m_objToWorld;
	GamePhysics::Mat4 m_worldToObj;
	GamePhysics::Mat4 m_scaledObjToWorld;
	GamePhysics::Mat4 m_worldToScaledObj;

	GamePhysics::Vec3 m_frameForce;
	GamePhysics::Vec3 m_frameTorque;
};
#endif