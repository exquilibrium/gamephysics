#include "rigidBody.hpp"

bool RigidBody::collide(RigidBody& body0, RigidBody& body1)
{
	CollisionInfo info = checkCollisionSAT(body0.m_objToWorld, body1.m_objToWorld); //, body0.m_scale, body1.m_scale
	/*if (!info.isValid){
		info = checkCollisionSAT(body1.m_objToWorld.toDirectXMatrix(), body0.m_objToWorld.toDirectXMatrix(),body1.m_scale.toDirectXVector(),body0.m_scale.toDirectXVector());
		info.normalWorld = -info.normalWorld;// we compute the impulse to body0
	}
	*/
	if(!info.isValid)return false;
	body0.collisonPoint = info.collisionPointWorld;
	body1.collisonPoint = body0.collisonPoint;
	body0.collisioNormal =  GamePhysics::Vec3(info.normalWorld);
	body1.collisioNormal = -GamePhysics::Vec3(info.normalWorld);
	GamePhysics::Vec3 xaWorld = info.collisionPointWorld - body0.m_center;
	GamePhysics::Vec3 xbWorld = info.collisionPointWorld - body1.m_center;

	// these obj positions are just used to print some message in cmd. Only for debug, not used in the impulse calculation
	GamePhysics::Vec3 xa_objA = body0.m_worldToObj.transformVector(info.collisionPointWorld);
	GamePhysics::Vec3 xb_objB = body1.m_worldToObj.transformVector(info.collisionPointWorld);
	// end obj positions

	GamePhysics::Mat4 rotation0 =  body0.m_rotation.getRotMat();
	GamePhysics::Mat4 rotation1 =  body1.m_rotation.getRotMat();
	GamePhysics::Mat4 rotationTranspose0 = rotation0;
	GamePhysics::Mat4 rotationTranspose1 = rotation1;
	rotationTranspose0.transpose();
	rotationTranspose1.transpose();
	GamePhysics::Mat4 currentInertiaTensorInverse0 = rotation0 * body0.m_inertiaTensorInverse * rotationTranspose0;
	GamePhysics::Mat4 currentInertiaTensorInverse1 = rotation1 * body1.m_inertiaTensorInverse * rotationTranspose1;
	GamePhysics::Vec3 angularVel_A = currentInertiaTensorInverse0.transformVector(body0.m_momentum);
	GamePhysics::Vec3 angularVel_B = currentInertiaTensorInverse1.transformVector(body1.m_momentum);

	GamePhysics::Vec3 velocityA = body0.m_velocity + cross(angularVel_A, xaWorld);
	GamePhysics::Vec3 velocityB = body1.m_velocity + cross(angularVel_B, xbWorld);
	
	body0.relVelocity = velocityA - velocityB;
	body0.totalVelocity = velocityA;
	body1.totalVelocity = velocityB;
	float relVelonNormal = dot(velocityA - velocityB, info.normalWorld);
	if ( relVelonNormal > 0.0f) return false; // leaving each other, collide before

	std::cout<<"collision detected at normal: "<<info.normalWorld<< std::endl;
	std::cout<<"x_a: "<<xa_objA<< std::endl;
	std::cout<<"x_b: "<<xb_objB<< std::endl;
	
	const float elasticity = 1.0f; // todo: set as a user input param
	const float numerator = - (1.0f + elasticity) * relVelonNormal;
	const float inverseMasses = body0.m_massInverse + body1.m_massInverse;

	GamePhysics::Vec3 rma = cross(currentInertiaTensorInverse0.transformVector(cross(xaWorld, info.normalWorld)), xaWorld);
	GamePhysics::Vec3 rmb = cross(currentInertiaTensorInverse1.transformVector(cross(xbWorld, info.normalWorld)), xbWorld);
	const float rmab = dot(rma + rmb, info.normalWorld);
	const float denominator = inverseMasses + rmab;
	
	const float impulse = numerator / denominator;

	GamePhysics::Vec3 impulseNormal = impulse * info.normalWorld;
	body0.m_velocity += impulseNormal * body0.m_massInverse;
	body1.m_velocity -= impulseNormal * body1.m_massInverse;

	body0.m_momentum += cross(xaWorld, impulseNormal);
	body1.m_momentum -= cross(xbWorld, impulseNormal);

	return true;
}

std::vector<XMVECTOR> RigidBody::getCorners()
{
	const XMVECTOR centerWorld = XMVector3Transform(XMVectorZero(), m_objToWorld.toDirectXMatrix());
	XMVECTOR edges[3];
	for (size_t i = 0; i < 3; ++i)
		edges[i] = XMVector3TransformNormal( XMVectorSetByIndex(XMVectorZero(), 0.5f, i), m_objToWorld.toDirectXMatrix());
	std::vector<XMVECTOR> results;
	results.push_back(centerWorld - edges[0] - edges[1] - edges[2]);
	results.push_back(centerWorld + edges[0] - edges[1] - edges[2]);
	results.push_back(centerWorld - edges[0] + edges[1] - edges[2]);
	results.push_back(centerWorld + edges[0] + edges[1] - edges[2]); // this +,+,-
	results.push_back(centerWorld - edges[0] - edges[1] + edges[2]);
	results.push_back(centerWorld + edges[0] - edges[1] + edges[2]); //this +,-,+
	results.push_back(centerWorld - edges[0] + edges[1] + edges[2]); //this -,+,+
	results.push_back(centerWorld + edges[0] + edges[1] + edges[2]);//this +,+,+
	return results;
}
/*
bool RigidBody::collide(RigidBody& body0, RigidBody& body1)
{
	CollisionInfo info = checkCollision(body0.m_objToWorld, body1.m_objToWorld);
	if (!info.isValid){
		info = checkCollision(body1.m_objToWorld, body0.m_objToWorld);
		info.normalWorld = -info.normalWorld;// we compute the impulse to body0
	}
	if (!info.isValid) return false;
	XMVECTOR xaWorld = info.collisionPointWorld - body0.m_center;//XMVector3Transform(XMVectorZero(), body0.m_objToWorld);
	XMVECTOR xbWorld = info.collisionPointWorld - body1.m_center;//XMVector3Transform(XMVectorZero(), body1.m_objToWorld);
	XMVECTOR xa_objA = XMVector3Transform(info.collisionPointWorld, body0.m_worldToObj);
	XMVECTOR xb_objB = XMVector3Transform(info.collisionPointWorld, body1.m_worldToObj);
	const XMMATRIX rotation0 = XMMatrixRotationQuaternion(body0.m_rotation);
	const XMMATRIX rotation1 = XMMatrixRotationQuaternion(body1.m_rotation);
	const XMMATRIX rotationTranspose0 = XMMatrixTranspose(rotation0);
	const XMMATRIX rotationTranspose1 = XMMatrixTranspose(rotation1);
	const XMMATRIX currentInertiaTensorInverse0 = rotation0 * body0.m_inertiaTensorInverse * rotationTranspose0;
	const XMMATRIX currentInertiaTensorInverse1 = rotation1 * body1.m_inertiaTensorInverse * rotationTranspose1;
	const XMVECTOR angularVel_A = XMVector4Transform(body0.m_momentum, currentInertiaTensorInverse0);
	const XMVECTOR angularVel_B = XMVector4Transform(body1.m_momentum, currentInertiaTensorInverse1);
// 	const XMVECTOR velocityA = body0.m_velocity + XMVector3TransformNormal( XMVector3Cross(angularVel_A, xa_objA) , body0.m_objToWorld);  
// 	const XMVECTOR velocityB = body1.m_velocity + XMVector3TransformNormal( XMVector3Cross(angularVel_B, xb_objB) , body1.m_objToWorld);
	const XMVECTOR velocityA = body0.m_velocity + XMVector3Cross(angularVel_A, xaWorld);
	const XMVECTOR velocityB = body1.m_velocity + XMVector3Cross(angularVel_B, xbWorld);
	
	float relVelonNormal = XMVectorGetX(XMVector3Dot(velocityA - velocityB, info.normalWorld));
	if ( relVelonNormal > 0.0f) return false; // leaving each other, collide before

	std::printf("collision detected at normal: %f, %f, %f\n",XMVectorGetX(info.normalWorld), XMVectorGetY(info.normalWorld), XMVectorGetZ(info.normalWorld));
	std::printf("x_a: %f, %f, %f\n",XMVectorGetX(xa_objA), XMVectorGetY(xa_objA), XMVectorGetZ(xa_objA));
	std::printf("x_b: %f, %f, %f\n",XMVectorGetX(xb_objB), XMVectorGetY(xb_objB), XMVectorGetZ(xb_objB));

	const float elasticity = 1.0f; // todo: set as a user input param
	const float numerator = - (1.0f + elasticity) * relVelonNormal;
	const float inverseMasses = body0.m_massInverse + body1.m_massInverse;

	const XMVECTOR rma = XMVector3Cross(XMVector3TransformNormal(XMVector3Cross(xaWorld, info.normalWorld), currentInertiaTensorInverse0), xaWorld);
	const XMVECTOR rmb = XMVector3Cross(XMVector3TransformNormal(XMVector3Cross(xbWorld, info.normalWorld), currentInertiaTensorInverse1), xbWorld);
	const float rmab = XMVectorGetX(XMVector3Dot(rma + rmb, info.normalWorld));
	const float denominator = inverseMasses + rmab;
	
	const float impulse = numerator / denominator;

	const XMVECTOR impulseNormal = impulse * info.normalWorld;
	body0.m_velocity += impulseNormal * body0.m_massInverse;
	body1.m_velocity -= impulseNormal * body1.m_massInverse;
	body0.m_momentum += XMVector3Cross(xaWorld, impulseNormal);
	body1.m_momentum -= XMVector3Cross(xbWorld, impulseNormal);

	return true;
}
*/
RigidBody::RigidBody() :
	m_center(),
	m_rotation(0,0,0,1),
	m_scale(XMVectorSplatOne()),
	m_velocity(),
	m_momentum(),
	m_massInverse(1.0f),
	m_inertiaTensorInverse(computeInertiaTensorInverse(m_scale, m_massInverse)),
	m_objToWorld(),
	m_scaledObjToWorld(),
	m_worldToScaledObj(),
	m_frameForce(),
	m_frameTorque()
{
}

RigidBody::RigidBody(const GamePhysics::Vec3 center, const GamePhysics::Vec3 size, float mass) :
	m_center(center),
	m_rotation(0,0,0,1),
	m_scale(size),
	m_velocity(),
	m_momentum(),
	m_massInverse(1.0f / mass),
	m_inertiaTensorInverse(computeInertiaTensorInverse(m_scale, m_massInverse)),
	m_objToWorld(),
	m_scaledObjToWorld(),
	m_worldToScaledObj(),
	m_frameForce(),
	m_frameTorque()
{
}

RigidBody::~RigidBody()
{
}

void RigidBody::addForce(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where)
{
	m_frameForce += force;
	m_frameTorque += GamePhysics::cross(where, force);
}

void RigidBody::addForceWorld(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where)
{
	addForce(force, where - m_center);
}

void RigidBody::update(float deltaTime)
{
	m_center += deltaTime * m_velocity;
	m_velocity += deltaTime * m_frameForce * m_massInverse;
	m_momentum += deltaTime * m_frameTorque;
	
	GamePhysics::Mat4 rotation = m_rotation.getRotMat();
	GamePhysics::Mat4 rotationTranspose(rotation);
	rotationTranspose.transpose();

	//MYPRM("oriTensorInverse", m_inertiaTensorInverse);
	GamePhysics::Mat4 currentInertiaTensorInverse =    rotation * m_inertiaTensorInverse * rotationTranspose;
	// the relationship between the derivative of the quaternion and the angular velocity
	// https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions

	m_angularV = currentInertiaTensorInverse.transformVector(m_momentum);
	
	//float angleRotate = XMVectorGetX(XMVector3Length(m_angularV)) * deltaTime;
	//DirectX::XMVECTOR deltaRotation = XMQuaternionRotationNormal(XMVector3Normalize(m_angularV), angleRotate);
	//m_rotation = XMQuaternionMultiply(deltaRotation, m_rotation);
	GamePhysics::Quat m_angularVq(m_angularV.x,m_angularV.y,m_angularV.z,0);
	m_rotation += deltaTime / 2.0f * m_angularVq * m_rotation;
	m_rotation = m_rotation.unit();

	m_frameForce = XMVectorZero();
	m_frameTorque = XMVectorZero();

	GamePhysics::Mat4 matrix;
	matrix.initTranslation(m_center.x,m_center.y,m_center.z);
	m_scaledObjToWorld = rotation * matrix;
	m_worldToScaledObj =  m_scaledObjToWorld.inverse();
	matrix.initScaling(m_scale.x,m_scale.y,m_scale.z);
	m_objToWorld = matrix * m_scaledObjToWorld;
	m_worldToObj = m_objToWorld.inverse();
}

void RigidBody::setCenter(const GamePhysics::Vec3 center)
{
	m_center = center;
}

void RigidBody::setVelocity(const GamePhysics::Vec3 velocity)
{
	m_velocity = velocity;
}

void RigidBody::setRotation(const GamePhysics::Quat rotation)
{
	m_rotation = rotation;
}

const GamePhysics::Vec3 RigidBody::getCenter() const
{
	return m_center;
}

GamePhysics::Mat4 RigidBody::computeInertiaTensorInverse(const GamePhysics::Vec3 size, float massInverse)
{
	// assumption: homogenous cuboid
	const float x = size.x;
	const float y = size.y;
	const float z = size.z;
	const float xSquared = x * x;
	const float ySquared = y * y;
	const float zSquared = z * z;
	
	return GamePhysics::Mat4(
		12.0f * massInverse / (ySquared + zSquared), 0.0f, 0.0f, 0.0f,
		0.0f, 12.0f * massInverse / (xSquared + zSquared), 0.0f, 0.0f,
		0.0f, 0.0f, 12.0f * massInverse / (xSquared + ySquared), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

const GamePhysics::Vec3 RigidBody::s_corners[8] =
{
	GamePhysics::Vec3(-0.5f, -0.5f, -0.5f),
	GamePhysics::Vec3(0.5f, -0.5f, -0.5f),
	GamePhysics::Vec3(-0.5f, 0.5f, -0.5f),
	GamePhysics::Vec3(0.5f, 0.5f, -0.5f),
	GamePhysics::Vec3(-0.5f, -0.5f, 0.5f),
	GamePhysics::Vec3(0.5f, -0.5f, 0.5f),
	GamePhysics::Vec3(-0.5f, 0.5f, 0.5f),
	GamePhysics::Vec3(0.5f, 0.5f, 0.5f)
};