#include "rigidBody.hpp"
#ifndef RIGIDBODYSYSTEM_h
#define RIGIDBODYSYSTEM_h
#include <vector>
#include <AntTweakBar.h>
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>

class RigidBodySystem
{
public:
	RigidBodySystem();
	virtual ~RigidBodySystem();

	void initTweakBar(TwBar* tweakBar);
	void SceneSetup(int sceneflag);
	void dragTogether();
	void addGlobalFrameForce(const GamePhysics::Vec3 force);

	void update(float deltaTime);
	std::vector<RigidBody> m_rigidBodies;
};
#endif