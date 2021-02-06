#include "RigidBody.hpp"
#ifndef FBCSYSTEM_h
#define FBCSYSTEM_h

#include <vector>
#include <AntTweakBar.h>
#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include "util\vectorbase.h"
using namespace GamePhysics;
using std::cout;

class FBCSystem
{
public:
	FBCSystem() : m_cube(0), m_rigidBodies(){}
	virtual ~FBCSystem();

	void initTweakBar(TwBar* tweakBar);
	void SceneSetup(int sceneflag);
	void dragTogether();
	void addGlobalFrameForce(const GamePhysics::Vec3 force);

	void update(float deltaTime);
	std::vector<RigidBody> m_rigidBodies;

    // Massspringsystem
	struct Spring
	{
		RigidBody* body1;
		RigidBody* body2;
		float initialLength;
	};

	void AddSpring(RigidBody* body1, RigidBody* body2, float initialLength)
	{
		Spring s;
		s.body1 = body1;
		s.body2 = body2;
		s.initialLength = initialLength;
		m_springs.push_back(s);
	}

	void AddSpring(int indexBody1, int indexBody2)
	{
		RigidBody* body1 = &m_rigidBodies[indexBody1];
		RigidBody* body2 = &m_rigidBodies[indexBody2];
		Vec3 d = (*body1).getCenter() - (*body2).getCenter();
		AddSpring(body1, body2, norm(d));
	}

	void AddSpring(int indexBody1, int indexBody2, float initialLenght)
	{
		RigidBody* body1 = &m_rigidBodies[indexBody1];
		RigidBody* body2 = &m_rigidBodies[indexBody2];
		AddSpring(body1, body2, initialLenght);
	}

	void SetMass     (float mass     ) {m_mass      = mass     ;}
	void SetStiffness(float stiffness) {m_stiffness = stiffness;}
	void SetDamping  (float damping  ) {m_damping   = damping  ;}
	void SetCube	 (int   cube	 ) {m_cube = cube;			}

	void SetGravity  (const Vec3& gravity) {m_gravity = gravity;}

	const std::vector<Spring>& GetSprings() {return m_springs;}

	//void BoundingBoxCheck(float times = 1.0f);

	void ComputeSpringForces();
private:

	std::vector<Spring> m_springs;

	float m_mass;
	float m_stiffness;
	float m_damping;

	int  m_cube;

	Vec3 m_gravity;
};
#endif