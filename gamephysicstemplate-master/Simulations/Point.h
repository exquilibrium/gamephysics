#pragma once
#include "Simulator.h"

class Point
{
public: 
	Vec3 pos, vel, force, tempPos, tempVel, tempAcc;
	float mass, damping;
	bool fixed;

	Point() {}
	
	Point(Vec3 pos, Vec3 vel, float mass, float damping, bool fixed_) {
		this->pos = pos;
		this->vel = vel;
		this->mass = mass;
		this->damping = damping;
		this->fixed = fixed_;
		this->tempPos = Vec3(0, 0, 0);
		this->tempVel = Vec3(0, 0, 0);
		this->force = Vec3(0, 0, 0);
	}


	void clearForce() {
		force = Vec3(0,0, 0);
	}
};

