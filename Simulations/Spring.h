#pragma once
#include "Point.h"
class Spring
{
public:
	Point *p1, *p2;
	float stiffnes, initLength;

	Spring(Point* p1_, Point* p2_, float stiffnes_, float initLength_) {
		this->p1 = p1_;
		this->p2 = p2_;

		this->stiffnes = stiffnes_;
		this->initLength = initLength_;
	}
};

