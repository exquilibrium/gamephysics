#pragma once
#include "Point.h"
class Spring
{
public:
	Point p1, p2;
	float stiffnes, initLength;

	Spring(Point p1_, Point p2_, float stiffnes_, float initLength_) {
		p1 = p1_;
		p2 = p2_;

		stiffnes = stiffnes_;
		initLength = initLength_;
	}
};

