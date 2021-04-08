#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
	Ray(const Vector& o, const Vector& dir, const double tm = 0.0 ) : origin(o), direction(dir) {
		id = next_id++;
		time = tm;
	};

	Vector origin;
	Vector direction;

	// Lab 4 - Acceleration Structures (mailboxing) //
	int id;
	static int next_id;

	// Assignment Extra - DRT to simulate Motion Blur //
	double time;
};
#endif