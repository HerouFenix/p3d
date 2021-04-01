#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
	Ray(const Vector& o, const Vector& dir ) : origin(o), direction(dir) {
		id = next_id++;
	};

	Vector origin;
	Vector direction;
	int id;
	static int next_id;
};
#endif