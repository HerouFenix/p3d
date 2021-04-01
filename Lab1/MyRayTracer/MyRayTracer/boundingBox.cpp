#ifndef AABB_H
#define AABB_H

#include "vector.h"
#include "boundingBox.h"

//-------------------------------------------------------------------- - default constructor
AABB::AABB(void) 
{
	min = Vector(-1.0f, -1.0f, -1.0f);
	max = Vector(1.0f, 1.0f, 1.0f);
	// TODO:
	// For UGrid the plane's bounding box should extend the entire screen, but since the default constructor
	// of the Bounding Box only goes from -1 to 1, it doesn't... is there a better way to do this?
	//min = Vector(-9.0f, -9.0f, -1.0f);
	//max = Vector(1.5f, 1.5f, 1.0f);
}

// --------------------------------------------------------------------- constructor
AABB::AABB(const Vector& v0, const Vector& v1)
{
	min = v0; max = v1;
}

// --------------------------------------------------------------------- copy constructor
AABB::AABB(const AABB& bbox) 
{
	min = bbox.min; max = bbox.max;
}

// --------------------------------------------------------------------- assignment operator
AABB AABB::operator= (const AABB& rhs) {
	if (this == &rhs)
		return (*this);
	min = rhs.min;
	max = rhs.max;
	return (*this);
}

// --------------------------------------------------------------------- destructor
AABB::~AABB() {}

// --------------------------------------------------------------------- inside
// used to test if a ray starts inside a grid

bool AABB::isInside(const Vector& p) 
{
	return ((p.x > min.x && p.x < max.x) && (p.y > min.y && p.y < max.y) && (p.z > min.z && p.z < max.z));
}

#endif