#include "grid.h"
#include "maths.h"

Grid::Grid(void)
{
}

int Grid::getNumObjects()
{
	return objects.size();
}

void Grid::addObject(Object* o)
{
	objects.push_back(o);
}

Object* Grid::getObject(unsigned int index)
{
	return objects.at(index);
}

void Grid::Build(void)
{
	// https://www.scratchapixel.com/code.php?id=36&origin=/lessons/advanced-rendering/introduction-acceleration-structure

	// Find Bounding box of scene
	Vector min = find_min_bounds();
	Vector max = find_max_bounds();

	bbox = AABB(min, max);

	// Choose grid resolution
	int num_objects = getNumObjects();

	Vector size = max - min;
	float cubeRoot = powf(num_objects / (size.x * size.y * size.z), 1 / 3);

	// TODO: CHECK THIS -  Somewhy without that + 1 some of the image gets cut..
	nx = trunc(m * size.x * cubeRoot) + 1;
	//if (nx < 1) nx = 1;

	ny = trunc(m * size.y * cubeRoot) + 1;
	//if (ny < 1) ny = 1;

	nz = trunc(m * size.z * cubeRoot) + 1;
	//if (nz < 1) nz = 1;

	int numCells = nx * ny * nz;

	// Create Cell Vector (empty cells)
	for (int i = 0; i < numCells; i++) {
		cells.push_back(vector<Object*>());
	}

	// Insert primitives into grid
	Object* obj;
	AABB obj_bound;
	Vector cur_min, cur_max;

	for (int i = 0; i < num_objects; i++) {
		obj = getObject(i);
		obj_bound = obj->GetBoundingBox();

		cur_min = obj_bound.min - min;
		cur_max = obj_bound.max - min;

		int zmin = clamp(cur_min.z * nz / (max.z - min.z), 0, nz - 1);
		int zmax = clamp(cur_max.z * nz / (max.z - min.z), 0, nz - 1);
		
		int ymin = clamp(cur_min.y * ny / (max.y - min.y), 0, ny - 1);
		int ymax = clamp(cur_max.y * ny / (max.y - min.y), 0, ny - 1);
		
		int xmin = clamp(cur_min.x * nx / (max.x - min.x), 0, nx - 1);
		int xmax = clamp(cur_max.x * nx / (max.x - min.x), 0, nx - 1);


		// Loop over the cells the box overlaps and insert
		for (int z = zmin; z <= zmax; z++) {
			for (int y = ymin; y <= ymax; y++) {
				for (int x = xmin; x <= xmax; x++) {
					int index = z * nx * ny + y * nx + x;
					cells.at(index).push_back(obj);
				}
			}
		}
	}
}

bool Grid::Traverse(Ray& ray, Object** hitobject, Vector& hitpoint)
{
	https://www.scratchapixel.com/lessons/advanced-rendering/introduction-acceleration-structure/grid
	
	int ix, iy, iz, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop;

	double t_next_x, t_next_y, t_next_z;
	double dt_x, dt_y, dt_z;

	// Init values (or cancel if init traverse returns false)
	//bool oof = Init_Traverse(ray, ix, iy, iz, dt_x, dt_y, dt_z, t_next_x, t_next_y, t_next_z, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop);
	//if (ix < -50 && oof) {
	//	oof = 5;
	//}

	if (!Init_Traverse(ray, ix, iy, iz, dt_x, dt_y, dt_z, t_next_x, t_next_y, t_next_z, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop)) {
		return false;
	}

	vector<Object*> cell;
	int cell_index;

	Object* obj, * min_obj;
	min_obj = NULL;
	float cur_t = FLT_MAX;
	float min_t = FLT_MAX;

	while (1) {
		cell_index = iz * nx * ny + iy * nx + ix;
		cell = cells.at(cell_index); // Get current cell

		// Iterate over all objects in cell and get closest to ray origin
		for (int i = 0; i < cell.size(); i++) {
			obj = cell.at(i);
			if (obj->intercepts(ray, cur_t)) {
				if (cur_t < min_t) {
					min_t = cur_t;
					min_obj = obj;
				}
			}
		}

		// Next cell - 3DDDA
		if (t_next_x < t_next_y && t_next_x < t_next_z) { // Move X
			if (min_obj != NULL && min_t < t_next_x) { // If the min object is not null and the next cell is further than the hit point, we done
				*hitobject = min_obj;
				hitpoint = ray.origin + ray.direction * min_t;
				return true;
			}

			t_next_x += dt_x; // Move to next cell
			ix += ix_step;

			if (ix == ix_stop) { // If we can't move anymore, stop
				return false;
			}
		}
		else if (t_next_y < t_next_z) { // Move Y
			if (min_obj != NULL && min_t < t_next_y) { // If the min object is not null and the next cell is further than the hit point, we done
				*hitobject = min_obj;
				hitpoint = ray.origin + ray.direction * min_t;
				return true;
			}

			t_next_y += dt_y; // Move to next cell
			iy += iy_step;

			if (iy == iy_stop) { // If we can't move anymore, stop
				return false;
			}
		}
		else { // Move Z
			if (min_obj != NULL && min_t < t_next_z) { // If the min object is not null and the next cell is further than the hit point, we done
				*hitobject = min_obj;
				hitpoint = ray.origin + ray.direction * min_t;
				return true;
			}

			t_next_z += dt_z; // Move to next cell
			iz += iz_step;

			if (iz == iz_stop) { // If we can't move anymore, stop
				return false;
			}
		}

	}

	return false;
}

bool Grid::Traverse(Ray& ray)
{ // Same as other traverse but we don't care where the intersection is. We just want to find ANY intersection
	https://www.scratchapixel.com/lessons/advanced-rendering/introduction-acceleration-structure/grid

	int ix, iy, iz, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop;

	double t_next_x, t_next_y, t_next_z;
	double dt_x, dt_y, dt_z;

	// Init values (or cancel if init traverse returns false)
	if (!Init_Traverse(ray, ix, iy, iz, dt_x, dt_y, dt_z, t_next_x, t_next_y, t_next_z, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop)) {
		return false;
	}

	vector<Object*> cell;
	int cell_index;

	Object* obj, * min_obj;
	min_obj = NULL;
	float cur_t = FLT_MAX;

	while (1) {
		cell_index = iz * nx * ny + iy * nx + ix;
		cell = cells.at(cell_index); // Get current cell

		// Iterate over all objects in cell and get closest to ray origin
		for (int i = 0; i < cell.size(); i++) {
			obj = cell.at(i);
			if (obj->intercepts(ray, cur_t)) {
				return true; // Found an intersection
			}
		}

		// Next cell - 3DDDA
		if (t_next_x < t_next_y && t_next_x < t_next_z) { // Move X
			t_next_x += dt_x; // Move to next cell
			ix += ix_step;

			if (ix == ix_stop) { // If we can't move anymore, stop
				return false;
			}
		}
		else if (t_next_y < t_next_z) { // Move Y
			t_next_y += dt_y; // Move to next cell
			iy += iy_step;

			if (iy == iy_stop) { // If we can't move anymore, stop
				return false;
			}
		}
		else { // Move Z
			t_next_z += dt_z; // Move to next cell
			iz += iz_step;

			if (iz == iz_stop) { // If we can't move anymore, stop
				return false;
			}
		}

	}

	return false;
}

Vector Grid::find_min_bounds(void)
{
	Vector cur_min;
	float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;

	int objects = getNumObjects();

	for (int i = 0; i < objects; i++) {
		cur_min = getObject(i)->GetBoundingBox().min;
		if (cur_min.x < min_x) {
			min_x = cur_min.x;
		}

		if (cur_min.y < min_y) {
			min_y = cur_min.y;
		}

		if (cur_min.z < min_z) {
			min_z = cur_min.z;
		}
	}

	return Vector(min_x - .0001f, min_y - .0001f, min_z - .0001f);
}

Vector Grid::find_max_bounds(void)
{
	Vector cur_max;
	float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;

	int objects = getNumObjects();

	for (int i = 0; i < objects; i++) {
		cur_max = getObject(i)->GetBoundingBox().max;
		if (cur_max.x > max_x) {
			max_x = cur_max.x;
		}

		if (cur_max.y > max_y) {
			max_y = cur_max.y;
		}

		if (cur_max.z > max_z) {
			max_z = cur_max.z;
		}
	}

	return Vector(max_x + .0001f, max_y + .0001f, max_z + .0001f);
}

bool Grid::Init_Traverse(Ray& ray, int& ix, int& iy, int& iz, double& dtx, double& dty, double& dtz, double& tx_next, double& ty_next, double& tz_next, int& ix_step, int& iy_step, int& iz_step, int& ix_stop, int& iy_stop, int& iz_stop)
{
	// Intersect ray with scene bounding box
	float tx_min = (bbox.min.x - ray.origin.x) / ray.direction.x;
	float tx_max = (bbox.max.x - ray.origin.x) / ray.direction.x;
	if (tx_min > tx_max) swap(tx_max, tx_min);

	float ty_min = (bbox.min.y - ray.origin.y) / ray.direction.y;
	float ty_max = (bbox.max.y - ray.origin.y) / ray.direction.y;
	if (ty_min > ty_max) swap(ty_max, ty_min);

	float tz_min = (bbox.min.z - ray.origin.z) / ray.direction.z;
	float tz_max = (bbox.max.z - ray.origin.z) / ray.direction.z;
	if (tz_min > tz_max) swap(tz_max, tz_min);

	float tMin = MAX3(tx_min, ty_min, tz_min);
	float tMax = MIN3(tx_max, ty_max, tz_max);

	if (tMin > tMax || tMax < 0) {
		return false;
	}

	// Ray movement increments per cell
	dtx = (tx_max - tx_min) / nx;
	dty = (ty_max - ty_min) / ny;
	dtz = (tz_max - tz_min) / nz;

	// Starting cell
	Vector cur_min, cur_max;
	if (!bbox.isInside(ray.origin)) { // If ray does not start inside bounding box		
		Vector p = ray.origin + ray.direction * tMin;
		ix = clamp((p.x - bbox.min.x) * nx / (bbox.max.x - bbox.min.x), 0, nx - 1);
		iy = clamp((p.y - bbox.min.y) * ny / (bbox.max.y - bbox.min.y), 0, ny - 1);
		iz = clamp((p.z - bbox.min.z) * nz / (bbox.max.z - bbox.min.z), 0, nz - 1);
	}
	else {
		ix = clamp((ray.origin.x - bbox.min.x) * nx / (bbox.max.x - bbox.min.x), 0, nx - 1);
		iy = clamp((ray.origin.y - bbox.min.y) * ny / (bbox.max.y - bbox.min.y), 0, ny - 1);
		iz = clamp((ray.origin.z - bbox.min.z) * nz / (bbox.max.z - bbox.min.z), 0, nz - 1);
	}

	// DDA Digital Differential Analyser Algorithm - https://www.scratchapixel.com/lessons/advanced-rendering/introduction-acceleration-structure/grid
	if (ray.direction.x < 0) {
		tx_next = tx_min + (nx - ix) * dtx;
		ix_step = -1;
		ix_stop = -1;
	}
	else {
		tx_next = tx_min + (ix + 1) * dtx;
		ix_step = 1;
		ix_stop = nx;
	}

	if (ray.direction.y < 0) {
		ty_next = ty_min + (ny - iy) * dty;
		iy_step = -1;
		iy_stop = -1;
	}
	else {
		ty_next = ty_min + (iy + 1) * dty;
		iy_step = 1;
		iy_stop = ny;
	}

	if (ray.direction.z < 0) {
		tz_next = tz_min + (nz - iz) * dtz;
		iz_step = -1;
		iz_stop = -1;
	}
	else {
		tz_next = tz_min + (iz + 1) * dtz;
		iz_step = 1;
		iz_stop = nz;
	}

	return true;
}
