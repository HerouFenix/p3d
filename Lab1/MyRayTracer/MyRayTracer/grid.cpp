#include "rayAccelerator.h"
#include "macros.h"


Grid::Grid(void) {}

int Grid::getNumObjects()
{
	return objects.size();
}


void Grid::setAABB(AABB& bbox_) { this->bbox = bbox_; }


void Grid::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Grid::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}

// ---------------------------------------------setup_cells
void Grid::Build(vector<Object*>& objs) {

	int xmin, xmax;
	int ymin, ymax;
	int zmin, zmax;
	int index;  	// cell's array index


	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	AABB grid_bbox = AABB(min, max);

	//build the Grid BB and //insert scene objects in the Grid objects list
	for (Object* obj : objs) {
		AABB o_bbox = obj->GetBoundingBox();
		grid_bbox.extend(o_bbox);
		this->addObject(obj);
		objects.push_back(obj);
	}
	//slightly enlarge the grid box just for case
	grid_bbox.min.x -= EPSILON; grid_bbox.min.y -= EPSILON; grid_bbox.min.z -= EPSILON;
	grid_bbox.max.x += EPSILON; grid_bbox.max.y += EPSILON; grid_bbox.max.z += EPSILON;

	this->setAABB(grid_bbox);


	// dimensions of the grid in the x, y, and z directions
	double wx = bbox.max.x - bbox.min.x;
	double wy = bbox.max.y - bbox.min.y;
	double wz = bbox.max.z - bbox.min.z;

	// compute the number of grid cells in the x, y, and z directions
	double s = pow(this->getNumObjects() / (wx * wy * wz), 0.3333333);  //number of objects per unit of length
	//double s = pow(100000 / (wx * wy * wz), 0.3333333);  //number of objects per unit of length
	nx = m * wx * s + 1;
	ny = m * wy * s + 1;
	nz = m * wz * s + 1;

	int cellCount = nx * ny * nz;

	// set up a array to hold the objects stored in each cell
	std::vector<Object*> obj_cell;
	for (int i = 0; i < cellCount; i++)
		cells.push_back(obj_cell);   //each cell has an array with zero elements

	// insert the objects into the cells
	for (auto& obj : objects) {   //vector iterator

		AABB obb = obj->GetBoundingBox();

		// Compute indices of both cells that contain min and max coord of obj bbox
		int ixmin = clamp((obb.min.x - bbox.min.x) * nx / (bbox.max.x - bbox.min.x), 0, nx - 1);
		int iymin = clamp((obb.min.y - bbox.min.y) * ny / (bbox.max.y - bbox.min.y), 0, ny - 1);
		int izmin = clamp((obb.min.z - bbox.min.z) * nz / (bbox.max.z - bbox.min.z), 0, nz - 1);
		int ixmax = clamp((obb.max.x - bbox.min.x) * nx / (bbox.max.x - bbox.min.x), 0, nx - 1);
		int iymax = clamp((obb.max.y - bbox.min.y) * ny / (bbox.max.y - bbox.min.y), 0, ny - 1);
		int izmax = clamp((obb.max.z - bbox.min.z) * nz / (bbox.max.z - bbox.min.z), 0, nz - 1);

		// add the object to the cells
		for (int iz = izmin; iz <= izmax; iz++) 					// cells in z direction
			for (int iy = iymin; iy <= iymax; iy++)					// cells in y direction
				for (int ix = ixmin; ix <= ixmax; ix++) 			// cells in x direction
					cells[ix + nx * iy + nx * ny * iz].push_back(obj);
	}

	printf("\nGRID: total cells = %d, total objects = %d, ResX = %d, ResY = %d, ResZ = %d\n\n", cellCount, this->getNumObjects(), nx, ny, nz);
	//Erase the vector that stores object pointers, but don't delete the objects
	objects.erase(objects.begin(), objects.end());
}

//Setup function for Grid traversal according to Amanatides&Woo algorithm
bool Grid::Init_Traverse(Ray& ray, int& ix, int& iy, int& iz, double& dtx, double& dty, double& dtz,
	double& tx_next, double& ty_next, double& tz_next, int& ix_step, int& iy_step, int& iz_step, int& ix_stop, int& iy_stop, int& iz_stop) {


	float t0, t1; //entering and leaving points

	float ox = ray.origin.x;
	float oy = ray.origin.y;
	float oz = ray.origin.z;
	float dx = ray.direction.x;
	float dy = ray.direction.y;
	float dz = ray.direction.z;

	float x0 = bbox.min.x;
	float y0 = bbox.min.y;
	float z0 = bbox.min.z;
	float x1 = bbox.max.x;
	float y1 = bbox.max.y;
	float z1 = bbox.max.z;


	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;

	float a = 1.0 / dx;
	if (a >= 0) {
		tx_min = (x0 - ox) * a;
		tx_max = (x1 - ox) * a;
	}
	else {
		tx_min = (x1 - ox) * a;
		tx_max = (x0 - ox) * a;
	}

	float b = 1.0 / dy;
	if (b >= 0) {
		ty_min = (y0 - oy) * b;
		ty_max = (y1 - oy) * b;
	}
	else {
		ty_min = (y1 - oy) * b;
		ty_max = (y0 - oy) * b;
	}

	float c = 1.0 / dz;
	if (c >= 0) {
		tz_min = (z0 - oz) * c;
		tz_max = (z1 - oz) * c;
	}
	else {
		tz_min = (z1 - oz) * c;
		tz_max = (z0 - oz) * c;
	}

	if (tx_min > ty_min)
		t0 = tx_min;
	else
		t0 = ty_min;

	if (tz_min > t0)
		t0 = tz_min;

	if (tx_max < ty_max)
		t1 = tx_max;
	else
		t1 = ty_max;

	if (tz_max < t1)
		t1 = tz_max;

	if (t0 > t1 || t1 < 0)   //crossover: ray does not intersect the Grid bounding box OR leaving point is behind the ray origin
		return(false);


	// Calculate initial cell coordinates

	if (bbox.isInside(ray.origin)) {  			// does the ray start inside the grid?
		ix = clamp((ox - x0) * nx / (x1 - x0), 0, nx - 1);
		iy = clamp((oy - y0) * ny / (y1 - y0), 0, ny - 1);
		iz = clamp((oz - z0) * nz / (z1 - z0), 0, nz - 1);
	}
	else {
		Vector p = ray.origin + ray.direction * t0;  // initial hit point with grid's bounding box
		ix = clamp((p.x - x0) * nx / (x1 - x0), 0, nx - 1);
		iy = clamp((p.y - y0) * ny / (y1 - y0), 0, ny - 1);
		iz = clamp((p.z - z0) * nz / (z1 - z0), 0, nz - 1);
	}

	// ray parameter increments per cell in the x, y, and z directions
	dtx = (tx_max - tx_min) / nx;
	dty = (ty_max - ty_min) / ny;
	dtz = (tz_max - tz_min) / nz;

	if (dx > 0) {
		tx_next = tx_min + (ix + 1) * dtx;
		ix_step = +1;
		ix_stop = nx;
	}
	else {
		tx_next = tx_min + (nx - ix) * dtx;
		ix_step = -1;
		ix_stop = -1;
	}

	if (dx == 0.0) {
		tx_next = FLT_MAX;
		//ix_step = -1;  //doesn't matter. Never used
	//	ix_stop = -1;  //doesn't matter. Never used
	}

	if (dy > 0) {
		ty_next = ty_min + (iy + 1) * dty;
		iy_step = +1;
		iy_stop = ny;
	}
	else {
		ty_next = ty_min + (ny - iy) * dty;
		iy_step = -1;
		iy_stop = -1;
	}

	if (dy == 0.0) {
		ty_next = FLT_MAX;
		//	iy_step = -1;
		//	iy_stop = -1;
	}

	if (dz > 0) {
		tz_next = tz_min + (iz + 1) * dtz;
		iz_step = +1;
		iz_stop = nz;
	}
	else {
		tz_next = tz_min + (nz - iz) * dtz;
		iz_step = -1;
		iz_stop = -1;
	}

	if (dz == 0.0) {
		tz_next = FLT_MAX;
		//iz_step = -1;
		//iz_stop = -1;
	}
	return true;
}

//-----------------------------------------------------------------------GRID TRAVERSAL
bool Grid::Traverse(Ray& ray, Object** hitobject, Vector& hitpoint) {
	int ix, iy, iz;
	double 	tx_next, ty_next, tz_next;
	double dtx, dty, dtz;

	int 	ix_step, iy_step, iz_step;
	int 	ix_stop, iy_stop, iz_stop;

	//Calculate the initial cell as well as the ray parameter increments per cell in the x, y, and z directions
	if (!Init_Traverse(ray, ix, iy, iz, dtx, dty, dtz, tx_next, ty_next, tz_next, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop))
		return false;   //ray does not intersect the Grid bounding box

	std::vector<Object*> objs;
	float closestDistance = FLT_MAX;
	Object* closestObj = NULL;
	float distance;

	while (true) {
		objs = cells[ix + nx * iy + nx * ny * iz];

		if (objs.size() != 0)
			for (auto obj : objs) //intersect Ray with all objects and find the closest hit point(if any)
				if (obj->intercepts(ray, distance) && distance < closestDistance) {
					closestDistance = distance;
					closestObj = obj;
				}

		if (tx_next < ty_next && tx_next < tz_next) {
			if (closestDistance < tx_next) {
				*hitobject = closestObj;
				hitpoint = ray.origin + ray.direction * closestDistance;
				return true;
			}
			tx_next += dtx;
			ix += ix_step;
			if (ix == ix_stop) return (false);
		}

		else if (ty_next < tz_next) {
			if (closestDistance < ty_next) {
				*hitobject = closestObj;
				hitpoint = ray.origin + ray.direction * closestDistance;
				return true;
			}
			ty_next += dty;
			iy += iy_step;
			if (iy == iy_stop) return (false);
		}

		else {
			if (closestDistance < tz_next) {
				*hitobject = closestObj;
				hitpoint = ray.origin + ray.direction * closestDistance;
				return true;
			}
			tz_next += dtz;
			iz += iz_step;
			if (iz == iz_stop) return (false);
		}

	}
}

//-----------------------------------------------------------------------GRID TRAVERSAL FOR SHADOW RAY
bool Grid::Traverse(Ray& ray) {

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	int ix, iy, iz;
	double 	tx_next, ty_next, tz_next;
	double dtx, dty, dtz;

	int 	ix_step, iy_step, iz_step;
	int 	ix_stop, iy_stop, iz_stop;

	/*Calculate the initial cell as well as the ray parameter increments per cell in the x, y, and z directions
	Shadow ray always intersect the Grid bounding box. However due to rounding it may starts at the boundaries, which may result as no intersecting. Consider it as in shadow. */
	if (!Init_Traverse(ray, ix, iy, iz, dtx, dty, dtz, tx_next, ty_next, tz_next, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop))
		return true;

	std::vector<Object*> objs;
	float distance;

	while (true) {
		objs = cells[ix + nx * iy + nx * ny * iz];
		if (objs.size() != 0)
			//intersect Ray with all objects of each cell
			for (auto& obj : objs) {
				if (obj->intercepts(ray, distance) && distance < length)
					return true;
			}

		if (tx_next < ty_next && tx_next < tz_next) {
			tx_next += dtx;
			ix += ix_step;
			if (ix == ix_stop) return (false);
		}
		else {
			if (ty_next < tz_next) {

				ty_next += dty;
				iy += iy_step;
				if (iy == iy_stop) return (false);
			}
			else {
				tz_next += dtz;
				iz += iz_step;
				if (iz == iz_stop) return (false);
			}
		}
	}
}


/*
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

	// Compute resolution
	nx = trunc(m * size.x * cubeRoot);
	if (nx < 1) nx = 1;

	ny = trunc(m * size.y * cubeRoot);
	if (ny < 1) ny = 1;

	nz = trunc(m * size.z * cubeRoot);
	if (nz < 1) nz = 1;

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
	// Used to initialize the parameters (increments per step, minimum, maximum, starting cell, ...) used in the traversal

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

	// 3DDDA Digital Differential Analyser Algorithm - https://www.scratchapixel.com/lessons/advanced-rendering/introduction-acceleration-structure/grid
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
*/