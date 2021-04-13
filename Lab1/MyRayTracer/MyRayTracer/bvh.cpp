#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	   //PUT YOUR CODE HERE


		//right_index, left_index and split_index refer to the indices in the objects vector
	   // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	    // node.index can have a index of objects vector or a index of nodes vector
		
	if ((right_index - left_index) <= Threshold) {
		node->makeLeaf(left_index, (right_index - left_index));
	} 
	else {

		// Get largest axis /////////////////////
		AABB node_bb = node->getAABB();

		int axis;

		Vector dist = node_bb.max - node_bb.min;

		if (dist.x >= dist.y && dist.x >= dist.z) {
			axis = 0; // X axis
		}
		else if (dist.y >= dist.x && dist.y >= dist.z) {
			axis = 1; // Y axis
		}
		else {
			axis = 2; // Z axis
		}
		////////////////////////////////////////

		// sorting of the objects//////////////
		Comparator cmp;
		cmp.dimension = axis;

		sort(objects.begin() + left_index, objects.begin() + right_index, cmp);
		///////////////////////////////////////

		// Find the split index //////////////

		float mid = (node_bb.max.getIndex(axis) + node_bb.min.getIndex(axis)) * 0.5;

		bool found = false;
		int split_index;

		// Check if there are no objects on the right/left side of the mid point
		if (objects[left_index]->getCentroid().getIndex(axis) > mid ||
			objects[right_index - 1]->getCentroid().getIndex(axis) <= mid) {
			mid = 0;
			for (split_index = left_index; split_index < right_index; split_index++) {
				mid += objects[split_index]->getCentroid().getIndex(axis);
			}
			// Mean
			mid /= (right_index - left_index);

			split_index = left_index + Threshold;  //???
		}
		else {
			for (split_index = left_index; split_index < right_index; split_index++) {
				if (objects[split_index]->getCentroid().getIndex(axis) > mid) {
					break;
				}
			}
		}
		//////////////////////////////////////

		// Define the left and right nodes //

		Vector min_right, min_left = min_right = Vector(FLT_MAX, FLT_MAX, FLT_MAX); //??
		Vector max_right, max_left = max_right = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX); //??

		AABB left_bbox(min_left, max_left), right_bbox(min_right, max_right);

		for (int j = left_index; j < split_index; j++) {
			left_bbox.extend(objects[j]->GetBoundingBox());
		}

		for (int j = split_index; j < right_index; j++) {
			right_bbox.extend(objects[j]->GetBoundingBox());
		}

		BVHNode* left_node = new BVHNode();
		BVHNode* right_node = new BVHNode();
		left_node->setAABB(left_bbox);
		right_node->setAABB(right_bbox);

		node->makeNode(nodes.size());
		nodes.push_back(left_node); 
		nodes.push_back(right_node);
		////////////////////////////////

		build_recursive(left_index, split_index, left_node);
		build_recursive(split_index, right_index, right_node);

	}
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {

			float tmp;
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			bool hit = false;

			BVHNode* currentNode = nodes[0];
			
			if (!currentNode->getAABB().intercepts(ray, tmp)) {
				return false;
			}

			while (true) {
				if (!currentNode->isLeaf()) {
					BVHNode* l_node = nodes[currentNode->getIndex()];
					BVHNode* r_node = nodes[currentNode->getIndex() + 1];
					float l_t, r_t;

					bool l_hit = l_node->getAABB().intercepts(ray, l_t);
					bool r_hit = r_node->getAABB().intercepts(ray, r_t);

					if (l_node->getAABB().isInside(ray.origin)) l_t = 0;
					if (r_node->getAABB().isInside(ray.origin)) r_t = 0;

					if (l_hit && r_hit) {
						if (l_t < r_t) {
							currentNode = l_node;
							hit_stack.push(StackItem(r_node, r_t));
						}
						else {
							currentNode = r_node;
							hit_stack.push(StackItem(l_node, l_t));
						}
						continue;
					}
					else if (l_hit) {
						currentNode = l_node;
						continue;
					}
					else if (r_hit) {
						currentNode = r_node;
						continue;
					}
				}
				else {
					Object* obj;
					float curr_t;
					for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
						obj = objects[i];
						if (obj->intercepts(ray, curr_t) && curr_t < tmin) {
							tmin = curr_t;
							*hit_obj = obj;
							hit = true;
						}
					}
				}

				bool changed = false;

				while (!hit_stack.empty()) {
					StackItem popped = hit_stack.top();
					hit_stack.pop();

					if (popped.t < tmin) {
						currentNode = popped.ptr;
						changed = true;
						break;
					}
				}

				if (changed) continue;

				if (hit_stack.empty()) {
					if (hit) {
						hit_point = ray.direction * tmin + ray.origin;
					}
					return hit;
				}
			}
	}

bool BVH::Traverse(Ray& ray) {  

			float tmp;
			BVHNode* currentNode = nodes[0];

			if (!currentNode->getAABB().intercepts(ray, tmp)) {
				return false;
			}

			while (true) {
				if (!currentNode->isLeaf()) {
					BVHNode* l_node = nodes[currentNode->getIndex()];
					BVHNode* r_node = nodes[currentNode->getIndex() + 1];
					float l_t, r_t;

					bool l_hit = l_node->getAABB().intercepts(ray, l_t);
					bool r_hit = r_node->getAABB().intercepts(ray, r_t);

					if (l_hit && r_hit) {
						if (l_t < r_t) {
							currentNode = l_node;
							// push l to stack
							hit_stack.push(StackItem(r_node, r_t));
						}
						else {
							currentNode = r_node;
							// push l to stack
							hit_stack.push(StackItem(l_node, l_t));
						}
						continue;
					}
					else if (l_hit) {
						currentNode = l_node;
						continue;
					}
					else if (r_hit) {
						currentNode = r_node;
						continue;
					}
				}
				else {
					Object* obj;
					float curr_t;
					for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
						obj = objects[i];
						if (obj->intercepts(ray, curr_t)) {
							return true;
						}
					}
				}

				bool changed = false;

				while (!hit_stack.empty()) {
					StackItem popped = hit_stack.top();
					hit_stack.pop();
					currentNode = popped.ptr;
					changed = true;
				}

				if (changed) continue;

				if (hit_stack.empty()) { return false; }
			}
}
