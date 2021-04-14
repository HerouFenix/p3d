#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>

#include "maths.h"
#include "scene.h"
#include <ctime>

Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	/* Calculate the normal */
	Vector V = P1 - P0;
	Vector W = P2 - P0;

	normal = Vector(0, 0, 0);
	normal.x = (V.y * W.z) - (V.z * W.y);
	normal.y = (V.z * W.x) - (V.x * W.z);
	normal.z = (V.x * W.y) - (V.y * W.x);

	normal.normalize();

	//Calculate the Min and Max for bounding box
	float x0 = min(min(P0.x, P1.x), P2.x);
	float y0 = min(min(P0.y, P1.y), P2.y);
	float z0 = min(min(P0.z, P1.z), P2.z);

	float x1 = max(max(P0.x, P1.x), P2.x);
	float y1 = max(max(P0.y, P1.y), P2.y);
	float z1 = max(max(P0.z, P1.z), P2.z);

	Min = Vector(x0, y0, z0);
	Max = Vector(x1, y1, z1);


	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

int Triangle::GetObjectType()
{
	return 1;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray& r, float& t) {
	if (USE_MAILBOX) {
		if (mailbox_id >= r.id) return false;
		mailbox_id = r.id;
	}

	// https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
	// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

	Vector v0 = points[0], v1 = points[1], v2 = points[2];

	// Find vectors for two edges sharing v0 
	Vector edge1 = v1 - v0;
	Vector edge2 = v2 - v0;

	Vector pVec = Vector(0, 0, 0);
	pVec.x = (r.direction.y * edge2.z) - (r.direction.z * edge2.y);
	pVec.y = (r.direction.z * edge2.x) - (r.direction.x * edge2.z);
	pVec.z = (r.direction.x * edge2.y) - (r.direction.y * edge2.x);
	
	float det = edge1 * pVec;

	if (det > -0.0000001f && det < 0.0000001f) return false; // ray and triangle are parallel if det is close to 0
	
	float inv_det = 1.0f / det;

	// Calculate distance from v0 to ray origin
	Vector tVec = r.origin - v0;
	
	// Calculate U parameter and test bounds 
	float u = inv_det * (tVec * pVec);
	if (u < 0.0f || u > 1.0f) return false;
	
	Vector qVec = Vector(0, 0, 0);
	qVec.x = (tVec.y * edge1.z) - (tVec.z * edge1.y);
	qVec.y = (tVec.z * edge1.x) - (tVec.x * edge1.z);
	qVec.z = (tVec.x * edge1.y) - (tVec.y * edge1.x);
	
	// Calculate V parameter and test bounds
	float v = inv_det * (r.direction * qVec);
	if (v < 0.0f || u + v > 1.0f) return false;
	
	// Calculate t, scale parameters, ray intersects triangle
	t = inv_det * (edge2 * qVec);
	if (t > 0.0000001f) return true;
	
	return false;
}


Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

/* NO CLUE WHAT D IS OR HOW TO COMPUTE IT SO INSTEAD I CHANGED THE PLANE REPRESENTATION TO BE A POINT AND A NORMAL
Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
	float l;

	//Calculate the normal plane: counter-clockwise vectorial product.
	PN = Vector(0, 0, 0);

	if ((l = PN.length()) == 0.0)
	{
		cerr << "DEGENERATED PLANE!\n";
	}
	else
	{
		PN.normalize();
		//Calculate D
		D = 0.0f;
	}
}
*/
Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
	float l;

	//Calculate the normal plane: counter-clockwise vectorial product.
	PN = Vector(0, 0, 0);
	Vector a = P2 - P1;
	Vector b = P0 - P1;

	PN = a % b;
	
	if ((l = PN.length()) == 0.0)
	{
		cerr << "DEGENERATED PLANE!\n";
	}
	else
	{
		PN.normalize();

		P = P0;
	}
}


//
// Ray/Plane intersection test.
//

bool Plane::intercepts(Ray& r, float& t)
{
	if (USE_MAILBOX) {
		if (mailbox_id >= r.id) return false;
		mailbox_id = r.id;
	}

	float numer = (r.origin - P) * PN;
	float divid = PN * r.direction;

	if (fabs(divid) < EPSILON) {
		return false;
	}

	t = -(numer / divid);

	if (t <= 0) {
		return false;
	}

	return true;
}

Vector Plane::getNormal(Vector point)
{
	return PN;
}

int Plane::GetObjectType()
{
	return 0;
}

bool Sphere::intercepts(Ray& r, float& t)
{
	if (USE_MAILBOX) {
		if (mailbox_id >= r.id) return false;
		mailbox_id = r.id;
	}

	// https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes

	Vector L = center - r.origin;
	//float a = r.direction * r.direction;
	float b = L * r.direction;
	float c = L * L - radius * radius;

	if (c > 0) {
		if (b < 0) {
			return false;
		}
	}

	float discriminant = (b * b - c);

	if (discriminant < 0) {
		return false;
	}

	if (c > 0) {
		t = b - sqrt(discriminant);
	}
	else {
		t = b + sqrt(discriminant);
	}

	return true;
}


Vector Sphere::getNormal(Vector point)
{
	Vector normal = point - center;
	return (normal.normalize());
}

int Sphere::GetObjectType()
{
	return 2;
}

AABB Sphere::GetBoundingBox() {
	Vector a_min = (center - Vector(radius, radius, radius));
	Vector a_max = (center + Vector(radius, radius, radius));
	return(AABB(a_min, a_max));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

int aaBox::GetObjectType()
{
	return 3;
}

bool aaBox::intercepts(Ray& ray, float& t)
{
	if (USE_MAILBOX) {
		if (mailbox_id >= ray.id) return false;
		mailbox_id = ray.id;
	}

	float tmin, tmax, tymin, tymax, tzmin, tzmax;

	float a = 1.0f / ray.direction.x;
	if (a >= 0) {
		tmin = (min.x - ray.origin.x) * a;
		tmax = (max.x - ray.origin.x) * a;
	}
	else {
		tmin = (max.x - ray.origin.x) * a;
		tmax = (min.x - ray.origin.x) * a;
	}

	float b = 1.0f / ray.direction.y;
	if (b >= 0) {
		tymin = (min.y - ray.origin.y) * b;
		tymax = (max.y - ray.origin.y) * b;
	}
	else {
		tymin = (max.y - ray.origin.y) * b;
		tymax = (min.y - ray.origin.y) * b;
	}


	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;


	float c = 1.0f / ray.direction.z;
	if (c >= 0) {
		tzmin = (min.z - ray.origin.z) * c;
		tzmax = (max.z - ray.origin.z) * c;
	}
	else {
		tzmin = (max.z - ray.origin.z) * c;
		tzmax = (min.z - ray.origin.z) * c;
	}
	
	if ((tmin > tzmax) || (tzmin > tmax))
		return false;

	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;

	t = tmin;

	if (t < 0) {
		if (tmax < 0) return false;
		t = tmax;
	}

	return true;
}

Vector aaBox::getNormal(Vector point)
{
	Vector center = (max + min) / 2;
	Vector co = point - center;
	int dir; 

	if (fabs(co.x) > fabs(co.y)) {
		if (fabs(co.z) > fabs(co.x)) {
			if (co.z >= 0) Normal = Vector(0, 0, 1);
			else Normal = Vector(0, 0, -1);
		}
		else {
			if (co.x >= 0) Normal = Vector(1, 0, 0);
			else Normal = Vector(-1, 0, 0);
		}
	}
	else {
		if (fabs(co.z) > fabs(co.y)) {
			if (co.z >= 0) Normal = Vector(0, 0, 1);
			else Normal = Vector(0, 0, -1);
		}
		else {
			if (co.y >= 0) Normal = Vector(0, 1, 0);
			else Normal = Vector(0, -1, 0);
		}
	}

	return Normal;
}

Vector aaBox::getCentroid() {
	return (max + min) / 2;
}

Scene::Scene()
{}

Scene::~Scene()
{
	/*for ( int i = 0; i < objects.size(); i++ )
	{
		delete objects[i];
	}
	objects.erase();
	*/
}

int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}


int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadSkybox(const char* sky_dir)
{
	char* filenames[6];
	char buffer[100];
	const char* maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char*)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}

	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte*)malloc(size);
		ILubyte* bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char* token, const char* name)
{
	file >> token;
	if (strcmp(token, name))
		cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char* name)
{
	const	int	lineSize = 1024;
	string	cmd;
	char		token[256];
	ifstream	file(name, ios::in);
	Material* material;

	material = NULL;

	if (file >> cmd)
	{
		while (true)
		{

			if (cmd == "f")   //Material
			{
				double Kd, Ks, Shine, T, ior;
				Color cd, cs;

				file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

				material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
			}

			else if (cmd == "s")    //Sphere
			{
				Vector center;
				float radius;
				Sphere* sphere;

				file >> center >> radius;
				sphere = new Sphere(center, radius);
				if (material) sphere->SetMaterial(material);
				this->addObject((Object*)sphere);
			}
			else if (cmd == "sm")    //Motion Sphere - EXTRA ASSIGNMENT - MOTION BLUR
			{
				Vector center0, center1;
				float radius;
				float t0, t1;
				MovingSphere* movingSphere;

				file >> center0 >> center1 >> radius >> t0 >> t1;
				movingSphere = new MovingSphere(center0, center1, radius, t0, t1);
				if (material) movingSphere->SetMaterial(material);
				this->addObject((Object*)movingSphere);
			}

			else if (cmd == "box")    //axis aligned box
			{
				Vector minpoint, maxpoint;
				aaBox* box;

				file >> minpoint >> maxpoint;
				box = new aaBox(minpoint, maxpoint);
				if (material) box->SetMaterial(material);
				this->addObject((Object*)box);
			}
			else if (cmd == "p")  // Polygon: just accepts triangles for now
			{
				Vector P0, P1, P2;
				Triangle* triangle;
				unsigned total_vertices;

				file >> total_vertices;
				if (total_vertices == 3)
				{
					file >> P0 >> P1 >> P2;
					triangle = new Triangle(P0, P1, P2);
					if (material) triangle->SetMaterial(material);
					this->addObject((Object*)triangle);
				}
				else
				{
					cerr << "Unsupported number of vertices.\n";
					break;
				}
			}

			else if (cmd == "mesh") {
				unsigned total_vertices, total_faces;
				unsigned P0, P1, P2;
				Triangle* triangle;
				Vector* verticesArray, vertex;

				file >> total_vertices >> total_faces;
				verticesArray = (Vector*)malloc(total_vertices * sizeof(Vector));
				for (int i = 0; i < total_vertices; i++) {
					file >> vertex;
					verticesArray[i] = vertex;
				}
				for (int i = 0; i < total_faces; i++) {
					file >> P0 >> P1 >> P2;
					triangle = new Triangle(verticesArray[P0 - 1], verticesArray[P1 - 1], verticesArray[P2 - 1]); //vertex index start at 1
					if (material) triangle->SetMaterial(material);
					this->addObject((Object*)triangle);
				}

			}

			else if (cmd == "pl")  // General Plane
			{
				Vector P0, P1, P2;
				Plane* plane;

				file >> P0 >> P1 >> P2;
				plane = new Plane(P0, P1, P2);
				if (material) plane->SetMaterial(material);
				this->addObject((Object*)plane);
			}

			else if (cmd == "l")  // Need to check light color since by default is white
			{
				Vector pos;
				Color color;

				file >> pos >> color;

				this->addLight(new Light(pos, color));

			}
			else if (cmd == "v")
			{
				Vector up, from, at;
				float fov, hither;
				int xres, yres;
				Camera* camera;
				float focal_ratio; //ratio beteween the focal distance and the viewplane distance
				float aperture_ratio; // number of times to be multiplied by the size of a pixel

				next_token(file, token, "from");
				file >> from;

				next_token(file, token, "at");
				file >> at;

				next_token(file, token, "up");
				file >> up;

				next_token(file, token, "angle");
				file >> fov;

				next_token(file, token, "hither");
				file >> hither;

				next_token(file, token, "resolution");
				file >> xres >> yres;

				next_token(file, token, "aperture");
				file >> aperture_ratio;

				next_token(file, token, "focal");
				file >> focal_ratio;
				// Create Camera
				camera = new Camera(from, at, up, fov, hither, 100.0 * hither, xres, yres, aperture_ratio, focal_ratio);
				this->SetCamera(camera);
			}

			else if (cmd == "bclr")   //Background color
			{
				Color bgcolor;
				file >> bgcolor;
				this->SetBackgroundColor(bgcolor);
			}

			else if (cmd == "env")
			{
				file >> token;

				this->LoadSkybox(token);
				this->SetSkyBoxFlg(true);
			}
			else if (cmd[0] == '#')
			{
				file.ignore(lineSize, '\n');
			}
			else
			{
				cerr << "unknown command '" << cmd << "'.\n";
				break;
			}
			if (!(file >> cmd))
				break;
		}
	}

	file.close();
	return true;
};

void Scene::create_random_scene() {
	Camera* camera;
	Material* material;
	Sphere* sphere;

	set_rand_seed(time(NULL) * time(NULL) * time(NULL));
	material = NULL;
	this->SetSkyBoxFlg(false);  //init with no skybox

	this->SetBackgroundColor(Color(0.5, 0.7, 1.0));
	//this->LoadSkybox("skybox");
	//this->SetSkyBoxFlg(true);

	camera = new Camera(Vector(13.0, 2.0, 3.0), Vector(0.0, 0.0, 0), Vector(0.0, 1.0, 0.0), 45.0, 0.01, 10000.0, 800, 600, 0, 1.5f);
	this->SetCamera(camera);

	this->addLight(new Light(Vector(7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(-7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(0, 10, 7), Color(1.0, 1.0, 1.0)));

	material = new Material(Color(0.5, 0.5, 0.5), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);


	sphere = new Sphere(Vector(0.0, -1000, 0.0), 1000.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	for (int a = -5; a < 5; a++)
		for (int b = -5; b < 5; b++) {

			double choose_mat = rand_double();

			Vector center = Vector(a + 0.9 * rand_double(), 0.2, b + 0.9 * rand_double());

			if ((center - Vector(4.0, 0.2, 0.0)).length() > 0.9) {
				if (choose_mat < 0.4) {  //diffuse
					material = new Material(Color(rand_double(), rand_double(), rand_double()), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else if (choose_mat < 0.9) {   //metal
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(rand_double(0.5, 1), rand_double(0.5, 1), rand_double(0.5, 1)), 1.0, 220, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else {   //glass 
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}

			}

		}

	material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
	sphere = new Sphere(Vector(0.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.9, Color(1.0, 1.0, 1.0), 0.1, 10, 0, 1.0);
	sphere = new Sphere(Vector(-4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.0, Color(0.7, 0.6, 0.5), 1.0, 220, 0, 1.0);
	sphere = new Sphere(Vector(4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);
}


// EXTRA ASSIGNMENT - MOTION BLUR //
bool MovingSphere::intercepts(Ray& r, float& t)
{
	if (USE_MAILBOX) {
		if (mailbox_id >= r.id) return false;
		mailbox_id = r.id;
	}

	// https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes

	setCenter(r.time);
	Vector L = center - r.origin;
	//float a = r.direction * r.direction;
	float b = L * r.direction;
	float c = L * L - radius * radius;

	if (c > 0) {
		if (b < 0) {
			return false;
		}
	}

	float discriminant = (b * b - c);

	if (discriminant < 0) {
		return false;
	}

	if (c > 0) {
		t = b - sqrt(discriminant);
	}
	else {
		t = b + sqrt(discriminant);
	}

	return true;
}

Vector MovingSphere::getNormal(Vector point)
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB MovingSphere::GetBoundingBox(void)
{
	Vector a_min = (center - Vector(radius, radius, radius));
	Vector a_max = (center + Vector(radius, radius, radius));
	return(AABB(a_min, a_max));
}

int MovingSphere::GetObjectType()
{
	return 4;
}

void MovingSphere::setCenter(float time)
{
	center = center0 + (center1 - center0) * ((time - time0) / (time1 - time0));
}

void Light::setColour(Color& col)
{
	color = col;
}

void Light::setPosition(Vector& pos)
{
	position = pos;
}
