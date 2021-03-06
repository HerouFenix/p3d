///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "sampler.h"

#define CAPTION "Whitted Ray-Tracer"

#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define MAX_DEPTH 6

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];

//Enable OpenGL drawing.  
bool drawModeEnabled = true;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float* colors;
float* vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t* img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;
int RES_X, RES_Y;

int WindowHandle = 0;

/* OPTIONS *///////////////////////////////
float SHADOW_BIAS = 0.001f;
bool SCHLICK_APPROX = false;

bool ANTIALIASING = true;
int SPP = 4; // (sqrt) Sample Per Pixel - (sqrt) Number of rays called for each pixel

bool SOFT_SHADOWS = true;
int NO_LIGHTS = 4; // (sqrt) Number of point lights used to represent area light (NOTE: SHOULD BE THE SAME AS SPP TO PRESENT SIMILAR EFFECTS WHEN WITH/WITHOUT ANTIALIASING)
int off_x, off_y; // Used to cause a more even distribution when using soft shadows + antialiasing

bool DEPTH_OF_FIELD = true;

bool FUZZY_REFLECTIONS = false;
float ROUGHNESS = 0.1f;

bool MOTION_BLUR = true;
float t0 = 0.0f, t1 = 1.0f; // Camera shutter time
///////////////////////////////////////////

/* ACCELERATION STRUCTURES *///////////////
int USE_ACCEL_STRUCT = 2; // 0 - No acceleration structure ; 1 - Uniform Grid ; 2 - Bounding Volume Hierarchy

Grid uGrid;
int Ray::next_id = 0; // For Mailboxing

BVH bvh;
///////////////////////////////////////////

template<typename Base, typename T>
inline bool instanceof(const T*) {
	return is_base_of<Base, T>::value;
}

void processLight(Vector L, Color& lightColor, Color& color, Material material, Ray ray, Vector hit_point, Vector normal) {
	Object* obj = NULL;
	float cur_dist = FLT_MAX;

	// If dot product of L and the normal is bigger than zero
	if (L * normal > 0) {
		// Check if point is NOT in shadow
		Ray feeler = Ray(hit_point, L); // Ray going from the intersection point pointing to the light
		double length;

		bool in_shadow = false;

		// LAB 4: ACCELERATION STRUCTURES //
		switch (USE_ACCEL_STRUCT) {
		case 0: // No acceleration structure
			length = feeler.direction.length(); //distance between light and intersection point
			feeler.direction.normalize();

			// Iterate over all objects to see if any are between the intersection point and the light source
			for (int j = 0; j < scene->getNumObjects(); j++) {
				obj = scene->getObject(j);

				if (obj->intercepts(feeler, cur_dist) && cur_dist < length) {

					in_shadow = true;
					break;
				}
			}
			break;

		case 1: // Uniform Grid
			// Traverse Grid
			if (uGrid.Traverse(feeler)) { // If the ray hits smth when traversing, we're in shadow
				in_shadow = true;
			}
			break;

		case 2: // BVH
			// Traverse BVH
			if (bvh.Traverse(feeler)) {
				in_shadow = true;
			}
			break;

		default:
			// Iterate over all objects to see if any are between the intersection point and the light source
			for (int j = 0; j < scene->getNumObjects(); j++) {
				obj = scene->getObject(j);

				if (obj->intercepts(feeler, cur_dist)) {

					in_shadow = true;
					break;
				}
			}
			break;
		}

		// If point not in shadow
		if (!in_shadow) {
			L = L.normalize();

			Vector H = ((L + (ray.direction * -1))).normalize();

			Color diff = (lightColor * material.GetDiffColor()) * (max(0, normal * L));
			Color spec = (lightColor * material.GetSpecColor()) * pow(max(0, H * normal), material.GetShine());

			//color = diffuse color + specular color
			color += (diff * material.GetDiffuse() + spec * material.GetSpecular());
		}
	}
}


Vector random_in_unit_sphere() {
	while (true) {
		auto p = Vector(rand_float(-1, 1), rand_float(-1, 1), rand_float(-1, 1));
		if (p.length() >= 1) continue;
		return p;
	}
}

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	Color color = Color(0.0f, 0.0f, 0.0f);

	Object* obj = NULL;
	Object* closest_obj = NULL;
	float cur_dist, min_dist = FLT_MAX;

	Vector hit_point;

	// LAB 4: ACCELERATION STRUCTURES //
	switch (USE_ACCEL_STRUCT) {
	case 0: // No acceleration structure
		//Intersect Ray with all objects and find the closest hit point (if it exists)
		for (int i = 0; i < scene->getNumObjects(); i++) {
			obj = scene->getObject(i);

			if (obj->intercepts(ray, cur_dist) && (cur_dist < min_dist)) {
				closest_obj = obj;
				min_dist = cur_dist;
			}
		}

		if (closest_obj != NULL) {
			hit_point = ray.origin + ray.direction * min_dist;
		}

		break;

	case 1: // Uniform Grid

		// Traverse Grid
		if (!uGrid.Traverse(ray, &closest_obj, hit_point)) {
			closest_obj = NULL;
		}

		break;

	case 2: // BHV

		// Traverse BVH
		if (!bvh.Traverse(ray, &closest_obj, hit_point)) {
			closest_obj = NULL;
		}
		break;

	default:
		//Intersect Ray with all objects and find the closest hit point (if it exists)
		for (int i = 0; i < scene->getNumObjects(); i++) {
			obj = scene->getObject(i);

			if (obj->intercepts(ray, cur_dist) && (cur_dist < min_dist)) {
				closest_obj = obj;
				min_dist = cur_dist;
			}
		}
		break;
	}

	// If no intersection, return BACKGROUND or SKYBOX if applicable
	if (closest_obj == NULL) {
		if (scene->GetSkyBoxFlg())
			return scene->GetSkyboxColor(ray);
		else
			return scene->GetBackgroundColor();
	}

	// Hitpoint computation with offset (remove Acne)
	Vector precise_hit_point = hit_point + closest_obj->getNormal(hit_point) * SHADOW_BIAS;


	// Compute normal at the hit point
	Vector normal = closest_obj->getNormal(precise_hit_point).normalize();

	Light* light = NULL;
	Vector L;
	Material* material = closest_obj->GetMaterial();
	//int objType = closest_obj->GetObjectType(); // Get the closest object type (for DEBUGING purposes)

	// For each light source
	for (int i = 0; i < scene->getNumLights(); i++) {
		light = scene->getLight(i);
		Vector pos;

		// LAB 3: SOFT SHADOWS //
		if (SOFT_SHADOWS) {
			float size = 0.5f; // size of the light jitter

			if (!ANTIALIASING) {// TODO: CHECK IF COMPUTING THE NEWLIGHT OUTSIDE THE RAYTRACING FUNCTION HAS BETTER PERFORMANCE (pq assim, smpr q formos buscar uma luz tamos a computar a area light toda again, this probably only needs to be done once)
				// represent the area light as a distributed set of N point lights, each with one Nth of the intensity of the base light

				float dist = size / NO_LIGHTS; // Distance between lights in light area

				// Start at the top left corner of the grid
				float cur_x = light->position.x - dist * NO_LIGHTS * 0.5f;
				float cur_y = light->position.y - dist * NO_LIGHTS * 0.5f;

				Color avg_col = light->color / (NO_LIGHTS * NO_LIGHTS);

				// Iterate over each line of the grid
				for (int y = 0; y < NO_LIGHTS; y++) {
					// Iterate over each column of the grid
					for (int x = 0; x < NO_LIGHTS; x++) {
						pos = Vector(cur_x, cur_y, light->position.z);

						// Get L - the unit vector from the hit point to the light source
						Vector L = (pos - hit_point);

						processLight(L, avg_col, color, *material, ray, precise_hit_point, normal);

						cur_x += dist; // Move to next column
					}
					cur_y += dist; // Go down a row
					cur_x = light->position.x - dist * NO_LIGHTS * 0.5f; // Go back to left col
				}
			}
			else {
				// represent the area light as an infinite number of point lights and choose one at random for each primary ray
				// Instead of picking randomly, consider that the area of light is also subdivided by the jitter offset
				//pos = Vector(
				//	rand_float(light->position.x-size/2, light->position.x + size/2), // Pick random x within area of light -> x + size goes to max right of area * rand float divides by number to pick any position from x to x+size.
				//	rand_float(light->position.y - size/2, light->position.y + size/2), // Pick random y within area of light
				//	light->position.z);

				//pos = Vector(
				//		rand_float(light->position.x- size/2 * off_x / SPP, light->position.x + size/2 * off_x / SPP), // Pick random x within area of light -> x + size goes to max right of area * rand float divides by number to pick any position from x to x+size. add offset
				//		rand_float(light->position.y - size/2 * off_y / SPP, light->position.y + size/2 * off_y / SPP), // Pick random y within area of light
				//		light->position.z);

				pos = Vector(
					light->position.x + size * ((off_x + rand_float()) / SPP), // Pick random x within area of light -> using the same formula we use when creating a pixel sample (pixel.x = x + (p + rand_float()) / SPP;) but taking the light jitter size into account (size)
					light->position.y + size * ((off_y + rand_float()) / SPP), // Pick random y within area of light
					light->position.z);


				// Get L - the unit vector from the hit point to the light source
				Vector L = (pos - hit_point);
				
				processLight(L, light->color, color, *material, ray, precise_hit_point, normal);
			}
		}
		else {
			// Get L - the unit vector from the hit point to the light source
			Vector L = (light->position - hit_point);
			processLight(L, light->color, color, *material, ray, precise_hit_point, normal);
		}
	}

	// If depth >= max depth, stop casting indirect lighting.
	if (depth >= MAX_DEPTH) {
		return color.clamp();
	}


	// LAB 2: REFLECTION & REFRACTION //

	Color tColor = Color(), rColor = Color();

	// https://www.scratchapixel.com/code.php?id=3&origin=/lessons/3d-basic-rendering/introduction-to-ray-tracing 

	bool inside = false; // Check if our ray is traveling within the object

	if (ray.direction * normal > 0) {
		inside = true;
		normal = normal * -1;
	}

	// REFLECTION //
	if (material->GetReflection() > 0) {
		//	compute ray in the reflected direction
		// https://www.scratchapixel.com/code.php?id=3&origin=/lessons/3d-basic-rendering/introduction-to-ray-tracing
		Vector rDir = ray.direction - normal * 2 * (ray.direction * normal);

		// ASSIGNMENT EXTRA - FUZZY REFLECTIONS //
		https://raytracing.github.io/books/RayTracingInOneWeekend.html#metal/fuzzyreflection
		Ray* rRay = nullptr;
		if (FUZZY_REFLECTIONS)
			rRay = &Ray(precise_hit_point, (rDir + (sample_unit_sphere() * ROUGHNESS)).normalize());
		else
			rRay = &Ray(precise_hit_point, rDir);

		//	compute reflection color using recursion (rColor = rayTracing(reflected ray direction, depth+1))
		rColor = rayTracing(*rRay, depth + 1, ior_1);
	}


	// REFRACTION //
	float Kr; // Reflection mix value 
	Vector view = ray.direction * -1; // View
	Vector viewNormal = (normal * (view * normal)); // ViewNormal
	Vector viewTangent = viewNormal - view; // ViewTangent

	if (material->GetTransmittance() > 0) {
		float Rperp = 1, Rparal = 1;

		// Change rerfraction index depending of we're traveling inside or not (Snell Law)
		float n = !inside ? ior_1 / material->GetRefrIndex() : ior_1;

		// https://www.scratchapixel.com/code.php?id=3&origin=/lessons/3d-basic-rendering/introduction-to-ray-tracing
		float cosOi = viewNormal.length();
		float sinOt = (n)*viewTangent.length();
		float insqrt = 1 - pow(sinOt, 2);

		if (insqrt >= 0) {
			float cosOt = sqrt(insqrt);

			// compute ray in the refracted direction
			Vector tDir = (viewTangent.normalize() * sinOt + normal * (-cosOt)).normalize();
			Vector intercept = hit_point + tDir * SHADOW_BIAS;

			Ray tRay = Ray(intercept, tDir);

			float newIoR = !inside ? material->GetRefrIndex() : 1; // New Indice of Refraction

			// compute refracted color using recursion (tColor = rayTracing(refracted ray direction, depth+1)
			tColor = rayTracing(tRay, depth + 1, newIoR);

			// Schlick Approximation
			if (SCHLICK_APPROX) {
				float r0 = pow(((ior_1 - newIoR) / (ior_1 + newIoR)), 2);
				Kr = r0 + (1 - r0) * pow(1 - cosOi, 5);
			}
			else {
				// Fresnel Equations https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
				Rparal = pow(fabs((ior_1 * cosOt - newIoR * cosOi) / (ior_1 * cosOt + newIoR * cosOi)), 2); // parallel
				Rperp = pow(fabs((ior_1 * cosOi - newIoR * cosOt) / (ior_1 * cosOi + newIoR * cosOt)), 2); // perpendicular
			}
		}

		if(!SCHLICK_APPROX || insqrt < 0){ // If we're not using schlick always do this, else if we're using schlick only do this if insqrt is < 0
			// ratio of reflected ligth (mirror reflection attenuation)
			Kr = 1 / 2 * (Rperp + Rparal);
		}

	}
	else { // Material is opaque
		Kr = material->GetSpecular();
	}

	//	reduce rColor and tColor by the reflection mix value and add to color
	color += rColor * Kr * material->GetSpecColor() + tColor * (1 - Kr);

	return color.clamp();
}

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte* errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if (isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");

	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs


void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);

	// unbind the VAO
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
	//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X * RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char* filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}

	// EXTRA ASSIGNMENT - MOTION BLUR //
	if (MOTION_BLUR) {
		scene->GetCamera()->SetShutterTime(t0, t1);
	}

	// Set random seed for this iteration
	set_rand_seed(time(NULL)); // https://www.cplusplus.com/reference/cstdlib/srand/

	// LAB 4: ACCELERATION STRUCTURES //
	if (USE_ACCEL_STRUCT == 1) { // Uniform Grid
		uGrid = Grid(); // Build Grid

		vector<Object*> objects;

		for (int i = 0; i < scene->getNumObjects(); i++) {
			objects.push_back(scene->getObject(i));
			//uGrid.addObject(scene->getObject(i));
		}

		uGrid.Build(objects);
	}
	else if (USE_ACCEL_STRUCT == 2) { // BVH
		bvh = BVH(); // Build BVH

		vector<Object*> objects;

		for (int i = 0; i < scene->getNumObjects(); i++) {
			objects.push_back(scene->getObject(i));
		}

		bvh.Build(objects);
	}


	// For each pixel
	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;

			Vector pixel;  //viewport coordinates

			/* NO ANTI ALIASING */
			if (!ANTIALIASING) {
				pixel.x = x + 0.5f;
				pixel.y = y + 0.5f;

				Ray* ray = nullptr;

				// LAB 3: DEPTH OF FIELD //
				if (DEPTH_OF_FIELD) {
					Vector lens;
					float aperture = scene->GetCamera()->GetAperture();

					// Compute the sample point on the "thin lens" 
					lens = sample_unit_disk() * aperture;

					ray = &scene->GetCamera()->PrimaryRay(lens, pixel, MOTION_BLUR);
				}
				else {
					ray = &scene->GetCamera()->PrimaryRay(pixel, MOTION_BLUR);
				}

				color = rayTracing(*ray, 1, 1.0).clamp();
			}
			else {
				// LAB 3: ANTIALIASING [JITTER] //
				for (int p = 0; p < SPP; p++) {
					for (int q = 0; q < SPP; q++) {
						off_x = p;
						off_y = q;
						pixel.x = x + (p + rand_float()) / SPP;
						pixel.y = y + (q + rand_float()) / SPP;

						Ray* ray = nullptr;

						// LAB 3: DEPTH OF FIELD //
						if (DEPTH_OF_FIELD) {
							Vector lens;
							float aperture = scene->GetCamera()->GetAperture();

							// Compute the sample point on the "thin lens"
							lens = sample_unit_disk() * aperture;

							ray = &scene->GetCamera()->PrimaryRay(lens, pixel, MOTION_BLUR);
						}
						else {
							ray = &scene->GetCamera()->PrimaryRay(pixel, MOTION_BLUR);
						}


						color += rayTracing(*ray, 1, 1.0).clamp();
					}
				}

				color = color / (SPP * SPP);
			}

			//color = scene->GetBackgroundColor(); //TO CHANGE - just for the template

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}
		}

	}
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}

// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top,
	float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

	case 27:
		glutLeaveMainLoop();
		break;

	case 'r':
		camX = Eye.x;
		camY = Eye.y;
		camZ = Eye.z;
		r = Eye.length();
		beta = asinf(camY / r) * 180.0f / 3.14f;
		alpha = atanf(camX / camZ) * 180.0f / 3.14f;
		break;

	case 'c':
		printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
		printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
		break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


/////////////////////////////////////////////////////////////////////// SETUP

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}

void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit();
	if (result != GLEW_OK) {
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	}
	GLenum err_code = glGetError();
	printf("Vendor: %s\n", glGetString(GL_VENDOR));
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("Version: %s\n", glGetString(GL_VERSION));
	printf("GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);

	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if (WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();

}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}
	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X * RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);
}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int ch;
	if (!drawModeEnabled) {

		do {
			init_scene();
			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while ((toupper(ch) == 'Y'));
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X * RES_Y * sizeof(float);
		size_colors = 3 * RES_X * RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////