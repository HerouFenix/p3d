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
#include "grid.h"
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
bool ANTIALIASING = false;
int SPP = 4; // (sqrt) Sample Per Pixel - (sqrt) Number of rays called for each pixel

bool SOFT_SHADOWS = true;
int NO_LIGHTS = 4; // (sqrt) Number of point lights used to represent area light
///////////////////////////////////////////

template<typename Base, typename T>
inline bool instanceof(const T*) {
	return is_base_of<Base, T>::value;
}

void processLight(Light light, Color& color, Material material, Ray ray, Vector hit_point, Vector normal) {
	Object* obj = NULL;
	Vector L;
	float cur_dist = FLT_MAX;

	// Get L - the unit vector from the hit point to the light source
	L = (light.position - hit_point).normalize();

	// If dot product of L and the normal is bigger than zero
	if (L * normal > 0) {
		// Check if point is NOT in shadow
		Ray feeler = Ray(hit_point, L); // Ray going from the intersection point pointing to the light
		bool in_shadow = false;

		// Iterate over all objects to see if any are between the intersection point and the light source
		for (int j = 0; j < scene->getNumObjects(); j++) {
			obj = scene->getObject(j);

			if (obj->intercepts(feeler, cur_dist)) {

				in_shadow = true;
				break;
			}
		}

		// If point not in shadow
		if (!in_shadow) {
			Vector H = ((L + (ray.direction * -1))).normalize();

			Color diff = (light.color * material.GetDiffColor()) * (max(0, normal * L));
			Color spec = (light.color * material.GetSpecColor()) * pow(max(0, H * normal), material.GetShine());

			//color = diffuse color + specular color
			color += (diff * material.GetDiffuse() + spec * material.GetSpecular());
		}
	}
}

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	Color color = Color(0.0f, 0.0f, 0.0f);

	Object* obj = NULL;
	Object* closest_obj = NULL;
	float cur_dist, min_dist = FLT_MAX;

	//Intersect Ray with all objects and find the closest hit point (if it exists)
	for (int i = 0; i < scene->getNumObjects(); i++) {
		obj = scene->getObject(i);

		if (obj->intercepts(ray, cur_dist) && (cur_dist < min_dist)) {
			closest_obj = obj;
			min_dist = cur_dist;
		}
	}

	// If no intersection, return BACKGROUND or SKYBOX if applicable
	if (closest_obj == NULL) {
		if (scene->GetSkyBoxFlg())
			return scene->GetSkyboxColor(ray);
		else
			return scene->GetBackgroundColor();
	}

	Vector hit_point = ray.origin + ray.direction * min_dist;

	// Hitpoint computation with offset (remove Acne)
	Vector precise_hit_point = hit_point + closest_obj->getNormal(hit_point) * .0001;


	// Compute normal at the hit point
	Vector normal = closest_obj->getNormal(precise_hit_point).normalize();

	Light* light = NULL;
	Vector L;
	Material* material = closest_obj->GetMaterial();
	int objType = closest_obj->GetObjectType(); // Get the closest object type (for DEBUGING purposes)

	// For each light source
	for (int i = 0; i < scene->getNumLights(); i++) {
		light = scene->getLight(i);

		// LAB 3: SOFT SHADOWS //
		if (SOFT_SHADOWS) {
			if (!ANTIALIASING) {
				// represent the area light as a distributed set of N point lights, each with one Nth of the intensity of the base light
				float size = 0.5f; // "Size" of each light (e.g in a 4x4 grid, what's the size of each cell

				float dist = size / NO_LIGHTS;
				// Start at the top left corner of the grid 
				float cur_x = light->position.x - dist * NO_LIGHTS/2; // TODO: MATHEMATICALLY CHECK IF THIS IS DOING WHAT I THINK ITS DOING
				float cur_y = light->position.y - dist * NO_LIGHTS/2;

				Color avg_col = light->color / (NO_LIGHTS * NO_LIGHTS);
				Light* newLight = NULL;

				// Iterate over each line of the grid
				for (int y = 0; y < NO_LIGHTS; y++) {
					// Iterate over each column of the grid
					for (int x = 0; x < NO_LIGHTS; x++) {
						Vector pos = Vector(cur_x, cur_y, light->position.z);
						newLight = new Light(pos, avg_col);

						processLight(*newLight, color, *material, ray, precise_hit_point, normal);

						cur_x += dist; // Move to next column
					}
					cur_y += dist;
					cur_x = light->position.x - dist * NO_LIGHTS/2;
				}
			}
		}
		else {
			processLight(*light, color, *material, ray, precise_hit_point, normal);
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

	float Kr; // Reflection mix value 
	Vector view = ray.direction * -1; // View
	Vector viewNormal = (normal * (view * normal)); // ViewNormal
	Vector viewTangent = viewNormal - view; // ViewTangent

	// REFRACTION //
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
			Vector intercept = hit_point + tDir * .0001;

			Ray tRay = Ray(intercept, tDir);

			float newIoR = !inside ? material->GetRefrIndex() : 1;

			// compute refracted color using recursion (tColor = rayTracing(refracted ray direction, depth+1)
			tColor = rayTracing(tRay, depth + 1, newIoR);

			// Fresnel Equations https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
			Rparal = pow(fabs((ior_1 * cosOt - newIoR * cosOi) / (ior_1 * cosOt + newIoR * cosOi)), 2); // parallel
			Rperp = pow(fabs((ior_1 * cosOi - newIoR * cosOt) / (ior_1 * cosOi + newIoR * cosOt)), 2); // perpendicular
		}

		// ratio of reflected ligth (mirror reflection attenuation)
		Kr = 1 / 2 * (Rperp + Rparal);
	}
	else { // Material is opaque
		Kr = material->GetSpecular();
	}


	// REFLECTION //
	if (material->GetReflection() > 0) {
		//	compute ray in the reflected direction
		// https://www.scratchapixel.com/code.php?id=3&origin=/lessons/3d-basic-rendering/introduction-to-ray-tracing
		Vector rDir = ray.direction - normal * 2 * (ray.direction * normal);

		Ray rRay = Ray(precise_hit_point, rDir);

		//	compute reflection color using recursion (rColor = rayTracing(reflected ray direction, depth+1))
		rColor = rayTracing(rRay, depth + 1, ior_1);
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

	// Set random seed for this iteration
	set_rand_seed(time(NULL)); // https://www.cplusplus.com/reference/cstdlib/srand/


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

				Ray ray = scene->GetCamera()->PrimaryRay(pixel);
				color = rayTracing(ray, 1, 1.0).clamp();
			}
			else {
				// LAB 3: ANTIALIASING [JITTER] //

				for (int p = 0; p < SPP; p++) {
					for (int q = 0; q < SPP; q++) {
						pixel.x = x + (p + rand_float()) / SPP;
						pixel.y = y + (q + rand_float()) / SPP;

						Ray ray = scene->GetCamera()->PrimaryRay(pixel);
						color += rayTracing(ray, 1, 1.0).clamp();
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