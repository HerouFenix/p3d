#define GLSLIFY 1
/**
* ver hash functions em
* https://www.shadertoy.com/view/XlGcRh hash functions GPU
* http://www.jcgt.org/published/0009/03/02/
 */

 #define GLSLIFY 1
/**
 * common.glsl
 * Common types and functions used for ray tracing.
 */

const float pi = 3.14159265358979;
const float epsilon = 0.001;

struct Ray {
    vec3 o;     // origin
    vec3 d;     // direction - always set with normalized vector
    float t;    // time, for motion blur
};

Ray createRay(vec3 o, vec3 d, float t)
{
    Ray r;
    r.o = o;
    r.d = d;
    r.t = t;
    return r;
}

Ray createRay(vec3 o, vec3 d)
{
    return createRay(o, d, 0.0);
}

vec3 pointOnRay(Ray r, float t)
{
    return r.o + r.d * t;
}

float gSeed = 0.0;

uint baseHash(uvec2 p)
{
    p = 1103515245U * ((p >> 1U) ^ (p.yx));
    uint h32 = 1103515245U * ((p.x) ^ (p.y>>3U));
    return h32 ^ (h32 >> 16);
}

float hash1(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    return float(n) / float(0xffffffffU);
}

vec2 hash2(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    uvec2 rz = uvec2(n, n * 48271U);
    return vec2(rz.xy & uvec2(0x7fffffffU)) / float(0x7fffffff);
}

vec3 hash3(inout float seed)
{
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1, seed += 0.1)));
    uvec3 rz = uvec3(n, n * 16807U, n * 48271U);
    return vec3(rz & uvec3(0x7fffffffU)) / float(0x7fffffff);
}

float rand(vec2 v)
{
    return fract(sin(dot(v.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 toLinear(vec3 c)
{
    return pow(c, vec3(2.2));
}

vec3 toGamma(vec3 c)
{
    return pow(c, vec3(1.0 / 2.2));
}

vec2 randomInUnitDisk(inout float seed) {
    vec2 h = hash2(seed) * vec2(1.0, 6.28318530718);
    float phi = h.y;
    float r = sqrt(h.x);
	return r * vec2(sin(phi), cos(phi));
}

vec3 randomInUnitSphere(inout float seed)
{
    vec3 h = hash3(seed) * vec3(2.0, 6.28318530718, 1.0) - vec3(1.0, 0.0, 0.0);
    float phi = h.y;
    float r = pow(h.z, 1.0/3.0);
	return r * vec3(sqrt(1.0 - h.x * h.x) * vec2(sin(phi), cos(phi)), h.x);
}

struct Camera
{
    vec3 eye;
    vec3 u, v, n;
    float width, height;
    float lensRadius;
    float planeDist, focusDist;
    float time0, time1;
};

Camera createCamera(
    vec3 eye,
    vec3 at,
    vec3 worldUp,
    float fovy,
    float aspect,
    float aperture,  //diametro em multiplos do pixel size
    float focusDist,  //focal ratio
    float time0,
    float time1)
{
    Camera cam;
    if(aperture == 0.0) cam.focusDist = 1.0; //pinhole camera then focus in on vis plane
    else cam.focusDist = focusDist;
    vec3 w = eye - at;
    cam.planeDist = length(w);
    cam.height = 2.0 * cam.planeDist * tan(fovy * pi / 180.0 * 0.5);
    cam.width = aspect * cam.height;

    cam.lensRadius = aperture * 0.5 * cam.width / iResolution.x;  //aperture ratio * pixel size; (1 pixel=lente raio 0.5)
    cam.eye = eye;
    cam.n = normalize(w);
    cam.u = normalize(cross(worldUp, cam.n));
    cam.v = cross(cam.n, cam.u);
    cam.time0 = time0;
    cam.time1 = time1;
    return cam;
}

Ray getRay(Camera cam, vec2 pixel_sample)  //rnd pixel_sample viewport coordinates
{
    vec2 ls = cam.lensRadius * randomInUnitDisk(gSeed);  //ls - lens sample for DOF
    float time = cam.time0 + hash1(gSeed) * (cam.time1 - cam.time0);
    
    vec3 rayTarget = vec3((gl_FragCoord.xy/iResolution.xy) * 2.0f - 1.0f, 1.0f);
    vec3 rayDir = normalize(rayTarget - cam.eye);

    return createRay(cam.eye, normalize(rayTarget - cam.eye), time);
}

// MT_ material type
#define MT_DIFFUSE 0
#define MT_METAL 1
#define MT_DIALECTRIC 2

struct Material
{
    int type;
    vec3 albedo;
    float roughness; // controls roughness for metals
    float refIdx; // index of refraction for dialectric
};

Material createDiffuseMaterial(vec3 albedo)
{
    Material m;
    m.type = MT_DIFFUSE;
    m.albedo = albedo;
    return m;
}

Material createMetalMaterial(vec3 albedo, float roughness)
{
    Material m;
    m.type = MT_METAL;
    m.albedo = albedo;
    m.roughness = roughness;
    return m;
}

Material createDialectricMaterial(vec3 albedo, float refIdx)
{
    Material m;
    m.type = MT_DIALECTRIC;
    m.albedo = albedo;
    m.refIdx = refIdx;
    return m;
}

struct HitRecord
{
    vec3 pos;
    vec3 normal;
    float t;            // ray parameter
    Material material;
};

float schlick(float cosine, float refIdx)
{
    //INSERT YOUR CODE HERE
    return 0.0f;
}

bool scatter(Ray rIn, HitRecord rec, out vec3 atten, out Ray rScattered)
{
    if(rec.material.type == MT_DIFFUSE)
    {
        //INSERT CODE HERE,
        atten = rec.material.albedo * max(dot(rScattered.d, rec.normal), 0.0) / pi;
        return true;
    }
    if(rec.material.type == MT_METAL)
    {
       //INSERT CODE HERE, consider fuzzy reflections
        atten = rec.material.albedo;
        return true;
    }
    if(rec.material.type == MT_DIALECTRIC)
    {
        atten = rec.material.albedo;
        vec3 outwardNormal;
        float niOverNt;
        float cosine;

        if(dot(rIn.d, rec.normal) > 0.0) //hit inside
        {
            outwardNormal = -rec.normal;
            niOverNt = rec.material.refIdx;
            cosine = rec.material.refIdx * dot(rIn.d, rec.normal); 
        }
        else  //hit from outside
        {
            outwardNormal = rec.normal;
            niOverNt = 1.0 / rec.material.refIdx;
            cosine = -dot(rIn.d, rec.normal); 
        }

        //Use probabilistic math to decide if scatter a reflected ray or a refracted ray

        float reflectProb;

        //if no total reflection  reflectProb = schlick(cosine, rec.material.refIdx);  
        //else reflectProb = 1.0;

        if( hash1(gSeed) < reflectProb)  //Reflection
        // rScattered = calculate reflected ray
          // atten *= vec3(reflectProb); not necessary since we are only scattering reflectProb rays and not all reflected rays
        
        //else  //Refraction
        // rScattered = calculate refracted ray
           // atten *= vec3(1.0 - reflectProb); not necessary since we are only scattering 1-reflectProb rays and not all refracted rays
        return true;
    }
    return false;
}

struct pointLight {
    vec3 pos;
    vec3 color;
};

pointLight createPointLight(vec3 pos, vec3 color) 
{
    pointLight l;
    l.pos = pos;
    l.color = color;
    return l;
}

struct Triangle {vec3 a; vec3 b; vec3 c; };

Triangle createTriangle(vec3 v0, vec3 v1, vec3 v2)
{
    Triangle t;
    t.a = v0; t.b = v1; t.c = v2;
    return t;
}

bool hit_triangle(Triangle tg, Ray r, float tmin, float tmax, out HitRecord rec)
{

    //INSERT YOUR CODE HERE
    //calculate a valid t and normal

    vec3 normal = vec3(0.0f);
    float t = 0.0f;

    // https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
	// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

	vec3 v0 = tg.a, v1 = tg.b, v2 = tg.c;

	// Find vectors for two edges sharing v0 
	vec3 edge1 = v1 - v0;
	vec3 edge2 = v2 - v0;

	vec3 pVec = vec3(0, 0, 0);
	pVec.x = (r.d.y * edge2.z) - (r.d.z * edge2.y);
	pVec.y = (r.d.z * edge2.x) - (r.d.x * edge2.z);
	pVec.z = (r.d.x * edge2.y) - (r.d.y * edge2.x);

    normal = normalize(pVec);

	float det = dot(edge1, pVec);

	if (det > -0.0000001f && det < 0.0000001f) return false; // ray and triangle are parallel if det is close to 0

	float inv_det = 1.0f / det;

	// Calculate distance from v0 to ray origin
	vec3 tVec = r.o - v0;

	// Calculate U parameter and test bounds 
	float u = inv_det * (dot(tVec, pVec));
	if (u < 0.0f || u > 1.0f) return false;

	vec3 qVec = vec3(0, 0, 0);
	qVec.x = (tVec.y * edge1.z) - (tVec.z * edge1.y);
	qVec.y = (tVec.z * edge1.x) - (tVec.x * edge1.z);
	qVec.z = (tVec.x * edge1.y) - (tVec.y * edge1.x);

	// Calculate V parameter and test bounds
	float v = inv_det * (dot(r.d, qVec));
	if (v < 0.0f || u + v > 1.0f) return false;

	// Calculate t, scale parameters, ray intersects triangle
	t = inv_det * (dot(edge2,qVec));

	if (t > 0.0000001f){
        if(t < tmax && t > tmin){
            rec.t = t;
            rec.normal = normal;
            rec.pos = pointOnRay(r, rec.t);
            return true;
        }
    }
    return false;
}

struct Sphere
{
    vec3 center;
    float radius;
};

Sphere createSphere(vec3 center, float radius)
{
    Sphere s;
    s.center = center;
    s.radius = radius;
    return s;
}

struct MovingSphere
{
    vec3 center0, center1;
    float radius;
    float time0, time1;
};

MovingSphere createMovingSphere(vec3 center0, vec3 center1, float radius, float time0, float time1)
{
    MovingSphere s;
    s.center0 = center0;
    s.center1 = center1;
    s.radius = radius;
    s.time0 = time0;
    s.time1 = time1;
    return s;
}

vec3 center(MovingSphere mvsphere, float time)
{
    return mvsphere.center0 + (mvsphere.center1 - mvsphere.center0) * ((time - mvsphere.time0) / (mvsphere.time1 - mvsphere.time0));
}

/*
 * The function naming convention changes with these functions to show that they implement a sort of interface for
 * the book's notion of "hittable". E.g. hit_<type>.
 */

bool hit_sphere(Sphere s, Ray r, float tmin, float tmax, out HitRecord rec)
{
    //INSERT YOUR CODE HERE
    //calculate a valid t and normal
	
    float t = 0.0f;

    // https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes
	vec3 L = s.center - r.o;
	//float a = r.direction * r.direction;
	float b = dot(L, r.d);
	float c = dot(L, L) - s.radius * s.radius;

	if (c > 0.0f) {
		if (b < 0.0f) {
			return false;
		}
	}

	float discriminant = (b * b - c);

	if (discriminant < 0.0f) {
		return false;
	}

	if (c > 0.0f) {
		t = b - sqrt(discriminant);
	}
	else {
		t = b + sqrt(discriminant);
	}

    if(t < tmax && t > tmin) {
        rec.t = t;
        rec.pos = pointOnRay(r, rec.t);
        rec.normal = normalize(rec.pos - s.center);
        return true;
    }
    else return false;
}

bool hit_movingSphere(MovingSphere s, Ray r, float tmin, float tmax, out HitRecord rec)
{
     //INSERT YOUR CODE HERE
     //calculate a valid t and normal

      float t = 0.0f;

    // https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes
	vec3 L = center(s, r.t) - r.o;
	//float a = r.direction * r.direction;
	float b = dot(L, r.d);
	float c = dot(L, L) - s.radius * s.radius;

	if (c > 0.0f) {
		if (b < 0.0f) {
			return false;
		}
	}

	float discriminant = (b * b - c);

	if (discriminant < 0.0f) {
		return false;
	}

	if (c > 0.0f) {
		t = b - sqrt(discriminant);
	}
	else {
		t = b + sqrt(discriminant);
	}
	
    if(t < tmax && t > tmin) {
        rec.t = t;
        rec.pos = pointOnRay(r, rec.t);
        rec.normal = normalize(rec.pos - center(s, r.t));
        return true;
    }
    else return false;
    
}

 #iChannel0 "self"

bool hit_world(Ray r, float tmin, float tmax, out HitRecord rec)
{
    bool hit = false;
    rec.t = tmax;
   
    if(hit_triangle(createTriangle(vec3(-10.0, -0.01, 10.0), vec3(10.0, -0.01, 10.0), vec3(-10.0, -0.01, -10.0)), r, tmin, rec.t, rec))
    {
        hit = true;
        rec.material = createDiffuseMaterial(vec3(0.2));
    }

    if(hit_triangle(createTriangle(vec3(-10.0, -0.01, -10.0), vec3(10.0, -0.01, 10), vec3(10.0, -0.01, -10.0)), r, tmin, rec.t, rec))
    {
        hit = true;
        rec.material = createDiffuseMaterial(vec3(0.2));
    }

    if(hit_sphere(
        createSphere(vec3(-4.0, 1.0, 0.0), 1.0),
        r,
        tmin,
        rec.t,
        rec))
    {
        hit = true;
        rec.material = createDiffuseMaterial(vec3(0.2, 0.95, 0.1));
        //rec.material = createDiffuseMaterial(vec3(0.4, 0.2, 0.1));
    }

    if(hit_sphere(
        createSphere(vec3(4.0, 1.0, 0.0), 1.0),
        r,
        tmin,
        rec.t,
        rec))
    {
        hit = true;
        rec.material = createMetalMaterial(vec3(0.7, 0.6, 0.5), 0.0);
    }

    if(hit_sphere(
        createSphere(vec3(0.0, 1.0, 0.0), 1.0),
        r,
        tmin,
        rec.t,
        rec))
    {
        hit = true;
        rec.material = createDialectricMaterial(vec3(1.0), 1.5);
    }

    if(hit_sphere(
        createSphere(vec3(0.0, 1.0, 0.0), -0.95),
        r,
        tmin,
        rec.t,
        rec))
    {
        hit = true;
        rec.material = createDialectricMaterial(vec3(1.0), 1.5);
    }
   
    int numxy = 5;
    
    for(int x = -numxy; x < numxy; ++x)
    {
        for(int y = -numxy; y < numxy; ++y)
        {
            float fx = float(x);
            float fy = float(y);
            float seed = fx + fy / 1000.0;
            vec3 rand1 = hash3(seed);
            vec3 center = vec3(fx + 0.9 * rand1.x, 0.2, fy + 0.9 * rand1.y);
            float chooseMaterial = rand1.z;
            if(distance(center, vec3(4.0, 0.2, 0.0)) > 0.9)
            {
                if(chooseMaterial < 0.3)
                {
                    vec3 center1 = center + vec3(0.0, hash1(gSeed) * 0.5, 0.0);
                    // diffuse
                    if(hit_movingSphere(
                        createMovingSphere(center, center1, 0.2, 0.0, 1.0),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                        rec.material = createDiffuseMaterial(hash3(seed) * hash3(seed));
                    }
                }
                else if(chooseMaterial < 0.5)
                {
                    // diffuse
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                        rec.material = createDiffuseMaterial(hash3(seed) * hash3(seed));
                    }
                }
                else if(chooseMaterial < 0.7)
                {
                    // metal
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                       // rec.material.type = MT_METAL;
                        rec.material = createMetalMaterial((hash3(seed) + 1.0) * 0.5, 0.0);
                    }
                }
                else if(chooseMaterial < 0.9)
                {
                    // metal
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                       // rec.material.type = MT_METAL;
                        rec.material = createMetalMaterial((hash3(seed) + 1.0) * 0.5, hash1(seed));
                    }
                }
                else
                {
                    // glass (dialectric)
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                        rec.material.type = MT_DIALECTRIC;
                        rec.material = createDialectricMaterial(hash3(seed), 1.5);
                    }
                }
            }
        }
    }
    return hit;
}

vec3 directlighting(pointLight pl, Ray r, HitRecord rec){
    vec3 diffCol, specCol;
    vec3 colorOut = vec3(0.0, 0.0, 0.0);
    float shininess;
    HitRecord dummy;

   //INSERT YOUR CODE HERE
    
	return colorOut; 
}

#define MAX_BOUNCES 10

vec3 rayColor(Ray r)
{
    HitRecord rec;
    vec3 col = vec3(0.0);
    vec3 throughput = vec3(1.0f, 1.0f, 1.0f);
    for(int i = 0; i < MAX_BOUNCES; ++i)
    {
        if(hit_world(r, 0.001, 10000.0, rec))
        {
            //calculate direct lighting with 3 white point lights:
            {
                //createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0))
                //createPointLight(vec3(8.0, 15.0, 3.0), vec3(1.0, 1.0, 1.0))
                //createPointLight(vec3(1.0, 15.0, -9.0), vec3(1.0, 1.0, 1.0))

                //for instance: col += directlighting(createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
            }
           
            //calculate secondary ray and update throughput
            Ray scatterRay;
            vec3 atten;
            if(scatter(r, rec, atten, scatterRay))
            {   //  insert your code here    }
        
        }
        else  //background
        {
            float t = 0.8 * (r.d.y + 1.0);
            col += throughput * mix(vec3(1.0), vec3(0.5, 0.7, 1.0), t);
            break;
        }
    }
    return col;
}

#define MAX_SAMPLES 10000.0

void main(){
    gSeed = float(baseHash(floatBitsToUint(gl_FragCoord.xy))) / float(0xffffffffU) + iTime;

    vec2 mouse = iMouse.xy / iResolution.xy;
    mouse.x = mouse.x * 2.0 - 1.0;

    vec3 camPos = vec3(mouse.x * 10.0, mouse.y * 5.0, 8.0);
    vec3 camTarget = vec3(0.0, 0.0, -1.0);
    float fovy = 60.0;
    float aperture = 0.0;
    float distToFocus = 2.5;
    float time0 = 0.0;
    float time1 = 1.0;
    Camera cam = createCamera(
        camPos,
        camTarget,
        vec3(0.0, 1.0, 0.0),    // world up vector
        fovy,
        iResolution.x / iResolution.y,
        aperture,
        distToFocus,
        time0,
        time1);

    //usa-se o 4 canal de cor para guardar o numero de samples e não o iFrame pois quando se mexe o rato faz-se reset
    
    vec4 prev = texture(iChannel0, gl_FragCoord.xy / iResolution.xy);
    vec3 prevLinear = toLinear(prev.xyz);  

    vec2 ps = gl_FragCoord.xy + hash2(gSeed);
    //vec2 ps = gl_FragCoord.xy;
    vec3 color = rayColor(getRay(cam, ps));

    if(iMouseButton.x != 0.0 || iMouseButton.y != 0.0)
    {
        gl_FragColor = vec4(toGamma(color), 1.0);  //samples number reset = 1
        return;
    }
    if(prev.w > MAX_SAMPLES)   
    {
        gl_FragColor = prev;
        return;
    }

    float w = prev.w + 1.0;
    color = mix(prevLinear, color, 1.0/w);
    gl_FragColor = vec4(toGamma(color), w);
}
