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

vec3 RandomUnitVector(inout float seed)
{
    float z = hash1(seed) * 2.0f - 1.0f;
    float a = hash1(seed) * 2.0f * pi;
    float r = sqrt(1.0f - z * z);
    float x = r * cos(a);
    float y = r * sin(a);
    return vec3(x, y, z);
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
    
    //Calculate eye_offset and ray direction
    //vec3 rayTarget = vec3((gl_FragCoord.xy/iResolution.xy) * 2.0f - 1.0f, 1.0f);
    //vec3 rayDir = normalize(rayTarget - cam.eye);
    
    vec3 p = vec3(cam.width * (pixel_sample.x / iResolution.x - 0.5f) * cam.focusDist, cam.height * (pixel_sample.y / iResolution.y - 0.5f) * cam.focusDist, 0);

    vec3 ray_dir = vec3(cam.u * (p.x - ls.x) + cam.v * (p.y - ls.y) + cam.n * (-cam.focusDist * cam.planeDist));
    vec3 eye_offset = cam.eye + (cam.u * ls.x) + (cam.v * ls.y); 

    return createRay(eye_offset, normalize(ray_dir), time);
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


float schlick(float cosOi, float ior_1, float ior_2)
{
    //INSERT YOUR CODE HERE
    float r0 = pow(((ior_1 - ior_2) / (ior_1 + ior_2)), 2.0);
	float Kr = r0 + (1.0 - r0) * pow(1.0 - cosOi, 5.0);
    return Kr;
}

bool scatter(Ray rIn, HitRecord rec, out vec3 atten, out Ray rScattered)
{
    if(rec.material.type == MT_DIFFUSE)
    {
        //INSERT CODE HERE,
        //vec3 rayPos = (rIn.o + rIn.d * rec.t) + rec.normal * epsilon;

        // scattering a secondary ray in a random direction within an hemisphere to accomplish the color bleeding effect
        //vec3 rayDir = normalize(rec.normal + RandomUnitVector(gSeed));
        //vec3 rayDir = normalize(rec.normal + hash3(gSeed));
        
        vec3 S = rec.pos + rec.normal + normalize(randomInUnitSphere(gSeed));
        vec3 rayDir = normalize(S - rec.pos);

        rScattered = createRay(rec.pos, rayDir, rIn.t);

        atten = rec.material.albedo * max(dot(rScattered.d, rec.normal), 0.0) / pi;
        return true;
    }
    if(rec.material.type == MT_METAL)
    {
        //INSERT CODE HERE, consider fuzzy reflections
        vec3 rayDir = reflect(rIn.d, rec.normal);

        // Fuzzy Reflections -  (rDir + (sample_unit_sphere() * ROUGHNESS)).normalize()
        rayDir = normalize(rayDir + (randomInUnitSphere(gSeed) * rec.material.roughness));

        rScattered = createRay(rec.pos, rayDir, rIn.t);

        atten = rec.material.albedo;
        return true;
    }
    if(rec.material.type == MT_DIALECTRIC)
    {
        atten = rec.material.albedo;
        vec3 outNormal;
        float n, cosine, kr;
        bool inside = false;

	    if (dot(rIn.d, rec.normal) > 0.0) {
		    inside = true;
		    outNormal = rec.normal * -1.0;
            cosine = rec.material.refIdx * dot(rIn.d, rec.normal);
            n = rec.material.refIdx / 1.0;
            kr = schlick(cosine, rec.material.refIdx, 1.0);
            
	    } else{
            inside = false;
            outNormal = rec.normal;
            cosine = -dot(rIn.d, rec.normal);
            n = 1.0 / rec.material.refIdx;
            kr = schlick(cosine, 1.0, rec.material.refIdx); 
        }

        if(hash1(gSeed) < kr){ // Reflection
            vec3 rayDir = reflect(rIn.d, outNormal);
            rayDir = normalize(rayDir + (randomInUnitSphere(gSeed) * rec.material.roughness));
            rScattered = createRay(rec.pos, rayDir, rIn.t);
        } else{ // Refraction
            vec3 view = rIn.d * -1.0; 
	        vec3 viewNormal = (outNormal * dot(view, outNormal));
	        vec3 viewTangent = viewNormal - view;
            float cosOi = length(viewNormal);
            float sinOt = n * length(viewTangent);
            float insqrt = 1.0 - pow(sinOt, 2.0);
            if(insqrt >= 0.0){
                float cosOt = sqrt(insqrt);
                vec3 tDir = normalize((normalize(viewTangent) * sinOt + outNormal * normalize(-cosOt)));
                vec3 intercept = rec.pos + rec.normal * epsilon;
                rScattered = createRay(rec.pos, tDir, rIn.t);
            }
        }  
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
