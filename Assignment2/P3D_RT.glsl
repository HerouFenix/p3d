/**
* ver hash functions em
* https://www.shadertoy.com/view/XlGcRh hash functions GPU
* http://www.jcgt.org/published/0009/03/02/
 */

#include "./common.glsl"
#iKeyboard
#iChannel0 "self"

bool USE_RUSSIAN_ROULETTE = false;
bool ORBIT_CAMERA = true;

bool SHOWCASE_DOF = false;
bool SHOWCASE_FUZZYREFL = false;
bool SHOWCASE_FUZZYREFR = false;
bool NO_NEGATIVE_SPHERE = false;

#iUniform float fovy = 60.0 in { 0.0, 120.0 } // This will expose a slider to edit the value ; It would've been better to be able to store the current fov in a channel, but our 4th channel is already occupied with the frame counter, and i can't figure out how to create a texture on a new channel

bool hit_world(Ray r, float tmin, float tmax, out HitRecord rec) {
    bool hit = false;
    rec.t = tmax;

    if(hit_triangle(createTriangle(vec3(-10.0, -0.01, 10.0), vec3(10.0, -0.01, 10.0), vec3(-10.0, -0.01, -10.0)), r, tmin, rec.t, rec)) {
        hit = true;
        rec.material = createDiffuseMaterial(vec3(0.2));
    }

    if(hit_triangle(createTriangle(vec3(-10.0, -0.01, -10.0), vec3(10.0, -0.01, 10), vec3(10.0, -0.01, -10.0)), r, tmin, rec.t, rec)) {
        hit = true;
        rec.material = createDiffuseMaterial(vec3(0.2));
    }

    // Left Ball
    if(hit_sphere(createSphere(vec3(-4.0, 1.0, 0.0), 1.0), r, tmin, rec.t, rec)) {
        hit = true;
        //rec.material = createDiffuseMaterial(vec3(0.2, 0.95, 0.1)); // Green
        rec.material = createDiffuseMaterial(vec3(0.4, 0.2, 0.1)); // Brown
    }

    // Right Ball
    if(hit_sphere(createSphere(vec3(4.0, 1.0, 0.0), 1.0), r, tmin, rec.t, rec)) {
        hit = true;
        if(!SHOWCASE_FUZZYREFL){
            rec.material = createMetalMaterial(vec3(0.7, 0.6, 0.5), 0.0);
        }else{
            rec.material = createMetalMaterial(vec3(0.7, 0.6, 0.5), 0.2); // Fuzzy Reflection
        }
    }

    // Center Ball (out)
    if(hit_sphere(createSphere(vec3(0.0, 1.0, 0.0), 1.0), r, tmin, rec.t, rec)) {
        hit = true;
        if(!SHOWCASE_FUZZYREFR){
            rec.material = createDialectricMaterial(vec3(1.0), 1.5, 0.0, 0.0); 
        }else{
            rec.material = createDialectricMaterial(vec3(1.0), 1.5, 0.0, 0.2); // Fuzzy Refraction
        }
    }

    // Center Ball (in)
    if(!NO_NEGATIVE_SPHERE){
        if(hit_sphere(
            createSphere(vec3(0.0, 1.0, 0.0), -0.95),
            //createSphere(vec3(0.0, 1.0, 0.0), -0.45),
        r, tmin, rec.t, rec)) {
            hit = true;
            if(!SHOWCASE_FUZZYREFR){
                rec.material = createDialectricMaterial(vec3(1.0), 1.5, 0.0, 0.0);
            }else{
                rec.material = createDialectricMaterial(vec3(1.0), 1.5, 0.0, 0.2); // Fuzzy Refraction
            }
        }     
    }

    int numxy = 5;

    for(int x = -numxy; x < numxy; ++x) {
        for(int y = -numxy; y < numxy; ++y) {
            float fx = float(x);
            float fy = float(y);
            float seed = fx + fy / 1000.0;
            vec3 rand1 = hash3(seed);
            vec3 center = vec3(fx + 0.9 * rand1.x, 0.2, fy + 0.9 * rand1.y);
            float chooseMaterial = rand1.z;
            if(distance(center, vec3(4.0, 0.2, 0.0)) > 0.9) {
                if(chooseMaterial < 0.3) {
                    vec3 center1 = center + vec3(0.0, hash1(gSeed) * 0.5, 0.0);
                    // diffuse
                    if(hit_movingSphere(createMovingSphere(center, center1, 0.2, 0.0, 1.0), r, tmin, rec.t, rec)) {
                        hit = true;
                        rec.material = createDiffuseMaterial(hash3(seed) * hash3(seed));
                    }
                } else if(chooseMaterial < 0.5) {
                    // diffuse
                    if(hit_sphere(createSphere(center, 0.2), r, tmin, rec.t, rec)) {
                        hit = true;
                        rec.material = createDiffuseMaterial(hash3(seed) * hash3(seed));
                    }
                } else if(chooseMaterial < 0.7) {
                    // metal
                    if(hit_sphere(createSphere(center, 0.2), r, tmin, rec.t, rec)) {
                        hit = true;
                       // rec.material.type = MT_METAL;
                        rec.material = createMetalMaterial((hash3(seed) + 1.0) * 0.5, 0.0);
                    }
                } else if(chooseMaterial < 0.9) {
                    // metal
                    if(hit_sphere(createSphere(center, 0.2), r, tmin, rec.t, rec)) {
                        hit = true;
                       // rec.material.type = MT_METAL;
                        rec.material = createMetalMaterial((hash3(seed) + 1.0) * 0.5, hash1(seed));
                    }
                } else {
                    // glass (dialectric)
                    if(hit_sphere(createSphere(center, 0.2), r, tmin, rec.t, rec)) {
                        hit = true;
                        rec.material.type = MT_DIALECTRIC;
                        rec.material = createDialectricMaterial(hash3(seed), 1.5, 0.0, 0.0);
                    }
                }
            }
        }
    }
    return hit;
}

vec3 directlighting(pointLight pl, Ray r, HitRecord rec) {
    vec3 diffCol, specCol;
    vec3 colorOut = vec3(0.0, 0.0, 0.0);
    float shininess;
    HitRecord dummy;

    float diffuse, specular;

   //INSERT YOUR CODE HERE
    vec3 L = (pl.pos - rec.pos);
    if(dot(L, rec.normal) > 0.0) {
        Ray feeler = createRay(rec.pos + epsilon * rec.normal, normalize(L));
        float len = length(pl.pos - rec.pos);

        if(hit_world(feeler, 0.0, len, dummy)) // If true, then we're in shadow. Return color as 0
        {
            return colorOut;
        }

        if(rec.material.type == MT_DIFFUSE) {
            specCol = vec3(0.1);
            diffCol = rec.material.albedo;
            shininess = 10.0;

            diffuse = 1.0;
            specular = 0.0;
        } else if(rec.material.type == MT_METAL) {
            specCol = rec.material.albedo;
            diffCol = vec3(0.0);
            shininess = 100.0;

            diffuse = 0.0;
            specular = 1.0;
        } else { // Dialletric Materials
            specCol = vec3(0.004);
            diffCol = vec3(0.0);
            shininess = 100.0;

            diffuse = 0.0;
            specular = 1.0;
        }

        L = normalize(L);
        vec3 H = normalize((L - r.d));

        diffCol = (pl.color * diffCol) * max(dot(rec.normal, L), 0.0);
        specCol = (pl.color * specCol) * pow(max(dot(H, rec.normal), 0.0), shininess);

        colorOut = diffCol * diffuse + specCol * specular;
    }

    return colorOut;
}

#define MAX_BOUNCES 10

vec3 rayColor(Ray r) {
    HitRecord rec;
    vec3 col = vec3(0.0);
    vec3 throughput = vec3(1.0f, 1.0f, 1.0f);
    for(int i = 0; i < MAX_BOUNCES; ++i) {
        if(hit_world(r, 0.001, 10000.0, rec)) {
            //calculate direct lighting with 3 white point lights:
            /* DIRECT LIGHTING */
            {
                //createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0))
                //createPointLight(vec3(8.0, 15.0, 3.0), vec3(1.0, 1.0, 1.0))
                //createPointLight(vec3(1.0, 15.0, -9.0), vec3(1.0, 1.0, 1.0))

                //for instance: col += directlighting(createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
                col += directlighting(createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
                col += directlighting(createPointLight(vec3(8.0, 15.0, 3.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
                col += directlighting(createPointLight(vec3(1.0, 15.0, -9.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
            }

            //calculate secondary ray and update throughput
            /* INDIRECT LIGHTING */
            Ray scatterRay;
            vec3 atten;
            if(scatter(r, rec, atten, scatterRay)) // Indirect Lighting
            {   //  insert your code here    
                // a secondary ray will be scattered which means that the initial ray will be simply 
                // overwritten with the result of the ray produced by the scattering event before the next 
                // loop iteration and the throughput of this new ray will be updated by multiplying it by the 
                // object’s albedo.

                r = scatterRay;
                throughput *= atten; 

                // Russian Roulette - https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
                // Smaller throughput => higher probability to terminate early (p is the max throughput value so the smaller it is, the higher the probability hash1(gseed) will generate a higher number)
                // Since we're eventually going to terminate early, boost the throughput value by the average to make up for the smaller number of samples
                if(USE_RUSSIAN_ROULETTE) {
                    float p = max(throughput.x, max(throughput.y, throughput.z));
                    if(hash1(gSeed) > p)
                        break;

                    throughput *= 1.0 / p;
                }

            }

        } else  //background
        {
            float t = 0.8 * (r.d.y + 1.0);
            col += throughput * mix(vec3(1.0), vec3(0.5, 0.7, 1.0), t);
            break;
        }
    }
    return col;
}

#define MAX_SAMPLES 10000.0

void main() {
    gSeed = float(baseHash(floatBitsToUint(gl_FragCoord.xy))) / float(0xffffffffU) + iTime;

    vec2 mouse = iMouse.xy / iResolution.xy;

    vec3 camPos;
    vec3 camTarget = vec3(0.0, 0.0, -1.0); // Camera At

    float camDist = 8.0; 

    // ORBIT CAMERA (CGJ PTSD)//
    if(!ORBIT_CAMERA) {
        mouse.x = mouse.x * 2.0 - 1.0;
        camPos = vec3(mouse.x * 10.0, mouse.y * 5.0, camDist); // Z -> Camera distance
    } else {
        if (dot(mouse, vec2(1.0f, 1.0f)) == 0.0f)
        { // Default position of camera
            camPos = vec3(0.0f, 0.0f, -camDist);
        }else{
            // calculate 2d coordinates of the ray target on the imaginary pixel plane.
            // -1 to +1 on each axis.
            mouse.x = mouse.x * 2.0 - 1.0;

            float minAngle = 0.01;
            float maxAngle = pi - 0.01;
            float sensitivity = 5.0;

            float angleX = -mouse.x * sensitivity;
            float angleY = mix(minAngle, maxAngle, mouse.y);

            camPos = vec3(sin(angleX) * sin(angleY) * camDist, -cos(angleY) * camDist, cos(angleX) * sin(angleY) * camDist);

            camPos += camTarget;
        }
    }

    //float fovy = 60.0;

    float aperture;
    float distToFocus;
    if(!SHOWCASE_DOF){
        /*Default (no dof)*/
        aperture = 0.0;
        distToFocus = 2.5;
    }else{
        /*Alternative (high dof, close objects in focus)*/
        distToFocus = 0.5;
        aperture = 10.0;
    }
    
    float time0 = 0.0;
    float time1 = 1.0;
    Camera cam = createCamera(camPos, camTarget, vec3(0.0, 1.0, 0.0),    // world up vector
    fovy, iResolution.x / iResolution.y, aperture, distToFocus, time0, time1);

    //usa-se o 4 canal de cor para guardar o numero de samples e não o iFrame pois quando se mexe o rato faz-se reset

    vec4 prev = texture(iChannel0, gl_FragCoord.xy / iResolution.xy);
    vec3 prevLinear = toLinear(prev.xyz);

    vec2 ps = gl_FragCoord.xy + hash2(gSeed);
    //vec2 ps = gl_FragCoord.xy;
    vec3 color = rayColor(getRay(cam, ps));

    if(iMouseButton.x != 0.0 || iMouseButton.y != 0.0 ) {
        gl_FragColor = vec4(toGamma(color), 1.0);  //samples number reset = 1
        return;
    }
    if(prev.w > MAX_SAMPLES) {
        gl_FragColor = prev;
        return;
    }

    float w = prev.w + 1.0;
    color = mix(prevLinear, color, 1.0 / w);
    gl_FragColor = vec4(toGamma(color), w);
}
