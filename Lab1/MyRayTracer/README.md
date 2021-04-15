

# P3D Assignment 1 - MyRayTracer

### Authors
-   **Diogo Silva** - [HerouFenix](https://github.com/HerouFenix)
-   **Daniel Gonçalves** - [4nd3l1](https://github.com/4nd3l1)
-   **Henrique Gaspar** - [HenriqueMetas](https://github.com/HenriqueMetas)

## Installation & Usage
The project has been built as a VSStudio solution. As such, to compile and run the program one must simply:

 1. Open the solution by double clicking on **MyRayTracer.sln** (make sure you have VSStudio installed).
 2. Within VSStudio, on the Solution Explorer tab, navigate to the **MyRayTracer** project
 3. Select **x64** on the platform (top bar) and **Release**
 4. Right click on the **MyRayTracer** project and select *Build*
 5. Finally, start the program by clicking on *Local Windows Debugger* on the top bar (Play)


## Options
There are several options for distributed ray-tracing techniques and acceleration structures that may be toggled on or off within the code. All of the following can be found in the file **main.cpp**:
 - **ANTIALIASING**
	 - Values: *true*, *false*
	 - Toggles the application of Antialiasing through jittering.
	 - Additionally, the variable **SPP** can also be changed to swap between the number of samples are created for each pixel. Higher values imply less performance but a sharper image.
 - **SOFT_SHADOWS**
	 - Values: *true*, *false*
	 - Toggles the application of Soft Shadows.
	  - Additionally, the variable **NO_LIGHTS** can also be changed to swap between the number of light samples that are created for each light. This value is only used if Antialiasing is disabled and it should, ideally, be the same as SPP.
 - **DEPTH_OF_FIELD**
	 - Values: *true*, *false*
	 - Toggles the application of Depth of Field. 
	 - Only noticeable in the *dof.p3f* scene
 - **FUZZY_REFLECTIONS**
	 - Values: *true*, *false*
	 - Toggles the application of Fuzzy Reflection effects
	 - Additionally, the variable **ROUGHNESS** can be used to define how "rough" the reflection is. The lower the value, the more "mirror-like" the reflections will be.
 - **MOTION_BLUR**
	 - Values: *true*, *false*
	 - Toggles the application of Motion Blur. 
	 - Only noticeable in the *ball_motion.p3f* scene
	 - Additionally, the variables **t0** and **t1** can be used to define the camera shutter times.
 - **USE_ACCEL_STRUCT**
	- Values: *0* (no acceleration structure),  *1* (uniform grid), 2 (BVH)
	 - Chooses which acceleration structure to use (or none at all)
 - **drawModeEnabled**
	 - Values: *true*, *false*
	 - If enabled, renders the scene in real time. If disabled, renders a single frame and saves it as a screenshot in the project directory under the name RT_OUTPUT.png
 - **P3F_scene**
	  - Values: *true*, *false*
	 - If enabled, allows the user to type which scene they want to render. Else, a default "debug" scene is presented.
	 
Additionally, in the file **scene.h** you can also toggle the option **USE_MAILBOX** between *true* or *false*, therefore enabling or disabling the usage of mailboxing

<img src="https://i.imgur.com/weU1wyW.png" alt="drawing" width="250" style="display: block; margin-left: auto; margin-right: auto;"/>


## Scenes
When running the program (and granted drawModeEnabled is set to true), the user will be prompted to type in a scene to load. Following are the available scenes

### Balls
 There are 3 "Balls_xxxx" scenes - **balls_low.p3f**, **balls_medium.p3f**, **balls_high.p3f**. All these scenes consist in a single reflective ball placed in the centered of the scene with several other balls surrounding it. They vary in the number of balls, and, therefore, computational demand.

<p float="left" style="display: block; margin-left: auto; margin-right: auto;">
  <img src="https://i.imgur.com/xAa6ObV.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>
  <img src="https://i.imgur.com/7ShTgxB.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/> 
 <img src="https://i.imgur.com/Q1DdAql.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>
</p>
 
### Mount
 There are 3 "mount_xxxx" scenes - **mount_low.p3f**, **mount_high.p3f**, **mount_very_high.p3f**. All these scenes consist in a three transparent balls placed above a mountain made up of triangle polygons. They vary in the number of fidelity of the mount.
<p float="left" style="display: block; margin-left: auto; margin-right: auto;">
<img src="https://i.imgur.com/KatgQ4y.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>
<img src="https://i.imgur.com/JDRLsp0.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>
<img src="https://i.imgur.com/DDIQJLu.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>
</p>
 





### Balls Box
A scene containing a red reflective box in the center surrounded by metallic spheres.

<img src="https://i.imgur.com/CvoEf8u.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>

### DOF
A scene used to visualize the Depth of Field effect. The camera has an aperture of 12.0, hence causing the desired effect (if enabled in the options).

<img src="https://i.imgur.com/bIwvKMD.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>

### Balls Motion
This scene is best seen in "live" mode since if an image is generated all that will be seen is a still ball. If in live mode the user should be able to see the ball oscilating rapidly, causing the motion blur effect.

<img src="https://i.imgur.com/81yidE5.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>

### Default Scene
If no scene is selected (P3F_scene is set to false), we render a default scene that looks as following.

<img src="https://i.imgur.com/4LWYHU6.png" alt="drawing" width="300" style="display: block; margin-left: auto; margin-right: auto;"/>

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/heroufenix/cgj-rose/blob/master/LICENSE) file for more information

