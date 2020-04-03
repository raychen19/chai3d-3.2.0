//==============================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D.
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    <http://www.chai3d.org>
\author    Francois Conti
\version   3.2.0 $Rev: 1925 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "commTool.h"
#include <fstream>
#include "PDController.h"
#include "Config.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include "HapticCommLib.h"


//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
C_STEREO_DISABLED:            Stereo is disabled
C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;
//bool ISS_enabled=true
/////////////////
double mu_max = 35;
float stiff_factor = 0.5;
cVector3d d_force(0, 0, 0);
double tau = 0.005;
//bool ISS_enabled = false;
cVector3d last_force(0, 0, 0);
float mu_factor = 1;


///////////

// mirrored display
bool mirroredDisplay = false;


std::ofstream OutFile1;
int delay =0;
bool pan = 0;
threadsafe_queue<cVector3d> velq;
threadsafe_queue<cVector3d> forq;
threadsafe_queue<cVector3d> posq;
threadsafe_queue<double> posq1;
threadsafe_queue<double> velq1;
threadsafe_queue<double> deta;
struct hapticdata {  
	__int64 timestamp;
	double force[3];
	double position[3];
	double posgoal[3];
	double posproxy[3];
	double computedforce[3];
	int i = 0;
	double deta = 0.0;
};
threadsafe_queue<hapticdata> hapticdataQ;


void datasave(void);
cVector3d master_force(0, 0, 0);
cVector3d master_position(0, 0, 0);
cVector3d master_forcedata(0, 0, 0);
cVector3d master_posgoal(0, 0, 0);
cVector3d master_proxy(0, 0, 0);

cVector3d master_velocity(0, 0, 0);
cVector3d slave_velocity(0, 0, 0);
cVector3d slave_force(0, 0, 0);
cVector3d slave_position(0, 0, 0);
cVector3d d0(0, 0, 0);
cVector3d position2(0, 0, 0);
double d[3] = { 0,0,0 };

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;
bool contact = false;

// a few objects that are placed in the scene
cMesh* base;
cMesh* teaPot;
cMesh* cylinder;
cMesh* cone;
cMesh* springcube;
cMultiSegment* segments;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;
cLabel* poslabel;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;
cThread* hapticsThread1;
cThread* hapticsThread2;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;
KalmanFilter VelocityKalmanFilter;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);
void datasave(void);
void springpos(void);


//==============================================================================
/*
DEMO:   13-primitives.cpp

This example illustrates how to build simple triangle based mesh primitives
using the functions provided in file graphics/CPrimitives.h
*/
//==============================================================================

int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "Demo: 13-primitives" << endl;
	cout << "Copyright 2003-2016" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[s] - Save copy of shadowmap to file" << endl;
	cout << "[f] - Enable/Disable full screen mode" << endl;
	cout << "[m] - Enable/Disable vertical mirroring" << endl;
	cout << "[q] - Exit application" << endl;
	cout << endl << endl;


	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setWhite();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(0.9, 0.0, 0.6),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

									 // set the near and far clipping planes of the camera
									 // anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.03);
	camera->setStereoFocalLength(1.8);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a light source
	light = new cSpotLight(world);

	// attach light to camera
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// position the light source
	light->setLocalPos(0.6, 0.6, 0.5);

	// define the direction of the light beam
	light->setDir(-0.5, -0.5, -0.5);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(true);

	// set the resolution of the shadow map
	//light->m_shadowMap->setQualityLow();
	light->m_shadowMap->setQualityMedium();

	// set light cone half angle
	light->setCutOffAngleDeg(30);


	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

	// if the haptic devices carries a gripper, enable it to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

	// create a tool (cursor) and insert into the world
	tool = new cToolCursor(world);
	world->addChild(tool);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// define the radius of the tool (sphere)
	double toolRadius = 0.05;

	// define a radius for the tool
	tool->setRadius(toolRadius);

	// hide the device sphere. only show proxy.
	tool->setShowContactPoints(true, 0);

	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->enableDynamicObjects(true);

	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// start the haptic tool
	tool->start();


	//--------------------------------------------------------------------------
	// CREATE OBJECTS
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// stiffness properties
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
	//cout << maxStiffness << endl;


	/////////////////////////////////////////////////////////////////////////
	// BASE
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* base = new cMesh();

	// add object to world
	world->addChild(base);

	// build mesh using a cylinder primitive
	cCreateCylinder(base,
		0.01,
		0.5,
		36,
		1,
		10,
		true,
		true,
		cVector3d(0.0, 0.0, -0.01),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	base->m_material->setGrayGainsboro();
	base->m_material->setStiffness(0.5 * maxStiffness);
	//std::cout << "kkk" << 0.5 * maxStiffness << std::endl;

	// build collision detection tree
	base->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	base->setUseDisplayList(true);

	cVector3d Localpos(0, 0, 0);

	////////////////////////////////////

	springcube = new cMesh();
	base->addChild(springcube);
	cCreateBox(springcube, 0.25, 0.25, 0.4,
		cVector3d(0.08, 0.15, 0.2), cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(90), C_EULER_ORDER_XYZ),
		cColorf(1.0, 1.0, 1.0)
	);
	springcube->m_material->setBlueCornflower();
	springcube->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	springcube->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	springcube->setUseDisplayList(true);
	Localpos = springcube->getLocalPos();
	//cout << Localpos << endl;
	/////////////////////////////////////////////////////////////////////////
	// TEA POT
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* teaPot = new cMesh();

	// add object to world
	//base->addChild(teaPot);

	// build mesh using a cylinder primitive
	cCreateTeaPot(teaPot,
		0.5,
		4,
		cVector3d(0.0, 0.0, 0.0),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(-90), C_EULER_ORDER_XYZ)
	);

	// position object
	teaPot->setLocalPos(0.1, 0.2, 0.0);

	// set material properties
	teaPot->m_material->setRedDark();
	teaPot->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	teaPot->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	teaPot->setUseDisplayList(true);


	/////////////////////////////////////////////////////////////////////////
	// CYLINDER
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh*  cylinder = new cMesh();

	cCreatePipe(cylinder,
		0.15,
		0.05,
		0.06,
		32,
		1,
		cVector3d(-0.05, -0.20, 0.0),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(170), C_EULER_ORDER_XYZ)
	);

	// set material properties
	cylinder->m_material->setBlueCornflower();
	cylinder->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	cylinder->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	cylinder->setUseDisplayList(true);


	/////////////////////////////////////////////////////////////////////////
	// CONE
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* cone = new cMesh();

	// add object to world
	//base->addChild(cone);

	// build mesh using a cylinder primitive
	cCreateCone(cone,
		0.15,
		0.05,
		0.01,
		32,
		1,
		1,
		true,
		true,
		cVector3d(0.30, 0.0, 0.0),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	cone->m_material->setGreenForest();
	cone->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	cone->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	cone->setUseDisplayList(true);


	/////////////////////////////////////////////////////////////////////////
	// SEGMENTS
	/////////////////////////////////////////////////////////////////////////

	// create a line segment object
	cMultiSegment* segments = new cMultiSegment();

	// add object to world
	//base->addChild(segments);

	// build some segment
	double l = 0.0;
	double dl = 0.001;
	double a = 0.0;
	double da = 0.2;
	double r = 0.05;
	for (int i = 0; i<200; i++)
	{
		double px0 = r * cos(a);
		double py0 = r * sin(a);
		double pz0 = l;

		double px1 = r * cos(a + da);
		double py1 = r * sin(a + da);
		double pz1 = l + dl;

		// create vertex 0
		int index0 = segments->newVertex(px0, py0, pz0);

		// create vertex 1
		int index1 = segments->newVertex(px1, py1, pz1);

		// create segment
		segments->newSegment(index0, index1);

		l = l + dl;
		a = a + da;
	}

	// set haptic properties
	segments->m_material->setStiffness(0.5 * maxStiffness);

	// position object
	segments->setLocalPos(0.22, -0.22, 0.0);

	// set segment properties
	cColorf color;
	color.setYellowGold();
	segments->setLineColor(color);
	segments->setLineWidth(4.0);
	segments->setUseDisplayList(true);

	// build collision detection tree
	segments->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	segments->setUseDisplayList(true);


	//--------------------------------------------------------------------------
	// CREATE SHADERS
	//--------------------------------------------------------------------------

	// create program shader
	cShaderProgramPtr shaderProgram = cShaderProgram::create(C_SHADER_FONG_VERT, C_SHADER_FONG_FRAG);

	// set uniforms
	shaderProgram->setUniformi("uShadowMap", C_TU_SHADOWMAP);

	// assign shader to mesh objects in the world
	tool->setShaderProgram(shaderProgram);
	base->setShaderProgram(shaderProgram);
	teaPot->setShaderProgram(shaderProgram);
	cylinder->setShaderProgram(shaderProgram);
	cone->setShaderProgram(shaderProgram);


	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelRates);

	poslabel = new cLabel(font);
	poslabel->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(poslabel);

	// create a background
	background = new cBackground();
	camera->m_backLayer->addChild(background);

	// set background properties
	background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
		cColorf(1.0f, 1.0f, 1.0f),
		cColorf(0.8f, 0.8f, 0.8f),
		cColorf(0.8f, 0.8f, 0.8f));
	OutFile1.open("C:/Users/PotatoFaker/Desktop/data1112/0ms/data.txt");
	for (int i = 0; i < delay; i++)
	{
		velq.push(d0);
	}
	for (int i = 0; i < delay; i++)
	{
		forq.push(d0);
	}
	for (int i = 0; i < delay; i++)
	{
		posq.push(d0);
	}
	for (int i = 0; i < delay; i++)
	{
		velq1.push(0);
	}
	//std::cout << velq.length() << std::endl;
	//std::cout << forq.length() << std::endl;


	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	hapticsThread1 = new cThread();
	hapticsThread1->start(datasave, CTHREAD_PRIORITY_HAPTICS);
	hapticsThread2 = new cThread();
	hapticsThread2->start(springpos, CTHREAD_PRIORITY_HAPTICS);
	// setup callback when application exits
	atexit(close);


	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();

	// exit
	return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - save shadow map to file
	else if (a_key == GLFW_KEY_S)
	{
		cImagePtr image = cImage::create();
		light->m_shadowMap->copyDepthBuffer(image);
		image->saveToFile("shadowmapshot.png");
		cout << "> Saved screenshot of shadowmap to file.       \r";
	}

	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	tool->stop();

	// delete resources
	delete hapticsThread;
	delete hapticsThread1;
	delete hapticsThread2;
	OutFile1.close();
	delete world;
	delete handler;
}

//------------------------------------------------------------------------------
void springpos(void)
{

	//springcube->setLocalPos(0.15)
}
void datasave(void)
{
	hapticdata masterdata1;
	__int64 lastTime = 0;
	//OutFile1 << hapticdataQ.length() <<std::endl;
	while (1)
	{
		__int64 currentTime;
		QueryPerformanceCounter((LARGE_INTEGER *)&currentTime);
		if (currentTime - lastTime < 1805)
			continue;
		lastTime = currentTime;
		if (hapticdataQ.length() > 1)
		{
			masterdata1 = hapticdataQ.front();
			hapticdataQ.try_pop();
			OutFile1 << "input" << " " << masterdata1.posgoal[0] << " " << masterdata1.posgoal[1] << " "
				<< masterdata1.posgoal[2] << " " << "output" << " " << masterdata1.position[0] << " "
				<< masterdata1.position[1] << " " << masterdata1.position[2] << " " << "force" << " " << masterdata1.deta << " "
				<< std::endl;

		}
	}
}

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);



	// update position of label
	poslabel->setLocalPos(240, 380, 330);



	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

enum cMode
{
	IDLE,
	SELECTION
};

void updateHaptics(void)
{
	hapticdata masterdata;
	cMode state = IDLE;
	cGenericObject* object = NULL;
	cTransform tool_T_object;

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;


	double master_vel[3] = { 0,0,0 };
	double master_posg[3] = { 0,0,0 };
	double master_posx[3] = { 0,0,0 };
	double poslable = 0;
	double slave_f[3] = { 0,0,0 };
	double master_f[3] = { 0,0,0 };
	double df[3] = { 0,0,0 };
	double dv[3] = { 0,0,0 };
	double lf[3] = { 0,0,0 };
	double lv[3] = { 0,0,0 };
	double slave_vel[3] = { 0,0,0 };
	double slave_pos[3] = { 0,0,0 };
	double posout[3];
	double detaf = 0.0;
	double detax = 0.0;
	//const double detaz = 0.0;
	//bool button1 = tool->getUserSwitch(0);
	//bool button2 = tool->getUserSwitch(1);
	//////pid
	double pid_pos[3] = { 0,0,0 };
	double pid_vel[3] = { 0,0,0 };
	double pid_force[3] = { 0,0,0 };
	double totalforce[3] = { 0,0,0 };
	double acc = 0.0;


	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////////
		// HAPTIC RENDERING
		/////////////////////////////////////////////////////////////////////////

		// signal frequency counter
		freqCounterHaptics.signal(1);

		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		// update position and orientation of tool
		tool->updateFromDevice();
		master_position = tool->getDeviceLocalPos();
		master_velocity = tool->getDeviceLocalLinVel();
		master_forcedata = tool->getDeviceLocalForce();
		master_vel[2] = master_velocity.z();
		master_vel[2] = master_vel[2] + df[2] / (mu_max*mu_factor);
		velq1.push(master_vel[2]);
		slave_vel[2] = velq1.front();
		velq1.try_pop();
		slave_velocity = master_velocity;
		slave_velocity.z(slave_vel[2]);
		posq.push(master_position);
		position2 = posq.front();
		posq.try_pop();
		//slave_position=slave_position + slave_velocity*0.001;
		masterdata.posgoal[0] = slave_position.x();
		masterdata.posgoal[1] = slave_position.y();
		masterdata.posgoal[2] = slave_position.z();
		if (tool->getUserSwitch(1) == true)
		{
			slave_position = position2;

		}


		//PDController mPD;
		//	slave_pos[2] = slave_position.z();
		//pid_force[2] = mPD.calculateForce(slave_pos[2], pid_pos[2],slave_vel[2],pid_vel[2]);
		//pid_force[2] = mPID.calculateForce(slave_pos[2], pid_pos[2]);
		//totalforce[2] = pid_force[2] + slave_f[2];
		//acc = totalforce[2] / 0.04;
		//pid_vel[2] += acc*0.001;
		//pid_vel[2] *= (1 - 0.04);
		//	pid_pos[2] += pid_vel[2] * 0.001;
		//cout << pid_pos[2] << endl;
		slave_position = slave_position + slave_velocity*0.001;
		//slave_velocity.z(pid_vel[2]);
		//slave_position.z(pid_pos[2]);
		slave_pos[2] = slave_position.z();
		tool->setDeviceLocalLinVel(slave_velocity);
		tool->setDeviceLocalPos(slave_position);
		tool->computeInteractionForces();

		if (tool->getHapticPoint(0)->getNumCollisionEvents())
		{
			cMesh* cube = dynamic_cast<cMesh*>(tool->getHapticPoint(0)->getCollisionEvent(0)->m_object->getOwner()->getOwner());

			if (springcube == cube)
				//cout << cube << " " << springcube << endl;
				contact = true;
			else
				contact = false;
		}

		if (contact == true)
		{
			if ((detax = slave_pos[2] - 0.3) < 0)
			{
				detax = abs(detax);
				slave_f[2] = 30 * pow(detax, 1.5) + slave_vel[2] * pow(detax, 1.5)*0.1;
				//cout << slave_vel[2] << " " << endl;
				detax = -1 * detax;
				if (detax >= -0.40)
				{	//detax = -0.40;
					masterdata.deta = detax;
					springcube->setLocalPos(0.00, 0.00, detax);

				}
				//cout << detax << endl;
				else
					//{
					//slave_f[2] = 30 * pow(0.4, 1.5) - slave_vel[2] * pow(0.4, 1.5)*0.1;
					springcube->setLocalPos(0.00, 0.00, -0.39);
				//slave_f[2] = 30 * pow(0.4, 1.5);
				//}
			}
			else
			{
				slave_f[2] = 0;
				springcube->setLocalPos(0.0, 0.0, 0.0);
			}

		}
		else
		{
			slave_f[2] = 0;
			springcube->setLocalPos(0.00, 0.00, 0.00);
		}

		poslable = detax;
		poslabel->setText(cStr(poslable));

		slave_force.z(slave_f[2]);
		forq.push(slave_force);
		master_force = forq.front();
		forq.try_pop();
		master_f[0] = master_force.x();
		master_f[1] = master_force.y();
		master_f[2] = master_force.z();
		df[0] = (master_f[0] - lf[0]) / 0.001;
		df[1] = (master_f[1] - lf[1]) / 0.001;
		df[2] = (master_f[2] - lf[2]) / 0.001;
		//cout << "df" << " " << df[2] << endl;
		lf[0] = master_f[0];
		lf[1] = master_f[1];
		lf[2] = master_f[2];
		master_f[0] = master_f[0] + df[0] * tau;
		master_f[1] = master_f[1] + df[1] * tau;
		master_f[2] = master_f[2] + df[2] * tau;
		master_force.x(0);
		master_force.y(0);
		master_force.z(master_f[2]);
		//cout << master_force.z() << endl;
		masterdata.position[0] = master_force.x();
		masterdata.position[1] = master_force.y();
		masterdata.position[2] = master_force.z();
		master_forcedata = master_force;
		masterdata.force[2] = poslable;
		tool->setDeviceLocalForce(master_force);
		hapticdataQ.push(masterdata);




		/////////////////////////////////////////////////////////////////////////
		// HAPTIC MANIPULATION
		/////////////////////////////////////////////////////////////////////////

		// compute transformation from world to tool (haptic device)
		cTransform world_T_tool = tool->getDeviceGlobalTransform();

		// get status of user switch
		bool button = tool->getUserSwitch(0);

		//
		// STATE 1:
		// Idle mode - user presses the user switch
		//
		if ((state == IDLE) && (button == true))
		{
			// check if at least one contact has occurred
			if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
			{
				// get contact event
				cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

				// get object from contact event
				object = collisionEvent->m_object;

				// get transformation from object
				cTransform world_T_object = object->getGlobalTransform();

				// compute inverse transformation from contact point to object 
				cTransform tool_T_world = world_T_tool;
				tool_T_world.invert();

				// store current transformation tool
				tool_T_object = tool_T_world * world_T_object;

				// update state
				state = SELECTION;
			}
		}

		//
		// STATE 2:
		// Selection mode - operator maintains user switch enabled and moves object
		//
		else if ((state == SELECTION) && (button == true))
		{
			// compute new tranformation of object in global coordinates
			cTransform world_T_object = world_T_tool * tool_T_object;

			// compute new tranformation of object in local coordinates
			cTransform parent_T_world = object->getParent()->getLocalTransform();
			parent_T_world.invert();
			cTransform parent_T_object = parent_T_world * world_T_object;

			// assign new local transformation to object
			object->setLocalTransform(parent_T_object);

			// set zero forces when manipulating objects
			tool->setDeviceGlobalForce(0.0, 0.0, 0.0);
		}

		//
		// STATE 3:
		// Finalize Selection mode - operator releases user switch.
		//
		else
		{
			state = IDLE;
		}


		/////////////////////////////////////////////////////////////////////////
		// FINALIZE
		/////////////////////////////////////////////////////////////////////////

		// send forces to haptic device
		tool->applyToDevice();
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------