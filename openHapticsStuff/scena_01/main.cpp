#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <stack>

// only Linux platform
#include <ncurses.h>
#define _getch getch

// OpenGL include
#include <GL/glut.h>

// Include OpenHaptics HL
#include <HL/hl.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#include <HDU/hduMath.h>
#include <HDU/hduBoundBox.h>
#include <HLU/hlu.h>

// Including library to handle OBJ models
#include "GLM.h"

using namespace std;

// Loading obj paths
char *cube_path("./models/my_models/cube_10.obj");
char *needle_path("./models/demo_obj_OH/LumbarBallProbe.obj");

// Mouse movements
static int mouse_old_X, mouse_old_Y, mouse_curr_X, mouse_curr_Y; // mouse pose
static bool mouse_right_butt_active, mouse_left_butt_active; //bool for clicks

// Initializing mouse translations and rotations of the scene
float mouse_x_translation = 0.0f;
float mouse_y_translation = 0.0f;
float mouse_z_translation = -1.5; // Initial 'zoom'

float mouse_x_rotation = 0.0f;
float mouse_y_rotation = 0.0f;


// Declaring data-structures to contain obj-models
//! Initially they point @ NULL-obj; 
//! they will be initialized by proper functions later
GLMmodel* cube_model = NULL;
GLMmodel* needle_model = NULL;

GLuint cube_obj_list;
GLuint needle_obj_list;

// Declaring haptic device 
//! Later it will be initialize by a proper function (initHL)
static HHD ghHD = HD_INVALID_HANDLE;

// Declaring rendering context
//! Later initialized by hlCreateContext
static HHLRC ghHLRC = 0;

// Shape id declaration
HLuint cube_id;
HLuint contact_point_id;

// Booalean flag set to true if the surface is touched
bool is_touched = false;



// *************************************************** //
// ************ FUNCTIONS PROTOTYPES ***************** //
// *************************************************** //

// Declaring OpenGL Functions
void initScene();
void initOpenGL();
void initHL();
void initCube();
void initNeedle();

// Graphical callbacks
void glutDisplayCB(void);

// Support functions
void drawSceneGraphics();
void drawSceneHaptics();
void drawCursor();

// Declaring callbacks' 
void HLCALLBACK hlTouchCubeCB(HLenum event,
		HLuint object, 
		HLenum thread,
		HLcache *cache,
		void *userdata);
void HLCALLBACK hlUntouchCubeCB(HLenum event,
		HLuint object, 
		HLenum thread,
		HLcache *cache,
		void *userdata);

// Main HD callbacks
HDCallbackCode HDCALLBACK hdBeginCB(void *data);
HDCallbackCode HDCALLBACK hdEndCB(void *data);




// *************************************************** //
// ********************** MAIN  ********************** //
// *************************************************** //

int main(int argc, char const *argv[])
{
	// Init window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Needle Insertion");

	glutDisplayFunc(glutDisplay);

	printf("Initializing scene and OpenGL\n");
	initScene();
	return 0;
}




// *************************************************** //
// ******************** FUNCTIONS  ******************* //
// *************************************************** //

// Initializing OpenHaptics and device
void initHL()
{
	HDErrorInfo error;
	ghHD = hdInitDevice( HD_DEFAULT_DEVICE );

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "Press any key to exit");
		getchar();
		exit(-1);
	}

	/*************************************************************
	ADDED
	*************************************************************/
	hdScheduleAsynchronous(hdBeginCB, 0, HD_MAX_SCHEDULER_PRIORITY);
	hdScheduleAsynchronous(hdEndCB, 0, HD_MIN_SCHEDULER_PRIORITY);

	/*************************************************************
	END ADDED
	*************************************************************/
	ghHLRC = hlCreateContext(ghHD);
	hlMakeCurrent(ghHLRC);

	// Enable optimization of the viewing parameters when rendering
	// geometry for OpenHaptics.
	hlEnable(HL_HAPTIC_CAMERA_VIEW);

	//! TO DO: Generate shape ID, add callbacks (hlAddEventCallBack)
	hlAddEventCallBack(HL_EVENT_TOUCH, 
			cube_id,
			HL_COLLISION_THREAD,
			hlTouchCubeCB, 0);
	hlAddEventCallBack(HL_EVENT_TOUCH, 
			cube_id,
			HL_COLLISION_THREAD,
			hlUntouchCubeCB, 0);
}


//! Copied, for new lights and other elements look here
void initOpenGL()
{
	// Light properties for the diffuse light, specular light, and light position. //
	static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
	static const GLfloat light0_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
	static const GLfloat light0_direction[] = {0.0f, -0.4f, 1.0f, 0.0f};
	static const GLfloat specularLight[] = {1.0f, 1.0f, 1.0f, 1.0f};

	// Enable depth buffering for hidden surface removal.
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	// Cull back faces.
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_TEXTURE_2D);
	glClearDepth(1.0f); // Depth Buffer Setup

	// Setup other misc features.
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);

	// Setup lighting model.
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);
	glEnable(GL_LIGHT0);

	// Set up the material information for our objects.
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv(GL_FRONT, GL_SPECULAR, specularLight);
	glMateriali(GL_FRONT, GL_SHININESS, 128);
}


void initScene()
{
	//! If obj models are not found, use absolute path for
	//! loading them
	initCube();
	initNeedle();
	initHL();
	initOpenGL();
}

// Initialize the cube_model
void initCube()
{
	if (!cube_model)
	{
		cube_model = glmReadOBJ(cube_path);

		if (!cube_model)
		{
			printf("ERROR! Impossible to load cube OBJ\n");
			exit(1);
		}

		//! OBJ spatial modification here:
		glmUnitize(cube_model);
		// glmScale(cube_model, 0.250);
		glmFacetNormals(cube_model);
		glmVertexNormals(cube_model, 90.0);
	}

	cube_obj_list = glGenLists(1);
	glNewList(cube_obj_list, GL_COMPILE);
	// glmDraw(objmodel, GLM_SMOOTH | GLM_TEXTURE); // if textures
	glmDraw(cube_model, GLM_SMOOTH );
	glEndList();

	//! If needed it is possible to add a bump map to the obj
}

void initNeedle()
{
	if(!needle_model)
	{
		needle_model = glmReadOBJ(needle_path);
		if (!needle_model)
		{
			printf("ERROR, Impossible to load needle OBJ\n");
			exit(1);
		}

		//! OBJ spatial modification here:
		glmUnitize(needle_model);
		glmScale(needle_model, 0.250);
		glmFacetNormals(needle_model);
		glmVertexNormals(needle_model, 90.0);
	}
	needle_obj_list = glGenLists(1);
	glNewList(needle_obj_list, GL_COMPILE);
	glmDraw(needle_model, GLM_SMOOTH);
	glEndList();
}

// Haptic callbacks
void HLCALLBACK hlTouchCubeCB(HLenum event,
		HLuint object, 
		HLenum thread,
		HLcache *cache,
		void *userdata)
{
	//! Placeholder
	is_touched = true;
	cout << "Touching the surface!" << endl;
}

void HLCALLBACK hlUntouchCubeCB(HLenum event,
		HLuint object, 
		HLenum thread,
		HLcache *cache,
		void *userdata)
{
	//! The force must be negative along the hole axis if we are 
	//! actually exiting
	is_touched = false;
	cout << "Now outiside"
}


HDCallbackCode HDCALLBACK hdBeginCB(void *data)
{
	// Starting the frame (HL)
	HHD current_device_handler = hdGetCurrentDevice();
	hdBeginFrame(current_device_handler);

	// Handling HD-errors
	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error in hdBeginCB callback\n");
		return HD_CALLBACK_DONE;
	}
	return HD_CALLBACK_CONTINUE;
}

// This function handles the force constraints due to penetration
HDCallbackCode HDCALLBACK hdEndCB(void *data)
{

	//! PLACEHOLDER

	HHD current_device_handler = hdGetCurrentDevice();
	hdEndFrame(current_device_handler);
	HDErrorInfo error;

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error in hdEndCB\n");
	}

	return HD_CALLBACK_CONTINUE;
}

// Graphical callbacks
void glutDisplayCB(void)
{
	drawSceneGraphics();
	drawSceneHaptics();

	// swap buffers to show graphics results on screen
	glutSwapBuffer();
}


void drawSceneGraphics()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// place camera/eye
	glTranslatef(mouse_x_translation ,mouse_y_translation ,mouse_z_translation);
	glRotatef(-mouse_y_rotation, 1.0,0.0,0.0);
	glRotatef(-mouse_x_rotation, 0.0,1.0,0.0);
	glDisable(GL_TEXTURE_2D);

	drawCursor();

	glEnable(GL_TEXTURE_2D);
	glPushMatrix();
	glCallList(cube_obj_list); //Displays regular OBJ model

	glPopMatrix();

	//Uncomment to see the Entry Point
	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glDisable(GL_TEXTURE_2D);
	glPushMatrix();
	glPointSize(15.0);
	glTranslatef(0.0, 0.0, 1.0);
	glBegin(GL_POINTS);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0,0.0,1.0);
	glVertex3f(0.05,-0.175,0.975);
	glEnd();

	glPopMatrix();
	glPopAttrib();
	DisplayInfo();
	glEnable(GL_TEXTURE_2D);
	updateWorkspace();
}

void drawCursor()
{
	
}