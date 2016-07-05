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

#define CURSOR_SIZE_PIXELS 20;

// Loading obj paths
char *cube_path("./models/my_models/cube_10.obj");
char *needle_path("./models/demo_obj_OH/LumbarBallProbe.obj");

// Mouse movements
static int mouse_old_X, mouse_old_Y, mouse_curr_X, mouse_curr_Y; // mouse pose
static bool mouse_mid_butt_active, mouse_left_butt_active; //bool for clicks

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
HLuint needle_id;

// Booalean flag set to true if the surface is touched
bool is_touched = false;

// Other stuff
static double gCursorScale;
static GLuint gCursorDisplayList = 0;
HDdouble needle_DOP;
HLfloat cursorToToolTranslation = 0.25; //! ??

hduVector3Dd proxy_normal(0.0,0.0,0.0);
hduVector3Dd proxy_position(0.0,0.0,0.0);
static hduVector3Dd proxy_contact_pos(0.0,0.0,0.0); //! Those must be generated in hlTouchCubeCB
											//! Rest-values may be wrong
static HDdouble proxy_contact_T[16];



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
void glutReshape(int width, int height);
void glutIdle();
void glutMenu(int entry_ID);
void glutMouse(int button, int state, int x, int y);
void glutMouseMotion(int x, int y);

// Support functions
void drawSceneGraphics();
void drawSceneHaptics();
void drawCursor();
void updateWorkspace();
void displayInfo();
void exitHandler();
void handleKeyboard(unsigned char key, int x, int y);
void DrawBitmapString(GLfloat x, GLfloat y, void *font, char *format,...);

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
void HLCALLBACK hlNeedleMotionCB(HLenum event,
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

int main(int argc, char *argv[])
{
	// Init window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Needle Insertion");

	glutDisplayFunc(glutDisplayCB);
	glutReshapeFunc(glutReshape);
	glutIdleFunc(glutIdle);
	glutCreateMenu(glutMenu);

	glutAddMenuEntry("Quit", 0);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
	glutKeyboardFunc(handleKeyboard);

	glutMouseFunc(glutMouse);
	glutMotionFunc(glutMouseMotion);

	atexit(exitHandler);
	initScene();
	glutMainLoop();

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
		fprintf(stderr, "\nPress any key to exit \n\n");
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

	// Generate id for the shape.
	cube_id = hlGenShapes(1);
	needle_id = hlGenShapes(1);
	hlTouchableFace(HL_FRONT);

	//! TO DO: Generate shape ID, add callbacks (hlAddEventCallback)
	hlAddEventCallback(HL_EVENT_TOUCH, 
			cube_id,
			HL_COLLISION_THREAD,
			hlTouchCubeCB, 0);
	hlAddEventCallback(HL_EVENT_TOUCH, 
			cube_id,
			HL_COLLISION_THREAD,
			hlUntouchCubeCB, 0);

	// MOTION
	hlAddEventCallback(HL_EVENT_MOTION,
			needle_id,
			HL_COLLISION_THREAD,
			hlNeedleMotionCB,0);
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
	initOpenGL();
	initHL();
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
		// glRotatef(45,0,0,1);
		glmScale(cube_model, 0.350);
		glmFacetNormals(cube_model);
		glmVertexNormals(cube_model, 90.0);
	}

	cube_obj_list = glGenLists(1);
	glNewList(cube_obj_list, GL_COMPILE);
	glTranslatef(0.0f,-0.15f,-0.19f);
	glRotatef(30.0f, 1.0f, 0.0f, 0.0f);
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

// *************************************
// Haptic callbacks
void HLCALLBACK hlTouchCubeCB(HLenum event,
		HLuint object, 
		HLenum thread,
		HLcache *cache,
		void *userdata)
{
	//! Placeholder
	

	if (!is_touched)
	{
		hlGetDoublev(HL_PROXY_POSITION, proxy_contact_pos);
		hlGetDoublev(HL_PROXY_TRANSFORM, proxy_contact_T);
		is_touched = true;

		printf("PROXY Position is: %f %f %f \n", proxy_contact_pos[0], proxy_contact_pos[1], proxy_contact_pos[2]);
		cout << " NEEDLE Current Transformation is: " << endl <<
			proxy_contact_T[0] << "\t" << proxy_contact_T[4] << "\t" << proxy_contact_T[8] << "\t" << proxy_contact_T[12] << endl <<
			proxy_contact_T[1] << "\t" << proxy_contact_T[5] << "\t" << proxy_contact_T[9] << "\t" << proxy_contact_T[13] << endl <<
			proxy_contact_T[2] << "\t" << proxy_contact_T[6] << "\t" << proxy_contact_T[10] << "\t" << proxy_contact_T[14] << endl <<
			proxy_contact_T[3] << "\t" << proxy_contact_T[7] << "\t" << proxy_contact_T[11] << "\t" << proxy_contact_T[15] << endl;
	}
	else
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
	cout << "Now outiside" << endl;
	// Reset the proxy_contact_pos to 0.0
	proxy_contact_pos[0] = 0.0; proxy_contact_pos[1] = 0.0; proxy_contact_pos[2] = 0.0;

}

void HLCALLBACK hlNeedleMotionCB(HLenum event,
		HLuint object, 
		HLenum thread,
		HLcache *cache,
		void *userdata)
{
	// Placeholder.
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


// *************************************
// Graphical callbacks
void glutDisplayCB(void)
{
	drawSceneGraphics();
	drawSceneHaptics();

	// swap buffers to show graphics results on screen
	glutSwapBuffers();
}

void glutReshape(int width, int height)
{
	static const double kPI = 3.1415926535897932384626433832795;
	static const double kFovY = 40;
	double nearDist, farDist, aspect;

	glViewport(0, 0, width, height);

	// Compute the viewing parameters based on a fixed fov and viewing
	// a canonical box centered at the origin.

	nearDist = 1.0 / tan((kFovY / 2.0) * kPI / 180.0);
	farDist = nearDist + 2.0;
	aspect = (double) width / height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, aspect, 0.1, farDist);

	// Place the camera down the Z axis looking at the origin.

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(	0, 0, -1.0+farDist,
				0, 0, 0,
				0, 1, 0);

	updateWorkspace();
}

void glutIdle()
{
	HLerror error;

	while (HL_ERROR(error = hlGetError()))
	{
		fprintf(stderr, "HL Error: %s\n", error.errorCode);

		if (error.errorCode == HL_DEVICE_ERROR)
		{
			hduPrintError(stderr, &error.errorInfo,
				"Error during haptic rendering\n");
		}
	}

	// char title[40];
	// sprintf(title, "Haptic Displacement Mapping %4.1f fps", DetermineFPS());

	// glutSetWindowTitle(title);
	glutPostRedisplay();
}

void glutMenu(int ID)
{
	switch(ID)
	{
		case 0:
			exit(0);
			break;
	}
}

void glutMouse(int button,int state,int x,int y)
{
	if (state == GLUT_UP)
		switch (button)
		{
			case GLUT_LEFT_BUTTON:
				mouse_left_butt_active = false;
				break;
			case GLUT_MIDDLE_BUTTON:
				mouse_mid_butt_active = false;
				break;
		}

	if (state == GLUT_DOWN)
	{
		mouse_old_X = mouse_curr_X = x;
		mouse_old_Y = mouse_curr_Y = y;

		switch (button)
		{
			case GLUT_LEFT_BUTTON:
				mouse_left_butt_active = true;
				cout << " ho premuto il pulsante sx" << endl;
				break;
			case GLUT_MIDDLE_BUTTON:
				mouse_mid_butt_active = true;
				cout << " ho premuto il pulsante porcodio" << endl;
				break;
		}
	}
}

void glutMouseMotion(int x,int y)
{ 
	mouse_curr_X = x; 
	mouse_curr_Y = y;

	if (mouse_left_butt_active && mouse_mid_butt_active) 
	{ 
		mouse_x_translation += (mouse_curr_X - mouse_old_X)/100.0f; 
		mouse_y_translation -= (mouse_curr_Y - mouse_old_Y)/100.0f; 
	} 
	else if (mouse_left_butt_active) 
	{ 
		mouse_x_rotation -= (mouse_curr_X - mouse_old_X); 
		mouse_y_rotation -= (mouse_curr_Y - mouse_old_Y); 
	} 
	else if (mouse_mid_butt_active) 
		mouse_z_translation -= (mouse_curr_Y - mouse_old_Y)/10.f;

	mouse_old_X = mouse_curr_X;
	mouse_old_Y = mouse_curr_Y;
}

// *************************************
// support function
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
	displayInfo();
	glEnable(GL_TEXTURE_2D);
	updateWorkspace();
}

void drawCursor()
{
	static const double kCursorRadius = 0.25;
	static const double kCursorHeight = 1.5;
	static const int kCursorTess = 15;

	HLdouble proxyxform[16];
	double proxyPos[3];

	hlGetDoublev(HL_PROXY_POSITION, proxyPos);
	GLUquadricObj *qobj = 0;
	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glPushMatrix();

	if (!gCursorDisplayList)
	{
		gCursorDisplayList = glGenLists(1);
		glNewList(gCursorDisplayList, GL_COMPILE);
		qobj = gluNewQuadric();

		gluCylinder(qobj, 0.0, kCursorRadius, kCursorHeight,
		kCursorTess, kCursorTess);

		glTranslated(0.0, 0.0, kCursorHeight);

		gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight / 5.0,
		kCursorTess, kCursorTess);

		gluDeleteQuadric(qobj);
		glEndList();
	}

	// Get the proxy transform in world coordinates.
	hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);

	//If entered hole, then freeze the rotations.
	if (is_touched)
	{
		proxyxform[0] = 1.0;
		proxyxform[1] = 0.0;
		proxyxform[2] = 0.0;
		proxyxform[3] = 0.0;
		proxyxform[4] = 0.0;
		proxyxform[5] = 1.0;
		proxyxform[6] = 0.0;
		proxyxform[7] = 0.0;
		proxyxform[8] = 0.0;
		proxyxform[9] = 0.0;
		proxyxform[10] = 1.0;
		proxyxform[11] = 0.0;
	}

	//Get the depth of Penetration from HLAPI.
	hlGetDoublev(HL_DEPTH_OF_PENETRATION, &needle_DOP);
	glMultMatrixd(proxyxform);

	//! NB: force is not yet defined, porco dio quando ce l'hai scommenta
	if (is_touched)
		cursorToToolTranslation = 0.35;
	else
		cursorToToolTranslation = 0.25;

	glTranslatef(0.0,0.0,cursorToToolTranslation);

	// if (is_touched && force[2]>=0.0)
	// 	glTranslatef(0.0,0.0,-1* needle_DOP);

	glCallList(needle_obj_list);
	glPopMatrix();
	glPopAttrib();
}


void updateWorkspace()
{
	GLdouble modelview[16];
	GLdouble projection[16];
	GLint viewport[4];

	//! Must set those two variables wrt the size of our objects
	HLdouble minn[3]={-0.4,-0.5, -0.4};
	HLdouble maxx[3]={0.4,0.5,0.4};

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	hlMatrixMode(HL_TOUCHWORKSPACE);
	hlLoadIdentity();
	hluFitWorkspaceBox(modelview, minn, maxx);

	// Compute cursor scale.
	gCursorScale = hluScreenToModelScale(modelview, projection, viewport);
	gCursorScale *= CURSOR_SIZE_PIXELS;
	hlMatrixMode(HL_MODELVIEW);
	hlLoadMatrixd(modelview);
}

// Drawing haptic stuff in the scene
void drawSceneHaptics()
{
	hlGetDoublev(HL_PROXY_TOUCH_NORMAL, proxy_normal);
	hlGetDoublev(HL_PROXY_POSITION, proxy_position);

	// Start haptic frame
	hlBeginFrame();
	hlPushMatrix();

	//************** SHAPE 1: cube **************//
	//! Here we have to modify those haptic params to 
	//! model the wanted interactions (POPTHROUGH, constraints ..)
	hlTouchModel(HL_CONTACT);
	hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.0f);
	hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.1);
	hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION,0.1 );
	hlHinti(HL_SHAPE_FEEDBACK_BUFFER_VERTICES, cube_model->numvertices);
	hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, cube_id); //! FEEDBACK or DEPTH??

	glPushMatrix();
	glCallList(cube_obj_list);
	glPopMatrix();
	hlEndShape();
	hlPopMatrix();


	//************** SHAPE 2: needle **************//
	// PLACEHOLDER, for the needle
	// hlTouchModel(HL_CONTACT);
	// hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.9f);
	// hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.0f);
	// hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.1);
	// hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION,0.1 );
	// hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, needle_id); //! FEEDBACK or DEPTH??
	// hlEndShape();
	// hlPopMatrix();

	// End the haptic frame.
	hlEndFrame();

}



void displayInfo()
{
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glPushMatrix();
	glLoadIdentity();

	int gwidth, gheight;
	gwidth = glutGet(GLUT_WINDOW_WIDTH);
	gheight = glutGet(GLUT_WINDOW_HEIGHT);

	// switch to 2d orthographic mMode for drawing text
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	gluOrtho2D(0, gwidth, gheight, 0);
	glMatrixMode(GL_MODELVIEW);

	glColor3f(1.0, 1.0, 1.0);

	int textRowDown = 0;                                    // lines of text already drawn downwards from the top

	int textRowUp = 0;                                      // lines of text already drawn upwards from the bottom

	// DrawBitmapString(0 , gheight-100 , GLUT_BITMAP_HELVETICA_18, "LAYER 1 penetrated: ");

	// if (needle_DOP > 0.1 && is_touched)
	// 	DrawBitmapString(200 , gheight-100 , GLUT_BITMAP_HELVETICA_18, "Yes");
	// else
	// 	DrawBitmapString(200 , gheight-100 , GLUT_BITMAP_HELVETICA_18, "No");

	// DrawBitmapString(0 , gheight-80 , GLUT_BITMAP_HELVETICA_18, "LAYER 2 penetrated: ");

	// if (needle_DOP > 0.25 && is_touched)
	// 	DrawBitmapString(200 , gheight-80 , GLUT_BITMAP_HELVETICA_18, "Yes");
	// else
	// 	DrawBitmapString(200 , gheight-80 , GLUT_BITMAP_HELVETICA_18, "No");

	// DrawBitmapString(0 , gheight-60 , GLUT_BITMAP_HELVETICA_18, "Depth of Penetration: %f ",needle_DOP );

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	// turn depth and lighting back on for 3D rendering
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
}


void exitHandler()
{
	// Deallocate the sphere shape id we reserved in initHL.
	if (cube_model)
	{
		hlDeleteShapes(cube_id, 1);
	}

	// Free up the haptic rendering context.
	hlMakeCurrent(NULL);

	if (ghHLRC != NULL)
	{
		hlDeleteContext(ghHLRC);
	}

	// Free up the haptic device.
	if (ghHD != HD_INVALID_HANDLE)
	{
		hdDisableDevice(ghHD);
	}
}

void handleKeyboard(unsigned char key, int x, int y)
{
	if (key == 27)
		exit(0);
}

void DrawBitmapString(GLfloat x, GLfloat y, void *font, char *format,...)
{
	int len, i;
	va_list args;
	char string[256];

	// special C stuff to interpret a dynamic set of arguments specified by "..."
	va_start(args, format);
	vsprintf(string, format, args);
	va_end(args);

	glRasterPos2f(x, y);
	len = (int) strlen(string);

	for (i = 0; i < len; i++)
	{
		glutBitmapCharacter(font, string[i]);
	}
}