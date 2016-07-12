#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <stack>

#if defined(WIN32)
#include <conio.h>
#include <windows.h>
#endif

#if defined(linux)
#include <ncurses.h>
#define _getch getch
#endif

#if defined(WIN32) || defined(linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif

#include <HL/hl.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#include <HDU/hduMath.h>
#include <HDU/hduBoundBox.h>

#include <HLU/hlu.h>
#include "GLM.h"

using namespace std;

#define CURSOR_SIZE_PIXELS 20;

// Loading obj paths
char *cube_path("./models/my_models/cube_1024f.obj");
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
GLuint line_list;

// Declaring haptic device 
static HHD ghHD = HD_INVALID_HANDLE;

// Declaring rendering context
static HHLRC ghHLRC = 0;

// Shape id declaration
HLuint cube_id;
HLuint line_id;
HLuint needle_id;
HLuint contact_point_id;

// Booalean flag set to true if the surface is touched
bool is_touched = false;

// Other stuff
static double gCursorScale;
static GLuint gCursorDisplayList = 0;
HLfloat cursorToToolTranslation = 0.25; //! ??

hduVector3Dd contact_normal(0.0,0.0,0.0);
hduVector3Dd proxy_position(0.0,0.0,0.0);
hduVector3Dd force(0.0f, 0.0f, 0.0f);
hduVector3Dd force_trans(0.0f,0.0f,0.0f);
hduVector3Dd device_pos(0.0f, 0.0f, 0.0f);
hduVector3Dd needle_vector(0.0f, 0.0f, 0.0f);
hduVector3Dd device_contact_pos(0.0f, 0.0f, 0.0f);
hduVector3Dd desired_direction(0.0f, 0.0f, 0.0f);

hduVector3Dd needle_penetration(0.0f, 0.0f, 0.0f);
hduVector3Dd total_force(0.0f,0.0f,0.0f);
hduVector3Dd max_reaction_force_vector(0.0f, 0.0f, 0.0f);
hduVector3Dd damping_force(0.0f,0.0f,0.0f);
hduVector3Dd damping_force_direction(0.0f,0.0f,0.0f);
hduVector3Dd push_proxy_position(0.0f,0.0f,0.0f);


double tissue_height[4] = {0.0f, 30.0f, 60.0f, 90.0f};
double stiffness[4] = {331.0f, 83.0f, 497.0f, 2483.f};
double damping[3] = {3.0f, 1.0f, 3.0f};
bool puncture[3] = {0, 0, 0}; 

HDdouble needle_DOP = 0.0f;
HDint current_button = 0;

static hduVector3Dd proxy_contact_pos(0.0,0.0,0.0); //! Those must be generated in hlTouchCubeCB
hduMatrix device_transf;
hduMatrix button_down_transform;
hduMatrix button_up_transform;
hduMatrix global_transform_matrix;
hduMatrix current_device_transform;
hduMatrix cube_transf;
hduMatrix temp_transform;

bool flag_release = false;
double f_scale = 1.0f;
int count = 0;
double old_needle_DOP = 0.0f;
double needle_DOP_difference = 0.0f;

int jj = 0;


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

void HLCALLBACK hlPushButtonCB(HLenum event,
	HLuint object, 
	HLenum thread,
	HLcache *cache,
	void *userdata);
void HLCALLBACK hlReleaseButtonCB(HLenum event,
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

	hdScheduleAsynchronous(hdBeginCB, 0, HD_MAX_SCHEDULER_PRIORITY);
	hdScheduleAsynchronous(hdEndCB, 0, HD_MIN_SCHEDULER_PRIORITY);

	ghHLRC = hlCreateContext(ghHD);
	hlMakeCurrent(ghHLRC);

	// Enable optimization of the viewing parameters when rendering
	// geometry for OpenHaptics.
	hlEnable(HL_HAPTIC_CAMERA_VIEW);

	// Generate id for the shape.
	cube_id = hlGenShapes(1);
	line_id = hlGenShapes(1);
	contact_point_id = hlGenShapes(1);
	needle_id = hlGenShapes(1);
	hlTouchableFace(HL_FRONT);

	hlAddEventCallback(HL_EVENT_TOUCH, 
		cube_id,
		HL_COLLISION_THREAD,
		hlTouchCubeCB, 0);
	hlAddEventCallback(HL_EVENT_UNTOUCH, 
		cube_id,
		HL_COLLISION_THREAD,
		hlUntouchCubeCB, 0);

	// BUTTON1
	hlAddEventCallback(HL_EVENT_1BUTTONUP,
		HL_OBJECT_ANY,
		HL_COLLISION_THREAD,
		&hlReleaseButtonCB, NULL);
	hlAddEventCallback(HL_EVENT_1BUTTONDOWN,
		HL_OBJECT_ANY,
		HL_COLLISION_THREAD,
		&hlPushButtonCB, NULL);
	// BUTTON2
	hlAddEventCallback(HL_EVENT_2BUTTONUP,
		HL_OBJECT_ANY,
		HL_COLLISION_THREAD,
		&hlReleaseButtonCB, NULL);
	hlAddEventCallback(HL_EVENT_2BUTTONDOWN,
		HL_OBJECT_ANY,
		HL_COLLISION_THREAD,
		&hlPushButtonCB, NULL);
	// BUTTON3
	hlAddEventCallback(HL_EVENT_3BUTTONUP,
		HL_OBJECT_ANY,
		HL_COLLISION_THREAD,
		&hlReleaseButtonCB, NULL);
	hlAddEventCallback(HL_EVENT_3BUTTONDOWN,
		HL_OBJECT_ANY,
		HL_COLLISION_THREAD,
		&hlPushButtonCB, NULL);

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

void initLine()
{
	line_list = glGenLists(1);
	auto temp = desired_direction * 5;

	glNewList(line_list, GL_COMPILE_AND_EXECUTE);
	glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(proxy_contact_pos[0], proxy_contact_pos[1], proxy_contact_pos[2]);
	glVertex3f(temp[0], temp[1], temp[2]);
	glEnd();
	glPointSize(4);
	// glBegin(GL_POINTS);
	// glVertex3f(proxy_contact_pos[0], proxy_contact_pos[1], proxy_contact_pos[2]);
	// glVertex3f(temp[0], temp[1], temp[2]);
	// glEnd();
	glPopAttrib();
	glEndList();

}

// Initialize the cube_model
void initCube()
{
	if (!cube_model)
	{
		cube_model = glmReadOBJ(cube_path);

		if (!cube_model)
		{
			cerr << "ERROR! Impossible to load cube OBJ\n" << endl;
			exit(1);
		}

		//! OBJ spatial modification here:
		glmUnitize(cube_model);
		// glRotatef(45,0,0,1);
		// glmScale(cube_model, 0.550);
		glmFacetNormals(cube_model);
		glmVertexNormals(cube_model, 90.0);
	}

	cube_obj_list = glGenLists(1);
	glNewList(cube_obj_list, GL_COMPILE);
	glTranslatef(0.0f,-0.15f,-0.19f);
	// glRotatef(30.0f, 1.0f, 0.0f, 0.0f);

	glScalef(0.35f, 0.1f, 0.35f);
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
	is_touched = true;

	device_contact_pos = device_pos;
	hlGetDoublev(HL_PROXY_POSITION, proxy_contact_pos);
	hlGetDoublev(HL_PROXY_TOUCH_NORMAL, contact_normal);
	hlGetDoublev(HL_DEVICE_TRANSFORM, device_transf);

	// device_transf = device_transf * global_transform_matrix;

	// Line drawing
	desired_direction = hduVector3Dd(0.0f, 0.0f, 1.0f);
	auto temp = device_transf.getInverse();
	temp.multMatrixDir(desired_direction,desired_direction);
	desired_direction = (-1) * desired_direction;
}

void HLCALLBACK hlUntouchCubeCB(HLenum event,
	HLuint object, 
	HLenum thread,
	HLcache *cache,
	void *userdata)
{
	/*is_touched = false;
	puncture[0] = 0.0f;
	puncture[1] = 0.0f;
	puncture[2] = 0.0f;*/
}

void HLCALLBACK hlNeedleMotionCB(HLenum event,
	HLuint object, 
	HLenum thread,
	HLcache *cache,
	void *userdata)
{
	// Placeholder.
}


void HLCALLBACK hlPushButtonCB(HLenum event,
	HLuint object, 
	HLenum thread,
	HLcache *cache,
	void *userdata)
{
	cout << "Button " << current_button << " pressed!" << endl;
	hlGetDoublev(HL_DEVICE_TRANSFORM, button_down_transform);
	hlGetDoublev(HL_PROXY_POSITION, push_proxy_position);

}
void HLCALLBACK hlReleaseButtonCB(HLenum event,
	HLuint object, 
	HLenum thread,
	HLcache *cache,
	void *userdata)
{
	flag_release = true;
	cout << "Button " << current_button << " released!" << endl;
	hlDisable(HL_PROXY_RESOLUTION);
	hlProxydv(HL_PROXY_POSITION, push_proxy_position);
	hlEnable(HL_PROXY_RESOLUTION);

	// hlGetDoublev(HL_DEVICE_TRANSFORM, button_up_transform);
	// auto button_up_transform_inverse = button_up_transform.getInverse();
	// global_transform_matrix = button_up_transform_inverse * 
	// 						button_down_transform * 
	// 						global_transform_matrix;
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
	// Error handling
	static const HDdouble kRampUpRate = 0.0001;
	static const HDdouble kImpulseLimit = 0.001;
	hdSetDoublev(HD_SOFTWARE_FORCE_IMPULSE_LIMIT,&kImpulseLimit);
	hdSetDoublev(HD_FORCE_RAMPING_RATE, &kRampUpRate );

	max_reaction_force_vector[0] = stiffness[0]*(tissue_height[1]-tissue_height[0])/2;
	max_reaction_force_vector[1] = stiffness[1]*(tissue_height[2]-tissue_height[1])/2;
	max_reaction_force_vector[2] = stiffness[2]*(tissue_height[3]-tissue_height[2])/2;

	hdGetIntegerv(HD_CURRENT_BUTTONS, &current_button);

	//Leggi la posizione corrente del tip (dev coords)
	hdGetDoublev(HD_CURRENT_POSITION, device_pos);

	//Leggi la velocità del tip (dev coords?)
	hduVector3Dd needle_velocity_vector(0.0f, 0.0f, 0.0f);
	hdGetDoublev(HD_CURRENT_VELOCITY, needle_velocity_vector);
	HDdouble needle_velocity_magnitude = needle_velocity_vector.magnitude();

	if(is_touched){

		//Calcola la DOP
		//! valido sse superficie orizzontale
		needle_DOP = device_contact_pos[1] - device_pos[1]; 

		//Calcola il needle_vector
		//! sempre calcolato rispetto alla posizione reale del tip e non a quella ideale (ideale=sulla linea constraint)
		needle_vector = device_pos - device_contact_pos; 

		//Calcola la total_needle_penetration		 
		HDdouble total_needle_penetration;
		total_needle_penetration = needle_vector.magnitude();

		//Calcola il vettore needle_penetration_vector
		//! non serve tutto sto casino basta mantenere total needle penetration e ultimo_tessuto_needle_penetration
		HDdouble needle_penetration = 0.0f;

		if(tissue_height[0] <= needle_DOP && needle_DOP < tissue_height[1])
		{
			needle_penetration=total_needle_penetration;
		}
		else if(tissue_height[1] <= needle_DOP && needle_DOP < tissue_height[2])
		{
			HDdouble needle_DOP_fat = needle_DOP - tissue_height[1];	
			needle_penetration = (needle_DOP_fat / needle_DOP) * total_needle_penetration;
		}

		else if(tissue_height[2] <= needle_DOP && needle_DOP < tissue_height[3])
		{
			HDdouble needle_DOP_fat = needle_DOP - tissue_height[1];
			HDdouble needle_DOP_muscle = needle_DOP - tissue_height[2];
			needle_penetration = (needle_DOP_muscle / needle_DOP) * total_needle_penetration;
		}

		//Controlli sulla DOP
		if (isnan(needle_DOP))
		{
			needle_DOP = -1.0f;
		}

		//ESCI DAL TESSUTO se la DOP è < 0 e la velocità è rivolta verso l'alto
		if (needle_DOP <= 0.0f && needle_velocity_vector[1]>0)
		{
			is_touched = false;
			total_needle_penetration = 0;
			puncture[0] = 0.0f;
			puncture[1] = 0.0f;
			puncture[2] = 0.0f;
		}

		needle_DOP_difference = abs(old_needle_DOP - needle_DOP);
		
		if(needle_DOP_difference > 10)
		{
			needle_DOP = old_needle_DOP;
		}

		old_needle_DOP = needle_DOP;

		//Calcola reaction_force_direction
		hduVector3Dd reaction_force_direction(-needle_vector);
		reaction_force_direction.normalize();

		//Calcola la magnitude della forza di reazione 
		HDdouble reaction_force_magnitude = 0.0f;

		//In base allo strato in cui ti trovi (DOP) cambiano i coefficienti
		int level = 0;
		if(tissue_height[0] <= needle_DOP && needle_DOP < tissue_height[1])
		{
			level = 1;
			//metti a 0 le puncture di tutti gli strati seguenti
			puncture[1] = 0;
			puncture[2] = 0;

			//calcola la forza statica
			reaction_force_magnitude = stiffness[0]*needle_penetration;

			//se la forza statica è > FstaticaMassima, se è già avvenuta la puncture, sostituisci la forza statica con quella dinamica
			if(puncture[0] == 1 || reaction_force_magnitude > max_reaction_force_vector[0] )
			{
				reaction_force_magnitude = damping[0] * needle_penetration * needle_velocity_magnitude;
				puncture[0] = 1;
			}

			reaction_force_magnitude = 0.4;
		}
		else if(tissue_height[1] <= needle_DOP && needle_DOP < tissue_height[2])
		{
			level = 2;
			//metti a 0 le puncture di tutti gli strati seguenti
			puncture[2] = 0;

			//calcola la forza statica
			reaction_force_magnitude = stiffness[1]*needle_penetration + 
			damping[0] * (tissue_height[1]-tissue_height[0]) * needle_velocity_magnitude;

			//se la forza statica è > FstaticaMassima o se è già avvenuta la puncture, sostituisci la forza statica con quella dinamica
			if(puncture[1] == 1 || reaction_force_magnitude > max_reaction_force_vector[1] )
			{
				reaction_force_magnitude = 	(damping[0] * (tissue_height[1]-tissue_height[0]) + 
											damping[1] * needle_penetration) * needle_velocity_magnitude;
				puncture[1] = 1;
			}

			reaction_force_magnitude = 1.5;
		}
		else if(tissue_height[2] <= needle_DOP && needle_DOP < tissue_height[3])
		{
			level = 3;
			 //calcola la forza statica
			reaction_force_magnitude = 	stiffness[2]*needle_penetration + 
										(damping[0] * (tissue_height[1]-tissue_height[0]) + damping[1] * (tissue_height[2]-tissue_height[1])) 
										* needle_velocity_magnitude;

			 //se la forza statica è > FstaticaMassima o se è già avvenuta la puncture, sostituisci la forza statica con quella dinamica
			if(puncture[2] == 1 || reaction_force_magnitude > max_reaction_force_vector[2])
			{
				reaction_force_magnitude = (damping[0] * (tissue_height[1]-tissue_height[0]) +
											damping[1] * (tissue_height[2]-tissue_height[1]) +
											damping[2] * needle_penetration ) * needle_velocity_magnitude;
				puncture[2] = 1;
			}

			reaction_force_magnitude = 2;
		}
		else if(needle_DOP >= tissue_height[3])
		{
			level = 4;
			//se stai toccando l'osso setta la forza statica
			reaction_force_magnitude = 	stiffness[3] * total_needle_penetration +
										(damping[0] * (tissue_height[1]-tissue_height[0]) +
										damping[1] * (tissue_height[2]-tissue_height[1]) +
										damping[2] * (tissue_height[3]-tissue_height[2]) ) * needle_velocity_magnitude;

			reaction_force_magnitude = 2.5;
		}

		// ******************************************************
		// **************** FORZA DI REAZIONE *******************
		// ******************************************************

		//placeholder
		//reaction_force_magnitude = 0.20f;
		hduVector3Dd reaction_force(0.0f,0.0f,0.0f);
		reaction_force = reaction_force_magnitude * (-desired_direction);

		//Calcola forza di richiamo sulla linea

/*		FORZA ELASTICA DI RICHIAMO
		hduVector3Dd elastic_force(0.0f,0.0f,0.0f);
		hduVector3Dd elastic_force_direction(0.0f,0.0f,0.0f);
		HDdouble elastic_force_magnitude = 0.0f;
		HDdouble displacement = 0.0f;
		HDdouble projected_needle_velocity = 0.0f;
		double elastic_force_stiffness = 0.5f;
		double elastic_force_damping = 0.0f;

		hduVector3Dd norm_desired_direction = desired_direction;
		norm_desired_direction.normalize();

		HDdouble projection_length = needle_vector.dotProduct( norm_desired_direction);

		hduVector3Dd projection_vector(0.0f,0.0f,0.0f);
		projection_vector = norm_desired_direction * projection_length;

		displacement = sqrt( pow( needle_vector.magnitude(), 2.0) - pow( projection_length, 2.0));
		elastic_force_magnitude = elastic_force_stiffness * displacement;

		elastic_force_direction = (projection_vector - needle_vector);
		elastic_force_direction.normalize();

		projected_needle_velocity = needle_velocity_vector.dotProduct(elastic_force_direction);
		
		elastic_force = (elastic_force_magnitude - elastic_force_damping * projected_needle_velocity) * elastic_force_direction;

		//Attenua la forza elastica se sei a meno di 3 mm dall'asse desiderato
		if(displacement  < 3)
		{
			elastic_force = elastic_force * displacement/3;
		}

		//Rescaling and Clamping della reaction force
		reaction_force_magnitude = reaction_force.magnitude();
		reaction_force_direction = reaction_force;
		reaction_force_direction.normalize();

		reaction_force_magnitude = reaction_force_magnitude*2.5/30000;
		if(reaction_force_magnitude > 2.6)
			reaction_force_magnitude = 2.6;
		reaction_force = reaction_force_magnitude * reaction_force_direction;
/**/


		// FORZA DI ATTRITO SPAZIALE
		HDdouble damping_force_magnitude= 0.0f;
		double damping_coeff = 0.2;

		damping_force_direction = -needle_velocity_vector;
		damping_force_direction.normalize();

		damping_force_magnitude = needle_velocity_magnitude * damping_coeff;

		damping_force = damping_force_magnitude * damping_force_direction;

		hduVector3Dd norm_desired_direction = desired_direction;
		norm_desired_direction.normalize();

		auto projected_damping_force_magnitude = damping_force.dotProduct(norm_desired_direction);
		auto projected_damping_force = projected_damping_force_magnitude * norm_desired_direction;
		
		damping_force = damping_force - projected_damping_force;


		//Calcola forza totale
		total_force = reaction_force*0 + damping_force/100;
		hdSetDoublev(HD_CURRENT_FORCE, total_force*1.0);

		HDdouble max_force=0.0f;
		hdGetDoublev(HD_NOMINAL_MAX_FORCE, &max_force);

		if (count == 750)
		{	
			//cout << "NEEDLE VELOCITY:\t" << needle_velocity_vector[1] << endl;
			cout << "total force =          \t" << total_force.magnitude() << endl; 
			cout << "damping force =        \t" << damping_force_magnitude << endl; 
			cout << "livello =              \t" << level << endl;

			cout << "Current Button         \t" << current_button << endl;

			cout << "******************************" << endl;
			count = 0;
		}
		count++;

	}

	// Error handler
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
	if (is_touched)
	{
		glPushMatrix();
		initLine();
		glPopMatrix();
	}

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

	// HLdouble proxyxform[16];
	hduMatrix proxyxform;
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

		gluCylinder(qobj, kCursorRadius, 
			0.0, kCursorHeight / 5.0,
			kCursorTess, kCursorTess);

		gluDeleteQuadric(qobj);
		glEndList();
	}

	// Get the proxy transform in world coordinates.
	// If entered hole, then freeze the rotations of the needle to the one at the contact
	hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
	hlGetDoublev(HL_DEVICE_TRANSFORM, current_device_transform);
/*
	temp_transform = button_up_transform.getInverse() * current_device_transform;
	if (jj == 25)
	{
		cout << endl << "temp_transform " << endl << temp_transform << endl;
		cout << "flag_release" << flag_release << endl;
		jj = 0;
	}
	jj++;

*/

	if (is_touched)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				proxyxform(i,j) = device_transf(i,j);
			}
		}
		auto y_contact_point = proxy_contact_pos[1];
		auto y_curr = proxyxform(3,1);

		proxyxform(3,0) = ((y_curr - y_contact_point)/desired_direction[1] * desired_direction[0]) + proxy_contact_pos[0];
		proxyxform(3,2) = ((y_curr - y_contact_point)/desired_direction[1] * desired_direction[2]) + proxy_contact_pos[2];
	}
/**/

/*	
	hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
	proxyxform = proxyxform * global_transform_matrix;
	if (current_button != 3)
	{
		// Get the proxy transform in world coordinates.
		// If entered hole, then freeze the rotations of the needle to the one at the contact
		if (is_touched)
		{
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					proxyxform(i,j) = device_transf(i,j);
				}
			}
			auto y_contact_point = proxy_contact_pos[1];
			auto y_curr = proxyxform(3,1);

			proxyxform(3,0) = ((y_curr - y_contact_point)/desired_direction[1] * desired_direction[0]) + proxy_contact_pos[0];
			proxyxform(3,2) = ((y_curr - y_contact_point)/desired_direction[1] * desired_direction[2]) + proxy_contact_pos[2];
		}
	}
	else
	{	
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				proxyxform(i,j) = device_transf(i,j);
			}
		}
	}
/**/

	// if (flag_release)
	// {
	// 	proxyxform = proxyxform * global_transform_matrix;
	// }


	//Get the depth of Penetration from HLAPI.
	//hlGetDoublev(HL_DEPTH_OF_PENETRATION, &needle_DOP);
	glMultMatrixd(proxyxform);



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
	hlGetDoublev(HL_PROXY_POSITION, proxy_position);

	// Start haptic frame
	hlBeginFrame();
	hlPushMatrix();

	//************** SHAPE 1: cube **************//
	//! Here we have to modify those haptic params to 
	//! model the wanted interactions (POPTHROUGH, constraints ..)
	if (!is_touched)
	{
		// cout << "*************\n NON HO ANCORA TOCCATO \n *********************" << endl;
		hlTouchModel(HL_CONTACT);
		hlMaterialf(HL_FRONT, HL_STIFFNESS, 0.9f);
		hlMaterialf(HL_FRONT, HL_DAMPING, 0.0f);
		hlMaterialf(HL_FRONT, HL_STATIC_FRICTION, 0.0);
		hlMaterialf(HL_FRONT, HL_DYNAMIC_FRICTION,0.0);
		// hlMaterialf(HL_FRONT, HL_POPTHROUGH, 0.3);
		hlHinti(HL_SHAPE_FEEDBACK_BUFFER_VERTICES, cube_model->numvertices);
		hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, cube_id); //! FEEDBACK or DEPTH??

		glPushMatrix();
		glCallList(cube_obj_list);
		glPopMatrix();
		hlEndShape();
	}

	if (is_touched)
	{
		glPushMatrix();
		/*hlTouchModel(HL_CONSTRAINT);
		hlMaterialf(HL_FRONT, HL_STIFFNESS, 0.9);
		hlMaterialf(HL_FRONT, HL_STATIC_FRICTION, 0.0);
		hlMaterialf(HL_FRONT, HL_DYNAMIC_FRICTION, 0.0);*/
		//hlTouchModelf(HL_SNAP_DISTANCE, 300);
		hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER,line_id);
		initLine();
		hlEndShape();
		glPopMatrix();
	}

	// if (is_touched)
	// {
	// 	hlPushMatrix();
	// 	hlTouchModel(HL_CONSTRAINT);
	// 	hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.4f);
	// 	hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.3f);
	// 	hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.1);
	// 	hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION,0.1 );

	// 	hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, contact_point_id);
	// 	glPushMatrix();
	// 	glPointSize(5.0);
	// 	// glTranslatef(0.0, 0.0, 1.0);
	// 	glTranslatef(proxy_contact_pos[0],
	// 		proxy_contact_pos[1],
	// 		proxy_contact_pos[2]);
	// 	glBegin(GL_POINTS);
	// 	// glVertex3f(0.05,-0.175,-0.975);
	// 	glVertex3f(0.0f,0.0f,0.0f);
	// 	glEnd();

	// 	glPopMatrix();
	// 	hlEndShape();
	// 	hlPopMatrix();
	// }

	// End the haptic frame.
	hlPopMatrix();
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
		hlDeleteShapes(line_id, 1);
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