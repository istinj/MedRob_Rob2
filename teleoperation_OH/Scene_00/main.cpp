/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name: 

  main.cpp

Description:

  Main program for rigid body simulation.

*******************************************************************************/

#pragma warning( disable : 4786 )  // identifier was truncated to '255' 
                                   // the "you are using STL" warning

#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

#include <HL/hl.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>

#include "World.h"

const int kMaxStepSizeMs = 33;

enum 
{  
    DEFAULT_WINDOW_WIDTH = 640,
    DEFAULT_WINDOW_HEIGHT = 480 
};
                
GLint mWindW=DEFAULT_WINDOW_WIDTH;
GLint mWindH=DEFAULT_WINDOW_HEIGHT;

HHD hHD = HD_INVALID_HANDLE;
HHLRC hHLRC = NULL;

World my_world;

//void exitHandler();
void initHL(void);
void initGraphics(void);
void setupCallbacks(void);
void createMenus(void);
void display_function(void);
void initialize_world(void);

int main(int argc, char **argv)
{
    // initialize GLUT
    glutInit(&argc, argv);

	// Initialize the Geomagic
	initHL();
	// World
	initialize_world();
	// Graphic stuff
	initGraphics();
	setupCallbacks();

    /* The GLUT main loop won't return control, so we need to perform cleanup
       using an exit handler. */
    //atexit(exitHandler);

    glutMainLoop();

    return 0;
}

void initHL(void)
{
	HDErrorInfo error;
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "Press any key to exit");
        getchar();
        exit(1);
    }
	hHLRC = hlCreateContext(hHD);
	hlMakeCurrent(hHLRC);

	hlTouchableFace(HL_FRONT);
}

void initGraphics(void)
{
    // display modes: 24 BIT, double buffer mode
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        
    glutInitWindowSize(mWindW,mWindH);
    glutCreateWindow("Demo");

    // clear the display
    //glClearColor(1.0,1.0,1.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* set default attributes */
    //glPolygonMode(GL_BACK,GL_POINT);
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glLineWidth(3);
    glPointSize(4);

    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
        
    GLfloat matSpecular[] = { 1.0, 1.0, 1.0, 1.0 };
    //GLfloat matSpecular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat matShininess[] = { 50.0 };
    GLfloat lightPosition[] = { -200.0, 200.0, 100.0, 0.0 };
    //GLfloat lightPosition[] = { 0.0, 400.0, 0.0, 0.0 };
    GLfloat whiteLight[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lModelAmbient[] = { 0.1, 0.1, 0.1, 0.1 };
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glMaterialfv(GL_FRONT, GL_SPECULAR, matSpecular);
    glMaterialfv(GL_FRONT, GL_SHININESS, matShininess);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, whiteLight);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lModelAmbient);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
}

void setupCallbacks(void)
{
	// Setup delle varie callback
	// Display function (haptic rendering e gl rendering)
	glutDisplayFunc(display_function);
}

void display_function(void) 
{
	// Frame haptico iniziato
	hlBeginFrame();
	hlCheckEvents();

	// Clear display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//! TO DO

}

void initialize_world(void)
{
	my_world.init();
	//! Initializzare la simulazione.
}

void createMenus(void);


/*******************************************************************************
 Cleanup
*******************************************************************************/
//void exitHandler()
//{
//    if (mWorld != NULL)
//    {
//        delete mWorld;
//    }
//
//    // free up the haptic rendering context
//    hlMakeCurrent(NULL);
//    if (hHLRC != NULL)
//    {
//        hlDeleteContext(hHLRC);
//    }
//
//    // free up the haptic device
//    if (hHD != HD_INVALID_HANDLE)
//    {
//        hdDisableDevice(hHD);
//    }
//}

/******************************************************************************/
