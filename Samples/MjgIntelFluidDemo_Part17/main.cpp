/** \file main.cpp

    \brief Main application driver for fluid simulation.

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "inteSiVis.h"

#include "Core/Performance/perf.h"

#if defined( WIN32 )
    #include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#pragma comment(lib, "glut32.lib")




/// Application entry point.
int main( int argc , char ** argv )
{
    fprintf( stderr , "Benchmark: " __DATE__ " " __TIME__ "\n" ) ;

#if 0 // DO NOT SUBMIT
    extern int other_sph_main (int argc, char** argv) ;
    other_sph_main( argc , argv ) ;
#endif

    XMM_Set_FlushToZero_DenormalAreZero() ;
    Setx87Precision( PRECISION_SINGLE ) ;
    //Unmaskx87FpExceptions() ;

#if PROFILE
    {   // Redirect stderr to a file to collect profile data.
        freopen( "profile.log" , "a" , stderr ) ;
        fprintf( stderr , "Benchmark: " __DATE__ " " __TIME__ "\n" ) ;
    }
#endif

    InteSiVis inteSiVis ;
    inteSiVis.InitializeDisplayandInputDevices( & argc , argv ) ;

    glutMainLoop() ; // Relinquish control to GLUT.  This never returns.

    return 0 ;
}
