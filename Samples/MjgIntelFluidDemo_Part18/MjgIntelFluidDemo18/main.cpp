/** \file main.cpp

    \brief Main application driver for fluid simulation.

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "inteSiVis.h"

#include "Core/Performance/perfBlock.h"

#include "Render/Platform/OpenGL/OpenGL_api.h"

#if defined( WIN32 )
    #include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <time.h>   // for time
#include <stdlib.h> // for atexit
#include <stdio.h>  // for sprintf, printf, freopen, etc.

#pragma comment(lib, "glut32.lib")




static time_t sStartTime = 0 ;  // Time when process started.  Used to disambiguate and coordinate log files.




static void AtExitHandler()
{
#if PROFILE
    printf( "Terminated: built " __DATE__ " " __TIME__ " ran %i\n\n" , sStartTime ) ;
#endif
}




/// Application entry point.
int main( int argc , char ** argv )
{
#if PROFILE
    // Create common suffix for log files.
    sStartTime = time( NULL ) ;
    char filenameSuffix[ 128 ] ;
    sprintf( filenameSuffix , "%i" , sStartTime ) ;

    // Redirect stdout to a file to collect profile and other messages.
    {
        char stdoutFilename[ 256 ] ;
        sprintf( stdoutFilename , "stdout-%s.log" , filenameSuffix ) ;
        DEBUG_ONLY( FILE * stdoutFilePointer = ) freopen( stdoutFilename , "a" , stdout ) ;
        ASSERT( stdoutFilePointer != NULL ) ;   // Could fail if user has no permission to access file.
        printf( "%s:\n" , argv[ 0 ] ) ;
        printf( "Written by Michael Jason Gourlay\n" ) ;
        printf( "Built " __DATE__ " " __TIME__ " ran %i\n" , sStartTime ) ;
    }

    // Redirect performance profile data to a CSV file.
    char perfFileName[ 256 ] ;
    sprintf( perfFileName , "perf-%s.csv" , filenameSuffix ) ;
    PerfLoggerExample perfLogger( perfFileName ) ;
    PerfBlock::SetLogFunc( PerfLoggerExample::LogFunc ) ;
#endif

    char mainPerfBlockName[ 256 ] ;
    sprintf( mainPerfBlockName , "VorteGrid__main__built_" __DATE__ " " __TIME__ "_run_%i" , sStartTime ) ;

    PERF_BLOCK_MAIN( mainPerfBlockName ) ;  // Must have exactly one PERF_BLOCK_MAIN per thread.

    atexit( AtExitHandler ) ;

    // Set floating-point unit options to ensure speed and consistency.
    XMM_Set_FlushToZero_DenormalAreZero() ;
    Setx87Precision( PRECISION_SINGLE ) ;
    //Unmaskx87FpExceptions() ;

    PeGaSys::Render::ApiBase * renderApi = NEW PeGaSys::Render::OpenGL_Api ;

    glutInit( & argc , argv ) ;

    InteSiVis inteSiVis( renderApi ) ;
    inteSiVis.InitializeDisplayAndInputDevices() ;

    glutMainLoop() ; // Relinquish control to GLUT.  This never returns.

    return 0 ;
}
