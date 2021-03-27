/** \file OpenGL_window.cpp

    \brief Window render target for OpenGL

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Platform/OpenGL/OpenGL_window.h"

#include "Render/Platform/OpenGL/OpenGL_api.h" // For GL_CHECK_ERROR

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#if defined( WIN32 )
#   include <windows.h>
#endif

#include <GL/glut.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct window render target.
        */
        OpenGL_Window::OpenGL_Window( class Render::System * renderSystem )
            : Window( renderSystem )
        {
            PERF_BLOCK( OpenGL_Window__OpenGL_Window ) ;
        }




        /** Destruct window render target.
        */
        OpenGL_Window::~OpenGL_Window()
        {
            PERF_BLOCK( OpenGL_Window__dtor ) ;
        }




        typedef BOOL (APIENTRY *PFNWGLSWAPINTERVALFARPROC)( int );
        static PFNWGLSWAPINTERVALFARPROC wglSwapIntervalEXT = 0;

        /** Set whether buffer swap happens during VSync.

        \param interval     Minimum number of video frames displayed before a buffer
        swap will occur.
        */
        void setVSync(int interval=1)
        {
            PERF_BLOCK( Render__setVSync ) ;

            const char * extensions = (char*) glGetString( GL_EXTENSIONS );

            if( strstr( extensions, "WGL_EXT_swap_control" ) == 0 )
            {   // Your computer does NOT support WGL_EXT_swap_control extension.
                return ;
            }
            else
            {
                wglSwapIntervalEXT = (PFNWGLSWAPINTERVALFARPROC) wglGetProcAddress( "wglSwapIntervalEXT" ) ;
                if( wglSwapIntervalEXT )
                {   // Set the vsync mode
                    wglSwapIntervalEXT( interval ) ;
                }
            }
        }




        void OpenGL_Window::Create()
        {
            PERF_BLOCK( OpenGL_Window__Create ) ;

            glutInitWindowSize( mWidth , mHeight );
            glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH ) ;

            // NOTE: Calling glutCreateWindow incorrect causes GLUT to think it has been initialized, and subsequent calls to glutInit will exit early.
            // Caller should therefore have called glutInit before calling this function.
            // NOTE: Calling glutCreateWindow causes AppVerifier to detect and report a first-chance exception in the call to CreateWindow.  Continuing past it appears to work.
            mGlutWindowId = glutCreateWindow( mName ) ;

            OpenGL_Api::GetProcAddresses() ;

            // Enable waiting for vsync.
            setVSync( 1 ) ;

            RENDER_CHECK_ERROR( Create ) ;
        }




        void OpenGL_Window::Change()
        {
            PERF_BLOCK( OpenGL_Window__Change ) ;
        }




        void OpenGL_Window::SetWindow()
        {
            PERF_BLOCK( OpenGL_Window__SetWindow ) ;

            glutSetWindow( mGlutWindowId ) ; // Set current GLUT window to that associated with this object, so glutGet applies to this window.
        }




        /** Obtain state information about this window.
        */
        int OpenGL_Window::GetState( StateE state )
        {
            PERF_BLOCK( OpenGL_Window__GetState ) ;

            const int originalGlutWindow = glutGetWindow() ;    // Remember original window to restore it before leaving this routine.

            glutSetWindow( mGlutWindowId ) ;    // Set current GLUT window to that associated with this object, so glutGet applies to this window.

            switch( state )
            {
            case POSITION_X : return glutGet( GLUT_WINDOW_X         ) ; break ;
            case POSITION_Y : return glutGet( GLUT_WINDOW_Y         ) ; break ;
            case WIDTH      : return glutGet( GLUT_WINDOW_WIDTH     ) ; break ;
            case HEIGHT     : return glutGet( GLUT_WINDOW_HEIGHT    ) ; break ;
            }

            glutSetWindow( originalGlutWindow ) ;   // Reinstate original GLUT window as current.

            FAIL() ;    // state has invalid value.
            return -1 ; // Return invalid value.
        }


    } ;
} ;



#if defined( _DEBUG )




void PeGaSys::Render::OpenGL_Window::UnitTest()
{
    DebugPrintf( "OpenGL_Window::UnitTest ----------------------------------------------\n" ) ;

    {
        OpenGL_Window window( 0 ) ;

        window.GetTypeId() ;
        window.SetWidth( 640 ) ;
        window.SetHeight( 480 ) ;
        window.SetDepth( true ) ;
        window.SetName( "MyWindow" ) ;
        window.SetFullScreen( false ) ;
        window.SetLeft( 20 ) ;
        window.SetTop( 20 ) ;

        window.Create() ;
        window.Change() ;
        window.RenderViewports( 0.0 ) ;
    }

    DebugPrintf( "OpenGL_Window::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif