/** \file window.cpp

    \brief Window render target

    \author Written and copyright 2010-2014 Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "Render/Device/window.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Macros -----------------------------------------------------------------------

#ifndef FREE    // Not using memory wrappers
#   define FREE free
#   define STRDUP strdup
#endif

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        static char * sDefaultWindowName = "PeGaSys" ;

        /** Construct window render target.
        */
        Window::Window( class Render::System * renderSystem )
            : Target( renderSystem , sTypeId )
            , mName( sDefaultWindowName )
            , mFullScreen( false )
        {
            PERF_BLOCK( Window__Window ) ;

            SetDepth( true ) ;
        }




        /** Destruct window render target.
        */
        Window::~Window()
        {
            PERF_BLOCK( Window__dtor ) ;

            FreeName() ;
        }




        void Window::FreeName()
        {
            PERF_BLOCK( Window__FreeName ) ;

            if( mName && ( mName != sDefaultWindowName ) )
            {
                FREE( mName ) ;
            }
        }




        void Window::SetName( const char * name )
        {
            PERF_BLOCK( Window__SetName ) ;

            FreeName() ;
            mName = STRDUP( name ) ;
        }




        /** Detect whether on-screen window was resized, and, if so, respond.
        */
        void Window::DetectAndHandleResize()
        {
            PERF_BLOCK( Window__DetectAndHandleResize ) ;

            const int newWidth  = GetState( WIDTH  ) ;
            const int newHeight = GetState( HEIGHT ) ;
            if( ( GetWidth() != newWidth ) || ( GetHeight() != newHeight ) )
            {   // On-screen window size does not match this object size.  It probably resized since last update.
                // Update this object to reflect on-screen window size.
                SetWidth( newWidth ) ;
                SetHeight( newHeight ) ;
            }
        }




        /* virtual */ void Window::PreRenderViewports()
        {
            PERF_BLOCK( Window__PreRenderViewports ) ;

            SetWindow() ;
            DetectAndHandleResize() ;
        }






#if defined( _DEBUG )




        class TestWindow : public PeGaSys::Render::Window
        {
        public:
            TestWindow( Render::System * renderSystem )
                : Window( renderSystem )
            {
            }

            virtual ~TestWindow() {}

            virtual void    Create() {}
            virtual void    Change() {}
            virtual int     GetState( StateE /*state*/ ) { return 0 ; }

        private:
            virtual void SetWindow() {}
        } ;

        void PeGaSys::Render::Window::UnitTest( void )
        {
            DebugPrintf( "Window::UnitTest ----------------------------------------------\n" ) ;

            {
                TestWindow window( 0 ) ;

                window.GetTypeId() ;
                window.SetWidth( 640 ) ;
                window.SetHeight(  480  ) ;
                window.SetDepth( true ) ;
                window.SetName( "MyWindow" ) ;
                window.SetFullScreen( false ) ;
                window.SetLeft( 20 ) ;
                window.SetTop( 20 ) ;

                window.Create() ;
                window.Change() ;
                window.RenderViewports( 0.0 ) ;
            }

            DebugPrintf( "Window::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
