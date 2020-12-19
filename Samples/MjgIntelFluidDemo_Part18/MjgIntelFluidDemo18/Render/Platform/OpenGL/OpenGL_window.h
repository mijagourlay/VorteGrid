/** \file OpenGL_window.h

    \brief Window render target for OpenGL

    \author Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_WINDOW_H
#define PEGASYS_RENDER_OPENGL_WINDOW_H

#include "Render/Device/window.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Window render target for OpenGL.
        */
        class OpenGL_Window : public Window
        {
            public:
                static const unsigned sType = 'OGWN' ; ///< Type identifier for this class

                OpenGL_Window( class Render::System * renderSystem ) ;
                virtual ~OpenGL_Window() ;

                virtual void    Create() ;
                virtual void    Change() ;
                virtual int     GetState( StateE state ) ;

            #if defined( _DEBUG )
                static void UnitTest() ;
            #endif

            private:
                virtual /* implement */ void SetWindow() ;

                int mGlutWindowId   ;   /// Index of GLUT window associated with this OpenGL_Window.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
