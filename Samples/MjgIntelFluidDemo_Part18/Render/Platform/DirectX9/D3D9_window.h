/** \file D3D9_window.h

    \brief Window render target for OpenGL

    \author Written and Copyright 2010-2014 Michael Jason Gourlay; All rights reserved.
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
        /** Window render target for Direct3D 9.
        */
        class D3D9_Window : public Window
        {
        public:
            static const unsigned sType = 'D9WN' ; ///< Type identifier for this class

            enum Parameters
            {
                pHinst  = 'hins'    ,
                NUM_PARAMETERS
            } ;

            D3D9_Window( class Render::System * renderSystem ) ;
            virtual ~D3D9_Window() ;

            virtual void    Create() ;
            virtual void    Change() ;
            virtual int     GetState( StateE state ) ;

#       if defined( _DEBUG )
            static void UnitTest() ;
#       endif

        private:
            virtual /* implements */ void SetWindow() ;

            HWND                mWindowHandle   ;   /// Handle to the window.
            IDirect3DSurface9 * mRenderTarget   ;   /// D3D render target associated with this Window.

        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;
} ;

#endif
