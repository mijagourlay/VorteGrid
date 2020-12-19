/** \file OpenGL_renderState.h

    \brief Wrapper for routines specific to OpenGL render system API

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_RENDER_STATE_H
#define PEGASYS_RENDER_OPENGL_RENDER_STATE_H

#include "Render/Resource/renderState.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Wrapper for routines specific to OpenGL render system API.
        */
        class OpenGL_RenderStateCache
        {
            public:
                static const unsigned sType = 'RAog' ; ///< Type identifier for this class

                OpenGL_RenderStateCache() ;
                ~OpenGL_RenderStateCache() ;

                void Apply( const RenderStateS & renderState  ) ;

                const RenderStateS & GetRenderStateCache() const { return mCurrentState ; }

                static void GetRenderState( RenderStateS & renderState ) ;

            private:
                friend class OpenGL_Api ; // Allow OpenGL_Api to set mCurrentState.

                RenderStateS mCurrentState   ;   ///< Cache of current render state.  Used to avoid unnecessary state change calls into underlying API.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------


    } ;
} ;

#endif
