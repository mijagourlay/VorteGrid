/** \file D3D9_renderState.h

    \brief Wrapper for routines specific to D3D9 render system API

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_D3D9_RENDER_STATE_H
#define PEGASYS_RENDER_D3D9_RENDER_STATE_H

#include "Render/Resource/renderState.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        struct RenderStateS ;

        /** Wrapper for routines specific to D3D9 render system API.
        */
        class D3D9_RenderStateCache
        {
            public:
                static const unsigned sType = 'RAd9' ; ///< Type identifier for this class

                D3D9_RenderStateCache() ;
                ~D3D9_RenderStateCache() ;

                void Apply( const RenderStateS & renderState ) ;

                const RenderStateS & GetRenderStateCache() const { return mCurrentState ; }

                static void GetRenderState( RenderStateS & renderState ) ;

            private:
                friend class D3D9_Api ; // Allow D3D9_Api to set mCurrentState.

                RenderStateS    mCurrentState   ;   ///< Cache of current render state.  Used to avoid unnecessary state change calls into underlying API.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
