/** \file viewport.cpp

    \brief Rectangular region of a render target

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.
*/

#include "Render/system.h"
#include "Render/Device/target.h"
#include "Render/Device/viewport.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#include <algorithm>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct rectangular region of a render target.
        */
        Viewport::Viewport( Camera * camera , Target * target , const Vec4 & viewport )
            : mCamera( camera )
            , mTarget( target )
            , mRelLeft( viewport.x )
            , mRelTop( viewport.y )
            , mRelWidth( viewport.z )
            , mRelHeight( viewport.w )
            , mClearColor( 1.0f , 0.0f , 1.0f , 1.0f )
            , mDiagnosticTextOverlay( target->GetRenderSystem()->GetApi() )
        {
            PERF_BLOCK( Viewport__Viewport ) ;
        }




        /** Destruct rectangular region of a render target.
        */
        Viewport::~Viewport()
        {
            PERF_BLOCK( Viewport__dtor ) ;
        }




        /** Trigger the camera into this viewport: Clear viewport, set viewport and render scene.

            Also render overlays.

            \todo   Maybe rename this routine to RenderSceneAndOverlays.

        */
        void Viewport::TriggerCamera()
        {
            PERF_BLOCK( Viewport__TriggerCamera ) ;

            // Cycle clear color.
            mClearColor.x = std::max( fmod( mClearColor.x + 0.05f , 1.0f ) , 0.2f ) ;
            mClearColor.y = std::max( fmod( mClearColor.y + 0.02f , 1.0f ) , 0.2f ) ;
            mClearColor.z = std::max( fmod( mClearColor.z + 0.01f , 1.0f ) , 0.2f ) ;

            // Set up render target.
            mTarget->GetRenderSystem()->GetApi()->SetViewport( * this ) ;

            const float viewportAspectRatio = GetRelWidth() * float( mTarget->GetWidth() ) / ( GetRelHeight() * float( mTarget->GetHeight() ) ) ;
            mCamera->SetAspectRatio( viewportAspectRatio ) ;

            // Render scene.
            mCamera->RenderScene() ;

            // TODO: Set orthogonal projection for overlay. Add SetOrthographic to either Api or Camera, and call from here.
            // TODO: Render overlays here.

            // Render debug text overlay.
            mDiagnosticTextOverlay.Render() ;
        }




    } ;
} ;




#if defined( _DEBUG )

void PeGaSys_Render_Viewport_UnitTest( void )
{
    DebugPrintf( "Viewport::UnitTest ----------------------------------------------\n" ) ;

    {
        PeGaSys::Render::Viewport viewport( 0 , 0 , Vec4( 0.0f , 0.0f , 1.0f , 1.0f ) )  ;

        // Update rectangular render target.
        viewport.TriggerCamera() ;
    }

    DebugPrintf( "Viewport::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif