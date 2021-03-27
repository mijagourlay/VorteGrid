/** \file light.cpp

    \brief Scene node representing a light

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Scene/light.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------

static const float sLightRangeMax = fsqrtf( FLT_MAX ) ;

// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct scene node representing a light.

            \note Default values roughly follow those of OpenGL.
        */
        Light::Light( ISceneManager * sceneManager )
            : SceneNodeBase( sceneManager , sTypeId )
            , mLightType( DIRECTIONAL )
            , mDirection( 0.0f , 0.0f , 1.0f , 0.0f )
            , mAmbientColor( 0.0f , 0.0f , 0.0f , 1.0f )
            , mDiffuseColor( 1.0f , 1.0f , 1.0f , 1.0f )
            , mSpecularColor( 1.0f , 1.0f , 1.0f , 1.0f )
            , mRange( sLightRangeMax )
            , mSpotFalloff( 0.0f )
            , mSpotInnerAngle( 0.0f )
            , mSpotOuterAngle( 180.0f )
            , mConstAttenuation( 1.0f )
            , mLinearAttenuation( 0.0f )
            , mQuadraticAttenuation( 0.0f )
        {
            PERF_BLOCK( Light__Light ) ;

            SetPosition( Vec4( 0.0f , 0.0f , 2.0f , 1.0f ) ) ;
        }




        /** Destruct scene node representing a light.
        */
        Light::~Light()
        {
            PERF_BLOCK( Light__dtor ) ;
        }




        /** Render a light.

            Normally, rendering a light does nothing, but for diagnosing a scene it
            could be useful to render geometry at the location of a light, to visualize
            light sources.
        */
        void Light::Render()
        {
            PERF_BLOCK( Light__Render ) ;

            RenderChildren() ;
        }




#if defined( _DEBUG )

        void PeGaSys_Render_Light_UnitTest()
        {
            DebugPrintf( "Light::UnitTest ----------------------------------------------\n" ) ;

            {
                Light light( 0 ) ;

                // Update light
                light.Update() ;
            }

            DebugPrintf( "Light::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
