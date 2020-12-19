/** \file pass.cpp

    \brief Render state and material within a Technique used to render a mesh.

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/pass.h"
#include "Render/Resource/texture.h"
#include "Render/Device/api.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct render state within a technique used to render a mesh.
        */
        Pass::Pass( Technique * parentTechnique )
            : mParentTechnique( parentTechnique )
        {
            PERF_BLOCK( Pass__Pass ) ;
        }




        /** Destruct render state within a technique used to render a mesh.
        */
        Pass::~Pass()
        {
            PERF_BLOCK( Pass__dtor ) ;
        }




        void Pass::Apply( ApiBase * renderApi ) const
        {
            PERF_BLOCK( Pass__Apply ) ;

            renderApi->ApplyRenderState( GetRenderState() ) ;
            if( ! mTextureStages.Empty() )
            {
                for( size_t unit = 0 ; unit < mTextureStages.Size() ; ++ unit )
                {   // For each texture unit...
                    TextureBase * texture = mTextureStages[ unit ].mTexture.Get() ;
                    ASSERT( texture ) ;
                    texture->Bind( renderApi , mTextureStages[ unit ].mSamplerState ) ;
                }
            }
            else
            {   // Mesh has no texture.
                renderApi->DisableTexturing() ;
            }
        }




        /** Add an empty TextureStage to this pass, for caller to populate.
        */
        void Pass::AddTextureStage()
        {
            PERF_BLOCK( Pass__AddTextureStage ) ;

            mTextureStages.PushBack( TextureStage() ) ;
        }




        /** Add an empty TextureStage to this pass, based on one passed in as a template.
        */
        void Pass::AddTextureStage( const TextureStage & textureStage )
        {
            PERF_BLOCK( Pass__AddTextureStage ) ;

            mTextureStages.PushBack( textureStage ) ;
        }




    } ;
} ;




#if defined( _DEBUG )

void PeGaSys::Render::Pass::UnitTest()
{
    DebugPrintf( "Pass::UnitTest ----------------------------------------------\n" ) ;

    {
        Pass pass( 0 ) ;
    }

    DebugPrintf( "Pass::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif