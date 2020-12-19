/** \file textureSampler.cpp

    \brief Texture unit stage

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/textureSampler.h"

#include "Render/Resource/texture.h"

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

        /** Construct texture stage.
        */
        TextureStage::TextureStage()
        {
            PERF_BLOCK( TextureStage__TextureStage ) ;
        }




        /** Destruct texture stage.
        */
        TextureStage::~TextureStage()
        {
            PERF_BLOCK( TextureStage__dtor ) ;
        }




    } ;
} ;




#if defined( _DEBUG )





void PeGaSys::Render::TextureStage::UnitTest()
{
    DebugPrintf( "TextureSampler::UnitTest ----------------------------------------------\n" ) ;

    {
        TextureStage textureSampler ;
    }

    DebugPrintf( "TextureSampler::UnitTest: THE END ----------------------------------------------\n" ) ;
}
#endif