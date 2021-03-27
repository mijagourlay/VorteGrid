/** \file pass.h

    \brief Render state and material within a Technique used to render a mesh.

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_PASS_H
#define PEGASYS_RENDER_PASS_H

#include "Render/Resource/renderState.h"
#include "Render/Resource/textureSampler.h"

#include "Core/Containers/vector.h"
#include "Core/Containers/intrusivePtr.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class ApiBase ;
        class Technique ;

        /** Render state and material within a Technique used to render a mesh.
        */
        class Pass
        {
            public:
                explicit Pass( Technique * parentTechnique ) ;
                ~Pass() ;

                void Apply( ApiBase * renderApi ) const ;

                      RenderState & GetRenderState()       { return mRenderState ; }
                const RenderState & GetRenderState() const { return mRenderState ; }

                size_t GetNumTextureStages() const { return mTextureStages.Size() ; }

                void AddTextureStage() ;
                void AddTextureStage( const TextureStage & textureStage ) ;

                      TextureStage & GetTextureStage( size_t idxStage )       { return mTextureStages[ idxStage ] ; }
                const TextureStage & GetTextureStage( size_t idxStage ) const { return mTextureStages[ idxStage ] ; }

                void SetActive( bool active ) { mActive = active ; }

                bool IsActive() const { return mActive ; }

            #if defined( _DEBUG )
                static void UnitTest() ;
            #endif

            private:
                Technique   *           mParentTechnique    ;   ///< Technique that owns this Pass.
                RenderState             mRenderState        ;   ///< RenderState.
                VECTOR< TextureStage >  mTextureStages      ;   ///< Array of texture stages.
                bool                    mActive             ;   ///< Whether or not to render with this pass.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
