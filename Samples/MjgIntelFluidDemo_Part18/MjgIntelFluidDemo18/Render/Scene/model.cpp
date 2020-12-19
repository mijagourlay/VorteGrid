/** \file model.cpp

    \brief Scene node representing a model

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Scene/model.h"

#include "Render/Device/api.h"
#include "Render/Resource/mesh.h"
#include "Render/Scene/iSceneManager.h"
#include "Render/Scene/modelData.h"
#include "Render/Scene/light.h"

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

        /** Construct scene node representing a model.
        */
        ModelNode::ModelNode( ISceneManager * sceneManager )
            : SceneNodeBase( sceneManager , sTypeId )
            , mNumLights( 0 )
        {
            PERF_BLOCK( ModelNode__ModelNode ) ;

            DEBUG_ONLY( memset( mLightsCache , 0 , sizeof( mLightsCache ) ) ) ;
        }




        /** Destruct scene node representing a model.
        */
        ModelNode::~ModelNode()
        {
            PERF_BLOCK( ModelNode__dtor ) ;
        }




        /** Render a model.
        */
        void ModelNode::Render()
        {
            PERF_BLOCK( ModelNode__Render ) ;

            SetLocalToWorld() ;

            const Camera *  camera      = GetSceneManager()->GetCurrentCamera() ;
            ApiBase *       renderApi   = GetSceneManager()->GetApi() ;

            renderApi->SetCamera( * camera ) ;

            // In OpenGL, lights are transformed by current MODELVIEW matrix but light node is in world space so must set view (camera) before setting lights.  Alternatively, could set view matrix in OpenGL_Api::SetLight.
            renderApi->SetLights( * this ) ;

            renderApi->SetLocalToWorld( GetLocalToWorld() ) ;

            mModelData->Render( renderApi ) ;

            // TODO: Render diagnostic text at this location (0,0,0)

            RenderChildren() ;
        }




        /** Assign a ModelData object to this ModelNode.  This implies the ModelData object is shared across ModelNodes.

            \seealso NewModelNode
        */
        void ModelNode::SetModelData( ModelData * modelData )
        {
            PERF_BLOCK( ModelNode__SetModelData ) ;

            ASSERT( ! mModelData ) ;
            mModelData = modelData ;
        }




        /** Create a new ModelData object and assign it to this ModelNode object.
        */
        ModelData * ModelNode::NewModelData()
        {
            PERF_BLOCK( ModelNode__NewModelData ) ;

            ModelData * newModelData    = NEW ModelData() ;
            SetModelData( newModelData ) ;
            return GetModelData() ;
        }




        /** Update lights cache for this model, given a light.

            \param  light   Light to propose that this model use.

            This routine compares the given light to other lights in the cache (if any).
            If the cache has room then the light is added to the cache.  Otherwise,
            if the given light is closer than the farthest light in the cache then
            the farthest light is replaced with the given light.
        */
        void ModelNode::UpdateLightsCache( const Light * light )
        {
            PERF_BLOCK( ModelNode__UpdateLightsCache ) ;

            bool addedLight = false ;

            // If given light is closer than any cached light, insert given light into cache.
            const Vec3  sepCandidate    = light->GetPosition() - GetPosition() ;
            const float dist2Candidate  = sepCandidate.Mag2() ;
            for( unsigned iLight = 0 ; iLight < mNumLights ; ++ iLight )
            {   // For each light in cache...
                const Vec3  sep     = light->GetPosition() - GetPosition() ;
                const float dist2   = sep.Mag2() ;
                if( dist2Candidate < dist2 )
                {   // Candidate is closer than current cached light.
                    DEBUG_BREAK() ; // not tested
                    for( unsigned j = iLight + 1 ; j < mNumLights ; ++ j )
                    {   // For each subsequent light...
                        mLightsCache[ j ] = mLightsCache[ j - 1 ] ; // Shift it forward.
                    }
                    mLightsCache[ iLight ] = light ;
                    addedLight = true ;
                    break ;
                }
            }

            if( ! addedLight && ( mNumLights < MAX_NUM_LIGHTS_PER_MODEL ) )
            {   // Did not add given light and cache is not full.
                mLightsCache[ mNumLights ] = light ;    // Append given light to end of cache.
                ++ mNumLights ;
            }
        }




#if defined( _DEBUG )

        void PeGaSys_Render_ModelNode_UnitTest()
        {
            DebugPrintf( "ModelNode::UnitTest ----------------------------------------------\n" ) ;

            {
                ModelNode testModelNode( 0 ) ;

                // Update model
                testModelNode.Render() ;
            }

            DebugPrintf( "ModelNode::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
