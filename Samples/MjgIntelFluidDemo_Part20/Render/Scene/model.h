/** \file model.h

    \brief Scene node representing a model

    \author Copyright 2010-2014 MJG; All rights reserved.

*/
#ifndef PEGASYS_RENDER_MODEL_H
#define PEGASYS_RENDER_MODEL_H

#include "Core/Containers/IntrusivePtr.h"
#include "Render/Scene/sceneNodeBase.h"
#include "Render/Scene/modelData.h"

// Macros ----------------------------------------------------------------------

#ifndef PRIVATE
#   define PRIVATE private
#endif

// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        // Forward declarations
        class MeshBase  ;
        class Light     ;

        /** Scene node representing a model.

            A model has ModelData which is essentially a collection of meshes,
            where each mesh has a material and geometry (e.g. vertices, triangles).
        */
        class ModelNode : public SceneNodeBase
        {
            public:
                static const unsigned sTypeId = 'MdlN' ; ///< Type identifier for a testModelNode scene node
                static const size_t MAX_NUM_LIGHTS_PER_MODEL = 8 ;

                explicit ModelNode( ISceneManager * sceneManager ) ;
                virtual ~ModelNode() ;

                virtual void Render() ;

                ModelData *         NewModelData() ;

                void                SetModelData( ModelData * modelData ) ;
                ModelData *         GetModelData()       { return mModelData.Get() ; }
                const ModelData *   GetModelData() const { return mModelData.Get() ; }

                /// Clear all lights cached for this model.
                void                ClearLightsCache() { mNumLights = 0 ; }

                void                UpdateLightsCache( const Light * light ) ;

                /// Return number of lights cached for this model.
                const unsigned &    GetNumLights() const { return mNumLights ; }

                /** Return address of cached light at given index.
                    \param idx  Index into light cache for this model
                */
                const Light *       GetLight( unsigned idx ) const
                {
                    ASSERT( idx < mNumLights ) ;
                    return mLightsCache[ idx ] ;
                }

            PRIVATE:
                typedef IntrusivePtr< ModelData > ModelDataPtr ;    ///< Smart pointer to reference-counted ModelData object.

                ModelDataPtr    mModelData                               ;  ///< Sharable model data such as meshes.
                const Light *   mLightsCache[ MAX_NUM_LIGHTS_PER_MODEL ] ;  ///< Cache of lights that apply to this model
                unsigned        mNumLights                               ;  ///< Number of lights in mLightsCache.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
