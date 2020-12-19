/** \file modelData.cpp

    \brief Sharable data for a model.

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Scene/modelData.h"

#include "Render/Device/api.h"

#include "Render/Resource/pass.h"
#include "Render/Resource/technique.h"
#include "Render/Resource/material.h"
#include "Render/Resource/mesh.h"

#include "Render/Scene/iSceneManager.h"

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

        /** Construct data used by a render model.
        */
        ModelData::ModelData()
        {
            PERF_BLOCK( ModelData__ModelData ) ;
        }




        /** Destruct data used by a render model.
        */
        ModelData::~ModelData()
        {
            PERF_BLOCK( ModelData__dtor ) ;

            while( ! mMeshes.Empty() )
            {   // For each mesh in this model...
                MeshBasePtr mesh = mMeshes.Front() ;    // Get first mesh in list.
                mesh.Reset() ;                          // Release object.
                mMeshes.PopFront() ;                    // Remove address from list.
            }
        }




        /** Render a model.
        */
        void ModelData::Render( ApiBase * renderApi )
        {
            PERF_BLOCK( ModelData__Render ) ;

            for( MeshIteratorT iter = mMeshes.Begin() ;
                iter != mMeshes.End() ;
                ++ iter )
            {   // For each mesh in this model...
                const MeshBasePtr & mesh = * iter ;
                // Set up render state for each pass.
                // TODO: FIXME: Improve support for multiple passes.
                //              Maybe use render queues: tuples of <pass,node> . Populate and manage queues.
                //              Iterate through the passes.
                //              Each pass could be associated with a particular target (or vice-versa).
                const Technique * technique = mesh->GetTechnique() ;
                ASSERT( technique ) ; // 
                if( technique )
                {   // Mesh has a render technique.
                    const Technique::PassContainer &    passes      = technique->GetPasses() ;
                    ASSERT( ! passes.Empty() ) ;
                    const Technique::PassConstIterator  endPass     = passes.End() ;
                    for( Technique::PassConstIterator iPass = passes.Begin() ; iPass != endPass ; ++ iPass )
                    {   // For each pass in this technique...
                        const Pass * pass = * iPass ;
                        pass->Apply( renderApi ) ;
                        mesh->Render() ;

                        //renderApi->RenderSimpleText( "ModelData" , Vec3( 0.0f , 0.0f, 0.0f ) , /* use screen space */ false ) ;

                    }
                }
            }
        }




        /** Add an existing geometry mesh to this model.
        */
        void ModelData::AddMesh( MeshBase * mesh )
        {
            PERF_BLOCK( ModelData__AddMesh ) ;

            mMeshes.PushBack( mesh ) ;
        }




        /** Create a new geometry mesh and add it to this model.
        */
        MeshBase * ModelData::NewMesh( ApiBase * renderApi )
        {
            PERF_BLOCK( ModelData__NewMesh ) ;

            MeshBase * mesh = renderApi->NewMesh( this ) ;
            AddMesh( mesh ) ;
            return mesh ;
        }




        /** Return addres of mesh at given index.
        */
        MeshBase *  ModelData::GetMesh( const size_t index )
        {
            PERF_BLOCK( ModelData__GetMesh ) ;

            ASSERT( index < mMeshes.Size() ) ;
            size_t count = 0 ;
            for( MeshIteratorT iter = mMeshes.Begin() ; iter != mMeshes.End() ; ++ iter , ++ count )
            {   // For each mesh in this model...
                if( index == count )
                {   // Found sought mesh.
                    MeshBasePtr mesh = * iter ;
                    return mesh.Get() ;
                }
            }
            return NULLPTR ;
        }




        /** Return number of meshes in this model.
        */
        size_t ModelData::GetNumMeshes() const
        {
            PERF_BLOCK( ModelData__GetNumMeshes ) ;

            return mMeshes.Size() ;
        }




        void ModelData::ReleaseReference()
        {
            PERF_BLOCK( ModelData__ReleaseReference ) ;

            if( _ReleaseReference() )
            {
                delete this ;
            }
        }




#if defined( _DEBUG )

        void PeGaSys_Render_ModelData_UnitTest( void )
        {
            DebugPrintf( "ModelData::UnitTest ----------------------------------------------\n" ) ;

            {
                ModelData testModelData ;

                // Update model
                testModelData.Render( NULLPTR ) ;
            }

            DebugPrintf( "ModelData::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#endif

    } ;
} ;
