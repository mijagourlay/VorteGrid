/** \file modelData.h

    \brief Sharable data for a model.

    \author Written and Copyright 2010-2014 MJG; All rights reserved.

*/
#ifndef PEGASYS_RENDER_MODEL_DATA_H
#define PEGASYS_RENDER_MODEL_DATA_H

#include <Core/Containers/slist.h>
#include <Core/Containers/intrusivePtr.h>

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
        // Forward declarations.
        class ApiBase  ;
        class MeshBase ;

        /** Sharable data for a model.

            The point of having this be separate from ModelNode is that a "model"
            (a collection of Meshes with its associated materials) can occur in
            a scene multiple times, but the system should share (not duplicate)
            those resources.  In principle, the Mesh and Material objects could
            be shared (that is, sharing could occur at a lower level than ModelData)
            but sharing at the ModelData level provides additional convenience.
        */
        class ModelData : public RefCountedMixin
        {
            public:
                typedef IntrusivePtr< MeshBase >    MeshBasePtr     ;
                typedef SLIST< MeshBasePtr >        MeshContainerT  ;
                typedef MeshContainerT::Iterator    MeshIteratorT   ;

                ModelData() ;
                ~ModelData() ;

                MeshBase *  NewMesh( ApiBase * renderApi ) ;
                size_t      GetNumMeshes() const ;
                MeshBase *  GetMesh( size_t index ) ;
                void        Render( ApiBase * renderApi ) ;

                void ReleaseReference() ; // Used by IntrusivePtr (indirectly, through global ReleaseReference)

            PRIVATE:
                void        ReserveMeshes( size_t numMeshes ) ;
                void        AddMesh( MeshBase * mesh ) ;

                //ApiBase     *   mRenderApi  ;   ///< Render API object used to create resources used by this object.
                MeshContainerT  mMeshes     ;   ///< Geometry meshes for this model.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

        /// Add reference to a ModelData, for use by IntrusivePtr.
        inline void AddReference( ModelData * modelData )     { modelData->AddReference() ; }

        /// Release reference to a ModelData, for use by IntrusivePtr.
        inline void ReleaseReference( ModelData * modelData ) { modelData->ReleaseReference() ; }

    } ;
} ;

#endif
