/** \file D3D9_Mesh.h

    \brief Geometry mesh for Direct3D version 9

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_MESH_H
#define PEGASYS_RENDER_OPENGL_MESH_H

#include "Render/Resource/mesh.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class ModelData ;

        /** Geometry mesh for Direct3D version 9.
        */
        class D3D9_Mesh : public MeshBase
        {
            public:
                D3D9_Mesh( ModelData * owningModelData ) ;
                virtual ~D3D9_Mesh() ;

                virtual void Render() ;

            private:
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
