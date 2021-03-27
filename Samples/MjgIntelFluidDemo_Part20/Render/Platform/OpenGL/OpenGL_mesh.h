/** \file OpenGL_Mesh.h

    \brief Geometry mesh for OpenGL

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

        /** Geometry mesh for OpenGL.
        */
        class OpenGL_Mesh : public MeshBase
        {
            public:
                OpenGL_Mesh( ModelData * owningModelData ) ;
                virtual ~OpenGL_Mesh() ;

                virtual void Render() ;
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
