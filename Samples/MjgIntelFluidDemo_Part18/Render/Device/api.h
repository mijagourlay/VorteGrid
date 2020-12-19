/** \file api.h

    \brief Wrapper for routines specific to a render system API

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_API_H
#define PEGASYS_RENDER_API_H

#include "Core/Math/vec2.h"
#include "Core/Math/mat4.h"
#include "Render/Device/window.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        // Forward declarations
        class System            ;
        class VertexBufferBase  ;
        class IndexBufferBase   ;
        class MeshBase          ;
        class TextureBase       ;
        class ModelNode         ;
        class ModelData         ;
        struct RenderStateS     ;

        /** Base class render system API.
        */
        class ApiBase
        {
        public:
            static const unsigned sType = 'RAPI' ; ///< Type identifier for this class

            enum Parameters
            {
                sName           ,   ///< Name of API
                xView           ,   ///< View matrix
                NUM_PARAMETERS
            } ;

            virtual ~ApiBase() {}

            virtual Window *    NewWindow( System * renderSystem ) = 0 ;
            virtual void        SetViewport( const Viewport & viewport ) = 0 ;
            virtual void        SetCamera( const Camera & xView ) = 0 ;
            virtual void        SetLights( const ModelNode & lightReceiver ) = 0 ;
            virtual void        SetLocalToWorld( const Mat44 & localToWorld ) = 0 ;

            virtual void        RenderSimpleText( const char * text , const Vec3 & position , bool useScreenSpace ) = 0 ;

            virtual void        ApplyRenderState( const RenderStateS & renderState ) = 0 ;
            virtual void        GetRenderState( RenderStateS & renderState ) = 0 ;
            virtual void        DisableTexturing() = 0 ;

            virtual VertexBufferBase *  NewVertexBuffer() = 0 ;
            virtual IndexBufferBase *   NewIndexBuffer() = 0 ;
            virtual void                DeleteIndexBuffer( IndexBufferBase * indexBuffer ) = 0 ;
            virtual MeshBase *          NewMesh( ModelData * owningModelData ) = 0 ;
            virtual TextureBase *       NewTexture() = 0 ;
        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;
} ;

#endif
