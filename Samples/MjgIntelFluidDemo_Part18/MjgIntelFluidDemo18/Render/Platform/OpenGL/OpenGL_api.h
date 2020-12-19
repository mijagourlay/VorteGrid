/** \file OpenGL_api.h

    \brief Wrapper for routines specific to OpenGL render system API

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_API_H
#define PEGASYS_RENDER_OPENGL_API_H

#include "Render/Device/viewport.h"
#include "Render/Device/api.h"

#include "Render/Platform/OpenGL/OpenGL_RenderState.h"

// Macros ----------------------------------------------------------------------

#if defined( _DEBUG )
#   define RENDER_CHECK_ERROR( sitch ) ::PeGaSys::Render::OpenGL_Api::CheckError( # sitch )
#else
#   define RENDER_CHECK_ERROR( sitch ) false
#endif

// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        // Forward declaration
        class Light ;
        struct RenderStateS ;

        /** Wrapper for routines specific to OpenGL render system API.
        */
        class OpenGL_Api : public ApiBase
        {
        public:
            static const unsigned sType = 'RAog' ; ///< Type identifier for this class

            OpenGL_Api() ;
            virtual ~OpenGL_Api() ;

            virtual Window *    NewWindow( class Render::System * renderSystem ) ;
            virtual void        SetViewport( const Viewport & viewport ) ;
            virtual void        SetCamera( const Camera & camera ) ;
            virtual void        SetLights( const ModelNode & lightReceiver ) ;
            virtual void        SetLocalToWorld( const Mat44 & localToWorld ) ;

            virtual void        RenderSimpleText( const char * text , const Vec3 & position , bool useScreenSpace ) ;

            virtual VertexBufferBase *  NewVertexBuffer() ;
            virtual IndexBufferBase  *  NewIndexBuffer() ;
            virtual void                DeleteIndexBuffer( IndexBufferBase * indexBuffer ) ;
            virtual MeshBase *          NewMesh( ModelData * owningModelData ) ;
            virtual TextureBase *       NewTexture() ;

            virtual void ApplyRenderState( const RenderStateS & renderState ) ;
            virtual void GetRenderState( RenderStateS & renderState ) ;
            virtual void DisableTexturing() ;

            static void GetProcAddresses() ;
            static bool CheckError( const char * situation ) ;

        private:
            void    SetLight( unsigned idx , const Light & light ) ;

            OpenGL_RenderStateCache mRenderStateCache   ;   ///< Cache of render state.
        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------


    } ;
} ;

#endif
