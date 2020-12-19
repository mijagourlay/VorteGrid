/** \file D3D9_api.h

    \brief Wrapper for routines specific to D3D9 render system API

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_D3D9_API_H
#define PEGASYS_RENDER_D3D9_API_H

#include "Render/Device/viewport.h"
#include "Render/Device/api.h"

#include "Render/Platform/DirectX9/D3D9_RenderState.h"

// Macros ----------------------------------------------------------------------

#if defined( _DEBUG )
#   define RENDER_CHECK_ERROR( sitch ) ::PeGaSys::Render::D3D9_Api::CheckError( # sitch )
#   define HROK( hrExpr ) if( FAILED( hrExpr ) ) { FAIL() ; }
#else
#   define RENDER_CHECK_ERROR( sitch ) false
#   define HROK( hrExpr ) { ( hrExpr ) ; }
#endif

// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        // Forward declaration
        class Light ;

        /** Wrapper for routines specific to D3D9 render system API.
        */
        class D3D9_Api : public ApiBase
        {
        public:
            static const unsigned sType = 'RAd9' ; ///< Type identifier for this class

            D3D9_Api() ;
            virtual ~D3D9_Api() ;

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

            static bool CheckError( const char * /*situation*/ ) { return false ; }

        private:
            void    SetLight( unsigned idx , const Light & light ) ;

            D3D9_RenderStateCache   mRenderStateCache   ;   ///< Cache of render state.
        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;
} ;

#endif
