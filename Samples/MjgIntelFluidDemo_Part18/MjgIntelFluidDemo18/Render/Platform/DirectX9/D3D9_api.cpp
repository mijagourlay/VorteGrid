/** \file D3D9_api.cpp

    \brief Wrapper for routines specific to D3D9 render system API

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#include <stdio.h>

#include <windows.h>

#include <d3d9.h>



#if defined( _MSC_VER ) && ( _MSC_VER > 1310 ) // Visual Studio 2003 (.NET) cannot use WinSDK, but later versions can and should.
// If Visual Studio cannot find DirectXMath.h, install WinSDK, then edit the property sheet named Microsoft.Cpp.Win32.user.
// Inside the property page for each build target, to Include Directories, add $(ProgramFiles)\Windows Kits\{version}\Include\um.
// Note: Replace {version} with the version of the WinSDK installed on your machine.  That should be the same as your Windows version,
// e.g. 7.0, 8.0 or 8.1.
#   include <DirectXMath.h> // Defines XMVECTOR, XMMATRIX.  Comes with WinSDK.  Resides in, e.g., C:\Program Files (x86)\Windows Kits\8.1\Include\um
#endif

// Wrapper macros to help migrate from D3DX Math to DirectXMath.
#ifdef DIRECTX_MATH_VERSION // New, from WinSDK

#   pragma warning( disable: 4324 ) // structure was padded due to __declspec(align())

    __declspec(align(16)) struct D3DXVECTOR3 : public DirectX::XMFLOAT3
    {
        D3DXVECTOR3( float x, float y, float z) : XMFLOAT3(x, y, z) {}
    } ;

    __declspec(align(16)) struct D3DXVECTOR4 : public DirectX::XMFLOAT4
    {
        D3DXVECTOR4( float x, float y, float z, float w) : XMFLOAT4(x, y, z, w ) {}
    } ;

    typedef DirectX::XMMATRIX D3DXMATRIX ;

#   define D3DXToRadian( deg )                         DirectX::XMConvertToRadians( deg )

#   define D3DXMatrixLookAtRH(m,e,f,u)                 (*(m)) = DirectX::XMMatrixLookAtRH(* reinterpret_cast<DirectX::XMVECTOR *>(e) , * reinterpret_cast<DirectX::XMVECTOR *>(f) , * reinterpret_cast<DirectX::XMVECTOR *>(u) )
#   define D3DXMatrixPerspectiveFovRH(m,fy,ar,nz,fz)   (*(m)) = DirectX::XMMatrixPerspectiveFovRH( (fy) , (ar) , (nz) , (fz) )
#   define D3DXMatrixIdentity(m)                       (*(m)) = DirectX::XMMatrixIdentity()
#   define D3DXMatrixTranslation(m,x,y,z)              (*(m)) = DirectX::XMMatrixTranslation((x),(y),(z))
#   define D3DXMatrixScaling(m,x,y,z)                  (*(m)) = DirectX::XMMatrixScaling((x),(y),(z))
#   define D3DXMatrixRotationX(m,angle)                (*(m)) = DirectX::XMMatrixRotationX(angle)
#   define D3DXMatrixRotationY(m,angle)                (*(m)) = DirectX::XMMatrixRotationY(angle)
#   define D3DXMatrixRotationZ(m,angle)                (*(m)) = DirectX::XMMatrixRotationZ(angle)
#   define D3DXMatrixRotationYawPitchRoll(m,y,p,r)     (*(m)) = DirectX::XMMatrixRotationRollPitchYaw(r,p,y)
#   define D3DXMatrixRotationAxis(m,axis,angle)        (*(m)) = DirectX::XMMatrixRotationAxis( * reinterpret_cast< DirectX::XMVECTOR * >( axis ) , angle )
#   define D3DXMatrixMultiply(m,a,b)                   (*(m)) = DirectX::XMMatrixMultiply( * reinterpret_cast< const DirectX::XMMATRIX * >( a ) , * reinterpret_cast< const DirectX::XMMATRIX * >( b ) )
#   define D3DXMatrixTranspose(mOut,mIn)               (*(mOut)) = DirectX::XMMatrixTranspose( * reinterpret_cast< const DirectX::XMMATRIX * >( mIn ) )
#   define D3DXVec4Transform(vOut,vIn,m)               (*reinterpret_cast< DirectX::XMVECTOR * >(vOut)) = DirectX::XMVector4Transform( * reinterpret_cast< const DirectX::XMVECTOR * >( vIn ) , * reinterpret_cast< const DirectX::XMMATRIX * >( m ) )
#   define D3DXVec3Transform(vOut,vIn,m)               (*reinterpret_cast< DirectX::XMVECTOR * >(vOut)) = DirectX::XMVector3Transform( * reinterpret_cast< const DirectX::XMVECTOR * >( vIn ) , * reinterpret_cast< const DirectX::XMMATRIX * >( m ) )

#else // Old, from DirectX SDK
#   include <d3dx9.h>  // D3D Extensions: Utility routines, for D3DXMATRIX, D3DXVECTOR

#   pragma comment(lib, "d3dx9.lib")
#endif




#include "Render/Scene/model.h"
#include "Render/Scene/light.h"
#include "Render/Platform/DirectX9/D3D9_window.h"
#include "Render/Platform/DirectX9/D3D9_api.h"
#include "Render/Platform/DirectX9/D3D9_vertexBuffer.h"
#include "Render/Platform/DirectX9/D3D9_indexBuffer.h"
#include "Render/Platform/DirectX9/D3D9_mesh.h"
#include "Render/Platform/DirectX9/D3D9_texture.h"

extern LPDIRECT3DDEVICE9   g_pd3dDevice ; // Direct3D rendering device

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct wrapper for routines specific to D3D9 render system API.
        */
        D3D9_Api::D3D9_Api()
        {
        }




        /** Destruct wrapper for routines specific to D3D9 render system API.
        */
        D3D9_Api::~D3D9_Api()
        {
        }




        /* virtual */ Window * D3D9_Api::NewWindow( Render::System * renderSystem )
        {
            Render::Window * window = NEW Render::D3D9_Window( renderSystem ) ;
            return window ;
        }




        static void SetRenderTarget( const Viewport & /*viewport*/ )
        {
            // TODO: FIXME: Set render target.

#if 0 // Placeholder code to show the general idea and API calls needed.  Obviously this could not stay this way because it leaks resources.
            LPDIRECT3DTEXTURE9 pRenderTexture = NULL ; // texture to render into
            LPDIRECT3DSURFACE9 pRenderSurface = NULL ; // for new target
            LPDIRECT3DSURFACE9 pBackBuffer    = NULL ; // for old target, to restore later

            D3DXMATRIX matOldProjection; // for old projection, to restore later.

            // Create texture.
            HROK( g_pd3dDevice->CreateTexture(
                viewport.GetTarget()->GetWidth()  // x resolution
                , viewport.GetTarget()->GetHeight() // y resolution
                , 1     // number of mip levels
                , D3DUSAGE_RENDERTARGET
                , D3DFMT_R5G6B5     // texture format
                , D3DPOOL_DEFAULT   // memory pool
                , & pRenderTexture
                , NULL ) ) ;

            // Surface to access texture memory
            pRenderTexture->GetSurfaceLevel( 0 , & pRenderSurface ) ;

            // Save copy of original projection matrix.
            HROK( g_pd3dDevice->GetTransform( D3DTS_PROJECTION , & matOldProjection ) ) ;

            // Save pointer to original back buffer.
            HROK( g_pd3dDevice->GetRenderTarget( 0 , & pBackBuffer ) ) ;

            // Set render target
            HROK( g_pd3dDevice->SetRenderTarget( 0 , pRenderSurface ) ) ;

            //clear texture
            HROK( g_pd3dDevice->Clear(
                0
                , NULL
                , D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER
                , D3DCOLOR_XRGB(100,100,100)
                , 1.0f
                , 0 ) ) ;

#endif
        }




        /* virtual */ void D3D9_Api::SetViewport( const Viewport & viewport )
        {
            SetRenderTarget( viewport ) ;

            // TODO: FIXME: GetTarget should return values commensurate with the actual render target.
            IDirect3DSurface9 * renderTarget ;
            HROK( g_pd3dDevice->GetRenderTarget( 0 /* render target index */ , & renderTarget ) ) ;

            D3DSURFACE_DESC surfaceDesc ;
            HROK( renderTarget->GetDesc( & surfaceDesc ) ) ;

            D3DVIEWPORT9 view_port = { 0 , 0 , 1 , 1 , 0.0f , 1.0f } ;
            view_port.X      = int( viewport.GetRelLeft()   * float( viewport.GetTarget()->GetWidth()  ) ) ;
            view_port.Y      = int( viewport.GetRelTop()    * float( viewport.GetTarget()->GetHeight() ) ) ;
            //view_port.Width  = int( viewport.GetRelWidth()  * float( viewport.GetTarget()->GetWidth()  ) ) ;
            //view_port.Height = int( viewport.GetRelHeight() * float( viewport.GetTarget()->GetHeight() ) ) ;
            // TODO: FIXME: Use target dimensions instead of this.  See comment above about GetTarget.
            view_port.Width  = int( viewport.GetRelWidth()  * float( surfaceDesc.Width ) ) ;
            view_port.Height = int( viewport.GetRelHeight() * float( surfaceDesc.Height ) ) ;
            HROK( g_pd3dDevice->SetViewport( & view_port ) ) ;

            {
                unsigned int iClearColor ; // clear color, ARGB format
                const unsigned int iRed   = unsigned( viewport.GetClearColor().x * 255.0f ) ;
                const unsigned int iGreen = unsigned( viewport.GetClearColor().y * 255.0f ) ;
                const unsigned int iBlue  = unsigned( viewport.GetClearColor().z * 255.0f ) ;
                const unsigned int iAlpha = unsigned( viewport.GetClearColor().w * 255.0f ) ;
                ASSERT( iAlpha != 0 ) ; // Clear would have no effect if transparent.
                iClearColor = D3DCOLOR_RGBA( iRed , iGreen , iBlue , iAlpha ) ;
#ifdef Clear // Wrapper macros defined Clear
#   undef Clear
#endif
                HRESULT hr = g_pd3dDevice->Clear( 0  , NULL , D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER /* | D3DCLEAR_STENCIL */ , iClearColor , 1.0f , 0 ) ;
                ASSERT( SUCCEEDED( hr ) ) ; (void) hr ;
            }
        }




        /* virtual */ void D3D9_Api::SetCamera( const Camera & camera )
        {
            {   // Set view transformation
                D3DXVECTOR3 eye   (  camera.GetEye().x          , camera.GetEye().y             , camera.GetEye().z             ) ;
                D3DXVECTOR3 lookat(  camera.GetLookAt().x       , camera.GetLookAt().y          , camera.GetLookAt().z          ) ;
                D3DXVECTOR3 up    (  camera.GetUpApproximate().x, camera.GetUpApproximate().y   , camera.GetUpApproximate().z   ) ;
                Mat44 & viewMatrix = mRenderStateCache.mCurrentState.mTransforms.mViewMatrix ;
                D3DXMATRIX  & matView = reinterpret_cast< D3DXMATRIX & >( viewMatrix ) ;
                D3DXMatrixLookAtRH( & matView , & eye , & lookat , & up ) ;
                HROK( g_pd3dDevice->SetTransform( D3DTS_VIEW , reinterpret_cast< D3DMATRIX * >( & matView ) ) ) ;

                // TODO: Instead of using D3D utils to set view transform, change code above to compose view matrix (as per below) and use SetTransform to set it.  That way, D3D and OGL would use identical matrices and would avoid querying render API for data.

#               if defined( _DEBUG )
                {   // TODO: Move this into a unit test.
                    Mat44 xViewFromApi ;
                    HROK( g_pd3dDevice->GetTransform( D3DTS_VIEW , reinterpret_cast< D3DMATRIX * >( & xViewFromApi ) ) ) ;

                    //Mat44_DebugPrint( xViewFromApi ) ;
                    ASSERT( xViewFromApi.GetRight().IsNormalized() ) ;
                    ASSERT( xViewFromApi.GetForward().IsNormalized() ) ;
                    ASSERT( xViewFromApi.GetUp().IsNormalized() ) ;
                    {
                        const Mat44 xViewTranspose = xViewFromApi.GetTranspose() ;
                        ASSERT( xViewTranspose.GetRight().IsNormalized() ) ;
                        ASSERT( xViewTranspose.GetForward().IsNormalized() ) ;
                        ASSERT( xViewTranspose.GetUp().IsNormalized() ) ;
                    }
                    Vec3 vPos = Mat44::CameraPositionFromViewMatrix( xViewFromApi ) ;
                    ASSERT( vPos.Resembles( camera.GetEye() ) ) ;

                    Mat44  pegasysMatrixView ;
                    pegasysMatrixView.SetView( camera.GetEye4() , camera.GetLookAt() , camera.GetUpApproximate() ) ;
                    ASSERT( pegasysMatrixView.Resembles( xViewFromApi ) ) ;
                }
#               endif
            }


            {   // Set projection transformation
                D3DXMATRIX matProj;
                D3DXMatrixPerspectiveFovRH( & matProj
                    , camera.GetFieldOfViewVert() * DEG2RAD
                    , camera.GetAspectRatio()
                    , camera.GetNearClipDist()
                    , camera.GetFarClipDist()     ) ;
                HROK( g_pd3dDevice->SetTransform( D3DTS_PROJECTION, reinterpret_cast< D3DMATRIX * >( & matProj ) ) ) ;
            }
        }




        void D3D9_Api::SetLight( unsigned idx , const Light & light )
        {
            D3DLIGHT9 d3dLight ;
            ZeroMemory( & d3dLight , sizeof( d3dLight ) ) ;

            d3dLight.Type         = static_cast< D3DLIGHTTYPE >( light.GetLightType() ) ;

            d3dLight.Diffuse.r    = light.GetDiffuseColor().x ;
            d3dLight.Diffuse.g    = light.GetDiffuseColor().y ;
            d3dLight.Diffuse.b    = light.GetDiffuseColor().z ;
            d3dLight.Diffuse.a    = light.GetDiffuseColor().w ;

            d3dLight.Specular.r   = light.GetSpecularColor().x ;
            d3dLight.Specular.g   = light.GetSpecularColor().y ;
            d3dLight.Specular.b   = light.GetSpecularColor().z ;
            d3dLight.Specular.a   = light.GetSpecularColor().w ;

            d3dLight.Ambient.r    = light.GetAmbientColor().x ;
            d3dLight.Ambient.g    = light.GetAmbientColor().y ;
            d3dLight.Ambient.b    = light.GetAmbientColor().z ;
            d3dLight.Ambient.a    = light.GetAmbientColor().w ;

            // TODO: FIXME: Position should be relative to parent transform.
            d3dLight.Position.x   = light.GetPosition().x ;
            d3dLight.Position.y   = light.GetPosition().y ;
            d3dLight.Position.z   = light.GetPosition().z ;

            // TODO: FIXME: Orientation should be relative to parent transform.
            // TODO: FIXME: SceneNode orientation should determine light direction (or vice-versa).
            d3dLight.Direction.x  = light.GetDirection().x ;
            d3dLight.Direction.y  = light.GetDirection().y ;
            d3dLight.Direction.z  = light.GetDirection().z ;
            //D3DXVec3Normalize( (D3DXVECTOR3*) & d3dLight.Direction , (D3DXVECTOR3*) & d3dLight.Direction ) ;

            d3dLight.Range        = light.GetRange() ;
            d3dLight.Falloff      = light.GetSpotFalloff() ;
            d3dLight.Attenuation0 = light.GetConstAttenuation() ;
            d3dLight.Attenuation1 = light.GetLinearAttenuation() ;
            d3dLight.Attenuation2 = light.GetQuadracticAttenuation() ;
            d3dLight.Theta        = D3DXToRadian( light.GetSpotInnerAngle() ) ;
            d3dLight.Phi          = D3DXToRadian( light.GetSpotOuterAngle() ) ;

            HROK( g_pd3dDevice->SetLight( idx , & d3dLight ) ) ;
            HROK( g_pd3dDevice->LightEnable( idx , TRUE ) ) ;

#           if defined( _DEBUG )
            {
                D3DLIGHT9 lite ;
                HRESULT hr = g_pd3dDevice->GetLight( idx , & lite ) ;
                hr = hr ;
            }
#           endif

        }




        /** Set the render state to use the light cached for the given receiver.

            \note   If the model does not have normals, setting lights will cause that
                    model to render as black.            
        */
        /* virtual */ void D3D9_Api::SetLights( const ModelNode & lightReceiver )
        {
            const unsigned numLights = lightReceiver.GetNumLights() ;
            if( numLights > 0 )
            {   // Scene has at least one light.
                HROK( g_pd3dDevice->SetRenderState( D3DRS_LIGHTING , TRUE ) ) ;

                // Set lighting render state.
                //g_pd3dDevice->SetRenderState( D3DRS_SPECULARENABLE, TRUE );
                HROK( g_pd3dDevice->SetRenderState( D3DRS_LOCALVIEWER , FALSE ) ) ;
                HROK( g_pd3dDevice->SetRenderState( D3DRS_AMBIENT, D3DCOLOR_XRGB(50, 50, 50)) ) ;
                HROK( g_pd3dDevice->SetRenderState( D3DRS_NORMALIZENORMALS , TRUE ) ) ;

                for( unsigned idx = 0 ; idx < numLights ; ++ idx )
                {   // For each light in the given model's cache...
                    const Light * light = lightReceiver.GetLight( idx ) ;
                    SetLight( idx , * light ) ;
                }
                for( unsigned idx = numLights ; idx < ModelNode::MAX_NUM_LIGHTS_PER_MODEL ; ++ idx )
                {
                    HROK( g_pd3dDevice->LightEnable( idx , FALSE ) ) ;
                }
            }
            else
            {   // Model has no lights so disable lighting.
                HROK( g_pd3dDevice->SetRenderState( D3DRS_LIGHTING , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 0 , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 1 , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 2 , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 3 , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 4 , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 5 , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 6 , FALSE ) ) ;
                HROK( g_pd3dDevice->LightEnable( 7 , FALSE ) ) ;
            }
        }




        /* virtual */ void D3D9_Api::SetLocalToWorld( const Mat44 & localToWorld )
        {
            HROK( g_pd3dDevice->SetTransform( D3DTS_WORLD , reinterpret_cast< const D3DMATRIX *>( & localToWorld ) ) ) ; // apply transform for this object
        }




        /** Render simple text for on-screen diagnostic messages.
        */
        /* virtual */ void  D3D9_Api::RenderSimpleText( const char * /*text*/ , const Vec3 & /*position*/ , bool /*useScreenSpace*/ )
        {
            // TODO: implement me
        }




        /* virtual */ VertexBufferBase *  D3D9_Api::NewVertexBuffer()
        {
            VertexBufferBase * vertexBuffer = NEW D3D9_VertexBuffer ;
            return vertexBuffer ;
        }




        /* virtual */ IndexBufferBase *  D3D9_Api::NewIndexBuffer()
        {
            IndexBufferBase * indexBuffer = NEW D3D9_IndexBuffer ;
            return indexBuffer ;
        }




        /* virtual */ void D3D9_Api::DeleteIndexBuffer( IndexBufferBase * indexBuffer )
        {
            delete indexBuffer ;
        }




        /* virtual */ MeshBase *  D3D9_Api::NewMesh( ModelData * owningModelData )
        {
            MeshBase * mesh = NEW D3D9_Mesh( owningModelData )  ;
            return mesh ;
        }




        /* virtual */ TextureBase *  D3D9_Api::NewTexture()
        {
            TextureBase * texture = NEW D3D9_Texture ;
            return texture ;
        }




        /* virtual */ void D3D9_Api::ApplyRenderState( const RenderStateS & renderState )
        {
            mRenderStateCache.Apply( renderState ) ;
        }




        /* virtual */ void D3D9_Api::GetRenderState( RenderStateS & renderState )
        {
            D3D9_RenderStateCache::GetRenderState( renderState ) ;
        }




        /* virtual */ void D3D9_Api::DisableTexturing()
        {
            HROK( g_pd3dDevice->SetTextureStageState( 0 , D3DTSS_COLOROP  , D3DTOP_DISABLE ) ) ;
        }




#if defined( _DEBUG )

#       include "Core/Math/Mat33.h"

        static void D3DxMatrix_DebugPrint( const D3DXMATRIX & d3dxMatrix )
        {
            Mat44_DebugPrint( ( Mat44 & ) d3dxMatrix ) ;
        }




        static bool Vector3sMatch( const D3DXVECTOR4 & d3dxVector4 , const Vec3 & pegasysVec3 , float tolerance = 1.0e-6f )
        {
            return pegasysVec3.Resembles( ( Vec3 & ) d3dxVector4 , tolerance ) ;
        }




        static bool Vector4sMatch( const D3DXVECTOR4 & d3dxVector4, const Vec4 & pegasysVec4 , float tolerance = 1.0e-6f )
        {
            return pegasysVec4.Resembles( ( Vec4 & ) d3dxVector4 , tolerance ) ;
        }




        static bool MatricesMatch( const D3DXMATRIX & d3dxMatrix , const Mat44 & pegasysMatrix , float tolerance = 1.0e-4f )
        {
            return pegasysMatrix.Resembles( ( const Mat44 & ) d3dxMatrix , tolerance ) ;
        }




        static bool MatricesMatch( const D3DXMATRIX & d3dxMatrix , const Mat33 & pegasysMatrix , float tolerance = 1.0e-4f )
        {
#       ifdef DIRECTX_MATH_VERSION
            using namespace DirectX ;
            Mat33 d3dMatrixUpper3x3(  Vec3( XMVectorGetX( d3dxMatrix.r[0] ) , XMVectorGetY( d3dxMatrix.r[0] ) , XMVectorGetZ( d3dxMatrix.r[0] ) )
                , Vec3( XMVectorGetX( d3dxMatrix.r[1] ) , XMVectorGetY( d3dxMatrix.r[1] ) , XMVectorGetZ( d3dxMatrix.r[1] ) )
                , Vec3( XMVectorGetX( d3dxMatrix.r[2] ) , XMVectorGetY( d3dxMatrix.r[2] ) , XMVectorGetZ( d3dxMatrix.r[2] ) )
                ) ;
#       else
            Mat33 d3dMatrixUpper3x3(  Vec3( d3dxMatrix.m[0][0] , d3dxMatrix.m[0][1] , d3dxMatrix.m[0][2] )
                , Vec3( d3dxMatrix.m[1][0] , d3dxMatrix.m[1][1] , d3dxMatrix.m[1][2] )
                , Vec3( d3dxMatrix.m[2][0] , d3dxMatrix.m[2][1] , d3dxMatrix.m[2][2] ) ) ;
#       endif
            return pegasysMatrix.Resembles( d3dMatrixUpper3x3 , tolerance ) ;
        }




        static void VectorMatrix_UnitTest()
        {
            DebugPrintf( "VectorMatrix_UnitTest::UnitTest ----------------------------------------------\n" ) ;

            {
                const D3DXMATRIX    d3dMatrix(      1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                D3DxMatrix_DebugPrint( d3dMatrix ) ;

                const Mat44  pegasysMatrix(      1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                //Mat44_DebugPrint( pegasysMatrix ) ;
                ASSERT( MatricesMatch( d3dMatrix , pegasysMatrix ) ) ;
            }

            // Verify that Direct3D matrix routines have same meaning as analogous routines in Mat44.

            // Test matrix assignment routines.
            {
                static const int    numAngles   = 10001 ;
                static const float  TWO_PI      = 6.283185307179586476925286766559f ;
                static const float  scale       = 2.0f * TWO_PI / float( numAngles ) ;
                for( int idx = 0 ; idx < numAngles ; ++ idx )
                {
                    const float angle = float( idx ) * scale - TWO_PI ;
                    {
                        D3DXMATRIX    d3dMatrixRotateX ;
                        D3DXMatrixRotationX( & d3dMatrixRotateX , angle ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixRotateX ) ;

                        {
                            Mat44  pegasysMatrixRotateX ;
                            pegasysMatrixRotateX.SetRotationX( angle ) ;
                            //Mat44_DebugPrint( pegasysMatrixRotateX ) ;
                            ASSERT( MatricesMatch( d3dMatrixRotateX , pegasysMatrixRotateX ) ) ;
                        }

                        {
                            Mat33 pegasysMatrix33RotateX ;
                            pegasysMatrix33RotateX.SetRotationX( angle ) ;
                            ASSERT( MatricesMatch( d3dMatrixRotateX , pegasysMatrix33RotateX ) ) ;
                        }
                    }
                    {
                        D3DXMATRIX    d3dMatrixRotateY ;
                        D3DXMatrixRotationY( & d3dMatrixRotateY , angle ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixRotateY ) ;

                        {
                            Mat44  pegasysMatrixRotateY ;
                            pegasysMatrixRotateY.SetRotationY( angle ) ;
                            //Mat44_DebugPrint( pegasysMatrixRotateY ) ;
                            ASSERT( MatricesMatch( d3dMatrixRotateY , pegasysMatrixRotateY ) ) ;
                        }

                        {
                            Mat33 pegasysMatrix33RotateY ;
                            pegasysMatrix33RotateY.SetRotationY( angle ) ;
                            ASSERT( MatricesMatch( d3dMatrixRotateY , pegasysMatrix33RotateY ) ) ;
                        }
                    }
                    {
                        D3DXMATRIX    d3dMatrixRotateZ ;
                        D3DXMatrixRotationZ( & d3dMatrixRotateZ , angle ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixRotateZ ) ;

                        {
                            Mat44  pegasysMatrixRotateZ ;
                            pegasysMatrixRotateZ.SetRotationZ( angle ) ;
                            //Mat44_DebugPrint( pegasysMatrixRotateZ ) ;
                            ASSERT( MatricesMatch( d3dMatrixRotateZ , pegasysMatrixRotateZ ) ) ;
                        }
                        {
                            Mat33 pegasysMatrix33RotateZ ;
                            pegasysMatrix33RotateZ.SetRotationZ( angle ) ;
                            ASSERT( MatricesMatch( d3dMatrixRotateZ , pegasysMatrix33RotateZ ) ) ;
                        }
                    }

                    // TODO: FIXME: Change these tests and associated code to work compatibly with WinSDK.
#                   ifndef DIRECTX_MATH_VERSION // DirectXMath has RollPitchYaw, which is backwards from XYZ.
                    {
                        D3DXMATRIX    d3dMatrixRotateXYZ ;
                        D3DXMatrixRotationYawPitchRoll( & d3dMatrixRotateXYZ , 0.0f , angle , 0.0f ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixRotateXYZ ) ;

                        Mat44  pegasysMatrixRotateXYZ ;
                        pegasysMatrixRotateXYZ.SetRotationXYZ( Vec3( angle , 0.0f , 0.0f ) ) ;
                        //Mat44_DebugPrint( pegasysMatrixRotateXYZ ) ;
                        ASSERT( MatricesMatch( d3dMatrixRotateXYZ , pegasysMatrixRotateXYZ ) ) ;
                    }
                    {
                        D3DXMATRIX    d3dMatrixRotateXYZ ;
                        D3DXMatrixRotationYawPitchRoll( & d3dMatrixRotateXYZ , angle , 0.0f , 0.0f ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixRotateXYZ ) ;

                        Mat44  pegasysMatrixRotateXYZ ;
                        pegasysMatrixRotateXYZ.SetRotationXYZ( Vec3( 0.0f , angle , 0.0f ) ) ;
                        //Mat44_DebugPrint( pegasysMatrixRotateXYZ ) ;
                        ASSERT( MatricesMatch( d3dMatrixRotateXYZ , pegasysMatrixRotateXYZ ) ) ;
                    }
                    {
                        D3DXMATRIX    d3dMatrixRotateXYZ ;
                        D3DXMatrixRotationYawPitchRoll( & d3dMatrixRotateXYZ , 0.0f , 0.0f , angle ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixRotateXYZ ) ;

                        Mat44  pegasysMatrixRotateXYZ ;
                        pegasysMatrixRotateXYZ.SetRotationXYZ( Vec3( 0.0f , 0.0f , angle ) ) ;
                        //Mat44_DebugPrint( pegasysMatrixRotateXYZ ) ;
                        ASSERT( MatricesMatch( d3dMatrixRotateXYZ , pegasysMatrixRotateXYZ ) ) ;
                    }
#                   endif

                    {
                        Vec3Aligned axis( angle , angle * 3.0f , angle * 7.0f ) ;
                        axis.Normalize() ;
                        D3DXMATRIX    d3dMatrixRotateAxis ;
                        D3DXMatrixRotationAxis( & d3dMatrixRotateAxis , (D3DXVECTOR3 *) & axis , angle ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixRotateAxis ) ;

                        Mat44  pegasysMatrixRotateAxis ;
                        pegasysMatrixRotateAxis.SetRotation( axis , angle ) ;
                        //Mat44_DebugPrint( pegasysMatrixRotateAxis ) ;
                        ASSERT( MatricesMatch( d3dMatrixRotateAxis , pegasysMatrixRotateAxis , 1.0e-6f ) ) ;
                    }
                    {
                        D3DXMATRIX    d3dMatrixScale ;
                        D3DXMatrixScaling( & d3dMatrixScale , angle , 2.0f * angle , 3.0f * angle ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixScale ) ;

                        Mat44  pegasysMatrixScale ;
                        pegasysMatrixScale.SetScale( angle , 2.0f * angle , 3.0f * angle ) ;
                        //Mat44_DebugPrint( pegasysMatrixScale ) ;
                        ASSERT( MatricesMatch( d3dMatrixScale , pegasysMatrixScale ) ) ;
                    }
                    {
                        D3DXMATRIX    d3dMatrixTranslate ;
                        D3DXMatrixTranslation( & d3dMatrixTranslate , angle , 5.0f * angle , 7.0f * angle ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixTranslate ) ;

                        Mat44  pegasysMatrixTranslate ;
                        pegasysMatrixTranslate.SetTranslation( Vec3( angle , 5.0f * angle , 7.0f * angle ) ) ;
                        //Mat44_DebugPrint( pegasysMatrixTranslate ) ;
                        ASSERT( MatricesMatch( d3dMatrixTranslate , pegasysMatrixTranslate ) ) ;
                    }
                    {
                        const Vec4 eye   (         angle , 11.0f * angle , 13.0f * angle , 1.0f ) ;
                        const Vec3Aligned lookAt( 17.0f * angle , 19.0f * angle , 23.0f * angle ) ;
                        const Vec3Aligned up    ( 0.0f , 1.0f , 0.0f ) ;
                        D3DXMATRIX    d3dMatrixView ;
                        D3DXMatrixLookAtRH( & d3dMatrixView , (D3DXVECTOR3*) & eye , (D3DXVECTOR3*) & lookAt , (D3DXVECTOR3*) & up ) ;
                        //D3DxMatrix_DebugPrint( d3dMatrixView ) ;

                        Mat44  pegasysMatrixView ;
                        pegasysMatrixView.SetView( eye , lookAt , up ) ;
                        //Mat44_DebugPrint( pegasysMatrixView ) ;
                        ASSERT( MatricesMatch( d3dMatrixView , pegasysMatrixView ) ) ;
                    }
                }
            }

            // Test matrix-matrix routines.
            {
                const D3DXMATRIX    d3dMatrix(      1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                D3DXMATRIX          d3dMatrixTranspose ;
                D3DXMatrixTranspose( & d3dMatrixTranspose , & d3dMatrix ) ;
                //D3DxMatrix_DebugPrint( d3dMatrixTranspose ) ;

                const Mat44  pegasysMatrix(      1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                Mat44  pegasysMatrixTranspose = pegasysMatrix.GetTranspose() ;
                //Mat44_DebugPrint( pegasysMatrixTranspose ) ;
                ASSERT( MatricesMatch( d3dMatrixTranspose , pegasysMatrixTranspose ) ) ;
            }
            {
                const D3DXMATRIX    d3dMatrixA(     1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                const D3DXMATRIX    d3dMatrixB(    17.0f , 18.0f , 19.0f , 20.0f 
                    ,  21.0f , 22.0f , 23.0f , 24.0f
                    ,  25.0f , 26.0f , 27.0f , 28.0f
                    ,  29.0f , 30.0f , 31.0f , 32.0f ) ;
                D3DXMATRIX          d3dMatrixC ;
                D3DXMatrixMultiply( & d3dMatrixC , & d3dMatrixA , & d3dMatrixB ) ;
                //D3DxMatrix_DebugPrint( d3dMatrixC ) ;

                const Mat44  pegasysMatrixA(     1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                const Mat44  pegasysMatrixB(    17.0f , 18.0f , 19.0f , 20.0f 
                    ,  21.0f , 22.0f , 23.0f , 24.0f
                    ,  25.0f , 26.0f , 27.0f , 28.0f
                    ,  29.0f , 30.0f , 31.0f , 32.0f ) ;
                Mat44  pegasysMatrixC = pegasysMatrixA * pegasysMatrixB ;
                //Mat44_DebugPrint( pegasysMatrixC ) ;
                ASSERT( MatricesMatch( d3dMatrixC , pegasysMatrixC ) ) ;
            }

            // Test matrix-vector routines.
            {
                const D3DXMATRIX    d3dMatrix(      1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                const D3DXVECTOR4 dvIn( 17.0f , 18.0f , 19.0f , 20.0f ) ;
                D3DXVECTOR4 dvOut( 0.0f , 0.0f , 0.0f , 0.0f ) ;
                D3DXVec4Transform( & dvOut , & dvIn , & d3dMatrix ) ;
                //D3DxMatrix_DebugPrint( d3dMatrix ) ;
                DebugPrintf( "dvIn TRANSFORMED_BY d3dMatrix = %g %g %g %g\n" , dvOut.x , dvOut.y , dvOut.z , dvOut.w ) ;

                const Mat44  pegasysMatrix(      1.0f ,  2.0f ,  3.0f ,  4.0f 
                    ,   5.0f ,  6.0f ,  7.0f ,  8.0f
                    ,   9.0f , 10.0f , 11.0f , 12.0f
                    ,  13.0f , 14.0f , 15.0f , 16.0f ) ;
                //Mat44_DebugPrint( pegasysMatrix ) ;
                const Vec4 pvIn( 17.0f , 18.0f , 19.0f , 20.0f ) ;
                const Vec4 pvOut = pegasysMatrix * pvIn ;
                //Mat44_DebugPrint( pegasysMatrix ) ;
                DebugPrintf( "pegasysMatrix * pvIn = %g %g %g %g\n" , pvOut.x , pvOut.y , pvOut.z , pvOut.w ) ;
                ASSERT( Vector4sMatch( dvOut , pvOut ) ) ;
            }

            {
                const D3DXMATRIX    d3dMatrix(      1.0f , 2.0f , 3.0f , 0.0f 
                    ,   4.0f , 5.0f , 6.0f , 0.0f
                    ,   7.0f , 8.0f , 9.0f , 0.0f
                    ,   0.0f , 0.0f , 0.0f , 1.0f ) ;
                const D3DXVECTOR3 dvIn( 10.0f , 11.0f , 12.0f ) ;
                D3DXVECTOR4 dvOut( 0.0f , 0.0f , 0.0f , 0.0f ) ;
                D3DXVec3Transform( & dvOut , & dvIn , & d3dMatrix ) ;
                //D3DxMatrix_DebugPrint( d3dMatrix ) ;
                DebugPrintf( "dvIn TRANSFORMED_BY d3dMatrix = %g %g %g %g\n" , dvOut.x , dvOut.y , dvOut.z , dvOut.w ) ;

                const Mat33  pegasysMatrix(   Vec3( 1.0f , 2.0f , 3.0f )
                    , Vec3( 4.0f , 5.0f , 6.0f )
                    , Vec3( 7.0f , 8.0f , 9.0f ) ) ;
                const Vec3 pvIn( 10.0f , 11.0f , 12.0f ) ;
                const Vec3 pvOut = pegasysMatrix * pvIn ;
                DebugPrintf( "pegasysMatrix * pvIn = %g %g %g\n" , pvOut.x , pvOut.y , pvOut.z ) ;
                ASSERT( Vector3sMatch( dvOut , pvOut ) ) ;
            }

            DebugPrintf( "VectorMatrix_UnitTest: THE END ----------------------------------------------\n" ) ;
        }




        void D3D9_Api_UnitTest()
        {
            DebugPrintf( "D3D9_Api::UnitTest ----------------------------------------------\n" ) ;

            VectorMatrix_UnitTest() ;

            {
                D3D9_Api renderApi ;
            }

            DebugPrintf( "D3D9_Api::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#   endif

    } ;
} ;
