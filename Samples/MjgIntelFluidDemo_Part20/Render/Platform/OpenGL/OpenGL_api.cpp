/** \file OpenGL_api.cpp

    \brief Wrapper for routines specific to OpenGL render system API

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/

#include "Render/Platform/OpenGL/OpenGL_api.h"
#include "Render/Platform/OpenGL/OpenGL_window.h"
#include "Render/Platform/OpenGL/OpenGL_vertexBuffer.h"
#include "Render/Platform/OpenGL/OpenGL_indexBuffer.h"
#include "Render/Platform/OpenGL/OpenGL_mesh.h"
#include "Render/Platform/OpenGL/OpenGL_texture.h"
#include "Render/Platform/OpenGL/OpenGL_extensions.h"
#include "Render/Scene/model.h"
#include "Render/Scene/light.h"

#include "Core/Performance/perfBlock.h"
#include "Core/Utility/macros.h"
#include "Core/Memory/newWrapper.h"
#include "Core/File/debugPrint.h"

#if defined( WIN32 )
#   include <windows.h>
#endif

#include <GL/glut.h>

// Macros -----------------------------------------------------------------------

#define USE_SEPARATE_MATRIX 1

// Types -----------------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        // Private variables -----------------------------------------------------------
        // Public variables ------------------------------------------------------------
        // Private functions -----------------------------------------------------------

        static void CALLBACK DebugCallback( unsigned int /*source*/ , unsigned int /*type*/ , unsigned int /*id*/ , unsigned int severity , int /*length*/ , const char* /*message*/ , void* /*userParam*/ )
        {
            if( GL_DEBUG_SEVERITY_HIGH_ARB == severity )
            {
                FAIL() ;
            }
        }




        /// Set the projection matrix to orthographic, for rendering text.
        static void setOrthographicProjection()
        {
            PERF_BLOCK( setOrthographicProjection ) ;

            RENDER_CHECK_ERROR( setOrthographicProjection ) ;

            // TODO: Store matrix mode

            // switch to projection mode
            glMatrixMode( GL_PROJECTION ) ;

            // save previous matrix which contains the settings for the perspective projection
            glPushMatrix() ;

            // reset matrix
            glLoadIdentity() ;

            int viewportXYWH[4] ;
            glGetIntegerv( GL_VIEWPORT , viewportXYWH ) ;

            if( ( 0 == viewportXYWH[ 2 ] ) || ( 0 == viewportXYWH[ 3 ] ) )
            {   // Zero-size viewport.
                // Render window is probably minimized, which is okay, but calling gluOrtho2D would result in an invalid value.
            }
            else
            {   // Viewport is probably valid.
                // set a 2D orthographic projection
                gluOrtho2D( 0 , viewportXYWH[ 2 ] , viewportXYWH[ 3 ] , 0 ) ;
            }

            // switch back to modelview mode
            glMatrixMode( GL_MODELVIEW ) ;

            glPushMatrix();

            glLoadIdentity();

            // TODO: Restore matrix mode

            RENDER_CHECK_ERROR( setOrthographicProjection ) ;
        }




        /// Set the projection matrix to projection, for rendering 3D objects.
        static void restorePerspectiveProjection()
        {
            PERF_BLOCK( restorePerspectiveProjection ) ;

            RENDER_CHECK_ERROR( restorePerspectiveProjection ) ;

            // TODO: Store matrix mode

            glMatrixMode(GL_PROJECTION);
            // restore previous projection matrix
            glPopMatrix();

            // get back to modelview mode
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix() ;

            // TODO: Restore matrix mode

            RENDER_CHECK_ERROR( restorePerspectiveProjection ) ;
        }




        /// Render the given string using the given font.
        static void oglRenderBitmapString( void * font , const char * string )
        {
            PERF_BLOCK( ogleRenderBitmapString_2 ) ;

            for( const char * c = string ; *c != '\0' ; c ++ )
            {
                RENDER_CHECK_ERROR( oglRenderBitmapString_2_pre ) ;

                glutBitmapCharacter( font , *c ) ;

                RENDER_CHECK_ERROR( oglRenderBitmapString_2_post ) ;
            }
        }




        static void oglRenderBitmapString( const Vec3 & pos , bool useScreenSpace , void * font , const Vec4 & color , const char * string  )
        {
            PERF_BLOCK( ogleRenderBitmapString_5 ) ;

            RENDER_CHECK_ERROR( oglRenderBitmapString_5_pre ) ;

            if( useScreenSpace )
            {
                setOrthographicProjection() ;
            }

            glPushAttrib( GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT ) ; // Should include GL_ENABLE_BIT or GL_ALL_ATTRIB_BITS but for some reason that *sometimes* makes things disappear, like vortex lines.

            glDisable( GL_LIGHTING ) ;
            glDisable( GL_TEXTURE_2D ) ;
            glEnable( GL_BLEND ) ;
            glBlendFunc( GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA ) ;  // Typical alpha blending

            if( useScreenSpace )
            {   // Only when drawing in screen space...
                glDisable( GL_DEPTH_TEST ) ;    // Disable depth test so text appears as if in front of everything already drawn.
            }

            glColor4fv( reinterpret_cast< const GLfloat * >( & color ) ) ;

            //int viewportXYWH[4] ;
            //glGetIntegerv( GL_VIEWPORT , viewportXYWH ) ;

            glRasterPos3f( pos.x , pos.y , pos.z ) ;
            oglRenderBitmapString( font , string ) ;

            glPopAttrib() ;

            if( useScreenSpace )
            {
                restorePerspectiveProjection() ;
            }

            RENDER_CHECK_ERROR( oglRenderBitmapString_5_post ) ;
        }




        /// Render the given string at the given screen-space position using the given font.
        static void oglRenderString( const Vec3 & pos , bool useScreenSpace , void * font , const char * format , ... )
        {
            PERF_BLOCK( oglRenderString ) ;

            char stringBuffer[ 512 ] ;
            va_list args ;
            va_start( args , format ) ;
            _vsnprintf( stringBuffer , sizeof( stringBuffer ) , format , args ) ;
            va_end( args ) ;
            oglRenderBitmapString( pos , useScreenSpace , font , Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) , stringBuffer ) ;
        }




        /// Render the given string at the given world-space position using the given font.
        static void oglRenderStringWorld( const Vec3 & pos , void * font , const Vec4 & color , const char * format , ... )
        {
            PERF_BLOCK( oglRenderStringWorld ) ;

            char stringBuffer[ 512 ] ;
            va_list args ;
            va_start( args , format ) ;
            vsprintf( stringBuffer , format , args ) ;
            va_end( args ) ;
            oglRenderBitmapString( pos , false , font , color , stringBuffer ) ;
        }

        // Public functions ------------------------------------------------------------

        /* static */ void OpenGL_Api::GetProcAddresses()
        {
            PERF_BLOCK( OpenGL_Api__GetProcAddresses ) ;

            OpenGL_Extensions::GetProcAddresses() ;

            if( glDebugMessageCallbackARB )
            {
                glDebugMessageCallbackARB( & DebugCallback , NULL ) ;
            }
        }




        /// Check whether OpenGL detected an error and, if so, report it.
        /* static */ bool OpenGL_Api::CheckError( const char * situation )
        {
            PERF_BLOCK( OpenGL_Api__CheckError ) ;

            bool   hitAnyErrors = false ;
            GLenum glError      = GL_NO_ERROR ;
            while( ( glError = glGetError() ) != GL_NO_ERROR )
            {
#           if defined( _DEBUG )
                const GLubyte * glErrorString = gluErrorString( glError ) ;
                DebugPrintf( "OpenGL: %s: %s\n" , situation , glErrorString ) ;
#           else
                UNUSED_PARAM( situation ) ;
#           endif

                hitAnyErrors = true ;
            }

            if( hitAnyErrors )
            {
                DEBUG_BREAK() ;
            }

            return hitAnyErrors ;
        }




        /** Construct wrapper for routines specific to OpenGL render system API.
        */
        OpenGL_Api::OpenGL_Api()
        {
            PERF_BLOCK( OpenGL_Api__OpenGL_Api ) ;
        }




        /** Destruct wrapper for routines specific to OpenGL render system API.
        */
        OpenGL_Api::~OpenGL_Api()
        {
            PERF_BLOCK( OpenGL_Api__dtor ) ;
        }




        /* virtual */ Window * OpenGL_Api::NewWindow( System * renderSystem )
        {
            PERF_BLOCK( OpenGL_Api__NewWindow ) ;

            Render::Window * window = NEW Render::OpenGL_Window( renderSystem ) ;
            return window ;
        }




        static void SetRenderTarget( const Viewport & /*viewport*/ )
        {
            PERF_BLOCK( SetRenderTarget ) ;

            RENDER_CHECK_ERROR( OpenGL_before_SetRenderTarget ) ;

            // TODO: FIXME: Set render target.

#if 0 // Placeholder code to show the general idea and API calls needed.  Obviously this could not stay this way because it leaks resources. See http://www.opengl.org/wiki/Framebuffer_Object_Examples
            // Framebuffer: textures & depth buffer.
            GLuint framebufferName = 0;
            glGenFramebuffersEXT( 1 , & framebufferName ) ;
            glBindFramebufferEXT( GL_FRAMEBUFFER , framebufferName ) ;

            RENDER_CHECK_ERROR( OpenGL_SetRenderTarget_glGenBindFramebuffers ) ;

            // TODO: Only create new texture if needed, e.g. if it does not yet exist or if it has the wrong properties (such as size -- remember that windows can be resized).
            {
                // Generate texture to render to
                GLuint renderedTexture ;
                glGenTextures( 1 , & renderedTexture ) ;

                // Bind newly created texture
                glBindTexture( GL_TEXTURE_2D , renderedTexture ) ;

                RENDER_CHECK_ERROR( OpenGL_SetRenderTarget_glGenBindTextures ) ;

                // NOTE that the render target would have to change if the window resized.

                // Give an empty image to OpenGL
                glTexImage2D(GL_TEXTURE_2D
                    , 0
                    , GL_RGB
                    , viewport.GetTarget()->GetWidth()  // x resolution
                    , viewport.GetTarget()->GetHeight() // y resolution
                    , 0
                    , GL_RGB
                    , GL_UNSIGNED_BYTE
                    , /* empty image */ 0);

                // Set up texture filtering.
                glTexParameteri( GL_TEXTURE_2D , GL_TEXTURE_MAG_FILTER , GL_NEAREST ) ;
                glTexParameteri( GL_TEXTURE_2D , GL_TEXTURE_MIN_FILTER , GL_NEAREST ) ;

                // Set depth buffer
                {
                    GLuint depthrenderbuffer;
                    glGenRenderbuffersEXT( 1 , & depthrenderbuffer );  // This might be an EXT
                    glBindRenderbufferEXT( GL_RENDERBUFFER , depthrenderbuffer );  // This might be an EXT
                    glRenderbufferStorageEXT( GL_RENDERBUFFER
                        , GL_DEPTH_COMPONENT
                        , viewport.GetTarget()->GetWidth()
                        , viewport.GetTarget()->GetHeight()  ) ;
                    glFramebufferRenderbufferEXT( GL_FRAMEBUFFER , GL_DEPTH_ATTACHMENT , GL_RENDERBUFFER , depthrenderbuffer ) ;
                }

                RENDER_CHECK_ERROR( OpenGL_SetRenderTarget_glGenBindRenderbuffers ) ;

                // Set renderedTexture as color attachement0
                glFramebufferTextureEXT( GL_FRAMEBUFFER , GL_COLOR_ATTACHMENT0 , renderedTexture , 0 ) ;
            }

            {
                // Set draw buffers.
                GLenum DrawBuffers[2] = { GL_COLOR_ATTACHMENT0 } ;
                glDrawBuffersEXT( /* size of DrawBuffers */ 1 , DrawBuffers ) ;
            }

            // Bind framebuffer so subsequent rendering goes there.
            glBindFramebufferEXT( GL_FRAMEBUFFER , framebufferName ) ;

            RENDER_CHECK_ERROR( OpenGL_SetRenderTarget_glDrawBindBuffers) ;

#endif

            RENDER_CHECK_ERROR( OpenGL_SetRenderTarget ) ;
        }




        /* virtual */ void OpenGL_Api::SetViewport( const Viewport & viewport )
        {
            PERF_BLOCK( OpenGL_Api__SetViewport ) ;

            RENDER_CHECK_ERROR( before_SetViewport ) ;

            SetRenderTarget( viewport ) ;

            const int left      = static_cast< int >( viewport.GetTarget()->GetWidth()  * viewport.GetRelLeft() ) ;
            const int bottom    = static_cast< int >( viewport.GetTarget()->GetHeight() * ( 1.0f - viewport.GetRelTop() - viewport.GetRelHeight() ) ) ;
            const int width     = static_cast< int >( viewport.GetTarget()->GetWidth()  * viewport.GetRelWidth() ) ;
            const int height    = static_cast< int >( viewport.GetTarget()->GetHeight() * viewport.GetRelHeight() ) ;

            glViewport( left , bottom , width , height ) ;
            // Enable scissor to clear only the viewport

            glEnable( GL_SCISSOR_TEST ) ;

            glScissor( left , bottom , width , height ) ;
            glDepthMask( GL_TRUE ) ;  // Enable writing to the depth buffer

            glClearColor( viewport.GetClearColor().x , viewport.GetClearColor().y , viewport.GetClearColor().z , viewport.GetClearColor().w ) ;

            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ) ;

            RENDER_CHECK_ERROR( SetViewport ) ;
        }




        /* virtual */ void OpenGL_Api::SetCamera( const Camera & camera )
        {
            PERF_BLOCK( OpenGL_Api__SetCamera ) ;

            {   // Set view transformation
                glPushAttrib( GL_TRANSFORM_BIT ) ;
                glMatrixMode( GL_MODELVIEW ) ;
                glLoadIdentity() ;
                gluLookAt(  camera.GetEye().x           , camera.GetEye().y             , camera.GetEye().z             ,
                            camera.GetLookAt().x        , camera.GetLookAt().y          , camera.GetLookAt().z          ,
                            camera.GetUpApproximate().x , camera.GetUpApproximate().y   , camera.GetUpApproximate().z   ) ;

                // TODO: Instead of querying OpenGL for the view transform, change code above to compose view matrix (as per below) and use glSetFloatv to set it.  That way, D3D and OGL would use identical matrices and would avoid querying render API for data.
                glGetFloatv( GL_MODELVIEW_MATRIX , (GLfloat *) & mRenderStateCache.mCurrentState.mTransforms.mViewMatrix ) ;

#               if defined( _DEBUG )
                {   // TODO: Move this into a unit test.
                    Mat44 xViewFromApi ;
                    glGetFloatv( GL_MODELVIEW_MATRIX , (float*) & xViewFromApi ) ;
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

                glPopAttrib() ;
            }


            {   // Set projection transformation
                glPushAttrib( GL_TRANSFORM_BIT ) ;
                glMatrixMode( GL_PROJECTION ) ;

                glLoadIdentity() ;  // If OpenGL is in SELECT mode, a pick projection matrix should already be in place, in which case setting this would be bad.

                gluPerspective(   camera.GetFieldOfViewVert()
                                , camera.GetAspectRatio()   // Note that if this aspect ratio does not match that of the viewport then the image will look distorted.
                                , camera.GetNearClipDist()
                                , camera.GetFarClipDist() ) ;
                glPopAttrib() ;
            }

            RENDER_CHECK_ERROR( SetCamera ) ;
        }




        /** Set the render state to use the given light.

            In contrast to Direct3D, OpenGL lights are transformed by the MODELVIEW
            matrix.  This routine assumes the MODELVIEW matrix is that of the current
            camera.
        */
        void OpenGL_Api::SetLight( unsigned idx , const Light & light )
        {
            PERF_BLOCK( OpenGL_Api__SetLight ) ;

            const int glIdx = GL_LIGHT0 + idx ;   // NOTE: this assumes GL_LIGHT0 through GL_LIGHT7 have consecutive values
            ASSERT( GL_LIGHT0 + 1 == GL_LIGHT1 ) ;
            ASSERT( GL_LIGHT0 + 2 == GL_LIGHT2 ) ;
            ASSERT( GL_LIGHT0 + 3 == GL_LIGHT3 ) ;
            ASSERT( GL_LIGHT0 + 4 == GL_LIGHT4 ) ;
            ASSERT( GL_LIGHT0 + 5 == GL_LIGHT5 ) ;
            ASSERT( GL_LIGHT0 + 6 == GL_LIGHT6 ) ;
            ASSERT( GL_LIGHT0 + 7 == GL_LIGHT7 ) ;

            glLightfv( glIdx , GL_AMBIENT  , reinterpret_cast< const float * >( & light.GetAmbientColor() ) ) ;
            glLightfv( glIdx , GL_DIFFUSE  , reinterpret_cast< const float * >( & light.GetDiffuseColor() ) ) ;
            glLightfv( glIdx , GL_SPECULAR , reinterpret_cast< const float * >( & light.GetSpecularColor() ) ) ;

            {   // TODO: Concatenate parent transform.
                glPushAttrib( GL_TRANSFORM_BIT ) ;
                glMatrixMode( GL_MODELVIEW ) ;
                glPushMatrix() ;

                //glLoadMatrixf( (GLfloat*) & mRenderStateCache.mCurrentState.mTransforms.mViewMatrix ); // This routine assumes the MODELVIEW matrix is set to the view transform.  Otherwise, this call would have to execute.

                if( Light::DIRECTIONAL == light.GetLightType() )
                {   // "Directional" light (excluding spotlights, which also have direction).
                    // OpenGL uses position to mean direction for directional lights.
                    ASSERT( 0.0f == light.GetDir4().w ) ;
                    // OpenGL "light direction" means the direction toward the light,
                    // which is the opposite of Light::GetDirection.
                    const Vec4 towardLight = - light.GetDir4() ;
                    glLightfv( glIdx , GL_POSITION , reinterpret_cast< const float * >( & towardLight ) ) ;
                }
                else
                {   // Light is not so-called "directional" (but could still be a spotlight, which has direction).
                    ASSERT( 1.0f == light.GetPos4().w ) ;
                    glLightfv( glIdx , GL_POSITION , reinterpret_cast< const float * >( & light.GetPos4() ) ) ;

                    ASSERT( 0.0f == light.GetDir4().w ) ;
                    glLightfv( glIdx , GL_SPOT_DIRECTION , reinterpret_cast< const float * >( & light.GetDirection() ) ) ;
                }
                glPopMatrix() ;
                glPopAttrib() ;
            }

            ASSERT( ( light.GetSpotFalloff() <= 0.0f ) && ( light.GetSpotFalloff() <= 128.0f ) ) ;
            glLightf( glIdx , GL_SPOT_EXPONENT          , light.GetSpotFalloff()            ) ;

            ASSERT( ( light.GetSpotOuterAngle() == 180.0f ) || ( ( light.GetSpotOuterAngle() >= 0.0f ) && ( light.GetSpotOuterAngle() <= 90.0f ) ) ) ;
            glLightf( glIdx , GL_SPOT_CUTOFF            , light.GetSpotOuterAngle()         ) ;
            glLightf( glIdx , GL_CONSTANT_ATTENUATION   , light.GetConstAttenuation()       ) ;
            glLightf( glIdx , GL_LINEAR_ATTENUATION     , light.GetLinearAttenuation()      ) ;
            glLightf( glIdx , GL_QUADRATIC_ATTENUATION  , light.GetQuadracticAttenuation()  ) ;

            // Enable this light.
            glEnable( glIdx ) ;

            RENDER_CHECK_ERROR( SetLight ) ;
        }




        /** Set the render state to use the light cached for the given receiver.

            \note   If the model does not have normals, setting lights will cause that
                    model to render as black.            
        */
        /* virtual */ void OpenGL_Api::SetLights( const ModelNode & lightReceiver )
        {
            PERF_BLOCK( OpenGL_Api__SetLights ) ;

            const unsigned numLights = lightReceiver.GetNumLights() ;
            ASSERT( numLights < GL_MAX_LIGHTS ) ;
            if( numLights > 0 )
            {   // Model has at least one light.
                glEnable( GL_LIGHTING ) ;
                for( unsigned idx = 0 ; idx < numLights ; ++ idx )
                {   // For each light in the given model's cache...
                    const Light * light = lightReceiver.GetLight( idx ) ;
                    SetLight( idx , * light ) ;
                }
            }
            else
            {   // Model has no lights so disable lighting.
                glDisable( GL_LIGHTING ) ;
            }

            RENDER_CHECK_ERROR( SetLights ) ;
        }




        /** Concatenate local-to-world transform to ModelView matrix for the given SceneNode.

            This modified the current ModelView matrix.
            Caller should store ModelView matrix and restore it later.

        */
        /* virtual */ void OpenGL_Api::SetLocalToWorld( const Mat44 & localToWorld )
        {
            PERF_BLOCK( OpenGL_Api__SetLocalToWorld ) ;

            glMultMatrixf( (GLfloat*) & localToWorld ) ;

            RENDER_CHECK_ERROR( SetLocalToWorld ) ;
        }




        /** Render simple text for on-screen diagnostic messages.

            \param  position    Position in whatever coordinate space of the current render state matrices.
        */
        /* virtual */ void  OpenGL_Api::RenderSimpleText( const char * text , const Vec3 & position , bool useScreenSpace )
        {
            PERF_BLOCK( OpenGL_Api__RenderSimpleText ) ;

            oglRenderString( position , useScreenSpace , GLUT_BITMAP_HELVETICA_10 , text ) ;
        }




        /* virtual */ VertexBufferBase *  OpenGL_Api::NewVertexBuffer()
        {
            PERF_BLOCK( OpenGL_Api__NewVertexBuffer ) ;

            VertexBufferBase * vertexBuffer = NEW OpenGL_VertexBuffer ;
            return vertexBuffer ;
        }




        /* virtual */ IndexBufferBase *  OpenGL_Api::NewIndexBuffer()
        {
            PERF_BLOCK( OpenGL_Api__NewIndexBuffer ) ;

            IndexBufferBase * indexBuffer = NEW OpenGL_IndexBuffer ;
            return indexBuffer ;
        }




        /* virtual */ void OpenGL_Api::DeleteIndexBuffer( IndexBufferBase * indexBuffer )
        {
            PERF_BLOCK( OpenGL_Api__DeleteIndexBuffer ) ;

            delete indexBuffer ;
        }




        /* virtual */ MeshBase *  OpenGL_Api::NewMesh( ModelData * owningModelData )
        {
            PERF_BLOCK( OpenGL_Api__NewMesh ) ;

            MeshBase * mesh = NEW OpenGL_Mesh( owningModelData ) ;
            return mesh ;
        }



        /* virtual */ TextureBase *  OpenGL_Api::NewTexture()
        {
            PERF_BLOCK( OpenGL_Api__NewTexture ) ;

            TextureBase * texture = NEW OpenGL_Texture ;
            return texture ;
        }




        /* virtual */ void OpenGL_Api::ApplyRenderState( const RenderStateS & renderState )
        {
            PERF_BLOCK( OpenGL_Api__ApplyRenderState ) ;

            mRenderStateCache.Apply( renderState ) ;
        }




        /* virtual */ void OpenGL_Api::GetRenderState( RenderStateS & renderState )
        {
            PERF_BLOCK( OpenGL_Api__GetRenderState ) ;

            OpenGL_RenderStateCache::GetRenderState( renderState ) ;
        }




        /* virtual */ void OpenGL_Api::DisableTexturing()
        {
            PERF_BLOCK( OpenGL_Api__DisableTexturing ) ;

            glDisable( GL_TEXTURE_1D ) ;
            glDisable( GL_TEXTURE_2D ) ;
            //glDisable( GL_TEXTURE_3D ) ;
            //glDisable( GL_TEXTURE_CUBE_MAP ) ;
        }








#   if defined( _DEBUG )

        void PeGaSys_Render_OpenGL_Api_UnitTest()
        {
            DebugPrintf( "OpenGL_Api::UnitTest ----------------------------------------------\n" ) ;

            {
                OpenGL_Api renderApi ;
            }

            DebugPrintf( "OpenGL_Api::UnitTest: THE END ----------------------------------------------\n" ) ;
        }
#   endif

    } ;
} ;
