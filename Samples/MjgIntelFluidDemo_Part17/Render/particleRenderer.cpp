/** \file particleRenderer.cpp

    \brief Class to render particles

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include <math.h>
#include <limits.h>

#include <xmmintrin.h>

#if defined( WIN32 )
    #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/gl_ext.h>

#include "Core/Math/mat4.h"
#include "Core/Performance/perf.h"

#include "particleMaterial.h"

#include "particleRenderer.h"

/// Quick and dirty way to quasi-randomize a floating point value.
#define SHUFFLE_BITS( i )     ( ( (i) & 0x80000000 ) >>  4 ) \
                            | ( ( (i) & 0x08000000 ) >> 14 ) \
                            | ( ( (i) & 0x00800000 ) >> 20 ) \
                            | ( ( (i) & 0x00100000 ) << 11 ) \
                            | ( ( (i) & 0x00040000 ) >> 10 ) \
                            | ( ( (i) & 0x00020000 ) >>  5 ) \
                            | ( ( (i) & 0x00004000 ) <<  4 ) \
                            | ( ( (i) & 0x00002000 ) >> 12 ) \
                            | ( ( (i) & 0x00001000 ) >>  2 ) \
                            | ( ( (i) & 0x00000400 ) << 13 ) \
                            | ( ( (i) & 0x00000100 ) >>  8 ) \
                            | ( ( (i) & 0x00000040 ) << 11 ) \
                            | ( ( (i) & 0x00000008 ) >>  1 ) \
                            | ( ( (i) & 0x00000004 ) << 12 ) \
                            | ( ( (i) & 0x00000002 ) <<  5 ) \
                            | ( ( (i) & 0x00000001 ) << 20 )



#if USE_ORIENTED_SORTED_PARTICLES
/// Custom vertex format with position+normal+texture coordinates.
struct VertexFormatPositionNormalTexture
{
    float tu, tv ;      // 2D texture coordinates
    float nx, ny, nz ;  // surface normal unit vector
    float px, py, pz ;  // untransformed (world-space) position
} ;
/// Nickname for vertex format with position+normal+texture coordinates.
#define VertexFormatFlags_NormalTexture (GL_T2F_N3F_V3F)
#endif

/// Custom vertex format with position+color+texture coordinates.
struct VertexFormatPos3Col4Tex2
{
    float           tu, tv      ;   ///< 2D texture coordinates.
    unsigned char   col[4]      ;   ///< Color (red, green, blue, alpha)
    float           px, py, pz  ;   ///< Untransformed (world-space) position.
} ;
/// Nickname for vertex format with position+color+texture coordinates.
#define VertexFormatFlags_Pos3Col4Tex2 (GL_T2F_C4UB_V3F)

/// Custom vertex format with position+texture coordinates.
struct VertexFormatPos3Tex2
{
    float tu , tv ;             ///< texture coordinates
    float px , py , pz ;        ///< untransformed (world-space) position
} ;
/// Nickname for vertex format with position+texture coordinates.
#define VertexFormatFlags_Pos3Tex2  (GL_T2F_V3F)

/// Custom vertex format with position+texture (special).
struct VertexFormatPos4Tex4
{
    float tu , tv , ts , tt ;   ///< particle orientation (xyz), particle size and vertex index (w=size+index*shift)
    float px , py , pz , pw ;   ///< untransformed (world-space) position (xyz) and birth time (w)
} ;
/// Nickname for vertex format with position+texture (special).
#define VertexFormatFlags_Pos4Tex4  (GL_T4F_V4F)




#if USE_SEPARATE_VBOS

/// Custom vertex format for position only.
struct VertexFormatPos3
{
    float px , py , pz ;   ///< untransformed (world-space) position (xyz)
} ;
#define VertexFormatFlags_Pos3 (GL_V3F)

/// Custom vertex format for position only.
struct VertexFormatPos3Col4
{
    unsigned char   col[4]          ;   ///< Color (red, green, blue, alpha)
    float           px , py , pz    ;   ///< Untransformed (world-space) position.
} ;
#define VertexFormatFlags_Pos3Col4 (GL_C4UB_V3F)

/// Custom vertex format for texture coordinates.
/// This format has no GL_* macro; it is used as a separate array.
/// Separating texture coordinates lets us use the fixed function
/// pipeline without having to refill tex coords each frame.
struct VertexFormatTex2
{
    float tu , tv ;             ///< texture coordinates
} ;

#endif




// VBO Extension Function Pointers
PFN_GL_GENBUFFERSARB_PROC       glGenBuffers    = 0 ;   ///< VBO Name Generation Procedure
PFN_GL_MAPBUFFERARB_PROC        glMapBuffer     = 0 ;   ///< VBO Map Buffer procedure
PFN_GL_UNMAPBUFFERARB_PROC      glUnmapBuffer   = 0 ;   ///< VBO Unmap Buffer procedure
PFN_GL_BINDBUFFERARB_PROC       glBindBuffer    = 0 ;   ///< VBO Bind Procedure
PFN_GL_BUFFERDATAARB_PROC       glBufferData    = 0 ;   ///< VBO Data Loading Procedure
PFN_GL_DELETEBUFFERSARB_PROC    glDeleteBuffers = 0 ;   ///< VBO Deletion Procedure

bool gVboSupported  =   false ; ///< ARB_vertex_buffer_object supported?

#if USE_TBB

    /** Function object to fill a vertex buffer using Threading Building Blocks.
    */
    class ParticleRenderer_FillVertexBuffer_TBB
    {
            ParticleRenderer * mParticleRenderer ;    ///< Address of ParticleRenderer object
            const double     & mTimeNow          ;
            const Mat44      & mViewMatrix       ;
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {
                SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
                SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
                mParticleRenderer->FillVertexBufferSlice( mTimeNow , mViewMatrix , r.begin() , r.end() ) ;
            }
            ParticleRenderer_FillVertexBuffer_TBB( ParticleRenderer * pParticleRenderer , const double & timeNow , const Mat44 & viewMatrix )
                : mParticleRenderer( pParticleRenderer )
                , mTimeNow( timeNow )
                , mViewMatrix( viewMatrix )
            {
                mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
                mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            }
        private:
            WORD        mMasterThreadFloatingPointControlWord   ;
            unsigned    mMasterThreadMmxControlStatusRegister   ;
    } ;
#endif




#if USE_VERTEX_BUFFER_OBJECT
static bool IsExtensionSupported( char* szTargetExtension )
{
    const unsigned char *pszExtensions = NULL;
    const unsigned char *pszStart;
    unsigned char *pszWhere, *pszTerminator;

    // Extension names should not have spaces
    pszWhere = (unsigned char *) strchr( szTargetExtension, ' ' );
    if( pszWhere || *szTargetExtension == '\0' )
        return false;

    // Get Extensions String
    pszExtensions = glGetString( GL_EXTENSIONS );
    CheckGlError() ;

    // Search The Extensions String For An Exact Copy
    pszStart = pszExtensions;
    for(;;)
    {
        pszWhere = (unsigned char *) strstr( (const char *) pszStart, szTargetExtension );
        if( !pszWhere )
            break;
        pszTerminator = pszWhere + strlen( szTargetExtension );
        if( pszWhere == pszStart || *( pszWhere - 1 ) == ' ' )
            if( *pszTerminator == ' ' || *pszTerminator == '\0' )
                return true;
        pszStart = pszTerminator;
    }
    return false;
}
#endif




/** Lookup various OpenGL extensions and prepare to use them.

    This application can use certain features of OpenGL which are
    not standard across all platforms and all implementations.
    For example, some of these extensions do not work when using
    Remote Desktop or on extremely old video adapters.
*/
void PrepareOpenGlExtensions()
{
    static bool sExtensionsInitialized  =   false ; // ARB Extention data initialized?
    if( ! sExtensionsInitialized )
    {   // VBO extention data has not yet been initialized
        gVboSupported = IsExtensionSupported( "GL_ARB_vertex_buffer_object" ) ;
        if( gVboSupported )
        {   // This machine supports Vertex Buffer Objects.
            // Get addresses of GL VBO functions.
            glGenBuffers    = (PFN_GL_GENBUFFERSARB_PROC   ) wglGetProcAddress( "glGenBuffers"    );
            CheckGlError() ;
            if( glGenBuffers )
            {	// IsExtensionSupported actually told the truth.
                glMapBuffer     = (PFN_GL_MAPBUFFERARB_PROC    ) wglGetProcAddress( "glMapBuffer"     );
                glUnmapBuffer   = (PFN_GL_UNMAPBUFFERARB_PROC  ) wglGetProcAddress( "glUnmapBuffer"   );
                glBindBuffer    = (PFN_GL_BINDBUFFERARB_PROC   ) wglGetProcAddress( "glBindBuffer"    );
                glBufferData    = (PFN_GL_BUFFERDATAARB_PROC   ) wglGetProcAddress( "glBufferData"    );
                glDeleteBuffers = (PFN_GL_DELETEBUFFERSARB_PROC) wglGetProcAddress( "glDeleteBuffers" );
                CheckGlError() ;
                sExtensionsInitialized = true ;
            }
            else
            {	// Failed to find routines that support vertex buffer objects.
                const bool glDebugOutputSupported = IsExtensionSupported( "GL_ARB_debug_output" ) ;
                const GLubyte * glVendor     = glGetString( GL_VENDOR ) ;
                const GLubyte * glRenderer   = glGetString( GL_RENDERER ) ;
                const GLubyte * glVersion    = glGetString( GL_VERSION ) ;
                const GLubyte * glExtensions = glGetString( GL_EXTENSIONS ) ;
                (void) glDebugOutputSupported , glVendor , glRenderer , glVersion , glExtensions ; // Used for debugging only.
                gVboSupported = false ;
            }
        }
    }
}




static inline float texCoordForPage0of2( float vZeroToOne )
{
    return vZeroToOne * 0.5f ;
}




static inline float texCoordForPage1of2( float vZeroToOne )
{
    return vZeroToOne * 0.5f + 0.5f ;
}




/** Construct object to render particles.
*/
ParticleRenderer::ParticleRenderer( const char * pParticleData , size_t stride , size_t offsetToAngVel , size_t offsetToSize , size_t offsetToDensityInfo , ParticleMaterial & material )
    : mParticleData( pParticleData )
    , mStride( stride )
    , mOffsetToAngVel( offsetToAngVel )
    , mOffsetToSize( offsetToSize )
    , mOffsetToDensityInfo( offsetToDensityInfo )
    , mVertexBuffer( 0 )
    , mVertexBufferCapacity( 0 )
    , mMaterial( material )
#if SORT_PARTICLES
    , mIndices( 0 )
    , mIndicesCapacity( 0 )
#endif
#if USE_SEPARATE_VBOS
    , mVertexBufferGrew( true )
#endif
{
#if USE_VERTEX_BUFFER_OBJECT
    memset( mVboNames , 0 , sizeof( mVboNames ) ) ;
#endif
}




/** Destruct object to render vortex particles.
*/
ParticleRenderer::~ParticleRenderer()
{
#if USE_VERTEX_BUFFER_OBJECT
    if( gVboSupported )
    {   // Used new-style vertex buffer objects.
        // Tell OpenGL to delete vertex buffer object allocated in "GPU domain"
        glDeleteBuffers( NUM_VBO_MAX , mVboNames ) ;
        CheckGlError() ;
    }
    else
#endif
    {   // Used old-style vertex arrays (not vertex buffer objects).
        delete[] mVertexBuffer ;    // Delete buffer allocated by CPU
    }

    mVertexBuffer           = 0 ;
    mVertexBufferCapacity   = 0 ;

#if SORT_PARTICLES
    if( mIndices != 0 )
    {
        free( mIndices ) ;
        mIndices = 0 ;
        mIndicesCapacity = 0 ;
    }
#endif
}




#if SORT_PARTICLES
static int CompareParticles( const void * p0 , const void * p1 )
{
    ParticleRenderer::ParticleIndex * pi0 = (ParticleRenderer::ParticleIndex *) p0 ;
    ParticleRenderer::ParticleIndex * pi1 = (ParticleRenderer::ParticleIndex *) p1 ;
    //return pi0->mDepth - pi1->mDepth ;
    if ( pi0->mDepth > pi1->mDepth )
    {
        return 1 ;
    }
    else if ( pi0->mDepth < pi1->mDepth )
    {
        return -1 ;
    }
    else
    {
        return 0 ;
    }
}
#endif




/** Assign vertices for particles.

    \param timeNow      Current virtual time.

    \param viewMatrix   Matrix describing camera transformation.

    \param iPclStart    Index of first particle to fill.

    \param iPclEnd      Index-plus-one of last particle to fill.

*/
void ParticleRenderer::FillVertexBufferSlice( const double & timeNow , const Mat44 & viewMatrix , size_t iPclStart , size_t iPclEnd )
{
    static const unsigned   nvpp    = 4 ; // number of vertices per particle

    // Extract world space direction vectors associated with view (used to 
    // compute camera-facing coordinates).
    // Note that these vectors are the unit vectors of the inverse of the view matrix.
    // They are the world-space unit vectors of the view transformation.
    Vec3        viewRight       ( viewMatrix.m[0][0] , viewMatrix.m[1][0] , viewMatrix.m[2][0] ) ;
    Vec3        viewUp          ( viewMatrix.m[0][1] , viewMatrix.m[1][1] , viewMatrix.m[2][1] ) ;
    Vec3        viewForward     ( viewMatrix.m[0][2] , viewMatrix.m[1][2] , viewMatrix.m[2][2] ) ;

    ASSERT( ( mMaterial.GetNumTexPages() == 1 ) || ( mMaterial.GetNumTexPages() == 2 ) ) ;
    const bool  lightAndHeavy   = mMaterial.GetNumTexPages() > 1 ;
    const float v0Heavy         = lightAndHeavy ? texCoordForPage1of2( 0.01f ) : 0.0f ;
    const float v1Heavy         = lightAndHeavy ? texCoordForPage1of2( 0.99f ) : 1.0f ;

#if USE_ORIENTED_SORTED_PARTICLES
    //ASSERT( mMaterial.GetNumTexPages() == 1 ) ; // Oriented sorted particles do not currently support heavy & light density.

    const float oneOverUintMax  = 1.0f / float( UINT_MAX ) ;
    const float fraction        = 0.1f ;

    // Fill vertex buffer with oriented particles.
    VertexFormatPositionNormalTexture * pVertices = ( VertexFormatPositionNormalTexture * ) mVertexBuffer ;
    for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
    {   // For each particle in this slice...
        const char *    pPos        = mParticleData + mIndices[ iPcl ].mPcl * mStride ;
        const char *    pAngVel     = pPos + mOffsetToAngVel ;
        const char *    pSize       = pPos + mOffsetToSize ;
        const Vec3  &   pclPos      = * ( (Vec3*) pPos ) ;
        const Vec3  &   pclAngVel   = * ( (Vec3*) pAngVel ) ;
        const float     fPhase      = TWO_PI * float( SHUFFLE_BITS( iPcl ) ) * oneOverUintMax ;
        const float     pclAngle    = ( pclAngVel * timeNow ).Magnitude() + fPhase ;
        const float     sinAngle    = sin( pclAngle ) ;
        const float     cosAngle    = cos( pclAngle ) ;
        const float &   rHalfSize   = * ( (float*) pSize ) * 0.5f ;
        Vec3            pclRight    = (   viewRight * cosAngle + viewUp * sinAngle ) * rHalfSize ;
        Vec3            pclUp       = ( - viewRight * sinAngle + viewUp * cosAngle ) * rHalfSize ;
        //const char *    pDensityInfo    = pPos + mOffsetToDensityInfo ;
        //const float &   rDensityInfo    = * ( (float*) pDensityInfo ) ;

        // Assign positions and texture coordinates for each vertex of the quadrilateral.
        pVertices[ iPcl*nvpp   ].tu = 1.0f ;
        pVertices[ iPcl*nvpp   ].tv = v0Heavy ;
        pVertices[ iPcl*nvpp   ].nx = ( pclRight.x + pclUp.x ) + fraction * viewForward.x ;
        pVertices[ iPcl*nvpp   ].ny = ( pclRight.y + pclUp.y ) + fraction * viewForward.y ;
        pVertices[ iPcl*nvpp   ].nz = ( pclRight.z + pclUp.z ) + fraction * viewForward.z ;
        ((Vec3*) (&pVertices[ iPcl*nvpp   ].nx))->NormalizeFast() ;
        pVertices[ iPcl*nvpp   ].px = ( pclPos.x + pclRight.x + pclUp.x ) ;
        pVertices[ iPcl*nvpp   ].py = ( pclPos.y + pclRight.y + pclUp.y ) ;
        pVertices[ iPcl*nvpp   ].pz = ( pclPos.z + pclRight.z + pclUp.z ) ;

        pVertices[ iPcl*nvpp+1 ].tu = 0.0f ;
        pVertices[ iPcl*nvpp+1 ].tv = v0Heavy ;
        pVertices[ iPcl*nvpp+1 ].nx = ( - pclRight.x + pclUp.x ) + fraction * viewForward.x ;
        pVertices[ iPcl*nvpp+1 ].ny = ( - pclRight.y + pclUp.y ) + fraction * viewForward.y ;
        pVertices[ iPcl*nvpp+1 ].nz = ( - pclRight.z + pclUp.z ) + fraction * viewForward.z ;
        ((Vec3*) (&pVertices[ iPcl*nvpp+1 ].nx))->NormalizeFast() ;
        pVertices[ iPcl*nvpp+1 ].px = ( pclPos.x - pclRight.x + pclUp.x ) ;
        pVertices[ iPcl*nvpp+1 ].py = ( pclPos.y - pclRight.y + pclUp.y ) ;
        pVertices[ iPcl*nvpp+1 ].pz = ( pclPos.z - pclRight.z + pclUp.z ) ;

        pVertices[ iPcl*nvpp+2 ].tu = 0.0f ;
        pVertices[ iPcl*nvpp+2 ].tv = v1Heavy ;
        pVertices[ iPcl*nvpp+2 ].nx = ( - pclRight.x - pclUp.x ) + fraction * viewForward.x ;
        pVertices[ iPcl*nvpp+2 ].ny = ( - pclRight.y - pclUp.y ) + fraction * viewForward.y ;
        pVertices[ iPcl*nvpp+2 ].nz = ( - pclRight.z - pclUp.z ) + fraction * viewForward.z ;
        ((Vec3*) (&pVertices[ iPcl*nvpp+2 ].nx))->NormalizeFast() ;
        pVertices[ iPcl*nvpp+2 ].px = ( pclPos.x - pclRight.x - pclUp.x ) ;
        pVertices[ iPcl*nvpp+2 ].py = ( pclPos.y - pclRight.y - pclUp.y ) ;
        pVertices[ iPcl*nvpp+2 ].pz = ( pclPos.z - pclRight.z - pclUp.z ) ;

        pVertices[ iPcl*nvpp+3 ].tu = 1.0f ;
        pVertices[ iPcl*nvpp+3 ].tv = v1Heavy ;
        pVertices[ iPcl*nvpp+3 ].nx = ( pclRight.x - pclUp.x ) + fraction * viewForward.x ;
        pVertices[ iPcl*nvpp+3 ].ny = ( pclRight.y - pclUp.y ) + fraction * viewForward.y ;
        pVertices[ iPcl*nvpp+3 ].nz = ( pclRight.z - pclUp.z ) + fraction * viewForward.z ;
        ((Vec3*) (&pVertices[ iPcl*nvpp+3 ].nx))->NormalizeFast() ;
        pVertices[ iPcl*nvpp+3 ].px = ( pclPos.x + pclRight.x - pclUp.x ) ;
        pVertices[ iPcl*nvpp+3 ].py = ( pclPos.y + pclRight.y - pclUp.y ) ;
        pVertices[ iPcl*nvpp+3 ].pz = ( pclPos.z + pclRight.z - pclUp.z ) ;
    }

#else // Fill vertex buffer with "plain" particles without normals

    const float v0Light         = lightAndHeavy ? texCoordForPage0of2( 0.01f ) : 0.0f ;
    const float v1Light         = lightAndHeavy ? texCoordForPage0of2( 0.99f ) : 1.0f ;
    #if USE_SEPARATE_VBOS
    if( gVboSupported )
    {
        if( mVertexBufferGrew ) // Assign texture coordinates only if the vertex buffer grew.
        // When density varies over time, and since density determines texture coordinates,
        // texture coordinates need to get assigned every frame, in which case separate VBO
        // does not help.  In fact, it hurts because the same memory gets visited multiple
        // times, and it's too much memory to fit in cache.
        {   // Assign texture coordinates.
            VertexFormatTex2 * pVertTC  = ( VertexFormatTex2 * ) mTexCoordBuffer ;
            for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
            {   // For each particle in this slice...
            #if SORT_PARTICLES
                const char *    pPos            = mParticleData + mIndices[ iPcl ].mPcl * mStride ;
            #else
                const char *    pPos            = mParticleData + iPcl * mStride ;
            #endif
                //const char *    pSize           = pPos + mOffsetToSize ;
                //const float &   rHalfSize       = * ( (float*) pSize ) * 0.5f ;
                const char *    pDensityInfo    = pPos + mOffsetToDensityInfo ;
                const float     rDensityInfo    = * ( (float*) pDensityInfo ) ;

                // Assign positions and texture coordinates for each vertex of the quadrilateral.

                // Assign which texture page to use based on sense of density variation.
                // This assignment coupled with the assumption that texture coordinates only
                // get assigned when the vertex buffer grows, assumes particle either that
                // density sense does not vary and that particles do not change indices (i.e.
                // that they do not die) or that particle density sense is uniform across all particles.
                const float v0 = rDensityInfo >= 1.0f ? v0Heavy : v0Light ;
                const float v1 = rDensityInfo >= 1.0f ? v1Heavy : v1Light ;

                // Assign texture coordinates for each vertex of the quadrilateral.
                // These can be reused each frame, as long as the vertex buffer does not grow.

                // Note, however, that for emitting sims where density could change each moment,
                // this approach will break down, and this code would need to regen
                // tex coord buffers each time a particle died or was emitted,
                // which would defeat the purpose of using separate buffers.

                pVertTC[ iPcl*nvpp   ].tu = 1.0f ;
                pVertTC[ iPcl*nvpp   ].tv = v0 ;

                pVertTC[ iPcl*nvpp+1 ].tu = 0.0f ;
                pVertTC[ iPcl*nvpp+1 ].tv = v0 ;

                pVertTC[ iPcl*nvpp+2 ].tu = 0.0f ;
                pVertTC[ iPcl*nvpp+2 ].tv = v1 ;

                pVertTC[ iPcl*nvpp+3 ].tu = 1.0f ;
                pVertTC[ iPcl*nvpp+3 ].tv = v1 ;
            }
        }

        VertexFormatPos3Col4 * pVertColPos = ( VertexFormatPos3Col4 * ) mVertexBuffer  ;
        for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
        {   // For each particle in this slice...
            // Obtain information about particle position, size and orientation.
            const char *    pPos            = mParticleData + iPcl * mStride ;
            const Vec3  &   pclPos          = * ( (Vec3*) pPos ) ;
            const char *    pSize           = pPos + mOffsetToSize ;
            const char *    pAngVel         = pPos + mOffsetToAngVel ;
            const Vec3  &   pclAngVel       = * ( (Vec3*) pAngVel ) ;
            const float     pclAngle        = ( pclAngVel * timeNow ).Magnitude() ;
            const float     sinAngle        = sin( pclAngle ) ;
            const float     cosAngle        = cos( pclAngle ) ;
            const float     rHalfSize       = mMaterial.GetScale() * * ( (float*) pSize ) * 0.5f ;
            Vec3            pclRight        = (   viewRight * cosAngle + viewUp * sinAngle ) * rHalfSize ;
            Vec3            pclUp           = ( - viewRight * sinAngle + viewUp * cosAngle ) * rHalfSize ;
            const char *    pDensityInfo    = pPos + mOffsetToDensityInfo ;
            const float     rDensityInfo    = * ( (float*) pDensityInfo ) ;

            int colorMod = 255 ;
            int alphaMod = 255 ;

            const float massFraction = lightAndHeavy ? fabsf( rDensityInfo - 1.0f ) : fabsf( rDensityInfo ) ;
            if( mMaterial.GetBlendMode() == QdMaterial::BM_ADDITIVE )
            {   // These are luminous particles.  Vary brightness (not opacity).
                colorMod = Min2( 255 , massFraction * mMaterial.GetDensityVisibility() ) ;
            }
            else
            {   // These are translucent particles.  Vary opacity (not color).
                ASSERT( mMaterial.GetBlendMode() == QdMaterial::BM_ALPHA ) ;
                alphaMod = Min2( 255 , massFraction * mMaterial.GetDensityVisibility() ) ;
            }

            // Assign positions for each vertex of the quadrilateral.
            pVertColPos[ iPcl*nvpp   ].col[0] = colorMod ;
            pVertColPos[ iPcl*nvpp   ].col[1] = colorMod ;
            pVertColPos[ iPcl*nvpp   ].col[2] = colorMod ;
            pVertColPos[ iPcl*nvpp   ].col[3] = alphaMod ;
            pVertColPos[ iPcl*nvpp   ].px     = ( pclPos.x + pclRight.x + pclUp.x ) ;
            pVertColPos[ iPcl*nvpp   ].py     = ( pclPos.y + pclRight.y + pclUp.y ) ;
            pVertColPos[ iPcl*nvpp   ].pz     = ( pclPos.z + pclRight.z + pclUp.z ) ;

            pVertColPos[ iPcl*nvpp+1 ].col[0] = colorMod ;
            pVertColPos[ iPcl*nvpp+1 ].col[1] = colorMod ;
            pVertColPos[ iPcl*nvpp+1 ].col[2] = colorMod ;
            pVertColPos[ iPcl*nvpp+1 ].col[3] = alphaMod ;
            pVertColPos[ iPcl*nvpp+1 ].px     = ( pclPos.x - pclRight.x + pclUp.x ) ;
            pVertColPos[ iPcl*nvpp+1 ].py     = ( pclPos.y - pclRight.y + pclUp.y ) ;
            pVertColPos[ iPcl*nvpp+1 ].pz     = ( pclPos.z - pclRight.z + pclUp.z ) ;

            pVertColPos[ iPcl*nvpp+2 ].col[0] = colorMod ;
            pVertColPos[ iPcl*nvpp+2 ].col[1] = colorMod ;
            pVertColPos[ iPcl*nvpp+2 ].col[2] = colorMod ;
            pVertColPos[ iPcl*nvpp+2 ].col[3] = alphaMod ;
            pVertColPos[ iPcl*nvpp+2 ].px     = ( pclPos.x - pclRight.x - pclUp.x ) ;
            pVertColPos[ iPcl*nvpp+2 ].py     = ( pclPos.y - pclRight.y - pclUp.y ) ;
            pVertColPos[ iPcl*nvpp+2 ].pz     = ( pclPos.z - pclRight.z - pclUp.z ) ;

            pVertColPos[ iPcl*nvpp+3 ].col[0] = colorMod ;
            pVertColPos[ iPcl*nvpp+3 ].col[1] = colorMod ;
            pVertColPos[ iPcl*nvpp+3 ].col[2] = colorMod ;
            pVertColPos[ iPcl*nvpp+3 ].col[3] = alphaMod ;
            pVertColPos[ iPcl*nvpp+3 ].px     = ( pclPos.x + pclRight.x - pclUp.x ) ;
            pVertColPos[ iPcl*nvpp+3 ].py     = ( pclPos.y + pclRight.y - pclUp.y ) ;
            pVertColPos[ iPcl*nvpp+3 ].pz     = ( pclPos.z + pclRight.z - pclUp.z ) ;
        }
    }
    else
    #endif
    {
        // Fill vertex buffer with "plain" particles without normals
        VertexFormatPos3Col4Tex2 * pVertices = ( VertexFormatPos3Col4Tex2 * ) mVertexBuffer ;
        for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
        {   // For each particle in this slice...
        #if SORT_PARTICLES
            const char *    pPos            = mParticleData + mIndices[ iPcl ].mPcl * mStride ;
        #else
            const char *    pPos            = mParticleData + iPcl * mStride ;
        #endif
            const Vec3  &   pclPos          = * ( (Vec3*) pPos ) ;
            const char *    pSize           = pPos + mOffsetToSize ;
            const char *    pAngVel         = pPos + mOffsetToAngVel ;
            const char *    pDensityInfo    = pPos + mOffsetToDensityInfo ;
            const Vec3  &   pclAngVel       = * ( (Vec3*) pAngVel ) ;
            const float     pclAngle        = ( pclAngVel * timeNow ).Magnitude() ;
            const float     sinAngle        = sin( pclAngle ) ;
            const float     cosAngle        = cos( pclAngle ) ;
#if 1
            const float     rHalfSize       = mMaterial.GetScale() * * ( (float*) pSize ) * 0.5f ;
#else
//#error DO NOT SUBMIT hard-coded
const float     rHalfSize       = 0.05f ; (void) pSize ;
#endif
            Vec3            pclRight        = (   viewRight * cosAngle + viewUp * sinAngle ) * rHalfSize ;
            Vec3            pclUp           = ( - viewRight * sinAngle + viewUp * cosAngle ) * rHalfSize ;
            const float     rDensityInfo    = * ( (float*) pDensityInfo ) ;

            // Assign positions and texture coordinates for each vertex of the quadrilateral.

            // Assign which texture page to use based on sense of density variation.
            const float v0 = rDensityInfo >= 1.0f ? v0Heavy : v0Light ;
            const float v1 = rDensityInfo >= 1.0f ? v1Heavy : v1Light ;

            int colorMod = 255 ;
            int alphaMod = 255 ;

            const float massFraction = lightAndHeavy ? fabsf( rDensityInfo - 1.0f ) : fabsf( rDensityInfo ) ;
            if( mMaterial.GetBlendMode() == QdMaterial::BM_ADDITIVE )
            {   // These are luminous particles.  Vary brightness (not opacity).
                colorMod = Min2( 255.0f , massFraction * mMaterial.GetDensityVisibility() ) ;
            }
            else
            {   // These are translucent particles.  Vary opacity (not color).
                ASSERT( mMaterial.GetBlendMode() == QdMaterial::BM_ALPHA ) ;
                alphaMod = Clamp( massFraction * mMaterial.GetDensityVisibility() , 0.0f , 255.0f ) ;
            }

            pVertices[ iPcl*nvpp   ].tu     = 1.0f ;
            pVertices[ iPcl*nvpp   ].tv     = v0 ;
            pVertices[ iPcl*nvpp   ].col[0] = colorMod ;
            pVertices[ iPcl*nvpp   ].col[1] = colorMod ;
            pVertices[ iPcl*nvpp   ].col[2] = colorMod ;
            pVertices[ iPcl*nvpp   ].col[3] = alphaMod ;
            pVertices[ iPcl*nvpp   ].px     = ( pclPos.x + pclRight.x + pclUp.x ) ;
            pVertices[ iPcl*nvpp   ].py     = ( pclPos.y + pclRight.y + pclUp.y ) ;
            pVertices[ iPcl*nvpp   ].pz     = ( pclPos.z + pclRight.z + pclUp.z ) ;

            pVertices[ iPcl*nvpp+1 ].tu     = 0.0f ;
            pVertices[ iPcl*nvpp+1 ].tv     = v0 ;
            pVertices[ iPcl*nvpp+1 ].col[0] = colorMod ;
            pVertices[ iPcl*nvpp+1 ].col[1] = colorMod ;
            pVertices[ iPcl*nvpp+1 ].col[2] = colorMod ;
            pVertices[ iPcl*nvpp+1 ].col[3] = alphaMod ;
            pVertices[ iPcl*nvpp+1 ].px     = ( pclPos.x - pclRight.x + pclUp.x ) ;
            pVertices[ iPcl*nvpp+1 ].py     = ( pclPos.y - pclRight.y + pclUp.y ) ;
            pVertices[ iPcl*nvpp+1 ].pz     = ( pclPos.z - pclRight.z + pclUp.z ) ;

            pVertices[ iPcl*nvpp+2 ].tu     = 0.0f ;
            pVertices[ iPcl*nvpp+2 ].tv     = v1 ;
            pVertices[ iPcl*nvpp+2 ].col[0] = colorMod ;
            pVertices[ iPcl*nvpp+2 ].col[1] = colorMod ;
            pVertices[ iPcl*nvpp+2 ].col[2] = colorMod ;
            pVertices[ iPcl*nvpp+2 ].col[3] = alphaMod ;
            pVertices[ iPcl*nvpp+2 ].px     = ( pclPos.x - pclRight.x - pclUp.x ) ;
            pVertices[ iPcl*nvpp+2 ].py     = ( pclPos.y - pclRight.y - pclUp.y ) ;
            pVertices[ iPcl*nvpp+2 ].pz     = ( pclPos.z - pclRight.z - pclUp.z ) ;

            pVertices[ iPcl*nvpp+3 ].tu     = 1.0f ;
            pVertices[ iPcl*nvpp+3 ].tv     = v1 ;
            pVertices[ iPcl*nvpp+3 ].col[0] = colorMod ;
            pVertices[ iPcl*nvpp+3 ].col[1] = colorMod ;
            pVertices[ iPcl*nvpp+3 ].col[2] = colorMod ;
            pVertices[ iPcl*nvpp+3 ].col[3] = alphaMod ;
            pVertices[ iPcl*nvpp+3 ].px     = ( pclPos.x + pclRight.x - pclUp.x ) ;
            pVertices[ iPcl*nvpp+3 ].py     = ( pclPos.y + pclRight.y - pclUp.y ) ;
            pVertices[ iPcl*nvpp+3 ].pz     = ( pclPos.z + pclRight.z - pclUp.z ) ;
        }
    }
#endif
}




#if SORT_PARTICLES
/** Assign indices used to access particle data and sort them by depth.
*/
void ParticleRenderer::AssignIndicesAndSortParticles( const Vec3 & viewForward , size_t numParticles )
{
    QUERY_PERFORMANCE_ENTER ;

    if( numParticles > mIndicesCapacity )
    {   // Need to allocate more space for particle index map
        if( mIndices != 0 )
        {   // Index map already exists so delete it first
            free( mIndices ) ;
        }
        mIndices    = (ParticleIndex *) malloc( sizeof( ParticleIndex ) * numParticles ) ;
        mIndicesCapacity = numParticles ;
    }

    for( unsigned iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {   // For each particle...
        const char *    pPos    = mParticleData + iPcl * mStride ;
        const Vec3   &  pclPos  = * ( (Vec3*) pPos) ;
        // Assign particle index map values.
        // Later these will be sorted and therefore by proxy so will the particles
        mIndices[ iPcl ].mPcl   = iPcl ;
        mIndices[ iPcl ].mDepth = pclPos * viewForward ;
    }

    static bool bSort = false ;
    if( bSort )
    {
        qsort( mIndices , numParticles , sizeof( ParticleIndex ) , CompareParticles ) ;
    }

    QUERY_PERFORMANCE_EXIT( ParticlesRender_AssignIndicesAndSortParticles ) ;
}
#endif




/** Render particles.

    \param timeNow      Current virtual time.

    \param timeChange   Change in virtual time since last call to Render.

    \param numParticles Number of particles to render, which mParticleData points to.

*/
void ParticleRenderer::Render( double timeNow , float /* timeChange */ , size_t numParticles )
{
    QUERY_PERFORMANCE_ENTER ;

    QUERY_PERFORMANCE_ENTER ;
    mMaterial.UseMaterial() ;
    CheckGlError() ;
    QUERY_PERFORMANCE_EXIT( ParticleRenderer_Render_SetParticleMaterial ) ;

    if( 0 == numParticles )
    {   // No particles to render so do nothing
        return ;
    }

#if USE_ORIENTED_SORTED_PARTICLES
    unsigned                vertexFormatFlags   = VertexFormatFlags_NormalTexture ;
    unsigned                vertexFormatSize    = sizeof( VertexFormatPositionNormalTexture ) ;
#else
    unsigned                vertexFormatFlags   = VertexFormatFlags_Pos3Col4Tex2 ;
    unsigned                vertexFormatSize    = sizeof( VertexFormatPos3Col4Tex2 ) ;
#endif
    static const unsigned   nvpp                = 4 ; // number of vertices per particle
    const size_t            numVertices         = numParticles * nvpp ;

#if USE_VERTEX_BUFFER_OBJECT
    PrepareOpenGlExtensions() ;

    if( gVboSupported )
    {
        #if USE_SEPARATE_VBOS
            // When using separate VBO's, allocate position and texCoord buffers separately.
            vertexFormatSize    = sizeof( VertexFormatPos3Col4 ) ;
            vertexFormatFlags   = VertexFormatFlags_Pos3Col4 ; // This is actually irrelevant, but I set it anyway.
        #endif

        if(     ( 0.0 == timeNow )                              // This is the first step in a simulation.
            ||  (   ( mVboNames[ 0 ] != 0 )                     // vertex buffer object already exists...
                &&  ( mVertexBufferCapacity < numVertices ) )   // new size needs more space than old buffer has
          )
        {   // Free previous vertex buffer object in preparation for creating a new one.
            // Note that one of the conditions under which this occurs is on the first time step.
            // That is to force rebuilding texture coordinate buffers so that
            // they correspond to the current initial conditions.
            // Note, however, that for emitting sims where density could change each moment,
            // this approach will break down, and this code would need to regen
            // tex coord buffers each time a particle died or was emitted,
            // which would defeat the purpose of using separate buffers.
            glDeleteBuffers( 1 , & mVboNames[ 0 ] ) ;
            CheckGlError() ;
            mVboNames[ 0 ] = 0 ;
        #if USE_SEPARATE_VBOS
            glDeleteBuffers( 1 , & mVboNames[ TEX_VBO ] ) ; // Delete texCoord VBO
            mVboNames[ TEX_VBO ] = 0     ;                  // Forget old texCoord VBO "name".
        #endif
        }

        if( 0 == mVboNames[ 0 ] )
        {   // Vertex buffer object is not allocated.
            // Reserve VBO buffer identifier
            glGenBuffers( 1 , & mVboNames[ 0 ] ) ;
            CheckGlError() ;
            mVertexBufferCapacity = numVertices ;
            // Create vertex buffer.  This allocates enough memory to hold all our vertex data.
            glBindBuffer( GL_ARRAY_BUFFER , mVboNames[ 0 ] ) ;
            CheckGlError() ;
            const unsigned uVboSize = vertexFormatSize * numVertices ;
            glBufferData( GL_ARRAY_BUFFER , uVboSize , NULL , GL_STREAM_DRAW ) ;
            CheckGlError() ;

        #if USE_SEPARATE_VBOS
            // Reserve VBO buffer identifier for texCoords
            glGenBuffers( 1 , & mVboNames[ TEX_VBO ] ) ;
            // Create vertex buffer for texCoords.  This allocates memory that the GPU can use directly.
            glBindBuffer( GL_ARRAY_BUFFER , mVboNames[ TEX_VBO ] ) ;
            {
                const unsigned uVboSize = (unsigned) sizeof( VertexFormatTex2 ) * numVertices ;
                glBufferData( GL_ARRAY_BUFFER , uVboSize , NULL , GL_STREAM_DRAW ) ;
            }
            mTexCoordBuffer     = (VERTEX_BUFFER_POINTER_TYPE) glMapBuffer( GL_ARRAY_BUFFER , GL_WRITE_ONLY ) ;
            ASSERT( mTexCoordBuffer ) ;
            mVertexBufferGrew   = true  ;   // Vertex buffer needs more texture coordinates
        }
        else
        {
            mVertexBufferGrew   = false ;   // Existing texture coordinate buffer is still useful
        #endif
        }

        glBindBuffer( GL_ARRAY_BUFFER , mVboNames[ 0 ] ) ;
        CheckGlError() ;
        mVertexBuffer = (VERTEX_BUFFER_POINTER_TYPE) glMapBuffer( GL_ARRAY_BUFFER , GL_WRITE_ONLY ) ;
        CheckGlError() ;
        ASSERT( mVertexBuffer != 0 ) ;
    }
    else
#endif
    {   // Use old-style vertex arrays (not vertex buffer objects)
        if(     ( mVertexBuffer != 0 )                      // vertex buffer already exists...
            &&  ( mVertexBufferCapacity < numVertices ) )   // new size needs more space than old buffer has
        {   // Free previous vertex buffer in preparation for creating a new one.
            delete[] mVertexBuffer ;
            mVertexBuffer           = 0 ;
            mVertexBufferCapacity   = 0 ;
        }

        if( 0 == mVertexBuffer )
        {   // Vertex buffer is not allocated.
            // Create vertex buffer.  This allocates enough memory to hold all our vertex data.
            // Also specify vertex format, so device knows what data vertex buffer contains.
            mVertexBuffer = new unsigned char[ vertexFormatSize * numVertices ] ;
            mVertexBufferCapacity = numVertices ;
        }
    }

    Mat44 viewMatrix ;
    glGetFloatv( GL_MODELVIEW_MATRIX , (GLfloat *) & viewMatrix ) ;
    CheckGlError() ;

    // Extract world space direction vectors associated with view (used to compute camera-facing coordinates).
    // Note that these vectors are the unit vectors of the inverse of the view matrix.
    // They are the world-space unit vectors of the view transformation.
    Vec3 viewRight  ( viewMatrix.m[0][0] , viewMatrix.m[1][0] , viewMatrix.m[2][0] ) ;
    Vec3 viewUp     ( viewMatrix.m[0][1] , viewMatrix.m[1][1] , viewMatrix.m[2][1] ) ;
    Vec3 viewForward( viewMatrix.m[0][2] , viewMatrix.m[1][2] , viewMatrix.m[2][2] ) ;

#if SORT_PARTICLES
    AssignIndicesAndSortParticles( viewForward , numParticles ) ;
#endif

    // Fill vertex buffer with geometric primitives (quadrilaterals)
    QUERY_PERFORMANCE_ENTER ;

#if USE_TBB
    // Estimate grain size based on size of problem and number of processors.
    const size_t grainSize =  Max2( size_t( 1 ) , numParticles / gNumberOfProcessors ) ;
    // Fill vertex buffer using threading building blocks
    parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , ParticleRenderer_FillVertexBuffer_TBB( this , timeNow , viewMatrix ) ) ;
#else
    FillVertexBufferSlice( timeNow , viewMatrix , 0 , numParticles ) ;
#endif
    QUERY_PERFORMANCE_EXIT( ParticlesRender_FillVertexBuffer ) ;

    // Render particles
    QUERY_PERFORMANCE_ENTER ;

    // Inform OpenGL where vertex data resides and what its layout is.
#if USE_VERTEX_BUFFER_OBJECT
    if( gVboSupported )
    {   // Use new-style vertex buffer objects (VBO's)
        #if USE_ORIENTED_SORTED_PARTICLES
        {
            static const GLubyte * offsetTu = (GLubyte*) offsetof( VertexFormatPositionNormalTexture , tu ) ;
            static const GLubyte * offsetNx = (GLubyte*) offsetof( VertexFormatPositionNormalTexture , nx ) ;
            static const GLubyte * offsetPx = (GLubyte*) offsetof( VertexFormatPositionNormalTexture , px ) ;
            glBindBuffer( GL_ARRAY_BUFFER , mVboNames[ 0 ] ) ;
            glUnmapBuffer( GL_ARRAY_BUFFER ) ;
            glTexCoordPointer( 2 , GL_FLOAT , vertexFormatSize , offsetTu ) ;
            glNormalPointer  (     GL_FLOAT , vertexFormatSize , offsetNx ) ;
            glVertexPointer  ( 3 , GL_FLOAT , vertexFormatSize , offsetPx ) ;
            CheckGlError() ;
        }
        #elif USE_SEPARATE_VBOS
        {
            static const GLubyte * offsetTu = (GLubyte*) offsetof( VertexFormatTex2     , tu  ) ;
            glBindBuffer( GL_ARRAY_BUFFER , mVboNames[ TEX_VBO ] ) ;
            CheckGlError() ;
            glUnmapBuffer( GL_ARRAY_BUFFER ) ;
            CheckGlError() ;
            glTexCoordPointer( 2 , GL_FLOAT , sizeof( VertexFormatTex2 ) , offsetTu ) ;
            CheckGlError() ;

            static const GLubyte * offsetCr = (GLubyte*) offsetof( VertexFormatPos3Col4 , col ) ;
            static const GLubyte * offsetPx = (GLubyte*) offsetof( VertexFormatPos3Col4 , px  ) ;
            glBindBuffer( GL_ARRAY_BUFFER , mVboNames[ POS_VBO ] ) ;
            glUnmapBuffer( GL_ARRAY_BUFFER ) ;
            glColorPointer ( 4 , GL_UNSIGNED_BYTE   , vertexFormatSize , offsetCr ) ;
            glVertexPointer( 3 , GL_FLOAT           , vertexFormatSize , offsetPx ) ;
            CheckGlError() ;
        }
        #else
        {
            static const GLubyte * offsetTu = (GLubyte*) offsetof( VertexFormatPos3Col4Tex2 , tu  ) ;
            static const GLubyte * offsetCr = (GLubyte*) offsetof( VertexFormatPos3Col4Tex2 , col ) ;
            static const GLubyte * offsetPx = (GLubyte*) offsetof( VertexFormatPos3Col4Tex2 , px  ) ;
            glBindBuffer( GL_ARRAY_BUFFER , mVboNames[ 0 ] ) ;
            glUnmapBuffer( GL_ARRAY_BUFFER ) ;
            glTexCoordPointer( 2 , GL_FLOAT         , vertexFormatSize , offsetTu ) ;
            glColorPointer   ( 4 , GL_UNSIGNED_BYTE , vertexFormatSize , offsetCr ) ;
            glVertexPointer  ( 3 , GL_FLOAT         , vertexFormatSize , offsetPx ) ;
            CheckGlError() ;
        }
        #endif

        glEnableClientState( GL_VERTEX_ARRAY ) ;
        CheckGlError() ;
    #if USE_ORIENTED_SORTED_PARTICLES
        glEnableClientState( GL_NORMAL_ARRAY ) ;
    #else
        glEnableClientState( GL_COLOR_ARRAY ) ;
    #endif
        CheckGlError() ;
        glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;
        CheckGlError() ;
    }
    else
#endif
    {   // Use old-style vertex arrays (not vertex buffer objects)
        glInterleavedArrays( vertexFormatFlags , vertexFormatSize , mVertexBuffer ) ;
        CheckGlError() ;
    }

#if 1   // Draw quads
    glDrawArrays( GL_QUADS , 0 , (GLsizei) numVertices ) ;
    CheckGlError() ;
#else   // Draw point sprites.  For this to be useful would require changing how vertex buffer gets filled:  Texture coordinates are overridden, texture should have 1 domain, only need 1 vert per particle, uniform size for all particles.  Faster but much less flexible.
    // Specify point sprite texture coordinate replacement mode for each texture unit
    glTexEnvf( GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE );
    glPointSize( 1.0f );
    glEnable( GL_POINT_SPRITE_ARB ) ;

    // Draw point sprites
    glDrawArrays( GL_POINTS , 0 , (GLsizei) numVertices ) ;

    glDisable( GL_POINT_SPRITE_ARB ) ;
    CheckGlError() ;
#endif

    QUERY_PERFORMANCE_EXIT( ParticlesRender_Draw ) ;

    QUERY_PERFORMANCE_EXIT( ParticlesRender ) ;
}
