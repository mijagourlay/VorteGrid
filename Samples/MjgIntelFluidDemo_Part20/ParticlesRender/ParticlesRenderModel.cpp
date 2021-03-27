/** \file particlesRenderModel.cpp

    \brief Model scene node for a particle system.

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.

*/
#include <ParticlesRender/particlesRenderModel.h>

#include <Particles/particle.h>
#include <Particles/particleSystemManager.h>

#include <Render/Device/api.h>

#include <Render/Resource/mesh.h>
#include <Render/Resource/renderState.h>

#include <Render/Scene/iSceneManager.h>
#include <Render/Scene/modelData.h>
#include <Render/Scene/light.h>
#include <Render/Scene/model.h>

#include <Render/Platform/OpenGL/OpenGL_vertexBuffer.h>
#include <Render/Platform/DirectX9/D3D9_vertexBuffer.h>

#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

#include <d3d9.h>
#include <d3d9types.h>

// Types -----------------------------------------------------------------------

typedef PeGaSys::Render::OpenGL_VertexBuffer::VertexFormatPositionColor4Texture2 VertexFormatPos3Col4Tex2 ;

#if USE_TBB

/** Function object to fill a vertex buffer, using Threading Building Blocks.
*/
class ParticleRenderer_FillVertexBuffer_TBB
{
    //QdParticleRenderer * mParticleRenderer ;    ///< Address of QdParticleRenderer object
    PeGaSys::ParticlesRender::ParticlesRenderModel::IVertexBufferFiller * mVbFiller ;
    unsigned char       * mVertBytes        ;
    const ParticleGroup * mParticleGroup    ;
    const double        & mTimeNow          ;
    const Mat44         & mViewMatrix       ;
public:
    void operator() ( const tbb::blocked_range<size_t> & range ) const
    {
        SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
        SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
        (*mVbFiller)( mVertBytes , mParticleGroup , mTimeNow , mViewMatrix , range.begin() , range.end() ) ;
    }

    ParticleRenderer_FillVertexBuffer_TBB( PeGaSys::ParticlesRender::ParticlesRenderModel::IVertexBufferFiller * vbFiller
        , unsigned char * vertBytes
        , const ParticleGroup * particleGroup
        , const double  & timeNow
        , const Mat44   & viewMatrix )
        : mVbFiller( vbFiller )
        , mVertBytes( vertBytes )
        , mParticleGroup( particleGroup )
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

// Private variables -----------------------------------------------------------

static const size_t sPclStride                  = sizeof( Particle )                      ;
static const size_t sPclOffsetToAngVel          = offsetof( Particle , mAngularVelocity ) ;
static const size_t sPclOffsetToSize            = offsetof( Particle , mSize            ) ;

static const size_t sPclOffsetToDensity         = offsetof( Particle , mDensity         ) ;
#if ENABLE_FIRE
static const size_t sPclOffsetToFuelFraction    = offsetof( Particle , mFuelFraction    ) ;
static const size_t sPclOffsetToSmokeFraction   = offsetof( Particle , mSmokeFraction   ) ;
static const size_t sPclOffsetToFlameFraction   = offsetof( Particle , mFlameFraction   ) ;
#endif

// Public variables ------------------------------------------------------------

namespace PeGaSys
{
    namespace ParticlesRender
    {

        using namespace Render ;


// Private functions -----------------------------------------------------------

        static inline float texCoordForPage0of2( float vZeroToOne )
        {
            return vZeroToOne * 0.5f ;
        }




        static inline float texCoordForPage1of2( float vZeroToOne )
        {
            return vZeroToOne * 0.5f + 0.5f ;
        }


#       define INDEX(     type , address , offsetInBytes )             reinterpret_cast< type >( reinterpret_cast< char * >( address ) + offsetInBytes )

#       define INCREMENT( type , pointer , strideInBytes ) ( pointer = reinterpret_cast< type >( reinterpret_cast< char * >( pointer ) + strideInBytes ) )

// Public functions ------------------------------------------------------------

        // VertexBufferFillerGeneric ------------------------------------------------------------

        /** Assign vertices for particles.

            \param timeNow      Current virtual time.

            \param viewMatrix   Matrix describing camera transformation.

            \param iPclStart    Index of first particle to fill.

            \param iPclEnd      Index-plus-one of last particle to fill.

        */
        void ParticlesRenderModel::VertexBufferFillerGeneric::operator()(
              unsigned char * vertexBytes
            , const ParticleGroup * particleGroup
            , const double & timeNow
            , const struct Mat44 & viewMatrix
            , size_t iPclStart
            , size_t iPclEnd
            )
        {
            ASSERT( mIsActive ) ; // VB filler must be active.  If it isn't, caller should set VB population to zero and not call this routine.
            ASSERT( vertexBytes   != NULLPTR ) ;  // Must have called InitializeResources beforehand
            ASSERT( particleGroup != NULLPTR ) ;  // Must have called AssociateWithParticleSystem beforehand
            ASSERT( mVertStride    > 0 ) ;

            if( iPclStart >= iPclEnd )
            {
                return ;
            }

            const VECTOR< Particle > &  particleVector  = particleGroup->GetParticles() ;
            const Particle *            particleData    = particleVector.Empty() ? NULLPTR : & particleVector[ 0 ] ;
            const char *                particleBytes   = reinterpret_cast< const char * >( particleData ) ;

            static const unsigned       numVerticesPerParticle = 4 ;

            // Extract world space direction vectors associated with view (used to 
            // compute camera-facing coordinates).
            // Note that these vectors are the unit vectors of the inverse of the view matrix.
            // They are the world-space unit vectors of the view transformation.
            Vec3        viewRight       ( viewMatrix.m[0][0] , viewMatrix.m[1][0] , viewMatrix.m[2][0] ) ;
            Vec3        viewUp          ( viewMatrix.m[0][1] , viewMatrix.m[1][1] , viewMatrix.m[2][1] ) ;
            Vec3        viewForward     ( viewMatrix.m[0][2] , viewMatrix.m[1][2] , viewMatrix.m[2][2] ) ;

            const float v0Heavy         = mUseDensityForTextureCoordinate ? texCoordForPage1of2( 0.01f ) : 0.0f ;
            const float v1Heavy         = mUseDensityForTextureCoordinate ? texCoordForPage1of2( 0.99f ) : 1.0f ;
            const float v0Light         = mUseDensityForTextureCoordinate ? texCoordForPage0of2( 0.01f ) : 0.0f ;
            const float v1Light         = mUseDensityForTextureCoordinate ? texCoordForPage0of2( 0.99f ) : 1.0f ;

            // Fill single vertex buffer with "plain" particles without normals

            for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
            {   // For each particle in this slice...
#           if SORT_PARTICLES
                const char *    pPos            = particleBytes + mIndices[ iPcl ].mPcl * mPclStride ;
#           else
                const char *    pPos            = particleBytes + iPcl * mPclStride ;
#           endif
                const Vec3  &   pclPos          = * ( (Vec3*) pPos ) ;
                const char *    pSize           = pPos + mOffsetToSize ;
                const char *    pAngVel         = pPos + mOffsetToAngVel ;
                const char *    pDensityInfo    = pPos + mOffsetToDensityInfo ;
                const Vec3  &   pclAngVel       = * ( (Vec3*) pAngVel ) ;
                const float     pclAngle        = ( pclAngVel * static_cast< float >( timeNow ) ).Magnitude() ;
                const float     sinAngle        = sin( pclAngle ) ;
                const float     cosAngle        = cos( pclAngle ) ;
                const float     rHalfSize       = mScale* * ( (float*) pSize ) * 0.5f ;
                Vec3            pclRight        = (   viewRight * cosAngle + viewUp * sinAngle ) * rHalfSize ;
                Vec3            pclUp           = ( - viewRight * sinAngle + viewUp * cosAngle ) * rHalfSize ;
                const float     rDensityInfo    = * ( (float*) pDensityInfo ) ;

                ASSERT( ! IsNan( pclPos       ) ) ;
                ASSERT( ! IsNan( pclAngVel    ) ) ;
                ASSERT( ! IsNan( pclRight     ) ) ;
                ASSERT( ! IsNan( pclUp        ) ) ;
                ASSERT( ! IsNan( rDensityInfo ) ) ;

                // Assign positions and texture coordinates for each vertex of the quadrilateral.

                // Assign which texture page to use based on sense of density variation.
                const float v0 = rDensityInfo >= 1.0f ? v0Heavy : v0Light ;
                const float v1 = rDensityInfo >= 1.0f ? v1Heavy : v1Light ;

                unsigned char colorMod = 255 ;
                unsigned char alphaMod = 255 ;

                const float massFraction = mUseDensityForTextureCoordinate ? fabsf( rDensityInfo - 1.0f ) : fabsf( rDensityInfo ) ;
                if( BLEND_MODE_ADDITIVE == mBlendMode )
                {   // These are luminous particles.  Vary brightness (not opacity).
                    colorMod = (unsigned char) Min2( 255.0f , massFraction * mDensityVisibility ) ;
                }
                else
                {   // These are translucent particles.  Vary opacity (not color).
                    ASSERT( BLEND_MODE_ALPHA == mBlendMode ) ;
                    alphaMod = (unsigned char) Clamp( massFraction * mDensityVisibility , 0.0f , 255.0f ) ;
                }

                const size_t idxVert = iPcl * numVerticesPerParticle ;
                const size_t offsetVert = idxVert * mVertStride ;

                Vec3 *  vPos = INDEX( Vec3 *  , vertexBytes , offsetVert + mVertPositionOffset ) ;
                float * vTex = INDEX( float * , vertexBytes , offsetVert + mVertTextureCoordOffset ) ;
                UINT8 * vRed = INDEX( UINT8 * , vertexBytes , offsetVert + mVertRedOffset ) ;
                UINT8 * vGrn = INDEX( UINT8 * , vertexBytes , offsetVert + mVertGrnOffset ) ;
                UINT8 * vBlu = INDEX( UINT8 * , vertexBytes , offsetVert + mVertBluOffset ) ;
                UINT8 * vAlp = INDEX( UINT8 * , vertexBytes , offsetVert + mVertAlpOffset ) ;

                vTex[ 0 ] = 1.0f ;
                vTex[ 1 ] = v0 ;
                * vRed    = colorMod ;
                * vGrn    = colorMod ;
                * vBlu    = colorMod ;
                * vAlp    = alphaMod ;
                vPos->x   = ( pclPos.x + pclRight.x + pclUp.x ) ;
                vPos->y   = ( pclPos.y + pclRight.y + pclUp.y ) ;
                vPos->z   = ( pclPos.z + pclRight.z + pclUp.z ) ;

                INCREMENT( Vec3 *  , vPos , mVertStride ) ;
                INCREMENT( float * , vTex , mVertStride ) ;
                INCREMENT( UINT8 * , vRed , mVertStride ) ;
                INCREMENT( UINT8 * , vGrn , mVertStride ) ;
                INCREMENT( UINT8 * , vBlu , mVertStride ) ;
                INCREMENT( UINT8 * , vAlp , mVertStride ) ;

                vTex[ 0 ] = 0.0f ;
                vTex[ 1 ] = v0 ;
                * vRed    = colorMod ;
                * vGrn    = colorMod ;
                * vBlu    = colorMod ;
                * vAlp    = alphaMod ;
                vPos->x   = ( pclPos.x - pclRight.x + pclUp.x ) ;
                vPos->y   = ( pclPos.y - pclRight.y + pclUp.y ) ;
                vPos->z   = ( pclPos.z - pclRight.z + pclUp.z ) ;

                INCREMENT( Vec3 *  , vPos , mVertStride ) ;
                INCREMENT( float * , vTex , mVertStride ) ;
                INCREMENT( UINT8 * , vRed , mVertStride ) ;
                INCREMENT( UINT8 * , vGrn , mVertStride ) ;
                INCREMENT( UINT8 * , vBlu , mVertStride ) ;
                INCREMENT( UINT8 * , vAlp , mVertStride ) ;

                vTex[ 0 ] = 0.0f ;
                vTex[ 1 ] = v1 ;
                * vRed    = colorMod ;
                * vGrn    = colorMod ;
                * vBlu    = colorMod ;
                * vAlp    = alphaMod ;
                vPos->x   = ( pclPos.x - pclRight.x - pclUp.x ) ;
                vPos->y   = ( pclPos.y - pclRight.y - pclUp.y ) ;
                vPos->z   = ( pclPos.z - pclRight.z - pclUp.z ) ;

                INCREMENT( Vec3 *  , vPos , mVertStride ) ;
                INCREMENT( float * , vTex , mVertStride ) ;
                INCREMENT( UINT8 * , vRed , mVertStride ) ;
                INCREMENT( UINT8 * , vGrn , mVertStride ) ;
                INCREMENT( UINT8 * , vBlu , mVertStride ) ;
                INCREMENT( UINT8 * , vAlp , mVertStride ) ;

                vTex[ 0 ] = 1.0f ;
                vTex[ 1 ] = v1 ;
                * vRed    = colorMod ;
                * vGrn    = colorMod ;
                * vBlu    = colorMod ;
                * vAlp    = alphaMod ;
                vPos->x   = ( pclPos.x + pclRight.x - pclUp.x ) ;
                vPos->y   = ( pclPos.y + pclRight.y - pclUp.y ) ;
                vPos->z   = ( pclPos.z + pclRight.z - pclUp.z ) ;
            }
        }




        void ParticlesRenderModel::VertexBufferFillerGeneric::SetPos3FCol4BTex2F()
        {
            mOffsetToDensityInfo               = offsetof( Particle , mDensity ) ;
            mVertStride                        = sizeof  ( VertexFormatPos3Col4Tex2 ) ;
            mVertPositionOffset                = offsetof( VertexFormatPos3Col4Tex2 , px       ) ;
            mVertTextureCoordOffset            = offsetof( VertexFormatPos3Col4Tex2 , ts       ) ;
            mVertRedOffset                     = offsetof( VertexFormatPos3Col4Tex2 , crgba[0] ) ;
            mVertGrnOffset                     = offsetof( VertexFormatPos3Col4Tex2 , crgba[1] ) ;
            mVertBluOffset                     = offsetof( VertexFormatPos3Col4Tex2 , crgba[2] ) ;
            mVertAlpOffset                     = offsetof( VertexFormatPos3Col4Tex2 , crgba[3] ) ;
            mUseDensityForTextureCoordinate    = false ;
            mDensityVisibility                 = FLT_MAX ;
            mBlendMode                         = PeGaSys::ParticlesRender::ParticlesRenderModel::VertexBufferFillerGeneric::BLEND_MODE_ALPHA ;
            mScale                             = 1.0f ;
        }




        // ParticlesRenderModel ------------------------------------------------------------

        ParticlesRenderModel::ParticlesRenderModel( Render::ISceneManager * sceneManager )
            : ModelNode( sceneManager )
            , mParticleSystem( NULLPTR )
            , mVertexBufferFillerContainers( NULLPTR )
        {
        }




        ParticlesRenderModel::~ParticlesRenderModel()
        {
        }




        void ParticlesRenderModel::Render()
        {
            ASSERT( mParticleSystem != NULLPTR ) ;  // Make sure ParticleSystem was assigned to this model.

            SetLocalToWorld() ;

            const Camera *  camera                      = GetSceneManager()->GetCurrentCamera() ;
            ApiBase *       renderApi                   = GetSceneManager()->GetApi() ;
            const double    currentVirtualTimeInSeconds = GetSceneManager()->GetCurrentVirtualTimeInSeconds() ;

            renderApi->SetLights( * this ) ;
            renderApi->SetCamera( * camera ) ;
            renderApi->SetLocalToWorld( GetLocalToWorld() ) ;

            // Create and fill vertex buffer with geometry for particles.
            CreateAndFillVertexBufferPerGroup( currentVirtualTimeInSeconds ) ;

            // Render particles geometry.
            GetModelData()->Render( renderApi ) ;

            // TODO: Render diagnostic text at this location (0,0,0)

            RenderChildren() ;
        }




        /** Count total number of vertex buffer fillers associated with this particle system.

            The particle system can have multiple particle groups.
            Each particle group can have multiple meshes.
            This routine counts the number of meshes implied by mVertexBufferFillerContainers.

            After the meshes are allocated, one can determine the number of meshes from GetModelData()->GetNumMeshes()

        */
        size_t ParticlesRenderModel::CountNumVertBufFillers() const
        {
            ASSERT( mVertexBufferFillerContainers != NULLPTR ) ; // Must have called AssociateWithParticleSystem beforehand.
            ASSERT( mVertexBufferFillerContainers->Size() == mParticleSystem->GetNumGroups() ) ; // Must have exactly one vbFillerContainer per particle group.
            const size_t numGroups = mVertexBufferFillerContainers->Size() ;
            size_t numVertBufFillers = 0 ;
            for( size_t groupIndex = 0 ; groupIndex < numGroups ; ++ groupIndex )
            {   // For each particle group (each of which has its own vbFiller container)...
                const PclGrpVertexBufferFillerContainer & vbFillers = mVertexBufferFillerContainers->operator[]( groupIndex ) ;
                numVertBufFillers += vbFillers.Size() ;
                ASSERT( vbFillers.Size() > 0 ) ;
            }
            return numVertBufFillers ;
        }




        size_t ParticlesRenderModel::GetNumVertBufFillersPerGroup( size_t groupIndex ) const
        {
            ASSERT( mVertexBufferFillerContainers != NULLPTR ) ; // Must have called AssociateWithParticleSystem beforehand.
            ASSERT( mVertexBufferFillerContainers->Size() == mParticleSystem->GetNumGroups() ) ; // Must have exactly one vbFillerContainer per particle group.
            ASSERT( groupIndex < mParticleSystem->GetNumGroups() ) ;

            const PclGrpVertexBufferFillerContainer & vbFillers = (*mVertexBufferFillerContainers)[ groupIndex ] ;

            const size_t numMeshesForGroup = vbFillers.Size() ;

            return numMeshesForGroup ;
        }




        size_t ParticlesRenderModel::GetInternalMeshIndex( size_t groupIndex , size_t meshIndexWithinGroup ) const
        {
            ASSERT( mVertexBufferFillerContainers != NULLPTR ) ; // Must have called AssociateWithParticleSystem beforehand.
            ASSERT( mVertexBufferFillerContainers->Size() == mParticleSystem->GetNumGroups() ) ; // Must have exactly one vbFillerContainer per particle group.
            ASSERT( groupIndex < mParticleSystem->GetNumGroups() ) ;

            size_t numMeshes = 0 ;
            for( size_t grpIdx = 0 ; grpIdx < groupIndex ; ++ grpIdx )
            {   // For each particle group (each of which has its own vbFiller container)...
                numMeshes += GetNumVertBufFillersPerGroup( grpIdx ) ;
                ASSERT( GetNumVertBufFillersPerGroup( grpIdx ) > 0 ) ;
            }
            size_t meshIndex = numMeshes + meshIndexWithinGroup ;
            ASSERT( meshIndex >= groupIndex ) ; // Paranoid sanity check
            ASSERT( meshIndex < GetModelData()->GetNumMeshes() ) ;
            return meshIndex ;
        }




        /** Get render mesh object associated with given particle group.

            Useful for assigning materials for the mesh associated with each particle group.
        */
        Render::MeshBase * ParticlesRenderModel::GetMesh( size_t groupIndex , size_t meshIndexWithinGroup )
        {
            ASSERT( mParticleSystem != NULLPTR ) ;  // Must have called AssociateWithParticleSystem beforehand
            ASSERT( GetModelData() != NULLPTR ) ;  // Must have called InitializeResources beforehand
            ASSERT( groupIndex < mParticleSystem->GetNumGroups() ) ;
            const size_t meshIndex = GetInternalMeshIndex( groupIndex , meshIndexWithinGroup ) ;
            return GetModelData()->GetMesh( meshIndex ) ;
        }




        /** Initialize rendering resources (e.g. meshes) for each group in the associated particle system.

            Afterwards, elsewhere, caller should declare vertex formats and create and assign materials to each mesh.
        */
        void ParticlesRenderModel::InitializeResources()
        {
            ASSERT( mParticleSystem != NULLPTR )               ; // Must have called AssociateWithParticleSystem beforehand
            ASSERT( mVertexBufferFillerContainers != NULLPTR ) ; // Must have provided VB fillers.
            ASSERT( NULLPTR == GetModelData() )                ; // This model must not yet have model data.

            Render::ApiBase   * renderApi = GetSceneManager()->GetApi() ;
            Render::ModelData * modelData = NewModelData() ;

            const size_t numMeshes = CountNumVertBufFillers() ;

            //modelData->ReserveMeshes( numMeshes ) ;

            // Create a mesh per vbFiller for this group.
            for( size_t meshIndex = 0 ; meshIndex < numMeshes ; ++ meshIndex )
            {   // For each mesh to associate with this system...
                // Create associated mesh.
                Render::MeshBase *  mesh = modelData->NewMesh( renderApi ) ;
                (void) mesh ;   // Variable used only for debugging / inspection.
            }
        }




        /** Set address of associated ParticleSystem object so this model can rebuild geometry from particle data, upon each call to Render.

            This also triggers initialization, including creating various rendering resources.

            After calling this, caller must iterate through meshes and create vertex buffers to declare vertex formats
            and assign materials, one per particle group.

        */
        void ParticlesRenderModel::AssociateWithParticleSystem( ParticleSystem * particleSystem , PclSysVertexBufferFillerContainers * vertexBufferFillers )
        {
            ASSERT( NULLPTR == mParticleSystem ) ;  // Only allowed to set particle system once because this performs one-time initialization.

            mParticleSystem      = particleSystem ;
            mVertexBufferFillerContainers = vertexBufferFillers ;

            InitializeResources() ;
        }




        /** Allocate meshes per particle group for this particle system.

            Use vertexFormat associated with mesh to determine rendering.

        */
        void ParticlesRenderModel::CreateVertexBuffersPerGroup()
        {
            ASSERT( mParticleSystem != NULLPTR ) ;  // Must have called AssociateWithParticleSystem beforehand
            ASSERT( GetModelData() != NULLPTR ) ;  // Must have called InitializeResources beforehand

            static const size_t numVerticesPerParticle = 4 ; // A quad has 4 vertices.

            size_t groupIndex = 0 ;
            for( ParticleSystem::Iterator iter = mParticleSystem->Begin() ; iter != mParticleSystem->End() ; ++ iter , ++ groupIndex )
            {   // For each group in the particle system associated with this model...

                ParticleGroup *     group           = * iter ;
                ASSERT( mParticleSystem->GetGroup( groupIndex ) == group ) ;
                const size_t        numParticles    = group->GetNumParticles() ;
                const size_t        numVertices     = numParticles * numVerticesPerParticle ;

                for( size_t meshIndexWithinGroup = 0 ; meshIndexWithinGroup < GetNumVertBufFillersPerGroup( groupIndex ) ; ++ meshIndexWithinGroup )
                {   // For each mesh associated with this group...
                    MeshBase *          mesh            = GetMesh( groupIndex , meshIndexWithinGroup ) ;
                    VertexBufferBase *  vertBuf         = mesh->GetVertexBuffer() ;
                    ASSERT( vertBuf ) ; // Caller must have created vertex buffer (even if empty) to declare vertex format.
                    const size_t        vertBufCapacity = vertBuf->GetCapacity() ;

                    if( numVertices > vertBufCapacity )
                    {   // Vertex buffer cannot accommodate all particles in group.
                        vertBuf->ChangeCapacityAndReallocate( numVertices ) ;
                    }
                }
            }
        }




        void ParticlesRenderModel::AssignIndicesAndSortParticles( const Vec3 & viewForward , size_t numParticles )
        {
            ASSERT( mParticleSystem != NULLPTR ) ;  // Must have called AssociateWithParticleSystem beforehand
            ASSERT( GetModelData() != NULLPTR ) ;  // Must have called InitializeResources beforehand
            (void) viewForward , numParticles ;
        }




        void ParticlesRenderModel::FillVertexBufferPerGroup( const double & timeNow , const struct Mat44 & viewMatrix )
        {
            ASSERT( mParticleSystem != NULLPTR ) ;  // Must have called AssociateWithParticleSystem beforehand
            ASSERT( GetModelData() != NULLPTR ) ;  // Must have called InitializeResources beforehand

            const size_t numGroups = mParticleSystem->GetNumGroups() ;
            static const nvpp      = 4 ;

            for( size_t groupIndex = 0 ; groupIndex < numGroups ; ++ groupIndex )
            {   // For each group in the particle system associated with this model...
                ParticleGroup *     group           = mParticleSystem->GetGroup( groupIndex ) ;
                const size_t        numParticles    = group->GetNumParticles() ;

                const size_t        numVertices     = nvpp * numParticles ;

                for( size_t meshIndexWithinGroup = 0 ; meshIndexWithinGroup < GetNumVertBufFillersPerGroup( groupIndex ) ; ++ meshIndexWithinGroup )
                {   // For each mesh associated with this particle group...
                    MeshBase *          mesh            = GetMesh( groupIndex , meshIndexWithinGroup ) ;
                    ASSERT( mesh != NULLPTR ) ;
                    VertexBufferBase *  vertBuf         = mesh->GetVertexBuffer() ;
                    ASSERT( vertBuf != NULLPTR ) ;

                    unsigned char *     vertBytes       = reinterpret_cast< unsigned char * >( vertBuf->LockVertexData() ) ;   // Obtain lock on vertex buffer's data
                    ASSERT( ( 0 == numParticles ) || ( vertBytes != NULLPTR ) ) ;

                    {
                        DEBUG_ONLY( const size_t    vertBufCapacity = vertBuf->GetCapacity() ) ;
                        DEBUG_ONLY( const size_t    maxVertIdx      = nvpp * numParticles ) ;
                        ASSERT( vertBufCapacity >= maxVertIdx ) ;
                    }

                    // TODO: Instead of rendering quads, provide a separate index buffer full of triangles.  D3D does not support quads anyway (except on Xbox360).
                    mesh->SetPrimitiveType( PRIMITIVE_QUADS ) ;

                    if( vertBytes != NULLPTR  )
                    {   // This group has a vertex buffer, implying it has particles and lock was acquired.
                        const ParticleGroup * particleGroup = mParticleSystem->GetGroup( groupIndex ) ;
                        ASSERT( VertexDeclaration::POSITION_COLOR_TEXTURE == mesh->GetVertexBuffer()->GetVertexDeclaration().GetVertexFormat() ) ;
                        ASSERT( mVertexBufferFillerContainers != NULLPTR ) ;

                        {   // Caller provided callbacks to fill vertex buffer from particle groups.
                            ASSERT( mVertexBufferFillerContainers->Size() == numGroups ) ;
                            PclGrpVertexBufferFillerContainer & vbfContainerPerGroup = (*mVertexBufferFillerContainers)[ groupIndex ] ;

                            IVertexBufferFiller * vbFiller = vbfContainerPerGroup[ meshIndexWithinGroup ] ;
                            ASSERT( vbFiller ) ;
                            // Invoke callback to fill vertex buffer from particles in this group.
                            if( vbFiller->mIsActive )
                            {   // Current VB filler is active so fill the VB!

#                           if USE_TBB
                                // Estimate grain size based on size of problem and number of processors.
                                const size_t grainSize =  Max2( size_t( 1 ) , numParticles / gNumberOfProcessors ) ;
                                // Fill vertex buffer using threading building blocks
                                parallel_for( tbb::blocked_range<size_t>( 0 , numParticles , grainSize ) , ParticleRenderer_FillVertexBuffer_TBB( vbFiller , vertBytes , particleGroup , timeNow , viewMatrix ) ) ;
#                           else
                                (*vbFiller)( vertBytes , particleGroup , timeNow , viewMatrix , 0 , numParticles ) ;
#                           endif

                                vertBuf->SetPopulation( numVertices ) ;
                            }
                            else
                            {   // Current VB filler is not active so (effectively) empty the VB.
                                vertBuf->SetPopulation( 0 ) ;
                            }
                        }
                        vertBuf->UnlockVertexData() ;
                    }
                }
            }
        }




        void ParticlesRenderModel::CreateAndFillVertexBufferPerGroup( const double & timeNow )
        {
            ASSERT( mParticleSystem != NULLPTR ) ;  // Must have called AssociateWithParticleSystem beforehand
            ASSERT( GetModelData() != NULLPTR ) ;  // Must have called InitializeResources beforehand

            CreateVertexBuffersPerGroup() ;

            ApiBase *   renderApi   = GetSceneManager()->GetApi() ;
            RenderState renderState ;
            renderApi->GetRenderState( renderState ) ;
            Mat44 & viewMatrix = renderState.mTransforms.mViewMatrix ;

#           if SORT_PARTICLES
            {
                // Extract world space direction vectors associated with view (used to compute camera-facing coordinates).
                // Note that these vectors are the unit vectors of the inverse of the view matrix.
                // They are the world-space unit vectors of the view transformation.
                Vec3 viewRight  ( viewMatrix.m[0][0] , viewMatrix.m[1][0] , viewMatrix.m[2][0] ) ;
                Vec3 viewUp     ( viewMatrix.m[0][1] , viewMatrix.m[1][1] , viewMatrix.m[2][1] ) ;
                Vec3 viewForward( viewMatrix.m[0][2] , viewMatrix.m[1][2] , viewMatrix.m[2][2] ) ;

                AssignIndicesAndSortParticles( viewForward , numParticles ) ;
            }
#           endif

            FillVertexBufferPerGroup( timeNow , viewMatrix ) ;
        }


    }
}
