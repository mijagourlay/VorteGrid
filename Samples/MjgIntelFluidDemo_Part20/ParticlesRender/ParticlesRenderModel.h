/** \file particlesRenderModel.h

    \brief Model scene node for a particle system.

    \author Written and Copyright 2005-2014 MJG; All rights reserved. Contact me at mijagourlay.com for licensing.

*/
#ifndef PEGASYS_PARTICLES_RENDER_PARTICLES_RENDER_MODEL_H
#define PEGASYS_PARTICLES_RENDER_PARTICLES_RENDER_MODEL_H

#include "Core/Containers/IntrusivePtr.h"

#include "Core/Containers/vector.h"


#include "Render/Scene/model.h"
#include "Render/Scene/modelData.h"

#include "Particles/particleSystem.h"

// Macros ----------------------------------------------------------------------

#ifndef PRIVATE
#   define PRIVATE private
#endif

// Types -----------------------------------------------------------------------

class ParticleRenderer_FillVertexBuffer_TBB ;

namespace PeGaSys
{
    namespace Render
    {
        // Forward declarations
        class MeshBase  ;
        class Light     ;
    }

    namespace ParticlesRender
    {
        /** Scene node representing a ParticleSystem model.

            A ParticleSystem is composed of "groups".
            Each group has its own material and vertex buffer.
        */
        class ParticlesRenderModel : public Render::ModelNode
        {
        public:
            class IVertexBufferFiller
            {
                public:
                    IVertexBufferFiller() : mIsActive( true ) {}

                    /** Operation to fill vertex buffer from particles in group.
                    */
                    virtual void operator()( unsigned char * vertexBytes , const ParticleGroup * particleGroup , const double & timeNow , const struct Mat44 & viewMatrix , size_t iPclStart , size_t iPclEnd ) = 0 ;

                    bool mIsActive ; /// Whether this filler is currently active, i.e. whether it should fill a vertex buffer.  If inactive, the associated render pass probably also ought to be inactive.
            } ;


            class VertexBufferFillerGeneric : public IVertexBufferFiller
            {
                public:
                    enum BlendModeE
                    {
                        BLEND_MODE_ALPHA    ,
                        BLEND_MODE_ADDITIVE
                    } ;

                    VertexBufferFillerGeneric()
                        : mPclStride( sizeof( Particle ) )
                        , mOffsetToAngVel     ( offsetof( Particle , mAngularVelocity ) )
                        , mOffsetToSize       ( offsetof( Particle , mSize            ) )
                        , mOffsetToDensityInfo( offsetof( Particle , mDensity         ) )

                        , mVertStride( 0 )
                        , mVertPositionOffset( ~size_t(0) )
                        , mVertTextureCoordOffset( ~size_t(0) )
                        , mVertRedOffset( ~size_t(0) )
                        , mVertGrnOffset( ~size_t(0) )
                        , mVertBluOffset( ~size_t(0) )
                        , mVertAlpOffset( ~size_t(0) )

                        , mUseDensityForTextureCoordinate( false )
                        , mBlendMode( BLEND_MODE_ALPHA )
                        , mDensityVisibility( 1.0f )
                        , mScale( 1.0f )
                    {}

                    /** Operation to fill vertex buffer from particles in group.
                    */
                    virtual void operator()( unsigned char * vertexBytes , const ParticleGroup * particleGroup , const double & timeNow , const struct Mat44 & viewMatrix , size_t iPclStart , size_t iPclEnd ) ;

                    void SetPos3FCol4BTex2F() ;

                    size_t      mPclStride                      ;   /// Number of bytes between each particle in particle data.
                    size_t      mOffsetToAngVel                 ;   /// Number of bytes to angular velocity.
                    size_t      mOffsetToSize                   ;   /// Number of bytes to particle size.
                    size_t      mOffsetToDensityInfo            ;   /// Number of bytes to density info (either density or mass fraction)

                    size_t      mVertStride                     ;   /// Number of bytes between adjacent vertex positions
                    size_t      mVertPositionOffset             ;   /// Number of bytes between adjacent vertex positions
                    size_t      mVertTextureCoordOffset         ;   /// Number of bytes between adjacent vertex texture coordinates
                    size_t      mVertRedOffset                  ;   /// Number of bytes from color dword to red   channel
                    size_t      mVertGrnOffset                  ;   /// Number of bytes from color dword to green channel
                    size_t      mVertBluOffset                  ;   /// Number of bytes from color dword to blue  channel
                    size_t      mVertAlpOffset                  ;   /// Number of bytes from color dword to alpha channel

                    bool        mUseDensityForTextureCoordinate ;
                    BlendModeE  mBlendMode                      ;
                    float       mDensityVisibility              ;
                    float       mScale                          ;
            } ;

            static const unsigned sTypeId = 'PcRM' ; ///< Type identifier for a ParticlesRenderModel scene node

            explicit ParticlesRenderModel( Render::ISceneManager * sceneManager ) ;
            virtual ~ParticlesRenderModel() ;

            virtual void Render() ;

            /// Container of vertex buffer fillers for a particle group.
            /// Each particle group can have multiple meshes.  Each mesh has its own vertex buffer filler.
            typedef VECTOR< IVertexBufferFiller * >             PclGrpVertexBufferFillerContainer  ;

            /// Container of vertex buffer filler containers for a particle system.
            /// Each particle system can have multiple groups.  Each group has an associated vbFiller container.
            /// Indexing could occur like this: vbFillerContainers[ groupIndex ][ meshIndex ]
            typedef VECTOR< PclGrpVertexBufferFillerContainer > PclSysVertexBufferFillerContainers ;

            void AssociateWithParticleSystem( ParticleSystem * particleSystem , PclSysVertexBufferFillerContainers * vertexBufferFillers = NULLPTR ) ;

            size_t CountNumVertBufFillers() const ;
            size_t GetNumVertBufFillersPerGroup( size_t groupIndex ) const ;

            Render::MeshBase * GetMesh( size_t groupIndex , size_t meshIndexWithinGroup ) ;

        private:
            ParticlesRenderModel( const ParticlesRenderModel & ) ; // Disallow copy construction
            ParticlesRenderModel & operator=( const ParticlesRenderModel & ) ; // Disallow assignment

            void InitializeResources() ;

            size_t GetInternalMeshIndex( size_t groupIndex , size_t meshIndexWithinGroup ) const ;

            void CreateVertexBuffersPerGroup() ;
            void AssignIndicesAndSortParticles( const Vec3 & viewForward , size_t numParticles ) ;

            void FillVertexBufferPerGroup( const double & timeNow , const struct Mat44 & viewMatrix ) ;

            void CreateAndFillVertexBufferPerGroup( const double & timeNow ) ;

            ParticleSystem *    mParticleSystem                 ;   /// Particle system associated with this object.

            PclSysVertexBufferFillerContainers * mVertexBufferFillerContainers ; // Jagged multidimensional array of vertex buffer fillers

#       if USE_TBB
            friend class ParticleRenderer_FillVertexBuffer_TBB ;
#       endif

        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;
} ;

#endif
