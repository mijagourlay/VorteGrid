/*! \file particleRenderer.h

    \brief Class to render particles

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_RENDERER_H
#define PARTICLE_RENDERER_H

#include "useTbb.h"

#define USE_FANCY_PARTICLES 0

#define OFFSET_OF_MEMBER( rObject , rMember ) (((char*)(&rObject.rMember))-((char*)&rObject))

/*! \brief Class to render vortex particles
*/
class ParticleRenderer
{
    public:
        struct ParticleIndex
        {
            int     mPcl    ;   ///< index into particle arrays
            float   mDepth  ;   ///< distance along view forward direction
        } ;

        ParticleRenderer( const char * pParticleData , size_t stride , size_t offsetToAngVel , size_t offsetToSize ) ;
        ~ParticleRenderer() ;

        void Render( double timeNow , float timeChange , size_t numParticles ) ;

        /*! \brief Set address of particle data
            \note When a dynamic array stores particle data, the address can change each frame
        */
        void SetParticleData( const char * pParticleData ) { mParticleData = pParticleData ; }

    private:
        const char *                mParticleData           ;   ///< Dynamic array of particles
        size_t                      mStride                 ;   ///< Number of bytes between particles
        size_t                      mOffsetToAngVel         ;   ///< Number of bytes to angular velocity
        size_t                      mOffsetToSize           ;   ///< Number of bytes to size
        unsigned char *             mVertexBuffer           ;   ///< Address of vertex buffer
        size_t                      mVertexBufferCapacity   ;   ///< number of vertices this buffer can hold
        ParticleIndex       *       mIndices                ;   ///< buffer used to sort particles
        size_t                      mIndicesCapacity        ;   ///< number of elements in mIndices

        ParticleRenderer( const ParticleRenderer & re) ;                // Disallow copy construction.  See comments in AttributedOld.
        ParticleRenderer & operator=( const ParticleRenderer & re ) ;   // Disallow assignment  See comments in AttributedOld.

        void FillVertexBufferSlice( const double & timeNow , const struct Mat4 & viewMatrix , size_t iPclStart , size_t iPclEnd ) ;

    #if USE_TBB
        friend class ParticleRenderer_FillVertexBuffer_TBB ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
