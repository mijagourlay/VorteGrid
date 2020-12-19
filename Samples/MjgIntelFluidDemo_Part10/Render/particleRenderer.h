/*! \file particleRenderer.h

    \brief Class to render particles

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_RENDERER_H
#define PARTICLE_RENDERER_H

#include "useTbb.h"

// "Fancy" particles are lit, shaded particles that take a lot of extra time to render.
// They also require special lighting setup.
// Normally, this demo should leave this disabled.
#define USE_FANCY_PARTICLES 0

// Vertex buffer objects facilitate directly filling vertex buffers into GPU-owned memory
// bypassing an intermediate CPU-owned buffer.  VBO's are therefore much faster.
// But not all versions of OpenGL support VBO's.
// Support for VBO is detected and selected at runtime.
#define USE_VERTEX_BUFFER_OBJECT 1

// Enabling separate VBO's keeps particle vertex position and texture coordinates in separate buffers.
// This allows for faster vertex buffer filling.
// In this demo, support for separate VBO's is only implemented for VBO's,
// but technically using separate arrays could also be done with old-style vertex arrays.
#if USE_VERTEX_BUFFER_OBJECT
#define USE_SEPARATE_VBOS 1
#endif

// Macro to provide the offset, in bytes, of a member variable within a struct or class.
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

        ParticleRenderer( const char * pParticleData , size_t stride , size_t offsetToAngVel , size_t offsetToSize , size_t offsetToDensity ) ;
        ~ParticleRenderer() ;

        void Render( double timeNow , float timeChange , size_t numParticles ) ;

        /*! \brief Set address of particle data
            \note When a dynamic array stores particle data, the address can change each frame
        */
        void SetParticleData( const char * pParticleData ) { mParticleData = pParticleData ; }

    private:
        typedef unsigned int        COLOR_UBYTES                ;
        typedef unsigned char *     VERTEX_BUFFER_POINTER_TYPE  ;
        typedef WORD *              INDEX_BUFFER_POINTER_TYPE   ;

        const char *                mParticleData               ;   ///< Dynamic array of particles
        size_t                      mStride                     ;   ///< Number of bytes between particles
        size_t                      mOffsetToAngVel             ;   ///< Number of bytes to angular velocity
        size_t                      mOffsetToSize               ;   ///< Number of bytes to size
        size_t                      mOffsetToDensity            ;   ///< Number of bytes to density
        VERTEX_BUFFER_POINTER_TYPE  mVertexBuffer               ;   ///< Address of vertex buffer
        size_t                      mVertexBufferCapacity       ;   ///< number of vertices this buffer can hold
        ParticleIndex       *       mIndices                    ;   ///< buffer used to sort particles
        size_t                      mIndicesCapacity            ;   ///< number of elements in mIndices

    #if USE_SEPARATE_VBOS
        VERTEX_BUFFER_POINTER_TYPE  mTexCoordBuffer             ;   ///< Address of texture coordinate buffer
        bool                        mVertexBufferGrew           ;   ///< Did vertex buffer grow?
    #endif

    #if USE_VERTEX_BUFFER_OBJECT
        #if USE_SEPARATE_VBOS
            static const size_t NUM_VBO_MAX = 2 ;
        #else
            static const size_t NUM_VBO_MAX = 1 ;
        #endif
        GLuint                      mVboNames[ NUM_VBO_MAX ]    ;   ///< Identifer for vertex buffer object
    #endif

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
