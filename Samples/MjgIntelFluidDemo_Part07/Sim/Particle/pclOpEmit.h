/*! \file PclOpEmit.h

    \brief Particle operation to emit particles

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef PARTICLE_OPERATION_EMIT_H
#define PARTICLE_OPERATION_EMIT_H

#include "particleOperation.h"

/*! \brief Particle operation to emit particles
*/
class PclOpEmit : public IParticleOperation
{
    public:
        PclOpEmit( void )
            : mParticles( 0 )
            , mEmitRate( 60.0f )
            , mRemainder( 0.0f )
        {}

        void Operate( float timeStep , unsigned uFrame )
        {
            QUERY_PERFORMANCE_ENTER ;

            const float fNumToEmit = timeStep * mEmitRate + mRemainder ;
            const int   iNumToEmit = int( fNumToEmit ) ;

            // Remember fractional particles for future emission.
            mRemainder = fNumToEmit - float( iNumToEmit ) ;

            for( int iPcl = 0 ; iPcl < iNumToEmit ; ++ iPcl )
            {   // For each new particle to emit...
                mParticles->PushBack( mTemplate ) ;
                Particle & rParticleNew = mParticles->Back() ;
                rParticleNew.mPosition          += RandomSpread( mSpread.mPosition          ) ;
                rParticleNew.mVelocity          += RandomSpread( mSpread.mVelocity          ) ;
                rParticleNew.mOrientation       += RandomSpread( mSpread.mOrientation       ) ;
                rParticleNew.mAngularVelocity   += RandomSpread( mSpread.mAngularVelocity   ) ;
                rParticleNew.mMass              += RandomSpread( mSpread.mMass              ) ;
                rParticleNew.mSize              += RandomSpread( mSpread.mSize              ) ;
                rParticleNew.mBirthTime          = uFrame ;
            }

            QUERY_PERFORMANCE_EXIT( Particles_Emit ) ;
        }

        Vector< Particle > *    mParticles  ;   ///< Dynamic array of particles into which to emit new particles
        Particle                mTemplate   ;   ///< Default values for new particle
        Particle                mSpread     ;   ///< Range of values for new particle
        float                   mEmitRate   ;   ///< Particles per second to emit
        float                   mRemainder  ;   ///< Fractional particles remaining.
} ;

#endif