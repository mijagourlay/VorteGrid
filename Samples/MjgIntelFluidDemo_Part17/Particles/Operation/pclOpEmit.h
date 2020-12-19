/** \file pclOpEmit.h

    \brief Operation to emit particles.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_EMIT_H
#define PARTICLE_OPERATION_EMIT_H

#include "particleOperation.h"

/** Operation to emit particles.
*/
class PclOpEmit : public IParticleOperation
{
    public:
        PclOpEmit()
            : mEmitRate( 60.0f )
            , mRemainder( 0.0f )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpEmit ) ;

        void Operate( VECTOR< Particle > & particles , float timeStep , unsigned uFrame ) ;

        static void Emit( VECTOR< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , const VECTOR< Particle > * vortons ) ;

        Particle    mTemplate   ;   ///< Default values for new particle
        Particle    mSpread     ;   ///< Range of values for new particle
        float       mEmitRate   ;   ///< Particles per second to emit
        float       mRemainder  ;   ///< Fractional particles remaining.
} ;

#endif








////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Move code below to its own file.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////








/** \file pclOpSeedSurfaceTracers.h

    \brief Operation to place particles within a narrow band on either side of a surface.

    \author Copyright 2013-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/
#ifndef PARTICLE_OPERATION_RESAMPLE_SURFACE_H
#define PARTICLE_OPERATION_RESAMPLE_SURFACE_H

#include "Core/SpatialPartition/uniformGrid.h"

#include "particleOperation.h"

/** Operation to place particles within a narrow band on either side of a surface.
*/
class PclOpSeedSurfaceTracers : public IParticleOperation
{
    public:

        static const float sBandWidthAutomatic ;

        PclOpSeedSurfaceTracers()
            : mBandWidth( sBandWidthAutomatic )
            , mAmbientDensity( - FLT_MAX )
            , mSignedDistanceGrid( 0 )
            , mReferenceGrid( 0 )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpSeedSurfaceTracers ) ;

        void Operate( VECTOR< Particle > & particles , float timeStep , unsigned uFrame ) ;

        static void Emit( VECTOR< Particle > & particles , const int tracerCountMultiplier , const UniformGrid< float > & densityDeviationGrid , float regionNearSurface , const float ambientDensity ) ;
        static void Replace( VECTOR< Particle > & particles , UniformGrid< float > & signedDistanceGrid , const UniformGridGeometry & referenceGrid , float regionNearSurface , const float ambientDensity ) ;

        Particle                        mTemplate           ;   ///< Default values for new particle
        Particle                        mSpread             ;   ///< Range of values for new particle
        float                           mBandWidth          ;   ///< Width of band
        float                           mAmbientDensity     ;   ///< Density of fluid in the absence of particles.
        const UniformGrid< float > *    mSignedDistanceGrid ;   ///< Signed distance field
        const UniformGridGeometry *     mReferenceGrid      ;   ///< Grid geometry to use for basis for SDF grid.
} ;

extern void ComputeImmediateSDFFromDensity_Nearest( UniformGrid< float > & signedDistanceGrid , UniformGrid< int > & sdfPinnedGrid , const UniformGrid< float > & densityDeviationGrid , float densityOutside ) ;
extern void ComputeRemainingSDFFromImmediateSDF_Nearest( UniformGrid< float > & signedDistanceGrid , const UniformGrid< int > & sdfPinnedGrid ) ;
extern void ComputeSDF_Sphere( UniformGrid< float > & signedDistanceGrid ) ;

#endif
