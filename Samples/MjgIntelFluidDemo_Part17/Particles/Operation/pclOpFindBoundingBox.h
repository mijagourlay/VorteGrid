/** \file pclOpFindBoundingBox.h

    \brief Operation to find the bounding box of a dynamic array of particles.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_OPERATION_FIND_BOUNDING_BOX_H
#define PARTICLE_OPERATION_FIND_BOUNDING_BOX_H

#include "particleOperation.h"

/** Operation to find the bounding box of a dynamic array of particles.
*/
class PclOpFindBoundingBox : public IParticleOperation
{
    public:
        PclOpFindBoundingBox()
            : mMinCorner( FLT_MAX , FLT_MAX , FLT_MAX )
            , mMaxCorner( - mMinCorner )
        {}

        VIRTUAL_CONSTRUCTORS( PclOpFindBoundingBox ) ;

        void Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ ) ;

        /// Return the minimal corner of axis-aligned bounding box that contains all particles.
        const Vec3 & GetMinCorner( void ) const { return mMinCorner ; }

        /// Return the maximal corner of axis-aligned bounding box that contains all particles.
        const Vec3 & GetMaxCorner( void ) const { return mMaxCorner ; }

        static void FindBoundingBox( const VECTOR< Particle > & particles , Vec3 & minCorner , Vec3 & maxCorner ) ;

    private:
        Vec3    mMinCorner ;    ///< Minimal corner of axis-aligned bounding box containing all particles.
        Vec3    mMaxCorner ;    ///< Maximal corner of axis-aligned bounding box containing all particles.
} ;

#endif