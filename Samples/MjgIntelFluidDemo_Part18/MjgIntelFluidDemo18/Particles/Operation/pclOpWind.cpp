/** \file pclOpWind.cpp

    \brief Operation to push particles along a direction up to a maximum speed.

    \see http://www.mijagourlay.com/

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "Particles/Operation/pclOpWind.h"

#include "Particles/particle.h"

#include "Core/Performance/perfBlock.h"

#include <stdlib.h>


void PclOpWind::Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
{
    PERF_BLOCK( PclOpWind__Operate ) ;

    if( ( 0.0f == mWindWeight ) && ( 1.0f == mSrcWeight ) )
    {   // Weights imply doing nothing.
        return ;
    }

	if( particles.Empty() )
	{
		return ;
	}

    const size_t numParticles = particles.Size() ;
    Particle * pPcls = & particles[ 0 ] ;
    const Vec3 windTerm = mWindWeight * mWind ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {   // For each particle in the given array...
        Particle & rPcl = pPcls[ iPcl ] ;
        ASSERT( ! IsInf( rPcl.mPosition ) ) ;
        // Update particle velocity.
        rPcl.mVelocity = windTerm + mSrcWeight * rPcl.mVelocity ;
    }
}