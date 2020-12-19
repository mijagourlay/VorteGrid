/** \file pclOpWind.cpp

    \brief Operation to push particles along a direction up to a maximum speed.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/Performance/perf.h"

#include "../particle.h"

#include "pclOpWind.h"


void PclOpWind::Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned /* uFrame */ )
{
    if( ( 0.0f == mWindWeight ) && ( 1.0f == mSrcWeight ) )
    {   // Weights imply doing nothing.
        return ;
    }

	if( particles.empty() )
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