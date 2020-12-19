/** \file pclOpKillAge.cpp

    \brief Operation to kill particles based on their age.

    \see http://www.mijagourlay.com/

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/Performance/perfBlock.h"

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Particles/particle.h"

#include "Particles/Operation/pclOpKillAge.h"




void PclOpKillAge::Operate(  VECTOR< Particle > & particles , float /* timeStep */ , unsigned uFrame )
{
    PERF_BLOCK( PclOpKillAge__Operate ) ;

	if( particles.Empty() )
	{
		return ;
	}
    const Particle * pPcls = & particles.operator[]( 0 ) ; // Modern STL supports vector.data() but not older versions like MSVS7.
    for( size_t iPcl = 0 ;
        iPcl < particles.Size() ; // Note: Size cannot be cached because loop body changes size.
        /* DO NOT ++ iPcl */ )      // Note: conditional increment below.
    {   // For each particle in the given array...
        const Particle & rPcl = pPcls[ iPcl ] ;
        const int age = uFrame - rPcl.mBirthTime ;
        if( age > mAgeMax )
        {
            Particles::Kill( particles , iPcl ) ;
        }
        else
        {
            ++ iPcl ;
        }
    }
}
