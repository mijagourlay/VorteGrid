/** \file PclOpVortonSim.h

    \brief Particle operation to compute a velocity grid from a collection of vortons

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-17/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

    \see http://www.mijagourlay.com/

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "pclOpVortonSim.h"

#include "Core/Performance/perfBlock.h"



/** \note   This routine cannot successfully duplicate mMinCorner or
            mMaxCorner, since they are references to externally owned
            objects.
            Completely cloning this object relies upon an external
            entity appropriately assigning mMinCorner and mMaxCorner.
*/
PclOpVortonSim & PclOpVortonSim::operator=( const PclOpVortonSim & that )
{
    mVortonSim = that.mVortonSim ;
    mMinCorner = 0 ;
    mMaxCorner = 0 ;

    return * this ;
}





/** Update the fluid and rigid bodies.

    \param particles Dynamic array of vortex particles on which to operate.

    \param timeStep Change in virtual time since last update.

    \param uFrame   Frame counter.

*/
void PclOpVortonSim::Operate(  VECTOR< Particle > & particles , float timeStep , unsigned uFrame )
{
    PERF_BLOCK( PclOpVortonSim__Operate ) ;

    //ASSERT( ( mVortonSim.GetVortons() == 0 ) || ( & particles == reinterpret_cast< VECTOR< Particle > * >( mVortonSim.GetVortons() ) ) ) ;
    mVortonSim.SetVortons( reinterpret_cast< VECTOR< Vorton > * >( & particles ) ) ;

    mVortonSim.FindBoundingBox() ;

    if( mMinCorner && mMaxCorner )
    {   // Additional min and max corners were set from an external source.
        mVortonSim.UpdateBoundingBox( * mMinCorner , * mMaxCorner , true ) ;
    }
    else
    {   // Need to finalize bounding box.
        const Vec3 pointInDomain = particles[ 0 ].mPosition ;
        mVortonSim.UpdateBoundingBox( pointInDomain , pointInDomain , true ) ;
    }

    mVortonSim.Update( timeStep , uFrame ) ;
}
