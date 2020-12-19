/** \file PclOpVortonSim.h

    \brief Particle operation to compute a velocity grid from a collection of vortons

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-14/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-15/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-16/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "pclOpVortonSim.h"

#include "Core/Performance/perf.h"



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
    //ASSERT( ( mVortonSim.GetVortons() == 0 ) || ( & particles == reinterpret_cast< VECTOR< Particle > * >( mVortonSim.GetVortons() ) ) ) ;
    mVortonSim.SetVortons( reinterpret_cast< VECTOR< Vorton > * >( & particles ) ) ;

    QUERY_PERFORMANCE_ENTER ;
    mVortonSim.FindBoundingBox() ;
    QUERY_PERFORMANCE_EXIT( PclOpVortonSim_Operate_FindBoundingBoxVortons ) ;

    QUERY_PERFORMANCE_ENTER ;
    if( mMinCorner && mMaxCorner )
    {   // Additional min and max corners were set from an external source.
        mVortonSim.UpdateBoundingBox( * mMinCorner , * mMaxCorner , true ) ;
    }
    else
    {   // Need to finalize bounding box.
        const Vec3 pointInDomain = particles[ 0 ].mPosition ;
        mVortonSim.UpdateBoundingBox( pointInDomain , pointInDomain , true ) ;
    }
    QUERY_PERFORMANCE_EXIT( PclOpVortonSim_Operate_UpdateBoundingBox ) ;

    QUERY_PERFORMANCE_ENTER ;
    mVortonSim.Update( timeStep , uFrame ) ;
    QUERY_PERFORMANCE_EXIT( PclOpVortonSim_Operate_VortonSim_Update ) ;
}
