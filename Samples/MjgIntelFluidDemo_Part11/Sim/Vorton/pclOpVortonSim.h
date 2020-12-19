/*! \file PclOpVortonSim.h

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PCL_OP_VORTON_SIM_H
#define PCL_OP_VORTON_SIM_H

#include "vortonSim.h"
#include "Sim/Particle/particleSystem.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Particle operation to compute a velocity grid from a collection of vortons

    This vorton simulator reuses Particle objects to represent
    vortons (tiny vortex elements), so it is possible to operate
    on vortons using any particle operator.

*/
class PclOpVortonSim : public IParticleOperation
{
    public:
        PclOpVortonSim()
            : mMinCorner( 0 )
            , mMaxCorner( 0 )
        {}

        /** Update the fluid and rigid bodies.

            \param particles Dynamic array of vortex particles on which to operate.

            \param timeStep Change in virtual time since last update.

            \param uFrame   Frame counter.

        */
        void Operate(  Vector< Particle > & particles , float timeStep , unsigned uFrame )
        {
            (void) particles ; // Avoid "unreferenced formal parameter" warning.

            QUERY_PERFORMANCE_ENTER ;
            mVortonSim.FindBoundingBox() ;
            QUERY_PERFORMANCE_EXIT( PclOpVortonSim_Operate_FindBoundingBoxVortons ) ;

            QUERY_PERFORMANCE_ENTER ;
            mVortonSim.UpdateBoundingBox( * mMinCorner , * mMaxCorner , true ) ;
            QUERY_PERFORMANCE_EXIT( PclOpVortonSim_Operate_UpdateBoundingBox ) ;

            QUERY_PERFORMANCE_ENTER ;
            mVortonSim.Update( timeStep , uFrame ) ;
            QUERY_PERFORMANCE_EXIT( PclOpVortonSim_Operate_VortonSim_Update ) ;
        }

        VortonSim       mVortonSim      ;   ///< Vorton fluid simulator
        const Vec3 *    mMinCorner      ;   ///< Minimal corner to generate velocity grid
        const Vec3 *    mMaxCorner      ;   ///< Maximal corner to generate velocity grid

    private:
        PclOpVortonSim( const PclOpVortonSim & ) ;
        PclOpVortonSim & operator=( const PclOpVortonSim & ) ;
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
