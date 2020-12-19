/*! \file PclOpVortonSim.h

    \brief Particle operation to compute a velocity grid from a collection of vortons

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
        PclOpVortonSim( void )
            : mMinCorner( 0 )
            , mMaxCorner( 0 )
        {}

        /*! \brief Update the fluid and rigid bodies

            \param timeStep - change in virtual time since last update

            \param uFrame - frame counter

            \note This routine is a relic of an earlier implementation.
                    Most of the functionality that was in this routine
                    now resides in a generic particle system.
                    The remainder of this functionality is destined
                    to move into ParticleOperations.
                    Most of this module will evaporate.

        */
        void Operate( float timeStep , unsigned uFrame )
        {
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
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
