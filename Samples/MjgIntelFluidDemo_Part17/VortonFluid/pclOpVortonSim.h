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
#ifndef PCL_OP_VORTON_SIM_H
#define PCL_OP_VORTON_SIM_H

#include "vortonSim.h"
#include "Particles/particleSystem.h"

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
        {
        }

        PclOpVortonSim( const PclOpVortonSim & that )
            : mMinCorner( 0 )
            , mMaxCorner( 0 )
        {
            this->operator=( that ) ;
        }

        PclOpVortonSim & operator=( const PclOpVortonSim & that ) ;

        VIRTUAL_CONSTRUCTORS( PclOpVortonSim ) ;

        void Operate(  VECTOR< Particle > & particles , float timeStep , unsigned uFrame ) ;

        VortonSim       mVortonSim      ;   ///< Vorton fluid simulator
        const Vec3 *    mMinCorner      ;   ///< Minimal corner to generate velocity grid
        const Vec3 *    mMaxCorner      ;   ///< Maximal corner to generate velocity grid
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
