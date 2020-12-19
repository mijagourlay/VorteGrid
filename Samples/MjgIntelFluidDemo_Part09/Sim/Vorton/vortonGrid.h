/*! \file vortonGrid.h

    \brief Utility routines for uniform grid of vortex particles

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef VORTON_GRID_H
#define VORTON_GRID_H

#include <math.h>

#include "wrapperMacros.h"
#include "Space/uniformGrid.h"
#include "vorton.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

extern void VortonGrid_ConservedQuantities( const UniformGrid< Vorton > & vortonGrid , Vec3 & vCirculation , Vec3 & vLinearImpulse ) ;
//extern void VorticityFromVortonGrid( UniformGrid< Vec3 > & vorticityGrid , const UniformGrid< Vorton > & vortonGrid ) ;

#endif
