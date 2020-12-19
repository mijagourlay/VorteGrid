/*! \file vortonGrid.h

    \brief Utility routines for uniform grid of vortex particles

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
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

#endif
