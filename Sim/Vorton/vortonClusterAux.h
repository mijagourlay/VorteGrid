/*! \file VortonClusterAux.h

    \brief Auxiliary information used to calculate a representation of a cluster of vortons

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef VORTON_CLUSTER_AUX_H
#define VORTON_CLUSTER_AUX_H

#include <math.h>

#include "Core/Math/vec3.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Auxiliary information used to calculate a representation of a cluster of vortons

    In this fluid simulation, a "vorton cluster" refers to a collection of vortons
    which are all inside the same region.

    Each vorton influences the motion of all other vortons.

    This fluid simulation approximates the influence of vorton clusters
    by representing them in a simpler form, i.e. a form which has
    fewer parameters and less computational complexity than expressing
    the influence of all individual vortons in the cluster.

    This structure is used to store information used to compute parameters
    of that simplified form.  This structure is NOT used to represent
    the simplified form itself, during the influence calculation.

*/
class VortonClusterAux
{
    public:
        /*! \brief Construct auxiliary info for aggregating vortons

            \note Initial values for mVortNormSum is initially almost 0
                    because it accumulates the sum of |vorticity|.
                    mVortNormSum is not, however, exactly 0; instead
                    it is FLT_MIN (the smallest value a float can
                    represent) in order to avoid divide-by-zero.
                    The only time when mVortNormSum would be 0
                    is when a grid cluster contains no vortons,
                    in which case the center-of-vorticity is undefined.

        */
        VortonClusterAux() : mVortNormSum( FLT_MIN ) { }

        float	mVortNormSum    ;   ///< Sum of vorticity magnitude for cluster.  Used to normalize center-of-vorticity.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
