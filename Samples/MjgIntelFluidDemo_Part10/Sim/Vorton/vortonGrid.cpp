/*! \file vortonGrid.cpp

    \brief Utility routines for uniform grid of vortex particles

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Math/vec3.h"

#include "vortonGrid.h"




/*! \brief Compute the total circulation in a uniform grid of vortons

    \param vortonGrid - uniform grid of vortons

    \param vCirculation - Total circulation, the volume integral of vorticity, computed by this routine.

    \param vLinearImpulse - Volume integral of circulation weighted by position, computed by this routine.

*/
void VortonGrid_ConservedQuantities( const UniformGrid< Vorton > & vortonGrid , Vec3 & vCirculation , Vec3 & vLinearImpulse )
{
    vCirculation = vLinearImpulse = Vec3( 0.0f , 0.0f , 0.0f ) ;
    const size_t & numGridPoints = vortonGrid.Size() ;
    for( unsigned offset = 0 ; offset < numGridPoints ; ++ offset )
    {
        const Vorton &  rVorton         = vortonGrid[ offset ]  ;
        const float     volumeElement   = POW3( rVorton.GetRadius() ) * FourPiOver3 ;
        // Accumulate total circulation.
        vCirculation    += rVorton.GetVorticity() * volumeElement ;
        // Accumulate total linear impulse.
        vLinearImpulse  += rVorton.mPosition ^ rVorton.GetVorticity() * volumeElement ;
    }
}




#if 0
/*! \brief Extract just the vorticity from a vorton grid.

    \note This routine was used during prototyping, to extract
            just the vorticity member of a UniformGrid containing
            vortons, to createa UniformGrid containing just vorticity.
            This allowed other routines (for example, Poisson solver)
            to be written in a more general manor.
            At this point, however, this routine is no longer used.
            Instead, another routine populates vorticity
            directly frmo vortons, using a better, smoothing
            insertion technique.

            So basically, this is dead code.

*/
void VorticityFromVortonGrid( UniformGrid< Vec3 > & vorticityGrid , const UniformGrid< Vorton > & vortonGrid )
{
    const size_t numGridPoints = vortonGrid.Size() ;
    for( size_t offset = 0 ; offset < numGridPoints ; ++ offset )
    {
        const Vorton &  rVorton = vortonGrid[ offset ]  ;
        vorticityGrid[ offset ] = rVorton.GetVorticity() ;
    }
}
#endif
