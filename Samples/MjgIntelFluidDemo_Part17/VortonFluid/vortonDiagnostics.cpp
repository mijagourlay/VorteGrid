/*! \file vorton.cpp

    \brief Vortex particle

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Math/vec3.h"

#include "vorton.h"




/* static */ void Vorton::UnitTest( void )
{
    DEBUG_ONLY( fprintf( stderr , "Vorton::UnitTest------------------------\n" ) ) ;

    // Test velocity-from-vorticity and vorticity-from-velocity.
    static const float  radiusVort          = 0.125f ;
    static const        numPointsPerSide    = 4 ;  // Make sure this is even so (0,0,0) is never touched
    static const float  gridSize            = 2.0f * radiusVort * float( numPointsPerSide ) ;
    for( int iz = 0 ; iz < numPointsPerSide ; ++ iz )
    {
        const float zv = gridSize * ( float( iz ) / float( numPointsPerSide - 1 ) - 0.5f ) ;
        for( int iy = 0 ; iy < numPointsPerSide ; ++ iy )
        {
            const float yv = gridSize * ( float( iy ) / float( numPointsPerSide - 1 ) - 0.5f ) ;
            for( int ix = 0 ; ix < numPointsPerSide ; ++ ix )
            {
                const float xv = gridSize * ( float( ix ) / float( numPointsPerSide - 1 ) - 0.5f ) ;

                for( int jz = 0 ; jz < numPointsPerSide ; ++ jz )
                {
                    const float zq = gridSize * ( float( jz ) / float( numPointsPerSide - 1 ) - 0.5f ) ;
                    for( int jy = 0 ; jy < numPointsPerSide ; ++ jy )
                    {
                        const float yq = gridSize * ( float( jy ) / float( numPointsPerSide - 1 ) - 0.5f ) ;
                        for( int jx = 0 ; jx < numPointsPerSide ; ++ jx )
                        {
                            const float xq = gridSize * ( float( jx ) / float( numPointsPerSide - 1 ) - 0.5f ) ;

                            for( int kz = 0 ; kz < numPointsPerSide ; ++ kz )
                            {
                                const float zw = gridSize * ( float( kz ) / float( numPointsPerSide - 1 ) - 0.5f ) ;
                                for( int ky = 0 ; ky < numPointsPerSide ; ++ ky )
                                {
                                    const float yw = gridSize * ( float( ky ) / float( numPointsPerSide - 1 ) - 0.5f ) ;
                                    for( int kx = 0 ; kx < numPointsPerSide ; ++ kx )
                                    {
                                        const float xw = gridSize * ( float( kx ) / float( numPointsPerSide - 1 ) - 0.5f ) ;

                                        // Create a vorton to induce a velocity field.
                                        const Vec3      positionOfVorton( Vec3( xv , yv , zv ) + Vec3(1,1,1) * radiusVort ) ;
                                        const Vorton    vort0( positionOfVorton , Vec3( xw , yw , zw ) , radiusVort ) ;
                                        // Compute the velocity induced by that vorton.
                                        const Vec3      posQuery( xq , yq , zq ) ;
                                        Vec3            velDueToVort0( 0.0f , 0.0f , 0.0f ) ;
                                        ASSERT( FAbs( vort0.mVorticity * velDueToVort0 ) == 0.0f ) ; // vorticity and velocity should be orthogonal
                                        vort0.AccumulateVelocity( velDueToVort0 , posQuery ) ;
                                        if( velDueToVort0.Mag2() > FLT_MIN )
                                        {   // Velocity induced by vort0 is significant.
                                            ASSERT( FAbs( ( positionOfVorton - posQuery ) * velDueToVort0 ) < 1.0e-4f ) ; // position and velocity should be orthogonal, otherwise AssignByVorticity is undefined.
                                            // Create a new vorton at the same position as vort0.
                                            Vorton          vortTest( positionOfVorton , Vec3( 0.0f , 0.0f , 0.0f ) , radiusVort ) ;
                                            // Calculate a vorticity for the new vorton such that it
                                            // induces the same velocity at the same query position.
                                            // Note that this will not, in general, yield the same vorticity as vort0,
                                            // but it should yield the same induced velocity at the query position.
                                            vortTest.AssignByVelocity( posQuery , velDueToVort0 ) ;
                                            // Calculate velocity induced by "test vorton"
                                            Vec3 velDueToVortTest( 0.0f , 0.0f , 0.0f ) ;
                                            vortTest.AccumulateVelocity( velDueToVortTest , posQuery ) ;
                                            // Velocity due to each vorton should match.
                                            ASSERT( velDueToVortTest.Resembles( velDueToVort0 ) ) ;
                                            SET_BREAKPOINT_HERE ;
                                        }

                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Vector<Vorton> vortons ;

    DEBUG_ONLY( fprintf( stderr , "Vorton::UnitTest END ------------------------\n" ) ) ;
}