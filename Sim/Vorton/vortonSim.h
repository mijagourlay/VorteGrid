/*! \file VortonSim.h

    \brief Dynamic simulation of a fluid, using tiny vortex elements.

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef VORTON_SIM_H
#define VORTON_SIM_H

#include <math.h>

#include "useTbb.h"

#include "Space/nestedGrid.h"
#include "vorton.h"
#include "particle.h"

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Dynamic simulation of a fluid, using tiny vortex elements.

    This implements a portion of a fluid simulation, and effectively
    neglects boundary conditions.  This module defers the enforcement
    of boundary conditions to another module.

    \see FluidBodySim

*/
class VortonSim
{
    public:
        /*! \brief Construct a vorton simulation
        */
        VortonSim( float viscosity = 0.0f , float density = 1.0f )
            : mMinCorner(FLT_MAX,FLT_MAX,FLT_MAX)
            , mMaxCorner( -mMinCorner )
            , mViscosity( viscosity )
            , mCirculationInitial( 0.0f , 0.0f , 0.0f )
            , mLinearImpulseInitial( 0.0f , 0.0f , 0.0f )
            , mAverageVorticity( 0.0f , 0.0f , 0.0f )
            , mFluidDensity( density )
            , mMassPerParticle( 0.0f )
        {}

        void                        Initialize( unsigned numTracersPerCellCubeRoot ) ;
              Vector< Vorton >  &   GetVortons( void )                  { return mVortons ; }
        const Vector< Vorton >  &   GetVortons( void ) const            { return mVortons ; }
              Vector< Particle > &  GetTracers( void )                  { return mTracers ; }
        const Vector< Particle > &  GetTracers( void ) const            { return mTracers ; }

        /*! \brief Kill the tracer at the given index
        */
        void                        KillTracer( size_t iTracer )
        {
            mTracers[ iTracer ] = mTracers[ mTracers.Size() - 1 ] ;
            mTracers.PopBack() ;
        }

        const Vec3 GetTracerCenterOfMass( void ) const ;

        const UniformGrid< Vec3 > & GetVelocityGrid( void ) const       { return mVelGrid ; }
        const float &               GetMassPerParticle( void ) const    { return mMassPerParticle ; }
        void                        Update( float timeStep , unsigned uFrame ) ;
        void                        Clear( void )
        {
            mVortons.Clear() ;
            mInfluenceTree.Clear() ;
            mVelGrid.Clear() ;
            mTracers.Clear() ;
        }

    private:
        void    AssignVortonsFromVorticity( UniformGrid< Vec3 > & vortGrid ) ;
        void    ConservedQuantities( Vec3 & vCirculation , Vec3 & vLinearImpulse ) const ;
        void    FindBoundingBox( void ) ;
        void    MakeBaseVortonGrid( void ) ;
        void    AggregateClusters( unsigned uParentLayer ) ;
        void    CreateInfluenceTree( void ) ;
        Vec3    ComputeVelocity( const Vec3 & vPosition , const unsigned idxParent[3] , size_t iLayer ) ;
        Vec3    ComputeVelocityBruteForce( const Vec3 & vPosition ) ;
        void    ComputeVelocityGridSlice( size_t izStart , size_t izEnd ) ;
        void    ComputeVelocityGrid( void ) ;
        void    StretchAndTiltVortons( const float & timeStep , const unsigned & uFrame ) ;
        void    ComputeAverageVorticity( void ) ;
        void    DiffuseVorticityGlobally( const float & timeStep , const unsigned & uFrame ) ;
        void    DiffuseVorticityPSE( const float & timeStep , const unsigned & uFrame ) ;
        void    AdvectVortons( const float & timeStep ) ;

        void    InitializePassiveTracers( unsigned multiplier ) ;
        void    AdvectTracersSlice( const float & timeStep , const unsigned & uFrame , size_t izStart , size_t izEnd ) ;
        void    AdvectTracers( const float & timeStep , const unsigned & uFrame ) ;

        Vector< Vorton >        mVortons                ;   ///< Dynamic array of tiny vortex elements
        NestedGrid< Vorton >    mInfluenceTree          ;   ///< Influence tree
        UniformGrid< Vec3 >     mVelGrid                ;   ///< Uniform grid of velocity values
        Vec3                    mMinCorner              ;   ///< Minimal corner of axis-aligned bounding box
        Vec3                    mMaxCorner              ;   ///< Maximal corner of axis-aligned bounding box
        float                   mViscosity              ;   ///< Viscosity.  Used to compute viscous diffusion.
        Vec3                    mCirculationInitial     ;   ///< Initial circulation, which should be conserved when viscosity is zero.
        Vec3                    mLinearImpulseInitial   ;   ///< Initial linear impulse, which should be conserved when viscosity is zero.
        Vec3                    mAverageVorticity       ;   ///< Hack, average vorticity used to compute a kind of viscous vortex diffusion.
        float                   mFluidDensity           ;   ///< Uniform density of fluid.
        float                   mMassPerParticle        ;   ///< Mass of each fluid particle (vorton or tracer).
        Vector< Particle >      mTracers                ;   ///< Passive tracer particles

    #if USE_TBB
        friend class VortonSim_ComputeVelocityGrid_TBB ;
        friend class VortonSim_AdvectTracers_TBB ;
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
