/*! \file VortonSim.h

    \brief Dynamic simulation of a fluid, using tiny vortex elements.

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
#ifndef VORTON_SIM_H
#define VORTON_SIM_H

#include <math.h>

#include "useTbb.h"

#include "Space/nestedGrid.h"
#include "vorton.h"

// Macros --------------------------------------------------------------

#define USE_MULTI_GRID 0

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
            : mMinCorner( FLT_MAX , FLT_MAX , FLT_MAX )
            , mMaxCorner( - mMinCorner )
            , mMinCornerEternal( FLT_MAX , FLT_MAX , FLT_MAX )
            , mMaxCornerEternal( - mMinCorner )
            , mViscosity( viscosity )
            , mCirculationInitial( 0.0f , 0.0f , 0.0f )
            , mLinearImpulseInitial( 0.0f , 0.0f , 0.0f )
            , mFluidDensity( density )
            , mVortonRadius( 0.0f )
        {}

        void                        Initialize( void ) ;
              Vector< Vorton >  &   GetVortons( void )                  { return mVortons ; }
        const Vector< Vorton >  &   GetVortons( void ) const            { return mVortons ; }

        const UniformGrid< Vec3 > & GetVelocityGrid( void ) const       { return mVelGrid ; }
        void                        SetViscosity( float viscosity )     { mViscosity = viscosity ; }
        const float &               GetViscosity( void ) const          { return mViscosity ; }
        void                        SetDensity( float density )         { mFluidDensity = density ; }
        const float &               GetDensity( void ) const            { return mFluidDensity ; }
        void                        FindBoundingBox( void ) ;
        void                        UpdateBoundingBox( const Vec3 & minCorner , const Vec3 & maxCorner , bool bFinal ) ;
        const UniformGridGeometry & GetGrid( void )                     { return mGridTemplate ; }
        const Vec3 &                GetMinCorner( void ) const          { return mMinCorner ; }
        const Vec3 &                GetMaxCorner( void ) const          { return mMaxCorner ; }
        void                        Update( float timeStep , unsigned uFrame ) ;



        /*! \brief Reset the simulation
        */
        void                        Clear( void )
        {
            mVortons.Clear() ;
            mInfluenceTree.Clear() ;
            mVorticityGrid.Clear() ;
            mVorticityMultiGrid.Clear() ;
            mVelGrid.Clear() ;
        }

        const Vec3 & GetMinCornerEternal( void ) const { return mMinCornerEternal ; }
        const Vec3 & GetMaxCornerEternal( void ) const { return mMaxCornerEternal ; }

    private:
        void        AssignVortonsFromVorticity( UniformGrid< Vec3 > & vortGrid ) ;
        void        ConservedQuantities( Vec3 & vCirculation , Vec3 & vLinearImpulse ) const ;

        // Integral-based velocity-from-vorticity routines
        void        MakeBaseVortonGrid( void ) ;
        void        AggregateClusters( unsigned uParentLayer ) ;
        void        CreateInfluenceTree( void ) ;
        Vec3        ComputeVelocityDirect( const Vec3 & vPosition ) ;
        Vec3        ComputeVelocityTree( const Vec3 & vPosition , const unsigned idxParent[3] , size_t iLayer ) ;
        void        ComputeVelocityGridSlice( size_t izStart , size_t izEnd ) ;

        // Differential-based velocity-from-vorticity routines
        void        PopulateNegativeVorticityGridFromVortons( UniformGrid< Vec3 > & vorticityGrid ) ;

        void        ComputeVelocityGrid( void ) ;
        void        StretchAndTiltVortons( const float & timeStep , const unsigned & uFrame ) ;

        void        DiffuseVorticityGlobally( const float & timeStep , const unsigned & uFrame ) ;
        void        DiffuseVorticityPSE( const float & timeStep , const unsigned & uFrame ) ;
        void        AdvectVortons( const float & timeStep , const unsigned & uFrame ) ;

        Vector< Vorton >        mVortons                ;   ///< Dynamic array of tiny vortex elements

        NestedGrid< Vorton >    mInfluenceTree          ;   ///< Influence tree
        UniformGrid< Vec3 >     mVorticityGrid          ;   ///< Grid populated with vorticity from vortons
        NestedGrid< Vec3 >      mVorticityMultiGrid     ;   ///< Multi-resolution grid populated with vorticity from vortons
        UniformGrid< Vec3 >     mVelGrid                ;   ///< Uniform grid of velocity values
        UniformGridGeometry     mGridTemplate           ;   ///< Geometry of grid used to contain velocity, vorticity, etc.
        Vec3                    mMinCorner              ;   ///< Minimal corner of axis-aligned bounding box
        Vec3                    mMaxCorner              ;   ///< Maximal corner of axis-aligned bounding box
        Vec3                    mMinCornerEternal       ;   ///< Minimal corner of axis-aligned bounding box, across all time
        Vec3                    mMaxCornerEternal       ;   ///< Maximal corner of axis-aligned bounding box, across all time
        Vec3                    mCirculationInitial     ;   ///< Initial circulation, which should be conserved when viscosity is zero.
        Vec3                    mLinearImpulseInitial   ;   ///< Initial linear impulse, which should be conserved when viscosity is zero.
        float                   mViscosity              ;   ///< Viscosity.  Used to compute viscous diffusion.
        float                   mFluidDensity           ;   ///< Uniform density of fluid.

        float                   mVortonRadius           ;   ///< Radius of actual vortons (not of supervortons)

    #if USE_TBB
        friend class VortonSim_ComputeVelocityGrid_TBB  ; ///< Multi-threading helper class
    #endif
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
