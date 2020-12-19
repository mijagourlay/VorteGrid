/*! \file VortonSim.cpp

    \brief Dynamic simulation of a fluid, using tiny vortex elements.

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
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include <stdlib.h>

#include "Core/Performance/perf.h"
#include "Space/uniformGridMath.h"
#include "vortonClusterAux.h"
#include "vorticityDistribution.h"
#include "vortonSim.h"




#if USE_TBB
    extern unsigned gNumberOfProcessors ;  ///< Number of processors this machine has.  This will get reassigned later.

    /*! \brief Function object to compute velocity grid using Threading Building Blocks
    */
    class VortonSim_ComputeVelocityGrid_TBB
    {
            VortonSim * mVortonSim ;    ///< Address of VortonSim object
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of velocity grid.
                mVortonSim->ComputeVelocityGridSlice( r.begin() , r.end() ) ;
            }
            VortonSim_ComputeVelocityGrid_TBB( VortonSim * pVortonSim )
                : mVortonSim( pVortonSim ) {}
    } ;

    /*! \brief Function object to compute fluid buoyancy using Threading Building Blocks
    */
    class VortonSim_GenerateBaroclinicVorticity_TBB
    {
            float       mTimeStep   ;   ///< Duration since last time step.
            VortonSim * mVortonSim  ;   ///< Address of VortonSim object
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of velocity grid.
                mVortonSim->GenerateBaroclinicVorticitySlice( mTimeStep , r.begin() , r.end() ) ;
            }
            VortonSim_GenerateBaroclinicVorticity_TBB( float timeStep , VortonSim * pVortonSim )
                : mTimeStep( timeStep )
                , mVortonSim( pVortonSim )
            {}
    } ;

    /*! \brief Function object to compute vorticity diffusion using Threading Building Blocks
    */
    class VortonSim_DiffuseVorticityPSE_TBB
    {
            float                               mTimeStep       ;   ///< Duration since last time step.
            VortonSim *                         mVortonSim      ;   ///< Address of VortonSim object
            UniformGrid< Vector< unsigned > > & mUgVortonIndices;   ///< Reference to uniform grid of vorton indices
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of vorticity diffusion.
                mVortonSim->DiffuseVorticityPSESlice( mTimeStep , mUgVortonIndices , r.begin() , r.end() ) ;
            }
            VortonSim_DiffuseVorticityPSE_TBB( float timeStep , VortonSim * pVortonSim , UniformGrid< Vector< unsigned > > & ugVortonIndices )
                : mTimeStep( timeStep )
                , mVortonSim( pVortonSim )
                , mUgVortonIndices( ugVortonIndices )
            {}
    } ;


    /*! \brief Function object to compute heat diffusion using Threading Building Blocks
    */
    class VortonSim_DiffuseHeatPSE_TBB
    {
            float                                       mTimeStep       ;   ///< Duration since last time step.
            VortonSim *                                 mVortonSim      ;   ///< Address of VortonSim object
            const UniformGrid< Vector< unsigned > > &   mUgVortonIndices;   ///< Reference to uniform grid of vorton indices
        public:
            void operator() ( const tbb::blocked_range<size_t> & r ) const
            {   // Compute subset of heat diffusion.
                mVortonSim->DiffuseHeatPSESlice( mTimeStep , mUgVortonIndices , r.begin() , r.end() ) ;
            }
            VortonSim_DiffuseHeatPSE_TBB( float timeStep , VortonSim * pVortonSim , const UniformGrid< Vector< unsigned > > & ugVortonIndices )
                : mTimeStep( timeStep )
                , mVortonSim( pVortonSim )
                , mUgVortonIndices( ugVortonIndices )
            {}
    } ;
#endif




                       // Number of times an influence-tree leaf-node was hit while calculating velocity.
                       // Descents while traversing influence tree, calculating velocity.





/*! \brief Assign vortons from a uniform grid of vorticity

    \param vortGrid - uniform grid of vorticity values

*/
void VortonSim::AssignVortonsFromVorticity( UniformGrid< Vec3 > & vortGrid )
{
    mVortons->Clear() ; // Empty out any existing vortons.

    // Obtain characteristic size of each grid cell.

    const UniformGridGeometry & ug  = vortGrid ;
    const float     fVortonRadius   = powf( ug.GetCellSpacing().x * ug.GetCellSpacing().y  * ug.GetCellSpacing().z , 1.0f / 3.0f ) * 0.5f ;
    const Vec3      Nudge           ( ug.GetExtent() * FLT_EPSILON * 4.0f ) ;
    const Vec3      vMin            ( ug.GetMinCorner()   + Nudge ) ;
    const Vec3      vSpacing        ( ug.GetCellSpacing() * ( 1.0f - 0.0f * FLT_EPSILON ) ) ;
    const unsigned  numPoints[3]    = { ug.GetNumPoints(0) , ug.GetNumPoints(1) , ug.GetNumPoints(2) } ;
    const unsigned  numXY           = numPoints[0] * numPoints[1] ;
    unsigned idx[3] ;
    for( idx[2] = 0 ; idx[2] < numPoints[2] ; ++ idx[2] )
    {
        Vec3 vPositionOfGridCellCenter ;
        vPositionOfGridCellCenter.z = vMin.z + float( idx[2] ) * vSpacing.z ;
        const unsigned offsetZ = idx[2] * numXY ;
        for( idx[1] = 0 ; idx[1] < numPoints[1] ; ++ idx[1] )
        {
            vPositionOfGridCellCenter.y = vMin.y + float( idx[1] ) * vSpacing.y ;
            const unsigned offsetYZ = idx[1] * vortGrid.GetNumPoints(0) + offsetZ ;
            for( idx[0] = 0 ; idx[0] < numPoints[0] ; ++ idx[0] )
            {
                vPositionOfGridCellCenter.x = vMin.x + float( idx[0] ) * vSpacing.x ;
                const unsigned offsetXYZ = idx[0] + offsetYZ ;
                const Vec3 & rVort = vortGrid[ offsetXYZ ] ;
                if( rVort.Mag2() > FLT_EPSILON )
                {   // This grid cell contains significant vorticity.
                    Vorton vorton( vPositionOfGridCellCenter , rVort , fVortonRadius ) ;
                    mVortons->PushBack( vorton ) ;
                }
            }
        }
    }
}




/*! \brief Compute the total circulation and linear impulse of all vortons in this simulation.

    \param vCirculation - Total circulation, the volume integral of vorticity, computed by this routine.

    \param vLinearImpulse - Volume integral of circulation weighted by position, computed by this routine.

*/
void    VortonSim::ConservedQuantities( Vec3 & vCirculation , Vec3 & vLinearImpulse ) const
{
    // Zero accumulators.
    vCirculation = vLinearImpulse = Vec3( 0.0f , 0.0f , 0.0f ) ;
    const size_t numVortons = mVortons->Size() ;
    for( unsigned iVorton = 0 ; iVorton < numVortons ; ++ iVorton )
    {   // For each vorton in this simulation...
        const Vorton &  rVorton         = (*mVortons)[ iVorton ] ;
        const float     volumeElement   = POW3( rVorton.GetRadius() ) ;
        // Accumulate total circulation.
        vCirculation    += rVorton.GetVorticity() * volumeElement ;
        // Accumulate total linear impulse.
        vLinearImpulse  += rVorton.mPosition ^ rVorton.GetVorticity() * volumeElement ;
    }
}




/*! \brief Initialize a vortex particle fluid simulation

    \note This method assumes the vortons have been initialized.
            That includes removing any vortons embedded inside
            rigid bodies.

*/
void VortonSim::Initialize( void )
{
#if USE_TBB
    // Query environment for number of processors on this machine.
    const char * strNumberOfProcessors = getenv( "NUMBER_OF_PROCESSORS" ) ;
    if( strNumberOfProcessors != 0 )
    {   // Environment contains a value for number of processors.
        gNumberOfProcessors = atoi( strNumberOfProcessors ) ;
    }
    #if PROFILE
    fprintf( stderr , "# CPU's: %u.  Built on " __DATE__ " " __TIME__ "\n" , gNumberOfProcessors ) ;
    #endif
#endif

#if 1 || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) || defined( _DEBUG )
    mVortonRadius = (*mVortons)[ 0 ].GetRadius() ;
#endif

    ConservedQuantities( mCirculationInitial , mLinearImpulseInitial ) ;
    

    // Find grid geometry to seed passive tracer particles.
    FindBoundingBox() ; // Find axis-aligned bounding box that encloses all vortons.

    // Create a preliminary grid template, useful for initializing tracers.
    //  This is kind of a hack because technically the grid should only be used to
    //  define the velocity grid but it ends up having multiple uses and meanings,
    //  which theoretically should differ from each other.
    UpdateBoundingBox( mMinCorner , mMaxCorner , /* finalize? See below. */ false ) ;

    // "Final" UpdateBoundingBox above did not "finalize", because
    // the finalization takes into account the velocity solver technique,
    // so the bounding box is padded differently for different solvers.
    // At this phase, we only want to know the snug-fitting box,
    // because we're using that to compute particle sizes,
    // which should not be influenced by the velocity solver technique.
    // So perform necessary finalizing operations here.
    {
        // Slightly enlarge bounding box to allow for round-off errors.
        const float margin = 1.0f * mVortonRadius ;
        const Vec3 nudge( margin * Vec3( 1.0f , 1.0f , 1.0f ) ) ;
        mMinCorner -= nudge ;
        mMaxCorner += nudge ;
        mGridTemplate.DefineShape( mVortons->Size() , mMinCorner , mMaxCorner , true ) ;
    }

#if 0 && defined( _DEBUG )
    {
        // Exercise regularized Biot-Savart formula.
        FILE * velFromVortFile = fopen( "velFromVort.dat" , "w" ) ;
        if( velFromVortFile != NULL )
        {
            const Vec3  vortonPosition( 0.0f , 0.0f , 0.0f ) ;
            const Vec3  vortonAngVel( 0.0f , 0.0f , 1.0f ) ;
            const float vortonDiameter           = 1.0f ;
            const float queryDistanceMax         = vortonDiameter * 10.0f ;
            const int   numQueryPositions        = 10000 ;
            const float oneOverNumQueryPositions = 1.0f / float( numQueryPositions ) ;
            fprintf( velFromVortFile , "# r vel\n" ) ;
            for( int iQueryPos = 0 ; iQueryPos < numQueryPositions ; ++ iQueryPos )
            {
                const float queryDist = queryDistanceMax * float( iQueryPos ) * oneOverNumQueryPositions ;
                Vec3        queryPositionAlongX( queryDist , 0.0f , 0.0f ) ;
                Vec3        velocityFromVorticity( 0.0f , 0.0f , 0.0f ) ;
                VORTON_ACCUMULATE_VELOCITY_private( velocityFromVorticity , queryPositionAlongX , vortonPosition , vortonAngVel , vortonDiameter ) ;
                {
                    Vec3        queryPositionAlongY( 0.0f , queryDist , 0.0f ) ;
                    Vec3        velocityFromVorticityCheck( 0.0f , 0.0f , 0.0f ) ;
                    VORTON_ACCUMULATE_VELOCITY_private( velocityFromVorticityCheck , queryPositionAlongY , vortonPosition , vortonAngVel , vortonDiameter ) ;
                }
                fprintf( velFromVortFile , "%g %g\n" , queryDist , velocityFromVorticity.y ) ;
            }
            fclose( velFromVortFile ) ;
        }
    }
#endif
}




/*! \brief Find axis-aligned bounding box for all vortons in this simulation.

    \see UpdateBoundingBox which is used to update the box to include tracers
*/
void VortonSim::FindBoundingBox( void )
{
    QUERY_PERFORMANCE_ENTER ;
    mMinCorner = Vec3( FLT_MAX , FLT_MAX , FLT_MAX ) ;
    mMaxCorner = - mMinCorner ;
    Particles::FindBoundingBox( reinterpret_cast< const Vector<Particle> & >( * GetVortons() ) , mMinCorner , mMaxCorner ) ;
    QUERY_PERFORMANCE_EXIT( VortonSim_CreateInfluenceTree_FindBoundingBox_Vortons ) ;
}




/*! \brief Update axis-aligned bounding box to include given region.

    \param minCorner - minimal corner of region to include

    \param maxCorner - maximal corner of region to include

    \param bFinal - whether this is the final update to the bounding box.
            On the final update, this routine performs finalization operations.
            They should only happen once, and only after all updates are done.

    \note that this operation does not shrink the box; it can enlarge it.

    \see FindBoundingBox
*/
void VortonSim::UpdateBoundingBox( const Vec3 & minCorner , const Vec3 & maxCorner , bool bFinal )
{
    mMinCorner.x = MIN2( mMinCorner.x , minCorner.x ) ; // Note: TODO: Perhaps SSE/VMX have a SIMD/vector form of this operation.
    mMinCorner.y = MIN2( mMinCorner.y , minCorner.y ) ;
    mMinCorner.z = MIN2( mMinCorner.z , minCorner.z ) ;
    mMaxCorner.x = MAX2( mMaxCorner.x , maxCorner.x ) ;
    mMaxCorner.y = MAX2( mMaxCorner.y , maxCorner.y ) ;
    mMaxCorner.z = MAX2( mMaxCorner.z , maxCorner.z ) ;

    if( bFinal )
    {   // This is the final amendment to the bounding box.

        // Enlarge bounding box to include entire vorton, not just its center.
        //const Vec3 nudge( extent * FLT_EPSILON ) ;
#if VELOCITY_TECHNIQUE  == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL
        // When using Poisson solver, we want the boundary to have strictly zero vorticity.
        static const float margin = 3.0f * mVortonRadius ;
#else
        static const float margin = 1.0f * mVortonRadius ;
#endif

#if JITTER_BOX  // Randomly jitter bounding box
        static const float  sOneOverRandMax = 0.05f / float( RAND_MAX ) ;
        const float jitter =  float( rand() ) * sOneOverRandMax ;
        const Vec3 extent( mMaxCorner - mMinCorner ) ;
        const Vec3 vJitter = jitter * extent ;
        const Vec3 nudge( margin * Vec3( 1.0f , 1.0f , 1.0f ) + vJitter ) ;
#else
        const Vec3 nudge( margin * Vec3( 1.0f , 1.0f , 1.0f )           ) ;
#endif

        mMinCorner -= nudge ;
        mMaxCorner += nudge ;

        mMinCornerEternal.x = MIN2( mMinCorner.x , mMinCornerEternal.x ) ;
        mMinCornerEternal.y = MIN2( mMinCorner.y , mMinCornerEternal.y ) ;
        mMinCornerEternal.z = MIN2( mMinCorner.z , mMinCornerEternal.z ) ;
        mMaxCornerEternal.x = MAX2( mMaxCorner.x , mMaxCornerEternal.x ) ;
        mMaxCornerEternal.y = MAX2( mMaxCorner.y , mMaxCornerEternal.y ) ;
        mMaxCornerEternal.z = MAX2( mMaxCorner.z , mMaxCornerEternal.z ) ;

        mGridTemplate.DefineShape( mVortons->Size() , mMinCorner , mMaxCorner , true ) ;
    }
}





/*! \brief Compute velocity at a given point in space, due to influence of vortons

    \param vPosition - point in space

    \return velocity at vPosition, due to influence of vortons

    \note This is a brute-force algorithm with time complexity O(N)
            where N is the number of vortons.  This is too slow
            for regular use but it is useful for comparisons.

*/
Vec3 VortonSim::ComputeVelocityDirect( const Vec3 & vPosition )
{
    const size_t    numVortons          = mVortons->Size() ;
    Vec3            velocityAccumulator( 0.0f , 0.0f , 0.0f ) ;

    for( unsigned iVorton = 0 ; iVorton < numVortons ; ++ iVorton )
    {   // For each vorton...
        const Vorton &  rVorton = (*mVortons)[ iVorton ] ;
        VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , rVorton ) ;
    }

    return velocityAccumulator ;
}




/*! \brief Create base layer of vorton influence tree.

    This is the leaf layer, where each grid cell corresponds (on average) to
    a single vorton.  Some cells might contain multiple vortons and some zero.
    Each cell effectively has a single "supervorton" which its parent layers
    in the influence tree will in turn aggregate.

    \note This implementation of gridifying the base layer is NOT suitable
            for Eulerian operations like approximating spatial derivatives
            of vorticity or solving a vector Poisson equation, because this
            routine associates each vortex with a single corner point of the
            grid cell that contains it.  To create a grid for Eulerian calculations,
            each vorton would contribute to all 8 corner points of the grid
            cell that contains it.

            We could rewrite this to suit "Eulerian" operations, in which case
            we would want to omit "size" and "position" since the grid would
            implicitly represent that information.  That concern goes hand-in-hand
            with the method used to compute velocity from vorticity.
            Ultimately we need to make sure theoretically conserved quantities behave as expected.

    \note This method assumes the influence tree skeleton has already been created,
            and the leaf layer initialized to all "zeros", meaning it contains no
            vortons.

*/
void VortonSim::MakeBaseVortonGrid( void )
{
    const size_t numVortons = mVortons->Size() ;

    UniformGrid< VortonClusterAux > ugAux( mInfluenceTree[0] ) ; // Temporary auxilliary information used during aggregation.
    ugAux.Init() ;

    

#if 1 || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) || defined( _DEBUG )
    // Assume all vortons have the same radius.
    
#endif

    UniformGrid<Vorton> & baseGrid = mInfluenceTree[0] ;

#if 0 && defined( _DEBUG )
    for( unsigned uVorton = 0 ; uVorton < numVortons ; ++ uVorton )
    {   // For each vorton in this simulation...
        const Vorton     &  rVorton     = (*mVortons)[ uVorton ] ;
        const Vec3       &  rPosition   = rVorton.mPosition   ;
        const unsigned      uOffset     = baseGrid.OffsetOfPosition( rPosition ) ;
        Vorton           &  rVortonCell = baseGrid[ uOffset ] ;
        rVortonCell.mTotalCirculation = Vec3( 0.0f , 0.0f , 0.0f ) ;
    }
#endif

    // Compute preliminary vorticity grid.


    for( unsigned uVorton = 0 ; uVorton < numVortons ; ++ uVorton )
    {   // For each vorton in this simulation...
        const Vorton     &  rVorton     = (*mVortons)[ uVorton ] ;
    #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
    #endif
        const Vec3       &  rPosition   = rVorton.mPosition   ;
        const unsigned      uOffset     = baseGrid.OffsetOfPosition( rPosition ) ;

    #if ( ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) )
        Vorton           &  rVortonCell = baseGrid[ uOffset ] ;
        VortonClusterAux &  rVortonAux  = ugAux[ uOffset ] ;
        const float         vortMag     = rVorton.GetVorticity().Magnitude() ;

        rVortonCell.mPosition  += rVorton.mPosition * vortMag ; // Compute weighted position -- to be normalized later.

        rVortonCell.mAngularVelocity += rVorton.mAngularVelocity    ; // Tally vorticity sum.
        rVortonCell.mSize             = rVorton.mSize               ; // Assign volume element size.
        
        
        

        // OBSOLETE. See comments below: UpdateBoundingBox( rVortonAux.mMinCorner , rVortonAux.mMaxCorner , rVorton.mPosition ) ;
        rVortonAux.mVortNormSum += vortMag ;
    #elif ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT )
    #else
        #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
    #endif
    }



    // Post-process preliminary grid; normalize center-of-vorticity and compute sizes, for each grid cell.
    

    const unsigned num[3] = {   mInfluenceTree[0].GetNumPoints( 0 ) ,
                                mInfluenceTree[0].GetNumPoints( 1 ) ,
                                mInfluenceTree[0].GetNumPoints( 2 ) } ;
    const unsigned numXY = num[0] * num[1] ;
    unsigned idx[3] ;
    for( idx[2] = 0 ; idx[2] < num[2] ; ++ idx[2] )
    {
        const unsigned zShift = idx[2] * numXY ;
        for( idx[1] = 0 ; idx[1] < num[1] ; ++ idx[1] )
        {
            const unsigned yzShift = idx[1] * num[0] + zShift ;
            for( idx[0] = 0 ; idx[0] < num[0] ; ++ idx[0] )
            {
                const unsigned      offset      = idx[0] + yzShift ;
                VortonClusterAux &  rVortonAux  = ugAux[ offset ] ;
                if( rVortonAux.mVortNormSum != FLT_MIN )
                {   // This cell contains at least one vorton.
                    Vorton & rVortonCell = mInfluenceTree[0][ offset ] ;
                    // Normalize weighted position sum to obtain center-of-vorticity.
                    rVortonCell.mPosition /= rVortonAux.mVortNormSum ;

                    
                }
                else
                {
                    
                }
            }
        }
    }

    // If this triggers but the above apparently identical assertions do not,
    // then the simulation probably has some zero-vorticity vortons.
    // That is okay, but there is logic elsewhere that does not include zero-vorticity
    // vortons into the influence tree, hence they do not get counted as "incorporated".
    // The solution to that is either to "incorporate" even zero-vorticity vortons
    // (which is obviously somewhat silly), ignore this assertion (which is dangerous
    // in situations where it catches legitimate errors), or assign some tiny,
    // otherwise negligible vorticity to all vortons, for example using FLT_EPSILON.
    // While epsilon-strength vortons might seem silly, in practice it helps with
    // diagnosing the algorithm.  For release code, zero-vorticity vortons work just fine.
}




/*! \brief Aggregate vorton clusters from a child layer into a parent layer of the influence tree

    This routine assumes the given parent layer is empty and its child layer (i.e. the layer
    with index uParentLayer-1) is populated.

    \param uParentLayer - index of parent layer into which aggregated influence information will be stored.
        This must be greater than 0 because the base layer, which has no children, has index 0.

    \see CreateInfluenceTree

*/
void VortonSim::AggregateClusters( unsigned uParentLayer )
{
    UniformGrid<Vorton> &   rParentLayer    = mInfluenceTree[ uParentLayer     ] ;
    UniformGrid<Vorton> &   rChildLayer     = mInfluenceTree[ uParentLayer - 1 ] ;
    const unsigned &        numXchild       = rChildLayer.GetNumPoints( 0 ) ;
    const unsigned          numXYchild      = numXchild * rChildLayer.GetNumPoints( 1 ) ;


    // number of cells in each grid cluster
    const unsigned * const pClusterDims = mInfluenceTree.GetDecimations( uParentLayer ) ;

    const unsigned  numCells[3]         = { rParentLayer.GetNumCells( 0 ) , rParentLayer.GetNumCells( 1 ) , rParentLayer.GetNumCells( 2 ) } ;
    const unsigned  numXY               = rParentLayer.GetNumPoints( 0 ) * rParentLayer.GetNumPoints( 1 ) ;
    // (Since this loop writes to each parent cell, it should readily parallelize without contention.)
    unsigned idxParent[3] ;
    for( idxParent[2] = 0 ; idxParent[2] < numCells[2] ; ++ idxParent[2] )
    {
        const unsigned offsetZ = idxParent[2] * numXY ;
        for( idxParent[1] = 0 ; idxParent[1] < numCells[1] ; ++ idxParent[1] )
        {
            const unsigned offsetYZ = idxParent[1] * rParentLayer.GetNumPoints( 0 ) + offsetZ ;
            for( idxParent[0] = 0 ; idxParent[0] < numCells[0] ; ++ idxParent[0] )
            {   // For each cell in the parent layer...
                const unsigned offsetXYZ = idxParent[0] + offsetYZ ;
                Vorton              & rVortonParent = rParentLayer[ offsetXYZ ] ;
                VortonClusterAux vortAux ;
                unsigned clusterMinIndices[ 3 ] ;
                mInfluenceTree.GetChildClusterMinCornerIndex( clusterMinIndices , pClusterDims , idxParent ) ;
                unsigned increment[3] ;
                // For each cell of child layer in this grid cluster...
                for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
                {
                    const unsigned offsetZ = ( clusterMinIndices[2] + increment[2] ) * numXYchild ;
                    for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
                    {
                        const unsigned offsetYZ = ( clusterMinIndices[1] + increment[1] ) * numXchild + offsetZ ;
                        for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
                        {
                            const unsigned  offsetXYZ       = ( clusterMinIndices[0] + increment[0] ) + offsetYZ ;
                            Vorton &        rVortonChild    = rChildLayer[ offsetXYZ ] ;
                            const float     vortMag         = rVortonChild.GetVorticity().Magnitude() ;

                            // Aggregate vorton cluster from child layer into parent layer:
                            rVortonParent.mPosition  += rVortonChild.mPosition * vortMag ;

                            rVortonParent.mAngularVelocity += rVortonChild.mAngularVelocity ;
                            
                            vortAux.mVortNormSum     += vortMag ;
                            if( rVortonChild.mSize != 0.0f )
                            {   // Child vorton exists
                            #if 0 && ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES )
                                // Compute parent-to-child distance.
                                const Vec3  vParentToChild  = rVortonParent.mPosition - rVortonChild.mPosition ;
                                const float fParentToChild2 = vParentToChild.Mag2() ;
                                const float fParentToChild4 = POW2( fParentToChild2 ) ;
                                const float fParentToChild8 = POW2( fParentToChild4 ) ;
                                // Formula for parent radius shall be N-norm sum of parent-child distance:
                                // rVortonParent.mRadius += pow( parentChildDist , N ) where N is larger than 2, perhaps 8.
                                // (plus mVortonRadius will be added at the very end, outside last loop below)
                                rVortonParent.SetRadius( rVortonParent.GetRadius() + fParentToChild8 ) ;
                            #else
                                rVortonParent.mSize = rVortonChild.mSize ;
                            #endif
                            }


                        #if VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_MONOPOLES
                            
                        #endif
                        }
                    }
                }

                

            #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
                //// Correct vorticity to preserve total circulation.
                //const float vorticityReduction = mVortonRadius / rVortonParent.GetRadius() ;
                //
                //rVortonParent.mAngularVelocity *= POW3( vorticityReduction ) ;
                for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
                {
                    const unsigned offsetZ = ( clusterMinIndices[2] + increment[2] ) * numXYchild ;
                    for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
                    {
                        const unsigned offsetYZ = ( clusterMinIndices[1] + increment[1] ) * numXchild + offsetZ ;
                        for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
                        {
                            const unsigned  offsetXYZ               = ( clusterMinIndices[0] + increment[0] ) + offsetYZ ;
                            Vorton &        rVortonChild            = rChildLayer[ offsetXYZ ] ;
                            const float     vortMag                 = rVortonChild.GetVorticity().Magnitude() ;
                            // If this parent cell contains only this child vorton then the vortNormSum will become zero.
                            // We need to avoid it becoming actually zero, because of the quotient below.
                            // When it would have become zero, the numerator would also be zero, so dividing by a small number is harmless.
                            // In contrast, doing a conditional branch instead costs more CPU time.
                            const float     vortNormSumMinusSelf    = vortAux.mVortNormSum - vortMag + sAvoidSingularity ;

                            // Dissect parent supervorton to remove sibling information and store in this child.
                            rVortonChild.mPositionSiblings  = ( rVortonParent.mPosition - rVortonChild.mPosition * vortMag ) / vortNormSumMinusSelf ;

                            rVortonChild.SetRadius( mVortonRadius ) ;

                            rVortonChild.mVorticitySiblings = rVortonParent.GetVorticity() - rVortonChild.GetVorticity() ;
                            

                        #if 0
                            // Correct radius by recomputing parent radius after removing this child.
                            // rVortonParent.mRadius is the sum of dist^N where N>2 (e.g. N=8, as defined above)
                            // so the intermediate corrected radius will be rVortonParent.mRadius - parentChildDist^N.
                            // The actual corrected radius will be the intermediate corrected radius + mVortonRadius (the radius of each actual vorton),
                            // then taking the 1/N root.
                            // The N-norm approximates using the farthest parent-to-sibling distance without having to remember each individual distance.
                            // Compute parent-to-child distance.
                            const Vec3 vParentToChild = rVortonParent.mPosition - rVortonChild.mPosition ;
                            const float fParentToChild2 = vParentToChild.Mag2() ;
                            const float fParentToChild4 = POW2( fParentToChild2 ) ;
                            const float fParentToChild8 = POW2( fParentToChild4 ) ;
                            rVortonChild.mRadiusSiblings = MAX2( rVortonParent.GetRadius() - fParentToChild8 , 0.0f ) ;
                            rVortonChild.mRadiusSiblings = pow( rVortonChild.GetRadius() , 0.125f /* 1/8 */ ) + mVortonRadius ;


                            // Correct vorticity to preserve total circulation.
                            const float vorticityReduction = mVortonRadius / rVortonChild.mRadiusSiblings ;
                            rVortonChild.mVorticitySiblings *= POW3( vorticityReduction ) ;
                            
                        #else
                            
                        #endif

                        }
                    }
                }
            #endif

                // Normalize weighted position sum to obtain center-of-vorticity.
                // (See analogous code in MakeBaseVortonGrid.)
                rVortonParent.mPosition /= vortAux.mVortNormSum ;
            }
        }
    }

}




/*! \brief Create nested grid vorticity influence tree

    Each layer of this tree represents a simplified, aggregated version of
    all of the information in its "child" layer, where each
    "child" has higher resolution than its "parent".

    \see MakeBaseVortonGrid, AggregateClusters

    Derivation:

    Using conservation properties, I_0 = I_0' , I_1 = I_1' , I_2 = I_2'

    I_0 : wx d = w1x d1 + w2x d2
        : wy d = w1y d1 + w2y d2
        : wz d = w1z d1 + w2z d2

        These 3 are not linearly independent:
    I_1 : ( y wz - z wy ) d = ( y1 wz1 - z1 wy1 ) d1 + ( y2 wz2 - z2 wy2 ) d2
        : ( z wx - x wz ) d = ( z1 wx1 - x1 wz1 ) d1 + ( z2 wx2 - x2 wz2 ) d2
        : ( x wy - y wx ) d = ( x1 wy1 - y1 wx1 ) d1 + ( x2 wy2 - y2 wx2 ) d2

    I_2 : ( x^2 + y^2 + z^2 ) wx d = (x1^2 + y1^2 + z1^2 ) wx1 d1 + ( x2^2 + y2^2 + z2^2 ) wx2 d2
        : ( x^2 + y^2 + z^2 ) wy d = (x1^2 + y1^2 + z1^2 ) wy1 d1 + ( x2^2 + y2^2 + z2^2 ) wy2 d2
        : ( x^2 + y^2 + z^2 ) wz d = (x1^2 + y1^2 + z1^2 ) wz1 d1 + ( x2^2 + y2^2 + z2^2 ) wz2 d2

    Can replace I_2 with its magnitude:
              ( x^2  + y^2  + z^2  ) ( wx^2  + wy^2  + wz^2  )^(1/2) d
            = ( x1^2 + y1^2 + z1^2 ) ( wx1^2 + w1y^2 + w1z^2 )^(1/2) d1
            + ( x2^2 + y2^2 + z2^2 ) ( wx2^2 + w2y^2 + w2z^2 )^(1/2) d2

*/
void VortonSim::CreateInfluenceTree( void )
{
    // Create skeletal nested grid for influence tree.
    //const size_t numVortons = mVortons->Size() ;
    mInfluenceTree.Initialize( mGridTemplate ) ; // Create skeleton of influence tree.

    QUERY_PERFORMANCE_ENTER ;
    MakeBaseVortonGrid() ;
    QUERY_PERFORMANCE_EXIT( VortonSim_CreateInfluenceTree_MakeBaseVortonGrid ) ;

    QUERY_PERFORMANCE_ENTER ;
    const size_t numLayers = mInfluenceTree.GetDepth() ;
    for( unsigned int uParentLayer = 1 ; uParentLayer < numLayers ; ++ uParentLayer )
    {   // For each layer in the influence tree...
        AggregateClusters( uParentLayer ) ;
    }
    QUERY_PERFORMANCE_EXIT( VortonSim_CreateInfluenceTree_AggregateClusters ) ;
}











/*! \brief Compute velocity at a given point in space, due to influence of vortons

    \param vPosition - point in space whose velocity to evaluate

    \param indices - indices of cell to visit in the given layer

    \param iLayer - which layer to process

    \return velocity at vPosition, due to influence of vortons

    \note This is a recursive algorithm with time complexity O(log(N)). 
            The outermost caller should pass in mInfluenceTree.GetDepth().

*/
Vec3 VortonSim::ComputeVelocityTree( const Vec3 & vPosition , const unsigned indices[3] , size_t iLayer )
{
    UniformGrid< Vorton > & rChildLayer             = mInfluenceTree[ iLayer - 1 ] ;
    unsigned                clusterMinIndices[3] ;
    const unsigned *        pClusterDims             = mInfluenceTree.GetDecimations( iLayer ) ;
    mInfluenceTree.GetChildClusterMinCornerIndex( clusterMinIndices , pClusterDims , indices ) ;

    const Vec3 &            vGridMinCorner          = rChildLayer.GetMinCorner() ;
    const Vec3              vSpacing                = rChildLayer.GetCellSpacing() ;
    unsigned                increment[3]            ;
    const unsigned &        numXchild               = rChildLayer.GetNumPoints( 0 ) ;
    const unsigned          numXYchild              = numXchild * rChildLayer.GetNumPoints( 1 ) ;
    Vec3                    velocityAccumulator( 0.0f , 0.0f , 0.0f ) ;

    // The larger this is, the more accurate (and slower) the evaluation.
    // Reasonable values lie in [0.00001,4.0].
    // Setting this to 0 leads to very bad errors, but values greater than (tiny) lead to drastic improvements.
    // Changes in margin have a quantized effect since they effectively indicate how many additional
    // cluster subdivisions to visit.
    static const float  marginFactor    = 0.0001f ; // 0.4f ; // ship with this number: 0.0001f ; test with 0.4
    // When domain is 2D in XY plane, min.z==max.z so vPos.z test below would fail unless margin.z!=0.
    const Vec3          margin          = marginFactor * vSpacing + ( 0.0f == vSpacing.z ? Vec3(0,0,FLT_MIN) : Vec3(0,0,0) );

    // For each cell of child layer in this grid cluster...
    for( increment[2] = 0 ; increment[2] < pClusterDims[2] ; ++ increment[2] )
    {
        unsigned idxChild[3] ;
        idxChild[2] = clusterMinIndices[2] + increment[2] ;
        Vec3 vCellMinCorner , vCellMaxCorner ;
        vCellMinCorner.z = vGridMinCorner.z + float( idxChild[2]     ) * vSpacing.z ;
        vCellMaxCorner.z = vGridMinCorner.z + float( idxChild[2] + 1 ) * vSpacing.z ;
        const unsigned offsetZ = idxChild[2] * numXYchild ;
        for( increment[1] = 0 ; increment[1] < pClusterDims[1] ; ++ increment[1] )
        {
            idxChild[1] = clusterMinIndices[1] + increment[1] ;
            vCellMinCorner.y = vGridMinCorner.y + float( idxChild[1]     ) * vSpacing.y ;
            vCellMaxCorner.y = vGridMinCorner.y + float( idxChild[1] + 1 ) * vSpacing.y ;
            const unsigned offsetYZ = idxChild[1] * numXchild + offsetZ ;
            for( increment[0] = 0 ; increment[0] < pClusterDims[0] ; ++ increment[0] )
            {
                idxChild[0] = clusterMinIndices[0] + increment[0] ;
                vCellMinCorner.x = vGridMinCorner.x + float( idxChild[0]     ) * vSpacing.x ;
                vCellMaxCorner.x = vGridMinCorner.x + float( idxChild[0] + 1 ) * vSpacing.x ;
                if(
                        ( iLayer > 1 )
                    &&  ( vPosition.x >= vCellMinCorner.x - margin.x )
                    &&  ( vPosition.y >= vCellMinCorner.y - margin.y )
                    &&  ( vPosition.z >= vCellMinCorner.z - margin.z )
                    &&  ( vPosition.x <  vCellMaxCorner.x + margin.x )
                    &&  ( vPosition.y <  vCellMaxCorner.y + margin.y )
                    &&  ( vPosition.z <  vCellMaxCorner.z + margin.z )
                  )
                {   // Test position is inside childCell and currentLayer > 0...
                    // Recurse child layer.
                    velocityAccumulator += ComputeVelocityTree( vPosition , idxChild , iLayer - 1 ) ;
                }
                else
                {   // Test position is outside childCell, or reached leaf node.
                    //    Compute velocity induced by cell at corner point x.
                    //    Accumulate influence, storing in velocityAccumulator.
                    const unsigned  offsetXYZ       = idxChild[0] + offsetYZ ;
                    const Vorton &  rVortonChild    = rChildLayer[ offsetXYZ ] ;
                    VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , rVortonChild ) ;
                    
                    
                    
                    
                }
            }
        }
    }

    return velocityAccumulator ;
}




#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
/*! \brief Compute velocity at a given point in space, due to influence of vortons

    \param indices - indices of grid cell of position whose velocity to evaluate

    \param vPosition - point in space whose velocity to evaluate

    \return velocity at vPosition, due to influence of vortons

    \note This is a recursive algorithm with time complexity O(log(N)). 
              The outermost caller should pass in mInfluenceTree.GetDepth().
  
  */
Vec3 VortonSim::ComputeVelocityMonopoles( const unsigned indices[3] , const Vec3 & vPosition )
{
    Vec3        velocityAccumulator( 0.0f , 0.0f , 0.0f ) ;
    unsigned    indicesVisited[4] = { indices[0] , indices[1] , indices[2] , 0 } ;


    
    
    

    // Special case: Compute velocity due to vortons in same gridcell as query point.
    {
        UniformGrid< Vorton > & rLayer  = mInfluenceTree[ 0 ] ;

        // The outermost layer of the grid contains no vortices.
        // Any queries that walk along that outer layer see no influence.
        // Make sure all queries start with indices that lie strictly within the region that contains influence data.
        indicesVisited[ 0 ] = MIN2( indicesVisited[ 0 ] , rLayer.GetNumCells( 0 ) - 1 ) ;
        indicesVisited[ 1 ] = MIN2( indicesVisited[ 1 ] , rLayer.GetNumCells( 1 ) - 1 ) ;
        indicesVisited[ 2 ] = MIN2( indicesVisited[ 2 ] , rLayer.GetNumCells( 2 ) - 1 ) ;

        const unsigned          offset  = rLayer.OffsetFromIndices( indicesVisited ) ;
        const Vorton          & rVorton = rLayer[ offset ] ;

        VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , rVorton ) ;

        
        
        
        
    }

    float clusterRadius = 1.0f * mVortonRadius ;    // Experimental: estimate cluster radius based on level in influence tree.

    const size_t numLayers = mInfluenceTree.GetDepth() ;
    for( size_t uLayer = 0 /* base */ ; uLayer < numLayers - 1 ; ++ uLayer )
    {   // For each layer in influence tree...
        UniformGrid< Vorton > & rLayer  = mInfluenceTree[ uLayer ] ;

        // The outermost layer of the grid contains no vortices.
        // Any queries that walk along that outer layer see no influence.
        // Make sure all queries start with indices that lie strictly within the region that contains influence data.
        indicesVisited[ 0 ] = MIN2( indicesVisited[ 0 ] , rLayer.GetNumCells( 0 ) - 1 ) ;
        indicesVisited[ 1 ] = MIN2( indicesVisited[ 1 ] , rLayer.GetNumCells( 1 ) - 1 ) ;
        indicesVisited[ 2 ] = MIN2( indicesVisited[ 2 ] , rLayer.GetNumCells( 2 ) - 1 ) ;

        const unsigned  offset  = rLayer.OffsetFromIndices( indicesVisited ) ;
        Vorton          vorton = rLayer[ offset ] ;
        // With this technique, we want to calculate the velocity due to siblings of this vorton
        // (not due to this vorton itself).
        vorton.mPosition  = vorton.mPositionSiblings ;

        vorton.SetVorticity( vorton.mVorticitySiblings ) ;

        //vorton.SetRadius( vorton.mRadiusSiblings ) ;

// Experimental: Estimate cluster radius based on level in influence tree, and adjust vorticity accordingly (to preserve total circulation)
vorton.SetRadius( clusterRadius ) ;
const float vorticityReduction = mVortonRadius / clusterRadius ;
vorton.SetVorticity( vorton.mVorticitySiblings * POW3( vorticityReduction ) ) ;

        // Compute velocity induced by this supervorton.
        // Accumulate influence, storing in velocityAccumulator.
        VORTON_ACCUMULATE_VELOCITY( velocityAccumulator , vPosition , vorton ) ;

        
        
        

        // Compute indices into parent layer
        mInfluenceTree.GetParentIndices( indicesVisited , indicesVisited , uLayer + 1 ) ;
        #if 0 && defined( _DEBUG )
        {
            unsigned indicesParentTest[4] ;
            mInfluenceTree[ uLayer + 1 ].IndicesOfPosition( indicesParentTest , vPosition ) ;
        }
        #endif
    #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
    #endif

        clusterRadius *= 1.4142135623730950488016887242097f ; // Experimental: Estimate cluster radius based on level in influence tree
    }

    //

    return velocityAccumulator ;
}
#endif




/*! \brief Compute velocity due to vortons, for a subset of points in a uniform grid

    \param izStart - starting value for z index

    \param izEnd - ending value for z index

    \see CreateInfluenceTree, ComputeVelocityGrid

    \note This routine assumes CreateInfluenceTree has already executed,
            and that the velocity grid has been allocated.

*/
void VortonSim::ComputeVelocityGridSlice( unsigned izStart , unsigned izEnd )
{
    const size_t        numLayers   = mInfluenceTree.GetDepth() ;
    const Vec3 &        vMinCorner  = mVelGrid.GetMinCorner() ;
    static const float  nudge       = 1.0f - 2.0f * FLT_EPSILON ;
    const Vec3          vSpacing    = mVelGrid.GetCellSpacing() * nudge ;
    const unsigned      dims[3]     =   { mVelGrid.GetNumPoints( 0 )
                                        , mVelGrid.GetNumPoints( 1 )
                                        , mVelGrid.GetNumPoints( 2 ) } ;
    const unsigned      numXY       = dims[0] * dims[1] ;
    unsigned            idx[ 3 ] ;
    for( idx[2] = izStart ; idx[2] < izEnd ; ++ idx[2] )
    {   // For subset of z index values...
        Vec3 vPosition ;
        // Compute the z-coordinate of the world-space position of this gridpoint.
        vPosition.z = vMinCorner.z + float( idx[2] ) * vSpacing.z ;
        // Precompute the z contribution to the offset into the velocity grid.
        const unsigned offsetZ = idx[2] * numXY ;
        for( idx[1] = 0 ; idx[1] < dims[1] ; ++ idx[1] )
        {   // For every gridpoint along the y-axis...
            // Compute the y-coordinate of the world-space position of this gridpoint.
            vPosition.y = vMinCorner.y + float( idx[1] ) * vSpacing.y ;
            // Precompute the y contribution to the offset into the velocity grid.
            const unsigned offsetYZ = idx[1] * dims[0] + offsetZ ;
            for( idx[0] = 0 ; idx[0] < dims[0] ; ++ idx[0] )
            {   // For every gridpoint along the x-axis...
                // Compute the x-coordinate of the world-space position of this gridpoint.
                vPosition.x = vMinCorner.x + float( idx[0] ) * vSpacing.x ;
                // Compute the offset into the velocity grid.
                const unsigned offsetXYZ = idx[0] + offsetYZ ;

                // Compute the fluid flow velocity at this gridpoint, due to all vortons.
            #if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT
                mVelGrid[ offsetXYZ ] = ComputeVelocityDirect( vPosition ) ;
            #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE
                
                
                
                static const unsigned zeros[3] = { 0 , 0 , 0 } ; // Starter indices for recursive algorithm
                mVelGrid[ offsetXYZ ] = ComputeVelocityTree( vPosition , zeros , numLayers - 1  ) ;
                #if ( USE_TBB == 0 )
                #endif
            #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
                mVelGrid[ offsetXYZ ] = ComputeVelocityMonopoles( idx , vPosition ) ;
            #elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL
                numLayers ; // Reference variable to avoid compiler warning.
            #else
                #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
            #endif
            }
        }
    }
#if VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES
    
#endif
}




/*! \brief Populate a UniformGrid with vorticity values from vortons

    \note   This routine populates the mVorticityGrid with the NEGATIVE
            of vorticity.  This is because the Poisson equation we want to
            solve has the form

                Laplacian vectorPotential = - vorticity

*/
void VortonSim::PopulateNegativeVorticityGridFromVortons( UniformGrid< Vec3 > & vorticityGrid )
{
    vorticityGrid.Clear() ;                            // Clear any stale vorticity information
    vorticityGrid.CopyShape( mGridTemplate ) ;         // Use same shape as base vorticity grid. (Note: could differ if you want.)
    vorticityGrid.Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ; // Reserve memory for velocity grid and initialize all values to zero.

    

    // In order for the Poisson and Tree/Monopole velocity-from-vorticity 
    // techniques to produce consistent results, this formula for the volume of
    // a vorton must match that implicitly used in VORTON_ACCUMULATE_VELOCITY_private.
    const float cellVolume          = vorticityGrid.GetCellVolume() ;
    const float vortonVolume        = POW3( mVortonRadius ) * FourPiOver3 ;
    // The grid resulting from this operation must preserve the
    // same total circulation that the vortons have, so multiply
    // by the ratio of the vorton volume to gridcell volume.
    const float volumeCorrection    = vortonVolume / cellVolume ;

    // Populate vorticity grid.
    const size_t numVortons = mVortons->Size() ;
    for( size_t uVorton = 0 ; uVorton < numVortons ; ++ uVorton )
    {   // For each vorton in this simulation...
        const Vorton     &  rVorton     = (*mVortons)[ uVorton ] ;
        const Vec3       &  rPosition   = rVorton.mPosition   ;
        

        // NOTE the MINUS here.  See note in header comment.
        // Also note that the use of a uniform volumeCorrection here
        // assumes that all vortons have the same volume.
        Vec3 vCirculationCorrectedVorticity = - rVorton.GetVorticity() * volumeCorrection ;
        vorticityGrid.Accumulate( rPosition , vCirculationCorrectedVorticity ) ;

        
    }

}




/*! \brief Compute velocity due to vortons, for every point in a uniform grid

    \see CreateInfluenceTree

    \note This routine assumes CreateInfluenceTree has already executed.

*/
void VortonSim::ComputeVelocityGrid( void )
{
    mVelGrid.Clear() ;                      // Clear any stale velocity information
    mVelGrid.CopyShape( mGridTemplate ) ;   // Use same shape as base vorticity grid. (Note: could differ if you want.)
    mVelGrid.Init() ;                       // Reserve memory for velocity grid.

    const unsigned numZ = mVelGrid.GetNumPoints( 2 ) ;

#if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES )
    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  MAX2( 1 , numZ / gNumberOfProcessors ) ;
        // Compute velocity grid using multiple threads.
        parallel_for( tbb::blocked_range<size_t>( 0 , numZ , grainSize ) , VortonSim_ComputeVelocityGrid_TBB( this ) ) ;
    #else
        ComputeVelocityGridSlice( 0 , numZ ) ;
    #endif
#elif ( ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL ) )
    {
    #if USE_MULTI_GRID
        NestedGrid< Vec3 >      vectorPotentialMultiGrid( mGridTemplate ) ;
        UniformGrid< Vec3 > &   vectorPotential = vectorPotentialMultiGrid[0] ;
    #else
        UniformGrid< Vec3 >     vectorPotential( mGridTemplate ) ;
    #endif

        QUERY_PERFORMANCE_ENTER ;
        {
            #if USE_MULTI_GRID
                #if 1
                    unsigned maxValidDepth = 0 ;
                    for( unsigned iLayer = 1 ; iLayer < mVorticityMultiGrid.GetDepth() ; ++ iLayer )
                    {
                        const unsigned minDim = MIN3( mVorticityMultiGrid[ iLayer ].GetNumPoints( 0 ) , mVorticityMultiGrid[ iLayer ].GetNumPoints( 1 ) , mVorticityMultiGrid[ iLayer ].GetNumPoints( 2 ) ) ;
                        if( minDim > 2 )
                        {
                            mVorticityMultiGrid.DownSampleInto( iLayer ) ;
                            vectorPotentialMultiGrid.DownSampleInto( iLayer ) ;
                            SolveVectorPoisson( vectorPotentialMultiGrid[ iLayer ] , mVorticityMultiGrid[ iLayer ] , 3 ) ;
                        }
                        else
                        {
                            maxValidDepth = iLayer - 1 ;
                        }
                    }
                    for( unsigned iLayer = maxValidDepth ; iLayer >= 1 ; -- iLayer )
                    {
                        vectorPotentialMultiGrid.UpSampleFrom( iLayer ) ;
                        SolveVectorPoisson( vectorPotentialMultiGrid[ iLayer - 1 ] , mVorticityMultiGrid[ iLayer - 1 ] , 3 ) ;
                    }
                #else
                    vectorPotentialMultiGrid[0].Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
                    SolveVectorPoisson( vectorPotentialMultiGrid[0] , mVorticityMultiGrid[ 0 ] ) ;
                #endif
            #else
                vectorPotential.Init( Vec3( 0.0f , 0.0f , 0.0f ) ) ;
                SolveVectorPoisson( vectorPotential , mVorticityGrid ) ;
            #endif
        }
        QUERY_PERFORMANCE_EXIT( VortonSim_ComputeVelocityGrid_SolveVectorPoisson ) ;

        UniformGrid< Mat33 > jacobian( mVelGrid ) ;

        QUERY_PERFORMANCE_ENTER ;
        jacobian.Init() ;
        // TODO: It would be expedient here to have a routine to compute curl directly from vectorPotential, skipping Jacobian.
        ComputeJacobian( jacobian , vectorPotential ) ;
        QUERY_PERFORMANCE_EXIT( VortonSim_ComputeVelocityGrid_ComputeJacobian ) ;

        QUERY_PERFORMANCE_ENTER ;
        ComputeCurlFromJacobian( mVelGrid , jacobian ) ;
        QUERY_PERFORMANCE_EXIT( VortonSim_ComputeVelocityGrid_ComputeCurlFromJacobian ) ;
    }
#else
    #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
#endif
}




/*! \brief Stretch and tilt vortons using velocity field

    \param timeStep - amount of time by which to advance simulation

    \param uFrame - frame counter

    \see GenerateBaroclinicVorticity, DiffuseVorticityPSE

    \see J. T. Beale, A convergent three-dimensional vortex method with
            grid-free stretching, Math. Comp. 46 (1986), 401-24, April.

    \note This routine assumes CreateInfluenceTree has already executed.

    \note This routine should run before advecting vortons, otherwise the vortons
            could leave the velocity grid boundaries.

*/
void VortonSim::StretchAndTiltVortons( const float & timeStep , const unsigned & uFrame )
{
#if ENABLE_FLUID_BODY_SIMULATION

    if(     ( 0.0f == mVelGrid.GetExtent().x )
        ||  ( 0.0f == mVelGrid.GetExtent().y )
        ||  ( 0.0f == mVelGrid.GetExtent().z ) )
    {   // Domain is 2D, so stretching & tilting does not occur.
        return ;
    }

    // Compute all gradients of all components of velocity.
    UniformGrid< Mat33 > velocityJacobianGrid( mVelGrid ) ;
    velocityJacobianGrid.Init() ;
    ComputeJacobian( velocityJacobianGrid , mVelGrid ) ;


    const size_t numVortons = mVortons->Size() ;

    for( unsigned offset = 0 ; offset < numVortons ; ++ offset )
    {   // For each vorton...
        Vorton &    rVorton     = (*mVortons)[ offset ] ;
        Mat33       velJac      ;
        velocityJacobianGrid.Interpolate( velJac , rVorton.mPosition ) ;
        // Compute stretching & tilting:
    #if 1
        const Vec3  stretchTilt = rVorton.mAngularVelocity * velJac ;    // ...using transpose formulation
    #else
        const Vec3  stretchTilt = velJac * rVorton.mAngularVelocity ;    // ...using "classical" formulation.
    #endif
        rVorton.mAngularVelocity += 0.5f * stretchTilt * timeStep ;
    }

#endif
}




/*! \brief Populate a UniformGrid with density values from vortons

    \param densityGrid - (in/out) grid into which to accumulate density
                        deviation values.  Note that any values already in
                        densityGrid are added to, not overwritten.  It is
                        therefore possible to accumulate density from multiple
                        sources, such as multiple particle sets.

    \param particles - dynamic array of particles whose density information to
                        accumulate into densityGrid.

*/
void VortonSim::PopulateDensityGrid( UniformGrid< float > & densityGrid , const Vector< Particle > & particles )
{
    densityGrid.Clear() ;                       // Clear any stale density information.
    densityGrid.CopyShape( mGridTemplate ) ;    // Use same shape as base vorticity grid. (Note: could differ if you want.)
    densityGrid.Init( 0.0f )               ;    // Reserve memory for density grid and initialize all values to zero.

    

    // The grid resulting from this operation must preserve the same total mass 
    // that the particles represent, so multiply by the ratio of the particle 
    // volume to gridcell volume.
    const float oneOverCellVolume   = 1.0f / densityGrid.GetCellVolume() ;

    // Populate density grid.
    const size_t numParticles = particles.Size() ;
    for( size_t uParticle = 0 ; uParticle < numParticles ; ++ uParticle )
    {   // For each particle in the array...
        const Particle  &   rParticle   = particles[ uParticle ] ;
        const Vec3      &   rPosition   = rParticle.mPosition   ;

        

        // Note that the formula below uses Particle::GetMassDeviation
        // instead of Particle::GetMass( ambientDensity ).
        // This is because we first want to accumulate the
        // deviation from ambient only.  The uniform ambient density
        // is irrelevant to the gradient anyway.  Also, this loop
        // only accumulates a quantity where particles exist,
        // but in the absence of particles, the fluid has mass anyway.
        // Note that this effectively means that particles with negative mass deviation
        // are "vacuums", and when multiple "vacuum" particles overlap,
        // their "vacuum" combines.  This can result to a region with negative
        // density (negative mass) which is, of course, nonsense.
        // The proper solution to this would be to populate this grid with
        // actual density, not density deviation, that is, to use rParticle.GetMass( ambientDensity )
        // instead of rParticle.GetMassDeviation().  But that contradicts
        // the notion of having an ambient density at all, because if you accumulate
        // actual density anywhere, then that effectively means you've double-accumulated
        // the ambient density.  Equivalently, you'd need to know where you've accumulated
        // particles and where you have not, then go back-fill the regions without
        // particles and supply an ambient density.
        // 
        // The proper solution to this would be to make sure there are particles everywhere,
        // and that the particles provide all mass information.  But for the sake
        // of speed and memory, we want to be able to have sparse particles.
        // Furthermore this fiction of "vacuum" particles only creates nonsense where
        // multiple such particles overlap, and that nonsense only creates numerical
        // problems where the density then becomes negative -- and then only when
        // we need the actual mass (instead of deviation) to compute something.
        // For example, whem computing density gradient, the negative mass does
        // not cause problems.

        const float volumeCorrectedDensity = rParticle.GetMassDeviation() * oneOverCellVolume ;
        densityGrid.Accumulate( rPosition , volumeCorrectedDensity ) ;

        // Likewise here we only want to accumulate the mass /deviations/ of particles,
        // since this will be compared later to the mass /deviations/ on the grid.
        
    }


}




/*! \brief Compute baroclinic generation of vorticity due to buoyancy

    \param timeStep - amount of time by which to advance simulation

    \param iPclStart - index of first particle to process

    \param iPclEnd - index of last particle to process

    \see DiffuseVorticityPSE, StretchAndTiltVortons

    \note This routine assumes CreateInfluenceTree has already executed,
            specifically that the velocity grid shape has already been calculated.

    \note This routine should run before advecting vortons, otherwise the vortons
            could leave the velocity grid boundaries, and this routine needs to interpolate
            within those boundaries.

*/
void VortonSim::GenerateBaroclinicVorticitySlice( float timeStep , size_t iPclStart , size_t iPclEnd )
{
#if 0
    const float oneOverDensity = 1.0f / GetAmbientDensity() ;
#else
    const float oneOverDensity = 10.0f ;
#endif
    for( size_t iPcl = iPclStart ; iPcl < iPclEnd ; ++ iPcl )
    {   // For each particle in this slice...
        Vorton &    rVorton         = (*mVortons)[ iPcl ] ;
        // Technically the baroclinic term should have density in the
        // denominator, but it's relatively expensive to calculate the local
        // value, plus the Boussinesq approximation assumes density variations
        // are small compared to the ambient.  Also, when density variations are
        // large, the interpolation sometimes leads to non-physical artifacts
        // like negative densities.  So instead, we use the ambient density in
        // the denominator, which yields the correct units and gets results in
        // the right ballpark.
        // This is yet another example of where, for visual effects,
        // we take drastic liberties in the name of speed over accuracy.
        //float       density ;
        //mDensityDeviationGrid.Interpolate( density , rVorton.mPosition ) ;
        //density += GetAmbientDensity() ;
        //
        Vec3        densityGradient ;
        mDensityGradientGrid.Interpolate( densityGradient , rVorton.mPosition ) ;
        const Vec3  baroclinicGeneration = densityGradient ^ mGravAccel * oneOverDensity ;
        // The formula for the line below is meant to update vorticity,
        // but since we store that as angular velocity, the formula has an extra factor of 0.5.
        rVorton.mAngularVelocity += 0.5f * baroclinicGeneration * timeStep ;
    }
}




/*! \brief Compute baroclinic generation of vorticity due to buoyancy

    \param timeStep - amount of time by which to advance simulation

    \param uFrame - frame counter

    \see DiffuseVorticityPSE, StretchAndTiltVortons

    \note This routine assumes CreateInfluenceTree has already executed,
            specifically that the velocity grid has already been calculated.

    \note This routine should run before advecting vortons, otherwise the vortons
            could leave the velocity grid boundaries.

*/
void VortonSim::GenerateBaroclinicVorticity( const float & timeStep , const unsigned & uFrame )
{
    (void) uFrame ; // Avoid "unreferenced local parameter" warning.

    if( fabsf( mGravAccel * mVelGrid.GetExtent() ) < FLT_EPSILON )
    {   // Domain is 2D and has no significant component along the gravity direction,
        // so baroclinic generation cannot occur in this Boussinesq approximation.
        return ;
    }

    // Populate density grid.
    //
    // NOTE: Particles carry density /deviation/ information so this routine really
    // only populates densityGrid with density deviation about the ambient.  The
    // actual density anywhere in the fluid is the value obtained from
    // densityGrid, plus the ambient fluid density.
    //
    // Passing mVortons to PopulateDensityGrid assumes Particle and Vorton
    // layout are identical.  This is a weird, fragile use of inheritance but I
    // desperately want to avoid virtual calls, while for pedagogic purposes I
    // want to distinguish between a regular tracer particle and a vortex 
    // particle, and still treat arrays of those things uniformly, for economy 
    // of code.

    PopulateDensityGrid( mDensityDeviationGrid , reinterpret_cast< Vector< Particle > & >( * mVortons ) ) ;


    // Compute density gradient.
    mDensityGradientGrid.Clear() ;                      // Clear any stale density gradient information
    mDensityGradientGrid.CopyShape( mGridTemplate ) ;   // Use same shape as base velocity grid. (Note: could differ if you want.)
    mDensityGradientGrid.Init() ;                       // Reserve memory for density gradient grid.
    ComputeGradient( mDensityGradientGrid , mDensityDeviationGrid ) ;

    const size_t    numVortons      = mVortons->Size() ;

#if ENABLE_FLUID_BODY_SIMULATION
    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  MAX2( 1 , numVortons / gNumberOfProcessors ) ;
        // Compute baroclinic generation of vorticity using threading building blocks
        parallel_for( tbb::blocked_range<size_t>( 0 , numVortons , grainSize ) , VortonSim_GenerateBaroclinicVorticity_TBB( timeStep , this ) ) ;
    #else
        GenerateBaroclinicVorticitySlice( timeStep , 0 , numVortons ) ;
    #endif
#endif
}




/*! \brief Partition vortons

    This routine partitions space into cells using the same grid
    as the "base vorton" grid.  Each vorton gets assigned to the
    cell that contains it.

    \param timeStep - amount of time by which to advance simulation

    \param uFrame - frame counter

    \param ugVortonIndices - Uniform grid of vorton indices.  Assign each cell
        to contain a vector of the indices of vortons that reside inside that cell.

    \see StretchAndTiltVortons, GenerateBaroclinicVorticity

    \note This routine assumes CreateInfluenceTree has already executed.

*/
void VortonSim::PartitionVortons( const float & timeStep , const unsigned & uFrame , UniformGrid< Vector< unsigned > > & ugVortonIndices )
{
    (void) uFrame ; // Avoid "unreferenced formal parameter" warning.
#if ENABLE_FLUID_BODY_SIMULATION

    const size_t numVortons = mVortons->Size() ;

    for( unsigned offset = 0 /* Start at 0th vorton */ ; offset < numVortons ; ++ offset )
    {   // For each vorton...
        Vorton &    rVorton         = (*mVortons)[ offset ] ;
        // Insert the vorton's offset into the spatial partition.
        ugVortonIndices[ rVorton.mPosition ].PushBack( offset ) ;
    }

#endif
}




/*! \brief Exchange vorticity or merge vortons

    \return true if particle was kept, false if deleted
*/
inline bool VortonSim::ExchangeVorticityOrMergeVortons( const unsigned & rVortIdxHere , Vorton & rVortonHere , Vec3 & rAngVelHere , const unsigned & ivThere , Vector< unsigned > & cell , const float & timeStep )
{
    const unsigned &    rVortIdxThere   = cell[ ivThere ] ;
    Vorton &            rVortonThere    = (*mVortons)[ rVortIdxThere ] ;
    if( rVortonThere.IsAlive() )
    {   // Particle is alive.
        //const Vec3          displacement    = rVortonHere.mPosition - rVortonThere.mPosition ;
        //const float         dist2           = displacement.Mag2() ;
        //const float         radiusSum       = ( rVortonHere.mSize + rVortonThere.mSize ) * 0.5f ; // 1/2 because size=2*radius
        //const float         radiusSum2      = POW2( radiusSum ) ;
        //static float        mergeThreshold  = 0.25f ; // A value of 0.25 means the center of 1 vorton is just barely inside the core of another.
        //if( dist2 < radiusSum2 * mergeThreshold )
        //{   // Vortices are close enough to merge.
        //    Particles::Merge( reinterpret_cast< Vector< Particle > & >( * mVortons ) , rVortIdxHere , rVortIdxThere , mAmbientDensity ) ;
        //    // Remove particle from cell.
        //    
        //    cell[ ivThere ] = cell[ cell.Size() - 1 ] ;
        //    cell.PopBack() ;
        //    return false ;   // Tell caller a particle was deleted.
        //}
        //else
        {   // Vortices are far enough to remain separate.
            Vec3 &              rAngVelThere    = rVortonThere.mAngularVelocity ;
            const Vec3          vortDiff        = rAngVelHere - rAngVelThere ;
            const Vec3          exchange        = 2.0f * mViscosity * timeStep * vortDiff ;    // Amount of vorticity to exchange between particles.
            rAngVelHere  -= exchange ;   // Make "here" vorticity a little closer to "there".
            rAngVelThere += exchange ;   // Make "there" vorticity a little closer to "here".

            return true ;  // Tell caller particles were kept.
        }
    }
    return true ;  // Tell caller particles were kept.
}




/*! \brief Diffuse vorticity using a particle strength exchange (PSE) method, for a slice of the domain

    \see DiffuseVorticityPSE
*/
void VortonSim::DiffuseVorticityPSESlice( const float & timeStep , UniformGrid< Vector< unsigned > > & ugVortonIndices , size_t izStart , size_t izEnd )
{
    // Exchange vorticity with nearest neighbors

    const size_t   & nx     = ugVortonIndices.GetNumPoints( 0 ) ;
    const size_t     nxm1   = nx - 1 ;
    const size_t   & ny     = ugVortonIndices.GetNumPoints( 1 ) ;
    const size_t     nym1   = ny - 1 ;
    const size_t     nxy    = nx * ny ;

    
    

    size_t idx[3] ;
    for( idx[2] = izStart ; idx[2] < izEnd ; ++ idx[2] )
    {   // For all points along z except the last...
        const size_t offsetZ0 = idx[2]         * nxy ;
        const size_t offsetZp = ( idx[2] + 1 ) * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all points along y except the last...
            const size_t offsetY0Z0 =   idx[1]       * nx + offsetZ0 ;
            const size_t offsetYpZ0 = ( idx[1] + 1 ) * nx + offsetZ0 ;
            const size_t offsetY0Zp =   idx[1]       * nx + offsetZp ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all points along x except the last...
                const size_t offsetX0Y0Z0 = idx[0]     + offsetY0Z0 ;
                for( unsigned ivHere = 0 ; ivHere < ugVortonIndices[ offsetX0Y0Z0 ].Size() ; ++ ivHere )
                {   // For each vorton in this gridcell...
                    const unsigned &    rVortIdxHere    = ugVortonIndices[ offsetX0Y0Z0 ][ ivHere ] ;
                    Vorton &            rVortonHere     = (*mVortons)[ rVortIdxHere ] ;
                    // Note: This algorithm assumes calls to ExchangeVorticityOrMergeVortons never
                    // delete rVortonHere.
                    if( rVortonHere.IsAlive() )
                    {
                        Vec3 &              rAngVelHere     = rVortonHere.mAngularVelocity ;

                        // Diffuse vorticity with other vortons in this same cell.
                        // Notice that the loop only visits vortons with indices
                        // following the "here" vorton.  That is because each
                        // visitation symmetrically modifies both vortons in the
                        // pair.  If "here" and "there" is visited, then "there" and
                        // "here" should not also be visited, because it would be
                        // redundant.
                        for( unsigned ivThere = ivHere + 1 ; ivThere < ugVortonIndices[ offsetX0Y0Z0 ].Size() ; /* Conditionally increment in body. */ )
                        {   // For each OTHER vorton within this same cell...
                            if( ExchangeVorticityOrMergeVortons( rVortIdxHere , rVortonHere , rAngVelHere , ivThere , ugVortonIndices[ offsetX0Y0Z0 ] , timeStep ) )
                            {   // Both particles remain.
                                ++ ivThere ;
                            }
                        }

                        // Diffuse vorticity with vortons in adjacent cells.
                        // Note that each of the following sections visits only
                        // cells in the positive direction.  That is because each
                        // visitation is symmetric so there is no need to visit
                        // vortons in cells in the negative direction.
                        {
                            const size_t offsetXpY0Z0 = idx[0] + 1 + offsetY0Z0 ; // offset of adjacent cell in +X direction
                            for( unsigned ivThere = 0 ; ivThere < ugVortonIndices[ offsetXpY0Z0 ].Size() ; /* Conditionally increment in body. */ )
                            {   // For each vorton in the adjacent cell in +X direction...
                                if( ExchangeVorticityOrMergeVortons( rVortIdxHere , rVortonHere , rAngVelHere , ivThere , ugVortonIndices[ offsetXpY0Z0 ] , timeStep ) )
                                {   // Both particles remain.
                                    ++ ivThere ;
                                }
                            }
                        }

                        {
                            const size_t offsetX0YpZ0 = idx[0]     + offsetYpZ0 ; // offset of adjacent cell in +Y direction
                            for( unsigned ivThere = 0 ; ivThere < ugVortonIndices[ offsetX0YpZ0 ].Size() ; /* Conditionally increment in body. */ )
                            {   // For each vorton in the adjacent cell in +Y direction...
                                if( ExchangeVorticityOrMergeVortons( rVortIdxHere , rVortonHere , rAngVelHere , ivThere , ugVortonIndices[ offsetX0YpZ0 ] , timeStep ) )
                                {   // Both particles remain.
                                    ++ ivThere ;
                                }
                            }
                        }

                        {
                            const size_t offsetX0Y0Zp = idx[0]     + offsetY0Zp ; // offset of adjacent cell in +Z direction
                            for( unsigned ivThere = 0 ; ivThere < ugVortonIndices[ offsetX0Y0Zp ].Size() ; /* Conditionally increment in body. */ )
                            {   // For each vorton in the adjacent cell in +Z direction...
                                if( ExchangeVorticityOrMergeVortons( rVortIdxHere , rVortonHere , rAngVelHere , ivThere , ugVortonIndices[ offsetX0Y0Zp ] , timeStep ) )
                                {   // Both particles remain.
                                    ++ ivThere ;
                                }
                            }
                        }

                        // Dissipate vorticity.  See notes in header comment for DiffuseVorticityPSE,
                        // related to dissipation at Kolmogorov scales.  Technically, that should
                        // get converted into heat, but the amounts are usually negligible.
                        // This also simulates diffusing vorticity into regions of fluid that
                        // have no vortons.  A more accurate approach would involve creating vortons
                        // in those regions, so that vorticity could be tracked better, but that would
                        // dramatically slow the simulation, yielding results qualitatively similar to
                        // what this formula does.
                        rAngVelHere  -= mViscosity * timeStep * rAngVelHere ;   // Reduce vorticity here.
                    }
                }
            }
        }
    }
}




/*! \brief Diffuse vorticity using a particle strength exchange (PSE) method.

    Each vorton exchanges some of its vorticity with its neighbors in its own
    and adjacent cells.

    This routine makes some simplifying assumptions to speed execution:

        -   Distance does not influence the amount of vorticity exchanged,
            except in as much as only vortons within a certain region of
            each other exchange vorticity.  This amounts to saying our kernel,
            eta, is a top-hat function.

        -   Theoretically, if an adjacent cell contains no vortons
            then this simulation should generate vorticity within
            that cell, e.g. by creating a new vorton in the adjacent cell.

        -   This simulation reduces the vorticity of each vorton, alleging that
            this vorticity is dissipated analogously to how energy dissipates at
            Kolmogorov microscales.  This treatment is not realistic but it
            retains qualitative characteristics that we want, e.g. that the flow
            dissipates at a rate related to viscosity. Dissipation in real flows
            is a more complicated phenomenon.

    \see Degond & Mas-Gallic (1989): The weighted particle method for
        convection-diffusion equations, part 1: the case of anisotropic viscosity.
        Math. Comput., v. 53, n. 188, pp. 485-507, October.

    \param timeStep - amount of time by which to advance simulation

    \param uFrame - frame counter

    \param ugVortonIndices - UniformGrid of Vorton indices.  \see ParititionVortons.

    \see StretchAndTiltVortons, GenerateBaroclinicVorticity, DiffuseHeatPSE

*/
void VortonSim::DiffuseVorticityPSE( const float & timeStep , const unsigned & uFrame , UniformGrid< Vector< unsigned > > & ugVortonIndices )
{
    (void) uFrame ; // Avoid "unreferenced formal parameter" warning.
#if ENABLE_FLUID_BODY_SIMULATION


    // Exchange vorticity with nearest neighbors

    const unsigned & nz     = ugVortonIndices.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  MAX2( 1 , nzm1 / gNumberOfProcessors ) ;
        // Compute vorticity diffusion using threading building blocks
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_DiffuseVorticityPSE_TBB( timeStep , this , ugVortonIndices ) ) ;
    #else
        DiffuseVorticityPSESlice( timeStep , ugVortonIndices , 0 , nzm1 ) ;
    #endif



#endif
}




/*! \brief Exchange heat between 2 vortons.

    \param rVortIdxHere Index within mVortons of vorton at "here" location.

    \param rVortonHere  Reference to mVortons[ rVortIdxHere ]

    \param  rDensityHere    Reference to mVortons[ rVortIdxHere ].mDensity

    \param  ivThere     Index within mVortons of vorton at "there" location.

    \param  cell    Reference to cell containing "here" vorton.

    \param  timeStep    Amount of virtual time by which to advance the simulation.

    This routine uses the particle strength exchange method to evolve temperature, T,
    according to the heat diffusion equation:

        DT/Dt = kappa laplacian T

    where D/Dt represents the convective derivative (where t is time),
    kappa represents thermal diffusivity of the fluid and "laplacian"
    is the laplacian differential operator (del^2, a.k.a. nabla^2).

    \note   Under the Boussinesq approximation,

                    density = ambientDensity * ( 1 - thermalExpansionCoefficient * ( temperature - ambientTemperature ) )
                --> temperature = density / ( ambientDensity * ( 1 - thermalExpansionCoefficient ) ) + ambientTemperature

            If the thermalExpansionCoefficient is constant with respect to density then
            density and temperature are linearly related (but with opposite sign).

            For an ideal gas, thermalExpansionCoefficient=1/temperature so the above formula becomes

                density = ambientDensity * ambientTemperature / temperature
                --> temperature = ambientTemperature * ambientDensity / density

            In this case, density and temperature are inversely proportional.

            Either way, as temperature increases, density decreases.
*/
inline void VortonSim::ExchangeHeat( const unsigned & rVortIdxHere , Vorton & rVortonHere , float & rDensityHere , const unsigned & ivThere , const Vector< unsigned > & cell , const float & timeStep )
{
    const unsigned &    rVortIdxThere   = cell[ ivThere ] ;
    Vorton &            rVortonThere    = (*mVortons)[ rVortIdxThere ] ;
    if( rVortonThere.IsAlive() )
    {   // Particle is alive.
        //const Vec3          displacement    = rVortonHere.mPosition - rVortonThere.mPosition ;
        //const float         dist2           = displacement.Mag2() ;
        //const float         radiusSum       = ( rVortonHere.mSize + rVortonThere.mSize ) * 0.5f ; // 1/2 because size=2*radius
        //const float         radiusSum2      = POW2( radiusSum ) ;
        float &             rDensityThere   = rVortonThere.mDensity ;
        const float         densityDiff     = rDensityHere - rDensityThere ;
        const float         exchange        = 2.0f * mThermalDiffusivity * timeStep * densityDiff ;    // Amount of heat to exchange between particles.
        rDensityHere  -= exchange ;   // Make "here"  temperature a little closer to "there".
        rDensityThere += exchange ;   // Make "there" temperature a little closer to "here".
    }
}




/*! \brief Diffuse heat using a particle strength exchange (PSE) method, for a slice of the domain

    \see DiffuseVorticityPSE, DiffuseHeatPSE, PartitionVortons
*/
void VortonSim::DiffuseHeatPSESlice( const float & timeStep , const UniformGrid< Vector< unsigned > > & ugVortonIndices , size_t izStart , size_t izEnd )
{
    // Exchange heat with nearest neighbors

    const size_t   & nx     = ugVortonIndices.GetNumPoints( 0 ) ;
    const size_t     nxm1   = nx - 1 ;
    const size_t   & ny     = ugVortonIndices.GetNumPoints( 1 ) ;
    const size_t     nym1   = ny - 1 ;
    const size_t     nxy    = nx * ny ;

    
    

    size_t idx[3] ;
    for( idx[2] = izStart ; idx[2] < izEnd ; ++ idx[2] )
    {   // For all points along z except the last...
        const size_t offsetZ0 = idx[2]         * nxy ;
        const size_t offsetZp = ( idx[2] + 1 ) * nxy ;
        for( idx[1] = 0 ; idx[1] < nym1 ; ++ idx[1] )
        {   // For all points along y except the last...
            const size_t offsetY0Z0 =   idx[1]       * nx + offsetZ0 ;
            const size_t offsetYpZ0 = ( idx[1] + 1 ) * nx + offsetZ0 ;
            const size_t offsetY0Zp =   idx[1]       * nx + offsetZp ;
            for( idx[0] = 0 ; idx[0] < nxm1 ; ++ idx[0] )
            {   // For all points along x except the last...
                const size_t offsetX0Y0Z0 = idx[0]     + offsetY0Z0 ;
                for( unsigned ivHere = 0 ; ivHere < ugVortonIndices[ offsetX0Y0Z0 ].Size() ; ++ ivHere )
                {   // For each vorton in this gridcell...
                    const unsigned &    rVortIdxHere    = ugVortonIndices[ offsetX0Y0Z0 ][ ivHere ] ;
                    Vorton &            rVortonHere     = (*mVortons)[ rVortIdxHere ] ;
                    if( rVortonHere.IsAlive() )
                    {
                        float &             rDensityHere    = rVortonHere.mDensity ;

                        // Diffuse heat with other vortons in this same cell.
                        // Notice that the loop only visits vortons with indices
                        // following the "here" vorton.  That is because each
                        // visitation symmetrically modifies both vortons in the
                        // pair.  If "here" and "there" is visited, then "there" and
                        // "here" should not also be visited, because it would be
                        // redundant.
                        for( unsigned ivThere = ivHere + 1 ; ivThere < ugVortonIndices[ offsetX0Y0Z0 ].Size() ; ++ ivThere )
                        {   // For each OTHER vorton within this same cell...
                            ExchangeHeat( rVortIdxHere , rVortonHere , rDensityHere , ivThere , ugVortonIndices[ offsetX0Y0Z0 ] , timeStep ) ;
                        }

                        // Diffuse heat with vortons in adjacent cells.
                        // Note that each of the following sections visits only
                        // cells in the positive direction.  That is because each
                        // visitation is symmetric so there is no need to visit
                        // vortons in cells in the negative direction.
                        {
                            const size_t offsetXpY0Z0 = idx[0] + 1 + offsetY0Z0 ; // offset of adjacent cell in +X direction
                            for( unsigned ivThere = 0 ; ivThere < ugVortonIndices[ offsetXpY0Z0 ].Size() ; ++ ivThere )
                            {   // For each vorton in the adjacent cell in +X direction...
                                ExchangeHeat( rVortIdxHere , rVortonHere , rDensityHere , ivThere , ugVortonIndices[ offsetXpY0Z0 ] , timeStep ) ;
                            }
                        }

                        {
                            const size_t offsetX0YpZ0 = idx[0]     + offsetYpZ0 ; // offset of adjacent cell in +Y direction
                            for( unsigned ivThere = 0 ; ivThere < ugVortonIndices[ offsetX0YpZ0 ].Size() ; ++ ivThere )
                            {   // For each vorton in the adjacent cell in +Y direction...
                                ExchangeHeat( rVortIdxHere , rVortonHere , rDensityHere , ivThere , ugVortonIndices[ offsetX0YpZ0 ] , timeStep ) ;
                            }
                        }

                        {
                            const size_t offsetX0Y0Zp = idx[0]     + offsetY0Zp ; // offset of adjacent cell in +Z direction
                            for( unsigned ivThere = 0 ; ivThere < ugVortonIndices[ offsetX0Y0Zp ].Size() ; ++ ivThere )
                            {   // For each vorton in the adjacent cell in +Z direction...
                                ExchangeHeat( rVortIdxHere , rVortonHere , rDensityHere , ivThere , ugVortonIndices[ offsetX0Y0Zp ] , timeStep ) ;
                            }
                        }

                        // "Dissipate" heat.
                        // This simulates diffusing heat into regions of fluid that have no explicit particles.
                        // The proper solution to this would involve either radiating heat into and out of the environment,
                        // and/or adding fluid particles to regions that lack them, but that would consume computational resources,
                        // and yield qualitatively very similar results.  Remember, this is for a video game eye candy, not
                        // science and engineering.
                        rDensityHere  -= mThermalDiffusivity * timeStep * rDensityHere ;   // Reduce heat deviation here.
                    }
                }
            }
        }
    }
}




/*! \brief Diffuse heat using a particle strength exchange (PSE) method.

    \see DiffuseVorticityPSE

*/
void VortonSim::DiffuseHeatPSE( const float & timeStep , const unsigned & uFrame , const UniformGrid< Vector< unsigned > > & ugVortonIndices )
{
    (void) uFrame ; // Avoid "unreferenced formal parameter" warning.
#if ENABLE_FLUID_BODY_SIMULATION

    // Exchange heat with nearest neighbors

    const unsigned & nz     = ugVortonIndices.GetNumPoints( 2 ) ;
    const unsigned   nzm1   = nz - 1 ;

    #if USE_TBB
        // Estimate grain size based on size of problem and number of processors.
        const size_t grainSize =  MAX2( 1 , nzm1 / gNumberOfProcessors ) ;
        // Compute heat diffusion using threading building blocks
        parallel_for( tbb::blocked_range<size_t>( 0 , nzm1 , grainSize ) , VortonSim_DiffuseHeatPSE_TBB( timeStep , this , ugVortonIndices ) ) ;
    #else
        DiffuseHeatPSESlice( timeStep , ugVortonIndices , 0 , nzm1 ) ;
    #endif

#endif
}




/*! \brief Update vortex particle fluid simulation to next time.

    \param timeStep - incremental amount of time to step forward

    \param uFrame - frame counter, used to generate files

    \note FindBoundingBox and UpdateBoundingBox must have been called
            before this routine starts, for each timestep.

    Effectively this routine generates the velocity field and prepares
    the simulation for the next step.  It does NOT, however, actually
    advect vortons.  The Particles::Advect routine does that,
    and it is up to the caller of this routine to invoke Particles::Advect.
    This separation of processes facilitates adding other motion
    to the vortons which are not due to the fluid simulation.

*/
void VortonSim::Update( float timeStep , unsigned uFrame )
{

#if ( ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE ) || ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES ) )
    QUERY_PERFORMANCE_ENTER ;
    CreateInfluenceTree() ;
    QUERY_PERFORMANCE_EXIT( VortonSim_CreateInfluenceTree ) ;
#elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL
    QUERY_PERFORMANCE_ENTER ;
    #if USE_MULTI_GRID
        mVorticityMultiGrid.Initialize( mGridTemplate ) ;
        PopulateNegativeVorticityGridFromVortons( mVorticityMultiGrid[ 0 ] ) ;
    #else
        PopulateNegativeVorticityGridFromVortons( mVorticityGrid ) ;
    #endif
    QUERY_PERFORMANCE_EXIT( VortonSim_PopulateVorticityGrid ) ;
#elif VELOCITY_TECHNIQUE != VELOCITY_TECHNIQUE_DIRECT
    #error Velocity technique is invalid or undefined.  Assign VELOCITY_TECHNIQUE in vorton.h or change this code.
#endif

    QUERY_PERFORMANCE_ENTER ;
    ComputeVelocityGrid() ;
    QUERY_PERFORMANCE_EXIT( VortonSim_ComputeVelocityGrid ) ;


    QUERY_PERFORMANCE_ENTER ;
    StretchAndTiltVortons( timeStep , uFrame ) ;
    QUERY_PERFORMANCE_EXIT( VortonSim_StretchAndTiltVortons ) ;

    QUERY_PERFORMANCE_ENTER ;
    GenerateBaroclinicVorticity( timeStep , uFrame ) ;
    QUERY_PERFORMANCE_EXIT( VortonSim_GenerateBaroclinicVorticity ) ;

    // Create a spatial partition for the vortons.
    // Each cell will contain a dynamic array of integers offsets into mVortons.
    UniformGrid< Vector< unsigned > > ugVortonIndices( mGridTemplate ) ;
    ugVortonIndices.Init() ;

    QUERY_PERFORMANCE_ENTER ;
    PartitionVortons( timeStep , uFrame , ugVortonIndices ) ;
    QUERY_PERFORMANCE_EXIT( VortonSim_PartitionVortons ) ;

    QUERY_PERFORMANCE_ENTER ;
    DiffuseVorticityPSE( timeStep , uFrame , ugVortonIndices ) ;
    QUERY_PERFORMANCE_EXIT( VortonSim_DiffuseVorticityPSE ) ;

    QUERY_PERFORMANCE_ENTER ;
    DiffuseHeatPSE( timeStep , uFrame , ugVortonIndices ) ;
    QUERY_PERFORMANCE_EXIT( VortonSim_DiffuseHeatPSE ) ;

    QUERY_PERFORMANCE_ENTER ;
    Particles::KillParticlesMarkedForDeath( reinterpret_cast< Vector< Particle > & >( * mVortons ) ) ;
    QUERY_PERFORMANCE_EXIT( VortonSim_KillParticlesMarkedForDeath ) ;
}
