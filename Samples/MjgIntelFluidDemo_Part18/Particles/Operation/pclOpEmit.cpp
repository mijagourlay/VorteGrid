//#pragma optimize( "" , off )
/** \file pclOpEmit.cpp

    \brief Operation to emit particles.

    \author Written and copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/
*/

#include "Particles/Operation/pclOpEmit.h"

#include "Particles/particle.h"

#include "Core/Performance/perfBlock.h"

#include <Core/SpatialPartition/uniformGridMath.h>

#include <stdlib.h>
#include <limits>



static const float  sOneMinusEpsilon        = 1.0f - FLT_EPSILON ;




/** Populate density deviation grid from particles that carry mass.

    This routine assumes densityDeviationGrid has a shape that encompasses all the given particles.
*/
void PopulateDensityDeviationGridFromParticles( const VECTOR< Particle > & particles , const float ambientFluidDensity , UniformGrid< float > & densityDeviationGrid )
{
    PERF_BLOCK( PopulateDensityDeviationGridFromParticles ) ;

    ASSERT( densityDeviationGrid.Empty() ) ;
    densityDeviationGrid.Init( 0.0f );

    // Populate density deviation grid based on particles (which have density).
    // Note that this is only a rough approximation of a proper density grid.
    // Here we only want to know whether a given cell has deviant density,
    // to get the overall shape of the density distribution.
    // This density deviation grid will NOT have the same total mass as the particles; it lacks the volume-correction factor.
    const size_t numParticles= particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {   // For each particle...
        const Particle  &   rPcl        = particles[ iPcl ] ;
        const Vec3      &   rPosition   = rPcl.mPosition   ;
        ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;
        DEBUG_ONLY( const unsigned  uOffset = densityDeviationGrid.OffsetOfPosition( rPosition ) ) ;
        ASSERT( uOffset < densityDeviationGrid.GetGridCapacity() ) ;
        const float densityDeviation = rPcl.mDensity - ambientFluidDensity ;
        densityDeviationGrid.Accumulate( rPosition , densityDeviation ) ;
    }
}




/** Create particles within the given region.

    \param particles    (out) Dynamic array of particles to populate.

    \param uniformGrid  Region within which to emit particles.

    \param multiplier   Number of particles to emit along each dimension
                        within each cell of the given uniformGrid.
                        This is the cube root of the number of particles to emit
                        per cell.

    \param vortons      Optional address of Dynamic array of vortons.  If
                        provided then particles are only emitted where vortons exist.
                        This allows for placing tracers at the location of vortons.

*/
void PclOpEmit::Emit( VECTOR< Particle > & particles , const UniformGridGeometry & uniformGrid , unsigned multiplier , const VECTOR< Particle > * vortons )
{
    PERF_BLOCK( PclOpEmit__Emit ) ;

    static const float ambientFluidDensity = 1.0f ; // TODO: FIXME: Pass in as a parameter.

    const Vec3      vSpacing        = uniformGrid.GetCellSpacing() ;
    // Must keep particles away from maximal boundary by at least cell.  Note the +vHalfSpacing in loop.
    //const unsigned  begin[3]        = { 1*uniformGrid.GetNumCells(0)/8 , 1*uniformGrid.GetNumCells(1)/8 , 1*uniformGrid.GetNumCells(2)/8 } ;
    //const unsigned  end[3]          = { 7*uniformGrid.GetNumCells(0)/8 , 7*uniformGrid.GetNumCells(1)/8 , 7*uniformGrid.GetNumCells(2)/8 } ;
    //const unsigned  begin[3]        = { 1,1,1 } ;
    //const unsigned  end[3]          = { uniformGrid.GetNumCells(0) - 1 , uniformGrid.GetNumCells(1) - 1 , uniformGrid.GetNumCells(2) - 1 } ;
    const unsigned  begin[3]        = { 0,0,0 } ;
    const unsigned  end[3]          = { uniformGrid.GetNumCells(0) , uniformGrid.GetNumCells(1) , uniformGrid.GetNumCells(2) } ;
    const float     pclSize         = 0.5f * powf( vSpacing.x * vSpacing.y * vSpacing.z , 1.0f / 3.0f ) / float( multiplier ) ;
    unsigned        idx[3]          ;

    const unsigned  nt[3]           = { multiplier , multiplier , multiplier } ;
    bool            emitParticle    = true ;

    UniformGrid< float > densityDeviationGrid  ; // used to mask out tracer particles
    float densityDeviationMin =  FLT_MAX ;
    float densityDeviationMax = -FLT_MAX ;

    if( vortons )
    {   // Caller provided vortons, to seed tracers.
        densityDeviationGrid.Clear() ;
        densityDeviationGrid.CopyShape( uniformGrid ) ;

        PopulateDensityDeviationGridFromParticles( static_cast< const VECTOR< Particle > & >( * vortons ) , ambientFluidDensity , densityDeviationGrid ) ;
        FindValueRange( densityDeviationGrid , densityDeviationMin , densityDeviationMax ) ;
    }
    const float densityDeviationScale   = Max2( fabsf( densityDeviationMax ) , fabsf( densityDeviationMin ) ) ;
    float       probMax                 = 0.0f ;

    // Shift each particle to center the distribution within each cell.
    const Vec3  vShift                  = 0.5f * vSpacing / float( multiplier ) ;

    // Jitter should distribute particles within their little subdomains,
    // and must not place particles outside grid, even by a tiny amount.
    const Vec3  jitterMagnitude         = 1.9999f * sOneMinusEpsilon * vShift ;

    for( idx[2] = begin[2] ; idx[2] < end[2] ; ++ idx[2] )
    for( idx[1] = begin[1] ; idx[1] < end[1] ; ++ idx[1] )
    for( idx[0] = begin[0] ; idx[0] < end[0] ; ++ idx[0] )
    {   // For each grid cell...
        Vec3 vPosMinCorner ;
        uniformGrid.PositionFromIndices( vPosMinCorner , idx ) ;
        Particle pcl ;
        pcl.mVelocity	        = Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mOrientation	    = Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mAngularVelocity	= Vec3( 0.0f , 0.0f , 0.0f ) ;
        pcl.mDensity            = ambientFluidDensity ;
    #if ENABLE_FIRE
        pcl.mFuelFraction       = 0.0f ;
        pcl.mFlameFraction      = 0.0f ;
        pcl.mSmokeFraction      = 1.0f ;
    #endif
        pcl.mSize		        = pclSize ;
        pcl.mBirthTime          = 0 ;
        //const unsigned  offset    = densityDeviationGrid.OffsetFromIndices( idx ) ;

        if( ( ! vortons )                           // Either vorton array was not provided...
            ||  ( densityDeviationScale != 0.0f )   // ...or it was provided and there is mass variation in this domain.
            )
        {   // Want to place tracer particles here.
            unsigned it[3] ;
            for( it[2] = 0 ; it[2] < nt[2] ; ++ it[2] )
            for( it[1] = 0 ; it[1] < nt[1] ; ++ it[1] )
            for( it[0] = 0 ; it[0] < nt[0] ; ++ it[0] )
            {   // For each subdomain within each grid cell...
                const Vec3 vDisplacement(   float( it[0] ) / float( nt[0] ) * vSpacing.x ,
                                            float( it[1] ) / float( nt[1] ) * vSpacing.y ,
                                            float( it[2] ) / float( nt[2] ) * vSpacing.z ) ;

                const Vec3 jitter = RandomSpread( jitterMagnitude ) ;
                pcl.mPosition   = vPosMinCorner + vDisplacement + vShift + jitter ;
                if( vortons )
                {   // Vortons were passed in so assign density based on
                    float densityDeviationHere ;
                    densityDeviationGrid.Interpolate( densityDeviationHere , pcl.mPosition ) ;
                    // Convert from additive deviation to multiplicative gain. (Inverse of above.)
                    pcl.mDensity = ( densityDeviationHere + densityDeviationScale ) * ambientFluidDensity ;
                    ASSERT( pcl.mDensity > 0.0f ) ;
                    const float probability = fabsf( densityDeviationHere / densityDeviationScale ) ;
                    probMax = Max2( probMax , probability ) ;
                    //const float dieRoll     = float( rand() ) / float( RAND_MAX ) ;
                    //emitParticle = dieRoll <= probability ;
                    emitParticle = probability >= 0.5f ;
                }
                if( emitParticle )
                {
                    ASSERT( pcl.mDensity > 0.0f ) ;
                    particles.PushBack( pcl ) ;
                }
            }
        }
    }
}




void PclOpEmit::Operate(  VECTOR< Particle > & particles , float timeStep , unsigned uFrame )
{
    PERF_BLOCK( PclOpEmit__Operate ) ;

    const float fNumToEmit = timeStep * mEmitRate + mRemainder ;
    const int   iNumToEmit = int( fNumToEmit ) ;

    // Remember fractional particles for future emission.
    mRemainder = fNumToEmit - float( iNumToEmit ) ;
    ASSERT( ( timeStep < 0.0f ) || ( mRemainder >= 0.0f ) ) ;

    for( int iPcl = 0 ; iPcl < iNumToEmit ; ++ iPcl )
    {   // For each new particle to emit...
        particles.PushBack( mTemplate ) ;
        Particle & rParticleNew = particles.Back() ;
        rParticleNew.mPosition          += RandomSpread( mSpread.mPosition          ) ;
        rParticleNew.mVelocity          += RandomSpread( mSpread.mVelocity          ) ;
        rParticleNew.mOrientation       += RandomSpread( mSpread.mOrientation       ) ;
        rParticleNew.mAngularVelocity   += RandomSpread( mSpread.mAngularVelocity   ) ;
        rParticleNew.mDensity           += RandomSpread( mSpread.mDensity           ) ;
    #if ENABLE_FIRE
        rParticleNew.mFuelFraction      += RandomSpread( mSpread.mFuelFraction      ) ;
        rParticleNew.mFlameFraction     += RandomSpread( mSpread.mFlameFraction     ) ;
        rParticleNew.mSmokeFraction     += RandomSpread( mSpread.mSmokeFraction     ) ;
    #endif
        rParticleNew.mSize              += RandomSpread( mSpread.mSize              ) ;
        rParticleNew.mBirthTime          = uFrame ;
        ASSERT( rParticleNew.mDensity >= 0.0f ) ;
    }
}








////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Move code below to its own file.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/** Level-set method:

    Given an externally-defined velocity field (the fluid velocity), track the interface surface between 2 regions (e.g. air and water).

    This entails using an Eulerian (grid-based) representation of the surface in the form of a signed distance function,
    and evolving (advecting) that function over time.

    Particle level-set method:

    Use passive tracer particles to enhance the Eulerian level-set.  It does not replace th Eulerian grid; it enhances its accuracy.

    How does this apply to this fluid simulation?

    This fluid sim provides the velocity field.

    It also happens (currently) to provide additional information that is somewhat redundant with what a level-set method aims to solve.
    For example, the SPH formulation already implicitly determines where liquid is.
    So why bother with a level-set?

    The SPH particles are too large and too few to define a reasonable surface.
    We would want to render a finer-detailed surface.
    A level-set method could help with that.

    For the VPM simulation, again, the vortons don't sufficiently define, for example, where dye would be.

    Why not just advect passive tracers?
    That is done now. But the passive tracers leave behind trails that don't look good.

    But level-set requires a grid, and these simulations don't have a persistent grid.  How could this use level-set?

    We could define the level set only for the duration of a time-step, during which this simulation does create auxiliary grids.
    Then there would not be a need for the Eulerian advection step, per se.  The SDF would just get created from the density data.
    Note: This is an untested idea.  It would seem a bit incestuous, to rely on the passive tracers, only, to define the SDF.

    Another option could be to use the SPH/VPM mother particles to define mass (as is done now) and base the level-set on those,
    at each time step, then use tracers to enhance its resolution on a finer temporary grid.

    Both of these ideas "miss the point" of the particle level-set method, which is that the particles exist only to enhance the Eulerian level-set.
    In other words, the level-set still defines the surface, but the particles "correct" the level-set function.
    If that level-set is not defined independently then the particles would not serve the same purpose.

    So, this idea of using passive tracers to define the level-set is new and not on firm ground.

    One questions whether we should not just use the density grid to define the surface.  Perhaps trying to get a finer-resolution surface (e..g from tracers) is hopeless.

    Still, it might be worth exercising this if only for the sake of implementing a level-set method.
    Also, it might turn out that the finer-resolution tracers will indeed yield a better surface.
    Just because these ideas are half-baked doesn't mean they're useless.  It just means we don't know yet.

    A good test case for the VPM simulation: Have a blob of dye somewhere other than where the vortons reside.

    Things I could do:

    Write routines:
        Seed:
            Input: vortons.
            Populate a grid with density values from vortons.
            Use that to seed tracer particles.

        Reseed:
            Input: density grid, tracers.
            Use density grid to determine which tracers are too far from density isosurface.
            Delete/move tracers that are too far but on the correct side.  These are just useless.
            Delete/move tracers that are too far and on the wrong side.  These potentially indicate anomalies in the front isosurface, and in particle-level-set are used to update the level-set function.
                This routine could be the basis for a more traditional particle-level-set method, where either vortons aren't mass-carriers but just velocity field generators, or in an eulerian system, or whatever.

        Advect:
            Input: grid of whatever values to advect, velocity grid
            Use velocity grid to advect UniformGrid< whatever > using Eulerian approach, e.g. backtracking + interpolation

*/






/** \file pclOpSeedSurfaceTracers.cpp

    \brief Operation to place particles within a narrow band on either side of a surface.

    \see http://www.mijagourlay.com/

    \author Copyright 2013-2013 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "Particles/Operation/pclOpEmit.h" // TODO: Change to pclOpSeedSurfaceTracers.h after splitting file.

#include "Particles/Operation/pclOpFindBoundingBox.h"

#include "Particles/particle.h"

#include "Core/Performance/perfBlock.h"
//#include <Core/SpatialPartition/uniformGridMath.h>

#include <stdlib.h>

#include "Core/useTbb.h"




/** Indices for a box of voxels, used to partition processing of a volume.
*/
struct Block
{
    int mBlockIndices[ 3 ] ;    // Indices of this block.

    Block()
    {
    #if defined( _DEBUG )
        // Initialize indices to an invalid value to ensure caller sets them later.
        mBlockIndices[ 0 ] = -1 ;
        mBlockIndices[ 1 ] = -1 ;
        mBlockIndices[ 2 ] = -1 ;
    #endif
    }


    Block( const int ix , const int iy , const int iz )
    {
        mBlockIndices[ 0 ] = ix ;
        mBlockIndices[ 1 ] = iy ;
        mBlockIndices[ 2 ] = iz ;
    }

    /** Compute index of first voxel in block, for given axis.
    */
    int BeginR( size_t axis , const int voxelsPerBlock[ 3 ] , const int numGridPoints[ 3 ] , const int inc[ 3 ] ) const
    {
        // When running backwards, beginning index is at maximal corner of block.
        const int shift         = ( inc[ axis ] > 0 ) ? 0 : 1 ;
        // First compute unconstrained index -- as if all blocks are fully occupied.
        const int unconstrained = ( mBlockIndices[ axis ] + shift ) * voxelsPerBlock[ axis ] ;
        // Finally, constrain index to account for blocks that are not fully occupied by voxels.
        // That can happen when numGridPoints is not an integer multiple of voxelsPerBlock.
        return Min2( unconstrained , numGridPoints[ axis ] - 1 ) ;
    }

    /** Compute index of one-past-last voxel in block, for given axis.
    */
    int EndR( size_t axis , const int voxelsPerBlock[ 3 ] , const int numGridPoints[ 3 ] , const int inc[ 3 ] ) const
    {
        const int begin = BeginR( axis , voxelsPerBlock , numGridPoints , inc ) ;
        if( inc[ axis ] > 0 )
        {   // Running forward.  End is (one past) maximal corner of block.  Here, "past" means "after".
            const int unconstrained = begin + voxelsPerBlock[ axis ] ;
            return Min2( unconstrained , numGridPoints[ axis ] ) ;
        }
        else
        {   // Running backward.  End is (one past) minimal corner of block.  Here, "past" means "before".
            const int unconstrained = begin - voxelsPerBlock[ axis ] - 1 ;
            return Max2( unconstrained , -1 ) ;
        }
    }


    /** Compute indices of first and one-past-last voxels in block, for all axes.
    */
    void SetRanges( int begin[ 3 ] , int end[ 3 ] , const int voxelsPerBlock[ 3 ] , const int numGridPoints[ 3 ] , const int inc[ 3 ] ) const
    {
        begin[ 0 ] = BeginR( 0 , voxelsPerBlock , numGridPoints , inc ) ;
        begin[ 1 ] = BeginR( 1 , voxelsPerBlock , numGridPoints , inc ) ;
        begin[ 2 ] = BeginR( 2 , voxelsPerBlock , numGridPoints , inc ) ;
        end[ 0 ] = EndR( 0 , voxelsPerBlock , numGridPoints , inc ) ;
        end[ 1 ] = EndR( 1 , voxelsPerBlock , numGridPoints , inc ) ;
        end[ 2 ] = EndR( 2 , voxelsPerBlock , numGridPoints , inc ) ;
        ASSERT( abs( begin[ 0 ] - end[ 0 ] ) <= voxelsPerBlock[ 0 ] + 1 ) ;
        ASSERT( abs( begin[ 1 ] - end[ 1 ] ) <= voxelsPerBlock[ 1 ] + 1 ) ;
        ASSERT( abs( begin[ 2 ] - end[ 2 ] ) <= voxelsPerBlock[ 2 ] + 1 ) ;
    }
} ;




#if USE_TBB

#   if _DEBUG

    static TbbAtomicBool sDebugMutex                 ;   /// Mutex lock, used to synchronize debug output.

    static void DebugLock()
    {
        while( sDebugMutex.fetch_and_store( true ) ) {} ; // Wait for lock
    }

    static void DebugRelease()
    {
        sDebugMutex = false ; // Release lock.
    }

#   endif
#else // Not USE_TBB
#   define DebugLock()
#   define DebugRelease()
#endif

#if USE_TBB

/** Functor (function object) to compute SDF for remainder of domain, using Threading Building Blocks.
*/
class ComputeRemainingSDFFromImmediateSDF_Nearest_TBB
{
        UniformGrid< float > &      mSignedDistanceGrid         ;   /// Grid of SDF values. This procedure solves for those value which are not pinned, constrained to those values which are pinned.
        const UniformGrid< int > &  mSdfPinnedGrid              ;   /// Grid of flags indicating which SDF values are "pinned", meaning their values is not meant to change during this solver.  They indicate essential boundary conditions.
        int                         mInc[ 3 ]                   ;   /// Processing loop increment for each axis.  +1 means forward, -1 means backward.
        int                         mVoxelsPerBlock[ 3 ]        ;   /// Number of voxels per block, along each axis.
        tbb::atomic< int > *        mPendingPredecessorCount    ;   /// Number of predecessors each block depends on, which have not yet been processed.

        void ShowBlockPredecessors() const
        {
            DEBUG_ONLY( DebugPrintf( "ShowBlockPredecessors:\n" ) ) ;

            int ind[ 3 ] ;
            for( ind[1] = 0 ; ind[1] != NumBlocks(1) ; ++ ind[1] )
            {   // For each row...
                for( ind[2] = 0 ; ind[2] != NumBlocks(2) ; ++ ind[2] )
                {   // For each plane...
                    DEBUG_ONLY( DebugPrintf( "    " ) ) ;
                    for( ind[0] = 0 ; ind[0] != NumBlocks(0) ; ++ ind[0] )
                    {   // For each column...
                        DEBUG_ONLY( DebugPrintf( "% 3i " , Predecessor( ind ) ) ) ;
                    }
                }
                DEBUG_ONLY( DebugPrintf( "\n" ) ) ;
            }
        }

        void FeedBlockToLoop( tbb::parallel_do_feeder< Block > & feeder , const int nextIndices[ 3 ] , const int axis , int ix , int iy , int iz ) const
        {
            DEBUG_ONLY( DebugLock() ) ;
            if( nextIndices[ axis ] != EndBlock( axis ) )
            {   // Next block along given direction is still within domain.
                // Atomically decrement pending-predecessor count for that block.
                if( -- Predecessor( ix , iy , iz ) == 0 )
                {   // Pending-predecessor count for adjacent block reached zero, so it is ready to process.
                    // Add that block to feeder so parallel_do will process it.

                    feeder.add( Block( ix , iy , iz ) ) ;
                    DEBUG_ONLY( DebugPrintf( "FeedBlockToLoop: %i %i %i (%i/%i)\n" , ix , iy , iz , ix + NumBlocks( 0 ) * ( iy + NumBlocks( 1 ) * iz ) , NumBlocks(0) * NumBlocks(1) * NumBlocks(2) ) ) ;
                    DEBUG_ONLY( ShowBlockPredecessors() ) ;
                }
            }
            DEBUG_ONLY( DebugRelease() ) ; // Release lock.
        }

    public:
        void operator() ( const Block & block , tbb::parallel_do_feeder< Block > & feeder ) const
        {   // ...
            void ComputeRemainingSDFFromImmediateSDF_Nearest_Block( UniformGrid< float > & signedDistanceGrid , const UniformGrid< int > & sdfPinnedGrid , const int voxelsPerBlock[ 3 ] , const int inc[ 3 ] , const Block & block ) ;

            SetFloatingPointControlWord( mMasterThreadFloatingPointControlWord ) ;
            SetMmxControlStatusRegister( mMasterThreadMmxControlStatusRegister ) ;
            ComputeRemainingSDFFromImmediateSDF_Nearest_Block( mSignedDistanceGrid , mSdfPinnedGrid , mVoxelsPerBlock , mInc , block ) ;
            int nextIndices[ 3 ] = { block.mBlockIndices[ 0 ] + mInc[ 0 ]
                                   , block.mBlockIndices[ 1 ] + mInc[ 1 ]
                                   , block.mBlockIndices[ 2 ] + mInc[ 2 ] } ;
            FeedBlockToLoop( feeder , nextIndices , 0 ,         nextIndices[ 0 ] , block.mBlockIndices[ 1 ] , block.mBlockIndices[ 2 ] ) ;
            FeedBlockToLoop( feeder , nextIndices , 1 , block.mBlockIndices[ 0 ] ,         nextIndices[ 1 ] , block.mBlockIndices[ 2 ] ) ;
            FeedBlockToLoop( feeder , nextIndices , 2 , block.mBlockIndices[ 0 ] , block.mBlockIndices[ 1 ] ,         nextIndices[ 2 ] ) ;
        }


        ComputeRemainingSDFFromImmediateSDF_Nearest_TBB( UniformGrid< float > & signedDistanceGrid ,  const UniformGrid< int > & sdfPinnedGrid , const int voxelsPerBlock[ 3 ] )
            : mSignedDistanceGrid( signedDistanceGrid )
            , mSdfPinnedGrid( sdfPinnedGrid )
            , mPendingPredecessorCount( NULL )
        {
            mMasterThreadFloatingPointControlWord = GetFloatingPointControlWord() ;
            mMasterThreadMmxControlStatusRegister = GetMmxControlStatusRegister() ;
            mVoxelsPerBlock[ 0 ] = voxelsPerBlock[ 0 ] ;
            mVoxelsPerBlock[ 1 ] = voxelsPerBlock[ 1 ] ;
            mVoxelsPerBlock[ 2 ] = voxelsPerBlock[ 2 ] ;
            mInc[ 0 ] = mInc[ 1 ] = mInc[ 2 ] = INT_MAX ;
            // Note: Caller must call Init after assigning mInc.
            ASSERT( ! sDebugMutex ) ;
        }


        tbb::atomic< int > & Predecessor( int ix , int iy , int iz ) const
        {
            const size_t offset = ix + NumBlocks( 0 ) * ( iy + NumBlocks( 1 ) * iz ) ;
            ASSERT( mPendingPredecessorCount[ offset ] >= 0 ) ;
            return mPendingPredecessorCount[ offset ] ;
        }


        tbb::atomic< int > & Predecessor( int ind[ 3 ] ) const
        {
            return Predecessor( ind[0] , ind[1] , ind[2] ) ;
        }


        int NumBlocks( int axis ) const
        {
            const int numBlocks = ( mSignedDistanceGrid.GetNumPoints( axis ) + mVoxelsPerBlock[ axis ] - 1 ) / mVoxelsPerBlock[ axis ] ;
            return numBlocks ;
        }


        void BeginBlock( int indices[ 3 ] ) const
        {
            for( int axis = 0 ; axis < 3 ; ++ axis )
            {
                if( mInc[ axis ] > 0 )
                {
                    ASSERT( 1 == mInc[ axis ] ) ;
                    indices[ axis ] = 0 ;
                }
                else
                {
                    ASSERT( -1 == mInc[ axis ] ) ;
                    ASSERT( NumBlocks( axis ) >= 1 ) ;
                    indices[ axis ] = NumBlocks( axis ) - 1 ;
                }
            }
        }


        int EndBlock( int axis ) const
        {
            if( mInc[ axis ] > 0 )
            {
                return NumBlocks( axis ) ;
            }
            else
            {
                return -1 ;
            }
        }


        void Init( int incX , int incY , int incZ )
        {
            // Initialize predecessor counts for blocks.
            const int numBlocksX = NumBlocks( 0 ) ;
            const int numBlocksY = NumBlocks( 1 ) ;
            const int numBlocksZ = NumBlocks( 2 ) ;

            const int numBlocks = numBlocksX * numBlocksY * numBlocksZ ;

            mInc[ 0 ] = incX ;
            mInc[ 1 ] = incY ;
            mInc[ 2 ] = incZ ;

            mPendingPredecessorCount = new tbb::atomic< int >[ numBlocks ] ;

            int beginBlock[ 3 ];
            BeginBlock( beginBlock ) ;

            for( int iz = 0 ; iz < numBlocksZ ; ++ iz )
            for( int iy = 0 ; iy < numBlocksY ; ++ iy )
            for( int ix = 0 ; ix < numBlocksX ; ++ ix )
            {   // For each block...
                const size_t offset = ix + numBlocksX * ( iy + numBlocksY * iz ) ;
                mPendingPredecessorCount[ offset ] = ( ix != beginBlock[0] ) + ( iy != beginBlock[1] ) + ( iz != beginBlock[2] ) ;
            }

            DEBUG_ONLY( DebugPrintf( "Init: inc=%i %i %i\n" , incX , incY , incZ ) ) ;
            DEBUG_ONLY( ShowBlockPredecessors() ) ;
        }


        static void Driver( UniformGrid< float > & signedDistanceGrid , const UniformGrid< int > & sdfPinnedGrid )
        {
#if 1
            static const int voxelsPerBlock[ 3 ] = { 8 , 8 , 8 } ;   // TODO: tune for optimal performance, balancing thread granularity (where smaller is better) with per-thread overhead (where larger is better).
#elif 1
            const int voxelsPerBlock[ 3 ] = { signedDistanceGrid.GetNumPoints( 0 )/2 , signedDistanceGrid.GetNumPoints( 1 )/2 , signedDistanceGrid.GetNumPoints( 2 ) } ;
#else
            // For testing only: Use a single block that spans entire domain.
            // This tests the overall TBB parallel_do machinery without the complication of having multiple threads or jobs.
            // The result should be identical to the serial version.
            const int voxelsPerBlock[ 3 ] = { signedDistanceGrid.GetNumPoints( 0 ) , signedDistanceGrid.GetNumPoints( 1 ) , signedDistanceGrid.GetNumPoints( 2 ) } ;
#endif

            ComputeRemainingSDFFromImmediateSDF_Nearest_TBB functor( signedDistanceGrid , sdfPinnedGrid , voxelsPerBlock ) ;

            static const int numPasses = 2 ;
            for( int pass = 0 ; pass < numPasses ; ++ pass )
            {   // For each pass...
                // Assign begin, end and increment for each axis.
                // Start at extremal corner.
                // Note that although this algorithm looks backwards from the direction of "travel",
                // it has to start at the first gridpoint even though it has no backward neighbors
                // *along that direction* because the other directions usually do have backward neighbors.
                // The backward-indexer below thus checks each index before accessing its backward neighbor.
                if( ( pass & 1 ) != 0 )
                {   // Odd-numbered pass.  Sweep in reverse direction.
                    functor.Init( -1 , -1 , -1 ) ;
                }
                else
                {   // Even-numbered pass.  Sweep in forward direction.
                    functor.Init( 1 , 1 , 1 ) ;
                }

                Block rootBlock ;

                // Compute indices of root block based on inc.
                functor.BeginBlock( rootBlock.mBlockIndices ) ;

                // Invoke parallel_do on rootBlock, using functor as the loop body.
                // The end argument passed in here is somewhat bogus;
                // parallel_do expects an InputIterator, but there is only one root block.
                // This call therefore passes in the address of the root block (for begin), and one element past it (for end).
                // parallel_do will treat this as an STL InputIterator, because it obeys all properties of one:
                // copy-constructible, copy-assignable, destructible, comparable using == and !=, dereferenceable, incrementable.
                // The only thing that matters here for the "end" value is that, once parallel_do "increments" the "iterator" and compares it to "end", the loop will terminate.
                tbb::parallel_do( & rootBlock , & rootBlock + 1 , functor ) ;
            }
        }

    private:
        WORD        mMasterThreadFloatingPointControlWord   ;
        unsigned    mMasterThreadMmxControlStatusRegister   ;
} ;



#endif




/* static */ const float PclOpSeedSurfaceTracers::sBandWidthAutomatic = - FLT_MAX ;




/** Assign sdfHere when its absolute value exceeds the absolute value of sdfCandidate.
*/
static inline void UpdateSdf( float & sdfHere , const float & sdfCandidate )
{
    if( fabsf( sdfCandidate ) < fabsf( sdfHere ) )
    {   // Found a closer value than before.
        sdfHere = sdfCandidate ;
    }
}




/**

     Identifying surfaces, for the case where the fluid interior has a density higher than the outside:

     Outside        Outside  Surface Inside          Inside          Inside        Inside  Surface  Outside
       Q               Z        S      A               M               M               B        S      Q
     --|---------------|--------+------|---------------|---------------|---------------|--------+------|--
      x_0             x_1      z_1    x_2             x_3             x_4             x_5      z_5    x_6
    
       |<-----h_0----->|

     Q are samples with the "outside" value of the density.
     A is a sample with value Z<A<M.  (For lighter-than-ambient fluids, this would be Q>A>m.)
     B is a sample with value Z<B<M.  (For lighter-than-ambient fluids, this would be Q>B>m.)
     S are estimates for where the indicator function transitions to "outside".
     M are samples with value M, the maximum value of the indicator function.  (For lighter-than-ambient fluids, this would be m for the minimum value.)
     h_i is the spacing between samples i and i+1.

     The surface lies at z, the point where the indicator function transitions to the ambient Q value.
     Assume the region between Q and A is such that the smaller A, the closer the surface is to where A was sampled.
     That is, assume that the value of x between Q and A where the surface occurs is linearly proportional to A/M:
       If I(x_i) < I(x_i+1):
           z = x_i + h_i * (1 - A/M)
       If I(x_i) > I(x_i+1):
           z = x_i + h_i * (    A/M)

    Assign values for I(x_i) and I(x_i+1) when each |value| exceeds the corresponding |candidate value|.

*/
void ComputeImmediateSDFFromDensity_Nearest( UniformGrid< float > & signedDistanceGrid
                                            , UniformGrid< int > & sdfPinnedGrid
                                            , const UniformGrid< float > & densityGrid
                                            , const float densityOutside )
{
    PERF_BLOCK( ComputeImmediateSDFFromDensity_Nearest ) ;

    ASSERT( signedDistanceGrid.ShapeMatches( densityGrid ) ) ;
    ASSERT( signedDistanceGrid.Size() == densityGrid.Size() ) ;
    ASSERT( signedDistanceGrid[ size_t( 0 ) ] == FLT_MAX ) ; // Quick-and-dirty check that the SDF is initialized to FLT_MAX.

    ASSERT( sdfPinnedGrid.ShapeMatches( densityGrid ) ) ;
    ASSERT( sdfPinnedGrid.Size() == densityGrid.Size() ) ;
    ASSERT( sdfPinnedGrid[ size_t( 0 ) ] == 0 ) ; // Quick-and-dirty check that the SDF-pinned is initialized to false.

    // Compute statistics for density.
    // Used for estimating surface locations.
    float densityMin , densityMax , densityMean, densityStdDev ;
    FindValueStats( densityGrid , densityMin , densityMax , densityMean , densityStdDev ) ;

    ASSERT( densityMin <= densityOutside ) ;

    const float densityInside = Min2( densityMax , densityMean + densityStdDev ) ;

    ASSERT( densityInside >= densityOutside ) ; // Currently only support finding surfaces of heavy fluids.
    ASSERT( densityMax >= densityInside ) ;

    const float densityDenominator = densityInside - densityOutside ;

    ASSERT( densityDenominator >= 0.0f ) ;

    const unsigned  begin[3]    = { 0,0,0 } ;
    const unsigned  end[3]      = { densityGrid.GetNumCells(0) , densityGrid.GetNumCells(1) , densityGrid.GetNumCells(2) } ;
    unsigned        idx[3]      ;
    const size_t    numX        = densityGrid.GetNumPoints( 0 ) ;
    const size_t    numXY       = numX * densityGrid.GetNumPoints( 1 ) ;

    //const float qNaN = std::numeric_limits< float >::quiet_NaN() ; // Used to initialize zeroCrossings to indicate none for that edge

    for( idx[2] = begin[2] ; idx[2] < end[2] ; ++ idx[2] )
    for( idx[1] = begin[1] ; idx[1] < end[1] ; ++ idx[1] )
    for( idx[0] = begin[0] ; idx[0] < end[0] ; ++ idx[0] )
    {   // For each grid cell...

        // Obtain density values at this gridpoint and its forward neighbors.
        const size_t    offsetX0Y0Z0  = densityGrid.OffsetFromIndices( idx ) ;
        const size_t    offsetX1Y0Z0  = offsetX0Y0Z0 + 1 ;
        const size_t    offsetX0Y1Z0  = offsetX0Y0Z0 + numX ;
        const size_t    offsetX0Y0Z1  = offsetX0Y0Z0 + numXY ;

        const float densityHere    = densityGrid[ offsetX0Y0Z0 ] - densityOutside ;
        const float densityX       = densityGrid[ offsetX1Y0Z0 ] - densityOutside ;
        const float densityY       = densityGrid[ offsetX0Y1Z0 ] - densityOutside ;
        const float densityZ       = densityGrid[ offsetX0Y0Z1 ] - densityOutside ;

        //float surfaceCrossings[ 3 ] = { FLT_MAX , FLT_MAX , FLT_MAX } ;

        float & sdfHere             = signedDistanceGrid[ offsetX0Y0Z0 ] ;
        float & sdfX                = signedDistanceGrid[ offsetX1Y0Z0 ] ;
        float & sdfY                = signedDistanceGrid[ offsetX0Y1Z0 ] ;
        float & sdfZ                = signedDistanceGrid[ offsetX0Y0Z1 ] ;

        // For each direction {x,y,z}, determine whether this edge has a surface crossing.
        // If it has a crossing, determine where along that edge.
        // Approximate SDF using smallest zero-crossing distance in any direction.
        // NOTE: This is not the best estimate.  A better estimate would be the shortest distance to the line segment or polygon whose vertices are the zero-crossings.
        // NOTE: Furthermore it should take into account zero-crossings along edges not adjacent to Here.

        if( ( densityHere <= 0.0f ) && ( 0.0f < densityX ) )
        {   // Found crossing along +X; here is OUTside.
            const float tweenX = Min2( densityX / densityDenominator , 1.0f ) ;
            ASSERT( ( 0.0f <= tweenX ) && ( tweenX <= 1.0f ) ) ;
            const float candidateHere = ( 1.0f - tweenX ) * densityGrid.GetCellSpacing().x ;
            const float candidateX    =        - tweenX   * densityGrid.GetCellSpacing().x ;
            sdfPinnedGrid[ offsetX0Y0Z0 ] = 1 ;
            sdfPinnedGrid[ offsetX1Y0Z0 ] = 1 ;
            UpdateSdf( sdfHere , candidateHere ) ;
            UpdateSdf( sdfX    , candidateX ) ;
        }
        else if( ( densityHere >  0.0f ) && ( 0.0f >= densityX ) )
        {   // Found crossing along +X; here is INside.
            const float tweenX = Min2( densityHere / densityDenominator , 1.0f ) ;
            ASSERT( ( 0.0f <= tweenX ) && ( tweenX <= 1.0f ) ) ;
            const float candidateHere =        - tweenX   * densityGrid.GetCellSpacing().x ;
            const float candidateX    = ( 1.0f - tweenX ) * densityGrid.GetCellSpacing().x ;
            sdfPinnedGrid[ offsetX0Y0Z0 ] = 1 ;
            sdfPinnedGrid[ offsetX1Y0Z0 ] = 1 ;
            UpdateSdf( sdfHere , candidateHere ) ;
            UpdateSdf( sdfX    , candidateX ) ;
        }

        if( ( densityHere <= 0.0f ) && ( 0.0f < densityY ) )
        {   // Found crossing along +Y; here is OUTside.
            const float tweenY = Min2( densityY / densityDenominator , 1.0f ) ;
            ASSERT( ( 0.0f <= tweenY ) && ( tweenY <= 1.0f ) ) ;
            const float candidateHere = ( 1.0f - tweenY ) * densityGrid.GetCellSpacing().y ;
            const float candidateY    =        - tweenY   * densityGrid.GetCellSpacing().y ;
            sdfPinnedGrid[ offsetX0Y0Z0 ] = 1 ;
            sdfPinnedGrid[ offsetX0Y1Z0 ] = 1 ;
            UpdateSdf( sdfHere , candidateHere ) ;
            UpdateSdf( sdfY    , candidateY ) ;
        }
        else if( ( densityHere >  0.0f ) && ( 0.0f >= densityY ) )
        {   // Found crossing along +Y; here is INside.
            const float tweenY = Min2( densityHere / densityDenominator , 1.0f ) ;
            ASSERT( ( 0.0f <= tweenY ) && ( tweenY <= 1.0f ) ) ;
            const float candidateHere =        - tweenY   * densityGrid.GetCellSpacing().y ;
            const float candidateY    = ( 1.0f - tweenY ) * densityGrid.GetCellSpacing().y ;
            sdfPinnedGrid[ offsetX0Y0Z0 ] = 1 ;
            sdfPinnedGrid[ offsetX0Y1Z0 ] = 1 ;
            UpdateSdf( sdfHere , candidateHere ) ;
            UpdateSdf( sdfY    , candidateY ) ;
        }

        if( ( densityHere <= 0.0f ) && ( 0.0f < densityZ ) )
        {   // Found crossing along +Z; here is OUTside.
            const float tweenZ = Min2( densityZ / densityDenominator , 1.0f ) ;
            ASSERT( ( 0.0f <= tweenZ ) && ( tweenZ <= 1.0f ) ) ;
            const float candidateHere = ( 1.0f - tweenZ ) * densityGrid.GetCellSpacing().z ;
            const float candidateZ    =        - tweenZ   * densityGrid.GetCellSpacing().z ;
            sdfPinnedGrid[ offsetX0Y0Z0 ] = 1 ;
            sdfPinnedGrid[ offsetX0Y0Z1 ] = 1 ;
            UpdateSdf( sdfHere , candidateHere ) ;
            UpdateSdf( sdfZ    , candidateZ ) ;
        }
        else if( ( densityHere >  0.0f ) && ( 0.0f >= densityZ ) )
        {   // Found crossing along +Z; here is INside.
            const float tweenZ = Min2( densityHere / densityDenominator , 1.0f ) ;
            ASSERT( ( 0.0f <= tweenZ ) && ( tweenZ <= 1.0f ) ) ;
            const float candidateHere =        - tweenZ   * densityGrid.GetCellSpacing().z ;
            const float candidateZ    = ( 1.0f - tweenZ ) * densityGrid.GetCellSpacing().z ;
            sdfPinnedGrid[ offsetX0Y0Z0 ] = 1 ;
            sdfPinnedGrid[ offsetX0Y0Z1 ] = 1 ;
            UpdateSdf( sdfHere , candidateHere ) ;
            UpdateSdf( sdfZ    , candidateZ ) ;
        }
    }

#define DIAGNOSE_IMMEDIATE_SDF 0
#if DIAGNOSE_IMMEDIATE_SDF // For diagnostic rendering (only), make a final pass to change FLT_MAX to NaN.  Otherwise, when rendering, the immediate SDF gets swamped by FLT_MAX values.

    for( idx[2] = begin[2] ; idx[2] <= end[2] ; ++ idx[2] )
    for( idx[1] = begin[1] ; idx[1] <= end[1] ; ++ idx[1] )
    for( idx[0] = begin[0] ; idx[0] <= end[0] ; ++ idx[0] )
    {   // For each grid cell...
        // Obtain SDF value at this gridpoint.
        const size_t    offsetX0Y0Z0  = signedDistanceGrid.OffsetFromIndices( idx ) ;
        if( FLT_MAX == signedDistanceGrid[ offsetX0Y0Z0 ] )
        {
            signedDistanceGrid[ offsetX0Y0Z0 ] = std::numeric_limits< float >::quiet_NaN() ;
            ASSERT( 0 == sdfPinnedGrid[ offsetX0Y0Z0 ] ) ;
        }
        else
        {
            ASSERT( 1 == sdfPinnedGrid[ offsetX0Y0Z0 ] ) ;
        }
    }
#endif
}




/** Assign sdfHere when its absolute value exceeds the absolute value of the candidate based on its neighbor.
*/
static inline void UpdateSdfFromNeighbor_Nearest( float & sdfHere , const float sdfNeighbor , const float cellSpacing )
{
    const float delta = FCopySign( cellSpacing , sdfNeighbor ) ;
    const float candidateSdf = sdfNeighbor + delta ;
    if( fabsf( candidateSdf ) < fabsf( sdfHere ) )
    {
        sdfHere = candidateSdf ;
    }
}




/** Compute indices for voxel first visited in a backward-difference iteration, given the loop increment.
*/
void ComputeGridBeginEnd( int begin[ 3 ] , int end[ 3 ] , const UniformGrid< float > & grid , const int inc[ 3 ] )
{
    for( int axis = 0 ; axis < 3 ; ++ axis )
    {
        if( inc[ axis ] > 0 )
        {
            ASSERT( 1 == inc[ axis ] ) ;
            begin[ axis ] = 0 ;
            end[ axis ] = grid.GetNumPoints( axis ) ;
        }
        else
        {
            ASSERT( -1 == inc[ axis ] ) ;
            ASSERT( grid.GetNumPoints( axis ) >= 1 ) ;
            begin[ axis ] = grid.GetNumPoints( axis ) - 1 ;
            end[ axis ] = -1 ;
        }
    }
}




/** For a single block, compute remaining SDF, given a grid with some populated SDF values.
*/
void ComputeRemainingSDFFromImmediateSDF_Nearest_Block( UniformGrid< float > & signedDistanceGrid , const UniformGrid< int > & sdfPinnedGrid , const int voxelsPerBlock[ 3 ] , const int inc[ 3 ] , const Block & block )
{
    const int       numGridPoints[ 3 ]  = { signedDistanceGrid.GetNumPoints( 0 ) , signedDistanceGrid.GetNumPoints( 1 ) , signedDistanceGrid.GetNumPoints( 2 ) } ;
    const size_t    numX                = signedDistanceGrid.GetNumPoints( 0 ) ;
    const size_t    numXY               = numX * signedDistanceGrid.GetNumPoints( 1 ) ;
    int             idx[3]              ;

    int             gridBegin[3]        ;
    int             gridEnd[3]          ;
    ComputeGridBeginEnd( gridBegin , gridEnd , signedDistanceGrid , inc ) ;

    int             blockBegin[3]       ;
    int             blockEnd[3]         ; // Note use of != for end.

    // Extract begin, end indices from block.
    block.SetRanges( blockBegin , blockEnd , voxelsPerBlock , numGridPoints , inc ) ;

    DEBUG_ONLY( bool previousSdfHadValue = false ) ; // Diagnosing algorithm

    DEBUG_ONLY( DebugLock() ) ;
    DEBUG_ONLY( DebugPrintf( "ComputeRemainingSDFFromImmediateSDF_Nearest_Block: {%2i,%2i,%2i} - {%2i,%2i,%2i}\n", blockBegin[0] , blockBegin[1] , blockBegin[2] , blockEnd[0] , blockEnd[1] , blockEnd[2] ) ) ;
    DEBUG_ONLY( DebugRelease() ) ;

    for( idx[2] = blockBegin[2] ; idx[2] != blockEnd[2] ; idx[2] += inc[2] )
    for( idx[1] = blockBegin[1] ; idx[1] != blockEnd[1] ; idx[1] += inc[1] )
    for( idx[0] = blockBegin[0] ; idx[0] != blockEnd[0] ; idx[0] += inc[0] )
    {   // For each grid point in block...

        // Obtain density values at this gridpoint and its BACKWARD neighbors.
        const size_t    offsetX0Y0Z0    = signedDistanceGrid.OffsetFromIndices( idx ) ;
        const size_t    offsetX1Y0Z0    = offsetX0Y0Z0 - inc[0] ;
        const size_t    offsetX0Y1Z0    = offsetX0Y0Z0 - inc[1] * numX ;
        const size_t    offsetX0Y0Z1    = offsetX0Y0Z0 - inc[2] * numXY ;

        float &         sdfHere         = signedDistanceGrid[ offsetX0Y0Z0 ] ;

        // Only access neighbors within the domain.
        const float     sdfX            = ( idx[0] != gridBegin[0] ) ? signedDistanceGrid[ offsetX1Y0Z0 ] : FLT_MAX ;
        const float     sdfY            = ( idx[1] != gridBegin[1] ) ? signedDistanceGrid[ offsetX0Y1Z0 ] : FLT_MAX ;
        const float     sdfZ            = ( idx[2] != gridBegin[2] ) ? signedDistanceGrid[ offsetX0Y0Z1 ] : FLT_MAX ;

        ASSERT( ! IsNan( sdfHere ) ) ;
        ASSERT( ! IsNan( sdfX    ) ) ;
        ASSERT( ! IsNan( sdfY    ) ) ;
        ASSERT( ! IsNan( sdfZ    ) ) ;

        float           aSdfX           = fabsf( sdfX ) ;
        float           aSdfY           = fabsf( sdfY ) ;
        float           aSdfZ           = fabsf( sdfZ ) ;

        const int &     sdfFrozen       = sdfPinnedGrid[ offsetX0Y0Z0 ] ;

        if( sdfFrozen )
        {   // Current SDF value is "frozen" meaning it cannot change.
            ASSERT( sdfHere != FLT_MAX ) ; // If it is frozen then it must have been assigned.
            DEBUG_ONLY( previousSdfHadValue = false ) ;
            continue ;
        }

        // Consider SDF of each neighbor.
        // Identify the neighbor with the smallest |SDF| (absolute value of SDF).
        // Assign current SDF to that of the minimal neighbor plus the commensurate grid spacing, with the appropriate sign.
        // NOTE: This is not the best estimate.  A better estimate would solve the Eikonal equation and take into account other neighbors and the gradient.
        if( aSdfX <= aSdfY )
        {   // Y is NOT the smallest neighbor.
            if( aSdfX <= aSdfZ )
            {   // X is the smallest neighbor.
                UpdateSdfFromNeighbor_Nearest( sdfHere , sdfX , signedDistanceGrid.GetCellSpacing().x ) ;
            }
            else
            {   // Z is the smallest neighbor.
                ASSERT( ( aSdfZ < aSdfX ) && ( aSdfZ < aSdfY ) ) ;
                UpdateSdfFromNeighbor_Nearest( sdfHere , sdfZ , signedDistanceGrid.GetCellSpacing().z ) ;
            }
        }
        else
        {   // X is NOT the smallest neighbor.
            ASSERT( aSdfY < aSdfX ) ;
            if( aSdfY <= aSdfZ )
            {   // Y is the smallest neighbor.
                UpdateSdfFromNeighbor_Nearest( sdfHere , sdfY , signedDistanceGrid.GetCellSpacing().y ) ;
            }
            else
            {   // Z is the smallest neighbor.
                ASSERT( ( aSdfZ < aSdfX ) && ( aSdfZ < aSdfY ) ) ;
                UpdateSdfFromNeighbor_Nearest( sdfHere , sdfZ , signedDistanceGrid.GetCellSpacing().z ) ;
            }
        }

        // If any neighbor has an assigned SDF then the current value must have been assigned.
        ASSERT( ! ( ( sdfX != FLT_MAX ) || ( sdfY != FLT_MAX ) || ( sdfZ != FLT_MAX ) ) || ( sdfHere != FLT_MAX ) ) ;

        ASSERT( ! previousSdfHadValue || ( sdfHere != FLT_MAX ) ) ;
        DEBUG_ONLY( previousSdfHadValue = ( ( sdfHere != FLT_MAX ) && ( idx[0]+inc[0] != blockEnd[0] ) ) ) ;
    }
}




void ComputeRemainingSDFFromImmediateSDF_Nearest_Serial( UniformGrid< float > & signedDistanceGrid , const UniformGrid< int > & sdfPinnedGrid )
{
    // When computing serially, there is only one block, and its size is the entire grid.
    const int voxelsPerBlock[3]    = { signedDistanceGrid.GetNumPoints(0) , signedDistanceGrid.GetNumPoints(1) , signedDistanceGrid.GetNumPoints(2) } ;

    static const int numPasses = 2 ;
    for( int pass = 0 ; pass < numPasses ; ++ pass )
    {   // For each pass...
        // Assign begin, end and increment for each axis.
        // Start at extremal corner.
        // Note that although this algorithm looks backwards from the direction of "travel",
        // it has to start at the first gridpoint even though it has no backward neighbors
        // *along that direction* because the other directions usually do have backward neighbors.
        // The backward-indexer below thus checks each index before accessing its backward neighbor.
        //
        // Default values assume forward-sweeping in each direction.
        int inc[3] = { 1,1,1 } ;

        if( ( pass & 1 ) != 0 )
        {   // Odd-numbered pass.  Sweep in reverse direction.
            inc[ 0 ] = -1 ;
            inc[ 1 ] = -1 ;
            inc[ 2 ] = -1 ;
        }

        // When computing serially, there is only one block, and its size is the entire grid.
        Block block( 0 , 0 , 0 ) ;

        ComputeRemainingSDFFromImmediateSDF_Nearest_Block( signedDistanceGrid , sdfPinnedGrid , voxelsPerBlock , inc , block ) ;
    }
}




void ComputeRemainingSDFFromImmediateSDF_Nearest( UniformGrid< float > & signedDistanceGrid , const UniformGrid< int > & sdfPinnedGrid )
{
    PERF_BLOCK( ComputeRemainingSDFFromImmediateSDF_Nearest ) ;

    ASSERT( signedDistanceGrid.ShapeMatches( sdfPinnedGrid ) ) ;
    ASSERT( signedDistanceGrid.Size() == sdfPinnedGrid.Size() ) ;

#if USE_TBB
    ComputeRemainingSDFFromImmediateSDF_Nearest_TBB::Driver( signedDistanceGrid , sdfPinnedGrid ) ;
#else
    ComputeRemainingSDFFromImmediateSDF_Nearest_Serial( signedDistanceGrid , sdfPinnedGrid ) ;
#endif

#define DIAGNOSE_FULL_SDF 1
#if DIAGNOSE_FULL_SDF
    {
        int         idx[3]      ;
        const int   begin[3]    = { 0,0,0 } ;
        const int   end[3]      = { signedDistanceGrid.GetNumPoints(0) , signedDistanceGrid.GetNumPoints(1) , signedDistanceGrid.GetNumPoints(2) } ;

        for( idx[2] = begin[2] ; idx[2] != end[2] ; ++ idx[2] )
        for( idx[1] = begin[1] ; idx[1] != end[1] ; ++ idx[1] )
        for( idx[0] = begin[0] ; idx[0] != end[0] ; ++ idx[0] )
        {   // For each grid cell...

            // Obtain SDF value at this gridpoint.
            const size_t    offsetX0Y0Z0  = signedDistanceGrid.OffsetFromIndices( idx ) ;
            if( FLT_MAX == signedDistanceGrid[ offsetX0Y0Z0 ] )
            {
                //DEBUG_BREAK() ;
                signedDistanceGrid[ offsetX0Y0Z0 ] = std::numeric_limits< float >::quiet_NaN() ;
            }
        }
    }
#endif
}




/** Populate signedDistanceGrid with an SDF for a perfect sphere centered on the domain.

    By eliminating a source of errors in the computation of the SDF from density,
    this is useful for diagnosing various algorithms that use SDF.

*/
void ComputeSDF_Sphere( UniformGrid< float > & signedDistanceGrid )
{
    PERF_BLOCK( ComputeSDF_Sphere ) ;

    ASSERT( ! signedDistanceGrid.HasZeroExtent() ) ;
    ASSERT( ! signedDistanceGrid.GetGridCapacity() ) ;

    const Vec3  center = signedDistanceGrid.GetCenter() ;
    //const float radius = 0.5f * MIN3( signedDistanceGrid.GetExtent().x , signedDistanceGrid.GetExtent().y , signedDistanceGrid.GetExtent().z ) - MIN3( signedDistanceGrid.GetCellSpacing().x , signedDistanceGrid.GetCellSpacing().y , signedDistanceGrid.GetCellSpacing().z ) ;
    const float radius = 0.25f * signedDistanceGrid.GetExtent().Magnitude() ;

    int       idx[3]      ;
    const int begin[3]    = { 0,0,0 } ;
    const int end[3]      = { signedDistanceGrid.GetNumPoints(0) , signedDistanceGrid.GetNumPoints(1) , signedDistanceGrid.GetNumPoints(2) } ; // Note use of != for end.
    const int inc         = 1 ;

    for( idx[2] = begin[2] ; idx[2] != end[2] ; idx[2] += inc )
    for( idx[1] = begin[1] ; idx[1] != end[1] ; idx[1] += inc )
    for( idx[0] = begin[0] ; idx[0] != end[0] ; idx[0] += inc )
    {   // For each grid point...

        // Obtain density values at this gridpoint and its BACKWARD neighbors.
        const size_t    offsetX0Y0Z0    = signedDistanceGrid.OffsetFromIndices( idx ) ;

        float & sdfHere             = signedDistanceGrid[ offsetX0Y0Z0 ] ;

        Vec3    pos = signedDistanceGrid.PositionFromIndices( idx[0] , idx[1] , idx[2] ) ;

        Vec3 displacement = pos - center ;
        sdfHere = displacement.Magnitude() - radius ;

        ASSERT( ! IsNan( sdfHere ) ) ;
    }
}




#if defined( _DEBUG )

/** Check to make sure particle contribution to each gridpoint has sane values.
*/
void CheckContribution( const UniformGrid< float > & contributionGrid )
{
    const unsigned contributionGridCapacity = contributionGrid.GetGridCapacity() ;
    for( unsigned offset = 0 ; offset < contributionGridCapacity ; ++ offset )
    {   // For each point in grid...
        const float & contribution = contributionGrid[ offset ] ;
        ASSERT( contribution >= 0.0f ) ;
    }
}

#endif




/** Populate a UniformGrid with signed distance values from surface tracers.

    \param signedDistanceGrid (out)    Grid into which to transfer signed distance values.

    \param particles    Dynamic array of particles whose signed distance information to
                        accumulate into signedDistanceGrid.
*/
void PopulateSignedDistanceGridFromSurfaceTracerParticles( UniformGrid< float > & signedDistanceGrid
                                                          , const UniformGridGeometry & referenceGridGeometry
                                                          , const VECTOR< Particle > & particles
                                                          , const float ambientDensity
                                                          , const unsigned uFrame )
{
    PERF_BLOCK( PopulateSignedDistanceGridFromSurfaceTracerParticles ) ;

    {
        Vec3 minCorner = Vec3( FLT_MAX , FLT_MAX , FLT_MAX ) ;
        Vec3 maxCorner = - minCorner ;
        PclOpFindBoundingBox::FindBoundingBox( particles , minCorner , maxCorner ) ;
        // TODO: Instead of computing bbox, use bounding box found earlier in PclGroup.
        //ASSERT( referenceGridGeometry.GetMinCorner() <= minCorner ) ;
        //ASSERT( referenceGridGeometry.GetMaxCorner() >= maxCorner ) ;

        signedDistanceGrid.Clear() ;                       // Clear any stale SDF information.
        // Instead of using grid template, use grid whose cell density depends on number of tracers (not number of vortons)
        // Restrict number of cells to no more than some factor compared to the reference grid.
        // Note that each grid is contrained to have a power-of-2 number of gridpoints along each direction,
        // so only power-of-2 cuberoots of expansionFactor directly translate to expanding the base grid by precisely root3(expansionFactor).
        const size_t expansionFactor = 16 ; // The SDF grid is encouraged to have approximately this many more cells than the reference grid.
        const size_t referenceGridCapacity = referenceGridGeometry.GetGridCapacity() ;
        const size_t gridCapacity = referenceGridCapacity ? Min2( particles.Size() , expansionFactor * referenceGridCapacity ) : particles.Size() ;
        signedDistanceGrid.DefineShape( gridCapacity , minCorner , maxCorner , true ) ;
        signedDistanceGrid.Init( 0.0f ) ; // Reserve memory for SDF grid and initialize all values to FLT_MAX.
    }

    UniformGrid< int > sdfPinnedGrid ;
    sdfPinnedGrid.CopyShape( signedDistanceGrid ) ; // Match shape of sdf grid.
    sdfPinnedGrid.Init( 0 ) ;                       // Initialize all values to false.

    // Amount of contribution to each grid point.
    UniformGrid< float > ugParticleContribution( static_cast< UniformGridGeometry >( signedDistanceGrid ) ) ;
    ugParticleContribution.Init( 0.0f ) ;

    // Populate signed distance grid.
    // First pass: sum signed distance values into each gridpoint, and keep track of contribution to each.
    const size_t numParticles = particles.Size() ;
    for( size_t uParticle = 0 ; uParticle < numParticles ; ++ uParticle )
    {   // For each particle in the array...
        const Particle  &   rParticle   = particles[ uParticle ] ;
        const Vec3      &   rPosition   = rParticle.mPosition   ;
        ASSERT( ! IsNan( rPosition ) && ! IsInf( rPosition ) ) ;

        DEBUG_ONLY( const unsigned      uOffset     = signedDistanceGrid.OffsetOfPosition( rPosition ) ) ;
        ASSERT( uOffset < signedDistanceGrid.GetGridCapacity() ) ;

        // Obtain signed distance from particle: Distance is radius (half mSize, which is diameter), sign comes from density.
        const float distanceSign = rParticle.mDensity < ambientDensity ? 0.5f : -0.5f ; // higher density means inside
        const float signedDistance = rParticle.mSize * distanceSign ;
        ASSERT( ! IsNan( signedDistance ) && ! IsInf( signedDistance ) ) ;

        signedDistanceGrid.Accumulate( rPosition , signedDistance ) ;

        // Tally particle contribution to each gridpoint.
        ugParticleContribution.Accumulate( rPosition , 1.0f ) ;

    #if 0 && defined( _DEBUG ) // This check is intolerably expensive and slow.  Enable it only to diagnose issues with Accumulate.
        CheckContribution( ugParticleContribution ) ;
    #endif
    }

    // Apply per-gridpoint corrections.
    const unsigned signedDistanceGridCapacity = signedDistanceGrid.GetGridCapacity() ;
    for( unsigned offset = 0 ; offset < signedDistanceGridCapacity ; ++ offset )
    {   // For each point in grid...
        // Turn sums into averages.
        const float & contribution = ugParticleContribution[ offset ] ;
        ASSERT( contribution >= 0.0f ) ;
        if( contribution > 0.0f )
        {   // SDF gridcell contained particles.
            const float oneOverContribution = 1.0f / contribution ;
            float & signedDistance = signedDistanceGrid[ offset ] ;
            ASSERT( signedDistance != FLT_MAX ) ;
            ASSERT( ! IsNan( signedDistance ) && ! IsInf( signedDistance ) ) ;
            signedDistance *= oneOverContribution ;
            sdfPinnedGrid[ offset ] = 1 ;
        }
        else
        {   // SDF gridcell had no particles.
            // Mark this gridpoint as not pinned (i.e. value will need to come from propagating pinned values).
            ASSERT( 0.0f == contribution ) ;
            sdfPinnedGrid[ offset ] = 0 ;

            // Reset SDF to special value meaning "unassigned".
            float & signedDistance = signedDistanceGrid[ offset ] ;
            ASSERT( 0.0f == signedDistance ) ;
            signedDistance = FLT_MAX ;
        }
    }

    ASSERT( ! signedDistanceGrid.HasZeroExtent() ) ;

    #if defined( _DEBUG ) || VORTON_SIM_SIGNED_DISTANCE
    static bool bOutputDiagnostics = false ;
    if( bOutputDiagnostics )
    {   // Output density for visualization.
        signedDistanceGrid.GenerateBrickOfBytes( "signedDistance-fromTracers" , uFrame ) ;
    }
    #else
        (void) uFrame ;
    #endif

    // Procedure above computes SDF on grid only where there surface tracer particles.
    // The rest of the grid has yet to be assigned SDF values.
    // Populate the rest of the domain with SDF values, propagated from those values already assigned (and pinned).
    ComputeRemainingSDFFromImmediateSDF_Nearest( signedDistanceGrid , sdfPinnedGrid ) ;
}




/** Place particles within a narrow band near a surface described implicitly by a signed distance field.

    \param particles    (out) Dynamic array of tracer particles to populate.

NOTE: These parameter comments are stale.  Do not trust them.

    \param densityDeviationGrid  Density deviation from ambient, used to indicate region within which to emit tracer particles.

    \param densityDeviationAtSurface Fluid density deviation at the interface surface between two fluids.

NOTE: These comments are stale.  Do not trust them.

    The idea:
        Use density from vortons to populate a densityDeviation grid.
        Use densityDeviation grid to populate seed tracer particles:
            For each gridpoint, compute densityScaled = densityDeviationFromGrid / densityDeviationScale.
                This routine assumes vortons have a non-zero density, hence the deviation will be non-zero.
                Furthermore, this routine assumes all vortons have the same density, and that vortons do not overlap much.
                So, in regions where there are no vortons, densityDeviation=0.
                And, in regions where there are plenty of vortons (little or no space unoccupied by vortons), densityDeviation~=max or densityDeviation~=min.
                So, only in regions well within (min,0) or (0,max) are near fluid interfaces.
                This is an indicator function.
                We want to ignore cells which are exactly zero and whose neighbors are all zeros.
                Likewise we want to ignore cells whose values are all below min*buffer or above max*buffer, where buffer is a small number in (0,1) (perhaps 0.1) -- experiment to find best choice), and whose neighbors all likewise satisfy the same criteria.
                Make a sweep to mark all such cells as "truncated", perhaps by setting their values to ZERO, -FLT_MAX, or FLT_MAX (respectively), or just use NaN or FLT_MAX everywhere.
                It might be useful/necessary to reassigning any zero-values to -FLT_EPSILON or FLT_EPSILON to differentiate between those and ignored ZERO values if ZERO==0.0f.
                Call the result of this the truncated indicator function.
                It will be useful to have a truncated signed distance function, where distance would be measured in grid-spacing units, i.e. it would indicate the distance in gridcells.
                For a given point (e.g. gridpoint) that would be its distance to the closest zero-crossing.
                Computing that exactly might be expensive (or it might not -- not sure yet).


        (After start-up, advect tracers as per usual.)

        Options:
            Could re-init each frame using vortons, advect tracers, render level set.  Then will remain locked to vorton density.
            Could re-seed outlier tracers but keep others as the level set.  Also remains locked to vortons.
            Could subsequently use that as the level set, even though it will depart from vorton density.

*/
void PclOpSeedSurfaceTracers::Emit( VECTOR< Particle > & particles
                                   , const int tracerCountMultiplier
                                   , const UniformGrid< float > & signedDistanceGrid
                                   , float regionNearSurface
                                   , const float ambientDensity )
{
    PERF_BLOCK( PclOpSeedSurfaceTracers__Emit ) ;

    if( signedDistanceGrid.Empty() )
    {   // Signed distance grid is empty; the surface is not identified.
        return ;
    }

    if( sBandWidthAutomatic == regionNearSurface )
    {
        regionNearSurface = 1.0f * signedDistanceGrid.GetCellSpacing().Magnitude() ;
    }

    particles.Clear() ;

    const Vec3      vSpacing        = signedDistanceGrid.GetCellSpacing() ;
    const unsigned  begin[3]        = { 0,0,0 } ;
    const unsigned  end[3]          = { signedDistanceGrid.GetNumCells(0) , signedDistanceGrid.GetNumCells(1) , signedDistanceGrid.GetNumCells(2) } ;
    unsigned        idx[3]          ;

    const unsigned  nt[3]           = { tracerCountMultiplier , tracerCountMultiplier , tracerCountMultiplier } ;

    // Shift each particle to center the distribution within each cell.
    const Vec3  vShift                  = 0.5f * vSpacing / float( tracerCountMultiplier ) ;

    // Jitter should distribute particles within their little subdomains,
    // and must not place particles outside grid, even by a tiny amount.
    const Vec3  jitterMagnitude         = 1.9999f * sOneMinusEpsilon * vShift ;

    Particle particlePrototype ;
    particlePrototype.mVelocity	        = Vec3( 0.0f , 0.0f , 0.0f ) ;
    particlePrototype.mOrientation	    = Vec3( 0.0f , 0.0f , 0.0f ) ;
    particlePrototype.mAngularVelocity	= Vec3( 0.0f , 0.0f , 0.0f ) ;
    particlePrototype.mDensity          = FLT_EPSILON ;
#if ENABLE_FIRE
    particlePrototype.mFuelFraction       = 0.0f ;
    particlePrototype.mFlameFraction      = 0.0f ;
    particlePrototype.mSmokeFraction      = 1.0f ;
#endif
    particlePrototype.mSize		        = 0.0f ; // Set below to signed distance from surface
    particlePrototype.mBirthTime        = 0 ;

    // Density values assigned to surface tracers outside and inside surfaces.
    //
    // This is used as a proxy for the sign of the distance, since elsewhere the particle system
    // does not like negative particle sizes.
    //
    // As an alternative to this, the particle system could change to accommodate signed particle
    // sizes, in which case tracer particle density value would not matter, e.g. could be set to
    // 0 or 1, and this routine would no longer need ambientDensity passed in.
    const float densityOutside = 0.5f * ambientDensity ;
    const float densityInside  = 2.0f * ambientDensity ;

    // Size of region containing outermost band of surface tracers:
    const float emergencyExteriorSdf = MAX3( vSpacing.x , vSpacing.y , vSpacing.z ) / float( tracerCountMultiplier ) ;

    for( idx[2] = begin[2] ; idx[2] < end[2] ; ++ idx[2] )
    for( idx[1] = begin[1] ; idx[1] < end[1] ; ++ idx[1] )
    for( idx[0] = begin[0] ; idx[0] < end[0] ; ++ idx[0] )
    {   // For each grid cell...
        Vec3 vPosMinCorner ;
        signedDistanceGrid.PositionFromIndices( vPosMinCorner , idx ) ;
        //const unsigned  offset    = densityDeviationGrid.OffsetFromIndices( idx ) ;
        //const float sdfGridpoint = signedDistanceGrid.Get( idx[0] , idx[1] , idx[2] ) ;

        //if( fabsf( sdfGridpoint ) < ( regionNearSurface + emergencyExteriorSdf ) )
        {   // distance from here to surface is within range such that we want to place tracers here...
            unsigned it[3] ;
            for( it[2] = 0 ; it[2] < nt[2] ; ++ it[2] )
            for( it[1] = 0 ; it[1] < nt[1] ; ++ it[1] )
            for( it[0] = 0 ; it[0] < nt[0] ; ++ it[0] )
            {   // For each subdomain within each grid cell...
                const Vec3 vDisplacement(   float( it[0] ) / float( nt[0] ) * vSpacing.x ,
                                            float( it[1] ) / float( nt[1] ) * vSpacing.y ,
                                            float( it[2] ) / float( nt[2] ) * vSpacing.z ) ;

                const Vec3 jitter = RandomSpread( jitterMagnitude ) ;
                particlePrototype.mPosition   = vPosMinCorner + vDisplacement + vShift /* + jitter */ ;
                // Assign tracer particle size based on signed distance to surface
                float sdfHere ;
                signedDistanceGrid.Interpolate( sdfHere , particlePrototype.mPosition ) ;
                if( fabsf( sdfHere ) < regionNearSurface )
                {
                    if(     sdfHere <= emergencyExteriorSdf
                        &&  (       (       ( 0 == it[0] && 0 == idx[0] )
                                        ||  ( 0 == it[1] && 0 == idx[1] )
                                        ||  ( 0 == it[2] && 0 == idx[2] )
                                    )
                                ||  (       ( nt[0] == it[0]+1 && end[0] == idx[0]+1 )
                                        ||  ( nt[1] == it[1]+1 && end[1] == idx[1]+1 )
                                        ||  ( nt[2] == it[2]+1 && end[2] == idx[2]+1 )
                                    )
                            )
                        )
                    {   // "interior" particle on outer boundary.
                        // Boundary particles must be exterior particles.
                        sdfHere = emergencyExteriorSdf ;
                    }
                    particlePrototype.mSize	= 2.0f * fabsf( sdfHere ) ; // Set size to signed distance from surface
//if( sdfHere < 0.0f )
//{
//particlePrototype.mSize = 0.0f ;
//}
                    particlePrototype.mDensity = ( sdfHere >= 0.0f ) ? densityOutside : densityInside ;
                    particles.PushBack( particlePrototype ) ;
                }
            }
        }
    }
}




/** Place particles within a narrow band near a surface described implicitly by a signed distance field.

    \param particles    (in/out) Dynamic array of tracer particles to reassign.

*/
void PclOpSeedSurfaceTracers::Replace( VECTOR< Particle > & particles
                                        , UniformGrid< float > & signedDistanceGrid
                                        , const UniformGridGeometry & referenceGridGeometry
                                        , float regionNearSurface
                                        , const float ambientDensity )
{
    PERF_BLOCK( PclOpSeedSurfaceTracers__Replace ) ;

    if( ( NULL == & signedDistanceGrid ) || signedDistanceGrid.Empty() )
    {   // Signed distance grid is empty; the surface is not identified.
        return ;
    }

    // Populate SDF grid from surface tracers.
    PopulateSignedDistanceGridFromSurfaceTracerParticles( signedDistanceGrid , referenceGridGeometry , particles , ambientDensity , 0 ) ;

    if( sBandWidthAutomatic == regionNearSurface )
    {
        regionNearSurface = 3.0f * signedDistanceGrid.GetCellSpacing().Magnitude() ;
    }

#if 1 // temporarily disable tracer re-seeding to diagnose PopulateSignedDistanceGridFromSurfaceTracerParticles

    // Create spatial partition.  Populate with particles.
    UniformGrid< VECTOR< unsigned > > particlePartition ;
    particlePartition.CopyShape( signedDistanceGrid ) ;
    Particles::PartitionParticles( particles , particlePartition , 0.0f ) ;

    const Vec3      vSpacing            = signedDistanceGrid.GetCellSpacing() ;
    const float     gridCellDiagonal    = vSpacing.Magnitude() ;
    const unsigned  begin[3]            = { 0,0,0 } ;
    const unsigned  end[3]              = { signedDistanceGrid.GetNumCells(0) , signedDistanceGrid.GetNumCells(1) , signedDistanceGrid.GetNumCells(2) } ;
    unsigned        idx[3]              ;

    // Shift each particle to center the distribution within each cell.
    const Vec3      vShift              = 0.5f * vSpacing ;

    // Jitter should distribute particles within their little subdomains,
    // and must not place particles outside grid, even by a tiny amount.
    const Vec3  jitterMagnitude         = 1.9999f * sOneMinusEpsilon * vShift ;

    Particle particlePrototype ;
    particlePrototype.mVelocity	        = Vec3( 0.0f , 0.0f , 0.0f ) ;
    particlePrototype.mOrientation	    = Vec3( 0.0f , 0.0f , 0.0f ) ;
    particlePrototype.mAngularVelocity	= Vec3( 0.0f , 0.0f , 0.0f ) ;
    particlePrototype.mDensity          = FLT_EPSILON ;
#if ENABLE_FIRE
    particlePrototype.mFuelFraction     = 0.0f ;
    particlePrototype.mFlameFraction    = 0.0f ;
    particlePrototype.mSmokeFraction    = 1.0f ;
#endif
    particlePrototype.mSize		        = 0.0f ; // Set below to signed distance from surface
    particlePrototype.mBirthTime        = 0 ;

    const float densityOutside = 0.5f * ambientDensity ;
    const float densityInside  = 2.0f * ambientDensity ;

    const float emergencyExteriorSdf = MAX3( vSpacing.x , vSpacing.y , vSpacing.z ) ;

    static const int maxNumPclsPerCell = 2 ;

    for( idx[2] = begin[2] ; idx[2] < end[2] ; ++ idx[2] )
    for( idx[1] = begin[1] ; idx[1] < end[1] ; ++ idx[1] )
    for( idx[0] = begin[0] ; idx[0] < end[0] ; ++ idx[0] )
    {   // For each grid cell...

        const VECTOR< unsigned > & indicesOfParticlesInCell = particlePartition[ idx ] ;

        if( indicesOfParticlesInCell.Empty() )
        {   // Cell has no particles.

            Vec3 vPosMinCorner ;
            signedDistanceGrid.PositionFromIndices( vPosMinCorner , idx ) ;
            //const size_t  offset    = signedDistanceGrid.OffsetFromIndices( idx ) ;
            const float sdfGridpoint = signedDistanceGrid.Get( idx[0] , idx[1] , idx[2] ) ;

            if( fabsf( sdfGridpoint ) < ( regionNearSurface + gridCellDiagonal + emergencyExteriorSdf ) )
            {   // distance from here to surface is within range such that we want to place tracer here...

                // Emit particle

                particlePrototype.mPosition   = vPosMinCorner + vShift ;

                // Interpolate SDF at particle position to get SDF value from grid.
                float sdfFromGridAtPclCandidatePosition ;
                signedDistanceGrid.Interpolate( sdfFromGridAtPclCandidatePosition , particlePrototype.mPosition ) ;

                // Assign tracer particle size based on signed distance to surface
                if( fabsf( sdfFromGridAtPclCandidatePosition ) < regionNearSurface )
                {
                    if(     sdfFromGridAtPclCandidatePosition <= emergencyExteriorSdf
                        &&  (       (       ( 0 == idx[0] )
                                        ||  ( 0 == idx[1] )
                                        ||  ( 0 == idx[2] )
                                    )
                                ||  (       ( end[0] == idx[0]+1 )
                                        ||  ( end[1] == idx[1]+1 )
                                        ||  ( end[2] == idx[2]+1 )
                                    )
                            )
                        )
                    {   // "interior" particle on outer boundary.
                        // Boundary particles must be exterior particles.
                        sdfFromGridAtPclCandidatePosition = emergencyExteriorSdf ;
                    }
                    particlePrototype.mSize	= 2.0f * fabsf( sdfFromGridAtPclCandidatePosition ) ; // Set size to signed distance from surface
//if( sdfHere < 0.0f )
//{
//particlePrototype.mSize = 0.0f ;
//}
                    particlePrototype.mDensity = ( sdfFromGridAtPclCandidatePosition >= 0.0f ) ? densityOutside : densityInside ;
                    particles.PushBack( particlePrototype ) ;
                }
            }

        }
        else
        {
            int numPclsSurvivingInThisCell = 0 ;
            for( unsigned pclOffset = 0 ; pclOffset < indicesOfParticlesInCell.Size() ; ++ pclOffset )
            {   // For each particle in current grid cell...
                const unsigned & pclIdx = indicesOfParticlesInCell[ pclOffset ] ;
                const Particle & rPcl = particles[ pclIdx ] ;

                // Interpolate SDF at particle position to get SDF value from grid.
                float sdfFromGridHere ;
                signedDistanceGrid.Interpolate( sdfFromGridHere , rPcl.mPosition ) ;

                if( fabsf( sdfFromGridHere ) > regionNearSurface )
                {   // Grid SDF at particle is too large.
                    // Kill particle.
                    Particles::MarkForKill( particles , pclIdx ) ;
                }
                else if( ( pclOffset > 0 ) && ( numPclsSurvivingInThisCell > maxNumPclsPerCell ) )
                {   // This cell has too many particles.
                    Particles::MarkForKill( particles , pclIdx ) ;
                }
                else
                {
                    //const float sdfSignFromPcl = rPcl.mDensity < ambientDensity ? 0.5f : -0.5f ; // higher density means inside
                    //float sdfFromPcl = sdfSignFromPcl * rPcl.mSize ;
                    //If particle SDF departs too far from grid SDF (e.g. if they have opposite sign and if their magnitude differs by a lot)
                    //    reassign particle values (radius, density).
                    ++ numPclsSurvivingInThisCell ;
                }
            }
        }
    }

    // Kill particles marked above.
    Particles::KillParticlesMarkedForDeath( particles ) ;

#endif
}




void PclOpSeedSurfaceTracers::Operate(  VECTOR< Particle > & particles , float timeStep , unsigned uFrame )
{
    (void) timeStep , uFrame ; 

    PERF_BLOCK( PclOpSeedSurfaceTracers__Operate ) ;

#if 0
    PclOpSeedSurfaceTracers::Emit( particles , 5 , * mSignedDistanceGrid , mBandWidth , mAmbientDensity ) ;
#else
    PclOpSeedSurfaceTracers::Replace( particles
                                   , const_cast< UniformGrid< float > & >( * mSignedDistanceGrid )
                                   , * mReferenceGrid
                                   , mBandWidth
                                   , mAmbientDensity ) ;
#endif
}
