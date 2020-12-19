/** \file particle.cpp

    \brief Basic particle for use in a visual effects particle system.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include <stdlib.h>

#include "Core/Performance/perf.h"
#include "Core/SpatialPartition/uniformGridMath.h"
#include "particle.h"


static const float  sOneMinusEpsilon        = 1.0f - FLT_EPSILON ;
const float Particle::sNaN = fsqrtf( -1.0f ) ;




#if ENABLE_PARTICLE_HISTORY
static const size_t sPclHistoryMaxNumTimeSteps  =  100 ;    ///< Maximum number of time steps to record particle history.
static const size_t sPclHistoryMaxNumPcls       = 1300 ;    ///< Maximum number of particles whose history to record.
size_t              gPclHistoryFrame            = 0    ;    ///< Current index into history buffer.
bool                gPclHistoryFirstRun         = true ;    ///< Whether this is the first run of recording particle history.

struct PclEntry
{
    Particle mParticle ;
    bool mTouched ;
} ;

static PclEntry pclHistory[ sPclHistoryMaxNumTimeSteps ][ sPclHistoryMaxNumPcls ] ;

/// Record particle history for diagnosing determinism.
void PclHistoryRecord( size_t offset , const Particle & pcl )
{
    if( gPclHistoryFrame >= sPclHistoryMaxNumTimeSteps ) return ; // Out of space
    if( offset >= sPclHistoryMaxNumPcls ) return ; // Too many particles
    if( ! gPclHistoryFirstRun )
    {
        if( pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mPosition != pcl.mPosition )
        {
            int i ; i = 0 ; ++ i ;
            DEBUG_BREAK() ;
        }
        if( pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mAngularVelocity != pcl.mAngularVelocity  )
        {
            int i ; i = 0 ; ++ i ;
            DEBUG_BREAK() ;
        }
        if( pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mSize != pcl.mSize       )
        {
            int i ; i = 0 ; ++ i ;
            DEBUG_BREAK() ;
        }
        if( pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mVelocity != pcl.mVelocity       )
        {
            const Vec3 diff = pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mVelocity - pcl.mVelocity ;
//#pragma warning( disable: 4189 ) // local variable is initialized but not referenced
            const unsigned * beforeX = reinterpret_cast< const unsigned * >( & pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mVelocity.x ) ;
            const unsigned * beforeY = reinterpret_cast< const unsigned * >( & pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mVelocity.y ) ;
            const unsigned * beforeZ = reinterpret_cast< const unsigned * >( & pclHistory[ gPclHistoryFrame ][ offset ].mParticle.mVelocity.z ) ;
            const unsigned * afterX = reinterpret_cast< const unsigned * >( & pcl.mVelocity.x ) ;
            const unsigned * afterY = reinterpret_cast< const unsigned * >( & pcl.mVelocity.y ) ;
            const unsigned * afterZ = reinterpret_cast< const unsigned * >( & pcl.mVelocity.z ) ;
            int i ; i = 0 ; ++ i ;
            DEBUG_BREAK() ;
        }
    }
    else
    {   // On first run, make sure every element gets touched.
        ASSERT( ! pclHistory[ gPclHistoryFrame ][ offset ].mTouched ) ;
        pclHistory[ gPclHistoryFrame ][ offset ].mTouched = true ;
    }
    pclHistory[ gPclHistoryFrame ][ offset ].mParticle = pcl ;
}
#endif




/** Kill particles marked for death.

    \param particles    (in/out) Reference to particles to kill if they are marked for death.

    Some operations, such as "Merge", can effectively kill particles,
    but those operations can cause the particle array to become reordered.
    That is because Particles::Kill fills the slot previously occupied by
    the killed particle, with a particle at the end of the array, so
    that the array has no "holes".

    In order to make parallelizing this easier, some routines that want to
    kill particles instead simply "mark them for death" so that this routine
    can sweep through and reorder the particles array.

    \see MarkDead, IsAlive.
*/
void Particles::KillParticlesMarkedForDeath( VECTOR< Particle > & particles )
{
    QUERY_PERFORMANCE_ENTER ;

    size_t numParticles = particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; /* Loop body increments iPcl. */ )
    {
        const Particle & pcl = particles[ iPcl ] ;
        if( ! pcl.IsAlive() )
        {   // Particle was marked for death.
            Particles::Kill( particles , iPcl ) ;
            -- numParticles ;
        }
        else
        {   // Particle remains alive.
            ++ iPcl ;
        }
    }

    QUERY_PERFORMANCE_EXIT( VortonSim_KillParticlesMarkedForDeath ) ;
}




/** Compute the geometric center of all given particles.

    The "geometric center" is simply the average position of all particles,
    in contrast to the center of mass in which each particle position is weighted
    by its mass.
*/
Vec3 Particles::ComputeGeometricCenter( const VECTOR< Particle > & particles )
{
    Vec3 vCenter( 0.0f , 0.0f , 0.0f ) ;
    const size_t numTracers = particles.Size() ;
    for( size_t iTracer = 0 ; iTracer < numTracers ; ++ iTracer )
    {
        const Particle & pcl = particles[ iTracer ] ;
        vCenter += pcl.mPosition ;
    }
    vCenter /= float( numTracers ) ;
    return vCenter ;
}




/** Compute the total angular momentum of all given particles.

    \note   Used to diagnose the simulation.  Angular momentum should be
            conserved, that is, constant over time.
*/
Vec3 Particles::ComputeAngularMomentum( const VECTOR< Particle > & particles )
{
    Vec3 vAngMom( 0.0f , 0.0f , 0.0f ) ;
#if 1
    const size_t numParticles = particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {
        const Particle & pcl = particles[ iPcl ] ;
        const float mass    = pcl.GetMass() ;
        const float radius  = pcl.GetRadius() ;
        const float momentOfInertia = 0.4f * mass * radius * radius ;
        vAngMom += pcl.mAngularVelocity * momentOfInertia ;
    }
#endif
    return vAngMom ;
}




/** Compute velocity statistics for particles.

    \param particles    Array of particles.

    \param min      Minimum velocity of all particles.

    \param max      Maximum velocity of all particles.

    \param mean     Average velocity of all particles.

    \param stddev   Standard deviation of velocity of all particles.

    \param centerOfVelocity Center of velocity -- velocity-weighted sum of particle positions
*/
void Particles::ComputeVelocityStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev , Vec3 & centerOfVelocity )
{
    min = FLT_MAX ;
    max = - min ;
    centerOfVelocity = Vec3( 0.0f , 0.0f , 0.0f ) ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = particles.Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each particle...
        const Particle &    pcl     = particles[ iPcl ] ;
        const float         velMag  = pcl.mVelocity.Magnitude() ;
        sum  += velMag ;
        sum2 += velMag * velMag ;
        min = Min2( min , velMag ) ;
        max = Max2( max , velMag ) ;
        centerOfVelocity += velMag * pcl.mPosition ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
    centerOfVelocity /= sum ;
}




/** Compute vorticity statistics for all vortex particles.

    \param min      Minimum angular velocity of all particles.

    \param max      Maximum angular velocity of all particles.

    \param mean     Average angular velocity of all particles.

    \param stddev   Standard deviation of angular velocity of all particles.

    \param centerOfAngularVelocity   Center of angular velocity -- angVel-weighted sum of particle positions
*/
void Particles::ComputeAngularVelocityStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev , Vec3 & centerOfAngularVelocity )
{
    min = FLT_MAX ;
    max = - min ;
    centerOfAngularVelocity = Vec3( 0.0f , 0.0f , 0.0f ) ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = particles.Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each particle...
        const Particle &    pcl         = particles[ iPcl ] ;
        const float         angVelMag   = pcl.mAngularVelocity.Magnitude() ;
        sum  += angVelMag ;
        sum2 += angVelMag * angVelMag ;
        min = Min2( min , angVelMag ) ;
        max = Max2( max , angVelMag ) ;
        centerOfAngularVelocity += angVelMag * pcl.mPosition ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
    centerOfAngularVelocity /= sum ;
}




/** Compute temperature statistics for all particles.

    \param min      Minimum temperature of all particles.

    \param max      Maximum temperature of all particles.

    \param mean     Average temperature of all particles.

    \param stddev   Standard deviation of temperature of all particles.
*/
void Particles::ComputeTemperatureStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev )
{
    min = FLT_MAX ;
    max = - min ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = particles.Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each particle...
        const Particle &    pcl         = particles[ iPcl ] ;
        const float         temperature = pcl.GetTemperature( 1.0f ) ;
        sum += temperature ;
        sum2 += temperature * temperature ;
        min = Min2( min , temperature ) ;
        max = Max2( max , temperature ) ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
}




/** Compute density statistics for all particles.

    \param min      Minimum density of all particles.

    \param max      Maximum density of all particles.

    \param mean     Average density of all particles.

    \param stddev   Standard deviation of density of all particles.
*/
void Particles::ComputeDensityStats( const VECTOR< Particle > & particles , float & min , float & max , float & mean , float & stddev )
{
    min = FLT_MAX ;
    max = - min ;
    float           sum     = 0.0f ;
    float           sum2    = 0.0f ;
    const size_t    numPcls = particles.Size() ;
    for( unsigned iPcl = 0 ; iPcl < numPcls ; ++ iPcl )
    {   // For each particle...
        const Particle &    pcl     = particles[ iPcl ] ;
        const float         density = pcl.GetDensity() ;
        sum += density ;
        sum2 += density * density ;
        min = Min2( min , density ) ;
        max = Max2( max , density ) ;
    }
    mean = sum / float( numPcls ) ;
    const float mean2 = sum2 / float( numPcls ) ;
    stddev = fsqrtf( mean2 - mean * mean ) ;
}




#if defined( _DEBUG )

void CheckSpatialPartition( const VECTOR< Particle > & itemArray , const UniformGrid< VECTOR< unsigned > > & itemGrid )
{
    FILE * fp = fopen( "sp.dat" , "w" ) ;

    static const float FAT_FACTOR = 64.0f * FLT_EPSILON ;

    size_t numInGrid = 0 ;
    for( unsigned iz = 0 ; iz < itemGrid.GetNumCells( 2 ) ; ++ iz )
    for( unsigned iy = 0 ; iy < itemGrid.GetNumCells( 1 ) ; ++ iy )
    for( unsigned ix = 0 ; ix < itemGrid.GetNumCells( 0 ) ; ++ ix )
    {   // For each grid cell...
        const VECTOR< unsigned > & cell = itemGrid.Get( ix , iy , iz ) ;
        const size_t numInCell = cell.Size() ;
        numInGrid += numInCell ;

        fprintf( fp , "[%i,%i,%i] = { " , ix , iy , iz ) ;
        for( size_t idxWithinCell = 0 ; idxWithinCell < numInCell ; ++ idxWithinCell )
        {
            size_t idxWithinArray = cell[ idxWithinCell ] ;
            const Vec3 & pos = itemArray[ idxWithinArray ].mPosition ;

#if 1   // Test particle partitioning using loose bounds.
            const Vec3 cellMinCorner        = itemGrid.PositionFromIndices( ix , iy , iz ) ;
            const Vec3 cellMinCornerBound   = Vec3(  cellMinCorner.x - fabsf( cellMinCorner.x ) * FAT_FACTOR
                                                   , cellMinCorner.y - fabsf( cellMinCorner.y ) * FAT_FACTOR
                                                   , cellMinCorner.z - fabsf( cellMinCorner.z ) * FAT_FACTOR ) ;
            const Vec3 cellMaxCorner        = cellMinCorner + itemGrid.GetCellSpacing() ;
            const Vec3 cellMaxCornerBound   = Vec3(  cellMaxCorner.x + fabsf( cellMaxCorner.x ) * FAT_FACTOR
                                                   , cellMaxCorner.y + fabsf( cellMaxCorner.y ) * FAT_FACTOR
                                                   , cellMaxCorner.z + fabsf( cellMaxCorner.z ) * FAT_FACTOR ) ;
            ASSERT( ( cellMinCornerBound <= pos ) && ( pos <= cellMaxCornerBound ) ) ;
#else   // Test particle partitioning using tight bounds.
            const Vec3 cellMinCorner = itemGrid.PositionFromIndices( ix , iy , iz ) ;
            const Vec3 cellMaxCorner = cellMinCorner + itemGrid.GetCellSpacing() ;
            ASSERT( ( cellMinCorner <= pos ) && ( pos <= cellMaxCorner ) ) ;
#endif

            unsigned indicesFromPos[ 4 ] ;
            itemGrid.IndicesOfPosition( indicesFromPos , pos ) ;
            ASSERT( ( ix == indicesFromPos[ 0 ] ) && ( iy == indicesFromPos[ 1 ] ) && ( iz == indicesFromPos[ 2 ] ) ) ;

            unsigned offset = itemGrid.OffsetOfPosition( pos ) ;
            unsigned indicesFromOffset[ 3 ] ;
            itemGrid.IndicesFromOffset( indicesFromOffset , offset ) ;
            ASSERT( ( ix == indicesFromOffset[ 0 ] ) && ( iy == indicesFromOffset[ 1 ] ) && ( iz == indicesFromOffset[ 2 ] ) ) ;

            fprintf( fp , " %i @ <%g,%g,%g> , " , idxWithinArray , pos.x , pos.y , pos.z ) ;
        }
        fprintf( fp , " }\n" ) ;
    }

    fclose( fp ) ;

    ASSERT( itemArray.Size() == numInGrid ) ;
}

#endif




/** Spatially partition particles.

    This routine partitions space into cells using the given grid, possibly modifying its geometry
    to satisfy the given minCellSpacing.  Each vorton gets assigned to the cell that contains it.

    \param particles Particles to partition.

    \param particleIndices (in/out) Uniform grid of particle indices.  Assign each cell
        to contain a vector of the indices of particles that reside inside that cell.
        The index values refer to elements in particles.
        This routine assumes the grid geometry has been defined, but this routine
        will clear any incoming contents.

*/
void Particles::PartitionParticles(
                                        const VECTOR< Particle > & particles
                                    ,
#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
                                        SpatialPartition & particleGrid
#else
                                        UniformGrid< VECTOR< unsigned > > & particleIndices
#endif
                                    , float minCellSpacing )
{
    QUERY_PERFORMANCE_ENTER ;

    const size_t numParticles = particles.Size() ;

#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
    particleGrid.Init( numVortons , mGridTemplate ) ;               // Initialize spatial partition.
#else
    // Do not call Clear here; that also erases geometry.
    particleIndices.FitShape( particleIndices , minCellSpacing ) ;  // Possibly modify grid to accommodate minCellSpacing
    particleIndices.Init() ;                                        // Reserve memory for grid and initialize all values to empty.
#endif

    for( unsigned offset = 0 ; offset < numParticles; ++ offset )
    {   // For each particle...
        const Particle &    rParticle = particles[ offset ] ;
        // Insert the particle's offset into the spatial partition.
#if USE_UNIFORM_GRID_LINKED_LIST_SPATIAL_PARTITION
        particleGrid.PushBack( offset , rVorton.mPosition ) ;
#else
        particleIndices[ rParticle.mPosition ].PushBack( offset ) ;
#endif
    }

#if defined( _DEBUG )
    CheckSpatialPartition( particles , particleIndices ) ;
#endif

    QUERY_PERFORMANCE_EXIT( PartitionParticles ) ;
}




#if ENABLE_PARTICLE_JERK_RECORD
/**
*/
extern void Particles::UpdateJerk( VECTOR< Particle > & particles , float timeStep )
{
    size_t numParticles = particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {
        Particle & pcl   = particles[ iPcl ] ;
        pcl.UpdateJerkDiagnostics( timeStep ) ;
    }
}




/**
*/
extern void Particles::ComputeJerkStatistics( const VECTOR< Particle > & particles , float & jerk2Avg , float & jerk2Dev , float & jerk2Min , float & jerk2Max )
{
    float jerk2Sum = 0.0f ;
    float jerk4Sum = 0.0f ;
    jerk2Min = FLT_MAX ;
    jerk2Max = - jerk2Min ;
    size_t numParticles = particles.Size() ;
    for( size_t iPcl = 0 ; iPcl < numParticles ; ++ iPcl )
    {
        const Particle & pcl   = particles[ iPcl ] ;
        const float      jerk2 = pcl.mJerk.Mag2() ;
        const float      jerk4 = POW2( jerk2 ) ;
        jerk2Sum += jerk2 ;
        jerk4Sum += jerk4 ;
        jerk2Min = Min2( jerk2Min , jerk2 ) ;
        jerk2Max = Max2( jerk2Max , jerk2 ) ;
    }
    jerk2Avg = jerk2Sum / float( numParticles ) ;
    const float jerk4Avg = jerk4Sum / float( numParticles ) ;
    jerk2Dev = sqrtf( jerk4Avg - POW2( jerk2Avg ) ) ;
}
#endif
