/** \file particleSystemManager.cpp

    \brief Container for multiple particle systems.

    \see http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/

#include "particleSystem.h"
#include "particleSystemManager.h"

#include <Core/Performance/perfBlock.h>

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

ParticleSystemManager::~ParticleSystemManager()
{
    Clear() ;
}




ParticleSystemManager::ParticleSystemManager( const ParticleSystemManager & that )
{
    this->operator=( that ) ;
}




ParticleSystemManager & ParticleSystemManager::operator=( const ParticleSystemManager & that )
{
    if( this != & that )
    {   // Not self-copy.
        Clear() ;   // Delete all previous items in this object.
        for( ConstIterator pclSysIter = that.mParticleSystems.Begin() ; pclSysIter != that.mParticleSystems.End() ; ++ pclSysIter )
        {   // For each particle system in the original manager...
            ParticleSystem * pclSysOrig = * pclSysIter ;
            // Duplicate the original.
            ParticleSystem * pclSysDupe = new ParticleSystem( * pclSysOrig ) ;
            // Remember the duplicate.
            mParticleSystems.PushBack( pclSysDupe ) ;
        }
    }
    return * this ;
}




void ParticleSystemManager::Clear()
{
    while( ! mParticleSystems.Empty() )
    {   // For each particle system this manager owns...
        ParticleSystem * particleSystem = mParticleSystems.Back() ;
        particleSystem->Clear() ;
        mParticleSystems.PopBack() ;
    }
}




/** Process all particle systems this manager owns.
    \see ParticleSystem
*/
void ParticleSystemManager::Update( float timeStep , unsigned uFrame )
{
    PERF_BLOCK( ParticleSystemManager__Update ) ;

    for( Iterator pclSysIter = mParticleSystems.Begin() ; pclSysIter != mParticleSystems.End() ; ++ pclSysIter )
    {
        ParticleSystem * pclSys = * pclSysIter ;
        pclSys->Update( timeStep , uFrame ) ;
    }
}








ParticleSystem * CreateSampleParticleSystem( const Vec3 & basePosition )
{
    ParticleSystem  * particleSystem = new ParticleSystem ;

    {
        const Vec3 buoyantAcceleration( 0.0f , 0.0f , 1.0f ) ;
        ParticleGroup * pclGrpFlames = CreateSampleParticleGroup( basePosition , buoyantAcceleration ) ;
        particleSystem->PushBack( pclGrpFlames ) ;
    }
    {
        const Vec3 gravitationalAcceleration( 0.0f , 0.0f , -1.0f ) ;
        ParticleGroup * pclGrpDebris = CreateSampleParticleGroup( basePosition , gravitationalAcceleration ) ;
        particleSystem->PushBack( pclGrpDebris ) ;
    }

    return particleSystem ;
}
