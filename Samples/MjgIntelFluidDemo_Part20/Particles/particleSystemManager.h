/** \file particleSystemManager.h

    \brief Container for multiple particle systems.

    \see http://www.mijagourlay.com/

    \author Copyright 2012-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_SYSTEM_MANAGER_H
#define PARTICLE_SYSTEM_MANAGER_H

#include "Core/Math/vec3.h"

#include "Core/Containers/vector.h"

class ParticleSystem ;

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/** Container for multiple particle systems.

    A ParticleSystemManager contains multiple ParticleSystems.
*/
class ParticleSystemManager
{
    public:
        typedef VECTOR< ParticleSystem * >     PclSysContainer ;
        typedef PclSysContainer::Iterator      Iterator ;
        typedef PclSysContainer::ConstIterator ConstIterator ;

        ParticleSystemManager() {}
        ~ParticleSystemManager() ;
        ParticleSystemManager( const ParticleSystemManager & that ) ;
        ParticleSystemManager & operator=( const ParticleSystemManager & that ) ;

        /// Add given ParticleSystem to the end of the current list of them.
        void PushBack( ParticleSystem * pclSys )
        {
            mParticleSystems.PushBack( pclSys ) ;
        }

        Iterator Begin() { return mParticleSystems.Begin() ; }
        Iterator End()   { return mParticleSystems.End()   ; }

        void Clear() ;
        void Update( float timeStep , unsigned uFrame ) ;

    private:
        PclSysContainer mParticleSystems ;    ///< Dynamic array of ParticleSystem objects that this manager owns.
} ;

// Public variables --------------------------------------------------------------

// Public functions --------------------------------------------------------------

#endif




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Move code below to its own file.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern ParticleSystem * CreateSampleParticleSystem( const Vec3 & basePosition ) ;
