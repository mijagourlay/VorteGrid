/** \file particleMaterial.h

    \brief Material for rendering particles.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-8/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-9/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-10/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-11/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-12/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-13/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-14/
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-15/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_MATERIAL_H
#define PARTICLE_MATERIAL_H

#include "qdMaterial.h"

/** Material for rendering particles.
*/
class ParticleMaterial : public QdMaterial
{
    public:
        ParticleMaterial()
            : mUniformScale( 1.0f )
            , mDensityVisibility( FLT_MAX )
        {}

        ~ParticleMaterial() {}

        const float &   GetScale() const                                { return mUniformScale ; }
        void            SetScale( float scale )                         { mUniformScale = scale ; }

        const float &   GetDensityVisibility() const                    { return mDensityVisibility ; }
        void            SetDensityVisibility( float densityVisibility ) { mDensityVisibility = densityVisibility ; }

    private:
        ParticleMaterial( const QdMaterial & re) ;                ///< Disallow copy construction.
        ParticleMaterial & operator=( const QdMaterial & re ) ;   ///< Disallow assignment.

        float   mUniformScale       ;   ///< Uniform scale to apply to particle size.
        float   mDensityVisibility  ;   ///< Factor to apply to density to determine opacity or brightness.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
