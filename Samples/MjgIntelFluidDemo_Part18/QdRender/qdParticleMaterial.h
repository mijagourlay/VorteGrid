/** \file qdParticleMaterial.h

    \brief Material for rendering particles.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef PARTICLE_MATERIAL_H
#define PARTICLE_MATERIAL_H

#include "qdMaterial.h"

/** Material for rendering particles.
*/
class QdParticleMaterial : public QdMaterial
{
    public:
        QdParticleMaterial()
            : mUniformScale( 1.0f )
            , mDensityVisibility( FLT_MAX )
        {}

        ~QdParticleMaterial() {}

        const float &   GetScale() const                                { return mUniformScale ; }
        void            SetScale( float scale )                         { mUniformScale = scale ; }

        const float &   GetDensityVisibility() const                    { return mDensityVisibility ; }
        void            SetDensityVisibility( float densityVisibility ) { mDensityVisibility = densityVisibility ; }

    private:
        QdParticleMaterial( const QdMaterial & re) ;                ///< Disallow copy construction.
        QdParticleMaterial & operator=( const QdMaterial & re ) ;   ///< Disallow assignment.

        float   mUniformScale       ;   ///< Uniform scale to apply to particle size.
        float   mDensityVisibility  ;   ///< Factor to apply to density to determine opacity or brightness.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
