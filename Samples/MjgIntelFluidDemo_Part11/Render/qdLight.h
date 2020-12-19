/*! \file qdLight.h

    \brief Class to set a light for rendering

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2011 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef QD_LIGHT_H
#define QD_LIGHT_H

#include "Core/Math/vec3.h"

/*! \brief Class to set a light for rendering
*/
class QdLight
{
    public:
        enum LightType
        {
            LT_POINT        ,
            LT_SPOT         ,
            LT_DIRECTIONAL  ,
            LT_NUM
        } ;

        QdLight( void ) ;
        ~QdLight() ;

        void        SetLight( int iLight , float timeNow ) ;

        static void DisableLights( void ) ;

        static const size_t MAX_NUM_VARIATIONS = 4 ;

        Vec3        mPosition                           ;   ///< Position of light
        Vec3        mColor                              ;   ///< Color of light
        Vec3        mAttenuation                        ;   ///< Constant, linear and quadratic attenuation coefficients
        LightType   mType                               ;   ///< Type of light: point, spot or directional
        float       mAmplitudes[ MAX_NUM_VARIATIONS ]   ;   ///< Amplitudes of variation
        float       mFrequencies[ MAX_NUM_VARIATIONS ]  ;   ///< Frequencies of variation

    private:
        QdLight( const QdLight & re) ;                // Disallow copy construction.
        QdLight & operator=( const QdLight & re ) ;   // Disallow assignment.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
