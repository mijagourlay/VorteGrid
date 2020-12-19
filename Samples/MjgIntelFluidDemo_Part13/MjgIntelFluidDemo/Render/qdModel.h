/*! \file qdModel.h

    \brief Model to render.

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

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2012 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef QD_MODEL_H
#define QD_MODEL_H

#include "Core/Math/vec3.h"
#include "Core/Math/mat33.h"

class QdMaterial ;
class QdLight ;

/** Model to render.
*/
class QdModel
{
    public:
        QdModel( GLuint displayList , QdMaterial * material ) ;
        ~QdModel() ;

        void SetPositionOrientation( const Vec3 & position , const Mat33 & orientation ) ;
        void SetScale( const Vec3 & scale ) ;
        void Render( Vector< QdLight > * lights , float timeNow ) ;

    private:
        QdMaterial  *   mMaterial       ;   ///< Material used to render this model.
        Vec3            mScale          ;   ///< Scale to apply to model.
        Mat33           mRotation       ;   ///< Model orientation.
        Vec3            mTranslation    ;   ///< Model position.
        GLuint          mDisplayList    ;   ///< Identifier for model data.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

extern GLuint MakeUnitSphere() ;
extern GLuint MakeUnitCube() ;

#endif
