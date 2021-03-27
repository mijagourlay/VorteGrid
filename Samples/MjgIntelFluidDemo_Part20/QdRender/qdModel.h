#if 0
/** \file qdModel.h

    \brief Model to render.

    \see Accompanying articles for more information:
        - http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

        - http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        - http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        - http://www.mijagourlay.com/

    \author Copyright 2009-2014 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.
*/
#ifndef QD_MODEL_H
#define QD_MODEL_H

#include "Core/Containers/vector.h"

#include "Core/Math/vec3.h"
#include "Core/Math/mat33.h"

#if defined( WIN32 )
    #include <windows.h>
#endif

#include <GL/gl.h>

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
        void Render( VECTOR< QdLight > * lights , float timeNow ) ;

    private:
        QdMaterial  *   mMaterial       ;   ///< Material used to render this model.
        Vec3            mScale          ;   ///< Scale to apply to model.
        Mat33           mRotation       ;   ///< Model orientation.
        Vec3            mTranslation    ;   ///< Model position.
        GLuint          mDisplayList    ;   ///< Identifier for model data.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

extern GLuint MakeSphere( float radius ) ;
extern GLuint MakeBox( const Vec3 & dimensions , float boundingSphereRadius ) ;

#endif

#endif
