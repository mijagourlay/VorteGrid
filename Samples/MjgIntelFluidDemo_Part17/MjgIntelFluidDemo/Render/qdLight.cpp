/*! \file qdLight.cpp

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

#include <math.h>

#if defined( WIN32 )
    #include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>

#include "Core/Math/vec4.h"

#include "Core/wrapperMacros.h"
#include "qdLight.h"




/*! \brief Construct object to set a light for rendering
*/
QdLight::QdLight( void )
    : mPosition( 0.0f , 0.0f , 1.0f )
    , mColor( 1.0f , 1.0f , 1.0f )
    , mAttenuation( 0.0f , 0.0f , 1.0f )
    , mType( LT_DIRECTIONAL )
{
    mAmplitudes[ 0 ] = 1.0f ;
    mAmplitudes[ 1 ] =
    mAmplitudes[ 2 ] =
    mAmplitudes[ 3 ] =
    mFrequencies[ 0 ] =
    mFrequencies[ 1 ] =
    mFrequencies[ 2 ] =
    mFrequencies[ 3 ] = 0.0f ;
}




/*! \brief Destruct object to set a light for rendering
*/
QdLight::~QdLight()
{
}




/*! \brief Set a light
*/
void QdLight::SetLight( int iLightIndex , float timeNow )
{
    glEnable( GL_LIGHTING ) ;
    // glDisable( GL_LIGHTING ) ;
    // glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE ) ; // Enable two-sided lighting

    // Code below here belongs in "QdLight"
    // Simplify (and speed up) specular computation
    glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE ) ;
    const int iLight = iLightIndex + GL_LIGHT0 ;
    glEnable( iLight ) ;

#if 0
    float globalAmbientLight[] = { 1.0f , 1.0f , 1.0f , 1.0f } ;
    glLightModelfv( GL_LIGHT_MODEL_AMBIENT , globalAmbientLight ) ;
#endif

    float modulation = 0.0f ;
    for( int iVariation = 0 ; iVariation < MAX_NUM_VARIATIONS ; ++ iVariation )
    {   // For each possible variation...
        static float freqShift = 1.0f ;
        freqShift = Clamp( freqShift + RandomSpread( 0.0001f ) , 0.8f , 1.1f ) ;
        const float phase = timeNow * ( mFrequencies[ iVariation ] * freqShift ) ;
        modulation += mAmplitudes[ iVariation ] * cosf( phase ) ;
    }
    const Vec3 colorNow( modulation * mColor ) ;

    Vec3 ambientColor( colorNow * 0.2f ) ;
    Vec3 diffuseColor( colorNow * 0.8f ) ;
    glLightfv( iLight , GL_AMBIENT , (float*) & ambientColor ) ;
    glLightfv( iLight , GL_DIFFUSE , (float*) & diffuseColor ) ;
    glLightf ( iLight , GL_CONSTANT_ATTENUATION  , mAttenuation.x ) ;
    glLightf ( iLight , GL_LINEAR_ATTENUATION    , mAttenuation.y ) ;
    glLightf ( iLight , GL_QUADRATIC_ATTENUATION , mAttenuation.z ) ;

    glPushMatrix() ;
    //glLoadIdentity() ;
    switch( mType )
    {
        case LT_POINT:
        {
            Vec4 position( mPosition , 1.0f ) ;
            glLightfv( iLight , GL_POSITION , (float*) & position ) ;
        }
        break ;

        case LT_DIRECTIONAL:
        {
            Vec4 position( mPosition , 0.0f ) ;
            glLightfv( iLight , GL_POSITION , (float*) & position ) ;
        }
        break ;

        default:
            ASSERT( 0 ) ;
        break ;
    }
    glPopMatrix() ;
}




/* static */ void QdLight::DisableLights( void )
{
    glDisable( GL_LIGHTING ) ;
    glDisable( GL_LIGHT0 ) ;
    glDisable( GL_LIGHT1 ) ;
    glDisable( GL_LIGHT2 ) ;
    glDisable( GL_LIGHT3 ) ;
    glDisable( GL_LIGHT4 ) ;
    glDisable( GL_LIGHT5 ) ;
    glDisable( GL_LIGHT6 ) ;
    glDisable( GL_LIGHT7 ) ;
}
