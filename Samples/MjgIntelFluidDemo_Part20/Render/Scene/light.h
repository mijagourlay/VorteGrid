/** \file light.h

    \brief Scene node representing a light

    \author Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_LIGHT_H
#define PEGASYS_RENDER_LIGHT_H

#include "Render/Scene/sceneNodeBase.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {

        /** Scene node representing a light.
        */
        class Light : public SceneNodeBase
        {
            public:
                static const TypeId sTypeId = 'lite' ;  ///< Type identifier for light scene nodes

                /// Light varieties.
                enum LightTypeE
                {
                    AMBIENT     ,   ///< Ambient light; affects ambient materials.  No position or direction.
                    POINT       ,   ///< Point light; has a position and shines in all directions.
                    SPOT        ,   ///< Spotlight; has a position and a direction.
                    DIRECTIONAL ,   ///< Direction light; has no position (effectively infinitely far away) but has a direction.
                    NUM_LIGHT_TYPES ///< Number of light varieties
                } ;

                explicit Light( ISceneManager * sceneManager ) ;
                virtual ~Light() ;

                virtual void Render() ;


                /// Set type of light this is.
                void SetLightType( const LightTypeE & lightType )   { mLightType = lightType ; }

                /// Return type of light this is.
                const LightTypeE & GetLightType() const             { return mLightType ; }

                /// Set world-space direction directional/spot light shines, relative to parent transform.
                /// TODO: FIXME: Direction should correspond to the SceneNode's orientation, specifically to the x-axis, i.e. to a vector initially along the x-axis, transformed by the orientation matrix.  This should correspond to the 1st column vector in the orientation matrix.
                void SetDirection( const Vec3 & direction )
                {
                    reinterpret_cast< Vec3 & >( mDirection ) = direction.GetDir() ;
                    mDirection.w = 0.0f ;
                }

                /// Return world-space direction directional/spot light shines, relative to parent transform.
                /// TODO: FIXME: Direction should correspond to the SceneNode's orientation, specifically to the x-axis, i.e. to a vector initially along the x-axis, transformed by the orientation matrix.  This should correspond to the 1st column vector in the orientation matrix.
                const Vec4 & GetDir4() const                        { return mDirection ; }

                const Vec3 & GetDirection() const                   { return reinterpret_cast< const Vec3 & >( mDirection ) ; }

                /// Set light ambient color.
                void SetAmbientColor( const Vec4 & ambientColor ) { mAmbientColor = ambientColor ; }

                /// Return Color of contribution from this light to ambient light term.
                const Vec4 & GetAmbientColor() const                { return mAmbientColor ; }

                /// Set light diffuse color.
                void SetDiffuseColor( const Vec4 & diffuseColor )   { mDiffuseColor = diffuseColor ; }

                /// Return Color of contribution from this light to diffuse light term.
                const Vec4 & GetDiffuseColor() const                { return mDiffuseColor ; }

                /// Return Color of contribution from this light to specular light term.
                const Vec4 & GetSpecularColor() const               { return mSpecularColor ; }

                /// Return Range, in world units, of this light's influence.
                const float & GetRange() const                      { return mRange ; }

                /// Return Exponent of intensity falloff between inner and outer angle.
                const float & GetSpotFalloff() const                { return mSpotFalloff ; }

                /// Return Angle over which bright center of spotlight shines (like an inverse umbra).
                const float & GetSpotInnerAngle() const             { return mSpotInnerAngle ; }

                /// Return Total angle over which spotlight shines (like an inverse penumbra).
                const float & GetSpotOuterAngle() const             { return mSpotOuterAngle ; }

                /// Set intensity attenuation coefficients
                void SetAttenuation( float a0 , float a1 , float a2 )
                {
                    mConstAttenuation = a0 ;
                    mLinearAttenuation = a1 ;
                    mQuadraticAttenuation = a2 ;
                }

                /// Return contant term of intensity attenuation formula.
                const float & GetConstAttenuation() const           { return mConstAttenuation ; }

                /// Return coefficient of intensity attenuation term linearly proportional to distance.
                const float & GetLinearAttenuation() const          { return mLinearAttenuation ; }

                /// Return coefficient of intensity attenuation term proportional to distance squared.
                const float & GetQuadracticAttenuation() const      { return mQuadraticAttenuation ; }

            private:
                LightTypeE  mLightType              ;   ///< Type of light this is
                Vec4        mDirection              ;   ///< Direction light travels.  NOTE: That differs from OpenGL's directional light, which points toward the light source.
                Vec4        mAmbientColor           ;   ///< Color of contribution from this light to ambient light term
                Vec4        mDiffuseColor           ;   ///< Color of contribution from this light to diffuse light term
                Vec4        mSpecularColor          ;   ///< Color of contribution from this light to specular light term
                float       mRange                  ;   ///< Range, in world units, of this light's influence
                float       mSpotFalloff            ;   ///< Exponent of intensity falloff between inner and outer angle
                float       mSpotInnerAngle         ;   ///< Angle over which bright center of spotlight shines (like an inverse umbra)
                float       mSpotOuterAngle         ;   ///< Total angle over which spotlight shines (like an inverse penumbra)
                float       mConstAttenuation       ;   ///< Contant term of intensity attenuation formula
                float       mLinearAttenuation      ;   ///< Coefficient of intensity attenuation term linearly proportional to distance
                float       mQuadraticAttenuation   ;   ///< Coefficient of intensity attenuation term proportional to distance squared
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
