/*! \file qdMaterial.h

    \brief Class to set a rendering material

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-2/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-3/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-4/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-5/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-6/
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-7/

        http://www.gamasutra.com/view/feature/4164/sponsored_feature_fluid_.php
        http://www.gamasutra.com/view/feature/4176/sponsored_feature_fluid_.php

        http://www.mijagourlay.com/

    \author Copyright 2009-2010 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef QD_MATERIAL_H
#define QD_MATERIAL_H

/*! \brief Class to set a rendering material
*/
class QdMaterial
{
    public:

        enum TextureProc
        {
            TP_NOISE_OPAQUE ,   ///< Noise
            TP_NOISE_BALL   ,   ///< Noisey color with a gradual opacity falloff
        } ;

        QdMaterial( void ) ;
        ~QdMaterial() ;

        void Initialize( TextureProc textureProc , int texWidth , int texHeight , float gamma ) ;

        void SetMaterial( void ) ;

        Vec4    mColor      ;
        bool    mDepthWrite ;
        int     mCullFace   ;

    private:
        static const int NUM_TEXTURES_MAX = 8 ;

        typedef GLuint TextureT ;

        TextureT    mTextures[ NUM_TEXTURES_MAX ]    ;   ///< opaque texture data
        GLuint      mTexName ;

        QdMaterial( const QdMaterial & re) ;                // Disallow copy construction.
        QdMaterial & operator=( const QdMaterial & re ) ;   // Disallow assignment.
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
