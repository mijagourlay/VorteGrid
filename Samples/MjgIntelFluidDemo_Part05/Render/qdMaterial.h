/*! \file qdMaterial.h

    \brief Class to set a rendering material

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/
#ifndef QD_MATERIAL_H
#define QD_MATERIAL_H

/*! \brief Class to set a rendering material
*/
class QdMaterial
{
    public:
        QdMaterial( void ) ;
        ~QdMaterial() ;

        void Initialize( void ) ;

        void SetMaterial( void ) ;

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
