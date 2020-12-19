/** \file textureSampler.h

    \brief Sampler used to access a texture

    \author Written and Copyright 2010-2013 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_TEXTURE_SAMPLER_H
#define PEGASYS_RENDER_TEXTURE_SAMPLER_H

#include "Core/Containers/intrusivePtr.h"
#include "Core/Containers/vector.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        class TextureBase   ; // Forward declaration




        struct SamplerStateS
        {
            enum FilterE
            {
                FILTER_NO_MIPMAP,   ///< Disable MIPmap.
                FILTER_LINEAR   ,   ///< Linearly interpolate between samples.
                FILTER_NEAREST  ,   ///< Use nearest sample.
            } ;

            enum AddressE
            {
                ADDRESS_CLAMP   ,   /// Texture coordinates outside [0,1] set to 0 or 1.
                ADDRESS_REPEAT  ,   /// Texture coordinates outside [0,1] repeat.
            } ;

            enum CombineOperationE
            {
                COMBO_OP_REPLACE    ,   ///< Replace
                COMBO_OP_MODULATE   ,   ///< Multiply material colors. Default.
                //COMBO_OP_DECAL      ,   ///< Overwrite or blend color with texture.  Keep alpha as-is.
            } ;

            FilterE             mMinFilter          ; ///< Filter mode used to interpret texture coordinates not exactly centered on a texel. Default is FILTER_LINEAR.
            FilterE             mMagFilter          ; ///< Filter mode used to interpret texture coordinates not exactly centered on a texel. Default is FILTER_LINEAR.
            FilterE             mMipFilter          ; ///< Filter mode used to interpret texture coordinates not exactly centered on a texel. Default is FILTER_LINEAR.
            AddressE            mAddressU           ; ///< Address mode used to interpret texture coordinates outside [0,1]. Default is ADDRESS_CLAMP.
            AddressE            mAddressV           ; ///< Address mode used to interpret texture coordinates outside [0,1]. Default is ADDRESS_CLAMP.
            AddressE            mAddressW           ; ///< Address mode used to interpret texture coordinates outside [0,1]. Default is ADDRESS_CLAMP.
            CombineOperationE   mCombineOperation   ; ///< Operation used to combine multiple materials.  Default is OP_MODULATE.
        } ;




        class SamplerState : public SamplerStateS
        {
        public:
            SamplerState()
            {
                mMinFilter          = FILTER_LINEAR ;
                mMagFilter          = FILTER_LINEAR ;
                mMipFilter          = FILTER_LINEAR ;
                mAddressU           = ADDRESS_CLAMP ;
                mAddressV           = ADDRESS_CLAMP ;
                mAddressW           = ADDRESS_CLAMP ;
                mCombineOperation   = COMBO_OP_MODULATE ;
            }
        } ;




        /** Sampler to access a texture.
        */
        class TextureStage
        {
            public:
                typedef IntrusivePtr< TextureBase > TexturePtr ;

                TextureStage() ;
                ~TextureStage() ;

            #if defined( _DEBUG )
                static void UnitTest() ;
            #endif

                SamplerState    mSamplerState   ;   ///< Sampler settings.
                TexturePtr      mTexture        ;   ///< Address of texture that this sampler accesses.
        } ;

// Public variables ------------------------------------------------------------
// Public functions ------------------------------------------------------------

    } ;
} ;

#endif
