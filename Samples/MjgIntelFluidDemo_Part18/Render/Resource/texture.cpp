/** \file texture.cpp

    \brief Texture base class.

    \author Copyright 2010-2014 MJG; All rights reserved.
*/

#include "Render/Resource/texture.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/Memory/newWrapper.h>
#include <Core/File/debugPrint.h>

// Types -----------------------------------------------------------------------
// Private variables -----------------------------------------------------------
// Public variables ------------------------------------------------------------
// Private functions -----------------------------------------------------------
// Public functions ------------------------------------------------------------

namespace PeGaSys {
    namespace Render {

        /** Construct texture base members.
        */
        TextureBase::TextureBase()
            : mWidth( 0 )
            , mHeight( 0 )
            , mNumPlanes( 0 )
            , mNumMipLevels( 0 )
            , mFormat( TEX_FORMAT_UNDEFINED )
            , mUsageFlags( TEX_USAGE_DEFAULT )
            , mShape( TEX_SHAPE_UNDEFINED )
        {
            PERF_BLOCK( TextureBase__TextureBase ) ;
        }




        /** Construct texture base members.
        */
        TextureBase::TextureBase( int width , int height , int numPlanes , int numMipLevels , FormatE format , UsageFlagsE usageFlags , ShapeE shape )
            : mWidth( 0 )
            , mHeight( 0 )
            , mNumPlanes( 0 )
            , mNumMipLevels( 0 )
            , mFormat( TEX_FORMAT_UNDEFINED )
            , mUsageFlags( TEX_USAGE_DEFAULT )
            , mShape( TEX_SHAPE_UNDEFINED )
        {
            PERF_BLOCK( TextureBase__TextureBase ) ;

            SetWidth( width ) ;
            SetHeight( height ) ;
            SetNumPlanes( numPlanes ) ;
            SetNumMipLevels( numMipLevels ) ;
            SetFormat( format ) ;
            SetUsageFlags( usageFlags ) ;
            SetShape( shape ) ;
        }




        /** Destruct texture base.
        */
        TextureBase::~TextureBase()
        {
            PERF_BLOCK( TextureBase__dtor ) ;
        }





        /** Release reference to this object and delete it if that was the last reference.
        */
        void TextureBase::ReleaseReference()
        {
            PERF_BLOCK( TextureBase__ReleaseReference ) ;

            if( _ReleaseReference() )
            {   // That was the last reference to this object.
                delete this ;
            }
        }




        void TextureBase::SetWidth( int width )
        {
            PERF_BLOCK( TextureBase__SetWidth ) ;

            ASSERT( 0 == mWidth ) ; // Not allowed to change after assigned
            ASSERT( width > 0 ) ;
            mWidth = width ;
        }




        void TextureBase::SetHeight( int height )
        {
            PERF_BLOCK( TextureBase__SetHeight ) ;

            ASSERT( 0 == mHeight ) ; // Not allowed to change after assigned
            ASSERT( height > 0 ) ;
            ASSERT( ( TEX_SHAPE_1D != GetShape() ) || ( 1 == height ) ) ; // If 1D texture, height must be 1.
            mHeight = height ;
        }




        void TextureBase::SetNumPlanes( int numPlanes )
        {
            PERF_BLOCK( TextureBase__SetNumPlanes ) ;

            UNUSED_PARAM( numPlanes ) ; // Avoid "unreferenced formal parameter" in non-DEBUG builds.
            ASSERT( 0 == mNumPlanes ) ; // Not allowed to change after assigned
            ASSERT( numPlanes > 0 ) ;
            ASSERT(     ( 1 == numPlanes )
                ||  ( TEX_SHAPE_UNDEFINED == GetShape() )
                ||  ( TEX_SHAPE_ARRAY     == GetShape() )
                ||  ( TEX_SHAPE_3D        == GetShape() ) ) ;
        }




        void TextureBase::SetNumMipLevels( int numMipLevels )
        {
            PERF_BLOCK( TextureBase__SetNumMipLevels ) ;

            ASSERT( 0 == mNumMipLevels ) ; // Not allowed to change after assigned
            ASSERT( numMipLevels > 0 ) ;
            mNumMipLevels = numMipLevels ;
        }




        void TextureBase::SetFormat( FormatE format )
        {
            PERF_BLOCK( TextureBase__SetFormat ) ;

            ASSERT( TEX_FORMAT_UNDEFINED == mFormat ) ; // Not allowed to change after assigned.
            ASSERT( format != TEX_FORMAT_UNDEFINED ) ;
            mFormat = format ;
        }




        void TextureBase::SetUsageFlags( UsageFlagsE usageFlags )
        {
            PERF_BLOCK( TextureBase__SetUsageFlags ) ;

            ASSERT( 0 == mUsageFlags ) ; // Not allowed to change after assigned.
            mUsageFlags = usageFlags ;
        }




        void TextureBase::SetShape( ShapeE shape )
        {
            PERF_BLOCK( TextureBase__SetShape ) ;

            NON_DEBUG_ONLY( UNUSED_PARAM( shape ) ) ; // Avoid "unreferenced formal parameter" in non-DEBUG builds.

            ASSERT( TEX_SHAPE_UNDEFINED == mShape ) ; // Not allowed to change after assigned.
            ASSERT( shape != TEX_SHAPE_UNDEFINED ) ;
            ASSERT( ( shape != TEX_SHAPE_1D      ) || ( ( 0 == GetHeight()    ) || ( 1 == GetHeight()    ) ) )      ; // If 1D texture, either height must be unassigned or 1.
            ASSERT( ( shape != TEX_SHAPE_2D      ) || ( ( 0 == GetNumPlanes() ) || ( 1 == GetNumPlanes() ) ) )      ; // If 2D texture, either depth must be unassigned or 1.
            ASSERT( ( shape != TEX_SHAPE_CUBEMAP ) || ( ( 0 == GetNumPlanes() ) || ( 1 == GetNumPlanes() ) ) )      ; // If cubemap texture, either depth must be unassigned or 1.
            ASSERT( ( shape != TEX_SHAPE_CUBEMAP ) || ( ( 0 == GetWidth() )     || ( GetWidth() == GetHeight() ) ) ) ; // If cubemap texture, width must equal height (square faces).
        }


    } ;
} ;




#if defined( _DEBUG )

#include "Image/image.h"
#include "Render/Device/api.h"

class TestTexture : public PeGaSys::Render::TextureBase
{
public:
    TestTexture()
    {
    }

    ~TestTexture() {}

    virtual void Bind( PeGaSys::Render::ApiBase * /*renderApi*/ , const PeGaSys::Render::SamplerStateS & /*samplerState*/ )
    {
    }

    virtual void CreateFromImages( const PeGaSys::Image * /*images*/ , size_t /*numImages*/ )
    {
    }

    virtual void CopyToImage( PeGaSys::Image & /*image*/ )
    {
    }

private:
} ;


void PeGaSys::Render::TextureBase::UnitTest()
{
    DebugPrintf( "Texture::UnitTest ----------------------------------------------\n" ) ;

    {
        TestTexture texture ;
    }

    DebugPrintf( "Texture::UnitTest: THE END ----------------------------------------------\n" ) ;
}

#endif
