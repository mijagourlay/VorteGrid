/* tga.h: Targa TrueVision image file handling routines
//
// Copyright (C) 1994-1999 by Michael J. Gourlay
//
// Provided as is.  No warrantees, express or implied.
*/
#ifndef _TGA_H__INCLUDED_
#define _TGA_H__INCLUDED_




#include <stdio.h>

#include "Core/RgbaImage.h"




/*! \brief Targe image file types
*/
enum TgaType
{
    TGA_Null        = 0     ,
    TGA_Map         = 1     ,   ///< Colormapped
    TGA_RGB         = 2     ,   ///< RGB
    TGA_Mono        = 3     ,   ///< Monochrome
    TGA_RLE         = 8     ,   ///< TGA_RLE is not an image type, but just a value that means "RLE" is used
    TGA_RLE_Map     = 9     ,   ///< Colormapped compressed
    TGA_RLE_RGB     = 10    ,   ///< RGB compressed
    TGA_RLE_Mono    = 11    ,   ///< Monochrome compressed
    TGA_CompMap     = 32    ,
    TGA_CompMap4    = 33
} ;




/* Interleave flag values */
#define TGA_IL_None 0
#define TGA_IL_Two  1
#define TGA_IL_Four 2




#define TARGA_MAGIC 'T' + 256 * 'G'




class ImageFileTga
{
    public:
        ImageFileTga( void ) ;
        int Read( RgbaImageT * pImg , FILE * fio ) ;
        int HeaderRead( RgbaImageT * pImg , FILE * fio ) ;
        int Write( const RgbaImageT * pImg , FILE * fio ) ;
        int HeaderWrite( const RgbaImageT * pImg , FILE * fio , TgaType eType , int bitsPerPixel ) ;

        unsigned char origin_bit;   ///< origin location: 0=lower 1=upper

    private:
        unsigned char id_len    ;
        unsigned char cmap_type ;
        unsigned char img_type  ;
        int           cmap_index;
        int           cmap_len  ;
        unsigned char cmap_size ;   ///< cmap entry size in bits
        int           x_off     ;
        int           y_off     ;
        unsigned char pixel_size;
        unsigned char att_bits  ;
        unsigned char reserved  ;
        unsigned char interleave;
        int           mapped    ;   ///< whether image is colormapped (This field does not appear in the TGA file.)
} ;





extern RgbaImageT tga_cmap;


extern int RgbaImageReadFromTga( RgbaImageT * pImg , const char * strFilename ) ;
extern int RgbaImageWriteToTga( const RgbaImageT * pImg , const char * strFilename , TgaType eType , int bitsPerPixel , bool bInvertVertically ) ;
extern void TgaTest( void ) ;

#endif /* _TGA_H__INCLUDED_ */
