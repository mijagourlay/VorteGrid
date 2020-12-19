/*! \file tga.c

    \brief Targa TrueVision image file handling routines

    \author Written and Copyright (C) 1994-1999 by Michael J. Gourlay

*/

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "Core/Memory/my_malloc.h"
#include "Core/Utility/macros.h"

#include "tga.h"




/* Global Targa colormap */
static unsigned char tga_cmap_r[16384];
static unsigned char tga_cmap_g[16384];
static unsigned char tga_cmap_b[16384];
static unsigned char tga_cmap_a[16384];

#if 0

RgbaImageT tga_cmap =
{
    0,             
    0, 
    0,
    0,
    0,
    tga_cmap_r,
    tga_cmap_g,
    tga_cmap_b,
    tga_cmap_a,
};
#else
RgbaImageT tga_cmap ; // under construction
#endif



#define CURR_ROW(irow) ( origin_bit ? irow : ( pImg->GetNumRows() - (irow) - 1 ) )




#define INTERLACE_ROW_INC                                           \
{                                                                   \
    row_count++;                                                    \
    if(     this->interleave==TGA_IL_Four)  { ilace_row += 4 ; }    \
    else if(this->interleave==TGA_IL_Two )  { ilace_row += 2 ; }    \
    else                                    { ilace_row ++ ;   }    \
    if( ilace_row > pImg->GetNumRows() )   ilace_row = ++ top_row ; \
}




// When reading the image data, we might need to invert the image vertically.
// This macro facilitates that.
#define ROW_INC_READ                                                \
{                                                                   \
    INTERLACE_ROW_INC ;                                             \
    c_row = CURR_ROW( ilace_row ) ;                                 \
}




// Always write images from start to finish in memory.
// If the file format needs to be upside-down from what is in memory,
// then set the origin_bit in the TGA file header.
#define ROW_INC_WRITE                                               \
{                                                                   \
    INTERLACE_ROW_INC ;                                             \
    c_row = ilace_row ;                                             \
}




/*! \brief get from fio a byte
*/
#define GET_BYTE(byte, fio)                             \
{                                                       \
    int rv;                                             \
    (byte)=(unsigned char)(rv=getc(fio));               \
    if(rv==EOF) {                                       \
        fprintf(stderr, "get_byte: EOF/read error\n");  \
        return EOF ;                                    \
    }                                                   \
}




/*! \brief put into fio a 2-byte little-endian unsigned integer
*/
int put_le_word(short le_int, FILE *fio)
{
    unsigned char b1, b2;

    b1 = le_int & 0xff;

    b2 = (le_int >> 8) & 0xff;

    /* least significant byte comes first */
    if(putc(b1, fio)==EOF) return EOF ;

    /* most significant byte comes last */
    if(putc(b2, fio)==EOF) return EOF ;

    return 0 ;
}




/*! \brief get from fio a 2-byte little-endian unsigned integer
*/
long get_le_word(FILE * fio)
{
    unsigned char b1, b2;

    /* least significant byte comes first */
    GET_BYTE(b1, fio);

    /* most significant byte comes last */
    GET_BYTE(b2, fio);

    return (b1 + b2*256);
}




/*! \brief get from fio an block of n bytes, and store it in buf
    
    
    \return EOF if there is a read error, or 0 otherwise
    
    
    \note get_block is nothing but fread with error reporting.
            Calls to get_block should probably just be replaced
            with fread.
*/
short get_block(FILE * fio, unsigned char *buf, size_t n)
{
    size_t rv ;

    if( feof(fio) )
        return EOF ;
    rv = fread( buf , 1 , (size_t) n , fio ) ;
    if( rv != n )
    {
        if(rv)
        {
            fprintf(stderr, "get_block: EOF/read error reading byte %i/%li\n", rv, n);
        }
        return EOF ;
    }

    return 0 ;
}




/*! \brief read a Targa pixel from fio into pImg

    \param iOffset - offset into image buffer to store pixel

    \param npixels - the number of literal pixels to read.

    \param size - the size of the pixel in the file, in bits.

    \param mapped - tells whether bits are gray, coded RGB, or cmap index.

*/
static int tgaPixelRead( FILE * fio , RgbaImageT * pImg , int iOffset , int numPixels , int size , int mapped )
{
    const int endCount = iOffset + numPixels ;

    switch( size )
    {
        case 8:
            if( get_block( fio , & pImg->R(iOffset) , numPixels ) )
            {
                fprintf( stderr , "tgaPixelRead: read error\n" ) ;
                return -1 ;
            }
            if( mapped )
            {
                for( int iPixel = iOffset ; iPixel < endCount ; iPixel ++ )
                {
                    /* colormap indices are stored in ri, so do lookups with r last  */
                    pImg->B(iPixel) = tga_cmap.B( pImg->R(iPixel) ) ;
                    pImg->G(iPixel) = tga_cmap.G( pImg->R(iPixel) ) ;
                    pImg->R(iPixel) = tga_cmap.R( pImg->R(iPixel) ) ;
                }
            }
            else
            {   // Image is 8-bit grayscale so copy red channel into green and blue
                if( ! pImg->IsInterleaved() )
                {
                    memcpy( & pImg->G(iOffset) , & pImg->R(iOffset) , numPixels ) ;
                    memcpy( & pImg->B(iOffset) , & pImg->R(iOffset) , numPixels ) ;
                }
                else
                {
                    ASSERT( 0 ) ; // not implemented.  Would need to copy each pixel separately.
                }
            }
            if( ! pImg->IsInterleaved() )
            {
                memset( & pImg->A(iOffset) , RGBA_IMAGE_OPAQUE , numPixels ) ;
            }
            else
            {
                ASSERT( 0 ) ; // not implemented. Would need to assign each pixel separately.
            }
            break;

        case 16: case 15:
            {
                int ip, jp;
                int pixel;

                for( int iPixel = iOffset ; iPixel < endCount ; iPixel ++ )
                {
                    GET_BYTE(ip, fio);
                    GET_BYTE(jp, fio);
                    if(mapped)
                    {
                        pixel = ((unsigned int) jp << 8) + ip;
                        pImg->R( iPixel ) = tga_cmap.R( pixel );
                        pImg->G( iPixel ) = tga_cmap.G( pixel );
                        pImg->B( iPixel ) = tga_cmap.B( pixel );
                    }
                    else
                    {
                        /* Unpack color bits (5 each for red, green, blue */
                        pImg->R(iPixel) =  (jp & 0x7c) >> 2 ;
                        pImg->G(iPixel) = ((jp & 0x03) << 3) + ((ip & 0xe0) >> 5) ;
                        pImg->B(iPixel) =  ip & 0x1f ;
                    }
                    pImg->A( iPixel ) = RGBA_IMAGE_OPAQUE ;
                }
            }
            break;

        case 32: case 24:
            {
                for( int iPixel = iOffset ; iPixel < endCount ; iPixel ++ )
                {
                    GET_BYTE( pImg->B( iPixel ) , fio ) ;
                    GET_BYTE( pImg->G( iPixel ) , fio ) ;
                    GET_BYTE( pImg->R( iPixel ) , fio ) ;
                    if(size == 32)
                    {
                        GET_BYTE( pImg->A( iPixel ) , fio ) ;
                    }
                    else
                    {
                        pImg->A( iPixel ) = RGBA_IMAGE_OPAQUE ;
                    }
                }
            }
            break;

        default:
            fprintf(stderr, "tgaPixelRead: unknown pixel size %i\n", size);
            return -1 ;

            /*NOTREACHED*/
            break;

    }
    return 0 ;
}



ImageFileTga::ImageFileTga( void )
    : id_len( 0 )
    , cmap_type( 0 )
    , img_type( 0 )
    , cmap_index( 0 )
    , cmap_len( 0 )
    , cmap_size( 0 )
    , x_off( 0 )
    , y_off( 0 )
    , pixel_size( 0 )
    , att_bits( 0 )
    , reserved( 0 )
    , origin_bit( 0 )
    , interleave( 0 )
    , mapped( 0 )
{
}




/*! \brief Load a Targa image file from fio into pImg
*/
int ImageFileTga::Read( RgbaImageT * pImg , FILE * fio )
{
    int        col;
    int        rle_count;      /* run-length of data */
    int        c_row;          /* current row index being loaded */
    int        rl_encoded;     /* boolean flag */
    int        row_count;      /* total count of rows loaded */
    int        ilace_row;      /* interlaced row counter */
    int        top_row;        /* where to start over for interlaced images */

    if(     this->img_type == TGA_RLE_Map
        ||  this->img_type == TGA_RLE_RGB
        ||  this->img_type == TGA_RLE_Mono )
    {
        rl_encoded = 1 ;
    }
    else
    {
        rl_encoded = 0 ;
    }

    row_count   = ilace_row = top_row = 0   ;
    c_row       = CURR_ROW( ilace_row )     ;

    if( rl_encoded )
    {
        int           numPixels;
        int           rle_compressed; /* whether run is encoded or literal */
        int           blir;           /* bytes left in row */
        unsigned char ibyte;

        while( row_count < pImg->GetNumRows() )
        {
            for( col=0 ; col < pImg->GetNumCols() ; )
            {
                const int iPixel = c_row * pImg->GetNumCols() + col ;

                /* Read in the RLE count */
                GET_BYTE( ibyte , fio ) ;
                if( ibyte & 0x80 )
                {   /* run-length encoded pixel */
                    rle_count = ibyte - 127;
                    rle_compressed = 1;
                    /* Read the repeated byte */
                    if( tgaPixelRead( fio , pImg , iPixel , 1 , this->pixel_size , this->mapped ) )
                    {
                        fprintf(stderr,"tgaRead: read error in rle row %i\n", c_row);
                        return EOF ;
                    }
                }
                else
                {   /* stream of unencoded pixels */
                    rle_count       = ibyte + 1 ;
                    rle_compressed  = 0         ;
                }

                blir = pImg->GetNumCols() - col;

                /* Put run data into image memory */
                while( rle_count )
                {
                    if( rle_count <= blir )
                    {   /* finish the RLE block */
                        numPixels = rle_count ;
                    }
                    else
                    {   /* finish the row */
                        numPixels = blir ;
                    }
                    if( row_count >= pImg->GetNumRows() )
                    {
                        fprintf(stderr, "tgaRead: overread image.\n");
                        col = pImg->GetNumCols() ;
                        break ;
                    }
                    if( rle_compressed )
                    {
                        if( ! pImg->IsInterleaved() )
                        {
                            memset( & pImg->R( iPixel ) , pImg->R( iPixel ) , numPixels ) ;
                            memset( & pImg->G( iPixel ) , pImg->G( iPixel ) , numPixels ) ;
                            memset( & pImg->B( iPixel ) , pImg->B( iPixel ) , numPixels ) ;
                            memset( & pImg->A( iPixel ) , pImg->A( iPixel ) , numPixels ) ;
                        }
                        else
                        {
                            const int endCount = iPixel + numPixels ;
                            for( int iPix = iPixel ; iPix < endCount ; ++ iPix )
                            {   // Duplicate the pixel just read
                                pImg->R( iPix ) = pImg->R( iPixel ) ;
                                pImg->G( iPix ) = pImg->G( iPixel ) ;
                                pImg->B( iPix ) = pImg->B( iPixel ) ;
                                pImg->A( iPix ) = pImg->A( iPixel ) ;
                            }
                        }
                    }
                    else
                    {
                        if( tgaPixelRead( fio , pImg , iPixel , numPixels , this->pixel_size , this->mapped ) )
                        {
                            fprintf(stderr, "tgaRead: read err 3 in row %i\n", c_row);
                            return EOF ;
                        }
                    }
                    if(rle_count <= blir)
                    {
                        /* just emptied the RLE block */
                        col += rle_count;
                        rle_count = 0;
                    }
                    else
                    {
                        /* just emptied a row */
                        rle_count -= blir;
                        col = 0;
                        blir = pImg->GetNumCols() ;
                        ROW_INC_READ ;
                    }
                }
            } /* for col */

            ROW_INC_READ ;
        } /* while row_count */

    }
    else
    {
        /* Not run-length encoded */
        /* load pixel data one row at a time */
        while( row_count < pImg->GetNumRows() )
        {
            const int iPixel = c_row * pImg->GetNumCols() ;
            if( tgaPixelRead( fio , pImg , iPixel , pImg->GetNumCols() , this->pixel_size , this->mapped ) )
            {
                fprintf(stderr, "tgaRead: read error in row %i\n", c_row);
                return EOF ;
            }
            ROW_INC_READ ;
        }
    }

    return 0 ;
}




/* \brief load a Targa image header from fio into this and pImg

    Since Targa files (aka tga files) have no magic number
    it is not a simple matter to determine whether a file is a valid
    Targa image.  Therefore, there are several consistency checks in
    this header reading routine to try to determine whether the file
    is a valid targa file.

    In the case that the file is not a Targa file, then you could
    lseek to the beginning of the file and try to read it as another
    type of image.

    Since there is no way to be certain of whether the error is because
    this is not a Targa at all, or if it is because the file is simply
    a corrupt or unsupported Targa, no error messages are reported
    by this routine.  Instead, a different value is returned for
    every different kind of reason why this routine rejected the
    header.  The caller routine is responsible for handling this return
    value appropriately.

    \return If this routine returns nonzero, then either the file is not a
            valid targa file, or we don't support this type.

*/
int ImageFileTga::HeaderRead( RgbaImageT * pImg , FILE * fio )
{
    unsigned char flags;

    GET_BYTE(this->id_len, fio);
    GET_BYTE(this->cmap_type, fio);
    GET_BYTE(this->img_type, fio);

    /* Verify that this is among the supported Targa types */
    switch(this->img_type)
    {
        case TGA_RLE_Map:
        case TGA_RLE_RGB:
        case TGA_RLE_Mono:
            break;

        case TGA_Map:
        case TGA_RGB:
        case TGA_Mono:
            break;

        default:
            /* This is not a Targa I can deal with */
            /* (or it is not a Targa at all) */
            return 1 ;

            /*NOTREACHED*/
            break;
    }

    /* Load rest of Targa header */
    this->cmap_index    = get_le_word(fio);
    this->cmap_len      = get_le_word(fio);
    GET_BYTE( this->cmap_size , fio ) ;
    this->x_off         = get_le_word(fio);
    this->y_off         = get_le_word(fio);
    pImg->SetNumCols( get_le_word(fio) ) ;
    pImg->SetNumRows( get_le_word(fio) ) ;
    GET_BYTE( this->pixel_size , fio ) ;

    GET_BYTE(flags, fio);
    this->att_bits      =  flags & 0xf        ;
    this->reserved      = (flags & 0x10) >> 4 ;
    this->origin_bit    = (flags & 0x20) >> 5 ;
    this->interleave    = (flags & 0xc0) >> 6 ;

    /* Load the ID field */
    if( this->id_len )
    {
        unsigned char * id_field ;
        id_field = MY_CALLOC( this->id_len , unsigned char ) ;

        if( get_block( fio , id_field , this->id_len ) )
        {
            fprintf(stderr, "tgaHeaderRead: read error in id field\n");
            return EOF ;
        }
        FREE(id_field);
    }

    /* Verify the validity of the colormap or pixel size */
    if(     this->img_type == TGA_Map
        ||  this->img_type == TGA_RLE_Map
        ||  this->img_type == TGA_CompMap
        ||  this->img_type == TGA_CompMap4 )
    {
        if(this->cmap_type != 1)
        {
            /* There was no valid colormap, but one was required */
            return 2 ;
        }

        this->mapped = 1;

        switch(this->cmap_size)
        {
            case 8:
            case 24: case 32:
            case 15: case 16:
                break;

            default:
                /* invalid colormap entry size */
                return 3 ;
                /*NOTREACHED*/
                break;
        }

        if(     this->pixel_size != 8
            &&  this->pixel_size != 15
            &&  this->pixel_size != 16 )
        {
            return 7 ;
        }
    }
    else
    {

        this->mapped = 0;

        switch( this->pixel_size )
        {
            case 8:
            case 15: case 16:
            case 24: case 32:
                break;

            default:
                /* invalid pixel size */
                return 4 ;
                /*NOTREACHED*/
                break;
        }
    }

    if( this->cmap_type )
    {
        if(this->cmap_index + this->cmap_len > 16384)
        {
            /* colormap is invalid length */
            return 5 ;
        }

        #ifdef CMAP256
        if(this->cmap_index + this->cmap_len > 256)
        {
            /* colormap will not fit */
            return 6 ;
        }
        #endif

        tgaPixelRead( fio , & tga_cmap , this->cmap_index , this->cmap_len , this->cmap_size , 0 ) ;
    }
    return 0 ;
}




/* ---------------------------------------------------------------------- */

#define RPIX( row , col ) ( pImg->R( (row) * pImg->GetNumCols() + (col) ) )
#define GPIX( row , col ) ( pImg->G( (row) * pImg->GetNumCols() + (col) ) )
#define BPIX( row , col ) ( pImg->B( (row) * pImg->GetNumCols() + (col) ) )
#define APIX( row , col ) ( pImg->A( (row) * pImg->GetNumCols() + (col) ) )




/*! \brief save Targa pixels to fio from pImg

    \param fio - pointer to the output image file opened for binary output.

   \param pImg - used for the image arrays (and pImg->ncols used to index)

   \param npixels - number of consecutive pixels to write.

   \param mpsize - size of the pixels in memory, not the size of the
     pixels being written.
        - For mpsize 8, use only the red channel.
        - For mpsize 15|16, use the red and green channels.
            Use red as the MSB and green as the LSB.

   \param mapped - tells whether pixel values are gray/coded RGB, or cmap index
                    if mpsize==8|15|16 then mapped implies that a lookup ought to be
                    done, and the mapped pixel should be written.

    There are two kinds of map: 24/32 bit pixel and 15 bit pixel.
    "mpsize" refers to the size of the stored image, not the written
    image.  To date, mapped saves are not supported, so this issue is moot.

*/
static int tgaPixelWrite(FILE *fio, const RgbaImageT *pImg, int col, int row, int npixels, int mpsize, int mapped)
{
    switch(mpsize)
    {
        case 8:
            if(mapped)
            {
                fprintf( stderr , "tgaPixelWrite: mapped not yet supported\n" ) ;
                return EOF ;
            }
            {
                for( int pcount = col ; pcount < col + npixels ; pcount ++ )
                {
                    if( putc( RPIX(row, pcount), fio)==EOF )
                        return EOF ;
                }
            }
            break ;

        case 15: case 16:
            if(mapped)
            {
                fprintf(stderr, "tgaPixelWrite: I only do non-mapped 15/16\n");
                return EOF ;
            }
            {
                for(int pcount=col; pcount < col+npixels; pcount++)
                {
                    if(putc(GPIX(row, pcount), fio)==EOF) return EOF ;
                    if(putc(RPIX(row, pcount), fio)==EOF) return EOF ;
                }
            }
            break;

        case 32: case 24:
            if(mapped)
            {
                fprintf(stderr, "tgaPixelWrite: 24/32 can't be mapped\n");
            }
            {
                for(int pcount=col; pcount < col+npixels; pcount++)
                {
                    if(putc(BPIX(row, pcount), fio)==EOF) return EOF ;
                    if(putc(GPIX(row, pcount), fio)==EOF) return EOF ;
                    if(putc(RPIX(row, pcount), fio)==EOF) return EOF ;
                    if(mpsize == 32)
                    {
                        if(putc(APIX(row, pcount), fio)==EOF) return EOF ;
                    }
                }
            }
            break;

        default:
            fprintf(stderr, "tgaPixelWrite: bad pixel size %i\n", mpsize);
            return EOF ;
            /*NOTREACHED*/
            break;
    }
    return 0 ;
}




/*! \brief find RLE run length for Targa image file

    Find RLE run length at current col, row of img
    depth is the number of bits per pixel

    For depth 8 Use only red channel.
    For depth 15|16 Use red as the MSB and green as the LSB.

    Only runs along rows;  Will not read into next row.

    \return If pixel repeat 2 or 3 times, return negative of number of repeats
            otherwise Return positive number of distinct pixels until a 2|3+

            If error, return 0.

            A return value of 0 is indistinguishible from a start-at-end-of-row
            occurance, so the caller must check for end-of-row before calling
            this routine;
*/
int tgaRunLength( const RgbaImageT * pImg , int col, int row, int depth)
{
    int xi, ri;
    int run_length;

    switch(depth)
    {
        case 8: case 15: case 16: case 24: case 32:
            break;

        default:
            fprintf(stderr, "tgaRunLength: invalid depth %i\n", depth);
            return 0 ;
    }

    // Check for a run of (at least 2 or 3, at most 128) identical pixels.
    // Skip the first pixel;  it's obviously equal to itself.
    for( ri = col + 1 ; ( ri < pImg->GetNumCols() ) && ( ri - col < 128 ) ; ri ++ )
    {
        if(RPIX(row, ri) != RPIX(row, col)) break;
        if(depth>8)
        {
            if(GPIX(row, ri) != GPIX(row, col)) break;
            if(depth>16) {
                if(BPIX(row, ri) != BPIX(row, col)) break;
                if(depth==32) {
                    if(APIX(row, ri) != APIX(row, col)) break;
                }
            }
        }
    }
    run_length = ri - col ;

    switch( depth )
    {
        case 8:
            if(run_length>=3) return -run_length ;
            break;

        case 15: case 16: case 24: case 32:
            if(run_length>=2) return -run_length ;
            break;
    }

    /* If we've reached this far, we've reached a run of distinct pixels. */
    /* Look for runs of (at most 128) distinct pixels. */
    for( xi = col + 1 ; ( xi < pImg->GetNumCols() ) && ( xi - col < 128 ) ; xi += run_length )
    {
        for( ri = xi + 1 ; ( ri < pImg->GetNumCols() ) && ( ri - xi < 3 ) ; ri++)
        {
            if(RPIX(row, ri) != RPIX(row, xi)) break;
            if(depth>8)
            {
                if(GPIX(row, ri) != GPIX(row, xi)) break;
                if(depth>16)
                {
                    if(BPIX(row, ri) != BPIX(row, xi)) break;
                    if(depth==32)
                    {
                        if(APIX(row, ri) != APIX(row, xi)) break;
                    }
                }
            }
        }
        run_length=ri-xi;

        switch(depth)
        {
            case 8:
                if(run_length>=3) return (xi-col);
                break;

            case 15: case 16: case 24: case 32:
                if(run_length>=2) return (xi-col);
                break;
        }
    }
    return (xi-col);
}




/*! \brief Save a Targa image file

    Save a Targa image file into fio from pImg
*/
int ImageFileTga::Write( const RgbaImageT * pImg , FILE * fio )
{
    int     ilace_row   = 0 ;   // interlaced row counter
    bool    rl_encoded  ;       // Whether file uses run-length encoding
    int     row_count   = 0 ;   // total count of rows saved
    int     top_row     = 0 ;   // where to start over for interlaced images
    int     c_row       = 0 ;   // current row index being saved.  Note that when writing this does not depend on origin_bit.

    if(     this->img_type == TGA_RLE_Map
        ||  this->img_type == TGA_RLE_RGB
        ||  this->img_type == TGA_RLE_Mono )
    {
        rl_encoded = 1 ;
    }
    else
    {
        rl_encoded = 0 ;
    }

    if( rl_encoded )
    {
        ASSERT( this->pixel_size != 8 ) ; // This routine does not correctly write 8-bit RLE images.  I do not know why.

        while( row_count < pImg->GetNumRows() )
        {
            for( int col = 0 ; col < pImg->GetNumCols() ; )
            {
                int rle_count = tgaRunLength( pImg , col , c_row , this->pixel_size ) ;
                if( rle_count < 0 )
                {
                    /* Write the repeat count (negative) */
                    putc(127 - rle_count, fio);

                    /* Write out the pixels */
                    if( tgaPixelWrite( fio , pImg , col , c_row , 1 , this->pixel_size , this->mapped ) )
                    {
                        fprintf( stderr , "tgaWrite: write error in row %i\n" , c_row ) ;
                        return EOF ;
                    }

                    /* Advance the column counter */
                    col += -rle_count ;

                }
                else if( rle_count > 0 )
                {
                    /* Write the distinct count (positive) */
                    putc( rle_count - 1 , fio ) ;

                    /* Write out the pixels */
                    if( tgaPixelWrite( fio , pImg , col , c_row , rle_count , this->pixel_size , this->mapped ) )
                    {
                        fprintf( stderr , "tgaWrite: write error in row %i\n" , c_row ) ;
                        return EOF ;
                    }

                    /* Advance the column counter */
                    col += rle_count ;
                }
                else
                {
                    fprintf(stderr, "tgaWrite: bad RLE count %i\n", rle_count);
                }
            }
            ROW_INC_WRITE ;
        }
    }
    else
    {
        /* Not run-length encoded */
        /* save pixel data one row at a time */
        while( row_count < pImg->GetNumRows() )
        {
            if( tgaPixelWrite( fio , pImg , 0 , c_row , pImg->GetNumCols() , this->pixel_size , this->mapped ) )
            {
                fprintf(stderr, "tgaWrite: write error in row %i\n", c_row);
                return EOF ;
            }
            ROW_INC_WRITE ;
        }
    }

    return 0 ;
}




/*! \brief save a Targa image header into fio from this and pImg
*/
int ImageFileTga::HeaderWrite( const RgbaImageT * pImg , FILE * fio , TgaType eType , int bitsPerPixel )
{
    this->id_len      = 0 ;
    this->cmap_type   = 0 ;             // cmap_type depends on the img_type
    this->img_type    = eType ;         // img_type comes from the user
    this->cmap_index  = 0 ;
    this->cmap_len    = 0 ;             // cmap_len depends on img_type and pixel_size
    this->cmap_size   = 0 ;             // cmap_size depends on img_type and pixel_size
    this->x_off       = 0 ;
    this->y_off       = 0 ;
    this->pixel_size  = bitsPerPixel ;  // valud values for pixel_size depend on img_type
    this->att_bits    = 0 ;
    this->reserved    = 0 ;
    this->interleave  = TGA_IL_None ;

    putc(this->id_len, fio);
    putc(this->cmap_type, fio);
    putc(this->img_type, fio);

    put_le_word(this->cmap_index, fio);
    put_le_word(this->cmap_len, fio);
    putc(this->cmap_size, fio);
    put_le_word(this->x_off, fio);
    put_le_word(this->y_off, fio);
    put_le_word(pImg->GetNumCols() , fio);
    put_le_word(pImg->GetNumRows() , fio);
    putc(this->pixel_size, fio);

    unsigned char flags ;

    flags  =  this->att_bits   & 0xf ;
    flags |= (this->reserved   & 0x1) << 4 ;
    flags |= (this->origin_bit & 0x1) << 5 ;
    flags |= (this->interleave & 0x3) << 6 ;
    putc(flags, fio);

    if(this->cmap_type)
    {
        this->mapped = 1;

        /* Save the colormap for the Targa file */
        tgaPixelWrite(fio, &tga_cmap, 0, 0, this->cmap_len, this->cmap_size, 0);
    }
    else
    {
        this->mapped = 0;
    }

    return 0 ;
}




/*! \brief load image from a TGA file into memory.

    \param self - pointer to RgbaImage

    \param filename - filename

    Frees old image channel space.
    Allocates new image channel space.
*/
int RgbaImageReadFromTga( RgbaImageT * pImg , const char * strFilename )
{
    FILE        *   pInputFile        =   NULL    ;

    // Open the input file for binary reading
    if(     ( strFilename != NULL )
        &&  ( pInputFile = fopen( strFilename , "rb" ) ) == NULL )
    {
        fprintf( stderr , "RgbaImageReadFromTga: could not open '%s' for input\n" , strFilename ) ;
        return -1;
    }

    ImageFileTga    tga_hdr     ;

    // Load the image header.
    // This will set 'pImg' members such as ncols, nrows, etc.
    int tga_return = tga_hdr.HeaderRead( pImg , pInputFile ) ;
    if( tga_return )
    {
        fprintf( stderr , "RgbaImageReadFromTga: ERROR reading TGA header\n" ) ;
        return tga_return ;
    }

    // Allocate memory for the new image channels.
    if( pImg->Alloc() )
        return -1 ;

    // Load the image body
    tga_hdr.Read( pImg , pInputFile ) ;

    // Close the input file.
    fclose( pInputFile ) ;

    return 0 ;
}




/*! \brief save image to TGA file

    \param filename - file name to save image to

    \param pImg - address of start of image data

    \param strFilename - address of file name string

    \param eType -  file format to use to store the image

    \param bitsPerPixel - number of bits per pixel in the image buffer.
                Valid values can be 8, 15, 16, 24 or 32.

    \param bInvertVertically - whether image data in memory starts at bottom.

*/
int RgbaImageWriteToTga( const RgbaImageT * pImg , const char * strFilename , TgaType eType , int bitsPerPixel , bool bInvertVertically )
{
    FILE    *   pOutputFile = NULL ;      /* output file pointer */

    // Open the output image file for binary writing
    if(     ( strFilename != NULL )
        &&  ( pOutputFile = fopen( strFilename , "wb" ) ) == NULL )
    {
        fprintf( stderr , "RgbaImageWriteToTga: could not open '%s' for output\n" , strFilename ) ;
        return -1 ;
    }

    ImageFileTga    tga_hdr     ;

    tga_hdr.origin_bit = bInvertVertically ? 1 : 0 ;

    // Save the image header
    int tga_return = tga_hdr.HeaderWrite( pImg , pOutputFile , eType , bitsPerPixel ) ;
    if( tga_return )
    {
        fprintf( stderr , "RgbaImageWriteToTga: ERROR writing TGA header\n" ) ;
        return tga_return ;
    }

    // Save the image body
    tga_hdr.Write( pImg , pOutputFile ) ;

    fclose( pOutputFile ) ;

    return 0 ;
}




void TgaTest( void )
{
    {
        RgbaImageT imgTestRle ;

        static const nx = 240 ;
        imgTestRle.Alloc( nx , 1 ) ;

        unsigned char * row = & imgTestRle.R( 0 ) ;

        for( unsigned char ix = 1; ix<nx; ix++)
        {
            memset(row   , 0, ix   );
            memset(row+ix, 1, nx-ix);
            int rl = tgaRunLength(&imgTestRle, 0, 0, 8);
            printf("repeats ix=%i rl=%i\n", ix, rl);
        }

        row[0]=0;
        for( unsigned char ix = 1; ix<nx; ix++)
        {
            for( unsigned char iy = 0; iy<ix; iy++)
            {
                row[iy]=iy;
            }
            memset(row+ix, 255, nx-ix);
            int rl = tgaRunLength(&imgTestRle, 0, 0, 8);
            printf("distincts ix=%i rl=%i\n", ix, rl);
        }

        row[0]=0;
        for( unsigned char ix = 1; ix<nx; ix++)
        {
            for( unsigned char iy = 0; iy<=ix; iy++)
            {
                row[iy]=iy/2;
            }
            memset(row+ix, 255, nx-ix);
            int rl = tgaRunLength(&imgTestRle, 0, 0, 8);
            printf("double distincts ix=%i rl=%i\n", ix, rl);
        }

        row[0]=0;
        for( unsigned char ix = 1; ix<nx; ix++)
        {
            for( unsigned char iy = 0; iy<=ix; iy++)
            {
                row[iy]=iy/3;
            }
            memset(row+ix, 255, nx-ix);
            int rl = tgaRunLength(&imgTestRle, 0, 0, 8);
            printf("triple distincts ix=%i rl=%i\n", ix, rl);
        }
    }

    {   // Test writing and reading monochrome TGA files.
        RgbaImageT imgTestFileOrig ;
        imgTestFileOrig.Alloc( 256 , 256 , false ) ;
        imgTestFileOrig.CreateTestPattern( 0 ) ;
        int rvWrite = RgbaImageWriteToTga( & imgTestFileOrig , "imgMono.tga" , TGA_Mono , 8 , true ) ;
        fprintf( stderr , "rvWrite=%i\n", rvWrite ) ;

        RgbaImageT imgTestFileRead ;
        int rvRead  = RgbaImageReadFromTga( & imgTestFileRead , "imgMono.tga" ) ;
        fprintf( stderr , "rvRead=%i\n", rvRead ) ;
        if( imgTestFileOrig == imgTestFileRead )
        {
            fprintf( stderr , "images match\n" ) ;
        }
        else
        {
            fprintf( stderr , "ERROR: images DO NOT match\n" ) ;
        }
    }

    if( false )
    {   // Test writing and reading monochrome run-length encoded TGA files.
        // This does not currently work.
        RgbaImageT imgTestFileOrig ;
        imgTestFileOrig.Alloc( 256 , 256 , false ) ;
        imgTestFileOrig.CreateTestPattern( 0 ) ;
        int rvWrite = RgbaImageWriteToTga( & imgTestFileOrig , "imgMonoRle.tga" , TGA_RLE_Mono , 8 , true ) ;
        fprintf( stderr , "rvWrite=%i\n", rvWrite ) ;

        RgbaImageT imgTestFileRead ;
        int rvRead  = RgbaImageReadFromTga( & imgTestFileRead , "imgMonoRle.tga" ) ;
        fprintf( stderr , "rvRead=%i\n", rvRead ) ;
        if( imgTestFileOrig == imgTestFileRead )
        {
            fprintf( stderr , "images match\n" ) ;
        }
        else
        {
            fprintf( stderr , "ERROR: images DO NOT match\n" ) ;
        }
    }

    {   // Test writing and reading red+green+blue TGA files.
        RgbaImageT imgTestFileOrig ;
        imgTestFileOrig.Alloc( 256 , 256 , false ) ;
        imgTestFileOrig.CreateTestPattern( 1 ) ;
        int rvWrite = RgbaImageWriteToTga( & imgTestFileOrig , "imgRgb.tga" , TGA_RGB , 24 , true ) ;
        fprintf( stderr , "rvWrite=%i\n", rvWrite ) ;

        RgbaImageT imgTestFileRead ;
        int rvRead  = RgbaImageReadFromTga( & imgTestFileRead , "imgRgb.tga" ) ;
        fprintf( stderr , "rvRead=%i\n", rvRead ) ;
        if( imgTestFileOrig == imgTestFileRead )
        {
            fprintf( stderr , "images match\n" ) ;
        }
        else
        {
            fprintf( stderr , "ERROR: images DO NOT match\n" ) ;
        }
    }

    {   // Test writing and reading run-length encoded red+green+blue TGA files.
        RgbaImageT imgTestFileOrig ;
        imgTestFileOrig.Alloc( 256 , 256 , false ) ;
        imgTestFileOrig.CreateTestPattern( 2 ) ;
        int rvWrite = RgbaImageWriteToTga( & imgTestFileOrig , "imgRgbRle.tga" , TGA_RLE_RGB , 24 , true ) ;
        fprintf( stderr , "rvWrite=%i\n", rvWrite ) ;

        RgbaImageT imgTestFileRead ;
        int rvRead  = RgbaImageReadFromTga( & imgTestFileRead , "imgRgbRle.tga" ) ;
        fprintf( stderr , "rvRead=%i\n", rvRead ) ;
        if( imgTestFileOrig == imgTestFileRead )
        {
            fprintf( stderr , "images match\n" ) ;
        }
        else
        {
            fprintf( stderr , "ERROR: images DO NOT match\n" ) ;
        }
    }

    {   // Test writing and reading red+green+blue+alpha TGA files.
        RgbaImageT imgTestFileOrig ;
        imgTestFileOrig.Alloc( 256 , 256 , false ) ;
        imgTestFileOrig.CreateTestPattern( 3 ) ;
        int rvWrite = RgbaImageWriteToTga( & imgTestFileOrig , "imgRgba.tga" , TGA_RGB , 32 , true ) ;
        fprintf( stderr , "rvWrite=%i\n", rvWrite ) ;

        RgbaImageT imgTestFileRead ;
        int rvRead  = RgbaImageReadFromTga( & imgTestFileRead , "imgRgba.tga" ) ;
        fprintf( stderr , "rvRead=%i\n", rvRead ) ;
        if( imgTestFileOrig == imgTestFileRead )
        {
            fprintf( stderr , "images match\n" ) ;
        }
        else
        {
            fprintf( stderr , "ERROR: images DO NOT match\n" ) ;
        }
    }

    {   // Test writing and reading run-length encoded red+green+blue+alpha TGA files.
        RgbaImageT imgTestFileOrig ;
        imgTestFileOrig.Alloc( 256 , 256 , false ) ;
        imgTestFileOrig.CreateTestPattern( 4 ) ;
        int rvWrite = RgbaImageWriteToTga( & imgTestFileOrig , "imgRgbaRle.tga" , TGA_RLE_RGB , 32 , true ) ;
        fprintf( stderr , "rvWrite=%i\n", rvWrite ) ;

        RgbaImageT imgTestFileRead ;
        int rvRead  = RgbaImageReadFromTga( & imgTestFileRead , "imgRgbaRle.tga" ) ;
        fprintf( stderr , "rvRead=%i\n", rvRead ) ;
        if( imgTestFileOrig == imgTestFileRead )
        {
            fprintf( stderr , "images match\n" ) ;
        }
        else
        {
            fprintf( stderr , "ERROR: images DO NOT match\n" ) ;
        }
    }
}
