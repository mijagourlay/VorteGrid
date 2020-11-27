/*! \file particle.cpp

    \brief Basic particle for use in a visual effects particle system

    \see Accompanying articles for more information:
        http://software.intel.com/en-us/articles/fluid-simulation-for-video-games-part-1/

    \author Copyright 2009 Dr. Michael Jason Gourlay; All rights reserved.
*/

#include "Core/Math/vec3.h"
#include "Space/uniformGrid.h"

#include "particle.h"

/*! \brief Generate brick-of-bytes volumetric data files

    \param uFrame - frame number.  Used to generate filenames.

    This routine also appends filenames to an OGLE script file.

    Example Ogle file which can make use of the files this routine generates:

    \verbatim

windowSize: 512 512
#
dataView: Streamline[0] subset.x_range = 31 32 4
dataView: Streamline[0] subset.y_range = 54 57 4
dataView: Streamline[0] subset.z_range =  7 10 4
dataView: Streamline[0] colormap.type  = COLORMAP_SPECTRUM
dataView: Streamline[0] criterion      = STREAMLINE_CRITERION_ALL
dataView: Streamline[0] decimation     = 5
dataView: Streamline[0] active         = true
#
dataView: Vector_Field subset.selected_var = 0
dataView: Vector_Field subset.x_stride = 2
dataView: Vector_Field subset.y_stride = 2
dataView: Vector_Field subset.z_stride = 2
dataView: Vector_Field colormap.type   = COLORMAP_HOT_COLD
dataView: Vector_Field vector_glyph    = HEDGEHOG_VECTOR_CONES_SOLID_LIT
dataView: Vector_Field active          = true
#
dataView: Opacity_Renderer[0] subset.selected_var = -1
dataView: Opacity_Renderer[0] colormap.alpha      = 0.5
dataView: Opacity_Renderer[0] colormap[0].type    = COLORMAP_SPECTRUM
dataView: Opacity_Renderer[0] colormap[1].alpha   = 0.6
dataView: Opacity_Renderer[0] colormap[2].alpha   = 0.7
dataView: Opacity_Renderer[0] active              = true
#
# Entries for various data sets:
#
64x64x16 densityX00000-64x64x16.dat densityY00000-64x64x16.dat densityZ00000-64x64x16.dat
64x64x16 densityM00000-64x64x16.dat

    \endverbatim

*/
void UniformGrid<Particle>::GenerateBrickOfBytes( const char * strFilenameBase , unsigned uFrame ) const
{
#if 0 && ! defined( _XBOX )
    Vorton vortonMin , vortonMax ;
    ComputeStatistics( vortonMin , vortonMax ) ;
    Vec3 & vortMin = vortonMin.mVorticity ;
    Vec3 & vortMax = vortonMax.mVorticity ;
    Vec3 vortExtreme( MAX2( -vortMin.x , vortMax.x ) , MAX2( -vortMin.y , vortMax.y ) , MAX2( -vortMin.z , vortMax.z ) ) ;
    const float fVortMagMax = vortExtreme.Magnitude() ; // Not the correct value for |vorticity|_max but a reasonable approximation for visualization purposes.
    vortMax = vortExtreme ;
    vortMin = - vortMax ;
    Vec3 vortRange( vortMax - vortMin ) ;
    vortRange.x = MAX2( FLT_MIN , vortRange.x ) ;  // Avoid divide-by-zero below
    vortRange.y = MAX2( FLT_MIN , vortRange.y ) ;  // Avoid divide-by-zero below
    vortRange.z = MAX2( FLT_MIN , vortRange.z ) ;  // Avoid divide-by-zero below
    char strDataFilenames[4][ 256 ] ;
    sprintf( strDataFilenames[0] , "Vols/vortX%05u-%ux%ux%u.dat" , uFrame , GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) ) ;
    sprintf( strDataFilenames[1] , "Vols/vortY%05u-%ux%ux%u.dat" , uFrame , GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) ) ;
    sprintf( strDataFilenames[2] , "Vols/vortZ%05u-%ux%ux%u.dat" , uFrame , GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) ) ;
    sprintf( strDataFilenames[3] , "Vols/vortM%05u-%ux%ux%u.dat" , uFrame , GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) ) ;
    {
        char strScriptFilename[ 256 ] ;
        sprintf( strScriptFilename , "vort.ogle" ) ;
        FILE * pScriptFile = fopen( strScriptFilename , "a" ) ;
        fprintf( pScriptFile , "# vort ranges: {%9.7g,%9.7g,%9.7g} to {%9.7g,%9.7g,%9.7g}\n" , vortMin.x , vortMin.y , vortMin.z , vortMax.x , vortMax.y , vortMax.z ) ;
        fprintf( pScriptFile , "%ux%ux%u %s %s %s\n" , GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) , strDataFilenames[0] , strDataFilenames[1] , strDataFilenames[2] ) ;
        fprintf( pScriptFile , "%ux%ux%u %s\n" , GetNumPoints( 0 ) , GetNumPoints( 1 ) , GetNumPoints( 2 ) , strDataFilenames[3] ) ;
        fclose( pScriptFile ) ;
    }

    FILE * pDataFiles[4] ;
    pDataFiles[0] = fopen( strDataFilenames[0] , "wb" ) ;
    pDataFiles[1] = fopen( strDataFilenames[1] , "wb" ) ;
    pDataFiles[2] = fopen( strDataFilenames[2] , "wb" ) ;
    pDataFiles[3] = fopen( strDataFilenames[3] , "wb" ) ;
    if( ! pDataFiles[0] ) { ASSERT( 0 ) ; return ; }
    if( ! pDataFiles[1] ) { ASSERT( 0 ) ; return ; }
    if( ! pDataFiles[2] ) { ASSERT( 0 ) ; return ; }
    if( ! pDataFiles[3] ) { ASSERT( 0 ) ; return ; }
    const unsigned numGridPoints = Size() ;
    for( unsigned offset = 0 ; offset < numGridPoints ; ++ offset )
    {
        const Vorton &  rVorton         = (*this)[ offset ] ;
        const Vec3   &  vVorticity      = rVorton.mVorticity ;
        const float     fVortMag        = vVorticity.Magnitude() ;
        ASSERT( ( fVortMag >= 0.0f ) && ( fVortMag <= fVortMagMax ) ) ;
        const Vec3      vortShifted     = vVorticity - vortMin ;
        ASSERT( vortShifted.x >= 0.0f ) ;
        ASSERT( vortShifted.y >= 0.0f ) ;
        ASSERT( vortShifted.z >= 0.0f ) ;
        const Vec3      vort0to1        = Vec3( vortShifted.x / vortRange.x , vortShifted.y / vortRange.y , vortShifted.z / vortRange.z ) ;
        ASSERT( ( vort0to1.x >= 0.0f ) && ( vort0to1.x <= 1.0f ) ) ;
        ASSERT( ( vort0to1.y >= 0.0f ) && ( vort0to1.y <= 1.0f ) ) ;
        ASSERT( ( vort0to1.z >= 0.0f ) && ( vort0to1.z <= 1.0f ) ) ;
        const Vec3      vort0to255      = vort0to1 * 255.0f ;
        const float     vortMag0to255   = 255.0f * fVortMag / fVortMagMax ;
        //printf( "vortF %3g %3g %3g %3g %3g\n", vort0to255.x , vort0to255.y , vort0to255.z , vortMag0to255 ) ;
        int iVort[4] = { vort0to255.x , vort0to255.y , vort0to255.z , vortMag0to255 } ;
        ASSERT( ( iVort[0] >= 0 ) && ( iVort[0] <= 255 ) ) ;
        ASSERT( ( iVort[1] >= 0 ) && ( iVort[1] <= 255 ) ) ;
        ASSERT( ( iVort[2] >= 0 ) && ( iVort[2] <= 255 ) ) ;
        ASSERT( ( iVort[3] >= 0 ) && ( iVort[3] <= 255 ) ) ;
        //printf( "vortI %3i %3i %3i %3i\n", iVort[0] , iVort[1] , iVort[2] , iVort[3] ) ;
        unsigned char cVort[4] = { iVort[0] , iVort[1] , iVort[2] , iVort[3] } ;
        fwrite( & cVort[0] , 1 , 1 , pDataFiles[0] ) ;
        fwrite( & cVort[1] , 1 , 1 , pDataFiles[1] ) ;
        fwrite( & cVort[2] , 1 , 1 , pDataFiles[2] ) ;
        fwrite( & cVort[3] , 1 , 1 , pDataFiles[3] ) ;
        //printf( "vortC %3u %3u %3u %3u\n", cVort[0] , cVort[1] , cVort[2] , cVort[3] ) ;
    }

    // Write minimum and maximum values for each component.
    // Without this, OGLE will interpret the value to be signed, when used with hedgehog and streamline dataviews.
    fprintf( pDataFiles[0] , "MIN %g MAX %g\n" , vortMin.x , vortMax.x   ) ;
    fprintf( pDataFiles[1] , "MIN %g MAX %g\n" , vortMin.y , vortMax.y   ) ;
    fprintf( pDataFiles[2] , "MIN %g MAX %g\n" , vortMin.z , vortMax.z   ) ;
    fprintf( pDataFiles[3] , "MIN %g MAX %g\n" , 0         , fVortMagMax ) ;

    fclose( pDataFiles[0] ) ;
    fclose( pDataFiles[1] ) ;
    fclose( pDataFiles[2] ) ;
    fclose( pDataFiles[3] ) ;
#endif
}




/* static */ void Particle::UnitTest( void )
{
    Vector<Particle> particles ;
    static const unsigned numParticlesMax = 8 * 8 * 8 ;
}