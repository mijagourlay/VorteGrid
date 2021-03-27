//#pragma optimize( "" , off )
/** \file diagnostics.cpp

    \brief Diagnostic routines for InteSiVis

    \author Copyright 2009-2016 Dr. Michael Jason Gourlay; All rights reserved.  Contact me at mijagourlay.com for licensing.

    \see http://www.mijagourlay.com/

*/
#include "inteSiVis.h"

#include "Core/SpatialPartition/uniformGridMath.h"

#include "Core/Performance/perfBlock.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <stdarg.h> // for va_list, va_start, va_end


static const float  sOneMinusEpsilon        = 1.0f - FLT_EPSILON ;
static const float  sOnePlusEpsilon         = 1.0f + FLT_EPSILON ;

static const Vec4 sColorArrayCyan[] =
{   // Cyan
    Vec4( 0.0f , 0.6f , 0.2f , 0.0f ) ,
    Vec4( 0.0f , 0.7f , 0.4f , 0.1f ) ,
    Vec4( 0.0f , 0.8f , 0.6f , 0.2f ) ,
    Vec4( 0.0f , 0.9f , 0.8f , 0.5f ) ,
    Vec4( 0.0f , 1.0f , 1.0f , 1.0f ) ,
} ;
static const int NUM_COLORS_IN_ARRAY_CYAN = sizeof( sColorArrayCyan ) / sizeof( sColorArrayCyan[0] ) ;


static const Vec4 sColorArrayOrangeRed[] =
{
    Vec4( 1.0f , 0.5f , 0.0f , 0.0f ) ,
    Vec4( 1.0f , 0.5f , 0.0f , 0.1f ) ,
    Vec4( 1.0f , 0.2f , 0.0f , 0.2f ) ,
    Vec4( 1.0f , 0.1f , 0.0f , 0.5f ) ,
    Vec4( 1.0f , 0.0f , 0.0f , 1.0f ) ,
} ;
static const int NUM_COLORS_IN_ARRAY_ORANGE_RED = sizeof( sColorArrayOrangeRed ) / sizeof( sColorArrayOrangeRed[0] ) ;


static const Vec4 sColorArrayRainbow[] =
{   // Rainbow
    Vec4( 0.0f , 0.0f , 0.0f , 0.0f ) , // Black transparent
    Vec4( 0.0f , 0.0f , 1.0f , 0.3f ) , // Blue
    Vec4( 0.0f , 0.5f , 1.0f , 0.4f ) ,
    Vec4( 0.0f , 0.8f , 0.0f , 0.5f ) , // Green
    Vec4( 1.0f , 0.5f , 0.0f , 0.6f ) ,
    Vec4( 1.0f , 0.0f , 0.0f , 0.7f ) , // Red
    Vec4( 1.0f , 0.0f , 0.5f , 0.8f ) ,
} ;
static const int NUM_COLORS_IN_ARRAY_RAINBOW = sizeof( sColorArrayRainbow ) / sizeof( sColorArrayRainbow[0] ) ;


static const Vec4 sColorArrayRGBYMC[] =
{   // RGB YMC, suitable for signed distance values.
    Vec4( 1.0f , 0.0f , 0.0f , 0.1f ) , // Red
    Vec4( 0.0f , 0.8f , 0.0f , 0.1f ) , // Green
    Vec4( 0.0f , 0.0f , 1.0f , 0.5f ) , // Blue
    Vec4( 1.0f , 1.0f , 0.0f , 0.5f ) , // Yellow
    Vec4( 1.0f , 0.0f , 1.0f , 0.1f ) , // Magenta
    Vec4( 0.0f , 1.0f , 1.0f , 0.1f ) , // Cyan
} ;
static const int NUM_COLORS_IN_ARRAY_RGBYMC = sizeof( sColorArrayRGBYMC ) / sizeof( sColorArrayRGBYMC[0] ) ;


static const Vec4 sColorArrayMagentaGreen[] =
{   // Magenta-green, suitable for signed values.
    Vec4( 1.0f , 0.5f , 1.0f , 1.0f ) , // Pale magenta
    Vec4( 1.0f , 0.0f , 1.0f , 1.0f ) , // Magenta
    Vec4( 0.5f , 0.0f , 0.5f , 1.0f ) , // half-magenta
    Vec4( 0.0f , 0.5f , 0.0f , 1.0f ) , // half-green
    Vec4( 0.0f , 1.0f , 1.0f , 1.0f ) , // Green
    Vec4( 0.5f , 1.0f , 0.5f , 1.0f ) , // Pale green
} ;
static const int NUM_COLORS_IN_ARRAY_MAGENTA_GREEN = sizeof( sColorArrayMagentaGreen ) / sizeof( sColorArrayMagentaGreen[0] ) ;




struct ColorRamp
{
    Vec4 const *    _colorArray         ;
    int             _numColorsInArray   ;
} ;

static ColorRamp sGridColorRamp     = { sColorArrayCyan      , NUM_COLORS_IN_ARRAY_CYAN       } ;
static ColorRamp sVortonColorRamp   = { sColorArrayOrangeRed , NUM_COLORS_IN_ARRAY_ORANGE_RED } ;

static size_t   sNestedGridLayerToRender = 0 ;   /// Which layer in multi-grid to render
static int      sNestedGridLayerReSample = 0 ;   /// Whether to upsample or downsample nested grid




/** Cycle through colormaps used to render grid.
*/
static void CycleColorMap( ColorRamp & colorRamp )
{
    if( sColorArrayCyan == colorRamp._colorArray )
    {
        colorRamp._colorArray      = sColorArrayOrangeRed ;
        colorRamp._numColorsInArray = NUM_COLORS_IN_ARRAY_ORANGE_RED ;
    }
    else if( sColorArrayOrangeRed == colorRamp._colorArray )
    {
        colorRamp._colorArray      = sColorArrayRainbow ;
        colorRamp._numColorsInArray = NUM_COLORS_IN_ARRAY_RAINBOW ;
    }
    else if( sColorArrayRainbow == colorRamp._colorArray )
    {
        colorRamp._colorArray      = sColorArrayRGBYMC ;
        colorRamp._numColorsInArray = NUM_COLORS_IN_ARRAY_RGBYMC ;
    }
    else if( sColorArrayRGBYMC == colorRamp._colorArray )
    {
        colorRamp._colorArray      = sColorArrayMagentaGreen ;
        colorRamp._numColorsInArray = NUM_COLORS_IN_ARRAY_MAGENTA_GREEN ;
    }
    else if( sColorArrayMagentaGreen == colorRamp._colorArray )
    {
        colorRamp._colorArray      = sColorArrayCyan ;
        colorRamp._numColorsInArray = NUM_COLORS_IN_ARRAY_CYAN ;
    }
}




/** Cycle through colormaps used to render grid.
*/
void CycleGridColorMap()
{
    CycleColorMap( sGridColorRamp ) ;
}




/** Cycle through colormaps used to render diagnostic vortons.
*/
void CycleDiagnosticVortonColorMap()
{
    CycleColorMap( sVortonColorRamp ) ;
}




void CycleNestedGridLayer( int direction )
{
    sNestedGridLayerToRender += direction ;
    sNestedGridLayerToRender = Max2( size_t( 0 ) , sNestedGridLayerToRender ) ;
}




void ResampleNestedGridLayer( int direction )
{
    sNestedGridLayerReSample = direction ;
}




/// Set the projection matrix to orthographic, for rendering text.
void setOrthographicProjection()
{
    CheckGlError() ;

    // TODO: Store matrix mode

    // switch to projection mode
    glMatrixMode( GL_PROJECTION ) ;

    // save previous matrix which contains the settings for the perspective projection
    glPushMatrix() ;

    // reset matrix
    glLoadIdentity() ;

    int viewportXYWH[4] ;
    glGetIntegerv( GL_VIEWPORT , viewportXYWH ) ;

    if( ( 0 == viewportXYWH[ 2 ] ) || ( 0 == viewportXYWH[ 3 ] ) )
    {   // Zero-size viewport.
        // Render window is probably minimized, which is okay, but calling gluOrtho2D would result in an invalid value.
    }
    else
    {   // Viewport is probably valid.
        // set a 2D orthographic projection
        gluOrtho2D( 0 , viewportXYWH[ 2 ] , viewportXYWH[ 3 ] , 0 ) ;
    }

    // switch back to modelview mode
    glMatrixMode( GL_MODELVIEW ) ;

    glPushMatrix();

    glLoadIdentity();

    // TODO: Restore matrix mode

    CheckGlError() ;
}




/// Set the projection matrix to projection, for rendering 3D objects.
void restorePerspectiveProjection()
{
    CheckGlError() ;

    // TODO: Store matrix mode

    glMatrixMode(GL_PROJECTION);
    // restore previous projection matrix
    glPopMatrix();

    // get back to modelview mode
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix() ;

    // TODO: Restore matrix mode

    CheckGlError() ;
}




/// Render the given string using the given font.
static void oglRenderBitmapString( void * font , const char * string )
{
    for( const char * c = string ; *c != '\0' ; c ++ )
    {
        CheckGlError() ;

        glutBitmapCharacter( font , *c ) ;

        CheckGlError() ;
    }
}




static void oglRenderBitmapString( const Vec3 & pos , void * font , const Vec4 & color , const char * string , bool useScreenSpace )
{
    CheckGlError() ;

    if( useScreenSpace )
    {
        setOrthographicProjection() ;
    }

    glPushAttrib( GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT ) ; // Should include GL_ENABLE_BIT or GL_ALL_ATTRIB_BITS but for some reason that *sometimes* makes things disappear, like vortex lines.

    glDisable( GL_LIGHTING ) ;
    glDisable( GL_TEXTURE_2D ) ;
    glEnable( GL_BLEND ) ;
    glBlendFunc( GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA ) ;  // Typical alpha blending

    if( useScreenSpace )
    {   // Only when drawing in screen space...
        glDisable( GL_DEPTH_TEST ) ;    // Disable depth test so text appears as if in front of everything already drawn.
    }

    glColor4fv( reinterpret_cast< const GLfloat * >( & color ) ) ;

    //int viewportXYWH[4] ;
    //glGetIntegerv( GL_VIEWPORT , viewportXYWH ) ;

    glRasterPos3f( pos.x , pos.y , pos.z ) ;
    oglRenderBitmapString( font , string ) ;

    glPopAttrib() ;

    if( useScreenSpace )
    {
        restorePerspectiveProjection() ;
    }

    CheckGlError() ;
}




/// Render the given string at the given screen-space position using the given font.
void oglRenderString( const Vec3 & pos , void * font , const char * format , ... )
{
    char stringBuffer[ 512 ] ;
    va_list args ;
    va_start( args , format ) ;
    _vsnprintf( stringBuffer , sizeof( stringBuffer ) , format , args ) ;
    va_end( args ) ;
    oglRenderBitmapString( pos , font , Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) , stringBuffer , true ) ;
}




/// Render the given string at the given world-space position using the given font.
void oglRenderStringWorld( const Vec3 & pos , void * font , const Vec4 & color , const char * format , ... )
{
    char stringBuffer[ 512 ] ;
    va_list args ;
    va_start( args , format ) ;
    vsprintf( stringBuffer , format , args ) ;
    va_end( args ) ;
    oglRenderBitmapString( pos , font , color , stringBuffer , false ) ;
}




/** Return a value based on the distance of the given position to the camera target.

    Value lies in [0,1] where the value 1 is for any distance smaller than a grid cell.
*/
float InteSiVis::CameraFocusEmphasis( const Vec3 position ) const
{
    if( mEmphasizeCameraTarget )
    {
        const Vec3 &    target               = mQdCamera.GetTarget() ;
        const Vec3      displacementToLookat = position - target ;
        const float     distToEye            = displacementToLookat.Magnitude() ;
        const float     characteristicSize   = powf( mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid().GetCellVolume() , 0.33333333333333333333333f ) ;
        const float     distRatio            = Min2( characteristicSize / distToEye , 1.0f ) ;
        return Pow3( distRatio ) ;
    }
    else
    {
        return 1.0f ;
    }
}




/** Given a color ramp and a value in [0,1], compute and return a blended color.

    \param colorRamp - array of colors.

    \param numColorsInRamp - number of elements in colorRamp

    \param zeroToOne - value in [0,1] used to choose a color.

    This routine assumes the colors in the ramp should be uniformly distributed
    across the range [0,1] of input values.  So, colorRamp[0] corresponds to 0,
    colorRamp[numColorsInRamp-1] corresponds to 1, and all colorRamp values in
    between are evenly distributed in [0,1].

*/
static inline Vec4 GetColorFromRamp( const ColorRamp & colorRamp , float zeroToOne )
{
    if( IsNan( zeroToOne ) )
    {
        return Vec4( 1.0f , 0.0f , 1.0f , 0.1f ) ;
    }
    if( zeroToOne < 0.0f )
    {
        FAIL() ;
        return Vec4( 1.0f , 0.5f , 1.0f , 1.0f ) ;
    }
    if( zeroToOne > 1.0f )
    {
        FAIL() ;
        return Vec4( 1.0f , 1.0f , 1.0f , 1.0f ) ;
    }
    ASSERT( ( zeroToOne >= 0.0f ) && ( zeroToOne <= 1.0f ) ) ;
    const float         idxMinFlt               = zeroToOne * float( colorRamp._numColorsInArray - 1 ) * sOneMinusEpsilon ;
    const int           idxMin                  = int( idxMinFlt ) ;
    ASSERT( ( idxMin >= 0 ) && ( idxMin < colorRamp._numColorsInArray - 1 ) ) ;
    // Interpolate between adjacent colors within the ramp.
    const int           idxMax                  = idxMin + 1 ;
    ASSERT( ( idxMax >= 1 ) && ( idxMax < colorRamp._numColorsInArray ) ) ;
    const float         spanBetweenRampElements = sOnePlusEpsilon / float( colorRamp._numColorsInArray - 1 ) ;
    const float         tween                   = ( zeroToOne - spanBetweenRampElements * float( idxMin ) ) / spanBetweenRampElements ;
    ASSERT( ( tween >= 0.0f ) && ( tween <= 1.0f ) ) ;
    return colorRamp._colorArray[ idxMin ] * ( 1.0f - tween ) + colorRamp._colorArray[ idxMax ] * tween ;
}




static inline Vec4 GetColorForGridPoint( const float val0to1 , const Vec3 gridpointPosition )
{
    Vec4 color = GetColorFromRamp( sGridColorRamp , val0to1 ) ; // Color based on magnitude.
    color.w *= InteSiVis::GetInstance()->CameraFocusEmphasis( gridpointPosition ) ;  // Modulate opacity based on distance to camera target.
    return color ;
}




static inline void SetColorForGridPoint( const float val0to1 , const Vec3 gridpointPosition )
{
    Vec4 color = GetColorForGridPoint( val0to1 , gridpointPosition ) ;
    glColor4f( color.x , color.y , color.z , color.w ) ;
}




static inline void SetVertexForGridPoint( const float val0to1 , const Vec3 gridpointPosition )
{
    SetColorForGridPoint( val0to1 , gridpointPosition ) ;
    glVertex3f( gridpointPosition.x , gridpointPosition.y , gridpointPosition.z ) ;
}




static inline void SetVertexForGridPoint_Scalar( const UniformGrid< float > & grid , const unsigned ix , const unsigned iy , const unsigned iz , float valMin , float oneOverValueRange )
{
    const float &   value               = grid.Get( ix , iy , iz ) ;
    const float     val0to1             = ( value - valMin ) * oneOverValueRange ;
    const Vec3      gridpointPosition   = grid.PositionFromIndices( ix , iy , iz ) ;

    SetVertexForGridPoint( val0to1 , gridpointPosition ) ;
}




/** Draw a single edge of a grid cell, using color appropriate to the given grid values.

    \param grid             Grid of scalar values whose values to render.

    \param ix0, iy0, iz0    Indices of the first gridpoint.

    \param ix1, iy1, iz1    Indices of the second gridpoint.

    \param magMin           Minimum magnitude across all values in the grid.

    \param oneOverMagRange  Reciprocal of (magMax-magMin) where magMax is the maximum magnitude across all values in the grid.

    \param gridDecorations  Render mode for grid decorations.

    \see DrawGridCells_VectorMagnitude

*/
inline void DrawGridCellEdge_Scalar( const UniformGrid< float > & grid , const unsigned ix0 , const unsigned iy0 , const unsigned iz0
                                                                        , const unsigned ix1 , const unsigned iy1 , const unsigned iz1
                                                                        , float valMin , float oneOverValueRange , InteSiVis::GridDecorationsE /*gridDecorations*/ )
{
    CheckGlError() ;

    glBegin( GL_LINES ) ;

    SetVertexForGridPoint_Scalar( grid , ix0 , iy0 , iz0 , valMin , oneOverValueRange ) ;
    SetVertexForGridPoint_Scalar( grid , ix1 , iy1 , iz1 , valMin , oneOverValueRange ) ;

    glEnd() ;

    CheckGlError() ;
}




static bool sMakeRangeSymmetric = true ; // Make range [-r,r]




/** Draw grid cells, given a grid of scalar values.

    This is a rudimentary form of volumetric rendering.

    \param grid                     Grid of scalar values to draw.

    \param material                 Material to use to draw grid.

    \param gridDecorations          Render mode for grid decorations.

    \param bRenderDiagnosticText    Whether to render diagnostic text.

    \see DrawGridCells_VectorMagnitude
*/
//#error TODO: Add a mode that renders cube faces, not just edges.
static void DrawGridCells_Scalar( const UniformGrid< float > & grid , const QdMaterial & material , InteSiVis::GridDecorationsE gridDecorations , bool /*bRenderDiagnosticText*/ )
{
    ASSERT( ! grid.HasZeroExtent() ) ;

    material.UseMaterial() ;

    // Gather min and max statistics for grid contents
    float magMin ;
    float magMax ;
    FindValueRange( grid , magMin , magMax ) ;
    if( sMakeRangeSymmetric && ( magMin < 0.0f ) && ( magMax > 0.0f ) )
    {
        magMax = Max2( -magMin , magMax ) ;
        magMin = - magMax ;
    }
    const float oneOverMagRange = 1.0f / ( magMax - magMin + FLT_EPSILON ) ;    // Add FLT_EPSILON to avoid divide-by-zero when domain is empty.
    const bool speedRangeIsNonZero = magMax != magMin ;

    const Vec3  cellSize   = grid.GetCellSpacing() /* shrink cell to visualize each, otherwise edges overlap */ * 0.99f ;

    {
        // Draw each grid cell.
        Vec4            color   = material.GetColor() ;
        const unsigned  nx      = grid.GetNumCells( 0 ) ;
        const unsigned  ny      = grid.GetNumCells( 1 ) ;
        const unsigned  nz      = grid.GetNumCells( 2 ) ;
        unsigned        indices[ 4 ] ;
        unsigned &      ix      = indices[ 0 ] ;
        unsigned &      iy      = indices[ 1 ] ;
        unsigned &      iz      = indices[ 2 ] ;
        for( iz = 0 ; iz < nz ; ++ iz )
        {
            for( iy = 0 ; iy < ny ; ++ iy )
            {
                for( ix = 0 ; ix < nx ; ++ ix )
                {
                    if( speedRangeIsNonZero )
                    {
                        glLineWidth( material.GetLineWidth() ) ;

                        // Draw 3 faces of the current grid cell.
                        DrawGridCellEdge_Scalar( grid , ix   , iy   , iz   , ix+1 , iy   , iz   , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix   , iy+1 , iz   , ix+1 , iy+1 , iz   , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix   , iy   , iz   , ix   , iy+1 , iz   , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix+1 , iy   , iz   , ix+1 , iy+1 , iz   , magMin , oneOverMagRange , gridDecorations ) ;

                        DrawGridCellEdge_Scalar( grid , ix   , iy   , iz+1 , ix+1 , iy   , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix   , iy+1 , iz+1 , ix+1 , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix   , iy   , iz+1 , ix   , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix+1 , iy   , iz+1 , ix+1 , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;

                        DrawGridCellEdge_Scalar( grid , ix   , iy   , iz   , ix   , iy   , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix+1 , iy   , iz   , ix+1 , iy   , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix   , iy+1 , iz   , ix   , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Scalar( grid , ix+1 , iy+1 , iz   , ix+1 , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                    }
                }
            }
        }
    }
}




static inline Vec4 GetColorForGridPoint_Scalar( const UniformGrid< float > & grid , const unsigned ix , const unsigned iy , const unsigned iz , float magMin , float oneOverMagRange )
{
    const float &   value               = grid.Get( ix , iy , iz ) ;
    const float     valMag              = value ;
    const float     val0to1             = ( valMag - magMin ) * oneOverMagRange ;
    const Vec3      gridpointPosition   = grid.PositionFromIndices( ix , iy , iz ) ;

    const Vec4 color = GetColorForGridPoint( val0to1 , gridpointPosition ) ;
    glColor4f( color.x , color.y , color.z , color.w ) ;
    return color ;
}




static inline Vec4 GetColorForGridPoint_VectorMagnitude( const UniformGrid<Vec3> & grid , const unsigned ix , const unsigned iy , const unsigned iz , float magMin , float oneOverMagRange )
{
    const Vec3 &    value               = grid.Get( ix , iy , iz ) ;
    const float     valMag              = value.Magnitude() ;
    const float     val0to1             = ( valMag - magMin ) * oneOverMagRange ;
    const Vec3      gridpointPosition   = grid.PositionFromIndices( ix , iy , iz ) ;

    const Vec4 color = GetColorForGridPoint( val0to1 , gridpointPosition ) ;
    glColor4f( color.x , color.y , color.z , color.w ) ;
    return color ;
}




static inline void SetVertexForGridPoint_VectorMagnitude( const UniformGrid<Vec3> & grid , const unsigned ix , const unsigned iy , const unsigned iz , float magMin , float oneOverMagRange )
{
    const Vec3 &    value               = grid.Get( ix , iy , iz ) ;
    const float     valMag              = value.Magnitude() ;
    const float     val0to1             = ( valMag - magMin ) * oneOverMagRange ;
    const Vec3      gridpointPosition   = grid.PositionFromIndices( ix , iy , iz ) ;

    SetVertexForGridPoint( val0to1 , gridpointPosition ) ;
}




/** Draw a single edge of a grid cell, using color appropriate to the given grid values.

    \param grid             Grid of vector values whose magnitude to render.

    \param ix0, iy0, iz0    Indices of the first gridpoint.

    \param ix1, iy1, iz1    Indices of the second gridpoint.

    \param magMin           Minimum magnitude across all values in the grid.

    \param oneOverMagRange  Reciprocal of (magMax-magMin) where magMax is the maximum magnitude across all values in the grid.

    \param gridDecorations  Render mode for grid decorations.

    \see DrawGridCells_VectorMagnitude

*/
inline void DrawGridCellEdge_Vectors( const UniformGrid<Vec3> & grid    , const unsigned ix0 , const unsigned iy0 , const unsigned iz0
                                                                        , const unsigned ix1 , const unsigned iy1 , const unsigned iz1
                                                                        , float magMin , float oneOverMagRange , InteSiVis::GridDecorationsE /*gridDecorations*/ )
{
    glBegin( GL_LINES ) ;

    SetVertexForGridPoint_VectorMagnitude( grid , ix0 , iy0 , iz0 , magMin , oneOverMagRange ) ;
    SetVertexForGridPoint_VectorMagnitude( grid , ix1 , iy1 , iz1 , magMin , oneOverMagRange ) ;

    glEnd() ;
}




/** Draw grid cells, given a grid of vector values.

    This is a rudimentary form of volumetric rendering.

    \param grid                     Grid of vector values to draw.

    \param material                 Material to use to draw grid.

    \param gridDecorations          Render mode for grid decorations.

    \param bRenderDiagnosticText    Whether to render diagnostic text.

    \see DrawGridPoints_Vectors
*/
//#error TODO: Add a mode that renders cube faces, not just edges.
static void DrawGridCells_VectorMagnitude( const UniformGrid<Vec3> & grid , const QdMaterial & material , InteSiVis::GridDecorationsE gridDecorations , bool /*bRenderDiagnosticText*/ )
{
    ASSERT( ! grid.HasZeroExtent() ) ;

    material.UseMaterial() ;

    // Gather min and max statistics for grid contents (e.g. speed or density gradient)
    float magMin ;
    float magMax ;
    FindMagnitudeRange( grid , magMin , magMax ) ;
    magMin -= FLT_EPSILON * magMin ;  // Account for diff between fsqrtf and sqrtf
    magMax += FLT_EPSILON * magMax ;  // Account for diff between fsqrtf and sqrtf
    const float oneOverMagRange = 1.0f / ( magMax - magMin + FLT_EPSILON ) ;    // Add FLT_EPSILON to avoid divide-by-zero when domain is empty.
    const bool rangeIsNonZero = magMax != magMin ;

    const Vec3  cellSize   = grid.GetCellSpacing() /* shrink cell to visualize each, otherwise edges overlap */ * 0.99f ;

    {
        // Draw each grid cell.
        Vec4            color   = material.GetColor() ;
        const unsigned  nx      = grid.GetNumCells( 0 ) ;
        const unsigned  ny      = grid.GetNumCells( 1 ) ;
        const unsigned  nz      = grid.GetNumCells( 2 ) ;
        unsigned        indices[ 4 ] ;
        unsigned &      ix      = indices[ 0 ] ;
        unsigned &      iy      = indices[ 1 ] ;
        unsigned &      iz      = indices[ 2 ] ;
        for( iz = 0 ; iz < nz ; ++ iz )
        {
            for( iy = 0 ; iy < ny ; ++ iy )
            {
                for( ix = 0 ; ix < nx ; ++ ix )
                {
                    if( rangeIsNonZero )
                    {
                        glLineWidth( material.GetLineWidth() ) ;

                        // Draw 3 faces of the current grid cell.
                        DrawGridCellEdge_Vectors( grid , ix   , iy   , iz   , ix+1 , iy   , iz   , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix   , iy+1 , iz   , ix+1 , iy+1 , iz   , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix   , iy   , iz   , ix   , iy+1 , iz   , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix+1 , iy   , iz   , ix+1 , iy+1 , iz   , magMin , oneOverMagRange , gridDecorations ) ;

                        DrawGridCellEdge_Vectors( grid , ix   , iy   , iz+1 , ix+1 , iy   , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix   , iy+1 , iz+1 , ix+1 , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix   , iy   , iz+1 , ix   , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix+1 , iy   , iz+1 , ix+1 , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;

                        DrawGridCellEdge_Vectors( grid , ix   , iy   , iz   , ix   , iy   , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix+1 , iy   , iz   , ix+1 , iy   , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix   , iy+1 , iz   , ix   , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                        DrawGridCellEdge_Vectors( grid , ix+1 , iy+1 , iz   , ix+1 , iy+1 , iz+1 , magMin , oneOverMagRange , gridDecorations ) ;
                    }
                }
            }
        }
    }
}




/** Draw grid points, given a grid of vector values.

    This is a rudimentary form of volumetric rendering, which is sometimes called a hedgehog.

    \param grid                     Grid of vector values to draw.

    \param material                 Material to use to draw grid.

    \param gridDecorations          Render mode for grid decorations.

    \param bRenderDiagnosticText    Whether to render diagnostic text.

    \see DrawGridCells_VectorMagnitude
*/
static void DrawGridPoints_Vectors( const UniformGrid<Vec3> & grid , const QdMaterial & material , InteSiVis::GridDecorationsE gridDecorations , bool bRenderDiagnosticText )
{
    ASSERT( ! grid.HasZeroExtent() ) ;

    // Draw bounding box
    material.UseMaterial() ;

    // Gather min and max statistics for grid contents (speeds)
    float magMin ;
    float magMax ;
    FindMagnitudeRange( grid , magMin , magMax ) ;
    magMin -= FLT_EPSILON * magMin ;  // Account for diff between fsqrtf and sqrtf
    magMax += FLT_EPSILON * magMax ;  // Account for diff between fsqrtf and sqrtf
    const float oneOverMagRange = 1.0f / ( magMax - magMin + FLT_EPSILON ) ;    // Add FLT_EPSILON to avoid divide-by-zero when domain is empty.

    const Vec3  cellSize   = grid.GetCellSpacing() /* shrink cell to visualize each, otherwise edges overlap */ * 0.99f ;
    const float cellLength = powf( grid.GetCellVolume() , 0.33333333333333333f ) ;

    const float lineWidth = ( ( InteSiVis::GRID_DECO_POINTS == gridDecorations ) ? 3.0f :  5.0f ) ; // thinner for just points, thicker for points and cells.
    const float pointSize = ( ( InteSiVis::GRID_DECO_POINTS == gridDecorations ) ? 5.0f : 11.0f ) ; // thinner for just points, thicker for points and cells.

    {
        // Draw each grid cell.
        const unsigned  nx      = grid.GetNumPoints( 0 ) ;
        const unsigned  ny      = grid.GetNumPoints( 1 ) ;
        const unsigned  nz      = grid.GetNumPoints( 2 ) ;
        const unsigned  npx     = grid.GetNumPoints( 0 ) ;
        const unsigned  npy     = grid.GetNumPoints( 1 ) ;
        const unsigned  npxy    = npx * npy ;
        unsigned        indices[ 4 ] ;
        unsigned &      ix      = indices[ 0 ] ;
        unsigned &      iy      = indices[ 1 ] ;
        unsigned &      iz      = indices[ 2 ] ;
        for( iz = 0 ; iz < nz ; ++ iz )
        {
            const unsigned zOffset = iz * npxy ;
            for( iy = 0 ; iy < ny ; ++ iy )
            {
                const unsigned yzOffset = zOffset + iy * npx ;
                for( ix = 0 ; ix < nx ; ++ ix )
                {
                    const unsigned xyzOffset = ix + yzOffset ;

                    {
                        DEBUG_ONLY( unsigned testIndices[ 4 ] ) ;
                        DEBUG_ONLY( grid.IndicesFromOffset( testIndices , xyzOffset ) ) ;
                        ASSERT( ( testIndices[0] == ix ) && ( testIndices[1] == iy ) && ( testIndices[2] == iz ) ) ;
                    }

                    const Vec3 & value  = grid[ xyzOffset ] ;
                    const float valMag  = value.Magnitude() ;
                    const float val0to1 = ( valMag - magMin ) * oneOverMagRange ;
                    const Vec3 gridpointPosition = grid.PositionFromIndices( indices ) ;

                    const Vec4 color = GetColorForGridPoint_VectorMagnitude( grid , ix , iy , iz , magMin , oneOverMagRange ) ;

                    if( bRenderDiagnosticText && color.w > 0.01f )
                    {   // Diagnostic text is enabled and opacity is sufficient to render it.
                    #if 1
                        // Draw value magnitude
                        oglRenderStringWorld( gridpointPosition , GLUT_BITMAP_HELVETICA_10 , 2.0f * color , "%g" , value.Magnitude() ) ;
                    #else
                        // Draw position and value.
                        oglRenderStringWorld( gridpointPosition , GLUT_BITMAP_HELVETICA_10 , 2.0f * color , "%g,%g,%g %g,%g,%g"
                            , gridpointPosition.x , gridpointPosition.y , gridpointPosition.z
                            , value.x , value.y , value.z
                            ) ;
                    #endif
                    }
                    {
                        // Draw an arrow indicating value direction and magnitude.
                        // Scale arrow according to cell size and speed.
                        Vec3 arrowEnd( gridpointPosition + value.GetDir() * val0to1 * cellLength ) ;
                        glLineWidth( lineWidth ) ;
                        glBegin( GL_LINES ) ;
                        glVertex3fv( reinterpret_cast< const GLfloat * >( & gridpointPosition ) ) ;
                        glVertex3fv( reinterpret_cast< const GLfloat * >( & arrowEnd  ) ) ;
                        glEnd() ;
                    }
                    {
                        glPointSize( pointSize ) ;
                        glBegin( GL_POINTS ) ;
                        glVertex3fv( reinterpret_cast< const GLfloat * >( & gridpointPosition ) ) ;
                        glEnd() ;
                    }
                }
            }
        }
    }
}




/** Draw grid points, given a grid of scalar values.

    This is a rudimentary form of volumetric rendering.

    \param grid                     Grid of scalar values to draw.

    \param material                 Material to use to draw grid.

    \param gridDecorations          Render mode for grid decorations.

    \param bRenderDiagnosticText    Whether to render diagnostic text.

    \see DrawGridCells_VectorMagnitude
*/
static void DrawGridPoints_Scalars( const UniformGrid< float > & grid , const QdMaterial & material , InteSiVis::GridDecorationsE gridDecorations , bool bRenderDiagnosticText )
{
    ASSERT( ! grid.HasZeroExtent() ) ;

    // Draw bounding box
    material.UseMaterial() ;

    // Gather min and max statistics for grid contents (speeds)
    float magMin ;
    float magMax ;
    FindValueRange( grid , magMin , magMax ) ;
    if( sMakeRangeSymmetric && ( magMin < 0.0f ) && ( magMax > 0.0f ) )
    {
        magMax = Max2( -magMin , magMax ) ;
        magMin = - magMax ;
    }
    const float oneOverMagRange = 1.0f / ( magMax - magMin + FLT_EPSILON ) ;    // Add FLT_EPSILON to avoid divide-by-zero when domain is empty.

    const Vec3  cellSize   = grid.GetCellSpacing() /* shrink cell to visualize each, otherwise edges overlap */ * 0.99f ;

    const float pointSize = ( ( InteSiVis::GRID_DECO_POINTS == gridDecorations ) ? 5.0f : 11.0f ) ; // thinner for just points, thicker for points and cells.
    //const float pointSize = ( ( InteSiVis::GRID_DECO_POINTS == gridDecorations ) ? 8.0f : 4.0f  ) ; // thicker for just points, thinner for points and cells.

    {
        // Draw each grid point.
        const unsigned  nx      = grid.GetNumPoints( 0 ) ;
        const unsigned  ny      = grid.GetNumPoints( 1 ) ;
        const unsigned  nz      = grid.GetNumPoints( 2 ) ;
        const unsigned  npx     = grid.GetNumPoints( 0 ) ;
        const unsigned  npy     = grid.GetNumPoints( 1 ) ;
        const unsigned  npxy    = npx * npy ;
        unsigned        indices[ 4 ] ;
        unsigned &      ix      = indices[ 0 ] ;
        unsigned &      iy      = indices[ 1 ] ;
        unsigned &      iz      = indices[ 2 ] ;
        for( iz = 0 ; iz < nz ; ++ iz )
        {
            const unsigned zOffset = iz * npxy ;
            for( iy = 0 ; iy < ny ; ++ iy )
            {
                const unsigned yzOffset = zOffset + iy * npx ;
                for( ix = 0 ; ix < nx ; ++ ix )
                {
                    const unsigned xyzOffset = ix + yzOffset ;

                    {
                        DEBUG_ONLY( unsigned testIndices[ 4 ] ) ;
                        DEBUG_ONLY( grid.IndicesFromOffset( testIndices , xyzOffset ) ) ;
                        ASSERT( ( testIndices[0] == ix ) && ( testIndices[1] == iy ) && ( testIndices[2] == iz ) ) ;
                    }

                    const float & value  = grid[ xyzOffset ] ;
                    const Vec3 gridpointPosition = grid.PositionFromIndices( indices ) ;

                    const Vec4 color = GetColorForGridPoint_Scalar( grid , ix , iy , iz , magMin , oneOverMagRange ) ;

                    if( bRenderDiagnosticText && color.w > 0.01f )
                    {   // Diagnostic text is enabled and opacity is sufficient to render it.
                    #if 1
                        // Draw value
                        oglRenderStringWorld( gridpointPosition , GLUT_BITMAP_HELVETICA_10 , 2.0f * color , "%g" , value ) ;
                    #else
                        // Draw position and value.
                        oglRenderStringWorld( gridpointPosition , GLUT_BITMAP_HELVETICA_10 , 2.0f * color , "%g,%g,%g %g"
                            , gridpointPosition.x , gridpointPosition.y , gridpointPosition.z
                            , value
                            ) ;
                    #endif
                    }
                    {
                        glPointSize( pointSize ) ;
                        glBegin( GL_POINTS ) ;
                        glVertex3fv( reinterpret_cast< const GLfloat * >( & gridpointPosition ) ) ;
                        glEnd() ;
                    }
                }
            }
        }
    }
}



/** Draw bounding box of given grid.
*/
static void DrawBoundingBox( const UniformGridGeometry & gridGeometry , const QdMaterial & material )
{
    if( gridGeometry.HasZeroExtent() )
    {   // Grid has no size.
        return ;    // There is nothing to draw.
    }

    // Draw bounding box
    material.UseMaterial() ;
    glLineWidth( material.GetLineWidth() ) ;
    {
        glPushMatrix() ;

        const Vec3 vCenter = gridGeometry.GetCenter() ;
        glTranslatef( vCenter.x , vCenter.y , vCenter.z ) ;
        const Vec3 gridExtent = gridGeometry.GetExtent() * 1.01f ;
        glScalef( gridExtent.x , gridExtent.y , gridExtent.z ) ;

        glutWireCube( 1.0 ) ;

        glPopMatrix() ;
    }
}




/** Draw grid of vector values.

    This is a rudimentary form of volumetric rendering.

    \param grid                     Grid of vector values to draw.

    \param material                 Material to use to draw grid.

    \param gridDecorations          Render mode for grid decorations.

    \param bRenderDiagnosticText    Whether to render diagnostic text.

    \see DrawBoundingBox, DrawGridCells_VectorMagnitude, DrawGridPoints_Vectors
*/
static void DrawGrid_Vectors( const UniformGrid<Vec3> & grid , const QdMaterial & material , InteSiVis::GridDecorationsE gridDecorations , bool bRenderDiagnosticText )
{
    if( grid.HasZeroExtent() )
    {   // Grid has no size.
        return ;    // There is nothing to draw.
    }

    // Draw bounding box using same grid as for field.  In principle (e.g. if some code has a bug) those could differ.
    // Note: Not all fields are populated for all simulations, so depending on the diagnostic field, the bounding might not render.
    //      Could instead choose to render which ever grid is not empty but then rendering would ambiguous regaring which box was rendered.
    DrawBoundingBox( grid , material ) ;

    if( grid.Empty() )
    {   // Grid has no data.
        return ;    // There is no data to draw.
    }

    material.UseMaterial() ;
    if(     ( InteSiVis::GRID_DECO_CELLS            == gridDecorations )
        ||  ( InteSiVis::GRID_DECO_CELLS_AND_POINTS == gridDecorations ) )
    {
        DrawGridCells_VectorMagnitude( grid , material , gridDecorations , bRenderDiagnosticText ) ;
    }
    if(     ( InteSiVis::GRID_DECO_POINTS           == gridDecorations )
        ||  ( InteSiVis::GRID_DECO_CELLS_AND_POINTS == gridDecorations ) )
    {
        DrawGridPoints_Vectors( grid , material , gridDecorations , bRenderDiagnosticText ) ;
    }
}




/** Draw grid of scalar values.

    This is a rudimentary form of volumetric rendering.

    \param grid                     Grid of scalar values to draw.

    \param material                 Material to use to draw grid.

    \param gridDecorations          Render mode for grid decorations.

    \param bRenderDiagnosticText    Whether to render diagnostic text.

    \see DrawBoundingBox, DrawGridCells_VectorMagnitude, DrawGridPoints_Vectors
*/
static void DrawGrid_Scalars( const UniformGrid< float > & grid , const QdMaterial & material , InteSiVis::GridDecorationsE gridDecorations , bool bRenderDiagnosticText )
{
    if( grid.HasZeroExtent() )
    {   // Grid has no size.
        return ;    // There is nothing to draw.
    }

    // Draw bounding box using same grid as for field.  In principle (e.g. if some code has a bug) those could differ.
    // Note: Not all fields are populated for all simulations, so depending on the diagnostic field, the bounding might not render.
    //      Could instead choose to render which ever grid is not empty but then rendering would ambiguous regaring which box was rendered.
    DrawBoundingBox( grid , material ) ;

    if( grid.Empty() )
    {   // Grid has no data.
        return ;    // There is no data to draw.
    }

    material.UseMaterial() ;
    if(     ( InteSiVis::GRID_DECO_CELLS            == gridDecorations )
        ||  ( InteSiVis::GRID_DECO_CELLS_AND_POINTS == gridDecorations ) )
    {
        DrawGridCells_Scalar( grid , material , gridDecorations , bRenderDiagnosticText ) ;
    }
    if(     ( InteSiVis::GRID_DECO_POINTS           == gridDecorations )
        ||  ( InteSiVis::GRID_DECO_CELLS_AND_POINTS == gridDecorations ) )
    {
        DrawGridPoints_Scalars( grid , material , gridDecorations , bRenderDiagnosticText ) ;
    }
}




void InteSiVis::SetDiagnosticPropertyValueRangeScale( float & rangeScale ) const
{
    VortonSim &                 rVortonSim  = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;

    float valMin = FLT_MAX , valMax = - FLT_MAX , valMean = 0.0f , valStdDev = 0.0f ;
    switch( mVortonProperty )
    {
    case VORTON_PROPERTY_POSITION:
        break ;

    case VORTON_PROPERTY_VELOCITY:
        {
            Vec3 centerOfVelocity ;
            Particles::ComputeVelocityStats( * reinterpret_cast< VECTOR< Particle > * >( mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVortons() ) , valMin , valMax , valMean , valStdDev , centerOfVelocity ) ;
        }
        break ;

    case VORTON_PROPERTY_VORTICITY:
        {
            Vec3 centerOfVorticity ;
            rVortonSim.GatherVorticityStats( valMin , valMax , valMean , valStdDev , centerOfVorticity ) ;
        }
        break ;

    case VORTON_PROPERTY_DENSITY:
        rVortonSim.GatherDensityStats( valMin , valMax , valMean , valStdDev ) ;
        break ;

    case VORTON_PROPERTY_DENSITY_SPH:
        rVortonSim.GatherDensitySphStats( valMin , valMax , valMean , valStdDev ) ;
        break ;

    case VORTON_PROPERTY_DENSITY_GRADIENT:
        rVortonSim.GatherDensityGradientStats( valMin , valMax , valMean , valStdDev ) ;
        break ;

    case VORTON_PROPERTY_PROXIMITY:
        rVortonSim.GatherProximityStats( valMin , valMax , valMean , valStdDev ) ;
        break ;
    }
    valMin *= ( 1.0f - FLT_EPSILON ) ;  // Account for diff between fsqrtf and sqrtf
    valMax *= ( 1.0f + FLT_EPSILON ) ;  // Account for diff between fsqrtf and sqrtf
    const float valMag          = Max2( fabsf( valMin ) , fabsf( valMax ) ) ;
    const float oneOverValMag   = 1.0f / Max2( valMag , FLT_EPSILON ) ;    // FLT_EPSILON to avoid divide-by-zero when range is empty.

    rangeScale = oneOverValMag ;
}




/** Render diagnostic text for vortex particles.
*/
void InteSiVis::QdRenderVortonDiagnosticText()
{
    PERF_BLOCK( InteSiVis__QdRenderVortonDiagnosticText ) ;

    ASSERT( mDiagnosticText == DIAG_TEXT_FULL ) ;

    // Enable opacity blending.
    glEnable( GL_BLEND ) ;
    glBlendFunc( GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA ) ;  // Typical alpha blending

    VortonSim &                 rVortonSim  = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
    VECTOR< Vorton > &          vortons     = * rVortonSim.GetVortons() ;
    const UniformGrid< Vec3 > & velGrid     = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid() ;

    float rangeScale ;
    SetDiagnosticPropertyValueRangeScale( rangeScale ) ;

    size_t numVortons = vortons.size() ;
    for( size_t iVort = 0 ; iVort < numVortons ; ++ iVort )
    {   // For each vorton...
        const Vorton & vort = vortons[ iVort ] ;

        if( velGrid.Encompasses( vort.mPosition ) )
        {   // Vorton is still inside grid.

            // Note that since this gets rendered after vorton advected, the
            // vorton is no longer necessarily near the gridpoint it was nearest
            // pre-advect. That means grid-derived diagnostic information is
            // suspect and has a good chance of being wrong fairly often.

            void * font = GLUT_BITMAP_HELVETICA_10 ;
            Vec4   color( 0.2f , 1.0f , 0.2f , 0.25f ) ;

            // Make opacity depend on distance to gridpoint.
            //color.w = 0.25f * Min2( vort.GetRadius() / distToNearestGridPoint , 1.0f ) ;

            unsigned    nearestGridPointIndices[ 4 ]    ; // indices of grid cell containing vorton.
            velGrid.IndicesOfNearestGridPoint( nearestGridPointIndices , vort.mPosition ) ; // Get indices of grid cell containing vorton.
            const Vec3  nearestGridPointPosition        = velGrid.PositionFromIndices( nearestGridPointIndices ) ;
            const Vec3  displacementToNearestGridPoint  = nearestGridPointPosition - vort.mPosition ;
            const float distToNearestGridPoint          = displacementToNearestGridPoint.Magnitude() ;

            if( ( distToNearestGridPoint < vort.GetRadius() ) && ( VORTON_PROPERTY_POSITION == mVortonProperty ) )
            {   // Gridpoint is inside vorton, and property being rendered is position.
                font  = GLUT_BITMAP_HELVETICA_12 ;
                color = Vec4( 1.0f , 0.2f , 0.2f , 0.5f ) ;
            }

            // Render diagnostic information about the vorton.
            char vortonPropertyText[ 128 ] = {} ;
            float val0to1 = 0.0f ;
            switch( mVortonProperty )
            {
            case VORTON_PROPERTY_POSITION:
                val0to1 = 1.0f ;
                sprintf( vortonPropertyText , "p=%g,%g,%g D=%g,%g,%s"
                    , vort.mPosition.x , vort.mPosition.y , vort.mPosition.z
                    , distToNearestGridPoint , distToNearestGridPoint / vort.GetRadius() , distToNearestGridPoint < vort.GetRadius() ? "IN" : "out" ) ;
                break ;

            case VORTON_PROPERTY_VELOCITY:
                val0to1 = vort.mVelocity.Magnitude() * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                sprintf( vortonPropertyText , "v=<%g,%g,%g> ||=%g"
                    , vort.mVelocity.x , vort.mVelocity.y , vort.mVelocity.z
                    , vort.mVelocity.Magnitude() ) ;
                break ;

            case VORTON_PROPERTY_VORTICITY:
                val0to1 = vort.GetVorticity().Magnitude() * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                sprintf( vortonPropertyText , "w=<%g,%g,%g> ||=%g"
                    , vort.GetVorticity().x , vort.GetVorticity().y , vort.GetVorticity().z
                    , vort.GetVorticity().Magnitude() ) ;
                break ;

            case VORTON_PROPERTY_DENSITY:
                val0to1 = vort.GetDensity() * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                sprintf( vortonPropertyText , "d=%g" , vort.GetDensity() ) ;
                break ;

            case VORTON_PROPERTY_DENSITY_SPH:
                {
                    const float & densitySph = rVortonSim.GetFluidDensitiesAtPcls()[ iVort ].mMassDensity ;
                    val0to1 = densitySph * rangeScale ;
                    color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                    sprintf( vortonPropertyText , "d=%g" , densitySph ) ;
                }
                break ;

            case VORTON_PROPERTY_DENSITY_GRADIENT:
            #if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
                if( ! rVortonSim.GetDensityGradientsAtPcls().Empty() )
                {
                    const Vec3 & densGrad = rVortonSim.GetDensityGradientsAtPcls()[ iVort ] ;
                    val0to1 = densGrad.Magnitude() * rangeScale ;
                    color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                    sprintf( vortonPropertyText , "dens grad=<%g,%g,%g> ||=%g"
                        , densGrad.x , densGrad.y , densGrad.z
                        , densGrad.Magnitude() ) ;
                }
            #endif
                break ;

            case VORTON_PROPERTY_PROXIMITY:
                {
                    const float & proximity = rVortonSim.GetProximities()[ iVort ] ;
                    const float val0to1     = proximity * rangeScale ;
                    color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                    sprintf( vortonPropertyText , "Prox=%g" , proximity ) ;
                }
                break ;

            }

            // Modulate opacity based on distance to camera target.
            color.w *= CameraFocusEmphasis( vort.mPosition ) ;

            if( color.w > 0.01f )
            {   // Diagnostic information is opaque enough to render.
                oglRenderStringWorld( vort.mPosition , font , color , "%i %s"
                    , iVort
                    , vortonPropertyText
                    ) ;
            }
        }
    }
}




#if ENABLE_PARTICLE_POSITION_HISTORY
/** Render particle pathlines.
*/
void InteSiVis::QdRenderDiagnosticPathlines( VECTOR< Particle > & particles )
{
    PERF_BLOCK( InteSiVis__QdRenderDiagnosticPathlines ) ;

    mPathlineMaterial.UseMaterial() ;

    // ASSERT( particles.empty() || particles.data() == & particles[0] ) ; // Modern STL supports vector.data() but not older versions like MSVS7.

    size_t numParticles = particles.size() ;
    for( size_t iVort = 0 ; iVort < numParticles ; ++ iVort )
    {   // For each particle...
        const Particle & pcl = particles[ iVort ] ;

        Vec4   color( mPathlineMaterial.GetColor() ) ;

        // Modulate opacity based on distance to camera target.
        color.w *= CameraFocusEmphasis( pcl.mPosition ) ;

        // Render pathline for current particle.
        glBegin( GL_LINE_STRIP ) ;

        static const float oneOverNumHistoryPositions = 1.0f / float( Particle::NUM_HISTORICAL_POSITIONS ) ;
        size_t segmentIndex = 0 ;
        const size_t iHistoryEnd = pcl.HistoryEnd() ;
        for( size_t iHistory = pcl.HistoryBegin() ; iHistory != iHistoryEnd ; iHistory = Particle::NextHistoryIndex( iHistory ) )
        {   // For each element in the particle position history...
            // Fade each segment: more recent is more opaque.
            const float fade = float( segmentIndex ) * oneOverNumHistoryPositions ;
            glColor4f( color.x , color.y , color.z , color.w * fade ) ;
            if( pcl.mPositionHistory[ iHistory ].x != Particle::sNaN )
            {   // This history element is valid.
                // (Invalid element could come from particles too young to have full history.)
                glVertex3fv( reinterpret_cast< const GLfloat * >( & pcl.mPositionHistory[ iHistory ] ) ) ;
            }
            ++ segmentIndex ;
        }
        ASSERT( pcl.mPositionHistory[ iHistoryEnd ].x != Particle::sNaN ) ; // Last element should always be valid.
        glVertex3fv( reinterpret_cast< const GLfloat * >( & pcl.mPositionHistory[ iHistoryEnd ] ) ) ;
        glEnd() ;
    }
}
#endif




/** Render diagnostic vectors for vortex particles.
*/
void InteSiVis::QdRenderVortonDiagnosticVectors()
{
    PERF_BLOCK( InteSiVis__QdRenderVortonDiagnosticVectors ) ;

    ASSERT(     ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_VECTORS )
            ||  ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_PARTICLES_AND_VECTORS )
            ||  ( mFluidScene.GetVortonRenderingStyle() == FluidScene::VORTON_RENDER_ALL ) ) ;

    mPathlineMaterial.UseMaterial() ;

    VortonSim &                 rVortonSim  = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;
    VECTOR< Vorton > &          vortons     = * rVortonSim.GetVortons() ;
    const UniformGrid< Vec3 > & velGrid     = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.GetVelocityGrid() ;

    float rangeScale ;
    SetDiagnosticPropertyValueRangeScale( rangeScale ) ;

    static const float lineWidth = 1.0f ;
    static const float pointSize = 3.0f ;

    size_t numVortons = vortons.size() ;
    for( size_t iVort = 0 ; iVort < numVortons ; ++ iVort )
    {   // For each vorton...
        const Vorton & vort = vortons[ iVort ] ;

        Vec4   color( 1.0f , 1.0f , 1.0f , 0.5f ) ;

        // Make opacity depend on distance to gridpoint.
        //color.w = 0.25f * Min2( vort.GetRadius() / distToNearestGridPoint , 1.0f ) ;

        // Modulate opacity based on distance to camera target.
        float emphasis = CameraFocusEmphasis( vort.mPosition ) ;

        if( emphasis > 0.01f )
        {   // Diagnostic information is opaque enough to render.

            if( ( mGridDecorations == GRID_DECO_CELLS ) || ( mGridDecorations == GRID_DECO_CELLS_AND_POINTS ) )
            {   // Grid cell rendering is enabled.
                if( velGrid.Encompasses( vort.mPosition ) && ( VORTON_PROPERTY_POSITION == mVortonProperty ) )
                {   // Vorton is still inside grid.
                    // Note that since this gets rendered after vorton advected, the
                    // vorton is no longer necessarily near the gridpoint it was nearest
                    // pre-advect. That means some of this diagnostic information is
                    // suspect and has a good chance of being wrong fairly often.

                    unsigned    nearestGridPointIndices[ 4 ]    ; // indices of grid cell containing vorton.
                    velGrid.IndicesOfNearestGridPoint( nearestGridPointIndices , vort.mPosition ) ; // Get indices of grid cell containing vorton.
                    const Vec3  nearestGridPointPosition        = velGrid.PositionFromIndices( nearestGridPointIndices ) ;
                    // Draw a line from the vorton to the nearest gridpoint.
                    glLineWidth( lineWidth ) ;
                    glBegin( GL_LINES ) ;
                    glColor4f( 1.0f , 0.0f , 1.0f , emphasis * color.w ) ; // Magenta
                    glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition           ) ) ;
                    glVertex3fv( reinterpret_cast< const GLfloat * >( & nearestGridPointPosition ) ) ;
                    glEnd() ;
                    glPointSize( pointSize ) ;
                    glBegin( GL_POINTS ) ;
                    glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                    glEnd() ;
                }
            }

            if( mVortonProperty == VORTON_PROPERTY_VELOCITY )
            {
                const float val0to1 = vort.mVelocity.Magnitude() * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                glColor4f( color.x , color.y , color.z , emphasis * color.w ) ;
                // Draw a line indicating velocity direction.
                Vec3 velDirEnd = vort.mPosition + vort.mVelocity * vort.GetRadius() * rangeScale ;
                glLineWidth( lineWidth ) ;
                glBegin( GL_LINES ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & velDirEnd       ) ) ;
                glEnd() ;
                glPointSize( pointSize ) ;
                glBegin( GL_POINTS ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glEnd() ;
            }

            if( VORTON_PROPERTY_VORTICITY == mVortonProperty )
            {
                const float val0to1 = vort.GetVorticity().Magnitude() * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                glColor4f( color.x , color.y , color.z , emphasis * color.w ) ;
                // Draw a line indicating vorticity.
                Vec3 vortDirEnd = vort.mPosition + vort.GetVorticity() * vort.GetRadius() * rangeScale ;
                glLineWidth( lineWidth ) ;
                glBegin( GL_LINES ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vortDirEnd      ) ) ;
                glEnd() ;
                glPointSize( pointSize ) ;
                glBegin( GL_POINTS ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glEnd() ;
            }

            if( mVortonProperty == VORTON_PROPERTY_DENSITY )
            {
                const float val0to1 = vort.GetDensity() * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                glColor4f( color.x , color.y , color.z , emphasis * color.w ) ;

                // Draw a point indicating vorton position.
                glPointSize( pointSize ) ;
                glBegin( GL_POINTS ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glEnd() ;
            }

            if( ( mVortonProperty == VORTON_PROPERTY_DENSITY_SPH ) && ! rVortonSim.GetFluidDensitiesAtPcls().Empty() )
            {
                const float & densitySph = rVortonSim.GetFluidDensitiesAtPcls()[ iVort ].mMassDensity ;
                const float val0to1 = densitySph * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                glColor4f( color.x , color.y , color.z , emphasis * color.w ) ;

                // Draw a point indicating vorton position.
                glPointSize( pointSize ) ;
                glBegin( GL_POINTS ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glEnd() ;
            }

#if COMPUTE_DENSITY_AND_GRADIENT_AT_AND_WITH_VORTONS_USING_SPH || USE_SMOOTHED_PARTICLE_HYDRODYNAMICS
            if( ( VORTON_PROPERTY_DENSITY_GRADIENT == mVortonProperty ) && ! rVortonSim.GetDensityGradientsAtPcls().Empty()  )
            {
                const Vec3 & densGrad = rVortonSim.GetDensityGradientsAtPcls()[ iVort ] ;
                const float val0to1 = densGrad.Magnitude() * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                glColor4f( color.x , color.y , color.z , emphasis * color.w ) ;
                // Draw a line indicating density gradient.
                Vec3 densGradDirEnd = vort.mPosition + densGrad * vort.GetRadius() * rangeScale ;
                glLineWidth( lineWidth ) ;
                glBegin( GL_LINES ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & densGradDirEnd  ) ) ;
                glEnd() ;
                glPointSize( pointSize ) ;
                glBegin( GL_POINTS ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glEnd() ;
            }
#endif

            if( ( mVortonProperty == VORTON_PROPERTY_PROXIMITY ) && ! rVortonSim.GetProximities().Empty() )
            {
                const float & proximity = rVortonSim.GetProximities()[ iVort ] ;
                const float val0to1     = proximity * rangeScale ;
                color = GetColorFromRamp( sVortonColorRamp , val0to1 ) ;
                glColor4f( color.x , color.y , color.z , emphasis * color.w ) ;

                // Draw a point indicating vorton position.
                glPointSize( pointSize ) ;
                glBegin( GL_POINTS ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glEnd() ;
            }

#if POISON_DENSITY_GRADIENT_BASED_ON_VORTONS_HITTING_WALLS
            if( vort.mHitBoundary )
            {   // Vorton hit boundary. Draw a line indicating hit state.
                glPushAttrib( GL_ENABLE_BIT | GL_LINE_BIT ) ;
                glEnable( GL_LINE_SMOOTH ) ;
                //glLineWidth( 3.0f ) ;
                glBegin( GL_LINES ) ;
                glColor4f( 0.0f , 0.0f , 0.0f , emphasis * color.w ) ;
                Vec3 velHitEnd = vort.mPosition + Vec3( 1.0f , 0.0f , 0.0f ) * vort.GetRadius() * 0.5f;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & velHitEnd       ) ) ;
                velHitEnd = vort.mPosition + Vec3( 0.0f , 1.0f , 0.0f ) * vort.GetRadius() * 0.5f;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & velHitEnd       ) ) ;
                velHitEnd = vort.mPosition + Vec3( 0.0f , 0.0f , 1.0f ) * vort.GetRadius() * 0.5f;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & vort.mPosition  ) ) ;
                glVertex3fv( reinterpret_cast< const GLfloat * >( & velHitEnd       ) ) ;
                glEnd() ;
                glPopAttrib() ;
            }
#endif
        }
    }
}




/** Render grid to visualize values.
*/
void InteSiVis::QdRenderDiagnosticGrid()
{
    PERF_BLOCK( InteSiVis__QdRenderDiagnosticGrid ) ;

    VortonSim & vortonSim = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;

    QdLight::DisableLights() ;

    if( mGridDecorations != GRID_DECO_NONE )
    {   // Grid decorations are enabled.

        const bool bRenderDiagnosticText = DIAG_TEXT_FULL  == mDiagnosticText  ;
        // Render bounding box -- intentionally before all opaque objects, with depth write disabled, so grid is behind everything else.
        if( GRID_FIELD_VELOCITY == mGridField )
        {
            DrawGrid_Vectors( vortonSim.GetVelocityGrid() , mBoundingBoxMaterial , mGridDecorations , bRenderDiagnosticText ) ;
        }
        else if( GRID_FIELD_DENSITY_GRADIENT == mGridField )
        {
            DrawGrid_Vectors( vortonSim.GetDensityGradientGrid() , mBoundingBoxMaterial , mGridDecorations , bRenderDiagnosticText ) ;
        }
#if COMPUTE_PRESSURE_GRADIENT
        else if( GRID_FIELD_PRESSURE_GRADIENT == mGridField )
        {
            DrawGrid_Vectors( vortonSim.GetPressureGradientGrid() , mBoundingBoxMaterial , mGridDecorations , bRenderDiagnosticText ) ;
        }
#endif
        else if( GRID_FIELD_NEGATIVE_VORTICITY == mGridField )
        {
            const NestedGrid< Vec3 > & negVorticityMultiGrid = vortonSim.GetNegativeVorticityMultiGrid() ;
            if( ! negVorticityMultiGrid.Empty() )
            {
                sNestedGridLayerToRender = Clamp( sNestedGridLayerToRender , size_t( 0 ) , negVorticityMultiGrid.GetDepth() - 1 ) ;
                const UniformGrid< Vec3 > & negVorticityGrid = negVorticityMultiGrid[ sNestedGridLayerToRender ] ;
                DrawGrid_Vectors( negVorticityGrid , mBoundingBoxMaterial , mGridDecorations , bRenderDiagnosticText ) ;
            }
        }
        else if( GRID_FIELD_VECTOR_POTENTIAL == mGridField )
        {
            NestedGrid< Vec3 > & vecPotMultiGrid = vortonSim.GetVectorPotentialMultiGrid() ;
            if( ! vecPotMultiGrid.Empty() )
            {
                sNestedGridLayerToRender = Clamp( sNestedGridLayerToRender , size_t( 0 ) , vecPotMultiGrid.GetDepth() - 1 ) ;
                const UniformGrid< Vec3 > & vecPotGrid = vecPotMultiGrid[ sNestedGridLayerToRender ] ;
                if( ( sNestedGridLayerReSample < 0 ) && ( sNestedGridLayerToRender > 0 ) )
                {   // Downsample nested grid.  Populate this layer (sNestedGridLayerToRender) with downsampled data from finer layer (sNestedGridLayerToRender-1)
                    vecPotMultiGrid.DownSampleInto( unsigned( sNestedGridLayerToRender ) ) ;
                }
                else if( ( sNestedGridLayerReSample > 0 ) && ( sNestedGridLayerToRender < vecPotMultiGrid.GetDepth() - 2 ) )
                {   // Upsample nested grid.  Populate this layer (sNestedGridLayerToRender ) with upsampled data from coarser layer (sNestedGridLayerToRender+1)
                    vecPotMultiGrid.UpSampleFrom( unsigned( sNestedGridLayerToRender + 1 ) ) ;
                }
                DrawGrid_Vectors( vecPotGrid , mBoundingBoxMaterial , mGridDecorations , bRenderDiagnosticText ) ;
            }
        }
        else if( GRID_FIELD_DENSITY == mGridField )
        {
            DrawGrid_Scalars( vortonSim.GetDensityGrid() , mBoundingBoxMaterial , mGridDecorations , bRenderDiagnosticText ) ;
        }
        else if( GRID_FIELD_SIGNED_DISTANCE == mGridField )
        {
            DrawGrid_Scalars( vortonSim.GetSignedDistanceGrid() , mBoundingBoxMaterial , mGridDecorations , bRenderDiagnosticText ) ;
        }
    }
}




static void RenderDiagnosticIntegralsText( float yPos , const char * name , const VortonSim::Integrals & integralsBefore , const VortonSim::Integrals & integralsAfter )
{
    const float delta = integralsBefore.ComputeMaxRelativeDifference( integralsAfter ) ;
    oglRenderString( Vec3( 10.0f , yPos , 0.0f ) , GLUT_BITMAP_8_BY_13 , "%s %6.1f I0={% 8.1f,% 8.1f,% 8.1f} I1={% 8.1f,% 8.1f,% 8.1f},{% 8.1f,% 8.1f,% 8.1f} I2={% 8.1f,% 8.1f,% 8.1f}"
        , name
        , delta
        , integralsAfter.mTotalCirculation.x            , integralsAfter.mTotalCirculation.y            , integralsAfter.mTotalCirculation.z
        , integralsAfter.mLinearImpulseFromVorticity.x  , integralsAfter.mLinearImpulseFromVorticity.y  , integralsAfter.mLinearImpulseFromVorticity.z
        , integralsAfter.mLinearImpulseFromVelocity.x   , integralsAfter.mLinearImpulseFromVelocity.y   , integralsAfter.mLinearImpulseFromVelocity.z
        , integralsAfter.mAngularImpulse.x              , integralsAfter.mAngularImpulse.y              , integralsAfter.mAngularImpulse.z              ) ;
}




void InteSiVis::SetTallyDiagnosticIntegrals()
{
    switch( mDiagnosticText )
    {
        case DIAG_TEXT_NONE:
        case DIAG_TEXT_TIMING:
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetTallyDiagnosticIntegrals( false ) ;
            break ;
        case DIAG_TEXT_SUMMARY:
        case DIAG_TEXT_FULL:
            mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim.SetTallyDiagnosticIntegrals( true ) ;
            break ;
    }
}




/** Render summary diagnostic text.
*/
void InteSiVis::QdRenderSummaryDiagnosticText()
{
    PERF_BLOCK( InteSiVis__QdRenderSummaryDiagnosticText ) ;

    VortonSim & vortonSim = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;

    if( mDiagnosticText != DIAG_TEXT_NONE )
    {   // Diagnostic text is enabled.

        // Render frame counter as text on screen.
        ASSERT( vortonSim.GetTallyDiagnosticIntegrals() ) ;
        oglRenderString( Vec3( 10.0f , 10.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "frame=% 5i  time=% 6.04f  step=%6.04f [%6.04f,%6.04f]  dur=%6.04f   fps=%3.0f" , mFrame , mTimeNow , mTimeStep , mTimeStepMin , mTimeStepMax , mFrameDurSecAvg , 1.0f / mFrameDurSecAvg ) ;
    }
    else
    {
        ASSERT( ! vortonSim.GetTallyDiagnosticIntegrals() ) ;
    }

    if( ( DIAG_TEXT_SUMMARY == mDiagnosticText ) || ( DIAG_TEXT_FULL == mDiagnosticText ) )
    {
        // Render bounding box info.
        const Vec3 & bboxCenter = vortonSim.GetBoundingBoxCenter() ;
        const Vec3 & bboxSize   = vortonSim.GetBoundingBoxSize() ;
        oglRenderString( Vec3( 10.0f , 20.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "bbox@{ %g , %g , %g } spans { %g , %g , %g }" , bboxCenter.x , bboxCenter.y , bboxCenter.z , bboxSize.x , bboxSize.y , bboxSize.z ) ;
        const Vec3 & velGridSize    = vortonSim.GetVelocityGrid().GetExtent() ;
        const Vec3 & velGridSpacing = vortonSim.GetVelocityGrid().GetCellSpacing() ;
        oglRenderString( Vec3( 10.0f , 30.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "velGrid spans { %g , %g , %g } spacing { %g , %g , %g }" , velGridSize.x , velGridSize.y , velGridSize.z , velGridSpacing.x , velGridSpacing.y , velGridSpacing.z ) ;
        // Render vorticity integrals.
        {
            const VortonSim::DiagnosticIntegrals & di = vortonSim.GetDiagnosticIntegrals() ;
            oglRenderString( Vec3( 10.0f , 40.0f , 0.0f ) , GLUT_BITMAP_8_BY_13 , "init        I0={% 8.1f,% 8.1f,% 8.1f} I1={% 8.1f,% 8.1f,% 8.1f},{% 8.1f,% 8.1f,% 8.1f} I2={% 8.1f,% 8.1f,% 8.1f}"
                , di.mInitial.mTotalCirculation.x                   , di.mInitial.mTotalCirculation.y                   , di.mInitial.mTotalCirculation.z
                , di.mInitial.mLinearImpulseFromVorticity.x         , di.mInitial.mLinearImpulseFromVorticity.y         , di.mInitial.mLinearImpulseFromVorticity.z
                , di.mInitial.mLinearImpulseFromVelocity.x          , di.mInitial.mLinearImpulseFromVelocity.y          , di.mInitial.mLinearImpulseFromVelocity.z
                , di.mInitial.mAngularImpulse.x                     , di.mInitial.mAngularImpulse.y                     , di.mInitial.mAngularImpulse.z              ) ;
            RenderDiagnosticIntegralsText(  50.0f , "befr" , di.mInitial         , di.mBefore           ) ;
            RenderDiagnosticIntegralsText(  60.0f , "advc" , di.mBefore          , di.mAfterAdvect      ) ;
            RenderDiagnosticIntegralsText(  70.0f , "rgrd" , di.mAfterAdvect     , di.mAfterRegrid      ) ;
            RenderDiagnosticIntegralsText(  80.0f , "velg" , di.mAfterRegrid     , di.mAfterVelGrid     ) ;
            RenderDiagnosticIntegralsText(  90.0f , "stre" , di.mAfterVelGrid    , di.mAfterStretch     ) ;
            RenderDiagnosticIntegralsText( 100.0f , "baro" , di.mAfterStretch    , di.mAfterBaroclinic  ) ;
            RenderDiagnosticIntegralsText( 110.0f , "difs" , di.mAfterBaroclinic , di.mAfterDiffuse     ) ;
            RenderDiagnosticIntegralsText( 120.0f , "heat" , di.mAfterDiffuse    , di.mAfterHeat        ) ;
        }
        // Render vorticity statistics.
        {
            float vorticityMin , vorticityMax , vorticityMean , vorticityStdDev ;
            Vec3 centerOfVorticity ;

            vortonSim.GatherVorticityStats( vorticityMin , vorticityMax , vorticityMean , vorticityStdDev , centerOfVorticity ) ;
            oglRenderString( Vec3( 10.0f , 130.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "omega:[%.0f,%.0f]<%g>+-%-.0f #=%i CoV=%g,%g,%g"
                , vorticityMin , vorticityMax , vorticityMean , vorticityStdDev , vortonSim.GetVortons()->size()
                , centerOfVorticity.x , centerOfVorticity.y , centerOfVorticity.z
                ) ;
            const VortonSim::VorticityTermsStatistics & vortTermStats = vortonSim.GetVorticityTermsStatistics() ;

            VortonSim::InvestigationTermE & vorticityInvestigationTerm = vortonSim.GetInvestigationTerm() ;
            oglRenderString( Vec3( 10.0f , 140.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "%c stretchTilt     :[%.7f,%.7f]<%g>+-%-.7f"
                , vorticityInvestigationTerm == VortonSim::INVESTIGATE_ALL || vorticityInvestigationTerm == VortonSim::INVESTIGATE_STRETCHING_TILTING ? '*' : ' '
                , vortTermStats.mStretchTilt.mMin , vortTermStats.mStretchTilt.mMax , vortTermStats.mStretchTilt.mMean , vortTermStats.mStretchTilt.mStdDev ) ;
            oglRenderString( Vec3( 10.0f , 150.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "%c baroclinic      :[%.7f,%.7f]<%g>+-%-.7f"
                , vorticityInvestigationTerm == VortonSim::INVESTIGATE_ALL || vorticityInvestigationTerm == VortonSim::INVESTIGATE_BAROCLINIC ? '*' : ' '
                , vortTermStats.mBaroclinic.mMin , vortTermStats.mBaroclinic.mMax , vortTermStats.mBaroclinic.mMean , vortTermStats.mBaroclinic.mStdDev ) ;
            oglRenderString( Vec3( 10.0f , 160.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "%c viscousDiffusion:[%.7f,%.7f]<%g>+-%-.7f"
                , vorticityInvestigationTerm == VortonSim::INVESTIGATE_ALL || vorticityInvestigationTerm == VortonSim::INVESTIGATE_VISCOUS_DIFFUSION ? '*' : ' '
                , vortTermStats.mViscousDiffusion.mMin , vortTermStats.mViscousDiffusion.mMax , vortTermStats.mViscousDiffusion.mMean , vortTermStats.mViscousDiffusion.mStdDev ) ;
        }
        // Render vorton velocity statistics.
        {
            float velocityMin , velocityMax , velocityMean , velocityStdDev ;
            Vec3 centerOfVelocity ;
            Particles::ComputeVelocityStats( * reinterpret_cast< VECTOR< Particle > * >( vortonSim.GetVortons() ) , velocityMin , velocityMax , velocityMean , velocityStdDev , centerOfVelocity ) ;

            oglRenderString( Vec3( 10.0f , 170.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "vel:[%.3f,%.3f]<%.3f>+-%-.3f"
                , velocityMin , velocityMax , velocityMean , velocityStdDev
                ) ;
        }
        // Render vorton fluid temperature statistics.
        {
            float temperatureMin , temperatureMax , temperatureMean , temperatureStdDev ;
            vortonSim.GatherTemperatureStats( temperatureMin , temperatureMax , temperatureMean , temperatureStdDev ) ;

            oglRenderString( Vec3( 10.0f , 180.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "Temp:[%.0f,%.0f]<%.0f>+-%-.0f"
                , temperatureMin , temperatureMax , temperatureMean , temperatureStdDev
                ) ;
        }
        // Render vorton fluid density statistics.
        {
            float densityMin , densityMax , densityMean , densityStdDev ;
            vortonSim.GatherDensityStats( densityMin , densityMax , densityMean , densityStdDev ) ;

            float densitySphMin , densitySphMax , densitySphMean , densitySphStdDev ;
            vortonSim.GatherDensitySphStats( densitySphMin , densitySphMax , densitySphMean , densitySphStdDev ) ;

            float numDensSphMin , numDensSphMax , numDensSphMean , numDensSphStdDev ;
            vortonSim.GatherNumberDensitySphStats( numDensSphMin , numDensSphMax , numDensSphMean , numDensSphStdDev ) ;

            float densGradSphMin , densGradSphMax , densGradSphMean , densGradSphStdDev ;
            vortonSim.GatherDensityGradientStats( densGradSphMin , densGradSphMax , densGradSphMean , densGradSphStdDev ) ;

            oglRenderString( Vec3( 10.0f , 190.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "dens:[%.3f,%.3f]<%.3f>+-%-.3f    densSph:[%.3f,%.3f]<%.3f>+-%-.3f    #densSph:[%.3f,%.3f]<%.3f>+-%-.3f    densGradSph:[%.3f,%.3f]<%.3f>+-%-.3f"
                , densityMin , densityMax , densityMean , densityStdDev
                , densitySphMin , densitySphMax , densitySphMean , densitySphStdDev
                , numDensSphMin , numDensSphMax , numDensSphMean , numDensSphStdDev
                , densGradSphMin , densGradSphMax , densGradSphMean , densGradSphStdDev
                ) ;
        }
        // Render fluid signed distance function statistics.
        {
            float sdfMin , sdfMax , sdfMean , sdfStdDev ;
            vortonSim.GatherSignedDistanceStats( sdfMin , sdfMax , sdfMean , sdfStdDev ) ;

            oglRenderString( Vec3( 10.0f , 200.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "sdf:[%.3f,%.3f]<%.3f>+-%-.3f"
                , sdfMin , sdfMax , sdfMean , sdfStdDev
                ) ;
        }
        // Render vorton fluid proximity-to-walls statistics.
        {
            float proximityMin , proximityMax , proximityMean , proximityStdDev ;
            vortonSim.GatherProximityStats( proximityMin , proximityMax , proximityMean , proximityStdDev ) ;

            oglRenderString( Vec3( 10.0f , 210.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "proximity:[%.3f,%.3f]<%.3f>+-%-.3f"
                , proximityMin , proximityMax , proximityMean , proximityStdDev
                ) ;
        }
        {
            extern float gVortonSim_DisplacementMax ;
            extern int gVortonSim_NumRelaxationIters ;
            oglRenderString( Vec3( 10.0f , 220.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "diplacementMax=%g    numIters=%i"
                , gVortonSim_DisplacementMax
                , gVortonSim_NumRelaxationIters ) ;
        }
        // Render tracer count and other info.
        {
            oglRenderString( Vec3( 10.0f , 230.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "tracers: #=%i    layer=%i  resample=%i"
                , mTracerPclGrpInfo.mParticleGroup->GetParticles().Size()
                , sNestedGridLayerToRender
                , sNestedGridLayerReSample
                ) ;
        }
        // Render diagnostic state.
        oglRenderString( Vec3( 10.0f , 240.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "%s    %s    %s    %s    %s    %s"
            , GridDecorationString() , GridFieldString()
            , mEmphasizeCameraTarget ? "emphasis" : "NO_emp"
            , mFluidScene.GetVortonRenderingString()
            , VortonPropertyString()
            , mFluidScene.GetTracerRenderingString() ) ;

        // Render fluid simulation technique and residual info if appropriate
        {
#if   VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT
            static const char sVelTeq[] = "VELOCITY_TECHNIQUE_DIRECT" ;
#elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE
            static const char sVelTeq[] = "VELOCITY_TECHNIQUE_TREE" ;
#elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL
#   if   VECTOR_POTENTIAL_TECHNIQUE == VECTOR_POTENTIAL_TECHNIQUE_DIRECT
#       define VEC_POT_TEQ "VECTOR_POTENTIAL_TECHNIQUE_DIRECT"
#   elif VECTOR_POTENTIAL_TECHNIQUE == VECTOR_POTENTIAL_TECHNIQUE_TREE
#       define VEC_POT_TEQ "VECTOR_POTENTIAL_TECHNIQUE_TREE"
#   endif
#   if VORTON_SIM_USE_MULTI_GRID
#       define WITH_MULTI_GRID " with VORTON_SIM_USE_MULTI_GRID"
#   else
#       define WITH_MULTI_GRID ""
#   endif
            static const char sVelTeq[] = "VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL " VEC_POT_TEQ WITH_MULTI_GRID ;

#elif VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_MONOPOLES // No longer supported
            static const char sVelTeq[] = "VELOCITY_TECHNIQUE_MONOPOLES" ;
#else
#           error "VELOCITY_TECHNIQUE invalid or not supported in this piece of code."
#endif

            // As of 2016 May these stats are neither thread-safe nor deterministic when run with TBB.  Currently only meaningful for ProfileWithoutTbb build.
            const Stats_Float & residuStats       = vortonSim.GetPoissonResidualStats() ;
            const Stats_Float & residuStats_xTime = vortonSim.GetPoissonResidualStats_AcrossTime() ;
            oglRenderString( Vec3( 10.0f , 250.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "simTeq=%s    velTeq=%s    poissonResidual=[%g,%g] %g+-%g    residualAcrossTime=[%g,%g] %g+-%g"
                , FluidSimulationTechniqueString()
                , sVelTeq
                , residuStats.mMin , residuStats.mMax , residuStats.mMean , residuStats.mStdDev
                , residuStats_xTime.mMin , residuStats_xTime.mMax , residuStats_xTime.mMean , residuStats_xTime.mStdDev
                ) ;
        }

        // Render rigid body info.
        for( size_t iSphere = 0 ; iSphere < GetSpheres().size() ; ++ iSphere )
        {   // For each sphere...
            // Display its temperature, linear velocity and angular velocity.
            const float yPos = 260.0f + float( 10 * iSphere ) ;
            oglRenderString( Vec3( 10.0f , yPos , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "ball[%i].T=%g  v={%g,%g,%g} w={%g,%g,%g}"
                , iSphere
                , GetSpheres()[ iSphere ].GetThermalProperties().GetTemperature()
                , GetSpheres()[ iSphere ].GetBody()->GetVelocity().x
                , GetSpheres()[ iSphere ].GetBody()->GetVelocity().y
                , GetSpheres()[ iSphere ].GetBody()->GetVelocity().z
                , GetSpheres()[ iSphere ].GetBody()->GetAngularVelocity().x
                , GetSpheres()[ iSphere ].GetBody()->GetAngularVelocity().y
                , GetSpheres()[ iSphere ].GetBody()->GetAngularVelocity().z
                ) ;
        }
        for( size_t iBox = 0 ; iBox < GetBoxes().size() ; ++ iBox )
        {   // For each box...
            // Display its temperature, linear velocity and angular velocity.
            const float yPos = 270.0f + float( 10 * ( iBox + GetSpheres().size() ) ) ;
            oglRenderString( Vec3( 10.0f , yPos , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "box[%i].T=%g v={%g,%g,%g} w={%g,%g,%g}"
                , iBox
                , GetBoxes()[ iBox ].GetThermalProperties().GetTemperature()
                , GetBoxes()[ iBox ].GetBody()->GetVelocity().x
                , GetBoxes()[ iBox ].GetBody()->GetVelocity().y
                , GetBoxes()[ iBox ].GetBody()->GetVelocity().z
                , GetBoxes()[ iBox ].GetBody()->GetAngularVelocity().x
                , GetBoxes()[ iBox ].GetBody()->GetAngularVelocity().y
                , GetBoxes()[ iBox ].GetBody()->GetAngularVelocity().z
                ) ;
        }


    #if ENABLE_PARTICLE_JERK_RECORD
        // Render jerk statistics.
        {
            static FILE * jerkFile = 0 ;
            if( 0 == jerkFile )
            {   // Jerk output file is not yet open.
                char filename[ 256 ] = "TestData/jerk" ;
            #if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT )
                strcat( filename , "-DIRECT" ) ;
            #elif ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE )
                strcat( filename , "-TREE" ) ;
            #elif ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL )
                strcat( filename , "-POISSON" ) ;
            #endif
            #if COMPUTE_VELOCITY_AT_VORTONS
                strcat( filename , "-VAV" ) ;
            #endif
            #if USE_PARTICLE_IN_CELL
                strcat( filename , "-PIC" ) ;
            #endif
            #if USE_ORIGINAL_VORTONS_IN_BASE_LAYER
                strcat( filename , "-OVIBL" ) ;
            #endif
            #if ENABLE_AUTO_MOLLIFICATION
                strcat( filename , "-AM" ) ;
            #endif
                strcat( filename , ".dat" ) ;

                jerkFile = fopen( filename , "w" ) ;

                fprintf( jerkFile , "# " ) ;
            #if ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_DIRECT )
                fprintf( jerkFile , "DIRECT " ) ;
            #elif ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_TREE )
                fprintf( jerkFile , "TREE " ) ;
            #elif ( VELOCITY_TECHNIQUE == VELOCITY_TECHNIQUE_POISSON_GAUSS_SEIDEL )
                fprintf( jerkFile , "POISSON " ) ;
            #endif
            #if COMPUTE_VELOCITY_AT_VORTONS
                fprintf( jerkFile , "COMPUTE_VELOCITY_AT_VORTONS " ) ;
            #endif
            #if USE_PARTICLE_IN_CELL
                fprintf( jerkFile , "USE_PARTICLE_IN_CELL " ) ;
            #endif
            #if USE_ORIGINAL_VORTONS_IN_BASE_LAYER
                fprintf( jerkFile , "USE_ORIGINAL_VORTONS_IN_BASE_LAYER " ) ;
            #endif
            #if ENABLE_AUTO_MOLLIFICATION
                fprintf( jerkFile , "ENABLE_AUTO_MOLLIFICATION " ) ;
            #endif
                fprintf( jerkFile , "\n" ) ;
                fflush( jerkFile ) ;
            }
            float jerk2Min , jerk2Max , jerk2Mean , jerk2StdDev ;
            VECTOR< Particle > & vortonsAsParticles = reinterpret_cast< VECTOR< Particle > & >( * vortonSim.GetVortons() ) ;
            Particles::ComputeJerkStatistics( vortonsAsParticles , jerk2Mean , jerk2StdDev , jerk2Min , jerk2Max ) ;
            if( IsNan( jerk2Mean ) || IsInf( jerk2Mean ) || IsNan( jerk2StdDev ) || IsInf( jerk2StdDev ) )
            {   // Not enough updates have occurred yet to compute jerk.  It takes 4 consecutive timesteps with TallyDiagnosticIntegrals enabled.
                oglRenderString( Vec3( 200.0f , 110.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "jerk: (insufficient data)" ) ;
            }
            else
            {   // Enough updates have occurred to compute jerk.
                oglRenderString( Vec3( 200.0f , 110.0f , 0.0f ) , GLUT_BITMAP_HELVETICA_10 , "jerk:[%.0f,%.0f]<%.0f>+-%-.0f" , sqrtf( jerk2Min ) , sqrtf( jerk2Max ) , sqrtf( jerk2Mean ) , sqrtf( jerk2StdDev ) ) ;
                fprintf( jerkFile , "%g %g %g %g %g\n" , mTimeNow , sqrtf( jerk2Mean ) , sqrtf( jerk2StdDev ) , sqrtf( jerk2Min ) , sqrtf( jerk2Max ) ) ;
                fflush( jerkFile ) ;
            }
        }
    #endif
    }
}




const char * InteSiVis::FluidSimulationTechniqueString() const
{
    VortonSim & vortonSim = mVortonPclGrpInfo.mPclOpVortonSim->mVortonSim ;

    switch( vortonSim.GetFluidSimulationTechnique() )
    {
        CASE_STRING_FROM_TOKEN( VortonSim::FLUID_SIM_VORTEX_PARTICLE_METHOD             ) ;
        CASE_STRING_FROM_TOKEN( VortonSim::FLUID_SIM_SMOOTHED_PARTICLE_HYDRODYNAMICS    ) ;
        CASE_STRING_FROM_TOKEN( VortonSim::FLUID_SIM_VPM_SPH_HYBRID                     ) ;
        CASE_STRING_FROM_TOKEN( VortonSim::FLUID_SIM_NUM                                ) ;
    }
    return "(invalid FluidSimulationTechniqueE)" ;
}




const char * InteSiVis::GridDecorationString() const
{
    switch( mGridDecorations )
    {
        CASE_STRING_FROM_TOKEN( GRID_DECO_NONE              ) ;
        CASE_STRING_FROM_TOKEN( GRID_DECO_BOUNDING_BOX      ) ;
        CASE_STRING_FROM_TOKEN( GRID_DECO_CELLS             ) ;
        CASE_STRING_FROM_TOKEN( GRID_DECO_POINTS            ) ;
        CASE_STRING_FROM_TOKEN( GRID_DECO_CELLS_AND_POINTS  ) ;
    }
    return "(invalid GridDecorationE)" ;
}




const char * InteSiVis::GridFieldString() const
{
    switch( mGridField )
    {
        CASE_STRING_FROM_TOKEN( GRID_FIELD_VELOCITY          ) ;
        CASE_STRING_FROM_TOKEN( GRID_FIELD_DENSITY_GRADIENT  ) ;
        CASE_STRING_FROM_TOKEN( GRID_FIELD_PRESSURE_GRADIENT ) ;
        CASE_STRING_FROM_TOKEN( GRID_FIELD_NEGATIVE_VORTICITY) ;
        CASE_STRING_FROM_TOKEN( GRID_FIELD_VECTOR_POTENTIAL  ) ;
        CASE_STRING_FROM_TOKEN( GRID_FIELD_DENSITY           ) ;
        CASE_STRING_FROM_TOKEN( GRID_FIELD_SIGNED_DISTANCE   ) ;
    }
    return "(invalid GridFieldE)" ;
}




const char * InteSiVis::VortonPropertyString() const
{
    switch( mVortonProperty )
    {
        CASE_STRING_FROM_TOKEN( VORTON_PROPERTY_POSITION            ) ;
        CASE_STRING_FROM_TOKEN( VORTON_PROPERTY_VELOCITY            ) ;
        CASE_STRING_FROM_TOKEN( VORTON_PROPERTY_VORTICITY           ) ;
        CASE_STRING_FROM_TOKEN( VORTON_PROPERTY_DENSITY             ) ;
        CASE_STRING_FROM_TOKEN( VORTON_PROPERTY_DENSITY_SPH         ) ;
        CASE_STRING_FROM_TOKEN( VORTON_PROPERTY_DENSITY_GRADIENT    ) ;
        CASE_STRING_FROM_TOKEN( VORTON_PROPERTY_PROXIMITY           ) ;
    }
    return "(invalid VortonPropertyE)" ;
}
