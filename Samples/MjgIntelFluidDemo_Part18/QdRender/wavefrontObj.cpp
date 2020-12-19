/** \file wavefrontObj.cpp

    \brief Utility routines for reading Wavefront Object files

    \author Copyright 2008-2014 MJG; All rights reserved.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <sys/stat.h>
#include <errno.h>

#include "Core/Memory/allocator.h"
#include "Core/File/debugPrint.h"
#include "Core/File/fileWrapper.h"
#include "Core/Utility/scan.h"
#include "WavefrontObj.h"

// Types --------------------------------------------------------------
// Private variables --------------------------------------------------------------
// Public variables --------------------------------------------------------------
// Private functions --------------------------------------------------------------

void WavefrontObjFile::ParseLine( const char * buffer , size_t bufferSize )
{
    ASSERT( buffer != 0 ) ;
    size_t offset = 0 ;
    scanWhiteSpace( buffer , & offset ) ;
    if( scanLiteral( buffer , & offset , "#" ) )
    {   // This line is a comment.  Ignore it.
        return ;
    }
    else if( '\0' == buffer[ offset ] )
    {   // Empty line.  Ignore it.
    }
    // ---- Vertex data ----
    else if( scanLiteral( buffer , & offset , "vn" ) )
    {   // This line contains a vertex normal.
        if( mInIncludedGroup )
        {   // An included group contains this data so read and store it.
            mVertexNormals.PushBack( Vec4(0,0,0,0) ) ;
            Vec4 & vNorm = mVertexNormals.Back() ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vNorm.x ) ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vNorm.y ) ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vNorm.z ) ;

            //ASSERT( vNorm.IsNormalizedv3() ) ;
            if( ( vNorm.x == 0.0f ) && ( vNorm.y == 0.0f ) && ( vNorm.z == 0.0f ) )
            {
                vNorm.x = 1.0f ;
            }
            else
            {
                vNorm.Normalizev3() ;
            }
        }
    }
    else if( scanLiteral( buffer , & offset , "vt" ) )
    {   // This line contains a vertex texture coordinate.
        if( mInIncludedGroup )
        {   // An included group contains this data so read and store it.
            mVertexTexCoords.PushBack( Vec4(0,0,0,0) ) ;
            Vec4 & vTexCoord = mVertexTexCoords.Back() ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vTexCoord.x ) ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vTexCoord.y ) ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vTexCoord.z ) ;
        }
    }
    else if( scanLiteral( buffer , & offset , "v" ) )   // NOTE: scanLiteral is fragile and requires scanning "v" after "vt" and "vn".  See comments in scanLiteral.
    {   // This line contains a vertex position.
        if( mInIncludedGroup )
        {   // An included group contains this data so read and store it.
            mVertexPositions.PushBack( Vec4(0,0,0,1) ) ;
            Vec4 & vPos = mVertexPositions.Back() ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vPos.x ) ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vPos.y ) ;
            scanWhiteSpace( buffer , & offset ) ;
            scanFloatConstant( buffer , & offset , 0 , & vPos.z ) ;
        }
    }
    // ---- Elements ----
    else if( scanLiteral( buffer , & offset , "f" ) )
    {   // This line contains a face.
        if( mInIncludedGroup )
        {   // An included group contains this data so read and store it.
            // Face info is organized as follows:
            // f v0/vt0/vn0 v1/vt1/vn1 v2/vt2/vn2 ...
            // Each number above (v0, vt0, etc.) is an index into the corresponding array.
            // In general the indices can be negative, indicating a "relative" value, but this implementation does not support that.
            // In other words the ordering for each index triple is position/textureCoordinate/normal.
            PolygonFace face ;

            for( int iVert = 0 ; iVert < PolygonFace::MAX_NUM_VERTS ; ++ iVert )
            {
                ASSERT( iVert == face.GetNumVerts() ) ;
                int iPos ;
                int iTexCoord ;
                int iNormal ;

                scanWhiteSpace( buffer , & offset ) ;
                if( scanSignedIntConstant( buffer , & offset , & iPos ) )
                {
                    if( scanLiteral( buffer , & offset , "/" ) )
                    {   // This vertex refers to more than just position.  It might have texcoord, normal or both
                        if( scanSignedIntConstant( buffer , & offset , & iTexCoord ) )
                        {
                            mHasTexCoords = true ;
                            if( scanLiteral( buffer , & offset , "/" ) )
                            {
                                mHasNormals = true ;
                                scanSignedIntConstant( buffer , & offset , & iNormal ) ;
                                face.AddVertPTN( iPos , iTexCoord , iNormal ) ;
                            }
                            else
                            {
                                face.AddVertPT( iPos , iTexCoord ) ;
                            }
                        }
                        else if( scanLiteral( buffer , & offset , "/" ) )
                        {   // Vertex has position and normal (but no texcoord)
                            mHasTexCoords   = false ;
                            mHasNormals     = true ;
                            scanSignedIntConstant( buffer , & offset , & iNormal ) ;
                            face.AddVertPN( iPos , iNormal ) ;
                        }
                    }
                    else
                    {
                        face.AddVertP( iPos ) ;
                    }
                }
                else
                {
                    break ;
                }
            }

            ASSERT( face.GetNumVerts() >= 3 ) ; // A face with fewer than 3 verts is erroneously truncated.

            if( face.GetNumVerts() > 3 )
            {
                mSimpleLayout = false ;
            }

            // Check whether indices for positions, texture coordinates and normals are identical.
            // That would mean that code elsewhere can use the vertex data as-is.
            // If the vertex arrays from the Wavefront OBJ file do not have a one-to-one correspondence with each other then using that data requires more asset conditioning.  See below.
            if(     mHasNormals
                &&  (   ( face.GetPos(0) != face.GetNormal(0) )
                    ||  ( face.GetPos(1) != face.GetNormal(1) )
                    ||  ( face.GetPos(2) != face.GetNormal(2) )
                    )
                )
            {   // Index of position does not match index of normal or texture coordinates.
                // Using this data will require additional work after reading in all the face info.
                mSimpleLayout = false ;
            }
            if(     mHasTexCoords
                &&  (   ( face.GetPos(0) != face.GetTexCoord(0) )
                    ||  ( face.GetPos(1) != face.GetTexCoord(1) )
                    ||  ( face.GetPos(2) != face.GetTexCoord(2) )
                    )
                )
            {   // Index of position does not match index of normal or texture coordinates.
                // Using this data will require additional work after reading in all the face info.
                mSimpleLayout = false ;
            }

            mFaces.PushBack( face ) ;
        }
    }
    // ---- Grouping ----
    else if( scanLiteral( buffer , & offset , "g" ) )
    {   // This line contains a group name.  Read it and update state according to whether current group is one of the included groups.
        // TODO: FIXME: IMPLEMENT ME.
    }
    else if( scanLiteral( buffer , & offset , "s" ) )
    {   // This line contains a smoothing group
    }
    else if( scanLiteral( buffer , & offset , "mg" ) )
    {   // This line contains a merging group
    }
    else if( scanLiteral( buffer , & offset , "o" ) )
    {   // This line contains an object name
    }
    // ---- Render attributes ----
    else if( scanLiteral( buffer , & offset , "mtllib" ) )
    {   // This line contains the name of a material library.  Read it and store material information.
    }
    else if( scanLiteral( buffer , & offset , "usemtl" ) )
    {   // This line contains the name of a material.  Read it and store material information.
        // NOTE: We could store material information as per-vertex color.  That would not correspond perfectly with what the material library provides but it is easier to implement.
        // NOTE: To support full material specification would entail separating this model into several meshes, each with its own material.
    }
    else
    {   // This line contains an unsupported feature.  Ignore it for now.  Maybe someday we will implement support for it.
        SET_BREAKPOINT_HERE ;
    }
}




// Public functions --------------------------------------------------------------

WavefrontObjFile::WavefrontObjFile( void )
    : mVertexPositionNumComponents( 0 )
    , mVertexTexCoordNumComponents( 0 )
    , mUniqueVertices( 16 , 3 * sizeof( Vec4 ) )
    , mInIncludedGroup( true )
    , mSimpleLayout( true )
    , mHasTexCoords( false )
    , mHasNormals( false )
{
    memset( mGroups , 0 , sizeof( mGroups ) ) ;

#if defined( _DEBUG )
    {   // Unit test
        Vertex v[5] ;
        v[0].p = Vec4( 1 , 2 , 3 , 1 ) ; v[0].t = Vec4( 0.1f , 0.2f , 0 , 0 ) ; v[0].n = Vec4( 0.1f , 0.5f , 0.2f , 0 ) ; v[0].n.Normalizev3() ;
        v[1].p = Vec4( 1 , 2 , 3 , 1 ) ; v[1].t = Vec4( 0.1f , 0.2f , 0 , 0 ) ; v[1].n = Vec4( 0.1f , 0.5f , 0.2f , 0 ) ; v[1].n.Normalizev3() ;
        v[2].p = Vec4( 1 , 2 , 9 , 1 ) ; v[2].t = Vec4( 0.1f , 0.2f , 0 , 0 ) ; v[2].n = Vec4( 0.1f , 0.5f , 0.2f , 0 ) ; v[2].n.Normalizev3() ;
        v[3].p = Vec4( 1 , 2 , 3 , 1 ) ; v[3].t = Vec4( 0.1f , 0.9f , 0 , 0 ) ; v[3].n = Vec4( 0.1f , 0.5f , 0.2f , 0 ) ; v[3].n.Normalizev3() ;
        v[4].p = Vec4( 1 , 2 , 3 , 1 ) ; v[4].t = Vec4( 0.1f , 0.2f , 0 , 0 ) ; v[4].n = Vec4( 0.1f , 0.5f , 0.9f , 0 ) ; v[4].n.Normalizev3() ;

                                        ASSERT( mUniqueVertices.Size() == 0 ) ;
        InsertUniqueVertex( v[0] ) ;    ASSERT( mUniqueVertices.Size() == 1 ) ; ASSERT( mUniqueVertices.FindEntry( (char*) & v[0] )->i == 0 ) ;
        // v[0] and v[1] effectively match so the next line should NOT increase the number of unique vertices
        InsertUniqueVertex( v[1] ) ;    ASSERT( mUniqueVertices.Size() == 1 ) ; ASSERT( mUniqueVertices.FindEntry( (char*) & v[1] )->i == 0 ) ;
        InsertUniqueVertex( v[2] ) ;    ASSERT( mUniqueVertices.Size() == 2 ) ; ASSERT( mUniqueVertices.FindEntry( (char*) & v[2] )->i == 1 ) ;
        InsertUniqueVertex( v[3] ) ;    ASSERT( mUniqueVertices.Size() == 3 ) ; ASSERT( mUniqueVertices.FindEntry( (char*) & v[3] )->i == 2 ) ;
        InsertUniqueVertex( v[4] ) ;    ASSERT( mUniqueVertices.Size() == 4 ) ; ASSERT( mUniqueVertices.FindEntry( (char*) & v[4] )->i == 3 ) ;
        mUniqueVertices.Clear() ;
    }
#endif
}




/*! \brief  Compile a list of unique vertices based on face info

    To be useful to the GPU, each "vertex" in the GPU sense must
    contain position, texcoord and normal info combined.
    Call this a "fully qualified vertex" meaning that this variety
    of vertex contains all info the GPU needs to use it.
    This method iterates through all faces and for each unique fully-qualified
    "vertex" into a table for use later.

    This method also reconstructs faces using unique fully-qualified vertices.
    The Wavefront OBJ file specifies faces in a form where
    an index triple (offsets into 3 separate and simpler vertex lists)
    represents each fully-qualified vertex.
    Although compact, the GPU cannot use info in that form.
    The index buffer that the GPU uses must refer to indices
    into the fully-qualified vertex list.  This method
    fills the "simple" index list (which implicitly represents
    faces in the form of triangles) with indices that
    refer to the fully-qualified vertex list.

*/
void WavefrontObjFile::ReconstructFaces( void )
{
    ASSERT( 0 == mIndices.Size() ) ;
    ASSERT( 0 == mUniqueVertices.Size() ) ;
    for( Fiea::Vector<PolygonFace>::Iterator iter = mFaces.Begin() ; iter != mFaces.End() ; ++ iter )
    {   // For each face in the model...
        PolygonFace & face = * iter ;
        ASSERT( face.GetNumVerts() >= 3 ) ;
        ASSERT( face.GetNumVerts() <= PolygonFace::MAX_NUM_VERTS ) ;

        // 0th vertex gets reused for each triangle.
        Vertex vert0 ;
        vert0.p = mVertexPositions[ face.GetPos( 0 ) ] ;
        if( this->mHasTexCoords )
        {
            vert0.t = mVertexTexCoords[ face.GetTexCoord( 0 ) ] ;
        }
        if( this->mHasNormals )
        {
            vert0.n = mVertexNormals  [ face.GetNormal( 0 ) ] ;
        }
        Vertex & vUniq0 = InsertUniqueVertex( vert0 ) ;

        // Initialize 1st vertex outside loop, then update inside loop.
        Vertex vert1 ;
        vert1.p = mVertexPositions[ face.GetPos( 1 ) ] ;
        if( mHasTexCoords )
        {
            vert1.t = mVertexTexCoords[ face.GetTexCoord( 1 ) ] ;
        }
        if( mHasNormals )
        {
            vert1.n = mVertexNormals  [ face.GetNormal( 1 ) ] ;
        }
        Vertex * vUniq1 = & InsertUniqueVertex( vert1 ) ;

        for( int iVert2 = 2 ; iVert2 < face.GetNumVerts() ; ++ iVert2 )
        {   // Iterate through each triangle in this n-gon.
            // We want to output triangles so convert this n-gon into triangles.
            // Each triangle starts with vertex 0, then uses vertex i and i+1.
            //   3 *---* 2
            //     |  /|  
            //     | / |  
            //     |/  |  
            //   0 *---* 1
            mIndices.PushBack( vUniq0.i ) ;
            mIndices.PushBack( vUniq1->i ) ;
            vert1.p = mVertexPositions[ face.GetPos( iVert2 ) ] ;
            if( mHasTexCoords )
            {
                vert1.t = mVertexTexCoords[ face.GetTexCoord( iVert2 ) ] ;
            }
            if( mHasNormals )
            {
                vert1.n = mVertexNormals  [ face.GetNormal( iVert2 ) ] ;
            }
            vUniq1 = & InsertUniqueVertex( vert1 ) ;
            mIndices.PushBack( vUniq1->i ) ;
        }
    }
}




/*! \brief  Fill individual vertex containers

    The underlying routines that use this module
    use a structure of arrays, i.e. it requires that
    the vertex position, normal and texcoords reside
    in 3 separate arrays.  This method desconstructs
    the fully-qualified vertex list into 3 separate,
    simpler vertex lists, overwriting the original
    lists used to read in the Wavefront OBJ data.

*/
void WavefrontObjFile::FillSimpleVertexContainers( void )
{
    mVertexPositions.Clear() ;
    mVertexPositions.Reserve( mUniqueVertices.Size() ) ;

    mVertexTexCoords.Clear() ;
    mVertexTexCoords.Reserve( mUniqueVertices.Size() ) ;

    mVertexNormals.Clear() ;
    mVertexNormals.Reserve( mUniqueVertices.Size() ) ;

    for( HashTable<Vertex>::Iterator iter = mUniqueVertices.Begin() ; iter != mUniqueVertices.End() ; ++ iter )
    {   // For each unique vertex...
        const Vertex &  rVert   = * iter ;
        const int &     iVert   = rVert.i ;
        mVertexPositions.At( iVert ) = rVert.p ;
        mVertexTexCoords.At( iVert ) = rVert.t ;
        mVertexNormals  .At( iVert ) = rVert.n ;
    }
    mUniqueVertices.Clear() ;
}




void WavefrontObjFile::LoadFromFile( const char * strFilename )
{
    const char * strPathChosen ;
    // Open file for reading (in "binary" mode, i.e. do not translate cr+lf)
    FILE * pFile = FileOpen( strFilename , "rb" , & strPathChosen ) ;
    if( NULL == pFile )
    {   // Could not open file
        int i ;
        i = errno ;
        ASSERT( 0 ) ;   // Check the value of errno to see what specific error occurred.
    }

    #if 0
        // Get file size
        fseek( pFile , 0, SEEK_END ) ;
        unsigned long dwFileSize = ftell( pFile ) ;
        fseek( pFile, 0, SEEK_SET ) ;
    #endif

    char buffer[ 1024 + 1 ] ;
    static unsigned int usableBufferSize = sizeof( buffer ) - 1 ; // Leave 1 element for emergency nul terminator
    memset( buffer , 0 , sizeof( buffer ) ) ;   // Fill with nul terminators
    while( ! feof( pFile ) )
    {
        char * rv = fgets( buffer , usableBufferSize , pFile ) ;
        ASSERT( '\0' == buffer[ usableBufferSize ] ) ; // Buffer should always have nul-terminator
        if( rv != 0 )
        {   // Read data from file into buffer
            ParseLine( buffer , sizeof( buffer ) ) ;
        }
        else
        {   // Failed to read data from a file.
            // Probably reached end-of-file.
            int i ;
            i = errno ;
            continue ;  // Set breakpoint here and inspect errno
        }
    } ;

    fclose( pFile ) ;

    if( mSimpleLayout )
    {   // Geometry satisfies special criteria that make using this data easier.
        mIndices.Reserve( mFaces.Size() ) ;
        for( Fiea::Vector<PolygonFace>::Iterator iter = mFaces.Begin() ; iter != mFaces.End() ; ++ iter )
        {
            PolygonFace & rFace = * iter ;
            mIndices.PushBack( rFace.GetPos(0) ) ;
            mIndices.PushBack( rFace.GetPos(1) ) ;
            mIndices.PushBack( rFace.GetPos(2) ) ;
        }
    }
    else
    {   // Geometry requires some processing before it can be used directly by GPU
        ReconstructFaces() ;
        FillSimpleVertexContainers() ;
    }
}
