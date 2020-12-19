/** \file wavefrontObj.h

    \brief Utility routines for reading Wavefront Object files

    \author Copyright 2008-2014 MJG; All rights reserved.
*/
#ifndef OBJ_H
#define OBJ_H

#include <Core/Math/vec4.h>
#include <Core/Containers/vector.h>
#include <Core/Containers/hash.h>

// Macros --------------------------------------------------------------
// Types --------------------------------------------------------------

/*! \brief Class to mediate access to Wavefront OBJ file
*/
class WavefrontObjFile
{
    public:
        WavefrontObjFile( void ) ;
        ~WavefrontObjFile()
        {
            mVertexPositions.Clear()    ;
            mVertexNormals.Clear()      ;
            mVertexTexCoords.Clear()    ;
            mIndices.Clear()            ;
            mFaces.Clear()              ;
        }

        void LoadFromFile( const char * strFilename ) ;

        /*! \brief Set a group name to include when extracting vertex information.
            
            A Wavefront OBJ file can contain multiple groups.
            The caller may choose to include a number of groups from within the file.
            If the set of groups is empty then all groups are included.
            When the file is read, vertex information from all included groups
            are stored within the appropriate buffers.

            \param strGroup - name of a group.

            \param index - index of group.  Up to sNumGroupsMax can be assigned.

        */
        void SetGroupName( const char * strGroup , int index = 0 )
        {
            ASSERT( index < sNumGroupsMax ) ;
            mGroups[ index ] = strGroup ;
        }

        /*! \brief Get number of vertices read from the OBJ file.
        */
        inline const int    GetNumVertices( void ) const        { return mVertexPositions.Size() ; }

        /*! \brief Get vertex position information as an array of Vec4
        */
        inline const Vec4 * GetVertexPositions( void ) const    { return & mVertexPositions.Front() ; }

        /*! \brief Get vertex normal information as an array of Vec4
        */
        inline const Vec4 * GetVertexNormals( void ) const      { return & mVertexNormals.Front() ; }

        /*! \brief Get vertex texture coordinate information as an array of Vec4
        */
        inline const Vec4 * GetVertexTexCoords( void ) const    { return & mVertexTexCoords.Front() ; }

        /*! \brief Get number of indices read from the OBJ file.
        */
        inline const int    GetNumIndices( void ) const         { return mIndices.Size() ; }

        /*! \brief Get face information as triads of indices into vertex arrays.

            \note   Wavefront OBJ files allow model geometry faces to use different position, normal and texture coordinate values.
                    In principle, those arrays do not have to have the same number of elements and values each array at specific indices
                    do not have to correspond to values in other arrays at the same indices.
                    This implementation, however, only supports OBJ files where the numbers of positions, normals and texture coordinates match.
                    In other words, the index array represents indices into the vertex positions array and assumes
                    the other vertex arrays use the same indices.
        */
        inline const int  * GetIndices( void ) const            { return & mIndices.Front() ; }

        /*! \brief Return whether this model has texture coordinates
        */
        inline const bool & HasTexCoords( void ) const          { return mHasTexCoords ; }
        inline const bool & HasNormals( void ) const            { return mHasNormals ; }

    private:
        /*! \brief Polygon face information

            \note   This implementation supports triangles only.
                    Wavefront OBJ files support n-sided polygons but renderers can only use triangles (and in some cases quads).
                    This struct contains enough space for quads.
        */
        struct PolygonFace
        {
            PolygonFace()
            {
                memset( p , 0 , sizeof( p ) ) ;
                memset( t , 0 , sizeof( t ) ) ;
                memset( n , 0 , sizeof( n ) ) ;
                numVerts = 0 ;
            }
            ~PolygonFace() {} ;
            void AddVertP( int iPos )
            {
                ASSERT( numVerts < MAX_NUM_VERTS ) ;
                p[ numVerts ] = iPos ;
                ++ numVerts ;
            }
            void AddVertPT( int iPos , int iTexCoord )
            {
                ASSERT( numVerts < MAX_NUM_VERTS ) ;
                ASSERT( iTexCoord >= 0 ) ;
                t[ numVerts ] = iTexCoord ;
                AddVertP( iPos ) ;
            }
            void AddVertPN( int iPos , int iNormal )
            {
                ASSERT( numVerts < MAX_NUM_VERTS ) ;
                n[ numVerts ] = iNormal ;
                AddVertP( iPos ) ;
            }
            void AddVertPTN( int iPos , int iTexCoord , int iNormal )
            {
                ASSERT( numVerts < MAX_NUM_VERTS ) ;
                n[ numVerts ] = iNormal ;
                AddVertPT( iPos , iTexCoord ) ;
            }
            inline const int  GetPos( const int & i )      const { return p[ i ] - 1 ; }
            inline const int  GetTexCoord( const int & i ) const
            {
                ASSERT( i >= 0 ) ;
                ASSERT( i < 6  ) ;
                ASSERT( t[i] >= 0 ) ;
                return t[ i ] - 1 ;
            }
            inline const int  GetNormal( const int & i )   const { return n[ i ] - 1 ; }

            inline const bool HasPos( const int & i )      const { return p[ i ] != UNASSIGNED ; }
            inline const bool HasTexCoord( const int & i ) const { return t[ i ] != UNASSIGNED ; }
            inline const bool HasNormal( const int & i )   const { return n[ i ] != UNASSIGNED ; }

            inline const int & GetNumVerts( void )          const { return numVerts ; }
            static const int MAX_NUM_VERTS = 6 ;
        private:
            static const int UNASSIGNED = 0 ; ///< Indicates face element has no value
            int p[6]        ;   ///< Vertex position indices
            int t[6]        ;   ///< Vertex texture coordinate indices
            int n[6]        ;   ///< Vertex normal indices
            int numVerts    ;   ///< Number of vertices in this face.  Usually this would be 3 (for triangle) but sometimes 4 (for quads).
        } ;

        /*! \brief Fully qualified vertex from Wavefront OBJ file

            \note   The layout of members within this struct is fragile.
                    When hashing, only the {p,t,n} members should be used.
                    The index "i" should NOT be part of the hash or comparisons.
        */
        struct Vertex // : public HashEntry
        {
            Vertex()
                : p( 0.0f , 0.0f , 0.0f , 0.0f )
                , t( 0.0f , 0.0f , 0.0f , 0.0f )
                , n( 0.0f , 0.0f , 0.0f , 0.0f )
            {}

            ~Vertex() {} ;
            /*! \brief  Determine whether vertex data matches

                \note   This ignores the vertex index, because this data is used
                        to create an ordered list of unique vertices, and the
                        ordering (provided by the index) is not part of what makes a vertex unique.
                        This could (perhaps should) use a "map" instead, where the map is first populated
                        with unique verts, then lookups are done on those and the 'index' is simply the
                        location within the map, instead of being stored explicitly with each vert.
                        But HashTable is faster than Map.

            */
            inline bool   operator == ( const Vertex & rhs ) const
            {
                return (    ( p == rhs.p )
                        &&  ( t == rhs.t )
                        &&  ( n == rhs.n )  ) ;
            }

            inline Vertex & operator = ( const Vertex & rhs )
            {
                ASSERT( p == rhs.p ) ;
                ASSERT( t == rhs.t ) ;
                ASSERT( n == rhs.n ) ;
                i = rhs.i ;
                return * this ;
            }

            explicit Vertex( const char * strKey , unsigned int keySize = 0 , bool bDeleteKey = true ) ;

            void                SetKey( const char * name, unsigned int keySize , bool bDeleteKey = true )
            {
            #if 0
                // Base class members
                mKey = 0 ;
                mDeleteKey  = bDeleteKey ;
            #endif

                // Derived class members
                Vertex * pRhs = (Vertex*) name ;
                p = pRhs->p  ;
                t = pRhs->t  ;
                n = pRhs->n  ;
            }

            //! \brief Access key data
            //! \return address of key data
            const char *        GetKey( void ) const        { ASSERT( 0 ) ; return (const char *) & p ; /* do not use */ }

            bool                CompareKey( const char * key , int keySize = 0 ) const
            {
                Vertex * pRhs = (Vertex*) key ;
                return * this == * pRhs ;   // Relies on overridden operator==
            }


            /*! \brief compute hash value from given key data

                \note To index the hash table, the hash value must be truncated.
                    The hash table (not the hash entry) dictates the truncation policy,
                    e.g. modulo prime or bit-masking.
            */
            static unsigned int HashCompute( const char * pKey , unsigned int keySize )
            {
                Vertex * pVert = (Vertex*) pKey ;
                // Skip past HashEntry part of key when computing hash value.
                const char * strKey = (const char *) & pVert->p ;
                unsigned int hash = 0 ;

                // hashCharactersMax is the maximum number of characters
                // of the name we use to generate the hash value.  The
                // larger this value, the less likely the hash value will
                // collide with other names, but the slower it takes to assign
                // names.
                // static const int hashCharactersMax = 4 ;

                // Compute simple hash value for this name
                if( 0 == keySize )
                {   // No size provided;  assume key is a string.
                    for( unsigned int iChar = 0 ; strKey[ iChar ] != '\0' ; ++ iChar )
                    {
                        hash = hash * 65 + strKey[ iChar ] ;
                    }
                }
                else
                {   // Treat key as a series of raw bytes.
                    for( unsigned int iChar = 0 ; iChar < keySize ; ++ iChar )
                    {
                        hash = hash * 3 + strKey[ iChar ] ;
                    }
                }

                return hash ;
            }

            Vec4 p ;    ///< position
            Vec4 t ;    ///< texture coordinate
            Vec4 n ;    ///< normal

            int  i ;    ///< index used for simplified format
        } ;

        void ParseLine( const char * buffer , size_t bufferSize ) ;

        Vertex & InsertUniqueVertex( const Vertex & v )
        {
            ASSERT( v.p.w == 1.0f ) ;
            ASSERT( v.t.w == 0.0f ) ;
            ASSERT( v.n.w == 0.0f ) ;
            ASSERT( ( v.n.Mag2() == 0.0f ) || v.n.IsNormalizedv3() ) ;
            Vertex * pVert = mUniqueVertices.FindEntry( (const char *) & v ) ;
            if( 0 == pVert )
            {   // v is a new unique vertex so insert it
                Vertex & rVert = mUniqueVertices.Insert( (const char *) & v ) ;
                rVert.i = mUniqueVertices.Size() - 1 ;
                return rVert ;
            }
            // ...else matching vertex was already in table so return it
            ASSERT( pVert->p.w == 1.0f ) ;
            ASSERT( pVert->t.w == 0.0f ) ;
            ASSERT( pVert->n.w == 0.0f ) ;
            ASSERT( ( pVert->n.Mag2() == 0.0f ) || pVert->n.IsNormalizedv3() ) ;
            ASSERT( ( pVert->i >= 0 ) && ( pVert->i < (signed) mUniqueVertices.Size() ) ) ;
            return * pVert ;
        }

        void ReconstructFaces( void ) ;
        void FillSimpleVertexContainers( void ) ;

        static const int            sNumGroupsMax = 32 ;
        const char *                mGroups[ sNumGroupsMax ]        ;   ///< addresses of names of groups to read
        Fiea::Vector<Vec4>          mVertexPositions                ;   ///< Container of vertex positions
        int                         mVertexPositionNumComponents    ;   ///< Number of components in each vertex position (typically 3 or 4)
        Fiea::Vector<Vec4>          mVertexNormals                  ;   ///< Container of vertex normals
        int                         mVertexTexCoordNumComponents    ;   ///< Number of components in each vertex tex coord (typically 3 or 4)
        Fiea::Vector<Vec4>          mVertexTexCoords                ;   ///< Container of texture coordinates
        Fiea::Vector<int>           mIndices                        ;   ///< Container of index triads for triangle faces
        Fiea::Vector<PolygonFace>   mFaces                          ;   ///< Container of polygon faces
        HashTable<Vertex>           mUniqueVertices                 ;   ///< Container of unique vertices
        bool                        mInIncludedGroup                ;   ///< Parser state variable: Whether cursor is inside an included group.
        bool                        mSimpleLayout                   ;   ///< Geometry satisfies special conditions that make using this file easier.
        bool                        mHasTexCoords                   ;   ///< Geometry has texture coordinates
        bool                        mHasNormals                     ;   ///< Geometry has normals
} ;

// Public variables --------------------------------------------------------------
// Public functions --------------------------------------------------------------

#endif
