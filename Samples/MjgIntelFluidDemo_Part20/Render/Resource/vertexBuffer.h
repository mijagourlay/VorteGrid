/** \file vertexBuffer.h

    \brief Vertex buffer base class

    \author Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_VERTEX_BUFFER_BASE_H
#define PEGASYS_RENDER_VERTEX_BUFFER_BASE_H

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

namespace PeGaSys
{
    namespace Render
    {
        /** Declaration of a vertex format.

            \todo   This is a placeholder for a more sophisticated, flexible
                    mechanism for declaring a vertex format.  Meanwhile it
                    exists only in a preliminary state, in the hope that once
                    it is fully implemented, the API will remain mostly unchanged.

                    Eventually, a generic "vertex declaration" will be a list of VertexElements.

                    (For now, this class merely wraps VertexFormatE.)

        */
        class VertexDeclaration
        {
        public:
            /** Element in a vertex format.

                The combination of offset and stride allows interleaved or contiguous data.

                \todo   Make use of this class.
            */
            class VertexElement
            {
            public:
                enum SemanticE
                {
                    SEMANTIC_NONE   ,
                    POSITION        ,
                    NORMAL          ,
                    BINORMAL        ,
                    TANGENT         ,
                    COLOR_AMBIENT   ,
                    COLOR_DIFFUSE   ,
                    COLOR_SPECULAR  ,
                    TEXTURE_0       ,
                    TEXTURE_1       ,
                    BLEND_WEIGHTS   ,
                    NUM_SEMANTICS   ,
                } ;

                enum DataTypeE
                {
                    DATA_TYPE_NONE  ,
                    BYTE            ,
                    SHORT           ,
                    INT             ,
                    FLOAT           ,
                    FLOAT2          ,
                    FLOAT3          ,
                    FLOAT4          ,
                    NUM_DATA_TYPES
                } ;

                SemanticE   mSemantic   ;   /// Semantic (meaning/purpose) for this vertex element.
                DataTypeE   mDataType   ;   /// Primitive type of data used by this vertex element.
                size_t      mOffset     ;   /// Offset, in bytes, from start of vertex buffer to first of this kind of element.
                size_t      mStride     ;   /// Number of bytes between elements of this type.
            } ;


            /** Simple common vertex formats.

                In principle, the underlying API supports more formats than these,
                and the "vertex format" should be more flexible than this.  See VertexElement.

                Note that some API's, like DirectX8, impose restrictions on the
                ordering of vertex elements, so the ordering implied by these names
                might not represent the actual ordering within the vertex data
                for all platforms.
            */
            enum VertexFormatE
            {
                VERTEX_FORMAT_NONE              ,

                POSITION                        ,

                POSITION_NORMAL                 ,
                POSITION_COLOR                  ,
                POSITION_TEXTURE                ,

                POSITION_NORMAL_COLOR           ,
                POSITION_NORMAL_TEXTURE         ,
                POSITION_COLOR_TEXTURE          ,

                POSITION_NORMAL_COLOR_TEXTURE   ,

                GENERIC                         ,

                NUM_FORMATS
            } ;


            /// Initialize a vertex declaration object.
            VertexDeclaration()
                : mVertexFormat( VERTEX_FORMAT_NONE )
            {}


            /** Initialize a vertex declaration object.

                \param vertexFormat     Format to use to represent a vertex

                \note   VertexFormatE is a simplistic, inflexible way to declare a vertex format.
                        It is convenient but a another mechanism could provide more flexible format declarations.
            */
            explicit VertexDeclaration( VertexFormatE vertexFormat )
                : mVertexFormat( vertexFormat )
            {
            }


            const VertexFormatE & GetVertexFormat() const
            {
                return mVertexFormat ;
            }


            /// Return whether this vertex declaration has normals, i.e. supports lighting/shading.
            bool HasNormals() const
            {
                switch( mVertexFormat )
                {
                case POSITION_NORMAL:
                case POSITION_NORMAL_COLOR:
                case POSITION_NORMAL_TEXTURE:
                case POSITION_NORMAL_COLOR_TEXTURE:
                    return true ;
                }
                return false ;
            }


            /// Return whether this vertex declaration has colors, i.e. should use per-vertex color.
            bool HasColors() const
            {
                switch( mVertexFormat )
                {
                case POSITION_COLOR:
                case POSITION_NORMAL_COLOR:
                case POSITION_COLOR_TEXTURE:
                case POSITION_NORMAL_COLOR_TEXTURE:
                    return true ;
                }
                return false ;
            }


            /// Return whether this vertex declaration supports textures.
            bool HasTextureCoordinates() const
            {
                switch( mVertexFormat )
                {
                case POSITION_TEXTURE:
                case POSITION_NORMAL_TEXTURE:
                case POSITION_COLOR_TEXTURE:
                case POSITION_NORMAL_COLOR_TEXTURE:
                    return true ;
                }
                return false ;
            }


            /// Return whether this vertex declaration is invalid.
            bool IsInvalid() const
            {
                return ( VERTEX_FORMAT_NONE == mVertexFormat ) ;
            }

            VertexFormatE   mVertexFormat ; ///< Format used to represent a vertex
        } ;


        /** Format of a generic vertex, used as a an intermediate format.

            Use this format to populate a vertex buffer from a generic file
            format (such as Wavefront OBJ) or procedurally, then pass that
            generic buffer to the platform-specific implementations to translate
            that into a format ready to render.

            \note   This is a temporary, quick-and-dirty hack.  More proper
                    alternatives to this include the following:

            -   Support "vertex declarations" as noted above.

            From there, multiple options follow:

            -   Use the generic format directly.  This would work in
                combination with using facilities in the underlying
                rendering API (such as OpenGL or Direct3D) to support
                supplying offset+stride information, so that the
                "generic format" would be identical to the
                platform-specific format.  This has the nice benefit
                that game-ready assets could be the same for all
                platforms.  But it relies on the rendering API to
                support offset+index facilities, which many legacy
                systems (such as D3D8, used by original Xbox, and
                probably version of OpenGL that precede vertex buffer
                arrays, possibly including OpenGL-ES) do not.

            -   Provide an abstract interface for each field of a vertex.
                Each platform-specific VertexBuffer would implement an
                accessor, given the vertex index and vertex element.
                This would entail a virtual call for each access, which
                would be incredibly slow, but it would work for all
                platforms and would only be slow when populating vertex
                buffers, which presumably would only happen at load time.

            -   Omit in-game support for generic memory formats.

            From there, multiple options follow:

            -   Each platform would supply a translator from a generic
                file format to a platform-specific memory format.

            -   The external asset conditioning pipeline must convert
                the assets from a generic file format to a game-ready
                format.

            This "generic intermediary" approach could work in conjunction
            with having the asset pipeline prepare platform-specific
            game-ready assets.  The generic form could be relegated to rapid
            prototyping or iterative development of assets, where the
            platform-specific, game-ready form would be meant for shipping.

            In practice, all of these approaches could work.  For platforms
            which allow use of generic memory formats, the translators would
            not be necessary -- thereby potentially reducing the amount of
            platform-specific code.  And for platforms that do not support
            generic memory formats, fall back to relying on translators,
            either in-game or exporter.

            Note that the in-game and exporter translators should use the
            same code -- but it should be possible to "disable" the in-game
            code, for compiling "ship" versions, to reduce code size.

        */
        struct GenericVertex
        {
            float ts, tt, tu, tv    ;   ///< 4D texture coordinates
            float cr, cg, cb, ca    ;   ///< color in RGBA form
            float nx, ny, nz        ;   ///< surface normal unit vector
            float px, py, pz, pw    ;   ///< untransformed (world-space) position
        } ;


        /** Vertex buffer base class.

            Each rendering platform specializes this class.
        */
        class VertexBufferBase
        {
        public:
            VertexBufferBase() ;
            virtual ~VertexBufferBase() ;

            /// Return type identifier for this object.
            const unsigned & GetTypeId() const
            { return mTypeId ; }

            /// Platform-specific routine to format vertex buffer.
            virtual void DeclareVertexFormat( const VertexDeclaration & vertexDeclaration ) = 0 ;

            /// Return reference to object that declares format for vertices in this buffer.
            const VertexDeclaration &   GetVertexDeclaration() const
            { return mVertexDeclaration ; }

            /// Return size, in bytes, of each vertex occupies.
            const size_t &              GetVertexSizeInBytes() const
            { return mVertexSize ; }

            GenericVertex * AllocateGeneric( size_t numVertices ) ;

            /// Platform-specific routine to allocate vertex buffer.
            virtual bool    Allocate( size_t numVertices ) = 0 ;

            /// Platform-specific routine to acquire a lock on vertex data and return its starting address.
            virtual void *  LockVertexData() = 0 ;

            /// Platform-specific routine to unlock vertex data previously locked by LockVertexData.
            virtual void    UnlockVertexData() = 0 ;

            /// Platform-specific routine to obtain address of first of a given element.
            /// If the vertex format has more than one element with the given semantic, which indicates which one to obtain.
            virtual void * GetElementStart( void * vertexData , VertexDeclaration::VertexElement::SemanticE semantic , size_t which ) = 0 ;

            /// Platform-specific routine to translate vertices from generic intermediate format to one ready to render.
            virtual void TranslateFromGeneric( const VertexDeclaration & targetVertexDeclaration ) = 0 ;

            /// Platform-specific routine to empty and release vertex buffer.
            virtual void Clear() = 0 ;

            bool ChangeCapacityAndReallocate( size_t numVertices ) ;

            void SetPopulation( size_t numVertices ) ;

            /// Return number of vertices in this buffer (i.e. actual population).
            const size_t &  GetPopulation() const
            { return mPopulation       ; }

            /// Return number of vertices this buffer can hold.
            const size_t &  GetCapacity() const
            { return mCapacity ; }

        protected:
            /// Platform-specific routine to deallocate vertex buffer.
            virtual void Deallocate() = 0 ;

                  GenericVertex * GetGenericVertexData()        { return mGenericVertexData ; }
            const GenericVertex * GetGenericVertexData() const  { return mGenericVertexData ; }

            void    SetVertexDeclaration( const VertexDeclaration & vertexDeclaration ) ;
            void    SetVertexSize( size_t vertexSize ) ;
            void    SetCapacity( size_t capacity ) ;

            static const unsigned sTypeId = 'VXBF' ;    ///< Type identifier for VertexBufferBase

            unsigned            mTypeId             ;   ///< Type identifier, for run-time type checking

        private:
            GenericVertex   *   mGenericVertexData  ;   ///< Address of generic vertex data.  Mutually exclusive with any platform-specific vertex data.
            VertexDeclaration   mVertexDeclaration  ;   ///< Declaration of vertex format
            size_t              mVertexSize         ;   ///< Size, in bytes, of a single vertex -- Stride between elements in mVertexData.
            size_t              mPopulation         ;   ///< Number of vertices in buffer (i.e. actual population).
            size_t              mCapacity           ;   ///< Number of vertices this buffer can hold.
        } ;

        // Public variables ------------------------------------------------------------
        // Public functions ------------------------------------------------------------

    } ;


    /// Return whether the given address is aligned to the given boundary.
    static inline bool IsAligned( void * address , size_t numBytes )
    {
        size_t addrInt   = reinterpret_cast< size_t >( address ) ;
        size_t remainder = addrInt % numBytes ;
        return 0 == remainder ;
    }

} ;

#endif
