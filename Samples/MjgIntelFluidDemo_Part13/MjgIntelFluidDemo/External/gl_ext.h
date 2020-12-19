#ifndef GL_ARB_texture_cube_map 
#define GL_ARB_texture_cube_map 
#define GL_TEXTURE_CUBE_MAP               0x8513
#define GL_REFLECTION_MAP                 0x8512
#define GL_TEXTURE_BINDING_CUBE_MAP       0x8514
#define GL_TEXTURE_CUBE_MAP_POSITIVE_X    0x8515
#define GL_TEXTURE_CUBE_MAP_NEGATIVE_X    0x8516
#define GL_TEXTURE_CUBE_MAP_POSITIVE_Y    0x8517
#define GL_TEXTURE_CUBE_MAP_NEGATIVE_Y    0x8518
#define GL_TEXTURE_CUBE_MAP_POSITIVE_Z    0x8519
#define GL_TEXTURE_CUBE_MAP_NEGATIVE_Z    0x851A
#define GL_MAX_CUBE_MAP_TEXTURE_SIZE      0x851C
#endif

#if 0 // GL_EXT_texture_cube_map
#define GL_EXT_texture_cube_map
# define GL_NORMAL_MAP_EXT                   0x8511
# define GL_REFLECTION_MAP_EXT               0x8512
# define GL_TEXTURE_CUBE_MAP_EXT             0x8513
# define GL_TEXTURE_BINDING_CUBE_MAP_EXT     0x8514
# define GL_TEXTURE_CUBE_MAP_POSITIVE_X_EXT  0x8515
# define GL_TEXTURE_CUBE_MAP_NEGATIVE_X_EXT  0x8516
# define GL_TEXTURE_CUBE_MAP_POSITIVE_Y_EXT  0x8517
# define GL_TEXTURE_CUBE_MAP_NEGATIVE_Y_EXT  0x8518
# define GL_TEXTURE_CUBE_MAP_POSITIVE_Z_EXT  0x8519
# define GL_TEXTURE_CUBE_MAP_NEGATIVE_Z_EXT  0x851A
# define GL_PROXY_TEXTURE_CUBE_MAP_EXT       0x851B
# define GL_MAX_CUBE_MAP_TEXTURE_SIZE_EXT    0x851C
#endif

#ifndef GL_ARB_point_sprite
#define GL_POINT_SPRITE_ARB               0x8861
#define GL_COORD_REPLACE_ARB              0x8862
#endif

#include <stddef.h>

typedef ptrdiff_t GLintptr;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptrARB;
typedef ptrdiff_t GLsizeiptrARB;

#define GLAPI extern
#define GL_ARRAY_BUFFER                   0x8892
#define GL_STREAM_DRAW                    0x88E0
#define GL_WRITE_ONLY                     0x88B9

typedef void (APIENTRY * PFN_GL_BINDBUFFERARB_PROC) (GLenum target, GLuint buffer);
typedef void (APIENTRY * PFN_GL_DELETEBUFFERSARB_PROC) (GLsizei n, const GLuint *buffers);
typedef void (APIENTRY * PFN_GL_GENBUFFERSARB_PROC) (GLsizei n, GLuint *buffers);
typedef void (APIENTRY * PFN_GL_BUFFERDATAARB_PROC) (GLenum target, int size, const GLvoid *data, GLenum usage);
typedef void * (APIENTRY * PFN_GL_MAPBUFFERARB_PROC ) (GLenum, GLenum) ;
typedef GLboolean ( APIENTRY * PFN_GL_UNMAPBUFFERARB_PROC ) (GLenum);
