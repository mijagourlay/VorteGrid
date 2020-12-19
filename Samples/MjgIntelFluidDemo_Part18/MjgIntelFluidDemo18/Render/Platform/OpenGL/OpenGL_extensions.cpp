/** \file OpenGL_extensions.cpp

    \brief Utility routines to obtain OpenGL extensions

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/

#include "OpenGL_Extensions.h"
#include "OpenGL_Api.h"

#include <Core/Performance/perfBlock.h>
#include <Core/Utility/macros.h>
#include <Core/File/debugPrint.h>

#if defined( WIN32 )
#   include <windows.h>
#endif

// Macros -----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

#if 1 // glext.h should define these, if that verion of glext.h corresponds to your graphics driver, and that driver supports these extensions.
typedef void (APIENTRYP PFNGLDEBUGMESSAGECONTROLARBPROC) (unsigned int source, unsigned int type, unsigned int severity, int count, const unsigned int* ids, bool enabled);
typedef void (APIENTRYP PFNGLDEBUGMESSAGEINSERTARBPROC) (unsigned int source, unsigned int type,  unsigned int id, unsigned int severity, int length, const char* buf);
typedef void (APIENTRY *GLDEBUGPROCARB)(unsigned int source, unsigned int type, unsigned int id,  unsigned int severity, int length, const char* message, void* userParam);
typedef void (APIENTRYP PFNGLDEBUGMESSAGECALLBACKARBPROC) (GLDEBUGPROCARB callback,  void* userParam);
typedef unsigned int (APIENTRYP PFNGLGETDEBUGMESSAGELOGARBPROC) (unsigned int count, int bufsize, unsigned int* sources,unsigned int* types, unsigned int* ids,  unsigned int* severities, int* lengths, char* messageLog);
#endif

// Public variables ------------------------------------------------------------

PFNGLDEBUGMESSAGECONTROLARBPROC   glDebugMessageControlARB   = NULL ;
PFNGLDEBUGMESSAGEINSERTARBPROC    glDebugMessageInsertARB    = NULL ;
PFNGLDEBUGMESSAGECALLBACKARBPROC  glDebugMessageCallbackARB  = NULL ;
PFNGLGETDEBUGMESSAGELOGARBPROC    glGetDebugMessageLogARB    = NULL ;

// Framebuffer object
PFNGLGENFRAMEBUFFERSEXTPROC                     glGenFramebuffersEXT                     = 0 ; // FBO name generation procedure
PFNGLDELETEFRAMEBUFFERSEXTPROC                  glDeleteFramebuffersEXT                  = 0 ; // FBO deletion procedure
PFNGLBINDFRAMEBUFFEREXTPROC                     glBindFramebufferEXT                     = 0 ; // FBO bind procedure
PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC              glCheckFramebufferStatusEXT              = 0 ; // FBO completeness test procedure
PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC glGetFramebufferAttachmentParameterivEXT = 0 ; // return various FBO parameters
PFNGLGENERATEMIPMAPEXTPROC                      glGenerateMipmapEXT                      = 0 ; // FBO automatic mipmap generation procedure
PFNGLFRAMEBUFFERTEXTURE2DEXTPROC                glFramebufferTexture2DEXT                = 0 ; // FBO texdture attachement procedure
PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC             glFramebufferRenderbufferEXT             = 0 ; // FBO renderbuffer attachement procedure

// Renderbuffer object
PFNGLGENRENDERBUFFERSEXTPROC                    glGenRenderbuffersEXT                    = 0 ; // renderbuffer generation procedure
PFNGLDELETERENDERBUFFERSEXTPROC                 glDeleteRenderbuffersEXT                 = 0 ; // renderbuffer deletion procedure
PFNGLBINDRENDERBUFFEREXTPROC                    glBindRenderbufferEXT                    = 0 ; // renderbuffer bind procedure
PFNGLRENDERBUFFERSTORAGEEXTPROC                 glRenderbufferStorageEXT                 = 0 ; // renderbuffer memory allocation procedure
PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC          glGetRenderbufferParameterivEXT          = 0 ; // return various renderbuffer parameters
PFNGLISRENDERBUFFEREXTPROC                      glIsRenderbufferEXT                      = 0 ; // determine renderbuffer object type

PFNGLFRAMEBUFFERTEXTUREEXTPROC                  glFramebufferTextureEXT                  = 0 ;

PFNGLDRAWBUFFERSPROC                            glDrawBuffersEXT                         = 0 ;

// VBO Extension Function Pointers
PFNGLGENBUFFERSARBPROC       glGenBuffers    = 0 ;   ///< VBO Name Generation Procedure
PFNGLMAPBUFFERARBPROC        glMapBuffer     = 0 ;   ///< VBO Map Buffer procedure
PFNGLUNMAPBUFFERARBPROC      glUnmapBuffer   = 0 ;   ///< VBO Unmap Buffer procedure
PFNGLBINDBUFFERARBPROC       glBindBuffer    = 0 ;   ///< VBO Bind Procedure
PFNGLBUFFERDATAARBPROC       glBufferData    = 0 ;   ///< VBO Data Loading Procedure
PFNGLDELETEBUFFERSARBPROC    glDeleteBuffers = 0 ;   ///< VBO Deletion Procedure

namespace PeGaSys {
    namespace Render {
        namespace OpenGL_Extensions {

            // Private variables -----------------------------------------------------------
            // Private functions -----------------------------------------------------------

            void GetVersionInfo()
            {
                PERF_BLOCK( OpenGL_Extensions__GetVersionInfo ) ;

                const GLubyte * glVendor     = glGetString( GL_VENDOR ) ;
                const GLubyte * glRenderer   = glGetString( GL_RENDERER ) ;
                const GLubyte * glVersion    = glGetString( GL_VERSION ) ;
                const GLubyte * glExtensions = glGetString( GL_EXTENSIONS ) ;
                (void) glVendor , glRenderer , glVersion , glExtensions ; // Used for debugging only.
            }

            // Public functions ------------------------------------------------------------

            /** Lookup various OpenGL extensions and make them available for use.

                This application can use certain features of OpenGL which are
                not standard across all platforms and all implementations.
                For example, some of these extensions do not work when using
                Remote Desktop or on extremely old video adapters.
            */
            /* static */ void GetProcAddresses()
            {
                PERF_BLOCK( OpenGL_Extensions__GetProcAddresses ) ;

                glDebugMessageControlARB   = (PFNGLDEBUGMESSAGECONTROLARBPROC)  wglGetProcAddress( "glDebugMessageControlARB"  ) ;
                glDebugMessageInsertARB    = (PFNGLDEBUGMESSAGEINSERTARBPROC)   wglGetProcAddress( "glDebugMessageInsertARB"   ) ;
                glDebugMessageCallbackARB  = (PFNGLDEBUGMESSAGECALLBACKARBPROC) wglGetProcAddress( "glDebugMessageCallbackARB" ) ;
                glGetDebugMessageLogARB    = (PFNGLGETDEBUGMESSAGELOGARBPROC)   wglGetProcAddress( "glGetDebugMessageLogARB"   ) ;

                glGenFramebuffersEXT                     = (PFNGLGENFRAMEBUFFERSEXTPROC                    ) wglGetProcAddress( "glGenFramebuffersEXT"                      ) ;
                glDeleteFramebuffersEXT                  = (PFNGLDELETEFRAMEBUFFERSEXTPROC                 ) wglGetProcAddress( "glDeleteFramebuffersEXT"                   ) ;
                glBindFramebufferEXT                     = (PFNGLBINDFRAMEBUFFEREXTPROC                    ) wglGetProcAddress( "glBindFramebufferEXT"                      ) ;
                glCheckFramebufferStatusEXT              = (PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC             ) wglGetProcAddress( "glCheckFramebufferStatusEXT"               ) ;
                glGetFramebufferAttachmentParameterivEXT = (PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC) wglGetProcAddress( "glGetFramebufferAttachmentParameterivEXT"  ) ;
                glGenerateMipmapEXT                      = (PFNGLGENERATEMIPMAPEXTPROC                     ) wglGetProcAddress( "glGenerateMipmapEXT"                       ) ;
                glFramebufferTexture2DEXT                = (PFNGLFRAMEBUFFERTEXTURE2DEXTPROC               ) wglGetProcAddress( "glFramebufferTexture2DEXT"                 ) ;
                glFramebufferRenderbufferEXT             = (PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC            ) wglGetProcAddress( "glFramebufferRenderbufferEXT"              ) ;

                glGenRenderbuffersEXT                    = (PFNGLGENRENDERBUFFERSEXTPROC                   ) wglGetProcAddress( "glGenRenderbuffersEXT"                     ) ;
                glDeleteRenderbuffersEXT                 = (PFNGLDELETERENDERBUFFERSEXTPROC                ) wglGetProcAddress( "glDeleteRenderbuffersEXT"                  ) ;
                glBindRenderbufferEXT                    = (PFNGLBINDRENDERBUFFEREXTPROC                   ) wglGetProcAddress( "glBindRenderbufferEXT"                     ) ;
                glRenderbufferStorageEXT                 = (PFNGLRENDERBUFFERSTORAGEEXTPROC                ) wglGetProcAddress( "glRenderbufferStorageEXT"                  ) ;
                glGetRenderbufferParameterivEXT          = (PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC         ) wglGetProcAddress( "glGetRenderbufferParameterivEXT"           ) ;
                glIsRenderbufferEXT                      = (PFNGLISRENDERBUFFEREXTPROC                     ) wglGetProcAddress( "glIsRenderbufferEXT"                       ) ;

                glFramebufferTextureEXT                  = (PFNGLFRAMEBUFFERTEXTUREEXTPROC                 ) wglGetProcAddress( "glFramebufferTextureEXT"                   ) ;

                glDrawBuffersEXT                         = (PFNGLDRAWBUFFERSPROC                           ) wglGetProcAddress( "glDrawBuffers"                             ) ;

                glGenBuffers                             = (PFNGLGENBUFFERSARBPROC                      ) wglGetProcAddress( "glGenBuffers"                              ) ;
                OpenGL_Api::CheckError( "GetProcAddresses_misc" ) ;

                if( glGenBuffers )
                {	// IsExtensionSupported actually told the truth.
                    glMapBuffer     = (PFNGLMAPBUFFERARBPROC    ) wglGetProcAddress( "glMapBuffer"     );
                    glUnmapBuffer   = (PFNGLUNMAPBUFFERARBPROC  ) wglGetProcAddress( "glUnmapBuffer"   );
                    glBindBuffer    = (PFNGLBINDBUFFERARBPROC   ) wglGetProcAddress( "glBindBuffer"    );
                    glBufferData    = (PFNGLBUFFERDATAARBPROC   ) wglGetProcAddress( "glBufferData"    );
                    glDeleteBuffers = (PFNGLDELETEBUFFERSARBPROC) wglGetProcAddress( "glDeleteBuffers" );
                    OpenGL_Api::CheckError( "GetProcAddresses_glGenBuffers" ) ;
                }
            }




            /** Return whether the given OpenGL API extension is supported.
            */
            bool IsExtensionSupported( char* szTargetExtension )
            {
                PERF_BLOCK( OpenGL_Extensions__IsExtensionSupported ) ;

                const unsigned char *pszExtensions = NULL;
                const unsigned char *pszStart;
                unsigned char *pszWhere, *pszTerminator;

                // Extension names should not have spaces
                pszWhere = (unsigned char *) strchr( szTargetExtension, ' ' );
                if( pszWhere || *szTargetExtension == '\0' )
                    return false;

                // Get Extensions String
                pszExtensions = glGetString( GL_EXTENSIONS );
                OpenGL_Api::CheckError( "IsExtensionSupported" ) ;

                // Search The Extensions String For An Exact Copy
                pszStart = pszExtensions;
                for(;;)
                {
                    pszWhere = (unsigned char *) strstr( (const char *) pszStart, szTargetExtension );
                    if( !pszWhere )
                        break;
                    pszTerminator = pszWhere + strlen( szTargetExtension );
                    if( pszWhere == pszStart || *( pszWhere - 1 ) == ' ' )
                        if( *pszTerminator == ' ' || *pszTerminator == '\0' )
                            return true;
                    pszStart = pszTerminator;
                }
                return false;
            }




#if defined( _DEBUG )
            void PeGaSys_Render_OpenGL_Extensions_UnitTest()
            {
                DebugPrintf( "OpenGL_Extensions::UnitTest ----------------------------------------------\n" ) ;

                {
                    GetProcAddresses() ;
                }

                DebugPrintf( "OpenGL_Extensions::UnitTest: THE END ----------------------------------------------\n" ) ;
            }
#endif

        } ;
    } ;
} ;
