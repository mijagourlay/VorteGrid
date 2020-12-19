/** \file OpenGL_extensions.h

    \brief Utility routines and variables to obtain and use OpenGL extensions.

    \author Written and Copyright 2010-2014 MJG; All rights reserved.
*/
#ifndef PEGASYS_RENDER_OPENGL_EXTENSIONS_H
#define PEGASYS_RENDER_OPENGL_EXTENSIONS_H

#include <GL/glut.h>
#include "glext.h"

// Macros ----------------------------------------------------------------------
// Types -----------------------------------------------------------------------

#ifndef GL_ARB_debug_output

    #define GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB               0x8242
    #define GL_MAX_DEBUG_MESSAGE_LENGTH_ARB               0x9143
    #define GL_MAX_DEBUG_LOGGED_MESSAGES_ARB              0x9144
    #define GL_DEBUG_LOGGED_MESSAGES_ARB                  0x9145
    #define GL_DEBUG_NEXT_LOGGED_MESSAGE_LENGTH_ARB       0x8243
    #define GL_DEBUG_CALLBACK_FUNCTION_ARB                0x8244
    #define GL_DEBUG_CALLBACK_USER_PARAM_ARB              0x8245
    #define GL_DEBUG_SOURCE_API_ARB                       0x8246
    #define GL_DEBUG_SOURCE_WINDOW_SYSTEM_ARB             0x8247
    #define GL_DEBUG_SOURCE_SHADER_COMPILER_ARB           0x8248
    #define GL_DEBUG_SOURCE_THIRD_PARTY_ARB               0x8249
    #define GL_DEBUG_SOURCE_APPLICATION_ARB               0x824A
    #define GL_DEBUG_SOURCE_OTHER_ARB                     0x824B
    #define GL_DEBUG_TYPE_ERROR_ARB                       0x824C
    #define GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR_ARB         0x824D
    #define GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR_ARB          0x824E
    #define GL_DEBUG_TYPE_PORTABILITY_ARB                 0x824F
    #define GL_DEBUG_TYPE_PERFORMANCE_ARB                 0x8250
    #define GL_DEBUG_TYPE_OTHER_ARB                       0x8251
    #define GL_DEBUG_SEVERITY_HIGH_ARB                    0x9146
    #define GL_DEBUG_SEVERITY_MEDIUM_ARB                  0x9147
    #define GL_DEBUG_SEVERITY_LOW_ARB                     0x9148 

    #define APIENTRYP APIENTRY *

#endif

typedef void (APIENTRYP PFNGLDEBUGMESSAGECONTROLARBPROC) (unsigned int source, unsigned int type, unsigned int severity, int count, const unsigned int* ids, bool enabled);
typedef void (APIENTRYP PFNGLDEBUGMESSAGEINSERTARBPROC) (unsigned int source, unsigned int type,  unsigned int id, unsigned int severity, int length, const char* buf);
typedef void (APIENTRY *GLDEBUGPROCARB)(unsigned int source, unsigned int type, unsigned int id,  unsigned int severity, int length, const char* message, void* userParam);
typedef void (APIENTRYP PFNGLDEBUGMESSAGECALLBACKARBPROC) (GLDEBUGPROCARB callback,  void* userParam);
typedef unsigned int (APIENTRYP PFNGLGETDEBUGMESSAGELOGARBPROC) (unsigned int count, int bufsize, unsigned int* sources,unsigned int* types, unsigned int* ids,  unsigned int* severities, int* lengths, char* messageLog);

namespace PeGaSys
{
    namespace Render
    {
        namespace OpenGL_Extensions
        {
// Public functions ------------------------------------------------------------
            void GetProcAddresses() ;
            bool IsExtensionSupported( char * szTargetExtension ) ;
        } ;
    } ;
} ;

// Public variables ------------------------------------------------------------

extern PFNGLDEBUGMESSAGECONTROLARBPROC                  glDebugMessageControlARB                ;
extern PFNGLDEBUGMESSAGEINSERTARBPROC                   glDebugMessageInsertARB                 ;
extern PFNGLDEBUGMESSAGECALLBACKARBPROC                 glDebugMessageCallbackARB               ;
extern PFNGLGETDEBUGMESSAGELOGARBPROC                   glGetDebugMessageLogARB                 ;

// Framebuffer object
extern PFNGLGENFRAMEBUFFERSEXTPROC                      glGenFramebuffersEXT                     ; // FBO name generation procedure
extern PFNGLDELETEFRAMEBUFFERSEXTPROC                   glDeleteFramebuffersEXT                  ; // FBO deletion procedure
extern PFNGLBINDFRAMEBUFFEREXTPROC                      glBindFramebufferEXT                     ; // FBO bind procedure
extern PFNGLCHECKFRAMEBUFFERSTATUSEXTPROC               glCheckFramebufferStatusEXT              ; // FBO completeness test procedure
extern PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVEXTPROC  glGetFramebufferAttachmentParameterivEXT ; // return various FBO parameters
extern PFNGLGENERATEMIPMAPEXTPROC                       glGenerateMipmapEXT                      ; // FBO automatic mipmap generation procedure
extern PFNGLFRAMEBUFFERTEXTURE2DEXTPROC                 glFramebufferTexture2DEXT                ; // FBO texdture attachement procedure
extern PFNGLFRAMEBUFFERRENDERBUFFEREXTPROC              glFramebufferRenderbufferEXT             ; // FBO renderbuffer attachement procedure

// Renderbuffer object
extern PFNGLGENRENDERBUFFERSEXTPROC                     glGenRenderbuffersEXT                    ; // renderbuffer generation procedure
extern PFNGLDELETERENDERBUFFERSEXTPROC                  glDeleteRenderbuffersEXT                 ; // renderbuffer deletion procedure
extern PFNGLBINDRENDERBUFFEREXTPROC                     glBindRenderbufferEXT                    ; // renderbuffer bind procedure
extern PFNGLRENDERBUFFERSTORAGEEXTPROC                  glRenderbufferStorageEXT                 ; // renderbuffer memory allocation procedure
extern PFNGLGETRENDERBUFFERPARAMETERIVEXTPROC           glGetRenderbufferParameterivEXT          ; // return various renderbuffer parameters
extern PFNGLISRENDERBUFFEREXTPROC                       glIsRenderbufferEXT                      ; // determine renderbuffer object type

extern PFNGLFRAMEBUFFERTEXTUREEXTPROC                   glFramebufferTextureEXT                  ;

extern PFNGLDRAWBUFFERSPROC                             glDrawBuffersEXT                         ;

// VBO Extension Function Pointers
extern PFNGLGENBUFFERSARBPROC                           glGenBuffers                            ;   ///< VBO Name Generation Procedure
extern PFNGLMAPBUFFERARBPROC                            glMapBuffer                             ;   ///< VBO Map Buffer procedure
extern PFNGLUNMAPBUFFERARBPROC                          glUnmapBuffer                           ;   ///< VBO Unmap Buffer procedure
extern PFNGLBINDBUFFERARBPROC                           glBindBuffer                            ;   ///< VBO Bind Procedure
extern PFNGLBUFFERDATAARBPROC                           glBufferData                            ;   ///< VBO Data Loading Procedure
extern PFNGLDELETEBUFFERSARBPROC                        glDeleteBuffers                         ;   ///< VBO Deletion Procedure

#endif
