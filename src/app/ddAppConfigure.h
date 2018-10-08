#ifndef __ddAppConfigure_h
#define __ddAppConfigure_h

#if defined(WIN32)
#  if defined(ddApp_EXPORTS)
#    define DD_APP_EXPORT __declspec( dllexport )
#  else
#    define DD_APP_EXPORT __declspec( dllimport )
#  endif
#else
#  define DD_APP_EXPORT
#endif

#if __cplusplus >= 201103L
# define DD_APP_DELETE_FUNCTION =delete
# define DD_APP_OVERRIDE override
#else
# define DD_APP_DELETE_FUNCTION
# define DD_APP_OVERRIDE
#endif

#endif
