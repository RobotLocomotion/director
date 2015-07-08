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

#endif
