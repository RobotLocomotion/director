#ifndef __directorConfigure_h
#define __directorConfigure_h

#if defined(WIN32)
#  if defined(director_EXPORTS)
#    define DD_APP_EXPORT __declspec( dllexport )
#  else
#    define DD_APP_EXPORT __declspec( dllimport )
#  endif
#else
#  define DD_APP_EXPORT
#endif

#endif
