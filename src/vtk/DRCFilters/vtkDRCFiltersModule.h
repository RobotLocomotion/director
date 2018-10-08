#ifndef __vtkDRCFiltersModel_h
#define __vtkDRCFiltersModel_h

#if defined(WIN32)
#  if defined(vtkDRCFilters_EXPORTS)
#    define VTKDRCFILTERS_EXPORT __declspec( dllexport )
#  else
#    define VTKDRCFILTERS_EXPORT __declspec( dllimport )
#  endif
#else
#  define VTKDRCFILTERS_EXPORT
#endif

#if __cplusplus >= 201103L
# define VTKDRCFILTERS_DELETE_FUNCTION =delete
# define VTKDRCFILTERS_OVERRIDE override
#else
# define VTKDRCFILTERS_DELETE_FUNCTION
# define VTKDRCFILTERS_OVERRIDE
#endif

#endif
