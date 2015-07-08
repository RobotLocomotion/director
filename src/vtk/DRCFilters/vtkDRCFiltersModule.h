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

#endif
