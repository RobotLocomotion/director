#ifndef __ddSharedPtr_h
#define __ddSharedPtr_h

#ifdef _WIN32
  #include <memory>
#else
  #include <tr1/memory>
#endif

#define ddSharedPtr std::tr1::shared_ptr
#define ddWeakPtr std::tr1::weak_ptr
#define ddPtrCast std::tr1::dynamic_pointer_cast

#define ddPtrMacro(className) \
  typedef ddSharedPtr<className> Ptr; \
  typedef const ddSharedPtr<className> ConstPtr;

#endif
