#ifndef __ddSharedPtr_h
#define __ddSharedPtr_h

#include <memory>

#define ddSharedPtr std::shared_ptr
#define ddWeakPtr std::weak_ptr
#define ddPtrCast std::dynamic_pointer_cast

#define ddPtrMacro(className) \
  typedef ddSharedPtr<className> Ptr; \
  typedef const ddSharedPtr<className> ConstPtr;

#endif
