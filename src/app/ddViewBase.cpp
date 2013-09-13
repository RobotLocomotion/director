#include "ddViewBase.h"


//-----------------------------------------------------------------------------
class ddViewBase::ddInternal
{
public:

};


//-----------------------------------------------------------------------------
ddViewBase::ddViewBase(QWidget* parent) : QWidget(parent)
{
  this->Internal = new ddInternal;
}

//-----------------------------------------------------------------------------
ddViewBase::~ddViewBase()
{
  delete this->Internal;
}
