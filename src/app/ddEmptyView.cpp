#include "ddEmptyView.h"

#include <QVBoxLayout>

//-----------------------------------------------------------------------------
class ddEmptyView::ddInternal
{
public:

  ddInternal()
  {

  }

};

//-----------------------------------------------------------------------------
ddEmptyView::ddEmptyView(QWidget* parent) : ddViewBase(parent)
{
  this->Internal = new ddInternal;

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);

  layout->addWidget(new QWidget);
}

//-----------------------------------------------------------------------------
ddEmptyView::~ddEmptyView()
{
  delete this->Internal;
}
