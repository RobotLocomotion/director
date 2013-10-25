#include "ddLumberSelection.h"


#include "ui_ddLumberSelection.h"

#include <QToolButton>

//-----------------------------------------------------------------------------
class ddLumberSelection::ddInternal : public Ui::ddLumberSelection
{
public:

  QList<QToolButton*> Buttons;
};


//-----------------------------------------------------------------------------
ddLumberSelection::ddLumberSelection()
{
  this->Internal = new ddInternal;
  this->Internal->setupUi(this);

  this->Internal->Buttons << this->Internal->TwoByFourButton
                          << this->Internal->TwoBySixButton
                          << this->Internal->FourByFourButton;

  foreach (QToolButton* button, this->Internal->Buttons)
  {
    this->connect(button, SIGNAL(clicked()), SLOT(onButtonClicked()));
  }
}

//-----------------------------------------------------------------------------
ddLumberSelection::~ddLumberSelection()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddLumberSelection::onButtonClicked()
{
  QToolButton* button = qobject_cast<QToolButton*>(this->sender());
  emit this->lumberSelected(this->Internal->Buttons.indexOf(button));
}
