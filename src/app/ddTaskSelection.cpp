#include "ddTaskSelection.h"


#include "ui_ddTaskSelection.h"

#include <QToolButton>

//-----------------------------------------------------------------------------
class ddTaskSelection::ddInternal : public Ui::ddTaskSelection
{
public:

  QList<QToolButton*> Buttons;
};


//-----------------------------------------------------------------------------
ddTaskSelection::ddTaskSelection()
{
  this->Internal = new ddInternal;
  this->Internal->setupUi(this);

  this->Internal->Buttons << this->Internal->Task1Button
                          << this->Internal->Task2Button
                          << this->Internal->Task3Button
                          << this->Internal->Task4Button
                          << this->Internal->Task5Button
                          << this->Internal->Task6Button
                          << this->Internal->Task7Button
                          << this->Internal->Task8Button;

  foreach (QToolButton* button, this->Internal->Buttons)
  {
    this->connect(button, SIGNAL(clicked()), SLOT(onButtonClicked()));
  }
}

//-----------------------------------------------------------------------------
ddTaskSelection::~ddTaskSelection()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddTaskSelection::onButtonClicked()
{
  QToolButton* button = qobject_cast<QToolButton*>(this->sender());
  emit this->taskSelected(this->Internal->Buttons.indexOf(button));
}
