#include "ddViewManager.h"
#include "ddMacros.h"

#include <QTabWidget>
#include <QVBoxLayout>
#include <QPushButton>

#include <cstdio>

//-----------------------------------------------------------------------------
class ddViewManager::ddInternal
{
public:

  QTabWidget* TabWidget;
};


//-----------------------------------------------------------------------------
ddViewManager::ddViewManager(QWidget* parent) : QWidget(parent)
{
  this->Internal = new ddInternal;

  QVBoxLayout* layout = new QVBoxLayout(this);

  QTabWidget* tabWidget = new QTabWidget(this);
  tabWidget->setTabPosition(QTabWidget::West);
  layout->addWidget(tabWidget);
  this->Internal->TabWidget = tabWidget;

  this->addDefaultPage();
}

//-----------------------------------------------------------------------------
ddViewManager::~ddViewManager()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
QTabWidget* ddViewManager::tabWidget() const
{
  return this->Internal->TabWidget;
}

//-----------------------------------------------------------------------------
void ddViewManager::addDefaultPage()
{
  QPushButton* button = new QPushButton("hello world");
  this->tabWidget()->addTab(button, "Create View");
}
