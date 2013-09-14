#include "ddViewManager.h"
#include "ddGLWidgetView.h"
#include "ddQVTKWidgetView.h"
#include "ddDRCView.h"
#include "ddMacros.h"

#include <QTabWidget>
#include <QTabBar>
#include <QVBoxLayout>
#include <QMap>

#include <cstdio>

class MyTabWidget : public QTabWidget
{
public:

  MyTabWidget(QWidget* parent=0) : QTabWidget(parent)
  {
  }

  void updateTabBar()
  {
    this->tabBar()->setVisible(this->count() > 1);
  }
};

//-----------------------------------------------------------------------------
class ddViewManager::ddInternal
{
public:

  QTabWidget* TabWidget;

  QMap<QString, ddViewBase*> Views;
};


//-----------------------------------------------------------------------------
ddViewManager::ddViewManager(QWidget* parent) : QWidget(parent)
{
  this->Internal = new ddInternal;

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);

  QTabWidget* tabWidget = new MyTabWidget(this);
  tabWidget->setTabPosition(QTabWidget::West);
  tabWidget->setDocumentMode(true);
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
ddViewBase* ddViewManager::findView(const QString& viewName) const
{
  return this->Internal->Views.value(viewName);
}

//-----------------------------------------------------------------------------
void ddViewManager::addView(ddViewBase* view, const QString& viewName)
{
  this->tabWidget()->addTab(view, viewName);
  this->Internal->Views[viewName] = view;
  static_cast<MyTabWidget*>(this->tabWidget())->updateTabBar();
}

//-----------------------------------------------------------------------------
void ddViewManager::addDefaultPage()
{
  //this->addView(new ddGLWidgetView, "OpenGL View");
  //this->addView(new ddQVTKWidgetView, "VTK View");
  this->addView(new ddDRCView, "DRC View");
}
