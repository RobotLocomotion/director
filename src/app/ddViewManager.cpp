#include "ddViewManager.h"
#include "ddGLWidgetView.h"
#include "ddQVTKWidgetView.h"
#include "ddDRCView.h"
#include "ddSpreadsheetView.h"
#include "ddMacros.h"

#include <QTabWidget>
#include <QTabBar>
#include <QVBoxLayout>
#include <QMap>
#include <QSplitter>

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

  QList<QSplitter*> Splitters;
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
void ddViewManager::addView(ddViewBase* view, const QString& viewName, int pageIndex)
{
  QSplitter* splitter = 0;
  if (pageIndex >= 0 && pageIndex < this->Internal->Splitters.length())
  {
    splitter = this->Internal->Splitters[pageIndex];
  }
  else
  {
    splitter = new QSplitter();
    this->Internal->Splitters.append(splitter);
    this->tabWidget()->addTab(splitter, viewName);
  }

  splitter->addWidget(view);


  this->Internal->Views[viewName] = view;
  static_cast<MyTabWidget*>(this->tabWidget())->updateTabBar();
}

//-----------------------------------------------------------------------------
ddViewBase* ddViewManager::createView(const QString& viewName, int pageIndex)
{
  ddViewBase* view = 0;
  if (viewName == "DRC View")
  {
    view = new ddDRCView;
  }
  else if (viewName == "Spreadsheet View")
  {
    view = new ddSpreadsheetView;
  }

  if (view)
  {
    this->addView(view, viewName, pageIndex);
  }

  return view;
}

//-----------------------------------------------------------------------------
void ddViewManager::addDefaultPage()
{
  //this->addView(new ddGLWidgetView, "OpenGL View");
  //this->addView(new ddQVTKWidgetView, "VTK View");
  this->addView(new ddDRCView, "DRC View");
  this->addView(new ddSpreadsheetView, "Spreadsheet View");

  //this->addView(new ddDRCView, "DRC View 2", 0);
  //QList<int> sizes;
  //sizes << 1 << 0;
  //this->Internal->Splitters[0]->setSizes(sizes);
}
