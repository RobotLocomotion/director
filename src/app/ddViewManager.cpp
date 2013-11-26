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

  ddInternal()
  {
    this->CurrentTabIndex = -1;
  }

  int CurrentTabIndex;

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

  this->connect(this->Internal->TabWidget, SIGNAL(currentChanged(int)), SLOT(onCurrentTabChanged(int)));

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
QString ddViewManager::viewName(ddViewBase* view)
{
  return this->Internal->Views.key(view);
}

//-----------------------------------------------------------------------------
void ddViewManager::switchToView(const QString& viewName)
{
  for (int i = 0; i < this->Internal->TabWidget->count(); ++i)
  {
    if (this->Internal->TabWidget->tabText(i) == viewName)
    {
      this->Internal->TabWidget->setCurrentIndex(i);
      return;
    }
  }
}

//-----------------------------------------------------------------------------
ddViewBase* ddViewManager::currentView() const
{
  return this->findView(this->Internal->TabWidget->tabText(this->Internal->TabWidget->currentIndex()));
}

//-----------------------------------------------------------------------------
void ddViewManager::onCurrentTabChanged(int currentIndex)
{
  ddViewBase* previousView = 0;
  if (this->Internal->CurrentTabIndex >= 0)
  {
    previousView = this->findView(this->Internal->TabWidget->tabText(this->Internal->CurrentTabIndex));
  }
  this->Internal->CurrentTabIndex = currentIndex;
  emit this->currentViewChanged(previousView, this->currentView());
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
ddViewBase* ddViewManager::createView(const QString& viewName, const QString& viewType, int pageIndex)
{
  ddViewBase* view = 0;
  if (viewName == "VTK View")
  {
    view = new ddQVTKWidgetView;
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
  this->addView(new ddQVTKWidgetView, "DRC View");
}
