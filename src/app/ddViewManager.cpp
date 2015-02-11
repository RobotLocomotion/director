#include "ddViewManager.h"
#include "ddGLWidgetView.h"
#include "ddQVTKWidgetView.h"
#include "ddSpreadsheetView.h"
#include "ddMacros.h"

#include <QTabWidget>
#include <QTabBar>
#include <QVBoxLayout>
#include <QMap>
#include <QSplitter>
#include <QEvent>

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
  QMap<ddViewBase*, int> PageIndexCache;
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
  tabWidget->setMovable(true);
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
void ddViewManager::popOut(ddViewBase* view)
{
  QSplitter* splitter = qobject_cast<QSplitter*>(view->parent());
  int pageIndex = this->Internal->TabWidget->indexOf(splitter);
  if (pageIndex < 0)
  {
    return;
  }

  view->setParent(0);
  view->show();
  view->installEventFilter(this);
  this->Internal->TabWidget->removeTab(pageIndex);
  this->Internal->PageIndexCache[view] = pageIndex;
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
  QSplitter* splitter = splitter = new QSplitter();
  this->Internal->Splitters.append(splitter);

  if (pageIndex >= 0)
  {
    this->tabWidget()->insertTab(pageIndex, splitter, viewName);
  }
  else
  {
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
  if (viewType == "VTK View")
  {
    view = new ddQVTKWidgetView;
  }
  else if (viewType == "Spreadsheet View")
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

//-----------------------------------------------------------------------------
bool ddViewManager::eventFilter(QObject* obj, QEvent* event)
{
  ddViewBase* view = qobject_cast<ddViewBase*>(obj);

  if (view && event->type() == QEvent::Close)
  {
    view->removeEventFilter(this);
    QString viewName = this->Internal->Views.key(view);
    int pageIndex = this->Internal->PageIndexCache[view];
    this->addView(view, viewName, pageIndex);
    this->Internal->TabWidget->setCurrentIndex(pageIndex);
    event->ignore();
    return true;
  }
  else
  {
    return QWidget::eventFilter(obj, event);
  }
}
