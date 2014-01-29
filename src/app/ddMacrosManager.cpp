#include "ddMacrosManager.h"

#include <QAction>
#include <QDir>
#include <QToolBar>
#include <QMap>
#include <QPointer>


//-----------------------------------------------------------------------------
class ddMacrosManager::ddInternal
{
public:

  QPointer<QToolBar> ToolBar;
  QStringList Paths;
  QMap<QAction*, QString> Actions;
};


//-----------------------------------------------------------------------------
ddMacrosManager::ddMacrosManager(QObject* parent) : QObject(parent)
{
  this->Internal = new ddInternal;
}

//-----------------------------------------------------------------------------
ddMacrosManager::~ddMacrosManager()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddMacrosManager::setToolBar(QToolBar* toolBar)
{
  this->clearToolBar();
  this->Internal->ToolBar = toolBar;
  this->updateToolBar();
}

//-----------------------------------------------------------------------------
void ddMacrosManager::addPath(const QString& path)
{
  if (!path.length())
  {
    return;
  }

  this->Internal->Paths.append(path);
  this->updateToolBar();
}

//-----------------------------------------------------------------------------
void ddMacrosManager::removePath(const QString& path)
{
  this->Internal->Paths.removeAll(path);
  this->updateToolBar();
}

//-----------------------------------------------------------------------------
void ddMacrosManager::updateToolBar()
{
  this->clearToolBar();

  foreach (const QString& path, this->Internal->Paths)
  {
    QStringList filenames = this->findMacrosInPath(path);
    foreach (const QString& filename, filenames)
    {
      this->addMacroToToolBar(QFileInfo(path, filename).canonicalFilePath());
    }
  }
}

//-----------------------------------------------------------------------------
void ddMacrosManager::clearToolBar()
{
  if (this->Internal->ToolBar)
  {
    this->Internal->ToolBar->clear();
  }

  this->Internal->Actions.clear();
}

//-----------------------------------------------------------------------------
void ddMacrosManager::addMacroToToolBar(const QString& filename)
{
  if (!this->Internal->ToolBar)
  {
    return;
  }

  QIcon icon = this->findIconForMacro(filename);
  QString name = QFileInfo(filename).baseName();

  QAction* action = this->Internal->ToolBar->addAction(icon, name);
  this->connect(action, SIGNAL(triggered()), SLOT(onMacroActivated()));
  this->Internal->Actions[action] = filename;
}

//-----------------------------------------------------------------------------
QStringList ddMacrosManager::findMacrosInPath(const QString& path)
{
  QStringList filters;
  filters << "*.py";
  return QDir(path).entryList(filters);
}

//-----------------------------------------------------------------------------
QIcon ddMacrosManager::findIconForMacro(const QString& macroFileName)
{
  QFileInfo macroFile = QFileInfo(macroFileName);

  QStringList extensions;
  extensions << ".jpg" << ".png";

  foreach (const QString& extension, extensions)
  {
    QFileInfo iconFile = QFileInfo(macroFile.dir(), macroFile.baseName() + extension);
    if (iconFile.exists())
    {
      return QIcon(iconFile.canonicalFilePath());
    }
  }

  return QIcon();
}

//-----------------------------------------------------------------------------
void ddMacrosManager::onMacroActivated()
{
  QAction* action = qobject_cast<QAction*>(this->sender());
  if (!action)
  {
    return;
  }

  QString filename = this->Internal->Actions.value(action);
  if (filename.length() && QFileInfo(filename).exists())
  {
    emit this->executeScriptRequested(filename);
  }
}
