#ifndef __ddMacrosManager_h
#define __ddMacrosManager_h

#include <QObject>
#include <QStringList>
#include <QIcon>
#include "ddAppConfigure.h"


class QToolBar;

class DD_APP_EXPORT ddMacrosManager : public QObject
{
    Q_OBJECT

public:

  ddMacrosManager(QObject* parent=0);
  virtual ~ddMacrosManager();

  void setToolBar(QToolBar* toolBar);
  void addPath(const QString& path);
  void removePath(const QString& path);

  static QStringList findMacrosInPath(const QString& path);
  static QIcon findIconForMacro(const QString& macroFileName);

signals:

  void executeScriptRequested(const QString& filename);

protected slots:

  void onMacroActivated();

protected:

  void updateToolBar();
  void clearToolBar();
  void addMacroToToolBar(const QString& filename);

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddMacrosManager);

};

#endif
