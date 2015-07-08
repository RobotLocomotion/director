#ifndef __ddObjectTree_h
#define __ddObjectTree_h

#include <QTreeWidget>
#include "ddAppConfigure.h"


class DD_APP_EXPORT ddObjectTree : public QTreeWidget
{
    Q_OBJECT

public:

  ddObjectTree(QWidget* parent=0);
  virtual ~ddObjectTree();

signals:

  void keyPressSignal(QKeyEvent* event);

protected slots:

  //void onShowContextMenu(const QPoint& pos);

protected:

  void keyPressEvent(QKeyEvent* event);

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddObjectTree);
};

#endif
