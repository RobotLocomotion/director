#include "ddObjectTree.h"

#include <QKeyEvent>

//-----------------------------------------------------------------------------
class ddObjectTree::ddInternal
{
public:

  ddInternal()
  {

  }

};

//-----------------------------------------------------------------------------
ddObjectTree::ddObjectTree(QWidget* parent) : QTreeWidget(parent)
{
  this->Internal = new ddInternal;


  this->setContextMenuPolicy(Qt::CustomContextMenu);
  //this->connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),
  //    SLOT(onShowContextMenu(const QPoint&)));
}

//-----------------------------------------------------------------------------
ddObjectTree::~ddObjectTree()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddObjectTree::keyPressEvent(QKeyEvent* event)
{
  event->setAccepted(false);

  emit this->keyPressSignal(event);
  if (!event->isAccepted())
  {
    QTreeWidget::keyPressEvent(event);
  }
}

