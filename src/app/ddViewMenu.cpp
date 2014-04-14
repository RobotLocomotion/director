/*=========================================================================

Program: ParaView
Module:    $RCSfile: ddViewMenu.cxx,v $

Copyright (c) 2005-2008 Sandia Corporation, Kitware Inc.
All rights reserved.

ParaView is a free software; you can redistribute it and/or modify it
under the terms of the ParaView license version 1.2. 

See License_v1.2.txt for the full ParaView license.
A copy of this license can be obtained by contacting
Kitware Inc.
28 Corporate Drive
Clifton Park, NY 12065
USA

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

========================================================================*/


#include "ddViewMenu.h"

#include <QAction>
#include <QDockWidget>
#include <QEvent>
#include <QIcon>
#include <QMap>
#include <QMenu>
#include <QMenuBar>
#include <QString>
#include <QToolBar>
#include <QtDebug>


class ddViewMenu::pqImplementation
{
public:
  pqImplementation(QMenu& menu) :
    Menu(menu)
    {
    }
  
  ~pqImplementation()
    {
    }

  QMenu& Menu;
  typedef QMap<QWidget*, QAction*> ActionMapT;
  ActionMapT ActionMap;
};

//-----------------------------------------------------------------------------
ddViewMenu::ddViewMenu(QMenu& menu, QObject* p) :
  QObject(p), Implementation(new pqImplementation(menu))
{
}

ddViewMenu::~ddViewMenu()
{
  delete this->Implementation;
}

bool ddViewMenu::eventFilter(QObject* watched, QEvent* e)
{
  if(e->type() == QEvent::Hide || e->type() == QEvent::Show)
    {
    if(QWidget* const widget = qobject_cast<QWidget*>(watched))
      {
      pqImplementation::ActionMapT::Iterator iter =
          this->Implementation->ActionMap.find(widget);
      if(iter != this->Implementation->ActionMap.end())
        {
        (*iter)->setChecked(e->type() == QEvent::Show);
        }
      }
    }

  return QObject::eventFilter(watched, e);
}

void ddViewMenu::addWidget(QWidget* widget, QAction* action)
{
  if (this->Implementation->ActionMap.contains(widget))
    {
    qCritical() << "can't add widget twice";
    return;
    }

  if (!widget)
    {
    qCritical() << "null widget";
    return;
    }

  if (!action)
    {
    qCritical() << "null action";
    return;
    }

  action->setCheckable(true);
  action->setChecked(widget->isVisible());

  this->connect(action, SIGNAL(triggered(bool)), widget, SLOT(setVisible(bool)));
  this->Implementation->ActionMap.insert(widget, action);
  widget->installEventFilter(this);
  this->Implementation->Menu.addAction(action);
}


void ddViewMenu::addWidget(QWidget* widget, const QString& text, const QIcon& icon)
{
  if(this->Implementation->ActionMap.contains(widget))
    {
    qCritical() << "can't add widget twice";
    return;
    }

  if(!widget)
    {
    qCritical() << "null widget";
    return;
    }

  QAction* const action = new QAction(text, this);
  if(!icon.isNull())
    {
    action->setIcon(icon);
    }

  this->addWidget(widget, action);
}

void ddViewMenu::addSeparator()
{
  this->Implementation->Menu.addSeparator();
}

void ddViewMenu::removeWidget(QWidget* widget)
{
  if(!this->Implementation->ActionMap.contains(widget))
    {
    return;
    }

  widget->removeEventFilter(this);

  QAction *action = this->Implementation->ActionMap[widget];

  this->Implementation->ActionMap.erase(
    this->Implementation->ActionMap.find(widget));
  this->Implementation->Menu.removeAction(action);

  delete action;
}

int ddViewMenu::getNumberOfWidgets() const
{
  return this->Implementation->ActionMap.size();
}
