/*=========================================================================

Program: ParaView
Module:    $RCSfile: ddViewMenu.h,v $

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

#ifndef __ddViewMenu_h
#define __ddViewMenu_h

#include <QObject>
#include <QIcon>
#include "ddAppConfigure.h"

class QAction;
class QEvent;
class QIcon;
class QMenu;
class QString;

/// Manages a menu containing a collection of widgets that can be shown/hidden
class DD_APP_EXPORT ddViewMenu : public QObject
{
  Q_OBJECT

public:
  ddViewMenu(QMenu& menu, QObject* p=0);
  ~ddViewMenu();

  /// \brief
  ///   Used to watch for show and hide events.
  ///
  /// The show and hide events for widgets are used
  /// to update the action associated with them in the view menu.
  ///
  /// \param watched The widget receiving the event.
  /// \param e The event that is about to be sent.
  /// \return
  ///   True if the event should be filtered out.
  virtual bool eventFilter(QObject* watched, QEvent* e);

  /// \brief
  ///   Adds a menu item for the widget to the view menu.
  ///
  /// Pass in a null icon if there is no icon for the widget. The
  /// text can be the window title for the dock window. The text can
  /// also include a menu shortcut key.
  ///
  /// \param widget The widget to add.
  /// \param icon An icon to display in the menu.
  /// \param text The text to display in the menu.
  void addWidget(QWidget* widget, const QString& text, const QIcon &icon = QIcon());

  void addWidget(QWidget* widget, QAction* action);

  /// Add a separator to the view menu
  void addSeparator();

  /// \brief
  ///   Removes the widget from the view menu.
  /// \param widget The widget to remove.
  void removeWidget(QWidget* widget);

  int getNumberOfWidgets() const;
private:
  class pqImplementation;
  pqImplementation* const Implementation;
};

#endif
