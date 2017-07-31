#ifndef __ddPythonUndoCommand_h
#define __ddPythonUndoCommand_h

#include <QUndoCommand>
#include <PythonQt.h>
#include "ddAppConfigure.h"

class QUndoStack;


class DD_APP_EXPORT ddPythonUndoCommand : public QObject, public QUndoCommand
{
    Q_OBJECT

public:

  ddPythonUndoCommand(const QString& text,
                      PythonQtObjectPtr undoFunction,
                      PythonQtObjectPtr redoFunction,
                      PythonQtObjectPtr userData,
                      PythonQtObjectPtr mergeFunction=nullptr,
                      int id=-1,
                      QUndoCommand* parent=nullptr);

  virtual ~ddPythonUndoCommand();

  PythonQtObjectPtr userData() const;

  void push(QUndoStack* stack);

  virtual bool mergeWith(const QUndoCommand* command);
  virtual void undo();
  virtual void redo();
  virtual int id() const;

protected:

  int Id;
  PythonQtObjectPtr UndoFunction;
  PythonQtObjectPtr RedoFunction;
  PythonQtObjectPtr UserData;
  PythonQtObjectPtr MergeFunction;

  Q_DISABLE_COPY(ddPythonUndoCommand);
};

#endif
