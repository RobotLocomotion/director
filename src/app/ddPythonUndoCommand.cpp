#include "ddPythonUndoCommand.h"
#include <QUndoStack>

//-----------------------------------------------------------------------------
ddPythonUndoCommand::ddPythonUndoCommand(const QString& text,
                                         PythonQtObjectPtr undoFunction,
                                         PythonQtObjectPtr redoFunction,
                                         PythonQtObjectPtr userData,
                                         PythonQtObjectPtr mergeFunction,
                                         int id,
                                         QUndoCommand* parent) : QUndoCommand(text, parent)
{
  this->Id = id;
  this->UndoFunction = undoFunction;
  this->RedoFunction = redoFunction;
  this->UserData = userData;
  this->MergeFunction = mergeFunction;
}

//-----------------------------------------------------------------------------
ddPythonUndoCommand::~ddPythonUndoCommand()
{
}

//-----------------------------------------------------------------------------
int ddPythonUndoCommand::id() const
{
  return this->Id;
}

//-----------------------------------------------------------------------------
PythonQtObjectPtr ddPythonUndoCommand::userData() const
{
  return this->UserData;
}

//-----------------------------------------------------------------------------
void ddPythonUndoCommand::undo()
{
  if (this->UndoFunction)
  {
    this->UndoFunction.call();
  }
}

//-----------------------------------------------------------------------------
void ddPythonUndoCommand::redo()
{
  if (this->RedoFunction)
  {
    this->RedoFunction.call();
  }
}

//-----------------------------------------------------------------------------
Q_DECLARE_METATYPE(const ddPythonUndoCommand*);

//-----------------------------------------------------------------------------
bool ddPythonUndoCommand::mergeWith(const QUndoCommand* command)
{
  if (this->MergeFunction)
  {
    QVariantList args;
    const ddPythonUndoCommand* undoCommand = dynamic_cast<const ddPythonUndoCommand*>(command);
    args << QVariant::fromValue<const ddPythonUndoCommand*>(undoCommand);
    QVariant result = this->MergeFunction.call(args);
    return result.toBool();
  }
  return false;
}

//-----------------------------------------------------------------------------
void ddPythonUndoCommand::push(QUndoStack* stack)
{
  if (stack)
  {
    stack->push(this);
  }
}
