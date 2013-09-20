#include "ddSpreadsheetView.h"

#include <QVBoxLayout>
#include <QStandardItemModel>
#include <QTableView>
#include <QKeyEvent>
#include <QClipboard>
#include <QItemSelectionModel>
#include <QAction>
#include <QApplication>


//-----------------------------------------------------------------------------
class ddSpreadsheetView::ddInternal
{
public:

  ddInternal()
  {
    this->Model = 0;
    this->TableView = 0;
    this->ClearAction = 0;
  }

  QStandardItemModel*   Model;
  QTableView*           TableView;
  QAction*              ClearAction;

};


//-----------------------------------------------------------------------------
ddSpreadsheetView::ddSpreadsheetView(QWidget* parent) : ddViewBase(parent)
{
  this->Internal = new ddInternal;


  this->Internal->Model = new QStandardItemModel(this);

  this->Internal->TableView = new QTableView();

  //this->Internal->TableView->setSortingEnabled(false);

  this->Internal->ClearAction = new QAction("Clear table", this);

  this->Internal->TableView->installEventFilter(this);


  this->Internal->TableView->setModel(this->Internal->Model);

  //this->connect(this->Internal->Model, SIGNAL(rowsInserted(const QModelIndex &, int, int)),
  //              this->Internal->TableView, SLOT(scrollToBottom()));


  this->Internal->TableView->setSelectionBehavior(QAbstractItemView::SelectItems);
  this->Internal->TableView->setSelectionMode(QAbstractItemView::ExtendedSelection);

  this->connect(this->Internal->ClearAction, SIGNAL(triggered()), SLOT(clear()));


  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);
  layout->addWidget(this->Internal->TableView);


  // init cells
  int rowCount = 50;
  int columnCount = 26;

  for (int row = 0; row < rowCount; ++row)
  {
    QStringList rowData;
    for (int column = 0; column < columnCount; ++column)
    {
      rowData << QString();
    }

    this->appendRow(rowData);

    for (int column = 0; column < columnCount; ++column)
    {
      this->Internal->Model->item(row, column)->setEditable(true);
    }
  }
}

//-----------------------------------------------------------------------------
ddSpreadsheetView::~ddSpreadsheetView()
{
  delete this->Internal;
}


//-----------------------------------------------------------------------------
namespace {

// Convert 0 thru 25 into A thru Z.
QString IndexToLetter(int value)
{
  return QString(QChar(65 + value));
}

// Convert index into letter code: A thru Z, AA thru AZ, etc.
QString Base26ColumnName(int value)
{
  int base = value / 26;
  int remainder = value % 26;
  if (base == 0) return IndexToLetter(remainder);
  else return Base26ColumnName(base) + IndexToLetter(remainder);
}

QString GetColumnName(int index)
{
  return Base26ColumnName(index);
}

}

//-----------------------------------------------------------------------------
void ddSpreadsheetView::appendRow(const QStringList& rowData)
{
  int numberOfColumns = rowData.length();

  // Increase the column count if needed
  for (int i = this->Internal->Model->columnCount(); i < numberOfColumns; ++i)
    {
    QString columnName = GetColumnName(i);
    this->Internal->Model->setHorizontalHeaderItem(i, new QStandardItem(columnName));
    }

  QList<QStandardItem*> items;
  foreach (const QString& data, rowData)
    {
    items << new QStandardItem(data);
    items.back()->setEditable(false);
    }

  this->Internal->Model->appendRow(items);
}

//-----------------------------------------------------------------------------
QAbstractItemModel* ddSpreadsheetView::model()
{
  return this->Internal->Model;
}

//-----------------------------------------------------------------------------
QString ddSpreadsheetView::headerString(QString sep) const
{
  QStringList headerData;
  for (int i = 0; i < this->columnCount(); ++i)
    {
    headerData << this->Internal->Model->horizontalHeaderItem(i)->text();
    }
  return headerData.join(sep);
}

//-----------------------------------------------------------------------------
QString ddSpreadsheetView::rowString(int row, QString sep) const
{
  return this->rowData(row).join(sep);
}

//-----------------------------------------------------------------------------
QStringList ddSpreadsheetView::rowData(int row) const
{
  QStringList rowData;
  if (row >= 0 && row < this->rowCount())
    {
    for (int column = 0; column < this->columnCount(); ++column)
      {
      QStandardItem* item = this->Internal->Model->item(row, column);
      if (item)
        {
        rowData << item->text();
        }
      else
        {
        rowData << QString();
        }
      }
    }
  return rowData;
}

//-----------------------------------------------------------------------------
int ddSpreadsheetView::rowCount() const
{
  return this->Internal->Model->rowCount();
}

//-----------------------------------------------------------------------------
int ddSpreadsheetView::columnCount() const
{
  return this->Internal->Model->columnCount();
}

//-----------------------------------------------------------------------------
void ddSpreadsheetView::removeRow(int index)
{
  this->Internal->Model->removeRow(index);
}

//-----------------------------------------------------------------------------
void ddSpreadsheetView::clear()
{
  this->Internal->Model->clear();
}


//-----------------------------------------------------------------------------
QString ddSpreadsheetView::textFromIndexes(QList<QModelIndex> indexes)
{
  QString text;
  if (indexes.length())
    {
    qSort(indexes);
    QStringList rowData;
    int currentRow = indexes[0].row();
    foreach (QModelIndex index, indexes)
      {
      if (index.row() != currentRow)
        {
        currentRow = index.row();
        text += rowData.join("\t") + "\n";
        rowData.clear();
        }
      rowData << index.data().toString();
      }
    text += rowData.join("\t");
    }
  return text;
}

//-----------------------------------------------------------------------------
void ddSpreadsheetView::addSelectedCellsToClipboard()
{
  QString text = this->textFromIndexes(this->Internal->TableView->selectionModel()->selectedIndexes());
  QApplication::clipboard()->setText(text);
}

//-----------------------------------------------------------------------------
bool ddSpreadsheetView::eventFilter(QObject* obj, QEvent* event)
{
  if (obj != this->Internal->TableView)
    {
    return false;
    }

  if (event->type() == QEvent::KeyPress)
    {
    QKeyEvent* e = static_cast<QKeyEvent*>(event);
    if (e->key() == Qt::Key_C && (e->modifiers() & Qt::ControlModifier))
      {
      this->addSelectedCellsToClipboard();
      return true;
      }
    }
  return false;
}
