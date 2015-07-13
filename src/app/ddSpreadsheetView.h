#ifndef __ddSpreadsheetView_h
#define __ddSpreadsheetView_h

#include "ddViewBase.h"
#include "ddAppConfigure.h"


class QAbstractItemModel;
class QModelIndex;

class DD_APP_EXPORT ddSpreadsheetView : public ddViewBase
{
    Q_OBJECT

public:

  ddSpreadsheetView(QWidget* parent=0);
  virtual ~ddSpreadsheetView();


  QAbstractItemModel* model();

  QString headerString(QString sep=", ") const;
  QString rowString(int row, QString sep=", ") const;
  QStringList rowData(int index) const;
  int rowCount() const;
  int columnCount() const;

  void appendRow(const QStringList& rowData);
  void removeRow(int index);

  virtual bool eventFilter(QObject* obj, QEvent* event);

public slots:

  void clear();


protected:

  void addSelectedCellsToClipboard();

  QString textFromIndexes(QList<QModelIndex> indexes);

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddSpreadsheetView);
};

#endif
