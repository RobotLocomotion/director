#ifndef __ddDRCView_h
#define __ddDRCView_h

#include "ddQVTKWidgetView.h"

class ddDrakeModel;

class ddDRCView : public ddQVTKWidgetView
{
    Q_OBJECT

public:

  ddDRCView(QWidget* parent=0);
  virtual ~ddDRCView();

  const QList<ddDrakeModel*>& models() const;

  ddDrakeModel* loadURDFModel(const QString& filename);
  void unloadModel(ddDrakeModel* model);
  void unloadModels();

protected:


  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddDRCView);
};

#endif
