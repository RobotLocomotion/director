#include "ddDRCView.h"
#include "ddDrakeModel.h"

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkAxesActor.h>
#include <QVBoxLayout>

//-----------------------------------------------------------------------------
class ddDRCView::ddInternal
{
public:

  ddInternal()
  {
  }

  QList<ddDrakeModel*> Models;
};


//-----------------------------------------------------------------------------
ddDRCView::ddDRCView(QWidget* parent) : ddQVTKWidgetView(parent)
{
  this->Internal = new ddInternal;
}

//-----------------------------------------------------------------------------
ddDRCView::~ddDRCView()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
const QList<ddDrakeModel*>& ddDRCView::models() const
{
  return this->Internal->Models;
}

//-----------------------------------------------------------------------------
void ddDRCView::unloadModels()
{
  foreach (ddDrakeModel* model, this->models())
  {
    this->unloadModel(model);
  }
}

//-----------------------------------------------------------------------------
void ddDRCView::unloadModel(ddDrakeModel* model)
{
  if (!model)
  {
    return;
  }

  model->removeFromRenderer(this->renderer());
  this->Internal->Models.removeAll(model);
  delete model;
}

//-----------------------------------------------------------------------------
ddDrakeModel* ddDRCView::loadURDFModel(const QString& filename)
{
  ddDrakeModel* model = new ddDrakeModel(this);

  if (!model->loadFromFile(filename))
  {
    delete model;
    return 0;
  }

  model->addToRenderer(this->renderer());
  this->Internal->Models.append(model);
  this->render();

  this->connect(model, SIGNAL(modelChanged()), SLOT(render()));
  return model;
}
