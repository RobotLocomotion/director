#include "ddPropertiesPanel.h"

#include <QtVariantPropertyManager>
#include <QtTreePropertyBrowser>
#include <QtVariantEditorFactory>


#include <QVBoxLayout>

//-----------------------------------------------------------------------------
class ddPropertiesPanel::ddInternal
{
public:

  QtVariantPropertyManager* Manager;
  QtTreePropertyBrowser* Browser;
};


//-----------------------------------------------------------------------------
ddPropertiesPanel::ddPropertiesPanel(QWidget* parent) : QWidget(parent)
{
  this->Internal = new ddInternal;

  QtVariantPropertyManager *manager = new QtVariantPropertyManager;
  this->Internal->Manager = manager;

  QtTreePropertyBrowser *browser = new QtTreePropertyBrowser;
  this->Internal->Browser = browser;

  QtVariantEditorFactory *variantFactory = new QtVariantEditorFactory;
  browser->setFactoryForManager(manager, variantFactory);
  browser->setPropertiesWithoutValueMarked(true);
  browser->setRootIsDecorated(false);

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);
  layout->addWidget(this->Internal->Browser);


  // add properties

  QtVariantProperty *item = manager->addProperty(QVariant::Bool, "Bool property");
  item->setValue(true);
  browser->addProperty(item);

  item = manager->addProperty(QVariant::Int, "Int property");
  item->setValue(20);
  item->setAttribute(QLatin1String("minimum"), 0);
  item->setAttribute(QLatin1String("maximum"), 10000);
  item->setAttribute(QLatin1String("singleStep"), 1);
  browser->addProperty(item);

  item = manager->addProperty(QVariant::Double, "Double property");
  item->setValue(1.2345);
  item->setAttribute(QLatin1String("singleStep"), 0.1);
  item->setAttribute(QLatin1String("decimals"), 3);
  browser->addProperty(item);

  item = manager->addProperty(QVariant::String, "String property");
  item->setValue("my str");
  browser->addProperty(item);

}

//-----------------------------------------------------------------------------
ddPropertiesPanel::~ddPropertiesPanel()
{
  delete this->Internal;
}
