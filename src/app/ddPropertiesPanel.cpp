#include "ddPropertiesPanel.h"

#include <QtVariantPropertyManager>
#include <QtTreePropertyBrowser>
#include <QtGroupBoxPropertyBrowser>
#include <QtVariantEditorFactory>


#include <QVBoxLayout>

//-----------------------------------------------------------------------------
class ddPropertiesPanel::ddInternal
{
public:

  QtVariantPropertyManager* Manager;
  //QtAbstractPropertyBrowser* Browser;
  QtTreePropertyBrowser* Browser;

  QMap<QString, QtVariantProperty*> Properties;
};


//-----------------------------------------------------------------------------
ddPropertiesPanel::ddPropertiesPanel(QWidget* parent) : QWidget(parent)
{
  this->Internal = new ddInternal;

  QtVariantPropertyManager *manager = new QtVariantPropertyManager;
  this->Internal->Manager = manager;

  QtTreePropertyBrowser *browser = new QtTreePropertyBrowser;
  //QtGroupBoxPropertyBrowser* browser = new QtGroupBoxPropertyBrowser;
  this->Internal->Browser = browser;

  QtVariantEditorFactory *variantFactory = new QtVariantEditorFactory;
  browser->setFactoryForManager(manager, variantFactory);
  browser->setPropertiesWithoutValueMarked(true);
  browser->setRootIsDecorated(true);


  this->connect(this->Internal->Manager,
      SIGNAL(valueChanged(QtProperty*, const QVariant&)),
      SLOT(onPropertyValueChanged(QtProperty*)));

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);
  layout->addWidget(this->Internal->Browser);
}

//-----------------------------------------------------------------------------
ddPropertiesPanel::~ddPropertiesPanel()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddPropertiesPanel::onPropertyValueChanged(QtProperty* property)
{
  emit this->propertyValueChanged(static_cast<QtVariantProperty*>(property));
}

//-----------------------------------------------------------------------------
QtVariantPropertyManager* ddPropertiesPanel::propertyManager() const
{
  return this->Internal->Manager;
}

//-----------------------------------------------------------------------------
QtAbstractPropertyBrowser* ddPropertiesPanel::propertyBrowser() const
{
  return this->Internal->Browser;
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::addGroup(const QString& name)
{
  QtVariantProperty* property = this->Internal->Manager->addProperty(QtVariantPropertyManager::groupTypeId(), name);
  this->Internal->Browser->addProperty(property);
  return property;
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::addProperty(const QString& name, const QVariant& value)
{
  QtVariantProperty* property = this->Internal->Manager->addProperty(value.type(), name);
  property->setValue(value);
  this->Internal->Browser->addProperty(property);
  this->Internal->Properties[name] = property;

  QtBrowserItem * browserItem = this->Internal->Browser->topLevelItem(property);
  if (browserItem)
  {
    this->Internal->Browser->setExpanded(browserItem, false);
  }

  return property;
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::addSubProperty(const QString& name, const QVariant& value, QtVariantProperty* parent)
{
  if (!parent)
  {
    return 0;
  }

  int subId = parent->subProperties().length();
  QString subName = QString("%1[%2]").arg(name).arg(subId);

  QtVariantProperty* property = this->Internal->Manager->addProperty(value.type(), subName);
  property->setValue(value);

  parent->addSubProperty(property);
  return property;
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::findProperty(const QString& name) const
{
  return this->Internal->Properties.value(name);
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::findSubProperty(const QString& name, QtVariantProperty* parent) const
{
  if (!parent)
  {
    return 0;
  }

  QList<QtProperty*> subProperties = parent->subProperties();
  foreach (QtProperty* subProperty, subProperties)
  {
    if (subProperty->propertyName() == name)
    {
      return static_cast<QtVariantProperty*>(subProperty);
    }
  }

  return 0;
}

//-----------------------------------------------------------------------------
void ddPropertiesPanel::clear()
{
  this->Internal->Manager->clear();
  this->Internal->Properties.clear();
}
