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

  ddInternal()
  {
    this->Manager = 0;
    this->Browser = 0;
  }

  QtVariantPropertyManager* Manager;
  QtAbstractPropertyBrowser* Browser;

  QMap<QString, QtVariantProperty*> Properties;
  QMap<QtVariantProperty*, QtVariantProperty*> SubPropertyMap;
};


//-----------------------------------------------------------------------------
ddPropertiesPanel::ddPropertiesPanel(QWidget* parent) : QWidget(parent)
{
  this->Internal = new ddInternal;

  QtVariantPropertyManager *manager = new QtVariantPropertyManager;
  this->Internal->Manager = manager;

  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->setMargin(0);
  this->setBrowserModeToTree();

  this->connect(this->Internal->Manager,
      SIGNAL(valueChanged(QtProperty*, const QVariant&)),
      SLOT(onPropertyValueChanged(QtProperty*)));
}

//-----------------------------------------------------------------------------
ddPropertiesPanel::~ddPropertiesPanel()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void ddPropertiesPanel::setBrowserModeToTree()
{
  this->clear();
  delete this->Internal->Browser;

  QtVariantEditorFactory *variantFactory = new QtVariantEditorFactory;
  QtTreePropertyBrowser *browser = new QtTreePropertyBrowser;
  browser->setFactoryForManager(this->Internal->Manager, variantFactory);
  browser->setPropertiesWithoutValueMarked(true);
  browser->setRootIsDecorated(true);
  this->layout()->addWidget(browser);
  this->Internal->Browser = browser;
}

//-----------------------------------------------------------------------------
void ddPropertiesPanel::setBrowserModeToWidget()
{
  this->clear();
  delete this->Internal->Browser;

  QtVariantEditorFactory *variantFactory = new QtVariantEditorFactory;
  QtGroupBoxPropertyBrowser* browser = new QtGroupBoxPropertyBrowser;
  browser->setFactoryForManager(this->Internal->Manager, variantFactory);
  this->layout()->addWidget(browser);
  this->Internal->Browser = browser;
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
QtVariantProperty* ddPropertiesPanel::addGroup(const QString& name, const QString& description)
{
  QtVariantProperty* property = this->Internal->Manager->addProperty(QtVariantPropertyManager::groupTypeId(), description);
  this->Internal->Browser->addProperty(property);
  this->Internal->Properties[name] = property;
  return property;
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::addProperty(const QString& name, const QVariant& value)
{
  int propertyType = value.type();

  QtVariantProperty* property = this->Internal->Manager->addProperty(propertyType, name);
  property->setValue(value);

  this->Internal->Browser->addProperty(property);
  this->Internal->Properties[name] = property;

  QtBrowserItem * browserItem = this->Internal->Browser->topLevelItem(property);
  QtTreePropertyBrowser* treeBrowser = qobject_cast<QtTreePropertyBrowser*>(this->Internal->Browser);
  if (browserItem && treeBrowser)
  {
    treeBrowser->setExpanded(browserItem, false);
  }

  return property;
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::addEnumProperty(const QString& name, const QVariant& value)
{
  int propertyType = this->Internal->Manager->enumTypeId();

  QtVariantProperty* property = this->Internal->Manager->addProperty(propertyType, name);
  property->setValue(value);

  this->Internal->Browser->addProperty(property);
  this->Internal->Properties[name] = property;

  QtBrowserItem * browserItem = this->Internal->Browser->topLevelItem(property);
  QtTreePropertyBrowser* treeBrowser = qobject_cast<QtTreePropertyBrowser*>(this->Internal->Browser);
  if (browserItem && treeBrowser)
  {
    treeBrowser->setExpanded(browserItem, false);
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
  this->Internal->SubPropertyMap[property] = parent;
  return property;
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::getProperty(const QString& name) const
{
  return this->Internal->Properties.value(name);
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::getSubProperty(QtVariantProperty* parent, const QString& name) const
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
QtVariantProperty* ddPropertiesPanel::getSubProperty(QtVariantProperty* parent, int childIndex) const
{
  if (!parent)
  {
    return 0;
  }

  QList<QtProperty*> subProperties = parent->subProperties();
  if (childIndex >= 0 && childIndex < subProperties.size())
  {
    return static_cast<QtVariantProperty*>(subProperties[childIndex]);
  }

  return 0;
}

//-----------------------------------------------------------------------------
int ddPropertiesPanel::getSubPropertyIndex(QtVariantProperty* property) const
{
  QtVariantProperty* parent = this->getParentProperty(property);
  if (!parent)
  {
    return 0;
  }

  return parent->subProperties().indexOf(property);
}

//-----------------------------------------------------------------------------
QtVariantProperty* ddPropertiesPanel::getParentProperty(QtVariantProperty* property) const
{
  return this->Internal->SubPropertyMap.value(property);
}

//-----------------------------------------------------------------------------
void ddPropertiesPanel::clear()
{
  this->Internal->Manager->clear();
  this->Internal->Properties.clear();
  this->Internal->SubPropertyMap.clear();
}
