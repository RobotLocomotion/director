#ifndef __ddPropertiesPanel_h
#define __ddPropertiesPanel_h

#include <QWidget>
#include "ddAppConfigure.h"

class QtVariantPropertyManager;
class QtAbstractPropertyBrowser;
class QtVariantProperty;
class QtProperty;

class DD_APP_EXPORT ddPropertiesPanel : public QWidget
{
    Q_OBJECT

public:

  ddPropertiesPanel(QWidget* parent=0);
  virtual ~ddPropertiesPanel();

  void setBrowserModeToTree();
  void setBrowserModeToWidget();

  QtVariantPropertyManager* propertyManager() const;
  QtAbstractPropertyBrowser* propertyBrowser() const;

  QtVariantProperty* addGroup(const QString& name, const QString& description);
  QtVariantProperty* addProperty(const QString& name, const QVariant& value);
  QtVariantProperty* addEnumProperty(const QString& name, const QVariant& value);
  QtVariantProperty* addSubProperty(const QString& name, const QVariant& value, QtVariantProperty* parent);
  QtVariantProperty* getProperty(const QString& name) const;
  QtVariantProperty* getSubProperty(QtVariantProperty* parent, const QString& name) const;
  QtVariantProperty* getSubProperty(QtVariantProperty* parent, int childIndex) const;
  int getSubPropertyIndex(QtVariantProperty* property) const;
  QtVariantProperty* getParentProperty(QtVariantProperty* parent) const;

public slots:

  void clear();

signals:

  void propertyValueChanged(QtVariantProperty* property);

protected slots:

  void onPropertyValueChanged(QtProperty* property);

protected:

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddPropertiesPanel);
};

#endif
