#ifndef __ddPropertiesPanel_h
#define __ddPropertiesPanel_h

#include <QWidget>


class QtVariantPropertyManager;
class QtAbstractPropertyBrowser;
class QtVariantProperty;
class QtProperty;

class ddPropertiesPanel : public QWidget
{
    Q_OBJECT

public:

  ddPropertiesPanel(QWidget* parent=0);
  virtual ~ddPropertiesPanel();

  void setBrowserModeToTree();
  void setBrowserModeToWidget();

  QtVariantPropertyManager* propertyManager() const;
  QtAbstractPropertyBrowser* propertyBrowser() const;

  QtVariantProperty* addGroup(const QString& name);
  QtVariantProperty* addProperty(const QString& name, const QVariant& value);
  QtVariantProperty* addSubProperty(const QString& name, const QVariant& value, QtVariantProperty* parent);
  QtVariantProperty* findProperty(const QString& name) const;
  QtVariantProperty* findSubProperty(const QString& name, QtVariantProperty* parent) const;

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
