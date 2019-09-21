import PythonQt
from PythonQt import QtCore, QtGui
import director.objectmodel as om
import director.visualization as vis
from director import cameracontrol
from director import propertyset
from director import frameupdater
from director import vieweventfilter


_contextMenuActions = []

def registerContextMenuActions(getActionsFunction):
    _contextMenuActions.append(getActionsFunction)


def getContextMenuActions(view, pickedObj, pickedPoint):
    actions = []
    for func in _contextMenuActions:
        actions.extend(func(view, pickedObj, pickedPoint))
    return actions

def getDefaultContextMenuActions(view, pickedObj, pickedPoint):

    def onDelete():
        om.removeFromObjectModel(pickedObj)

    def onHide():
        pickedObj.setProperty('Visible', False)

    def onSelect():
        om.setActiveObject(pickedObj)

    actions = [
      (None, None),
      ('Select', onSelect),
      ('Hide', onHide)
      ]

    if pickedObj.getProperty('Deletable'):
        actions.append(['Delete', onDelete])

    return actions

registerContextMenuActions(getDefaultContextMenuActions)


def getShortenedName(name, maxLength=30):
    if len(name) > maxLength:
        name = name[:maxLength-3] + '...'
    return name


def showRightClickMenu(displayPoint, view):

    pickedObj, pickedPoint = vis.findPickedObject(displayPoint, view)
    if not pickedObj:
        return

    objectName = pickedObj.getProperty('Name')
    if objectName == 'grid':
        return

    objectName = getShortenedName(objectName)

    displayPoint = displayPoint[0], view.height - displayPoint[1]

    globalPos = view.mapToGlobal(QtCore.QPoint(*displayPoint))

    menu = QtGui.QMenu(view)

    widgetAction = QtGui.QWidgetAction(menu)
    label = QtGui.QLabel('<b>%s</b>' % objectName)
    label.setContentsMargins(9,9,6,6)
    widgetAction.setDefaultWidget(label)
    menu.addAction(widgetAction)
    menu.addSeparator()


    propertiesPanel = PythonQt.dd.ddPropertiesPanel()
    propertiesPanel.setBrowserModeToWidget()
    panelConnector = propertyset.PropertyPanelConnector(pickedObj.properties, propertiesPanel)
    def onMenuHidden():
      panelConnector.cleanup()
    menu.connect('aboutToHide()', onMenuHidden)

    propertiesMenu = menu.addMenu('Properties')
    propertiesWidgetAction = QtGui.QWidgetAction(propertiesMenu)
    propertiesWidgetAction.setDefaultWidget(propertiesPanel)
    propertiesMenu.addAction(propertiesWidgetAction)

    actions = getContextMenuActions(view, pickedObj, pickedPoint)

    for actionName, func in actions:
        if not actionName:
            menu.addSeparator()
        else:
            action = menu.addAction(actionName)
            action.connect('triggered()', func)

    selectedAction = menu.popup(globalPos)


def zoomToPick(displayPoint, view):
    pickedPoint, prop, _ = vis.pickProp(displayPoint, view)
    if not prop:
        return
    flyer = cameracontrol.Flyer(view)
    flyer.zoomTo(pickedPoint)


def getChildFrame(obj):
    if hasattr(obj, 'getChildFrame'):
        return obj.getChildFrame()


def toggleFrameWidget(displayPoint, view):

    obj, _ = vis.findPickedObject(displayPoint, view)

    if not isinstance(obj, vis.FrameItem):
        obj = getChildFrame(obj)

    if not obj:
        return False

    edit = not obj.getProperty('Edit')
    obj.setProperty('Edit', edit)

    parent = obj.parent()
    if getChildFrame(parent) == obj and not isinstance(parent, vis.GridItem):
        parent.setProperty('Alpha', 0.5 if edit else 1.0)

    return True


class ViewBehaviors(vieweventfilter.ViewEventFilter):

    CONSUMED_KEYS = ('r', 's', 'w', 'l', '3', 'p', 'f')

    def onLeftDoubleClick(self, event):

        displayPoint = self.getMousePositionInView(event)
        if toggleFrameWidget(displayPoint, self.view):
            self.consumeEvent()
        else:
            self.callHandler(self.LEFT_DOUBLE_CLICK_EVENT, displayPoint, self.view, event)

    def onRightClick(self, event):
        displayPoint = self.getMousePositionInView(event)
        showRightClickMenu(displayPoint, self.view)

    def onKeyPress(self, event):

        consumed = False

        key = str(event.text()).lower()

        if key == 'f':
            consumed = True
            zoomToPick(self.getCursorDisplayPosition(), self.view)

        elif key == 'r':
            consumed = True
            self.view.resetCamera()
            self.view.render()
        else:
            consumed = key in self.CONSUMED_KEYS

        if consumed:
            self.consumeEvent()

    def onKeyPressRepeat(self, event):

        consumed = frameupdater.handleKey(event)

        # prevent these keys from going to vtkRenderWindow's default key press handler
        key = str(event.text()).lower()
        consumed = key in ViewBehaviors.CONSUMED_KEYS

        if consumed:
            self.consumeEvent()
