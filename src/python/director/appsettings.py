

def saveState(settings, widget, key):
    settings.beginGroup(key)
    settings.setValue('position', widget.pos)
    settings.setValue('size', widget.size)
    if hasattr(widget, 'saveState'):
        settings.setValue('state', widget.saveState())
    settings.endGroup()


def restoreState(settings, widget, key):

    settings.beginGroup(key)

    if settings.contains('size'):
        widget.resize(settings.value('size'))

    if settings.contains('position'):
        widget.move(settings.value('position'))

    if settings.contains('state') and hasattr(widget, 'restoreState'):
        widget.restoreState(settings.value('state'))

    settings.endGroup()
