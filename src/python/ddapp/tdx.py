import functools

_editor = None

def motionEvent(view, obj, eventId):
    _editor.handleTDxMotionEvent(view.lastTDxMotion())

def init(view, editor):

    tdxStyle = view.renderWindow().GetInteractor().GetInteractorStyle().GetTDxStyle()
    if tdxStyle is None:
        return

    global _editor
    _editor = editor
    tdxSettings = tdxStyle.GetSettings()

    translationSensitivity = 0.1
    angleSensitivity = 0.1
    tdxSettings.SetAngleSensitivity(angleSensitivity)
    tdxSettings.SetTranslationXSensitivity(translationSensitivity)
    tdxSettings.SetTranslationYSensitivity(translationSensitivity)
    tdxSettings.SetTranslationZSensitivity(translationSensitivity)

    eventHandler = functools.partial(motionEvent, view)
    tdxStyle.AddObserver('TDxMotionEvent', eventHandler)
