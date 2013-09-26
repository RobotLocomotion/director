import functools

def motionEvent(view, obj, eventId):
    e.handleTDxMotionEvent(view.lastTDxMotion())

def init(view):

    tdxStyle = view.renderWindow().GetInteractor().GetInteractorStyle().GetTDxStyle()
    if tdxStyle is None:
        return

    tdxSettings = tdxStyle.GetSettings()

    translationSensitivity = 0.01
    angleSensitivity = 0.1
    tdxSettings.SetAngleSensitivity(angleSensitivity)
    tdxSettings.SetTranslationXSensitivity(translationSensitivity)
    tdxSettings.SetTranslationYSensitivity(translationSensitivity)
    tdxSettings.SetTranslationZSensitivity(translationSensitivity)

    eventHandler = functools.partial(motionEvent, view)
    tdxStyle.AddObserver('TDxMotionEvent', eventHandler)
