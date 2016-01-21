from director import robotsystem
from director import robotposegui
from director.consoleapp import ConsoleApp
from director import ikplanner
from director import playbackpanel

app = ConsoleApp()

app.setupGlobals(globals())

view = app.createView()

robotSystem = robotsystem.create(view)


testPlanning = True
if testPlanning:

    robotSystem.startIkServer() # launches matlab server

    playbackPanel = playbackpanel.PlaybackPanel(
                          robotSystem.planPlayback, robotSystem.playbackRobotModel,
                          robotSystem.playbackJointController, robotSystem.robotStateModel,
                          robotSystem.robotStateJointController, robotSystem.manipPlanner)

    robotSystem.ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)
    robotSystem.manipPlanner.connectPlanReceived(playbackPanel.setPlan)
    playbackPanel.widget.show()


jc = robotSystem.robotStateJointController
pose = robotSystem.ikPlanner.getMergedPostureFromDatabase(jc.q, 'General', 'arm up pregrasp')
jc.setPose('merged', pose)


view.show()
ikplanner.RobotPoseGUIWrapper.show()


app.start()
