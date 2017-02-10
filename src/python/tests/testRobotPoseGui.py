import director
from director import robotsystem
from director import robotposegui
from director.consoleapp import ConsoleApp
from director import ikplanner
from director import playbackpanel

app = ConsoleApp()

app.setupGlobals(globals())

view = app.createView()

robotSystem = robotsystem.create(view, planningOnly=True)

ikPlanner = robotSystem.ikPlanner
jc = robotSystem.robotStateJointController


if ikPlanner.planningMode == 'pydrake':
    ikPlanner.plannerPub._setupLocalServer()

elif ikPlanner.planningMode == 'matlabdrake':
    robotSystem.startIkServer() # launches matlab server

groupName = 'General'
poseName = 'arm up pregrasp'
poseNames = ikplanner.RobotPoseGUIWrapper.getPoseNamesInGroup(groupName)

if poseName not in poseNames:
    poseName = poseNames[0]

ikPlanner.addPostureGoalListener(jc)
pose = ikPlanner.getMergedPostureFromDatabase(jc.q, groupName, poseName)
jc.setPose('merged', pose)

view.show()
ikplanner.RobotPoseGUIWrapper.show()
robotSystem.playbackPanel.widget.show()

app.start()
