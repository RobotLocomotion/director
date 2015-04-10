from ddapp.consoleapp import ConsoleApp
from ddapp import lcmspy
from ddapp import lcmUtils
from ddapp import simpletimer as st

app = ConsoleApp()

app.setupGlobals(globals())

if app.getTestingInteractiveEnabled():
    app.showPythonConsole()


lcmspy.findLCMModulesInSysPath()

timer = st.SimpleTimer()
stats = {}

channelToMsg = {}
items = {}

def item(r, c):
    rowDict = items.setdefault(r, {})
    try:
        return rowDict[c]

    except KeyError:
        i = QtGui.QTableWidgetItem('')
        table.setItem(r, c, i)
        rowDict[c] = i
        return i


def printStats():
    print '\n------------------------\n'

    averages = [(channel, stat.getAverage()) for channel, stat in stats.iteritems()]

    averages.sort(key=lambda x: x[1])

    table.setRowCount(len(averages))
    i = 0
    for channel, bytesPerSecond in reversed(averages):
        print channel, '%.3f kbps' % (bytesPerSecond/1024.0)

        item(i, 0).setText(channel)
        item(i, 1).setText(channelToMsg[channel])
        item(i, 2).setText('%.3f kbps' % (bytesPerSecond/1024.0))
        i += 1


def onMessage(messageData, channel):

    messageData = str(messageData)
    msgType = lcmspy.getMessageClass(messageData)

    if not msgType:
        #print 'failed decode:', channel
        pass
    else:
        name = lcmspy.getMessageTypeFullName(msgType)

    stat = stats.get(channel)
    if not stat:
        stat = st.AverageComputer()
        stats[channel] = stat

    stat.update(len(messageData))

    if channel not in channelToMsg:
        channelToMsg[channel] = lcmspy.getMessageTypeFullName(msgType) if msgType else '<unknown msg type>'

    if timer.elapsed() > 3:
        printStats()
        timer.reset()

        for stat in stats.values():
            stat.reset()

    #msg = lcmspy.decodeMessage(messageData)


sub = lcmUtils.addSubscriber(channel='.+', callback=onMessage)
sub.setNotifyAllMessagesEnabled(True)


from PythonQt import QtGui, QtCore

table = QtGui.QTableWidget()
table.setColumnCount(3)
table.setHorizontalHeaderLabels(['channel', 'type', 'bandwidth'])
table.verticalHeader().setVisible(False)


table.show()




app.start()
