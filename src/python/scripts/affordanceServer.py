from ddapp import consoleapp
from ddapp import lcmobjectcollection
from ddapp.timercallback import TimerCallback
import datetime

def main():

    app = consoleapp.ConsoleApp()

    meshCollection = lcmobjectcollection.LCMObjectCollection('MESH_COLLECTION_COMMAND')
    affordanceCollection = lcmobjectcollection.LCMObjectCollection('AFFORDANCE_COLLECTION_COMMAND')

    meshCollection.sendEchoRequest()
    affordanceCollection.sendEchoRequest()

    def printCollection():
        print
        print '----------------------------------------------------'
        print datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print '%d affordances' % len(affordanceCollection.collection)
        for desc in affordanceCollection.collection.values():
            print
            print 'name:', desc['Name']
            print 'type:', desc['classname']


    timer = TimerCallback(targetFps=0.2)
    timer.callback = printCollection
    timer.start()

    #app.showPythonConsole()
    app.start()


if __name__ == '__main__':
    main()
