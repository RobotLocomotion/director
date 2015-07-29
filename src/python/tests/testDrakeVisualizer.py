from ddapp import consoleapp
from ddapp import drakevisualizer

def main():

    # use global so the variable is available in the python console
    global app

    app = drakevisualizer.DrakeVisualizerApp()
    app.mainWindow.show()
    consoleapp.ConsoleApp.start()

if __name__ == '__main__':
    main()
