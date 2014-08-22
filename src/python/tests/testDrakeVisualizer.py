from ddapp import drakevisualizer

def main():

    # use global so the variable is available in the python console
    global app

    app = drakevisualizer.DrakeVisualizerApp()
    app.setupGlobals(globals())
    app.mainWindow.show()
    app.start()

if __name__ == '__main__':
    main()
