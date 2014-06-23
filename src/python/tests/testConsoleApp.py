from ddapp.consoleapp import ConsoleApp


def main():

    app = ConsoleApp()

    app.setupGlobals(globals())
    app.showPythonConsole()

    view = app.createView()
    view.show()

    app.start()



if __name__ == '__main__':
    main()
