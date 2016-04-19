from director.consoleapp import ConsoleApp
from director import atlasdriver
from director import atlasdriverpanel

def main():

    atlasdriver.init()
    panel = atlasdriverpanel.AtlasDriverPanel(atlasdriver.driver)
    panel.widget.show()
    ConsoleApp.start()


if __name__ == '__main__':
    main()
