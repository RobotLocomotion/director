from ddapp.consoleapp import ConsoleApp
from ddapp import atlasdriver
from ddapp import atlasdriverpanel

def main():

    atlasdriver.init()
    panel = atlasdriverpanel.AtlasDriverPanel(atlasdriver.driver)
    panel.widget.show()
    ConsoleApp.start()


if __name__ == '__main__':
    main()
