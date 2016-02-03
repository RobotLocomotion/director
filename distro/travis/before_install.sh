

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
	sudo apt-get update -qq
  sudo apt-get install -y libqt4-dev libvtk5-dev libvtk5-qt4-dev python-vtk python-numpy
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
	brew update
  brew tap homebrew/science
  brew tap homebrew/python
  brew install python numpy
  brew install vtk5 --with-qt
fi
