scriptDir=$(cd $(dirname $0) && pwd)


if [ "$TRAVIS_OS_NAME" = "linux" ]; then
	sudo apt-get update -qq
  sudo apt-get install -y libqt4-dev libvtk5-dev libvtk5-qt4-dev python-vtk python-numpy
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
	brew update
  brew tap homebrew/science
  brew tap homebrew/python
  brew install python numpy
  $scriptDir/brew_install.sh vtk5 --with-qt
  brew bottle vtk5
  $scriptDir/copy_files.sh vtk5*.tar.gz
fi
