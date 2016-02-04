scriptDir=$(cd $(dirname $0) && pwd)


make_vtk_homebrew_bottle()
{
  brew tap homebrew/science
  $scriptDir/brew_install.sh vtk5 --with-qt
  brew bottle vtk5
  $scriptDir/copy_files.sh vtk5*.tar.gz
}

install_vtk_homebrew_bottle()
{
  wget https://dl.dropboxusercontent.com/s/mtacwrjgmtanfaz/vtk5-5.10.1_2.mavericks.bottle.1.tar.gz
  brew tap homebrew/science
  brew install vtk5-5.10.1_2.mavericks.bottle.1.tar.gz
}

if [ "$TRAVIS_OS_NAME" = "linux" ]; then
	sudo apt-get update -qq
  sudo apt-get install -y libqt4-dev libvtk5-dev libvtk5-qt4-dev python-vtk python-numpy
elif [ "$TRAVIS_OS_NAME" = "osx" ]; then
	brew update
  brew tap homebrew/python
  brew install python numpy
  brew install qt
  install_vtk_homebrew_bottle
fi
