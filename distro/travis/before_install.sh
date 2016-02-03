if [ "$TRAVIS_OS_NAME" = "osx" ]; then
	brew update
  brew tap homebrew/science
  brew tap homebrew/python
  brew install python numpy
  brew install vtk5 --with-qt
fi
