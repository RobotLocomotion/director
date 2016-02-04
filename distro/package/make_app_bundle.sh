#!/bin/bash

scriptDir=$(cd $(dirname $0) && pwd)


appName=DirectorConsole
bundleDir=$scriptDir/$appName.app
superbuildInstallDir=$scriptDir/../../build/install

######
libDir=$bundleDir/Contents/MacOS/lib
binDir=$bundleDir/Contents/MacOS/bin
sitePackagesDir=$libDir/python2.7/site-packages

rm -rf $bundleDir
cp -r $scriptDir/bundle_template.app $bundleDir

cp -r $superbuildInstallDir/{bin,lib,include} $bundleDir/Contents/MacOS/
cp /usr/local/bin/python $binDir/
touch $binDir/qt.conf

mkdir -p $sitePackagesDir
cp -r /usr/local/opt/vtk5/lib/python2.7/site-packages/vtk $sitePackagesDir/
cp -r /usr/local/lib/python2.7/site-packages/numpy $sitePackagesDir/
cp -r /usr/local/lib/python2.7/site-packages/scipy $sitePackagesDir/


python $scriptDir/fixup_mach_o.py $superbuildInstallDir $bundleDir $libDir

# remove broken symlink to homebrew python site-packages
rm $libDir/Python.framework/Versions/Current/lib/python2.7/site-packages
