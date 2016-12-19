#!/bin/bash

scriptDir=$(cd $(dirname $0) && pwd)


superbuildInstallDir=$scriptDir/../../build/install

if [ ! -d "$superbuildInstallDir" ]; then
  superbuildInstallDir=$scriptDir/../../../build/install
fi

versionString=$($superbuildInstallDir/bin/directorPython -c 'import director.version as ver; print ver.versionString()')

osid=$( cat /etc/os-release | grep "^ID=" | sed s/ID=// )
osversion=$( cat /etc/os-release | grep "VERSION_ID=" | sed s/VERSION_ID=// | sed 's/"\(.*\)"/\1/' )

echo "OS ID and version set as: $osid-$osversion"

packageName=director-$versionString-$osid-$osversion

######


install_patchelf()
{
  cd $scriptDir
  wget http://nixos.org/releases/patchelf/patchelf-0.8/patchelf-0.8.tar.gz
  tar -zxf patchelf-0.8.tar.gz
  pushd  patchelf-0.8
  ./configure --prefix=$scriptDir/patchelf-install
  make install
  popd
  rm -rf patchelf-0.8 patchelf-0.8.tar.gz
}

patchelfExe=$scriptDir/patchelf-install/bin/patchelf

if [ ! -f "$patchelfExe" ]; then
  install_patchelf
fi

cd $scriptDir
echo 'running fixup_elf script'
python fixup_elf.py $superbuildInstallDir $superbuildInstallDir/lib $patchelfExe

cp -r $superbuildInstallDir $packageName
tar -czf $packageName.tar.gz $packageName
