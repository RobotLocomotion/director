#!/bin/bash

scriptDir=$(cd $(dirname $0) && pwd)


superbuildInstallDir=$scriptDir/../../build/install

if [ ! -d "$superbuildInstallDir" ]; then
  superbuildInstallDir=$scriptDir/../../../build/install
fi

######


install_patchelf()
{
  cd $scriptDir
  wget http://nixos.org/releases/patchelf/patchelf-0.8/patchelf-0.8.tar.gz
  tar -zxf patchelf-0.8.tar.gz
  pushd  patchelf-0.8
  ./configure --prefix=$scriptDir/patchelf-install
  make install
  rm -r patchelf-0.8 patchelf-0.8.tar.gz
  popd
}

patchelfExe=$scriptDir/patchelf-install/bin/patchelf

if [ ! -f "$patchelfExe" ]; then
  install_patchelf
fi

cd $scriptDir
python fixup_elf.py $superbuildInstallDir $superbuildInstallDir/lib $patchelfExe

cp -r $superbuildInstallDir director-install
tar -czf director-install.tar.gz director-install
