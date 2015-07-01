#!/bin/bash

set -x

tarName=director_0.1.orig
sourceName=director_0.1-0ppa0
baseDir=$(cd $(dirname $0)/../.. && pwd)
name=$(basename $baseDir)

cd $baseDir
ln -s distro/debian .
cd ..
tar -acf $tarName.tar.gz $name
cd $baseDir
debuild -S -sd
cd ..
dput ppa:patmarion/ppa ${sourceName}_source.changes

rm director/debian
mkdir -p ppa_uploads
mv ${tarName}.tar.gz ppa_uploads/
mv ${sourceName}* ppa_uploads/
