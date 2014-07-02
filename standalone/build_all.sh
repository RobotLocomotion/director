thisFile=${BASH_SOURCE[0]}
baseDir=$(cd $(dirname $thisFile) && pwd)

cd $baseDir
rm -rf build
mkdir build
cd build
cmake ../superbuild
make -j8
