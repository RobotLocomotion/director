set -xe

Xvfb :99 -ac -screen 0 1280x1024x16 &
ctest -j 1 --dashboard Experimental --track travis --output-on-failure
cd /root/distro/package
./make_linux_package.sh 2>&1 > log.txt || cat log.txt
  /root/distro/travis/copy_files.sh /root/distro/package/*.tar.gz