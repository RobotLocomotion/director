set -xe

# We need to run Xvfb in the same shell script in which we 
# will run ctest and the packaging script (which both use
# directorPython. That's because if we run Xvfb from a 
# separate docker RUN command, it will get killed before we
# run the tests. 
Xvfb :99 -ac -screen 0 1280x1024x16 &


ctest -j 1 --dashboard Experimental --track travis --output-on-failure

if [ "$TRAVIS_PULL_REQUEST" = "false" ]; then
	cd /root/distro/package
	./make_linux_package.sh 2>&1 > log.txt || cat log.txt
	  /root/distro/travis/copy_files.sh /root/distro/package/*.tar.gz
fi
