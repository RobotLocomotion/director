Xvfb :99 -ac -screen 0 1280x1024x16 &
ctest -j 1 --dashboard Experimental --track travis --output-on-failure
