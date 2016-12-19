FROM ubuntu:16.04
ENV DISPLAY :99.0
ENV USE_LCM ON
ENV MAKE_PACKAGE ON

RUN apt-get update
RUN apt-get install -y \
	gcc-4.9 \
	g++-4.9 \
	git

RUN apt-get install -y \
	build-essential \
	cmake \
    doxygen \
    graphviz \
    libeigen3-dev \
	libqt4-dev \
	libvtk5-dev \
	libvtk5-qt4-dev \
    libvtk-java \
    python-coverage \
    python-dev \
    python-lxml \
    python-numpy \
    python-pip \
    python-sphinx \
    python-vtk \
    python-yaml \
    xvfb

RUN apt-get install -y \
	libglib2.0-dev

RUN pip install sphinx-rtd-theme
RUN /sbin/start-stop-daemon --start --quiet --pidfile /tmp/custom_xvfb_99.pid --make-pidfile \
                          --background --exec /usr/bin/Xvfb -- :99 -ac -screen 0 1280x1024x16

ENV CXX g++-4.9
ENV CC gcc-4.9
ENV TRAVIS_OS_NAME linux

COPY . /root/

RUN mkdir build
WORKDIR /root/build
RUN cmake -DUSE_LCM:BOOL=$USE_LCM ../distro/superbuild
RUN make -j2

WORKDIR /root/build/src/director-build
RUN cmake -DSITE:STRING=travis-docker -DBUILDNAME:STRING=travis-docker_lcm-$USE_LCM .
RUN ctest -j 1 --dashboard Experimental --track travis --output-on-failure

WORKDIR /root
RUN /root/distro/travis/after_success.sh
