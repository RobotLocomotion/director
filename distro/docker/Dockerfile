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
	libglib2.0-dev \
	wget

RUN pip install sphinx-rtd-theme

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
RUN /root/distro/docker/run_tests_and_package.sh
