FROM ubuntu:16.04
ENV DISPLAY :99.0

ARG USE_LCM
ENV USE_LCM ${USE_LCM:-ON}

ARG MAKE_PACKAGE
ENV MAKE_PACKAGE ${MAKE_PACKAGE:-ON}

ARG TRAVIS_OS_NAME
ENV TRAVIS_OS_NAME ${TRAVIS_OS_NAME:-linux}

ARG TRAVIS_PULL_REQUEST
ENV TRAVIS_PULL_REQUEST $TRAVIS_PULL_REQUEST

ARG encrypted_444f3458e047_key
ENV encrypted_444f3458e047_key $encrypted_444f3458e047_key

ARG encrypted_444f3458e047_iv
ENV encrypted_444f3458e047_iv $encrypted_444f3458e047_iv

ARG encrypted_copyfiles_host
ENV encrypted_copyfiles_host $encrypted_copyfiles_host

RUN apt-get update

# Install packages required to build and test
RUN apt-get install -y \
  build-essential \
  cmake \
  doxygen \
  git \
  graphviz \
  libeigen3-dev \
  libexpat-dev \
  libfreetype6-dev \
  libglib2.0-dev \
  libjpeg-dev \
  libqt5webkit5-dev \
  libqt5x11extras5-dev \
  libtiff5-dev \
  libxml2-dev \
  libxt-dev \
  python-coverage \
  python-dev \
  python-lxml \
  python-numpy \
  python-pip \
  python-sphinx \
  python-yaml \
  python2.7-dev \
  qt5-default \
  qtbase5-dev \
  qtbase5-private-dev \
  qtmultimedia5-dev \
  qttools5-dev \
  wget \
  xvfb

RUN pip install sphinx-rtd-theme

COPY . /root/

# Build
RUN mkdir build
WORKDIR /root/build
RUN cmake \
  -DDD_QT_VERSION:STRING=5 \
  -DUSE_SYSTEM_VTK:BOOL=OFF \
  -DDOWNLOAD_VTK_PACKAGE:BOOL=ON \
  -DUSE_LCM:BOOL=$USE_LCM \
  ../distro/superbuild
RUN make -j2
WORKDIR /root/build/src/director-build
RUN cmake -DSITE:STRING=travis-docker -DBUILDNAME:STRING=travis-docker_lcm-$USE_LCM .

# Run the tests and build the package
ENV LD_LIBRARY_PATH /root/lib/:/root/build/install/lib
ENV PYTHONPATH /root/lib/python2.7/site-packages/:/root/build/install/lib/python2.7/site-packages
RUN /root/distro/docker/run_tests_and_package.sh
