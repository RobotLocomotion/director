FROM ubuntu:18.04

WORKDIR /root
ARG config=vtk8-qt5-python2-ubuntu18
COPY configurations/${config} commands

RUN bash commands/install_deps
RUN bash commands/download
RUN bash commands/configure
RUN bash commands/build
RUN bash commands/package
