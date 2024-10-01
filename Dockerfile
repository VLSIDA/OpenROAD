FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive
RUN ln -s /usr/share/zoneinfo/America/Los_Angeles /etc/localtime
RUN echo "America/Los_Angeles" > /etc/timezone

RUN apt-get update
RUN apt-get --no-install-recommends -y upgrade

### Dependencies ###
# General tools for building etc.
RUN apt-get install --no-install-recommends -y build-essential git ssh vim
# Use bash instead of dash
# Must be on one line or else ln won't work without a shell!
RUN rm /bin/sh && ln -s /bin/bash /bin/sh


### OpenROAD ###
COPY . /openroad_build
WORKDIR /openroad_build
RUN ./etc/DependencyInstaller.sh
RUN mkdir build
WORKDIR /openroad_build/build
RUN cmake ..
RUN make -j$(nproc)
RUN make install
WORKDIR /openroad_build


### CLEAN UP ###
# Remove development tools to save space
RUN apt-get remove -y build-essential autoconf automake libtool bison flex  tcl-dev tk-dev cmake
# Cleanup to save some space
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*
