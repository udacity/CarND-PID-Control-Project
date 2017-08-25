FROM ubuntu:14.04

FROM gcc:5.4

MAINTAINER Malcolm Ma <malichaooo@gmail.com>

# Arguments
ARG user
ARG uid
ARG home
ARG shell

# Basic Utilities
RUN apt-get -y update && apt-get install -y sudo ssh build-essential git

# Install cmake 3.9
RUN wget https://cmake.org/files/v3.9/cmake-3.9.0-rc5.tar.gz && \
	tar xf cmake-3.9.0-rc5.tar.gz && \
	cd cmake-3.9.0-rc5 && ./configure && make -j && make install && \
	cd .. && rm -rf cmake*

# Install dependencies for uWebSockets
RUN apt-get install -y apt-utils libssl-dev autotools-dev

# Build libuv1 from source as it's not available on ubuntu 14
RUN wget https://launchpad.net/ubuntu/+archive/primary/+files/libuv1_1.9.1.orig.tar.gz && \
	tar xf libuv1_1.9.1.orig.tar.gz && \
	cd libuv-v1.9.1 && ./autogen.sh && ./configure && make -j && \
	make check && make install && \
	cd .. && rm -rf libuv* && ldconfig

# Install uWebSockets
RUN git clone https://github.com/uWebSockets/uWebSockets && cd uWebSockets && \
	git checkout e94b6e1 && mkdir build && cd build && cmake .. && make -j &&\
	make install && cd ../.. && ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so &&\
	rm -r uWebSockets

# Make SSH available
EXPOSE 22

# Flask Server
EXPOSE 4567

# Mount the user's home directory
VOLUME "${home}"

# Clone user into docker image and set up X11 sharing 
RUN \
  echo "${user}:x:${uid}:${uid}:${user},,,:${home}:${shell}" >> /etc/passwd && \
  echo "${user}:x:${uid}:" >> /etc/group && \
  echo "${user} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${user}" && \
  chmod 0440 "/etc/sudoers.d/${user}"

# Switch to user
USER "${user}"

# This is required for sharing Xauthority
ENV QT_X11_NO_MITSHM=1

