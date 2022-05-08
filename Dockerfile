FROM ubuntu:20.04

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt -y install tzdata
RUN DEBIAN_FRONTEND=noninteractive apt install -y keyboard-configuration
RUN apt install -y clang gnuplot make cmake libeigen3-dev git libgmp3-dev

ADD . /root/
