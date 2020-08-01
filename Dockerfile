# ffmpeg - http://ffmpeg.org/download.html
#
# From https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
#
# https://hub.docker.com/r/jrottenberg/ffmpeg/
#
#

FROM ubuntu:18.04 AS base

WORKDIR     /image_test

# Start
RUN echo "\e[1;33;41m ==========  Start  ========= \e[0m"

COPY sources.list /etc/apt/

RUN     apt-get -yqq update && \
        apt-get -y upgrade && \
        apt-get install -yq --no-install-recommends ca-certificates expat libgomp1 && \
        apt-get autoremove -y && \
        apt-get clean -y

RUN     buildDeps="wget \
                   unzip \
                   gcc \
                   g++ \
                   cmake \
                   make \
                   python3 \
                   python3-pip \
                   git" && \
        apt-get -yqq update && \
        apt-get install -yq --no-install-recommends ${buildDeps}

# JPEG2000
RUN echo "\e[1;33;41m ==========  Kakadu  ========= \e[0m"

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O kakadu.zip https://kakadusoftware.com/wp-content/uploads/2020/06/KDU805_Demo_Apps_for_Linux-x86-64_200602.zip && \
    unzip kakadu.zip -d kakadu && \
    rm -f kakadu.zip

ENV     LD_LIBRARY_PATH=/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602

# JPEG
RUN echo "\e[1;33;41m ==========  JPEG  ========= \e[0m"

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O jpeg.tar.gz http://www.ijg.org/files/jpegsrc.v9d.tar.gz && \
    tar  xzvf jpeg.tar.gz  && \
    rm -f jpeg.tar.gz   && \
    cd /tools/jpeg-9d   && \
    ./configure  && \
    make &&\
    make test

# WEBP
RUN echo "\e[1;33;41m ==========  WebP  ========= \e[0m"

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O libwebp.tar.gz https://storage.googleapis.com/downloads.webmproject.org/releases/webp/libwebp-1.1.0-linux-x86-64.tar.gz  && \
    tar xvzf libwebp.tar.gz && \
    rm -f libwebp.tar.gz
# libGL libraries are needed for webp.
RUN apt install -y libglu1 libxi6

# BPG
RUN echo "\e[1;33;41m ==========  BPG  ========= \e[0m"

COPY libbpg-master.zip /tools

RUN     buildDeps="libpng-dev \
                   libjpeg-dev \
                   libsdl1.2-dev \
                   libslang2-dev \
                   libsdl-image1.2-dev \
                   yasm" && \
        apt-get -yqq update && \
        apt-get install -yq --no-install-recommends ${buildDeps}

RUN mkdir -p /tools && \
    cd /tools && \
#   wget -O bpg.tar.gz https://bellard.org/bpg/libbpg-0.9.8.tar.gz  && \
    unzip libbpg-master.zip && \
    rm -f libbpg-master.zip && \
    cd /tools/libbpg-master  && \
    make