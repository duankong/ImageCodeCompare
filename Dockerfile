# ffmpeg - http://ffmpeg.org/download.html
#
# From https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
#
# https://hub.docker.com/r/jrottenberg/ffmpeg/
#
#

FROM archlinux:20200705 AS base

WORKDIR     /image_test          

# Start
RUN     echo " ====================  START  ==================== "

ARG     PREFIX=/opt/ffmpeg

ENV     X265_VERSION=3.2

RUN     pacman -Syu --noconfirm

RUN     buildDeps="wget \
                   vi \
                   unzip \
                   gcc \
                   make \
                   cmake \
                   diffutils \
                   python \
                   python-pip \
                   imagemagick \
                   tree \
                   git \
                   automake \
                   sqlite3 \
                   yasm \
                   sdl \
                   sdl_image \
                   pkg-config" && \
        pacman -S --noconfirm ${buildDeps} && \
       	buildPacket="scikit-image" &&\
       	pip install ${buildPacket}


### JPEG2000
RUN echo " ====================  JPEG2000  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O kakadu.zip https://kakadusoftware.com/wp-content/uploads/2020/06/KDU805_Demo_Apps_for_Linux-x86-64_200602.zip && \
    unzip kakadu.zip -d kakadu && \
    rm -f kakadu.zip

ENV     LD_LIBRARY_PATH=/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602

### JPEG
RUN echo " ====================  JPEG  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O jpeg.tar.gz http://www.ijg.org/files/jpegsrc.v9d.tar.gz && \
    tar  xzvf jpeg.tar.gz  && \
    rm -f jpeg.tar.gz   && \
    cd /tools/jpeg-9d   && \
    ./configure  && \
    make &&\
    make test

### WEBP
RUN echo " ====================  WebP  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O libwebp.tar.gz https://storage.googleapis.com/downloads.webmproject.org/releases/webp/libwebp-1.1.0-linux-x86-64.tar.gz  && \
    tar xvzf libwebp.tar.gz && \
    rm -f libwebp.tar.gz

### JPEG-XT
RUN echo " ====================  JPEG-XT  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O jpeg.zip https://jpeg.org/downloads/jpegxt/reference1367abcd89.zip && \
    unzip jpeg.zip -d jpeg && \
    rm -f jpeg.zip && \
    cd jpeg && \
    ./configure && \
    make final

### FLIF
RUN echo " ====================  FLIF  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O flif.zip  https://github.com/FLIF-hub/FLIF/archive/master.zip  && \
    unzip flif.zip && \
    rm -f flif.zip && \
    cd /tools/FLIF-master  && \
    make flif && \
    make install


### BPG
RUN echo " ====================  BPG  ==================== "

COPY libbpg-master.zip /tools

RUN mkdir -p /tools && \
    cd /tools && \
    unzip libbpg-master.zip && \
    rm -f libbpg-master.zip && \
    cd /tools/libbpg-master  && \
    make

#COPY mirrorlist /etc/pacman.d/
#
#COPY pacman.conf /etc/
#
#RUN pacman -Syu --noconfirm







