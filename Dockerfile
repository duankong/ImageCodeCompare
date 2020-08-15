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

COPY mirrorlist /etc/pacman.d/
COPY pacman.conf /etc/
COPY bash.bashrc /etc/

RUN     pacman -Sy && \
#        pacman -Syu --noconfirm && \
        source /etc/bash.bashrc

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
                   pkg-config \
                   libjpeg\
                   ffmpeg" && \
        pacman -S --noconfirm ${buildDeps} && \
       	buildPacket="scikit-image" &&\
       	pip install ${buildPacket}


### JPEG2000
RUN echo " ====================  JPEG2000 --- Kakadu  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O kakadu.zip https://kakadusoftware.com/wp-content/uploads/2020/06/KDU805_Demo_Apps_for_Linux-x86-64_200602.zip && \
    unzip kakadu.zip -d kakadu && \
    rm -f kakadu.zip

ENV     LD_LIBRARY_PATH=/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602

### JPEG
RUN echo " ====================  JPEG --- Independent JPEG Group  ==================== "

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
RUN echo " ====================  WebP --- Goole  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O libwebp.tar.gz https://storage.googleapis.com/downloads.webmproject.org/releases/webp/libwebp-1.1.0-linux-x86-64.tar.gz  && \
    tar xvzf libwebp.tar.gz && \
    rm -f libwebp.tar.gz

### JPEG-XT
RUN echo " ====================  JPEG-XT --- Reference from jpeg.org ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O jpeg.zip https://jpeg.org/downloads/jpegxt/reference1367abcd89.zip && \
    unzip jpeg.zip -d jpeg && \
    rm -f jpeg.zip && \
    cd jpeg && \
    ./configure && \
    make final

### FLIF
RUN echo " ====================  FLIF --- http://flif.info/  ==================== "

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O flif.tar.gz  https://github.com/FLIF-hub/FLIF/archive/v0.3.tar.gz  && \
    tar xvf flif.tar.gz && \
    rm -f flif.tar.gz && \
    cd /tools/FLIF-0.3  && \
    make flif && \
    make install

### BPG
RUN echo " ====================  BPG --- bellard.org ==================== "

COPY libbpg-master.zip  /tools

RUN mkdir -p /tools && \
    cd /tools && \
    unzip libbpg-master.zip && \
    rm -f libbpg-master.zip && \
    cd /tools/libbpg-master  && \
    make

### AVIF
RUN echo " ====================  AVIF --- https://aomedia.org/ ==================== "

RUN     buildDeps="aom" && \
        pacman -S --noconfirm ${buildDeps}

### HEIF
RUN echo " ====================  HEIF --- strukturag/libheif  ==================== "

RUN     buildDeps="x265 \
                   libde265 \
                   cbindgen \
                   autoconf \
                   gdk-pixbuf2 \
                   libtool" && \
        pacman -S --noconfirm ${buildDeps}

RUN mkdir -p /tools && \
    cd /tools && \
    wget https://github.com/strukturag/libheif/releases/download/v1.7.0/libheif-1.7.0.tar.gz && \
    tar xvf libheif-1.7.0.tar.gz && \
    rm -f libheif-1.7.0.tar.gz && \
    cd /tools/libheif-1.7.0  && \
    ./autogen.sh && \
     autoreconf -ivf &&\
    mkdir build && \
    cd build && \
    ../configure --prefix=/tools/libheif-1.7.0/build && \
    make && \
    make install

### OPENJPEG
RUN echo " ====================  OPENJPEG --- uclouvain/openjpeg  ==================== "

RUN pacman -Sy openjpeg2 --noconfirm


