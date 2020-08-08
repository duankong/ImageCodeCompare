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
RUN     echo "\e[1;33;41m ==========  Start  ========= \e[0m"

COPY    sources.list /etc/apt/

ARG     PREFIX=/opt/ffmpeg

ENV     X265_VERSION=3.2

RUN     apt-get -y upgrade && \
        apt-get -yqq update && \
        apt-get install -yq --no-install-recommends ca-certificates expat libgomp1 cmake && \
        apt-get autoremove -y && \
        apt-get clean -y

RUN     buildDeps="wget \
                   vim \
                   unzip \
                   gcc \
                   g++ \
                   make \
                   python3 \
                   python3-pip \
                   imagemagick \
                   curl \
                   libsm6 \
                   libxext6 \
                   libxrender-dev \
                   libglib2.0-0 \
                   tree \
                   git \
                   libpng-dev \
                   libjpeg-dev \
                   libsdl1.2-dev \
                   libslang2-dev \
                   libsdl-image1.2-dev \
                   yasm \
                   libglu1 \
                   libxi6 \
                   automake \
                   libtool" && \
        apt-get -yqq update && \
        apt-get install -yq --no-install-recommends ${buildDeps} && \
        pip3 install scikit-image==0.15.0

# SQLite3 is to store metrics.
RUN apt-get install -y sqlite3 libsqlite3-dev

### JPEG2000
RUN echo "\e[1;33;41m ==========  Kakadu  ========= \e[0m"

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O kakadu.zip https://kakadusoftware.com/wp-content/uploads/2020/06/KDU805_Demo_Apps_for_Linux-x86-64_200602.zip && \
    unzip kakadu.zip -d kakadu && \
    rm -f kakadu.zip

ENV     LD_LIBRARY_PATH=/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602

### JPEG
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

### WEBP
RUN echo "\e[1;33;41m ==========  WebP  ========= \e[0m"

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O libwebp.tar.gz https://storage.googleapis.com/downloads.webmproject.org/releases/webp/libwebp-1.1.0-linux-x86-64.tar.gz  && \
    tar xvzf libwebp.tar.gz && \
    rm -f libwebp.tar.gz


### BPG
RUN echo "\e[1;33;41m ==========  BPG  ========= \e[0m"

COPY libbpg-master.zip /tools

RUN mkdir -p /tools && \
    cd /tools && \
#   wget -O bpg.tar.gz https://bellard.org/bpg/libbpg-0.9.8.tar.gz  && \
    unzip libbpg-master.zip && \
    rm -f libbpg-master.zip && \
    cd /tools/libbpg-master  && \
    make

### FLIF
RUN echo "\e[1;33;41m ==========  FLIF  ========= \e[0m"

RUN mkdir -p /tools && \
    cd /tools && \
    wget -O flif.zip  https://github.com/FLIF-hub/FLIF/archive/master.zip  && \
    unzip flif.zip && \
    rm -f flif.zip && \
    cd /tools/FLIF-master  && \
    apt-get install -yq libsdl2-dev && \
    make flif && \
    make install

### JPEG-XT
RUN mkdir -p /tools && \
    cd /tools && \
    wget -O jpeg.zip https://jpeg.org/downloads/jpegxt/reference1367abcd89.zip && \
    unzip jpeg.zip -d jpeg && \
    rm -f jpeg.zip && \
    cd jpeg && \
    ./configure && \
    make final

# Prepare
RUN echo "\e[1;33;41m ==========  Prepare  ========= \e[0m"

#ARG        PKG_CONFIG_PATH=/opt/ffmpeg/lib/pkgconfig
#ARG        LD_LIBRARY_PATH=/opt/ffmpeg/lib


### x265 http://x265.org/
RUN DIR=/tmp/x265 && \
    mkdir -p ${DIR} && \
    cd ${DIR} && \
    curl -sL https://download.videolan.org/pub/videolan/x265/x265_${X265_VERSION}.tar.gz  | \
    tar -zx && \
    cd x265_${X265_VERSION}/build/linux && \
    sed -i "/-DEXTRA_LIB/ s/$/ -DCMAKE_INSTALL_PREFIX=\${PREFIX}/" multilib.sh && \
    sed -i "/^cmake/ s/$/ -DENABLE_CLI=OFF/" multilib.sh && \
    ./multilib.sh && \
    make -C 8bit install && \
    rm -rf ${DIR}



