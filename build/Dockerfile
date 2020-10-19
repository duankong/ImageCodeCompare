# ffmpeg - http://ffmpeg.org/download.html
#
# From https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
#
# https://hub.docker.com/r/jrottenberg/ffmpeg/
#
#

FROM    archlinux:20200705 AS base

WORKDIR     /image_test          

# Start
RUN     echo " ====================  START  ==================== "

COPY    mirrorlist /etc/pacman.d/
COPY    pacman.conf /etc/
COPY    bash.bashrc /etc/

RUN     pacman -Sy && \
        pacman -Syu --noconfirm && \
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
# ============================== VMAFLIB ==============================
### VMAF library
RUN     echo " ====================  METRIC || 0 VMAFlib --- Netflix/vmaf  ==================== "
RUN     buildDeps="meson" && \
        pacman -Sy --noconfirm ${buildDeps}
RUN     buildPacket="Cython \
                     argparse \
                     pypng" &&\
       	pip install ${buildPacket}
RUN     DIR=/tmp/vmaf && \
        mkdir -p ${DIR} && \
        cd ${DIR} && \
        wget -O vmaf.zip https://github.com/Netflix/vmaf/archive/v1.5.2.zip && \
        unzip vmaf.zip && \
        rm -f vmaf.zip && \
        cd vmaf-1.5.2 && \
        make && \
        make install && \
        cd python && \
        python3 setup.py install
# ============================== IMAGE ==============================
### JPEG(IJG)
RUN     echo " ====================  IMAGE || 1 JPEG --- Independent JPEG Group  ==================== "
RUN     mkdir -p /tools && \
        cd /tools && \
        wget -O jpeg.tar.gz http://www.ijg.org/files/jpegsrc.v9d.tar.gz && \
        tar  xzvf jpeg.tar.gz  && \
        rm -f jpeg.tar.gz   && \
        cd /tools/jpeg-9d   && \
        ./configure  && \
        make &&\
        make test
### JPEG-XT
RUN     echo " ====================  IMAGE || 1 JPEG --- JPEG-XT --- Reference from jpeg.org  ==================== "
RUN     mkdir -p /tools && \
        cd /tools && \
        wget -O jpeg.zip https://jpeg.org/downloads/jpegxt/reference1367abcd89.zip && \
        unzip jpeg.zip -d jpeg && \
        rm -f jpeg.zip && \
        cd jpeg && \
        ./configure && \
        make final
### JPEG2000
RUN     echo " ====================  IMAGE || 2 JPEG2000 --- Kakadu  ==================== "
RUN     mkdir -p /tools && \
        cd /tools && \
        wget -O kakadu.zip https://kakadusoftware.com/wp-content/uploads/2020/06/KDU805_Demo_Apps_for_Linux-x86-64_200602.zip && \
        unzip kakadu.zip -d kakadu && \
        rm -f kakadu.zip
ENV     LD_LIBRARY_PATH=/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602

### OPENJPEG
RUN     echo " ====================  IMAGE || 3 OPENJPEG --- uclouvain/openjpeg  ==================== "
RUN     pacman -Sy openjpeg2 --noconfirm

### FLIF
RUN     echo " ====================  IMAGE || 4 FLIF --- http://flif.info/  ==================== "
RUN     mkdir -p /tools && \
        cd /tools && \
        wget -O flif.tar.gz  https://github.com/FLIF-hub/FLIF/archive/v0.3.tar.gz  && \
        tar xvf flif.tar.gz && \
        rm -f flif.tar.gz && \
        cd /tools/FLIF-0.3  && \
        make flif && \
        make install
### WEBP
RUN     echo " ====================  IMAGE || 5 WebP --- Goole  ==================== "
RUN     mkdir -p /tools && \
        cd /tools && \
        wget -O libwebp.tar.gz https://storage.googleapis.com/downloads.webmproject.org/releases/webp/libwebp-1.1.0-linux-x86-64.tar.gz  && \
        tar xvzf libwebp.tar.gz && \
        rm -f libwebp.tar.gz
### BPG
RUN     echo " ====================  IMAGE || 6 BPG --- bellard.org ==================== "
COPY    libbpg-master.zip  /tools
RUN     mkdir -p /tools && \
        cd /tools && \
        unzip libbpg-master.zip && \
        rm -f libbpg-master.zip && \
        cd /tools/libbpg-master  && \
        make
# LIBAOM
RUN     echo " ====================  IMAGE || 9 LIBAVIF  ==================== "
RUN     buildDeps="ninja \
                   nasm" && \
        pacman -Sy --noconfirm ${buildDeps}
RUN     mkdir -p /tools && \
        cd /tools && \
        git clone https://github.com/AOMediaCodec/libavif
COPY    aom-refs_tags_v2.0.0.tar.gz /tools/libavif/ext
RUN     cd /tools/libavif/ext && \
        mkdir /tools/libavif/ext/aom && \
        tar xvzf  aom-refs_tags_v2.0.0.tar.gz -C aom && \
        cd aom && mkdir build.libavif && cd build.libavif && \
        cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DENABLE_DOCS=0 -DENABLE_EXAMPLES=0 -DENABLE_TESTDATA=0 -DENABLE_TESTS=0 -DENABLE_TOOLS=0 .. && \
        ninja && \
        cd /tools/libavif && \
        mkdir build && cd build && \
        cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DAVIF_CODEC_AOM=ON -DAVIF_LOCAL_AOM=ON -DAVIF_CODEC_DAV1D=OFF -DAVIF_LOCAL_DAV1D=OFF -DAVIF_CODEC_RAV1E=OFF -DAVIF_LOCAL_RAV1E=OFF -DAVIF_CODEC_LIBGAV1=OFF -DAVIF_LOCAL_LIBGAV1=OFF -DAVIF_BUILD_TESTS=1 -DAVIF_BUILD_APPS=1 && \
        ninja
# ============================== VIDEO ==============================
### HEVC (HM)
RUN     echo " ====================  VIDEO || 1 HEVC(HM) --- svn_HEVCSoftware/HM-16.20+SCM-8.8  ==================== "
RUN     pacman -Sy --noconfirm hm svn
RUN     cd /tools && \
        svn checkout https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/tags/HM-16.20+SCM-8.8/
COPY    makefile.base  /tools/HM-16.20+SCM-8.8/build/linux/common
RUN     cd  /tools/HM-16.20+SCM-8.8/build/linux && \
        make release_highbitdepth
### AOM
RUN     echo " ====================  VIDEO || 2 AOM --- https://aomedia.org/ ==================== "
RUN     buildDeps="aom" && \
        pacman -Syu --noconfirm ${buildDeps}
### AVS3
RUN     echo " ====================  VIDEO || 3 AVS3 --- https://github.com/uavs3  ==================== "
COPY    uavs3e-master.zip  /tools
COPY    uavs3d-master.zip  /tools

RUN     mkdir -p /tools && \
        cd /tools && \
        unzip uavs3e-master.zip && \
        cd /tools/uavs3e-master/build && \
        mkdir linux && cd linux && \
        cmake ../.. && make && make install && \
        cd /tools && \
        unzip uavs3d-master.zip && \
        cd /tools/uavs3d-master/build && \
        mkdir linux && cd linux && \
        cmake ../.. && make && make install
COPY    config.cfg /tools/uavs3e-master/build/linux

### HEVC (HM)
RUN     echo " ====================  VIDEO || 4 VVC(VTC) --- /jvet/VVCSoftware_VTM/VTM-10.0  ==================== "

COPY    VVCSoftware_VTM-VTM-10.0.zip /tools

RUN     cd /tools && \
        unzip VVCSoftware_VTM-VTM-10.0.zip && \
        cd VVCSoftware_VTM-VTM-10.0 && \
        mkdir build && cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make -j 4

### rav1e
RUN     echo " ====================  VIDEO || 5 rav1e ==================== "
RUN     mkdir -p /tools/rav1e
COPY    rav1e-0.3.4-linux.tar.gz /tools/rav1e

RUN     cd /tools/rav1e  && \
        tar xvzf rav1e-0.3.4-linux.tar.gz  && \
        rm -f rav1e-0.3.4-linux.tar.gz

### SVT-AV1
RUN     echo " ====================  VIDEO || 6 SVT-AV1 ==================== "
COPY    SVT-AV1-0.8.5.zip /tools

RUN     cd /tools  && \
        unzip SVT-AV1-0.8.5.zip  && \
        cd /tools/SVT-AV1-0.8.5/Build/linux && \
        ./build.sh release
### Libvpx
RUN     echo " ====================  VIDEO || 7 VP8-VP9 ==================== "
COPY    libvpx-1.9.0.zip /tools

RUN     cd /tools  && \
        unzip libvpx-1.9.0.zip  && \
        cd libvpx-1.9.0 && mkdir build_dk && cd build_dk && \
        ../configurer --as=nasm && \
        make -j 4

### SVT-VP9
RUN     echo " ====================  VIDEO || 8 SVT-VP9 ==================== "
COPY    SVT-VP9-0.2.2.zip /tools

RUN     cd /tools  && \
        unzip SVT-VP9-0.2.2.zip  && \
        cd /tools/SVT-VP9-0.2.2/Build/linux && \
        ./build.sh --release -j 4
### CLEAR
RUN     rm -f /tools/*.zip
RUN     rm -f /tools/*.gz





#### HEIF
#RUN echo " ====================  HEIF --- strukturag/libheif  ==================== "
#
#RUN     buildDeps="x265 \
#                   libde265 \
#                   cbindgen \
#                   autoconf \
#                   gdk-pixbuf2 \
#                   libtool" && \
#        pacman -S --noconfirm ${buildDeps}
#
#RUN mkdir -p /tools && \
#    cd /tools && \
#    wget https://github.com/strukturag/libheif/releases/download/v1.7.0/libheif-1.7.0.tar.gz && \
#    tar xvf libheif-1.7.0.tar.gz && \
#    rm -f libheif-1.7.0.tar.gz && \
#    cd /tools/libheif-1.7.0  && \
#    ./autogen.sh && \
#     autoreconf -ivf &&\
#    mkdir build && \
#    cd build && \
#    ../configure --prefix=/tools/libheif-1.7.0/build && \
#    make && \
#    make install








