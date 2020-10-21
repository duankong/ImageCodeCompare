/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2002-2016, Audio Video coding Standard Workgroup of China
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of Audio Video coding Standard Workgroup of China
*    nor the names of its contributors maybe used to endorse or promote products
*    derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "../../lcommon/inc/commonStructures.h"

#ifndef _TDRDO_H_
#define _TDRDO_H_

#define MAXBLOCKSIZE  64
#define WORKBLOCKSIZE 64
#define SEARCHRANGE   64

typedef struct Block {
    unsigned int    BlockWidth;
    unsigned int    BlockHeight;
    unsigned int    OriginX;
    unsigned int    OriginY;
    byte            lume[MAXBLOCKSIZE * MAXBLOCKSIZE];  // 4096==64*64
    byte            cr[MAXBLOCKSIZE * MAXBLOCKSIZE / 4]; // 1024==4096/4
    byte            cb[MAXBLOCKSIZE * MAXBLOCKSIZE / 4]; // 1024==4096/4
} Block;

typedef struct Frame {
    unsigned int    FrameWidth;
    unsigned int    FrameHeight;
    unsigned int    nStrideY;
    unsigned int    nStrideC;
    byte     *base;
    byte     *Y;
    byte     *U;
    byte     *V;
} Frame;

typedef struct BlockDistortion {
    unsigned int    GlobalBlockNumber;
    unsigned short  BlockNumInHeight;
    unsigned short  BlockNumInWidth;
    unsigned short  BlockWidth;
    unsigned short  BlockHeight;
    unsigned short  OriginX;
    unsigned short  OriginY;
    unsigned short  SearchRange;
    short           MVx;
    short           MVy;
    double          MSE;
    double          MAD;
    double          MVL;
    short           BlockQP;
    double          BlockLambda;
    short           BlockType;
} BlockDistortion, BD;

typedef struct FrameDistortion {
    unsigned int    FrameNumber;
    unsigned int    BlockSize;
    unsigned int    CUSize;
    unsigned int    TotalNumOfBlocks;
    unsigned int    TotalBlockNumInHeight;
    unsigned int    TotalBlockNumInWidth;
    BD              *BlockDistortionArray;
    struct FrameDistortion *subFrameDistortionArray;
} FrameDistortion, FD;

typedef struct DistortionList {
    unsigned int    TotalFrameNumber;
    unsigned int    FrameWidth;
    unsigned int    FrameHeight;
    unsigned int    BlockSize;
    FD              *FrameDistortionArray;
} DistortionList, DL;

DL *CreatDistortionList(unsigned int totalframenumber, unsigned int w, unsigned int h, unsigned int blocksize,
                        unsigned int cusize);
void DestroyDistortionList(DL *SeqD);
void MotionDistortion(FD *currentFD, Frame *FA, Frame *FB, unsigned int searchrange);
void StoreLCUInf(FD *curRealFD, int LeaderBlockNumber, int cuinwidth, int iqp, double dlambda, int curtype);
void CaculateKappaTableLDP(DL *omcplist, DL *realDlist, int keyframenum, int FrameQP);
void AdjustLcuQPLambdaLDP(FD *curOMCPFD, int LeaderBlockNumber, int cuinwidth, int *pQP, double *plambda);
FD *SearchFrameDistortionArray(DL *omcplist, int FrameNumberInFile, int StepLength, int IntraPeriod);

Frame *porgF, *ppreF, *precF, *prefF;

DL *OMCPDList;
DL *RealDList;
FD *pOMCPFD, *pRealFD, *subpOMCPFD ;

int     StepLength;
double  AvgBlockMSE;
double  *KappaTable;
double  *KappaTable1;
double  GlobeLambdaRatio;
int     GlobeFrameNumber;
int     CurMBQP;
int     QpOffset[32];
int     GroupSize;

#if AQPO
int     AQPStepLength;
double  LogMAD[32];
double  preLogMAD[32];
int     AQPoffset[32];
int     MaxQPoffset;
int     OffsetIndex;
int     FNIndex;
#if AQPOM3762
int     GopQpbase;
#else
int     subGopqpbase;
#endif
int     preGopQPF[4];
#endif

#endif
