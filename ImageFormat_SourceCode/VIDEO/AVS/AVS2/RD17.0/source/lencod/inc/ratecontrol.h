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

#ifndef _RATECONTRAL_H_
#define _RATECONTRAL_H_

#include <stdio.h>
#include <stdlib.h>

typedef struct RCLCU {
    int     GlobeLCUNum;
    int     CurBits;
    int     PreBits;
    double  PreBpp;
    double  CurBpp;
    double  LCUTBL;
    double  LCUTBLdelta;
    double  LCUBuffer;
    double  LCUBufferError;
    double  LCUBufferDifError;
    double  LCUPreBufferError;
    int     LCUPreLCUQP;
    int     LCUCurLCUQP;
    int     LCUdeltaQP;
    double  LCUCurLambda;
} RCLCU;

typedef struct RateControl {
    int     LCUbaseQP;
    int     RConoff;
    int     LCURConoff;
    int     IntraPeriod;
    int     TotalFrames;
    int     GlobalImageNumber;
    int     CodedFrameNumber;
    int     ImageType;
    int     ImageQP;
    int     DeltaQP;
    int     GopLeaderDeltaQP;
    int     RcQP;
    int     RcMBQP;
    int     SumMBQP;
    int     NumMB;
    int     ImageBits;
    int     FrameWidth;
    int     FrameHeight;
    double  ImageBpp;
    double  Belta;
    double  GopBpp;
    int     GopAvgBaseQP;
    int     GopAvgBaseQPCount;
    double  GopAllKeyBpp;
    double  FirstBufferSizeLevel;
    double  IntraFrameBpp;
    double  InterFrameBpp;
    double  TargetBitrate;
    double  TargetBitPerPixel;
    double  TargetBufferLevel;
    double  DeltaBufferLevel;
    double  CurrentBufferSizeBpp;
    double  BufferError;
    double  BufferDifError;
    double  PreBufferError;
    double  LowestBufferSizeBpp;
    double  HighestBufferSizeBpp;
    int     GopFlag;
    FILE    *FReportFrame;
    FILE      *FReportLCU;
    RCLCU   *prclcu;
} RateControl;

void Init_LCURateControl(RateControl *prc, int NumUnitsLCU);
int  CalculateLCUDeltaQP(RateControl *prc);
void UpdataLCURateControl(RateControl *prc, int qp, double lambda, int bits, int NumLCU);
void Init_RateControl(RateControl *prc, int rconoff, int lcuonoff, int totalframes, int intraperiod, int targetbitrate,
                      int framerate, int iniQP, int w, int h, char *filename);
void Updata_RateControl(RateControl *prc, int framebits, int frameqp, int imgtype, int framenumber, int goplength);
int  CalculateGopDeltaQP_RateControl(RateControl *prc, int imgtype, int framenumber, int goplength);
void Init_FuzzyController(double ScaFactor);
RateControl *pRC;
RCLCU        *pRCLCU;

#endif
