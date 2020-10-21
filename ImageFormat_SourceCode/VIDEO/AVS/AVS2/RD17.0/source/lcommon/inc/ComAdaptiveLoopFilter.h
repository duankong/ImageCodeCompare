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


#ifndef _COMADAPTIVELOOPFILTER_H_
#define _COMADAPTIVELOOPFILTER_H_
#include "commonVariables.h"
#define ALF_MAX_NUM_COEF       9
#define NO_VAR_BINS            16
#define LOG2_VAR_SIZE_H        2
#define LOG2_VAR_SIZE_W        2
#define NO_VAR_BINS            16
#define ALF_FOOTPRINT_SIZE     7
#define DF_CHANGED_SIZE        3
#define ALF_NUM_BIT_SHIFT      6
#define MAX_DOUBLE             1.7e+308
extern int weightsShape1Sym[ALF_MAX_NUM_COEF + 1];
extern unsigned int g_MaxSizeInbit;

void setFilterImage(byte *pDecY, byte **pDecUV, int stride, byte **imgY, byte *** imgUV, int img_width, int img_height);
void reconstructCoefficients(ALFParam *alfParam, int **filterCoeff);
void reconstructCoefInfo(int compIdx, ALFParam *alfParam, int **filterCoeff, int *varIndTab);
void checkFilterCoeffValue(int *filter, int filterLength, Boolean isChroma);
void copyALFparam(ALFParam *dst, ALFParam *src);
void filterOneCompRegion(byte *imgRes, byte *imgPad, int stride, Boolean isChroma, int yPos, int lcuHeight, int xPos,
                         int lcuWidth, int **filterSet, int *mergeTable,
                         byte **varImg, int sample_bit_depth, int isLeftAvail, int isRightAvail, int isAboveAvail, int isBelowAvail,
                         int isAboveLeftAvail, int isAboveRightAvail);
void ExtendPicBorder(byte *img, int iHeight, int iWidth, int iMarginY, int iMarginX, byte *imgExt);
int getLCUCtrCtx_Idx(int ctu, int numLCUInPicWidth, int numLCUInPicHeight, int NumCUInFrame, int compIdx,
                     Boolean **AlfLCUEnabled);
int check_filtering_unit_boundary_extension(int x, int y, int lcuPosX, int lcuPosY, int startX, int startY, int endX,
        int endY, int isAboveLeftAvail, int isLeftAvail, int isAboveRightAvail, int isRightAvail);
#endif
