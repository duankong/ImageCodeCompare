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




/*
*************************************************************************************
* File name:
* Function:
*
*************************************************************************************
*/

#ifndef _AVS_TRANSFORM_H_
#define _AVS_TRANSFORM_H_

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"


// functions

int  find_sad_8x8(int iMode, int iSizeX, int iSizeY, int iOffX, int iOffY, int resiY[MAX_CU_SIZE][MAX_CU_SIZE]);
int  sad_hadamard(int iSizeX, int iSizeY, int iOffX, int iOffY, int resiY[MIN_CU_SIZE][MIN_CU_SIZE]);

int  sad_hadamard4x4(int resiY[MIN_CU_SIZE][MIN_CU_SIZE]);

void xUpdateCandList(int uiMode, double uiCost, int uiFullCandNum, int *CandModeList, double *CandCostList);
int calcHAD(int **pi, int iWidth, int iHeight);
int  Mode_Decision_for_AVS_IntraBlocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic,
                                       int b8, double lambda, int *min_cost);
double RDCost_for_AVSIntraBlocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic,
                                 int *nonzero, int b8, int ipmode, double lambda, double  min_rdcost, int *mostProbableMode, int isChroma);
void quantization(int qp, int mode, int b8, int **curr_blk,  unsigned int uiBitSize, unsigned int uiPositionInPic,
                  codingUnit *currMB, int isChroma, int intraPredMode);
int  inverse_quantization(int qp, int mode, int b8, int **curr_blk, int scrFlag, int *cbp,  unsigned int uiBitSize,
                          unsigned int uiPositionInPic, codingUnit *currMB, int isChroma);
void inverse_transform(int b8, int **curr_blk, unsigned int uiBitSize, unsigned int uiPositionInPic,
                       codingUnit *currMB, int isChroma);

extern short IQ_SHIFT[80];
extern unsigned short IQ_TAB[80];
extern unsigned short Q_TAB[80];

#endif // _AVS_TRANSFORM_H_
