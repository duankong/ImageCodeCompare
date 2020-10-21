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

#ifndef _MACROBLOCK_H_
#define _MACROBLOCK_H_

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"

int writeReferenceIndex(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int pos,
                        int *rate_top, int *rate_bot);
int writeCBPandDqp(codingUnit *currMB, int pos, int *rate_top, int *rate_bot, int uiPositionInPic);
int writeMVD(codingUnit *currMB, int pos, int *rate_top, int *rate_bot, int uiPositionInPic);

void encode_golomb_word(unsigned int symbol, unsigned int grad0, unsigned int max_levels, unsigned int *res_bits,
                        unsigned int *res_len);  //returns symbol coded. (might be cropped if max_levels is too small)
void encode_multilayer_golomb_word(unsigned int symbol, const unsigned int *grad, const unsigned int *max_levels,
                                   unsigned int *res_bits, unsigned int *res_len);  //terminate using a max_levels value of 30UL.
int writeSyntaxElement_GOLOMB(SyntaxElement *se, Bitstream *bitstream);
int estLumaCoeff8x8(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, int intra_mode,
                    int uiPositionInPic);
int estChromaCoeff8x8(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, int b8 , int uiPositionInPic);
void copyMBInfo(codingUnit *srcMB, codingUnit *dstMB);
int estCGLastRate(int iCG, BiContextTypePtr pCTX, int bitSize, int *CGLastX, int *CGLastY, int isChroma,
                  int ctxmode, codingUnit *currMB, int intraPredMode);
int estLastRunRate(int lastRun, BiContextTypePtr pCTX, int rank, int iCG, int isChroma, int ctxmode);
int estRunRate(int run, BiContextTypePtr pCTX, int pos, int iCG, int remainingPos, int isChroma, int ctxmode ,
               int bitSize);
int estLastPosRate(int lastPosX, int lastPosY, BiContextTypePtr pCTX, int isLastCG, int CGLastX, int CGLastY ,
                   int iCG, int ctxmode, unsigned int uiBitSize, int isChroma);
int estLastCGLastPosRate(int lastPosX, int lastPosY, BiContextTypePtr pCTX, int offset, int CGLastX, int CGLastY);
int estLevelRate(int absLevel, BiContextTypePtr pCTX, int rank, int pairsInCG, int iCG, int pos, int isChroma,
                 int bitSize);
int estSignRate(int level);
int estSigCGFlagRate(int sigCGFlag, BiContextTypePtr pCTX, int ctx);


#endif

