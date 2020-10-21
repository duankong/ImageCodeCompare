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
#include "commonVariables.h"
#include "defines.h"
#include "contributors.h"

#define shift_trans 10   // 8-pt trans: 9 + 17 - 16 = 10
#define shift_out   15   // 8-pt inv_trans: 13 + (28-17) - 9 = 15#
#define PIXEL_BIT       8
#define RESID_BIT       (PIXEL_BIT + 1)
#define LIMIT_BIT       16
#define FACTO_BIT       5

/////////////////////////////////////////////////////////////////////////////
/// external function declaration
/////////////////////////////////////////////////////////////////////////////
void partialButterfly4(int **src, int **dst, int iNumRows);
void partialButterflyInverse4(int **src, int **dst, int iNumRows);

void partialButterfly8(int **src, int **dst, int iNumRows);

void partialButterflyInverse8(int **src, int **dst, int iNumRows);

void partialButterfly16(int **src, int **dst, int iNumRows);

void partialButterflyInverse16(int **src, int **dst, int iNumRows);

void partialButterfly32(int **src, int **dst, int iNumRows);

void partialButterflyInverse32(int **src, int **dst, int iNumRows);

void  wavelet64(int **curr_blk);
void  wavelet_NSQT(int **curr_blk, int is_Hor);
void  inv_wavelet_B64(int **curr_blk);
void  inv_wavelet_NSQT(int **curr_blk, int is_Hor);




void array_shift_clip(int **src, int shift, int iSizeY, int iSizeX, int bit_depth);
void transform_B8(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma, int secT_enabled,
                  int sample_bit_depth);
void transform_NSQT(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma, int secT_enabled,
                    int sample_bit_depth);


void quant_B8(int qp, int mode, int **curr_blk, int iSizeX, int iSizeY, int uiBitSize, int sample_bit_depth);



void inv_transform_B8(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma, int secT_enabled,
                      int sample_bit_depth);


void inv_transform_NSQT(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma,
                        int secT_enabled, int sample_bit_depth);


extern short IQ_SHIFT[80];
extern unsigned short IQ_TAB[80];
extern unsigned short Q_TAB[80];
