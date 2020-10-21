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

#ifndef _RDOQH_
#define _RDOQH_

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "block.h"
#include "rdopt_coding_state.h"
#include "codingUnit.h"

typedef struct level_data_struct {
    int     level[3];    // candidate levels
    int     levelDouble; // coefficient before quantization
    int     levelDouble2; // coefficient before quantization
    double  errLevel[3]; // quantization errors of each candidate
    int     noLevels;    // number of candidate levels
    int     xx;          // x coordinate of the coeff in transform block
    int     yy;          // y coordinate of the coeff in transform block
    int     scanPos;     // position in transform block zig-zag scan order
} levelDataStruct;


void quant_init(int qp, int mode, int **curr_blk, unsigned int uiBitSize, levelDataStruct *levelData);
void choose_level(levelDataStruct *levelData, int **curr_blk, unsigned int uiBitSize, int mode, double lambda,
                  int isChroma, unsigned int uiPositionInPic);
void rdoq_block(int qp, int mode, int **curr_blk, unsigned int uiBitSize, int isChroma,
                unsigned int uiPositionInPic, int intraPredMode, codingUnit *currMB);
int  estimate_bits(int **Quant_Coeff, unsigned int uiBitSize, int uiPositionInPic);

#endif
