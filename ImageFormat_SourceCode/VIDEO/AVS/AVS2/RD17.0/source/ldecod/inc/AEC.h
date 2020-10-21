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


#ifndef _AEC_H_
#define _AEC_H_

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/commonStructures.h"
void  init_contexts();

SyntaxInfoContexts *create_contexts_SyntaxInfo(void);

void delete_contexts_SyntaxInfo(SyntaxInfoContexts *enco_ctx);

void AEC_new_slice();

void readcuTypeInfo(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readcuTypeInfo_SFRAME(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readcuTypeInfo_SDIP(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *currMB, int uiPosition);
void readPdir(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readPdir_dhp(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readB8TypeInfo_dhp(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readB8TypeInfo(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readIntraPredMode(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readRefFrame(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readDmhMode(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readMVD(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);

int readBRPFlag(int uiBitSize);
void readBRPFlag_AEC(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);


void readWPM(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void read_b_dir_skip(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void read_p_skip_mode(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readCBP(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readRunLevelRef(SyntaxElement *se,  DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
unsigned int biari_decode_symbolW(DecodingEnvironmentPtr dep, BiContextTypePtr bi_ct1, BiContextTypePtr bi_ct2);
void readDquant(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readCIPredMode(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);

int  readSyntaxElement_AEC(SyntaxElement *se, DataPartition *this_dataPart, codingUnit *MB, int uiPosition);


int readSplitFlag(int uiBitSize);
void readSplitFlag_AEC(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readTrSize(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);

void readPartBiPredictionFlag(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *MB, int uiPosition);
void readPMVindex_AEC(SyntaxElement *se,   DecodingEnvironmentPtr dep_dp, codingUnit *currMB , int uiPosition);
int read_sao_mergeflag(int mergeleft_avail, int mergeup_avail, int uiPositionInPic);
void read_sao_mergeflag_AEC(SyntaxElement *se,  DecodingEnvironmentPtr dep_dp, codingUnit *currMB,
                            int uiPositionInPic);
int read_sao_mode(int uiPositionInPic);
void read_sao_mode_AEC(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *currMB, int uiPositionInPic);
int read_sao_offset(SAOBlkParam *saoBlkParam, int uiPositionInPic, int offsetTh, int *offset);
void read_sao_offset_AEC(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *currMB, int uiPositionInPic);
int read_sao_type(SAOBlkParam *saoBlkParam, int uiPositionInPic);
void read_sao_type_AEC(SyntaxElement *se, DecodingEnvironmentPtr dep_dp, codingUnit *currMB, int uiPositionInPic);
extern int saoclip[NUM_SAO_OFFSET][3];
extern int EO_OFFSET_INV__MAP[];
void readAlfCoeff(ALFParam *Alfp);
unsigned int readAlfLCUCtrl(ImageParameters *img, DecodingEnvironmentPtr dep_dp, int compIdx, int ctx_idx);

#endif
