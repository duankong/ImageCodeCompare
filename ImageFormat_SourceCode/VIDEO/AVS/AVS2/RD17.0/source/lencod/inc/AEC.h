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
#include "../../lencod/inc/EncAdaptiveLoopFilter.h"
#if MB_DQP
void AEC_new_slice();
void lastdqp2tempdqp();
void tempdqp2lastdqp();
#endif

SyntaxInfoContexts *create_contexts_SyntaxInfo(void);   //ZHAOHAIWU
void delete_contexts_SyntaxInfo(SyntaxInfoContexts *codec_ctx);   //ZHAOHAIWU

void  init_contexts();
// AEC
void arienco_start_encoding(EncodingEnvironmentPtr eep, unsigned char *code_buffer,
                            int *code_len, /* int *last_startcode, */int slice_type);
int  arienco_bits_written(EncodingEnvironmentPtr eep);
void arienco_done_encoding(EncodingEnvironmentPtr eep);

void biari_init_context_logac(BiContextTypePtr ctx);
int biari_encode_symbol_est(unsigned char  symbol, BiContextTypePtr bi_ct);
int biari_encode_symbolW_est(unsigned char  symbol, BiContextTypePtr bi_ct1,  BiContextTypePtr bi_ct2);
int biari_encode_symbol_eq_prob_est(unsigned char  symbol);
int biari_encode_symbol_final_est(unsigned char symbol);
int estRunLevelRef(codingUnit *currMB, int context);
void biari_encode_symbol(EncodingEnvironmentPtr eep, unsigned char  symbol, BiContextTypePtr bi_ct);

void biari_encode_symbol_eq_prob(EncodingEnvironmentPtr eep, unsigned char  symbol);
void biari_encode_symbol_final(EncodingEnvironmentPtr eep, unsigned char  symbol);
SyntaxInfoContexts *create_contexts_SyntaxInfo(void);
void delete_contexts_SyntaxInfo(SyntaxInfoContexts *enco_ctx);

int  writeSyntaxElement_AEC(codingUnit *currMB, SyntaxElement *se, DataPartition *this_dataPart, int uiPosition);
void writeCuTypeInfo(codingUnit *, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeCuTypeInfo_SFRAME(codingUnit *, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeCuTypeInfo_SDIP(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeIntraPredMode(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeB8TypeInfo(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeRefFrame(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeMVD_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);

int writeBRPFlag(int splitFlag, codingUnit *currMB, int uiBitSize);
void writeBRPFlag_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);


void writeDmhMode(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writePMVindex_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);

void writeTrSize(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);


void writePdir(codingUnit *, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writePdir_dhp(codingUnit *, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeB8TypeInfo_dhp(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeWPM(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void write_b_dir_skip(codingUnit *, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void write_p_skip_mode(codingUnit *, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeCBP(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);

#if MB_DQP
void writeDqp(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
#endif

void writeRunLevelRef(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);

void writeCIPredMode(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
void writeCbpBit(int b8, int bit, int cbp, codingUnit *currMB, int inter, EncodingEnvironmentPtr eep_dp,
                 int uiPosition);
int writeSplitFlag(int splitFlag, codingUnit *currMB, int uiBitSize);
void writeSplitFlag_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);


void writePartBiPredictionFlag(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition);
int write_sao_mergeflag(int mergeleft_avail, int mergeup_avail, SAOBlkParam *saoBlkParam, int uiPositionInPic);
void write_sao_mergeflag_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic);
int write_sao_mode(SAOBlkParam *saoBlkParam, int uiPositionInPic);
void write_sao_mode_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic);
int write_sao_offset(SAOBlkParam *saoBlkParam, int uiPositionInPic, int offsetTh);
void write_sao_offset_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic);
int write_sao_type(SAOBlkParam *saoBlkParam, int uiPositionInPic);
void write_sao_type_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic);
extern int saoclip[NUM_SAO_OFFSET][3];
extern int EO_OFFSET_MAP[];
int writeAlfLCUCtrl(int iflag, DataPartition *this_dataPart, int compIdx, int ctx_idx);
void writeAlfCoeff(ALFParam *Alfp);
#endif  // AEC_H

