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

#ifndef _LOOP_FILTER_H_
#define _LOOP_FILTER_H_

#include "commonVariables.h"
#include "defines.h"
#include "contributors.h"

void CreateEdgeFilter();//ZHAOHAIWU
void SetEdgeFilter();   // Falei LUO
void xSetEdgeFilter_One_SMB(unsigned int uiBitSize, unsigned int uiPositionInPic); //ZHAOHAIWU
void Copy_SMB_for_SAO(int smb_index, int mb_y, int mb_x, int smb_mb_height, int smb_mb_width);//ZHAOHAIWU
void Deblock_One_SMB(int smb_index, int mb_y, int mb_x, int smb_mb_height, int smb_mb_width);//ZHAOHAIWU

extern int saoclip[NUM_SAO_OFFSET][3];
void off_sao(SAOBlkParam *saoblkparam);
void copySAOParam_for_blk(SAOBlkParam *saopara_dst, SAOBlkParam *saopara_src);
void copySAOParam_for_blk_onecomponent(SAOBlkParam *saopara_dst, SAOBlkParam *saopara_src);
void Copy_frame_for_SAO();
void getMergeNeighbor(int smb_index, int pix_y, int pix_x, int smb_pix_width, int smb_pix_height,
                      int input_MaxsizeInBit, SAOBlkParam **rec_saoBlkParam, int *MergeAvail,
                      SAOBlkParam sao_merge_param[][NUM_SAO_COMPONENTS]);
void checkBoundaryAvail(int mb_y, int mb_x, int smb_pix_height, int smb_pix_width,
                        int *smb_available_left, int *smb_available_right, int *smb_available_up, int *smb_available_down,
                        int *smb_available_upleft, int *smb_available_upright, int *smb_available_leftdown, int *smb_available_rightdwon,
                        int filter_on);
void checkBoundaryProc(int pix_y, int pix_x, int smb_pix_height, int smb_pix_width, int comp,
                       int *smb_process_left, int *smb_process_right, int *smb_process_up, int *smb_process_down,
                       int *smb_process_upleft, int *smb_process_upright, int *smb_process_leftdown, int *smb_process_rightdwon,
                       int filter_on);
void checkBoundaryPara(int mb_y, int mb_x, int smb_pix_height, int smb_pix_width,
                       int *smb_process_left, int *smb_process_right, int *smb_process_up, int *smb_process_down, int *smb_process_upleft,
                       int *smb_process_upright, int *smb_process_leftdown, int *smb_process_rightdwon, int filter_on);
void SAOFrame(int input_MaxSizeInBit, SAOBlkParam **rec_saoBlkParam, int *slice_sao_on,
              int sample_bit_depth);
void SAO_on_smb(int smb_index, int pix_y, int pix_x, int smb_pix_width, int smb_pix_height,
                SAOBlkParam *saoBlkParam, int sample_bit_depth);
void SAO_on_block(SAOBlkParam *saoBlkParam, int compIdx, int smb_index, int smb_y, int smb_x, int smb_pix_height,
                  int smb_pix_width, int isLeftAvail, int isRightAvail, int isAboveAvail, int isBelowAvail, int isAboveLeftAvail,
                  int isAboveRightAvail, int isBelowLeftAvail, int isBelowRightAvail, int sample_bit_depth);

#endif
