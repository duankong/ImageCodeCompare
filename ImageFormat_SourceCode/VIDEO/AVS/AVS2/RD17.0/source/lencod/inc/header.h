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
* File name: header.h
* Function: Prototypes for header.c
*
*************************************************************************************
*/



#ifndef _HEADER_H_
#define _HEADER_H_
#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
int bbv_check_times;

int slice_vertical_position_extension;
int slice_horizontal_position_extension;    //added by mz, 2008.04

int picture_coding_type;
int frame_rate;

int marker_bit;

int  SliceHeader(int slice_nr, int slice_qp);
int  IPictureHeader(int frame);
int  PBPictureHeader();
int  CheckInterlaceSetting(int idx);
void write_terminating_bit(unsigned char);

void DeblockFrame(byte **imgY, byte ***imgUV);
void getStatisticsSAO_one_SMB(int smb_index, int mb_y, int mb_x, int smb_mb_height, int smb_mb_width,
                              SAOStatData **saostatData);
void getParaSAO_one_SMB(int smb_index, int mb_y, int mb_x, int smb_mb_height, int smb_mb_width, int *slice_sao_on,
                        SAOStatData **saostatData, SAOBlkParam *saoBlkParam, SAOBlkParam **rec_saoBlkParam, int *num_off_sao,
                        double sao_labmda);
void writeParaSAO_one_SMB(int smb_index, int mb_y, int mb_x, int smb_mb_height, int smb_mb_width, int *slice_sao_on,
                          SAOBlkParam *saoBlkParam);

#endif
