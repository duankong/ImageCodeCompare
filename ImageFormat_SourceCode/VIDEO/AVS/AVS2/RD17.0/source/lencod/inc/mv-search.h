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

#ifndef _MV_SEARCH_H_
#define _MV_SEARCH_H_

void BackwardPred(int *best_bw_ref, int *bw_mcost, int mode, int block, double lambda_motion, int max_ref,
                  int adjust_ref , int mb_nr, int bit_size); //ZHAOHAIWU

int check_mvd(int mvd_x, int mvd_y);
#if Mv_check_bug
	int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype, int dmh_x, int dmh_y);
#else
	int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype);
#endif
#if Mv_check_bug
	int check_mv_range_sym(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype, int ref, int dmh_x, int dmh_y);
#else
	int check_mv_range_sym(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype, int ref);
#endif

void SetMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  pmv[2], int  **refFrArr
                              , int  ***tmp_mv , int  ref_frame, int  mb_pix_x, int  mb_pix_y, int  blockshape_x, int  blockshape_y, int  ref,
                              int  direct_mv);

void SetSkipMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  **fw_refFrArr,
                                  int **bw_refFrArr, int  ***tmp_fwmv , int ***tmp_bwmv,
                                  int  ref_frame, int  mb_pix_x, int  mb_pix_y, int  blockshape_x, int  blockshape_y,  int  direct_mv);

#endif
