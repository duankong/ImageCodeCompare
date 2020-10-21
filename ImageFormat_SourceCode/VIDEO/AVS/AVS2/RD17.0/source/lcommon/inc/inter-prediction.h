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

#ifndef _INTER_PREDICTION_H_
#define _INTER_PREDICTION_H_

#include "commonVariables.h"
#include "defines.h"
#include "contributors.h"


/////////////////////////////////////////////////////////////////////////////
/// external function declaration
/////////////////////////////////////////////////////////////////////////////
#if Mv_Clip
int scale_motion_vector(int motion_vector, int currblkref, int neighbourblkref,
	int ref, int delta2);  //, int currsmbtype, int neighboursmbtype, int block_y_pos, int curr_block_y,  int direct_mv);
#else
int scale_motion_vector(int motion_vector, int currblkref, int neighbourblkref,
                        int ref);  //, int currsmbtype, int neighboursmbtype, int block_y_pos, int curr_block_y,  int direct_mv);
#endif
void scalingMV(int *cur_mv_x, int *cur_mv_y, int curT, int ref_mv_x, int ref_mv_y, int refT, int factor_sign);

void get_reference_list_info(char *str);

#if MV_SCALE
int scale_mv(int mv, int dist_dst, int dist_src);
int scale_mv_y1(int mvy, int dist_dst, int dist_src);
int scale_mv_y2(int mvy, int dist_dst, int dist_src);
int scale_motion_vector_y1(int mvy, int currblkref, int neighbourblkref, int ref);
int scale_motion_vector_y2(int mvy, int currblkref, int neighbourblkref, int ref);
int scale_mv_direct(int mv, int dist_dst, int dist_src);
void scale_mv_direct_x(int mv_x, int dist2, int dist4, int dist5, int *Fwmv_x, int *Bwmv_x);
void scale_mv_direct_y(int mv_y, int dist1, int dist2, int dist3, int dist4, int dist5, int *Fwmv_y, int *Bwmv_y);
int derive_dv(int neigh_mv);
#endif

#if HALF_PIXEL_COMPENSATION || HALF_PIXEL_CHROMA
int calculate_distance(int blkref, int fw_bw);
#endif

#endif
