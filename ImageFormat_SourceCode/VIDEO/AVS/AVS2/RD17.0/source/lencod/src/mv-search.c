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
#include "../../lcommon/inc/contributors.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/memalloc.h"
#include "../../lcommon/inc/inter-prediction.h"
#include "../../lcommon/inc/block_info.h"
#include "global.h"
#include "mv-search.h"
#include "refbuf.h"

#include "block.h"


#include "fast_me.h"
#include "../../lcommon/inc/defines.h"


// These procedure pointers are used by motion_search() and one_eigthpel()
static pel_t (*PelY_14)(pel_t **, int, int);

// Statistics, temporary
int    max_mvd;
int   *spiral_search_x;
int   *spiral_search_y;
int   *mvbits;
int   mvpidxbits[MAX_MVP_CAND_NUM][MAX_MVP_CAND_NUM + 1];
int   *refbits;
int   *byte_abs;
int ** *motion_cost;

int ** *motion_cost_sym;
int ** *motion_cost_dual;

int  *int_motion_cost;


#define MEDIAN(a,b,c)  (a + b + c - min(a, min(b, c)) - max(a, max(b, c)));  //jlzheng 6.30

extern int g_Up_Right_Avail_Matrix64[16][16];
extern int g_Up_Right_Avail_Matrix32[8][8];
extern int g_Up_Right_Avail_Matrix16[4][4];
extern int g_Up_Right_Avail_Matrix8[2][2];

/*
******************************************************************************
*  Function: Determine the MVD's value (1/4 pixel) is legal or not.
*  Input:
*  Output:
*  Return: 0: out of the legal mv range; 1: in the legal mv range
*  Attention:
*  Author: xiaozhen zheng, 20071009
******************************************************************************
*/
int check_mvd(int mvd_x, int mvd_y)
{

    if (mvd_x > 4095 || mvd_x < -4096 || mvd_y > 4095 || mvd_y < -4096) {
        return 0;
    }

    return 1;
}


/*
******************************************************************************
*  Function: Determine the mv's value (1/4 pixel) is legal or not.
*  Input:
*  Output:
*  Return: 0: out of the legal mv range; 1: in the legal mv range
*  Attention:
*  Author: xiaozhen zheng, 20071009
******************************************************************************
*/
#if Mv_check_bug
int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype, int dmh_x, int dmh_y)
#else
int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype)
#endif
{
    int curr_max_x, curr_min_x, curr_max_y, curr_min_y;
#if Mv_check_bug
	int ddx,ddy;
	int pos_xx,pos_yy;
	int mv_xx,mv_yy;
#endif
    int bx[9] = {MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE / 2, MIN_BLOCK_SIZE * 3 / 2, MIN_BLOCK_SIZE};
    int by[9] = {MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE / 2, MIN_BLOCK_SIZE * 3 / 2, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE};

#if Mv_Rang   //M3959 he-yuan.lin@mstarsemi.com
	int is_x_subpel = 0;
	int is_y_subpel = 0;
    int x_size = bx[blocktype];
    int y_size = by[blocktype];
#if Mv_check_bug
	ddx = (pix_x*4 + mv_x + dmh_x)&3;
	ddy = (pix_y*4 + mv_y + dmh_y)&3;
	pos_xx = (pix_x*4 + mv_x + dmh_x - ddx)/4;
	pos_yy = (pix_y*4 + mv_y + dmh_y - ddy)/4;
	mv_xx = pos_xx*4 - pix_x*4;
	mv_yy = pos_yy*4 - pix_y*4;
	if(ddx !=0)
		is_x_subpel = 1;

	if(ddy !=0)
		is_y_subpel = 1;
#else
	if(mv_x % 4 !=0)
		is_x_subpel = 1;

	if(mv_y % 4 !=0)
		is_y_subpel = 1;
#endif

    if( (pix_x % (1 << uiBitSize)) != 0 && blocktype == 6)
        x_size = bx[7];
    if( (pix_y % (1 << uiBitSize)) != 0 && blocktype == 4)
        y_size = bx[5];
#endif

#if Mv_Rang
    curr_max_x = (img->width - (pix_x + x_size * (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)) + is_x_subpel * 4)) * 4
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#else
    curr_max_x = (img->width - (pix_x + bx[blocktype] * (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)))) * 4
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#endif

#if Mv_Rang
	  curr_min_x = pix_x * 4 + /*(16<<(uiBitSize-4))*/64 * 4 - is_x_subpel * 3 * 4;
#else
    curr_min_x = pix_x * 4 + /*(16<<(uiBitSize-4))*/64 * 4;
#endif

#if Mv_Rang
    curr_max_y = (img->height - (pix_y + y_size * (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)) + is_y_subpel * 4)) * 4
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#else
    curr_max_y = (img->height - (pix_y + by[blocktype] * (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)))) * 4
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#endif

#if Mv_Rang
	curr_min_y = pix_y * 4 + /*(16<<(uiBitSize-4))*/64 * 4 - is_y_subpel * 3 * 4;
#else
    curr_min_y = pix_y * 4 + /*(16<<(uiBitSize-4))*/64 * 4;
#endif

#if Mv_check_bug
	if (mv_xx > curr_max_x || mv_xx < -curr_min_x || mv_xx > hc->Max_H_MV || mv_xx < hc->Min_H_MV) {
		return 0;
	}

	if (mv_yy > curr_max_y || mv_yy < -curr_min_y || mv_yy > hc->Max_V_MV || mv_yy < hc->Min_V_MV) {
		return 0;
	}
#else
	if (mv_x > curr_max_x || mv_x < -curr_min_x || mv_x > hc->Max_H_MV || mv_x < hc->Min_H_MV) {
		return 0;
	}

	if (mv_y > curr_max_y || mv_y < -curr_min_y || mv_y > hc->Max_V_MV || mv_y < hc->Min_V_MV) {
		return 0;
	}
#endif
    
    return 1;
}

/*
******************************************************************************
*  Function: Determine the forward and backward mvs' value (1/4 pixel) is legal or not.
*  Input:
*  Output:
*  Return: 0: out of the legal mv range; 1: in the legal mv range
*  Attention:
*  Author: xiaozhen zheng, 20071009
******************************************************************************
*/
#if Mv_check_bug
int check_mv_range_sym(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype, int ref, int dmh_x, int dmh_y)
#else
int check_mv_range_sym(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype, int ref)
#endif
{
    int bw_mvx, bw_mvy;
    int delta_P, TRp, DistanceIndexFw, DistanceIndexBw, delta_PB;
#if !Mv_check_bug
    int curr_max_x, curr_min_x, curr_max_y, curr_min_y;
#endif
    int snd_ref = ref == 0 ? 1 : 0;
#if !Mv_check_bug
    int bx[9] = {MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE / 2, MIN_BLOCK_SIZE * 3 / 2, MIN_BLOCK_SIZE};
    int by[9] = {MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE / 2, MIN_BLOCK_SIZE * 3 / 2, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE};

#if Mv_Rang
	int is_x_subpel = 0;
	int is_y_subpel = 0;
    int x_size = bx[blocktype];
    int y_size = by[blocktype];

    if(mv_x % 4 !=0)
		is_x_subpel = 1;

	if(mv_y % 4 !=0)
		is_y_subpel = 1;

    if( (pix_x % (1 << uiBitSize)) != 0 && blocktype == 6)
        x_size = bx[7];
    if( (pix_y % (1 << uiBitSize)) != 0 && blocktype == 4)
        y_size = bx[5];
#endif
#endif

    if (img->type == B_IMG) {
        delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
        delta_P = (delta_P + 512) % 512;
        TRp = delta_P;
        delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);
        TRp  = (TRp + 512) % 512;
        delta_PB = (delta_PB + 512) % 512;
        DistanceIndexFw = delta_PB;

        DistanceIndexBw    = (TRp - DistanceIndexFw + 512) % 512;

#if MV_SCALE
        bw_mvx = -scale_mv(mv_x, DistanceIndexBw, DistanceIndexFw);
        bw_mvy = -scale_mv(mv_y, DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
        if (img->is_field_sequence) {
            bw_mvy = -scale_mv_y2(mv_y, DistanceIndexBw, DistanceIndexFw);
        }
#endif
#else
        bw_mvx = - ((mv_x * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
        bw_mvy = - ((mv_y * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
#if HALF_PIXEL_COMPENSATION_MVD
        if (img->is_field_sequence) {
            int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
            oriPOC = 2 * hc->picture_distance;
            oriRefPOC = oriPOC - DistanceIndexFw;
            scaledPOC = 2 * hc->picture_distance;
            scaledRefPOC = scaledPOC - DistanceIndexBw;
            getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
            bw_mvy = - (((mv_y + delta) * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
        }
#endif
#endif
    } else if (img->type == F_IMG) {
        DistanceIndexFw = 2 * (hc->picture_distance - fref[ref]->imgtr_fwRefDistance);
        DistanceIndexFw = (DistanceIndexFw + 512) % 512;
        DistanceIndexBw = 2 * (hc->picture_distance - fref[snd_ref]->imgtr_fwRefDistance);
        DistanceIndexBw = (DistanceIndexBw + 512) % 512;
        if (ref == img->num_of_references - 1 && he->background_reference_enable) {
            DistanceIndexFw = 1;
        }
        if (snd_ref == img->num_of_references - 1 && he->background_reference_enable) {
            DistanceIndexBw = 1;
        }

#if MV_SCALE
        bw_mvx = scale_mv(mv_x, DistanceIndexBw, DistanceIndexFw);
        bw_mvy = scale_mv(mv_y, DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
        if (img->is_field_sequence) {
            bw_mvy = scale_mv_y1(mv_y, DistanceIndexBw, DistanceIndexFw);
        }
#endif
#else
        bw_mvx = ((mv_x * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
        bw_mvy = ((mv_y * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
#if HALF_PIXEL_COMPENSATION_MVD
        if (img->is_field_sequence) {
            int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
            oriPOC = 2 * hc->picture_distance;
            oriRefPOC = oriPOC - DistanceIndexFw;
            scaledPOC = 2 * hc->picture_distance;
            scaledRefPOC = scaledPOC - DistanceIndexBw;
            getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
            bw_mvy = (((mv_y + delta) * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
        }
#endif
#endif
    }
#if Mv_check_bug
	if( !check_mv_range(uiBitSize, mv_x, mv_y, pix_x, pix_y, blocktype, dmh_x,dmh_y) )
		return 0;
	if( !check_mv_range(uiBitSize, bw_mvx, bw_mvy, pix_x, pix_y, blocktype, dmh_x,dmh_y) )
		return 0;
#endif
#if !Mv_check_bug
#if Mv_Rang
    curr_max_x = (img->width - (pix_x + (x_size << (uiBitSize - MIN_CU_SIZE_IN_BIT)) + is_x_subpel * 4)) * 4
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#else
    curr_max_x = (img->width - (pix_x + (bx[blocktype] << (uiBitSize - MIN_CU_SIZE_IN_BIT)))) * 4
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#endif

#if Mv_Rang
	curr_min_x = pix_x * 4 + /*(16<<(uiBitSize-4))*/64 * 4 - is_x_subpel * 3 * 4;
#else
    curr_min_x = pix_x * 4 + /*(16<<(uiBitSize-4))*/64 * 4;
#endif
#if Mv_Rang
    curr_max_y = (img->height - (pix_y + (y_size << (uiBitSize - MIN_CU_SIZE_IN_BIT)) + is_y_subpel * 4)) * 4
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#else
    curr_max_y = (img->height - (pix_y + (by[blocktype] << (uiBitSize - MIN_CU_SIZE_IN_BIT)))) * 4
#endif
                 + /*(16<<(uiBitSize-4))*/64 * 4;
#if Mv_Rang
	curr_min_y = pix_y * 4 + /*(16<<(uiBitSize-4))*/64 * 4 - is_y_subpel * 3 * 4;
#else
	curr_min_y = pix_y * 4 + /*(16<<(uiBitSize-4))*/64 * 4;
#endif

    if (mv_x > curr_max_x || mv_x < -curr_min_x || mv_x > hc->Max_H_MV || mv_x < hc->Min_H_MV) {
        return 0;
    }

    if (mv_y > curr_max_y || mv_y < -curr_min_y || mv_y > hc->Max_V_MV || mv_y < hc->Min_V_MV) {
        return 0;
    }

    if (bw_mvx > curr_max_x || bw_mvx < -curr_min_x || bw_mvx > hc->Max_H_MV || bw_mvx < hc->Min_H_MV) {
        return 0;
    }

    if (bw_mvy > curr_max_y || bw_mvy < -curr_min_y || bw_mvy > hc->Max_V_MV || bw_mvy < hc->Min_V_MV) {
        return 0;
    }
#endif
    return 1;
}

#if MV_SCALE
Boolean IsInterViewPredict(int ref_frame)
{
    if (fref[ref_frame]->imgtr_fwRefDistance == img->tr)
    { return 1; }//inter view predict
    return 0;
}
#endif

/*
*************************************************************************
* Function:setting the motion vector predictor
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void SetMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  pmv[2], int  **refFrArr,
                              int  ***tmp_mv, int  ref_frame, int  mb_pix_x, int  mb_pix_y, int  blockshape_x, int  blockshape_y, int  ref,
                              int  direct_mv)
{
    int mb_nr                = uiPositionInPic;
    int mb_width             = img->width / MIN_CU_SIZE;
    int mb_x = mb_nr % mb_width;
    int mb_y = mb_nr / mb_width;

    int mb_pix_x_temp = mb_pix_x % MIN_BLOCK_SIZE == 0 ? mb_pix_x : MIN_BLOCK_SIZE;
    int mb_pix_y_temp = mb_pix_y % MIN_BLOCK_SIZE == 0 ? mb_pix_y : MIN_BLOCK_SIZE;
    int pic_block_x          = (mb_x << 1) + (mb_pix_x_temp >> MIN_BLOCK_SIZE_IN_BIT);
    int pic_block_y          = (mb_y << 1) + (mb_pix_y_temp >> MIN_BLOCK_SIZE_IN_BIT);

    int mb_available_up;
    int mb_available_left;
    int mb_available_upleft;
    int mb_available_upright;
    int block_available_up, block_available_left, block_available_upright, block_available_upleft;
    int mv_a, mv_b, mv_c, mv_d, pred_vec = 0;
    int mvPredType, rFrameL, rFrameU, rFrameUR;
    int hv;
    int mva[3] , mvb[3], mvc[3];
    int y_up = 1, y_upright = 1, y_upleft = 1, off_y = 0;
    int rFrameUL;
    int upright;
    int mvc_temp;



    blockshape_x = blockshape_x % MIN_BLOCK_SIZE == 0 ? blockshape_x : MIN_BLOCK_SIZE;
    blockshape_y = blockshape_y % MIN_BLOCK_SIZE == 0 ? blockshape_y : MIN_BLOCK_SIZE;


    mb_available_up      = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - mb_width  ].slice_nr);
    mb_available_left    = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - 1         ].slice_nr);
    mb_available_upleft  = (mb_x == 0 ||
                            mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - mb_width - 1].slice_nr);

    if ((pic_block_y == 0) || ((pic_block_x << MIN_BLOCK_SIZE_IN_BIT) + blockshape_x == img->width)) {
        mb_available_upright = 1;
    } else if (mb_pix_y > 0) {
        mb_available_upright = 0;
    } else {
        mb_available_upright = (img->mb_data[uiPositionInPic].slice_nr != img->mb_data[uiPositionInPic - img->PicWidthInMbs +
                                (mb_pix_x_temp + blockshape_x) / MIN_CU_SIZE].slice_nr);
    }

    block_available_up   = mb_available_up   || (mb_pix_y > 0);
    block_available_left = mb_available_left || (mb_pix_x > 0);

    if (input->g_uiMaxSizeInBit == B64X64_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix64[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    } else if (input->g_uiMaxSizeInBit == B32X32_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix32[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    } else if (input->g_uiMaxSizeInBit == B16X16_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix16[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    }

    if ((pic_block_x << MIN_BLOCK_SIZE_IN_BIT) + blockshape_x >= img->width) {
        upright = 0;
    }

    block_available_upright = upright && (!mb_available_upright);

    if (mb_pix_x > 0) {
        block_available_upleft = (mb_pix_y > 0 ? 1 : block_available_up);
    } else if (mb_pix_y > 0) {
        block_available_upleft = block_available_left;
    } else {
        block_available_upleft = mb_available_upleft;
    }

    /*Lou 1016 Start*/
#if XY_MIN_PMV
    mvPredType = MVPRED_xy_MIN;
#else
    mvPredType = MVPRED_MEDIAN;
#endif
    rFrameL    = block_available_left    ? refFrArr[pic_block_y  - off_y]  [pic_block_x - 1] : -1;
    rFrameU    = block_available_up      ? refFrArr[pic_block_y - y_up][pic_block_x]   : -1;
    rFrameUR   = block_available_upright ? refFrArr[pic_block_y - y_upright][pic_block_x +
                 (blockshape_x / MIN_BLOCK_SIZE) ] : block_available_upleft  ? refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameUL   = block_available_upleft  ? refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;

    if (rFrameL != -1) {
        if (((ref_frame == img->num_of_references - 1 && rFrameL != img->num_of_references - 1) ||
             (ref_frame != img->num_of_references - 1 && rFrameL == img->num_of_references - 1)) &&
            (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) {
            rFrameL = -1;
        }
        if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
            rFrameL = -1;
        }

    }
    if (rFrameU != -1) {
        if ((ref_frame == img->num_of_references - 1 && rFrameU != img->num_of_references - 1 ||
             (ref_frame != img->num_of_references - 1 && rFrameU == img->num_of_references - 1)) &&
            (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) {
            rFrameU = -1;
        }
        if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
            rFrameU = -1;
        }

    }
    if (rFrameUR != -1) {
        if (((ref_frame == img->num_of_references - 1 && rFrameUR != img->num_of_references - 1) ||
             (ref_frame != img->num_of_references - 1 && rFrameUR == img->num_of_references - 1)) &&
            (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) {
            rFrameUR = -1;
        }
        if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
            rFrameUR = -1;
        }

    }
    if (rFrameUL != -1) {
        if (((ref_frame == img->num_of_references - 1 && rFrameUL != img->num_of_references - 1) ||
             (ref_frame != img->num_of_references - 1 && rFrameUL == img->num_of_references - 1)) &&
            (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) {
            rFrameUL = -1;
        }
        if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
            rFrameUL = -1;
        }

    }


    if ((rFrameL != -1) && (rFrameU == -1) && (rFrameUR == -1)) {
        mvPredType = MVPRED_L;
    } else if ((rFrameL == -1) && (rFrameU != -1) && (rFrameUR == -1)) {
        mvPredType = MVPRED_U;
    } else if ((rFrameL == -1) && (rFrameU == -1) && (rFrameUR != -1)) {
        mvPredType = MVPRED_UR;
    } else if (blockshape_x < blockshape_y) {
        if (mb_pix_x == 0) {
            if (rFrameL == ref_frame) {
                mvPredType = MVPRED_L;
            }
        } else {
            if (rFrameUR == ref_frame) {
                mvPredType = MVPRED_UR;
            }
        }
    } else if (blockshape_x > blockshape_y) {
        if (mb_pix_y == 0) {
            if (rFrameU == ref_frame) {
                mvPredType = MVPRED_U;
            }
        } else {
            if (rFrameL == ref_frame) {
                mvPredType = MVPRED_L;
            }
        }
    }

    for (hv = 0; hv < 2; hv++) {
        mva[hv] = mv_a = block_available_left ? tmp_mv[pic_block_y - off_y][pic_block_x - 1][hv] : 0;
        mvb[hv] = mv_b = block_available_up ? tmp_mv[pic_block_y - y_up][pic_block_x][hv] : 0;
        mv_d = block_available_upleft ? tmp_mv[pic_block_y - y_upleft][pic_block_x - 1][hv] : 0;
        mvc[hv] = mv_c = block_available_upright ? tmp_mv[pic_block_y - y_upright][pic_block_x +
                         (blockshape_x / MIN_BLOCK_SIZE)][hv] : mv_d;

        if ((rFrameL == -1 && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) ||
            (rFrameL == -1 && img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) {
            mva[hv] = 0;
        }
        else


#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1 && rFrameL != -1) {
                mva[hv] = scale_motion_vector_y1(mva[hv], ref_frame, rFrameL, ref);
            } else {
#if Mv_Clip
				mva[hv] = scale_motion_vector(mva[hv], ref_frame, rFrameL,
					ref,0);     //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
#else
                mva[hv] = scale_motion_vector(mva[hv], ref_frame, rFrameL,
                                              ref);     //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
#endif
			}
#else
            if (img->is_field_sequence && hv == 1 && rFrameL != -1) {
                int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                int mult_distance = calculate_distance(ref_frame, ref);
                int devide_distance = calculate_distance(rFrameL, ref);

                oriPOC = 2 * hc->picture_distance;
                oriRefPOC = oriPOC - devide_distance;
                scaledPOC = 2 * hc->picture_distance;
                scaledRefPOC = scaledPOC - mult_distance;
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if Mv_Clip
				mva[hv] = scale_motion_vector(mva[hv] + delta, ref_frame, rFrameL,
					ref,-delta2);  //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
#else
                mva[hv] = scale_motion_vector(mva[hv] + delta, ref_frame, rFrameL,
                                              ref);  //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
                mva[hv] -= delta2;
#endif
            } else {
#if Mv_Clip
				mva[hv] = scale_motion_vector(mva[hv], ref_frame, rFrameL,
					ref,0);     //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
#else
                mva[hv] = scale_motion_vector(mva[hv], ref_frame, rFrameL,
                                              ref);     //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
#endif            
			}
#endif
        }
#else
#if Mv_Clip
			mva[hv] = scale_motion_vector(mva[hv], ref_frame, rFrameL,
			ref,0);  //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
#else
            mva[hv] = scale_motion_vector(mva[hv], ref_frame, rFrameL,
                                          ref);  //, smbtypecurr, smbtypeL, pic_block_y-off_y, pic_block_y, ref, direct_mv);
#endif
#endif
//#endif

        if ((rFrameU == -1 && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) ||
            (rFrameU == -1 && img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable))

            mvb[hv] = 0;
        else


#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1 && rFrameU != -1) {
                mvb[hv] = scale_motion_vector_y1(mvb[hv], ref_frame, rFrameU, ref);
            } else {
#if Mv_Clip
				mvb[hv] = scale_motion_vector(mvb[hv], ref_frame, rFrameU,
					ref,0);     //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#else
                mvb[hv] = scale_motion_vector(mvb[hv], ref_frame, rFrameU,
                                              ref);     //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#endif          
			}
#else
            if (img->is_field_sequence && hv == 1 && rFrameU != -1) {
                int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                int mult_distance = calculate_distance(ref_frame, ref);
                int devide_distance = calculate_distance(rFrameU, ref);

                oriPOC = 2 * hc->picture_distance;
                oriRefPOC = oriPOC - devide_distance;
                scaledPOC = 2 * hc->picture_distance;
                scaledRefPOC = scaledPOC - mult_distance;
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if Mv_Clip
				mvb[hv] = scale_motion_vector(mvb[hv] + delta, ref_frame, rFrameU,
					ref,-delta2);  //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#else
                mvb[hv] = scale_motion_vector(mvb[hv] + delta, ref_frame, rFrameU,
                                              ref);  //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
                mvb[hv] -= delta2;
#endif
            } else {
#if Mv_Clip
				mvb[hv] = scale_motion_vector(mvb[hv], ref_frame, rFrameU,
					ref,0);     //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#else
                mvb[hv] = scale_motion_vector(mvb[hv], ref_frame, rFrameU,
                                              ref);     //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#endif            
			}
#endif
        }
#else
            mvb[hv] = scale_motion_vector(mvb[hv], ref_frame, rFrameU,
                                          ref);  //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#endif
//#endif
        if ((rFrameUL == -1 && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) ||
            (rFrameUL == -1 && img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable))

            mv_d = 0;
        else


#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1 && rFrameUL != -1) {
                mv_d = scale_motion_vector_y1(mv_d, ref_frame, rFrameUL, ref);
            } else {
#if Mv_Clip
				mv_d = scale_motion_vector(mv_d, ref_frame, rFrameUL,
					ref,0);     //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
#else
                mv_d = scale_motion_vector(mv_d, ref_frame, rFrameUL,
                                           ref);     //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
#endif
            }

#else
            if (img->is_field_sequence && hv == 1 && rFrameUL != -1) {
                int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                int mult_distance = calculate_distance(ref_frame, ref);
                int devide_distance = calculate_distance(rFrameUL, ref);

                oriPOC = 2 * hc->picture_distance;
                oriRefPOC = oriPOC - devide_distance;
                scaledPOC = 2 * hc->picture_distance;
                scaledRefPOC = scaledPOC - mult_distance;
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if Mv_Clip
				mv_d = scale_motion_vector(mv_d + delta, ref_frame, rFrameUL,
					ref,-delta2);  //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
#else
                mv_d = scale_motion_vector(mv_d + delta, ref_frame, rFrameUL,
                                           ref);  //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
                mv_d -= delta2;
#endif
            } else {
#if Mv_Clip
				mv_d = scale_motion_vector(mv_d, ref_frame, rFrameUL,
					ref,0);     //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
#else
                mv_d = scale_motion_vector(mv_d, ref_frame, rFrameUL,
                                           ref);     //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
#endif  
			}
#endif
        }
#else
#if Mv_Clip
			mv_d = scale_motion_vector(mv_d, ref_frame, rFrameUL,
			ref,0);  //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
#else
            mv_d = scale_motion_vector(mv_d, ref_frame, rFrameUL,
                                       ref);  //, smbtypecurr, smbtypeUL, pic_block_y-y_upleft, pic_block_y, ref, direct_mv);
#endif
#endif
//#endif

        if ((rFrameUR == -1 && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) ||
            (rFrameUR == -1 && img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable))

            mvc_temp = 0;
        else

#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1 && rFrameUR != -1) {
                mvc_temp = scale_motion_vector_y1(mvc[hv], ref_frame, rFrameUR, ref);
            } else {
#if Mv_Clip
                mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref,0);
#else
                mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref);
#endif
            }
#else
            if (img->is_field_sequence && hv == 1 && rFrameUR != -1) {
                int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                int mult_distance = calculate_distance(ref_frame, ref);
                int devide_distance = calculate_distance(rFrameUR, ref);

                oriPOC = 2 * hc->picture_distance;
                oriRefPOC = oriPOC - devide_distance;
                scaledPOC = 2 * hc->picture_distance;
                scaledRefPOC = scaledPOC - mult_distance;
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if Mv_Clip
                mvc_temp = scale_motion_vector(mvc[hv] + delta, ref_frame, rFrameUR, ref,-delta2);
#else
                mvc_temp = scale_motion_vector(mvc[hv] + delta, ref_frame, rFrameUR, ref);
                mvc_temp -= delta2;
#endif
            } else {
#if Mv_Clip
				mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref,0);
#else
                mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref);
#endif
            }
#endif
        }
#else
#if Mv_Clip
			mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref);
#else
            mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref);
#endif
#endif
//#endif
        mvc[hv] = block_available_upright ? mvc_temp : mv_d;


        {
            switch (mvPredType) {
#if XY_MIN_PMV
            case MVPRED_xy_MIN:
                if (hv == 1) {
                    //for x component
                    if (((mva[0] < 0) && (mvb[0] > 0) && (mvc[0] > 0)) || (mva[0] > 0) && (mvb[0] < 0) && (mvc[0] < 0)) {
                        pmv[0] = (mvb[0] + mvc[0]) / 2;
                    } else if (((mvb[0] < 0) && (mva[0] > 0) && (mvc[0] > 0)) || ((mvb[0] > 0) && (mva[0] < 0) && (mvc[0] < 0))) {
                        pmv[0] = (mvc[0] + mva[0]) / 2;
                    } else if (((mvc[0] < 0) && (mva[0] > 0) && (mvb[0] > 0)) || ((mvc[0] > 0) && (mva[0] < 0) && (mvb[0] < 0))) {
                        pmv[0] = (mva[0] + mvb[0]) / 2;
                    } else {
                        // !! for Ax
                        //
                        mva[2] = abs(mva[0] - mvb[0]);
                        // !! for Bx
                        //
                        mvb[2] = abs(mvb[0] - mvc[0]);
                        // !! for Cx
                        //
                        mvc[2] = abs(mvc[0] - mva[0]);

                        pred_vec = min(mva[2], min(mvb[2], mvc[2]));

                        if (pred_vec == mva[2]) {
                            pmv[0] = (mva[0] + mvb[0]) / 2;
                        } else if (pred_vec == mvb[2]) {
                            pmv[0] = (mvb[0] + mvc[0]) / 2;
                        } else {
                            pmv[0] = (mvc[0] + mva[0]) / 2;
                        }
                    }

                    //for y component
                    if (((mva[1] < 0) && (mvb[1] > 0) && (mvc[1] > 0)) || (mva[1] > 0) && (mvb[1] < 0) && (mvc[1] < 0)) {
                        pmv[1] = (mvb[1] + mvc[1]) / 2;
                    } else if (((mvb[1] < 0) && (mva[1] > 0) && (mvc[1] > 0)) || ((mvb[1] > 0) && (mva[1] < 0) && (mvc[1] < 0))) {
                        pmv[1] = (mvc[1] + mva[1]) / 2;
                    } else if (((mvc[1] < 0) && (mva[1] > 0) && (mvb[1] > 0)) || ((mvc[1] > 0) && (mva[1] < 0) && (mvb[1] < 0))) {
                        pmv[1] = (mva[1] + mvb[1]) / 2;
                    } else {
                        // !! for Ay
                        //
                        mva[2] = abs(mva[1] - mvb[1]);
                        // !! for By
                        //
                        mvb[2] = abs(mvb[1] - mvc[1]);
                        // !! for Cy
                        //
                        mvc[2] = abs(mvc[1] - mva[1]);

                        pred_vec = min(mva[2], min(mvb[2], mvc[2]));

                        if (pred_vec == mva[2]) {
                            pmv[1] = (mva[1] + mvb[1]) / 2;
                        } else if (pred_vec == mvb[2]) {
                            pmv[1] = (mvb[1] + mvc[1]) / 2;
                        } else {
                            pmv[1] = (mvc[1] + mva[1]) / 2;
                        }
                    }

                }
                break;
#else
            case MVPRED_MEDIAN:


                if (hv == 1) {
                    // jlzheng 7.2
                    // !! for A
                    //
                    mva[2] = abs(mva[0] - mvb[0]) + abs(mva[1] - mvb[1]) ;
                    // !! for B
                    //
                    mvb[2] = abs(mvb[0] - mvc[0]) + abs(mvb[1] - mvc[1]);
                    // !! for C
                    //
                    mvc[2] = abs(mvc[0] - mva[0]) + abs(mvc[1] - mva[1]) ;

                    pred_vec = MEDIAN(mva[2], mvb[2], mvc[2]);

                    if (pred_vec == mva[2]) {
                        pmv[0] = mvc[0];
                        pmv[1] = mvc[1];
                    }

                    else if (pred_vec == mvb[2]) {
                        pmv[0] = mva[0];
                        pmv[1] = mva[1];
                    } else {
                        pmv[0] = mvb[0];
                        pmv[1] = mvb[1];
                    }// END

                }

                break;
#endif
            case MVPRED_L:
                pred_vec = mva[hv];
                break;
            case MVPRED_U:
                pred_vec = mvb[hv];
                break;
            case MVPRED_UR:
                pred_vec = mvc[hv];
                break;
            default:
                break;
            }
#if XY_MIN_PMV
            if (mvPredType != MVPRED_xy_MIN)
#else
            if (mvPredType != MVPRED_MEDIAN)
#endif
            {
                pmv[hv] = pred_vec;
            }
        }
    }
}
void SetSkipMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  **fw_refFrArr,
                                  int **bw_refFrArr, int  ***tmp_fwmv , int ***tmp_bwmv,
                                  int  ref_frame, int  mb_pix_x, int  mb_pix_y, int  blockshape_x, int  blockshape_y, int  direct_mv)
{
    int mb_nr                = uiPositionInPic;
    int mb_width             = img->width / MIN_CU_SIZE;
    int mb_x = mb_nr % mb_width;
    int mb_y = mb_nr / mb_width;

    int mb_pix_x_temp = mb_pix_x % MIN_BLOCK_SIZE == 0 ? mb_pix_x : MIN_BLOCK_SIZE;
    int mb_pix_y_temp = mb_pix_y % MIN_BLOCK_SIZE == 0 ? mb_pix_y : MIN_BLOCK_SIZE;
    int pic_block_x          = (mb_x << 1) + (mb_pix_x_temp >> MIN_BLOCK_SIZE_IN_BIT);
    int pic_block_y          = (mb_y << 1) + (mb_pix_y_temp >> MIN_BLOCK_SIZE_IN_BIT);

    int mb_available_up;
    int mb_available_left;
    int mb_available_upleft;
    int mb_available_upright;
    int block_available_left1, block_available_up1;
    int block_available_up, block_available_left, block_available_upright, block_available_upleft;
    int rFrameL[2], rFrameU[2], rFrameUR[2], rFrameUL[2], rFrameL1[2], rFrameU1[2];
    int hv;
    int mva[2][2] , mvb[2][2], mvc[2][2];
    int mva1[2][2], mvb1[2][2], mve[2][2];
    int y_up = 1, y_upright = 1, y_upleft = 1, off_y = 0;
    int upright;
    int mb_nb_L, mb_nb_L1, mb_nb_U, mb_nb_U1, mb_nb_UR, mb_nb_UL;
    int bRefFrame[2][6];
    int pmv[2][2][6];
    int i, j, dir;
    int bid_flag = 0, bw_flag = 0, fw_flag = 0, sym_flag = 0, bid2;
    int mode_info[6];
    PixelPos block_L, block_U, block_UR, block_UL, block_L1, block_U1;
    codingUnit *neighborMB;
    int sizeOfNeighborBlock_x, sizeOfNeighborBlock_y;


    blockshape_x = blockshape_x % MIN_BLOCK_SIZE == 0 ? blockshape_x : MIN_BLOCK_SIZE;
    blockshape_y = blockshape_y % MIN_BLOCK_SIZE == 0 ? blockshape_y : MIN_BLOCK_SIZE;


    mb_available_up      = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           mb_width  ].slice_nr);
    mb_available_left    = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           1         ].slice_nr);
    mb_available_upleft  = (mb_x == 0 ||
                            mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - mb_width - 1].slice_nr);
    if ((pic_block_y == 0) || ((pic_block_x << MIN_BLOCK_SIZE_IN_BIT) + blockshape_x == img->width)) {
        mb_available_upright = 1;
    } else if (mb_pix_y > 0) {
        mb_available_upright = 0;
    } else {
        mb_available_upright = (img->mb_data[uiPositionInPic].slice_nr != img->mb_data[uiPositionInPic - img->PicWidthInMbs +
                                (mb_pix_x_temp + blockshape_x) / MIN_CU_SIZE].slice_nr);
    }

    block_available_up   = mb_available_up   || (mb_pix_y > 0);
    block_available_left = mb_available_left || (mb_pix_x > 0);
    block_available_left1 = block_available_left && (blockshape_y > MIN_BLOCK_SIZE);
    block_available_up1 = block_available_up && (blockshape_x > MIN_BLOCK_SIZE);

    if (input->g_uiMaxSizeInBit == B64X64_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix64[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    } else if (input->g_uiMaxSizeInBit == B32X32_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix32[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    } else if (input->g_uiMaxSizeInBit == B16X16_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix16[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    }

    if ((pic_block_x << MIN_BLOCK_SIZE_IN_BIT) + blockshape_x >= img->width) {
        upright = 0;
    }

    block_available_upright = upright && (!mb_available_upright);

    if (mb_pix_x > 0) {
        block_available_upleft = (mb_pix_y > 0 ? 1 : block_available_up);
    } else if (mb_pix_y > 0) {
        block_available_upleft = block_available_left;
    } else {
        block_available_upleft = mb_available_upleft;
    }
    mb_nb_L1 = mb_width * ((pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1) >> 1) + ((pic_block_x - 1) >> 1);
    mb_nb_L = mb_width * ((pic_block_y  - off_y) >> 1) + ((pic_block_x - 1) >> 1);
    mb_nb_U = mb_width * ((pic_block_y - y_up) >> 1) + ((pic_block_x) >> 1);
    mb_nb_U1 = mb_width * ((pic_block_y - y_up) >> 1) + ((pic_block_x + blockshape_x / MIN_BLOCK_SIZE - 1) >> 1);
    mb_nb_UR = mb_width * ((pic_block_y - y_upright) >> 1) + ((pic_block_x + blockshape_x / MIN_BLOCK_SIZE) >> 1);
    mb_nb_UL = mb_width * ((pic_block_y - y_upleft) >> 1) + ((pic_block_x - 1) >> 1);

    rFrameL[0]    = block_available_left    ? bw_refFrArr[pic_block_y  - off_y]  [pic_block_x - 1] : -1;
    rFrameU[0]    = block_available_up      ? bw_refFrArr[pic_block_y - y_up][pic_block_x]   : -1;
    rFrameUR[0]   = block_available_upright ? bw_refFrArr[pic_block_y - y_upright][pic_block_x +
                    (blockshape_x / MIN_BLOCK_SIZE) ] :  -1;
    rFrameUL[0]   = block_available_upleft  ? bw_refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameL1[0] = block_available_left1 ? bw_refFrArr[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x -
                  1] : -1;
    rFrameU1[0] = block_available_up1 ? bw_refFrArr[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE - 1] :
                  -1;

    rFrameL[1]    = block_available_left    ? fw_refFrArr[pic_block_y  - off_y]  [pic_block_x - 1] : -1;
    rFrameU[1]    = block_available_up      ? fw_refFrArr[pic_block_y - y_up][pic_block_x]   : -1;
    rFrameUR[1]   = block_available_upright ? fw_refFrArr[pic_block_y - y_upright][pic_block_x +
                    (blockshape_x / MIN_BLOCK_SIZE) ] :  -1;
    rFrameUL[1]   = block_available_upleft  ? fw_refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameL1[1] = block_available_left1 ? fw_refFrArr[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x -
                  1] : -1;
    rFrameU1[1] = block_available_up1 ? fw_refFrArr[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE - 1] :
                  -1;

    for (i = 0; i < 2; i++) {
        bRefFrame[i][0] = rFrameUL[i];
        bRefFrame[i][1] = rFrameU[i];
        bRefFrame[i][2] = rFrameL[i];
        bRefFrame[i][3] = rFrameUR[i];
        bRefFrame[i][4] = rFrameU1[i];
        bRefFrame[i][5] = rFrameL1[i];
    }
    for (hv = 0; hv < 2; hv++) {

        mva[0][hv]  = block_available_left    ? tmp_bwmv[pic_block_y - off_y][pic_block_x - 1][hv]              : 0;
        mva1[0][hv]  = block_available_left1 ? tmp_bwmv[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x -
                       1][hv] : 0;
        mvb[0][hv] = block_available_up      ? tmp_bwmv[pic_block_y - y_up][pic_block_x][hv]                : 0;
        mvb1[0][hv]  = block_available_up1 ? tmp_bwmv[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE - 1][hv] :
                       0;
        mve[0][hv] = block_available_upleft  ? tmp_bwmv[pic_block_y - y_upleft][pic_block_x - 1][hv]              : 0;
        mvc[0][hv]  = block_available_upright ? tmp_bwmv[pic_block_y - y_upright][pic_block_x +
                      (blockshape_x / MIN_BLOCK_SIZE) ][hv] : 0;

        mva[1][hv]  = block_available_left    ? tmp_fwmv[pic_block_y - off_y][pic_block_x - 1][hv]              : 0;
        mva1[1][hv]  = block_available_left1 ? tmp_fwmv[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x -
                       1][hv] : 0;
        mvb[1][hv] = block_available_up      ? tmp_fwmv[pic_block_y - y_up][pic_block_x][hv]                : 0;
        mvb1[1][hv]  = block_available_up1 ? tmp_fwmv[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE - 1][hv] :
                       0;
        mve[1][hv] = block_available_upleft  ? tmp_fwmv[pic_block_y - y_upleft][pic_block_x - 1][hv]             : 0;
        mvc[1][hv]  = block_available_upright ? tmp_fwmv[pic_block_y - y_upright][pic_block_x +
                      (blockshape_x / MIN_BLOCK_SIZE) ][hv] : 0;


        for (i = 0; i < 2; i++) {
            pmv[i][hv][0] = mve[i][hv];   //i-direction, hv-x or y
            pmv[i][hv][1] = mvb[i][hv];
            pmv[i][hv][2] = mva[i][hv];
            pmv[i][hv][3] = mvc[i][hv];
            pmv[i][hv][4] = mvb1[i][hv];
            pmv[i][hv][5] = mva1[i][hv];
        }
    }

    for (i = 0; i < 2; i++) {
        for (dir = 0; dir < 5; dir++) {
            img->tmp_bwBSkipMv[dir][i] = 0;
            img->tmp_fwBSkipMv[dir][i] = 0;
        }
    }


    // PixelPos block_L, block_U,block_UR, block_UL,block_L1, block_U1;

    getNeighbour(-1, 0, 1, &block_L, uiPositionInPic, uiBitSize, &img->mb_data[uiPositionInPic]);
    getNeighbour(-1, (1 << uiBitSize) - 1, 1, &block_L1, uiPositionInPic, uiBitSize, &img->mb_data[uiPositionInPic]);
    getNeighbour(0, -1, 1, &block_U, uiPositionInPic, uiBitSize, &img->mb_data[uiPositionInPic]);
    getNeighbour((1 << uiBitSize) - 1, -1, 1, &block_U1, uiPositionInPic, uiBitSize, &img->mb_data[uiPositionInPic]);
    getNeighbour(-1, -1, 1, &block_UL, uiPositionInPic, uiBitSize, &img->mb_data[uiPositionInPic]);
    getNeighbour((1 << uiBitSize), -1, 1, &block_UR, uiPositionInPic, uiBitSize, &img->mb_data[uiPositionInPic]);

    /*
    bRefFrame[i][0] = rFrameUL[i];
    bRefFrame[i][1] = rFrameU[i];
    bRefFrame[i][2] = rFrameL[i];
    bRefFrame[i][3] = rFrameUR[i];
    bRefFrame[i][4] = rFrameU1[i];
    bRefFrame[i][5] = rFrameL1[i];
    */

    if (block_UL.available && block_available_upleft) {
        neighborMB = &img->mb_data[block_UL.mb_addr];
        if (neighborMB->cuType == PHOR_UP) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PHOR_DOWN) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        if (neighborMB->cuType == PVER_LEFT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PVER_RIGHT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 2;
        }


        block_UL.x = (block_UL.x / sizeOfNeighborBlock_x) ? 1 : 0;
        block_UL.y = (block_UL.y / sizeOfNeighborBlock_y) ? 1 : 0;
        mode_info[0] =  img->mb_data[block_UL.mb_addr].b8pdir[block_UL.x + block_UL.y * 2];

    } else {
        mode_info[0] = -1;
    }


    if (block_U.available && block_available_up) {
        neighborMB = &img->mb_data[block_U.mb_addr];
        if (neighborMB->cuType == PHOR_UP) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PHOR_DOWN) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        if (neighborMB->cuType == PVER_LEFT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PVER_RIGHT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 2;
        }


        block_U.x = (block_U.x / sizeOfNeighborBlock_x) ? 1 : 0;
        block_U.y = (block_U.y / sizeOfNeighborBlock_y) ? 1 : 0;
        mode_info[1] =  img->mb_data[block_U.mb_addr].b8pdir[block_U.x + block_U.y * 2];

    } else {
        mode_info[1] = -1;
    }


    if (block_L.available && block_available_left) {
        neighborMB = &img->mb_data[block_L.mb_addr];
        if (neighborMB->cuType == PHOR_UP) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PHOR_DOWN) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        if (neighborMB->cuType == PVER_LEFT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PVER_RIGHT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        block_L.x = (block_L.x / sizeOfNeighborBlock_x) ? 1 : 0;
        block_L.y = (block_L.y / sizeOfNeighborBlock_y) ? 1 : 0;
        mode_info[2] =  img->mb_data[block_L.mb_addr].b8pdir[block_L.x + block_L.y * 2];

    } else {
        mode_info[2] = -1;
    }



    if (block_UR.available && block_available_upright) {
        neighborMB = &img->mb_data[block_UR.mb_addr];
        if (neighborMB->cuType == PHOR_UP) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PHOR_DOWN) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        if (neighborMB->cuType == PVER_LEFT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PVER_RIGHT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        block_UR.x = (block_UR.x / sizeOfNeighborBlock_x) ? 1 : 0;
        block_UR.y = (block_UR.y / sizeOfNeighborBlock_y) ? 1 : 0;
        mode_info[3] = img->mb_data[block_UR.mb_addr].b8pdir[block_UR.x + block_UR.y * 2];

    } else {
        mode_info[3] = -1;
    }




    if (block_U1.available && block_available_up1) {
        neighborMB = &img->mb_data[block_U1.mb_addr];
        if (neighborMB->cuType == PHOR_UP) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PHOR_DOWN) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        if (neighborMB->cuType == PVER_LEFT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PVER_RIGHT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        block_U1.x = (block_U1.x / sizeOfNeighborBlock_x) ? 1 : 0;
        block_U1.y = (block_U1.y / sizeOfNeighborBlock_y) ? 1 : 0;
        mode_info[4] = img->mb_data[block_U1.mb_addr].b8pdir[block_U1.x + block_U1.y * 2];

    } else {
        mode_info[4] = -1;
    }

    if (block_L1.available && block_available_left1) {
        neighborMB = &img->mb_data[block_L1.mb_addr];
        if (neighborMB->cuType == PHOR_UP) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PHOR_DOWN) {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_y = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        if (neighborMB->cuType == PVER_LEFT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4;
        } else if (neighborMB->cuType == PVER_RIGHT) {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 4 * 3;
        } else {
            sizeOfNeighborBlock_x = (1 << neighborMB->ui_MbBitSize) / 2;
        }

        block_L1.x = (block_L1.x / sizeOfNeighborBlock_x) ? 1 : 0;
        block_L1.y = (block_L1.y / sizeOfNeighborBlock_y) ? 1 : 0;
        mode_info[5] =  img->mb_data[block_L1.mb_addr].b8pdir[block_L1.x + block_L1.y * 2];

    } else {
        mode_info[5] = -1;
    }


    for (j = 0; j < 6; j++) {
        if (mode_info[j] == BID) {
            img->tmp_bwBSkipMv[DS_BID][0] = pmv[0][0][j];
            img->tmp_bwBSkipMv[DS_BID][1] = pmv[0][1][j];
            img->tmp_fwBSkipMv[DS_BID][0] = pmv[1][0][j];
            img->tmp_fwBSkipMv[DS_BID][1] = pmv[1][1][j];
            bid_flag++;
            if (bid_flag == 1) {
                bid2 = j;
            }
        } else if (mode_info[j] == SYM) {
            img->tmp_bwBSkipMv[DS_SYM][0] = pmv[0][0][j];
            img->tmp_bwBSkipMv[DS_SYM][1] = pmv[0][1][j];
            img->tmp_fwBSkipMv[DS_SYM][0] = pmv[1][0][j];
            img->tmp_fwBSkipMv[DS_SYM][1] = pmv[1][1][j];
            sym_flag++;
        } else if (mode_info[j] == BACKWARD) {
            img->tmp_bwBSkipMv[DS_BACKWARD][0] = pmv[0][0][j];
            img->tmp_bwBSkipMv[DS_BACKWARD][1] = pmv[0][1][j];
            bw_flag++;
        } else if (mode_info[j] == FORWARD) {
            img->tmp_fwBSkipMv[DS_FORWARD][0] = pmv[1][0][j];
            img->tmp_fwBSkipMv[DS_FORWARD][1] = pmv[1][1][j];
            fw_flag++;
        }
    }

    if (bid_flag == 0 && fw_flag != 0 && bw_flag != 0) {
        img->tmp_bwBSkipMv[DS_BID][0] = img->tmp_bwBSkipMv[DS_BACKWARD][0];
        img->tmp_bwBSkipMv[DS_BID][1] = img->tmp_bwBSkipMv[DS_BACKWARD][1];
        img->tmp_fwBSkipMv[DS_BID][0] = img->tmp_fwBSkipMv[DS_FORWARD][0];
        img->tmp_fwBSkipMv[DS_BID][1] = img->tmp_fwBSkipMv[DS_FORWARD][1];
    }

    if (sym_flag == 0 && bid_flag > 1) {
        img->tmp_bwBSkipMv[DS_SYM][0] = pmv[0][0][bid2];
        img->tmp_bwBSkipMv[DS_SYM][1] = pmv[0][1][bid2];
        img->tmp_fwBSkipMv[DS_SYM][0] = pmv[1][0][bid2];
        img->tmp_fwBSkipMv[DS_SYM][1] = pmv[1][1][bid2];
    } else if (sym_flag == 0 && bw_flag != 0) {
        img->tmp_bwBSkipMv[DS_SYM][0] = img->tmp_bwBSkipMv[DS_BACKWARD][0];
        img->tmp_bwBSkipMv[DS_SYM][1] = img->tmp_bwBSkipMv[DS_BACKWARD][1];
        img->tmp_fwBSkipMv[DS_SYM][0] = -img->tmp_bwBSkipMv[DS_BACKWARD][0];
        img->tmp_fwBSkipMv[DS_SYM][1] = -img->tmp_bwBSkipMv[DS_BACKWARD][1];
    }

    else if (sym_flag == 0 && fw_flag != 0) {
        img->tmp_bwBSkipMv[DS_SYM][0] = -img->tmp_fwBSkipMv[DS_FORWARD][0];
        img->tmp_bwBSkipMv[DS_SYM][1] = -img->tmp_fwBSkipMv[DS_FORWARD][1];
        img->tmp_fwBSkipMv[DS_SYM][0] = img->tmp_fwBSkipMv[DS_FORWARD][0];
        img->tmp_fwBSkipMv[DS_SYM][1] = img->tmp_fwBSkipMv[DS_FORWARD][1];
    }

    if (bw_flag == 0 && bid_flag > 1) {
        img->tmp_bwBSkipMv[DS_BACKWARD][0] = pmv[0][0][bid2];
        img->tmp_bwBSkipMv[DS_BACKWARD][1] = pmv[0][1][bid2];
    } else if (bw_flag == 0 && bid_flag != 0) {
        img->tmp_bwBSkipMv[DS_BACKWARD][0] = img->tmp_bwBSkipMv[DS_BID][0];
        img->tmp_bwBSkipMv[DS_BACKWARD][1] = img->tmp_bwBSkipMv[DS_BID][1];
    }

    if (fw_flag == 0 && bid_flag > 1) {
        img->tmp_fwBSkipMv[DS_FORWARD][0] = pmv[1][0][bid2];
        img->tmp_fwBSkipMv[DS_FORWARD][1] = pmv[1][1][bid2];
    } else if (fw_flag == 0 && bid_flag != 0) {
        img->tmp_fwBSkipMv[DS_FORWARD][0] = img->tmp_fwBSkipMv[DS_BID][0];
        img->tmp_fwBSkipMv[DS_FORWARD][1] = img->tmp_fwBSkipMv[DS_BID][1];
    }

#if Mv_Clip
	img->tmp_fwBSkipMv[DS_SYM][0] =Clip3(-32768, 32767, img->tmp_fwBSkipMv[DS_SYM][0]);
	img->tmp_fwBSkipMv[DS_SYM][1] =Clip3(-32768, 32767,img->tmp_fwBSkipMv[DS_SYM][1]);
	img->tmp_bwBSkipMv[DS_SYM][0] =Clip3(-32768, 32767,img->tmp_bwBSkipMv[DS_SYM][0]);
	img->tmp_bwBSkipMv[DS_SYM][1] =Clip3(-32768, 32767,img->tmp_bwBSkipMv[DS_SYM][1]);
#endif
}



void setPSkipMotionVector(unsigned int uiBitSize, unsigned int uiPositionInPic, int  ref_frame, int  mb_pix_x,
                          int  mb_pix_y, int  blockshape_x, int  blockshape_y, int  direct_mv)
{
    int mb_nr                = uiPositionInPic;
    int mb_width             = img->width / MIN_CU_SIZE;
    int mb_x = mb_nr % mb_width;
    int mb_y = mb_nr / mb_width;

    int mb_pix_x_temp = mb_pix_x % MIN_BLOCK_SIZE == 0 ? mb_pix_x : MIN_BLOCK_SIZE;
    int mb_pix_y_temp = mb_pix_y % MIN_BLOCK_SIZE == 0 ? mb_pix_y : MIN_BLOCK_SIZE;

    int pic_block_x          = (mb_x << 1) + (mb_pix_x_temp >> MIN_BLOCK_SIZE_IN_BIT);
    int pic_block_y          = (mb_y << 1) + (mb_pix_y_temp >> MIN_BLOCK_SIZE_IN_BIT);

    int mb_available_up;
    int mb_available_left;
    int mb_available_upleft;
    int mb_available_upright;
    int block_available_left1, block_available_up1;
    int block_available_up, block_available_left, block_available_upright, block_available_upleft;
    int rFrameL[2], rFrameU[2], rFrameUR[2], rFrameUL[2], rFrameL1[2], rFrameU1[2];
    int hv;
    int mva[2][2] , mvb[2][2], mvc[2][2];
    int mva1[2][2], mvb1[2][2], mve[2][2];
    int y_up = 1, y_upright = 1, y_upleft = 1, off_y = 0;
    int upright;
    int mb_nb_L, mb_nb_L1, mb_nb_U, mb_nb_U1, mb_nb_UR, mb_nb_UL;
    int pRefFrame[2][6];
    int pmv[2][2][6];
    int i, j, dir;
    int bid_flag = 0, bw_flag = 0, fw_flag = 0, sym_flag = 0, bid2 = 0, fw2 = 0;



    blockshape_x = blockshape_x % MIN_BLOCK_SIZE == 0 ? blockshape_x : MIN_BLOCK_SIZE;
    blockshape_y = blockshape_y % MIN_BLOCK_SIZE == 0 ? blockshape_y : MIN_BLOCK_SIZE;

    mb_available_up      = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           mb_width  ].slice_nr);
    mb_available_left    = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           1         ].slice_nr);
    mb_available_upleft  = (mb_x == 0 ||
                            mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - mb_width - 1].slice_nr);

    if ((pic_block_y == 0) || ((pic_block_x << MIN_BLOCK_SIZE_IN_BIT) + blockshape_x == img->width)) {
        mb_available_upright = 1;
    } else if (mb_pix_y > 0) {
        mb_available_upright = 0;
    } else {
        mb_available_upright = (img->mb_data[uiPositionInPic].slice_nr != img->mb_data[uiPositionInPic - img->PicWidthInMbs +
                                (mb_pix_x_temp + blockshape_x) / MIN_CU_SIZE].slice_nr);
    }

    block_available_up   = mb_available_up   || (mb_pix_y > 0);
    block_available_left = mb_available_left || (mb_pix_x > 0);
    block_available_left1 = block_available_left && (blockshape_y > MIN_BLOCK_SIZE);
    block_available_up1 = block_available_up && (blockshape_x > MIN_BLOCK_SIZE);


    if (input->g_uiMaxSizeInBit == B64X64_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix64[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    } else if (input->g_uiMaxSizeInBit == B32X32_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix32[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    } else if (input->g_uiMaxSizeInBit == B16X16_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix16[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];
    }

    if ((pic_block_x << MIN_BLOCK_SIZE_IN_BIT) + blockshape_x >= img->width) {
        upright = 0;
    }

    block_available_upright = upright && (!mb_available_upright);

    if (mb_pix_x > 0) {
        block_available_upleft = (mb_pix_y > 0 ? 1 : block_available_up);
    } else if (mb_pix_y > 0) {
        block_available_upleft = block_available_left;
    } else {
        block_available_upleft = mb_available_upleft;
    }
    mb_nb_L1 = mb_width * ((pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1) >> 1) + ((pic_block_x - 1) >> 1);
    mb_nb_L = mb_width * ((pic_block_y  - off_y) >> 1) + ((pic_block_x - 1) >> 1);
    mb_nb_U = mb_width * ((pic_block_y - y_up) >> 1) + ((pic_block_x) >> 1);
    mb_nb_U1 = mb_width * ((pic_block_y - y_up) >> 1) + ((pic_block_x + blockshape_x / MIN_BLOCK_SIZE - 1) >> 1);
    mb_nb_UR = mb_width * ((pic_block_y - y_upright) >> 1) + ((pic_block_x + blockshape_x / MIN_BLOCK_SIZE) >> 1);
    mb_nb_UL = mb_width * ((pic_block_y - y_upleft) >> 1) + ((pic_block_x - 1) >> 1);

    rFrameL[0]    = block_available_left    ? hc->refFrArr[pic_block_y  - off_y]  [pic_block_x - 1] : -1;
    rFrameU[0]    = block_available_up      ? hc->refFrArr[pic_block_y - y_up][pic_block_x]   : -1;
    rFrameUR[0]   = block_available_upright ? hc->refFrArr[pic_block_y - y_upright][pic_block_x +
                    (blockshape_x / MIN_BLOCK_SIZE) ] :  -1;
    rFrameUL[0]   = block_available_upleft  ? hc->refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameL1[0] = block_available_left1 ? hc->refFrArr[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x
                  - 1] : -1;
    rFrameU1[0] = block_available_up1 ? hc->refFrArr[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE - 1] :
                  -1;


    rFrameL[1]    = block_available_left    ? hc->p_snd_refFrArr[pic_block_y  - off_y]  [pic_block_x - 1] : -1;
    rFrameU[1]    = block_available_up      ? hc->p_snd_refFrArr[pic_block_y - y_up][pic_block_x]   : -1;
    rFrameUR[1]   = block_available_upright ? hc->p_snd_refFrArr[pic_block_y - y_upright][pic_block_x +
                    (blockshape_x / MIN_BLOCK_SIZE) ] :  -1;
    rFrameUL[1]   = block_available_upleft  ? hc->p_snd_refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameL1[1] = block_available_left1 ? hc->p_snd_refFrArr[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE -
                  1][pic_block_x - 1] : -1;
    rFrameU1[1] = block_available_up1 ? hc->p_snd_refFrArr[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                  1]
                  : -1;


    for (i = 0; i < 2; i++) {
        pRefFrame[i][0] = rFrameUL[i];
        pRefFrame[i][1] = rFrameU[i];
        pRefFrame[i][2] = rFrameL[i];
        pRefFrame[i][3] = rFrameUR[i];
        pRefFrame[i][4] = rFrameU1[i];
        pRefFrame[i][5] = rFrameL1[i];
    }
    for (hv = 0; hv < 2; hv++) {
        mva[0][hv]  = block_available_left    ? img->tmp_mv[pic_block_y - off_y][pic_block_x - 1][hv]              : 0;
        mva1[0][hv]  = block_available_left1 ? img->tmp_mv[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x
                       - 1][hv] : 0;
        mvb[0][hv] = block_available_up      ? img->tmp_mv[pic_block_y - y_up][pic_block_x][hv]                : 0;
        mvb1[0][hv]  = block_available_up1 ? img->tmp_mv[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                       1][hv] : 0;
        mve[0][hv] = block_available_upleft  ? img->tmp_mv[pic_block_y - y_upleft][pic_block_x - 1][hv]              : 0;
        mvc[0][hv]  = block_available_upright ? img->tmp_mv[pic_block_y - y_upright][pic_block_x +
                      (blockshape_x / MIN_BLOCK_SIZE) ][hv] : 0;

        mva[1][hv]  = block_available_left    ? img->p_snd_tmp_mv[pic_block_y - off_y][pic_block_x - 1][hv]              : 0;
        mva1[1][hv]  = block_available_left1 ? img->p_snd_tmp_mv[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE -
                       1][pic_block_x - 1][hv] : 0;
        mvb[1][hv] = block_available_up      ? img->p_snd_tmp_mv[pic_block_y - y_up][pic_block_x][hv]                : 0;
        mvb1[1][hv]  = block_available_up1 ? img->p_snd_tmp_mv[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                       1][hv] : 0;
        mve[1][hv] = block_available_upleft  ? img->p_snd_tmp_mv[pic_block_y - y_upleft][pic_block_x - 1][hv]             : 0;
        mvc[1][hv]  = block_available_upright ? img->p_snd_tmp_mv[pic_block_y - y_upright][pic_block_x +
                      (blockshape_x / MIN_BLOCK_SIZE) ][hv] : 0;

        for (i = 0; i < 2; i++) {
            pmv[i][hv][0] = mve[i][hv];   //i:0-first;1-second, hv-x or y
            pmv[i][hv][1] = mvb[i][hv];
            pmv[i][hv][2] = mva[i][hv];
            pmv[i][hv][3] = mvc[i][hv];
            pmv[i][hv][4] = mvb1[i][hv];
            pmv[i][hv][5] = mva1[i][hv];
        }
    }

    for (dir = 0; dir < MH_PSKIP_NUM + NUM_OFFSET + 1; dir++) {
        for (i = 0; i < 3; i++) {
            img->tmp_fstPSkipMv[dir][i] = 0;
            img->tmp_sndPSkipMv[dir][i] = 0;
        }
        img->tmp_pref_fst[dir] = 0;
        img->tmp_pref_snd[dir] = 0;
    }

    for (j = 0; j < 6; j++) {
        // bid
        if (pRefFrame[0][j] != -1 && pRefFrame[1][j] != -1) {
            img->tmp_pref_fst[BID_P_FST] = pRefFrame[0][j];
            img->tmp_pref_snd[BID_P_FST] = pRefFrame[1][j];
            img->tmp_fstPSkipMv[BID_P_FST][0] = pmv[0][0][j];
            img->tmp_fstPSkipMv[BID_P_FST][1] = pmv[0][1][j];
            img->tmp_sndPSkipMv[BID_P_FST][0] = pmv[1][0][j];
            img->tmp_sndPSkipMv[BID_P_FST][1] = pmv[1][1][j];
            bid_flag++;
            if (bid_flag == 1) {
                bid2 = j;
            }
        }
        // fw
        else if (pRefFrame[0][j] != -1 && pRefFrame[1][j] == -1) {
            img->tmp_pref_fst[FW_P_FST] = pRefFrame[0][j];
            img->tmp_fstPSkipMv[FW_P_FST][0] = pmv[0][0][j];
            img->tmp_fstPSkipMv[FW_P_FST][1] = pmv[0][1][j];
            fw_flag++;
            if (fw_flag == 1) {
                fw2 = j;
            }
        }
    }

    //first bid
    if (bid_flag == 0 && fw_flag > 1) {
        img->tmp_pref_fst[BID_P_FST] = img->tmp_pref_fst[FW_P_FST];
        img->tmp_pref_snd[BID_P_FST] = pRefFrame[0][fw2];

        img->tmp_fstPSkipMv[BID_P_FST][0] = img->tmp_fstPSkipMv[FW_P_FST][0];
        img->tmp_fstPSkipMv[BID_P_FST][1] = img->tmp_fstPSkipMv[FW_P_FST][1];
        img->tmp_sndPSkipMv[BID_P_FST][0] = pmv[0][0][fw2];
        img->tmp_sndPSkipMv[BID_P_FST][1] = pmv[0][1][fw2];
    }

    //second bid
    if (bid_flag > 1) {
        img->tmp_pref_fst[BID_P_SND] = pRefFrame[0][bid2];
        img->tmp_pref_snd[BID_P_SND] = pRefFrame[1][bid2];
        img->tmp_fstPSkipMv[BID_P_SND][0] = pmv[0][0][bid2];
        img->tmp_fstPSkipMv[BID_P_SND][1] = pmv[0][1][bid2];
        img->tmp_sndPSkipMv[BID_P_SND][0] = pmv[1][0][bid2];
        img->tmp_sndPSkipMv[BID_P_SND][1] = pmv[1][1][bid2];
    } else if (bid_flag == 1 && fw_flag > 1) {
        img->tmp_pref_fst[BID_P_SND] = img->tmp_pref_fst[FW_P_FST];
        img->tmp_pref_snd[BID_P_SND] = pRefFrame[0][fw2];
        img->tmp_fstPSkipMv[BID_P_SND][0] = img->tmp_fstPSkipMv[FW_P_FST][0];
        img->tmp_fstPSkipMv[BID_P_SND][1] = img->tmp_fstPSkipMv[FW_P_FST][1];
        img->tmp_sndPSkipMv[BID_P_SND][0] = pmv[0][0][fw2];
        img->tmp_sndPSkipMv[BID_P_SND][1] = pmv[0][1][fw2];
    }

    //first fw
    if (fw_flag == 0 && bid_flag > 1) {
        img->tmp_pref_fst[FW_P_FST] = pRefFrame[0][bid2];
        img->tmp_fstPSkipMv[FW_P_FST][0] = pmv[0][0][bid2];
        img->tmp_fstPSkipMv[FW_P_FST][1] = pmv[0][1][bid2];
    } else if (fw_flag == 0 && bid_flag == 1) {
        img->tmp_pref_fst[FW_P_FST] = img->tmp_pref_fst[BID_P_FST];
        img->tmp_fstPSkipMv[FW_P_FST][0] = img->tmp_fstPSkipMv[BID_P_FST][0];
        img->tmp_fstPSkipMv[FW_P_FST][1] = img->tmp_fstPSkipMv[BID_P_FST][1];
    }

    // second fw
    if (fw_flag > 1) {
        img->tmp_pref_fst[FW_P_SND] = pRefFrame[0][fw2];
        img->tmp_fstPSkipMv[FW_P_SND][0] = pmv[0][0][fw2];
        img->tmp_fstPSkipMv[FW_P_SND][1] = pmv[0][1][fw2];
    } else if (bid_flag > 1) {
        img->tmp_pref_fst[FW_P_SND] = pRefFrame[1][bid2];
        img->tmp_fstPSkipMv[FW_P_SND][0] = pmv[1][0][bid2];
        img->tmp_fstPSkipMv[FW_P_SND][1] = pmv[1][1][bid2];
    } else if (bid_flag == 1) {
        img->tmp_pref_fst[FW_P_SND] = img->tmp_pref_snd[BID_P_FST];
        img->tmp_fstPSkipMv[FW_P_SND][0] = img->tmp_sndPSkipMv[BID_P_FST][0];
        img->tmp_fstPSkipMv[FW_P_SND][1] = img->tmp_sndPSkipMv[BID_P_FST][1];
    }
}


/*
*************************************************************************
* Function:Initialize the motion search
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void Init_Motion_Search_Module()
{
    int bits, i, imin, imax, k, l;
    int search_range               = input->search_range;
    int number_of_reference_frames = img->buf_cycle;
    int max_search_points          = (2 * search_range + 1) * (2 * search_range + 1);
    int max_ref_bits               = 1 + 2 * (int) floor(log(max(16,
                                     number_of_reference_frames + 1)) / log(2) + 1e-10);
    int max_ref                    = (1 << ((max_ref_bits >> 1) + 1)) - 1;
    int number_of_subpel_positions = 4 * (2 * search_range + 3);
    int max_mv_bits                = 3 + 2 * (int) ceil(log(number_of_subpel_positions + 1) / log(2) + 1e-10);

    max_mvd                        = (1 << ((max_mv_bits >> 1))) - 1;

    //=====   CREATE ARRAYS   =====
    //-----------------------------
    if ((spiral_search_x = (int *) calloc(max_search_points, sizeof(int))) == NULL) {
        no_mem_exit("Init_Motion_Search_Module: spiral_search_x");
    }
    if ((spiral_search_y = (int *) calloc(max_search_points, sizeof(int))) == NULL) {
        no_mem_exit("Init_Motion_Search_Module: spiral_search_y");
    }
    if ((mvbits = (int *) calloc(2 * max_mvd + 1, sizeof(int))) == NULL) {
        no_mem_exit("Init_Motion_Search_Module: mvbits");
    }
    if ((refbits = (int *) calloc(max_ref, sizeof(int))) == NULL) {
        no_mem_exit("Init_Motion_Search_Module: refbits");
    }
    if ((byte_abs = (int *) calloc(2048, sizeof(int))) == NULL) {
        no_mem_exit("Init_Motion_Search_Module: byte_abs");
    }


    get_mem3Dint(&motion_cost, 9, 2 * (BUF_CYCLE + 1), 4);
    get_mem3Dint(&motion_cost_sym, 9, 2 * (BUF_CYCLE + 1), 4);
    get_mem3Dint(&motion_cost_dual, 9, 2 * (BUF_CYCLE + 1), 4);

    if ((int_motion_cost = (int *) calloc(2 * (img->buf_cycle + 1), sizeof(int))) == NULL) {
        no_mem_exit("Init_Motion_Search_Module: int_motion_cost");
    }


    //--- set array offsets ---
    mvbits   += max_mvd;
    byte_abs += 1024;

    //=====   INIT ARRAYS   =====
    //---------------------------
    //--- init array: motion vector bits ---
    mvbits[0] = 1;

    for (bits = 3; bits <= max_mv_bits; bits += 2) {
        imax = 1    << (bits >> 1);
        imin = imax >> 1;

        for (i = imin;

             i < imax;
             i++) {
            mvbits[-i] = mvbits[i] = bits;
        }
    }
    for (i = 0; i <= MAX_MVP_CAND_NUM; i++) {
        for (k = 0; k < MAX_MVP_CAND_NUM; k++) {
            if (i == 1 || i == 0) {
                mvpidxbits[k][i] = 0;
            } else if (k == i - 1) {
                mvpidxbits[k][i] = k;
            } else {
                mvpidxbits[k][i] = k + 1;
            }
        }
    }
    //--- init array: reference frame bits ---
    refbits[0] = 1;

    for (bits = 3; bits <= max_ref_bits; bits += 2) {
        imax = (1   << ((bits >> 1) + 1)) - 1;
        imin = imax >> 1;

        for (i = imin;

             i < imax;
             i++) {
            refbits[i] = bits;
        }
    }
    //--- init array: absolute value ---
    byte_abs[0] = 0;

    for (i = 1; i < 1024; i++) {
        byte_abs[i] = byte_abs[-i] = i;
    }
    //--- init array: search pattern ---
    spiral_search_x[0] = spiral_search_y[0] = 0;
    for (k = 1, l = 1; l <= max(1, search_range); l++) {
        for (i = -l + 1; i < l; i++) {
            spiral_search_x[k] =  i;
            spiral_search_y[k++] = -l;
            spiral_search_x[k] =  i;
            spiral_search_y[k++] =  l;
        }

        for (i = -l;   i <= l; i++) {
            spiral_search_x[k] = -l;
            spiral_search_y[k++] =  i;
            spiral_search_x[k] =  l;
            spiral_search_y[k++] =  i;
        }
    }
}



/*
*************************************************************************
* Function:Free memory used by motion search
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void Clear_Motion_Search_Module()
{
    //--- correct array offset ---
    mvbits   -= max_mvd;
    byte_abs -= 1024;

    //--- delete arrays ---
    free(spiral_search_x);
    free(spiral_search_y);
    free(mvbits);
    free(refbits);
    free(byte_abs);

    free_mem3Dint(motion_cost, 9);
    free_mem3Dint(motion_cost_sym, 9);
    free_mem3Dint(motion_cost_dual, 9);


    free(int_motion_cost);

}

int pmvr_adapt_mv1(int *cand_mv_x, int *cand_mv_y, int ctr_x, int ctr_y, int mv_x, int mv_y, int pos)
{
    //jcma
    if (abs(mv_x - ctr_x) > TH || abs(mv_y - ctr_y) > TH) {
        *cand_mv_x = mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
        *cand_mv_y = mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units
        return (abs(*cand_mv_x - ctr_x) > TH || abs(*cand_mv_y - ctr_y) > TH);
    } else {
        *cand_mv_x = mv_x + spiral_search_x[pos];    // quarter-pel units
        *cand_mv_y = mv_y + spiral_search_y[pos];    // quarter-pel units
        return (abs(*cand_mv_x - ctr_x) <= TH && abs(*cand_mv_y - ctr_y) <= TH);
    }
}

/*
*************************************************************************
* Function:Full pixel block motion search
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


int FullPelBlockMotionSearch(pel_t orig_pic[MAX_CU_SIZE][MAX_CU_SIZE], int ref, int pic_pix_x, int pic_pix_y,
                             int blocktype, int pred_mv_x, int pred_mv_y, int *mv_x, int *mv_y, int search_range, int min_mcost, double lambda,
                             int bit_size, int block)
{
    int   pos, cand_x, cand_y, y, mcost;
    pel_t *orig_line, *ref_line;
    pel_t * (*get_ref_line)(int, pel_t *, int, int);
    pel_t  *ref_pic     = img->type == B_IMG ? he->Refbuf11 [ref + 1] : he->Refbuf11[ref];

    int   best_pos      = 0;                                        // position with minimum motion cost
    int   max_pos       = (2 * search_range + 1) * (2 * search_range + 1);     // number of search positions
    int   lambda_factor = LAMBDA_FACTOR(lambda);                    // factor for determining lagragian motion cost
    int   blocksize_y   = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);    // vertical block size
    int   blocksize_x   = g_blk_size[blocktype * 2 + block][0] << (bit_size -
                          MIN_CU_SIZE_IN_BIT);   // horizontal block size


    int   blocksize_x4  = blocksize_x >> 2;                         // horizontal block size in 4-pel units
    int   x4;

    int   pred_x        = (pic_pix_x << 2) + pred_mv_x;       // predicted position x (in sub-pel units)
    int   pred_y        = (pic_pix_y << 2) + pred_mv_y;       // predicted position y (in sub-pel units)
    int   center_x      = pic_pix_x + *mv_x;                        // center position x (in pel units)
    int   center_y      = pic_pix_y + *mv_y;                        // center position y (in pel units)

    int   height        = img->height;/*lgp*/

    if (input->bg_enable && he->background_reference_enable &&
        ref == img->num_of_references - 1 &&
        (img->type == P_IMG || img->type == F_IMG) && img->typeb != BP_IMG) {
        ref_pic = &he->background_frame[0][0][0];
    }

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        ref_pic = &he->background_frame[0][0][0];
    }


    //===== set function for getting reference picture lines =====
    if ((center_x > search_range) && (center_x < img->width - 1 - search_range - blocksize_x) &&
        (center_y > search_range) && (center_y < height - 1 - search_range - blocksize_y)) {
        get_ref_line = FastLineX; //add by wuzhongmou 0610
    } else { //add by wuzhongmou 0610
        get_ref_line = UMVLineX;
    }

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        cand_x = center_x;
        cand_y = center_y;

        mcost = MV_COST(lambda_factor, 2, cand_x, cand_y, pred_x, pred_y);

        //--- add residual cost to motion cost ---
        for (y = 0; y < blocksize_y; y++) {
            ref_line  = get_ref_line(blocksize_x, ref_pic, cand_y + y, cand_x);
            orig_line = orig_pic [y];

            for (x4 = 0; x4 < blocksize_x4; x4++) {
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
            }

        }
        min_mcost = mcost;
        *mv_x = 0;
        *mv_y = 0;
        return min_mcost;
    }


    //===== loop over all search positions =====
    for (pos = 0; pos < max_pos; pos++) {
        //--- set candidate position (absolute position in pel units) ---
        cand_x = center_x + spiral_search_x[pos];
        cand_y = center_y + spiral_search_y[pos];

        //--- initialize motion cost (cost for motion vector) and check ---
        mcost = MV_COST(lambda_factor, 2, cand_x, cand_y, pred_x, pred_y);

        if (mcost >= min_mcost) {
            continue;
        }

        //--- add residual cost to motion cost ---
        for (y = 0; y < blocksize_y; y++) {
            ref_line  = get_ref_line(blocksize_x, ref_pic, cand_y + y, cand_x);
            orig_line = orig_pic [y];

            for (x4 = 0; x4 < blocksize_x4; x4++) {
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
                mcost += byte_abs[ *orig_line++ - *ref_line++ ];
            }
            if (mcost >= min_mcost) {
                break;
            }
        }

        //--- check if motion cost is less than minimum cost ---
        if (mcost < min_mcost) {
            best_pos  = pos;
            min_mcost = mcost;
        }
    }

    //===== set best motion vector and return minimum motion cost =====
    if (best_pos) {
        *mv_x += spiral_search_x[best_pos];
        *mv_y += spiral_search_y[best_pos];
    }

    return min_mcost;
}


/*
*************************************************************************
* Function:Sub pixel block motion search
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int SubPelBlockMotionSearch(pel_t orig_pic[MAX_CU_SIZE][MAX_CU_SIZE], int ref, int pic_pix_x, int pic_pix_y,
                            int blocktype, int pred_mv_x, int pred_mv_y,
                            int      *mv_x,        // <--> in: search center (x) / out: motion vector (x) - in pel units
                            int *mv_y, int search_pos2, int search_pos4 , int min_mcost , double lambda, int bit_size, int block)
{
    int   diff[64], *d;
    int   pos, best_pos, mcost, abort_search;
    int   y0, x0, ry0, rx0, ry;
    int   cand_mv_x, cand_mv_y;
    pel_t *orig_line;
    int   incr            = 0;//qyu 0926
    pel_t **ref_pic;
    int   lambda_factor   = LAMBDA_FACTOR(lambda);
    int   mv_shift        = 0;

    int   blocksize_x     = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_y     = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);

    int   pic4_pix_x      = (pic_pix_x << 2);
    int   pic4_pix_y      = (pic_pix_y << 2);
    int   max_pos_x4      = ((img->width - blocksize_x) << 2);
    int   max_pos_y4      = ((img->height - blocksize_y) << 2);
    int   min_pos2        = (input->hadamard ? 0 : 1);
    int   max_pos2        = (input->hadamard ? max(1, search_pos2) : search_pos2);

    int   curr_diff[64][64]; // for AVS 8x8 SATD calculation
    int   xx, yy, kk;                              // indicees for curr_diff

    int   ctr_x = (pred_mv_x >> 1) << 1;
    int   ctr_y = (pred_mv_y >> 1) << 1;

    if (img->type == B_IMG) {
        incr = 1;
    }

    ref_pic = img->type == B_IMG ? fref[ref + incr]->oneForthRefY  : fref[ref]->oneForthRefY ;

    if (input->bg_enable && he->background_reference_enable &&
        ref == img->num_of_references - 1 &&
        (img->type == P_IMG || img->type == F_IMG)) {
        ref_pic = he->background_frame_quarter;
    }

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        ref_pic = he->background_frame_quarter;
    }


    //////////////////////////////////////////////////////////////////

    /*********************************
    *****                       *****
    *****  HALF-PEL REFINEMENT  *****
    *****                       *****
    *********************************/
    //===== convert search center to quarter-pel units =====
    *mv_x <<= 2;
    *mv_y <<= 2;

    //===== set function for getting pixel values =====
    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)) {
        PelY_14 = FastPelY_14;
    } else {
        PelY_14 = UMVPelY_14;
    }

    //===== loop over search positions =====
    for (best_pos = 0, pos = min_pos2; pos < max_pos2; pos++) {
        cand_mv_x = *mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
        cand_mv_y = *mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units

        //----- set motion vector cost -----
        mcost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

        //----- add up SATD -----
        for (y0 = 0, abort_search = 0; y0 < blocksize_y && !abort_search; y0 += 4) {
            ry0 = ((pic_pix_y + y0) << 2) + cand_mv_y;

            for (x0 = 0; x0 < blocksize_x; x0 += 4) {
                rx0 = ((pic_pix_x + x0) << 2) + cand_mv_x;
                d   = diff;

                orig_line = orig_pic [y0  ];
                ry = ry0;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d++      = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                orig_line = orig_pic [y0 + 1];
                ry = ry0 + 4;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d++      = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                orig_line = orig_pic [y0 + 2];
                ry = ry0 + 8;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d++      = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                orig_line = orig_pic [y0 + 3];
                ry = ry0 + 12;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d        = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                for (yy = y0, kk = 0; yy < y0 + 4; yy++) {
                    for (xx = x0; xx < x0 + 4; xx++, kk++) {
                        curr_diff[yy][xx] = diff[kk];
                    }
                }
            }
        }

        mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);

        if (mcost < min_mcost) {
            min_mcost = mcost;
            best_pos  = pos;
        }
    }

    if (best_pos) {
        *mv_x += (spiral_search_x [best_pos] << 1);
        *mv_y += (spiral_search_y [best_pos] << 1);
    }

    /************************************
    *****                          *****
    *****  QUARTER-PEL REFINEMENT  *****
    *****                          *****
    ************************************/
    //===== set function for getting pixel values =====
    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 1) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 1)) {
        PelY_14 = FastPelY_14;
    } else {
        PelY_14 = UMVPelY_14;
    }

    //===== loop over search positions =====
    for (best_pos = 0, pos = 1; pos < search_pos4; pos++) {
        if (input->b_pmvr_enabled) {
            if (!pmvr_adapt_mv1(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, *mv_x, *mv_y, pos)) {
                continue;
            }
        } else {
            cand_mv_x = *mv_x + spiral_search_x[pos];    // quarter-pel units
            cand_mv_y = *mv_y + spiral_search_y[pos];    // quarter-pel units
        }

        // xiaozhen zheng, mv_rang, 20071009
#if Mv_check_bug
		img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
		img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype);
#endif
        

        if (!img->mv_range_flag) {
            img->mv_range_flag = 1;
            continue;
        }

        // xiaozhen zheng, mv_rang, 20071009

        //----- set motion vector cost -----
        mcost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

        //----- add up SATD -----
        for (y0 = 0, abort_search = 0; y0 < blocksize_y && !abort_search; y0 += 4) {
            ry0 = ((pic_pix_y + y0) << 2) + cand_mv_y;

            for (x0 = 0; x0 < blocksize_x; x0 += 4) {
                rx0 = ((pic_pix_x + x0) << 2) + cand_mv_x;
                d   = diff;

                orig_line = orig_pic [y0  ];
                ry = ry0;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d++      = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                orig_line = orig_pic [y0 + 1];
                ry = ry0 + 4;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d++      = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                orig_line = orig_pic [y0 + 2];
                ry = ry0 + 8;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d++      = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                orig_line = orig_pic [y0 + 3];
                ry = ry0 + 12;
                *d++      = orig_line[x0  ]  -  PelY_14(ref_pic, ry, rx0);
                *d++      = orig_line[x0 + 1]  -  PelY_14(ref_pic, ry, rx0 + 4);
                *d++      = orig_line[x0 + 2]  -  PelY_14(ref_pic, ry, rx0 + 8);
                *d        = orig_line[x0 + 3]  -  PelY_14(ref_pic, ry, rx0 + 12);

                for (yy = y0, kk = 0; yy < y0 + 4; yy++) {
                    for (xx = x0; xx < x0 + 4; xx++, kk++) {
                        curr_diff[yy][xx] = diff[kk];
                    }
                }
            }
        }

        mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);

        if (mcost < min_mcost) {
            min_mcost = mcost;
            best_pos  = pos;
        }
    }

    if (best_pos) {
        if (input->b_pmvr_enabled) {
            pmvr_adapt_mv1(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, *mv_x, *mv_y, best_pos);
            *mv_x = cand_mv_x;
            *mv_y = cand_mv_y;
        } else {
            *mv_x += spiral_search_x [best_pos];
            *mv_y += spiral_search_y [best_pos];
        }
    }

    //===== return minimum motion cost =====
    return min_mcost;
}

/*
*************************************************************************
* Function:Sub pixel block motion search
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int SubPelBlockMotionSearch_sym(pel_t orig_pic[MAX_CU_SIZE][MAX_CU_SIZE], int ref, int pic_pix_x, int pic_pix_y,
                                int blocktype, int pred_mv_x,   // <--  motion vector predictor (x) in sub-pel units
                                int pred_mv_y, int *mv_x, int *mv_y, int search_pos2, int search_pos4, int min_mcost, double lambda, int bit_size,
                                int block)

{
    int   diff[64], *d;
    int   pos, best_pos, mcost, abort_search;
    int   y0, x0, ry0, rx0, ry;
    int xx, yy, kk;
    int   curr_diff[MAX_CU_SIZE][MAX_CU_SIZE]; // for AVS 8x8 SATD calculation
    int ry0_sym, rx0_sym, ry_sym;
    int   cand_mv_x, cand_mv_y;
    pel_t *orig_line;
    int   incr            = 0;//qyu 0926
    pel_t **ref_pic, **ref_pic_sym;
    int   lambda_factor   = LAMBDA_FACTOR(lambda);
    int   mv_shift        = 0;

    int   blocksize_x     = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_y     = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);

    int   pic4_pix_x      = (pic_pix_x << 2);
    int   pic4_pix_y      = (pic_pix_y << 2);
    int   max_pos_x4      = ((img->width - blocksize_x) << 2);
    int   max_pos_y4      = ((img->height - blocksize_y) << 2);
    int   min_pos2        = (input->hadamard ? 0 : 1);
    int   max_pos2        = (input->hadamard ? max(1, search_pos2) : search_pos2);
    int   delta_P, TRp, DistanceIndexFw, DistanceIndexBw, refframe , delta_PB;
    int   ctr_x = (pred_mv_x >> 1) << 1;
    int   ctr_y = (pred_mv_y >> 1) << 1;
    int   snd_ref = ref == 0 ? 1 : 0;
    if (img->type == B_IMG) {
        refframe = ref;
        delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
        delta_P = (delta_P + 512) % 512;   // Added by Xiaozhen ZHENG, 2007.05.05

        TRp = (refframe + 1) * delta_P;
        delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);   // Tsinghua 200701
        TRp  = (TRp + 512) % 512;
        delta_PB = (delta_PB + 512) % 512;   // Added by Xiaozhen ZHENG, 2007.05.05

        DistanceIndexFw = delta_PB;

        //DistanceIndexBw    = TRp - DistanceIndexFw;
        DistanceIndexBw    = (TRp - DistanceIndexFw + 512) % 512;   // Added by Zhijie Yang, 20070419, Broadcom

        //xyji 11.23
        if (img->type == B_IMG) {
            incr = 1;
        }

        ref_pic = img->type == B_IMG ? fref[ref + incr]->oneForthRefY  : fref[ref]->oneForthRefY;

        ref_pic_sym = img->type == B_IMG ? fref[0]->oneForthRefY : fref[ref]->oneForthRefY;
    } else if (img->type == F_IMG) {
        DistanceIndexFw = 2 * (hc->picture_distance - fref[ref]->imgtr_fwRefDistance);
        DistanceIndexFw = (DistanceIndexFw + 512) % 512;
        DistanceIndexBw = 2 * (hc->picture_distance - fref[snd_ref]->imgtr_fwRefDistance);
        DistanceIndexBw = (DistanceIndexBw + 512) % 512;
        if (ref == img->num_of_references - 1 && he->background_reference_enable) {
            DistanceIndexFw = 1;
        }
        if (snd_ref == img->num_of_references - 1 && he->background_reference_enable) {
            DistanceIndexBw = 1;
        }

        ref_pic = fref[ref]->oneForthRefY;
        ref_pic_sym = fref[snd_ref]->oneForthRefY;
    }
    /*********************************
    *****                       *****
    *****  HALF-PEL REFINEMENT  *****
    *****                       *****
    *********************************/
    //===== convert search center to quarter-pel units =====
    *mv_x <<= 2;
    *mv_y <<= 2;

    //===== set function for getting pixel values =====
    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)) {
        PelY_14 = UMVPelY_14;//FastPelY_14;//xyji
    } else {
        PelY_14 = UMVPelY_14;
    }

    //===== loop over search positions =====
    for (best_pos = 0, pos = min_pos2; pos < max_pos2; pos++) {
        cand_mv_x = *mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
        cand_mv_y = *mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units

        //----- set motion vector cost -----
        mcost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

        //----- add up SATD -----
        for (y0 = 0, abort_search = 0; y0 < blocksize_y && !abort_search; y0 += 4) {
            ry0 = ((pic_pix_y + y0) << 2) + cand_mv_y;
#if MV_SCALE
            if (img->type == B_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) - scale_mv(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
            } else if (img->type == F_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) + scale_mv(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
            }
#if HALF_PIXEL_COMPENSATION_MVD
            if (img->is_field_sequence) {
                if (img->type == B_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) - scale_mv_y2(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
                } else if (img->type == F_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) + scale_mv_y1(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
                }
            }
#endif
#else
            if (img->type == B_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) - ((cand_mv_y * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                     OFFSET);
            } else if (img->type == F_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) + ((cand_mv_y * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                     OFFSET);
            }
#if HALF_PIXEL_COMPENSATION_MVD
            if (img->is_field_sequence) {
                int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                oriPOC = 2 * hc->picture_distance;
                oriRefPOC = oriPOC - DistanceIndexFw;
                scaledPOC = 2 * hc->picture_distance;
                scaledRefPOC = scaledPOC - DistanceIndexBw;
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                if (img->type == B_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) - (((cand_mv_y + delta) * DistanceIndexBw *
                                                          (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                } else if (img->type == F_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) + (((cand_mv_y + delta) * DistanceIndexBw *
                                                          (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                }
            }
#endif
#endif
            ry0_sym = Clip3(-32768, 32767, ry0_sym);

            for (x0 = 0; x0 < blocksize_x; x0 += 4) {
                rx0 = ((pic_pix_x + x0) << 2) + cand_mv_x;
#if MV_SCALE
                if (img->type == B_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) - scale_mv(cand_mv_x, DistanceIndexBw, DistanceIndexFw);
                } else if (img->type == F_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) + scale_mv(cand_mv_x, DistanceIndexBw, DistanceIndexFw);
                }
#else
                if (img->type == B_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) - ((cand_mv_x * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                         OFFSET);
                } else if (img->type == F_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) + ((cand_mv_x * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                         OFFSET);
                }
#endif
                rx0_sym = Clip3(-32768, 32767, rx0_sym);

                d   = diff;

                orig_line = orig_pic [y0  ];
                ry = ry0;
                ry_sym = ry0_sym;

                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 1];
                ry = ry0 + 4;
                ry_sym = ry0_sym + 4;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 2];
                ry = ry0 + 8;
                ry_sym = ry0_sym + 8;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 3];
                ry = ry0 + 12;
                ry_sym = ry0_sym + 12;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d        = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                for (yy = y0, kk = 0; yy < y0 + 4; yy++) {
                    for (xx = x0; xx < x0 + 4; xx++, kk++) {
                        curr_diff[yy][xx] = diff[kk];
                    }
                }
            }
        }

        mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);

        if (mcost < min_mcost) {
            min_mcost = mcost;
            best_pos  = pos;
        }
    }

    if (best_pos) {
        *mv_x += (spiral_search_x [best_pos] << 1);
        *mv_y += (spiral_search_y [best_pos] << 1);
    }


    /************************************
    *****                          *****
    *****  QUARTER-PEL REFINEMENT  *****
    *****                          *****
    ************************************/
    //===== set function for getting pixel values =====
    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 1) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 1)) {
        PelY_14 = UMVPelY_14;//FastPelY_14;//xyji
    } else {
        PelY_14 = UMVPelY_14;
    }

    //===== loop over search positions =====
    for (best_pos = 0, pos = 1; pos < search_pos4; pos++) {
        if (input->b_pmvr_enabled) {
            if (!pmvr_adapt_mv1(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, *mv_x, *mv_y, pos)) {
                continue;
            }
        } else {
            cand_mv_x = *mv_x + spiral_search_x[pos];    // quarter-pel units
            cand_mv_y = *mv_y + spiral_search_y[pos];    // quarter-pel units
        }

        // xiaozhen zheng, mv_rang, 20071009
#if Mv_check_bug
		img->mv_range_flag = check_mv_range_sym(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, ref, 0, 0);
#else
		img->mv_range_flag = check_mv_range_sym(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, ref);
#endif
        
        if (!img->mv_range_flag) {
            img->mv_range_flag = 1;
            continue;
        }

        // xiaozhen zheng, mv_rang, 20071009

        //----- set motion vector cost -----
        mcost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

        //----- add up SATD -----
        for (y0 = 0, abort_search = 0; y0 < blocksize_y && !abort_search; y0 += 4) {
            ry0 = ((pic_pix_y + y0) << 2) + cand_mv_y;
#if MV_SCALE
            if (img->type == B_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) - scale_mv(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
            } else if (img->type == F_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) + scale_mv(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
            }
#if HALF_PIXEL_COMPENSATION_MVD
            if (img->is_field_sequence) {
                if (img->type == B_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) - scale_mv_y2(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
                } else if (img->type == F_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) + scale_mv_y1(cand_mv_y, DistanceIndexBw, DistanceIndexFw);
                }
            }
#endif
#else
            if (img->type == B_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) - ((cand_mv_y * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                     OFFSET);
            } else if (img->type == F_IMG) {
                ry0_sym = ((pic_pix_y + y0) << 2) + ((cand_mv_y * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                     OFFSET);
            }
#if HALF_PIXEL_COMPENSATION_MVD
            if (img->is_field_sequence) {
                int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                oriPOC = 2 * hc->picture_distance;
                oriRefPOC = oriPOC - DistanceIndexFw;
                scaledPOC = 2 * hc->picture_distance;
                scaledRefPOC = scaledPOC - DistanceIndexBw;
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                if (img->type == B_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) - (((cand_mv_y + delta) * DistanceIndexBw *
                                                          (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                } else if (img->type == F_IMG) {
                    ry0_sym = ((pic_pix_y + y0) << 2) + (((cand_mv_y + delta) * DistanceIndexBw *
                                                          (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                }
            }
#endif
#endif
            for (x0 = 0; x0 < blocksize_x; x0 += 4) {
                rx0 = ((pic_pix_x + x0) << 2) + cand_mv_x;
#if MV_SCALE
                if (img->type == B_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) - scale_mv(cand_mv_x, DistanceIndexBw , DistanceIndexFw);
                } else if (img->type == F_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) + scale_mv(cand_mv_x, DistanceIndexBw, DistanceIndexFw);
                }
#else
                if (img->type == B_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) - ((cand_mv_x * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                         OFFSET);
                } else if (img->type == F_IMG) {
                    rx0_sym = ((pic_pix_x + x0) << 2) + ((cand_mv_x * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                                         OFFSET);
                }
#endif
                d   = diff;

                orig_line = orig_pic [y0  ];
                ry = ry0;
                ry_sym = ry0_sym;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym)) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4)) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8)) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12)) / 2;

                orig_line = orig_pic [y0 + 1];
                ry = ry0 + 4;
                ry_sym = ry0_sym + 4;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym)) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4)) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8)) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12)) / 2;

                orig_line = orig_pic [y0 + 2];
                ry = ry0 + 8;
                ry_sym = ry0_sym + 8;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym)) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4)) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8)) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12)) / 2;

                orig_line = orig_pic [y0 + 3];
                ry = ry0 + 12;
                ry_sym = ry0_sym + 12;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym)) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4)) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8)) / 2;
                *d        = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12)) / 2;

                for (yy = y0, kk = 0; yy < y0 + 4; yy++) {
                    for (xx = x0; xx < x0 + 4; xx++, kk++) {
                        curr_diff[yy][xx] = diff[kk];
                    }
                }
            }
        }

        mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);

        if (mcost < min_mcost) {
            min_mcost = mcost;
            best_pos  = pos;
        }
    }

    if (best_pos) {
        if (input->b_pmvr_enabled) {
            pmvr_adapt_mv1(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, *mv_x, *mv_y, best_pos);
            *mv_x = cand_mv_x;
            *mv_y = cand_mv_y;
        } else {
            *mv_x += spiral_search_x [best_pos];
            *mv_y += spiral_search_y [best_pos];
        }
    }

    //===== return minimum motion cost =====
    return min_mcost;
}


int SubPelBlockMotionSearch_bid(pel_t orig_pic[MAX_CU_SIZE][MAX_CU_SIZE], int ref, int pic_pix_x, int pic_pix_y,
                                int blocktype, int pred_mv_x,   // <--  motion vector predictor (x) in sub-pel units
                                int pred_mv_y, int *mv_x, int *mv_y, int pred_bmv_x_bid, int pred_bmv_y_bid, int *bmv_x_bid, int *bmv_y_bid,
                                int search_pos2, int search_pos4, int min_mcost, double lambda, int bit_size, int block)

{
    int   diff[64], *d;
    int   pos, best_pos, mcost, abort_search;
    int   y0, x0, ry0, rx0, ry;
    int xx, yy, kk;
    int   curr_diff[MAX_CU_SIZE][MAX_CU_SIZE]; // for AVS 8x8 SATD calculation
    int ry0_sym = 0, rx0_sym = 0, ry_sym = 0;
    int   cand_mv_x, cand_mv_y;

    int cand_bmv_x_bid, cand_bmv_y_bid;

    pel_t *orig_line;
    int   incr            = 0;//qyu 0926
    pel_t **ref_pic, **ref_pic_sym;
    int   lambda_factor   = LAMBDA_FACTOR(lambda);
    int   mv_shift        = 0;

    int   blocksize_x     = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_y     = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);

    int   pic4_pix_x      = (pic_pix_x << 2);
    int   pic4_pix_y      = (pic_pix_y << 2);
    int   max_pos_x4      = ((img->width - blocksize_x) << 2);
    int   max_pos_y4      = ((img->height - blocksize_y) << 2);
    int   min_pos2        = (input->hadamard ? 0 : 1);
    int   max_pos2        = (input->hadamard ? max(1, search_pos2) : search_pos2);
    int   delta_P, TRp, DistanceIndexFw, DistanceIndexBw, refframe , delta_PB;
    int   ctr_x = (pred_mv_x >> 1) << 1;
    int   ctr_y = (pred_mv_y >> 1) << 1;
    int snd_ref = 0;
    refframe = ref;
    delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
    delta_P = (delta_P + 512) % 512;   // Added by Xiaozhen ZHENG, 2007.05.05

    TRp = (refframe + 1) * delta_P;
    delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);   // Tsinghua 200701
    TRp  = (TRp + 512) % 512;
    delta_PB = (delta_PB + 512) % 512;   // Added by Xiaozhen ZHENG, 2007.05.05

    DistanceIndexFw = delta_PB;

    //DistanceIndexBw    = TRp - DistanceIndexFw;
    DistanceIndexBw    = (TRp - DistanceIndexFw + 512) % 512;   // Added by Zhijie Yang, 20070419, Broadcom

    //xyji 11.23

    if (img->type == B_IMG) {
        incr = 1;
    }

    ref_pic = img->type == B_IMG ? fref[ref + incr]->oneForthRefY : fref[ref]->oneForthRefY;
    ref_pic_sym = img->type == B_IMG ? fref[0]->oneForthRefY  : fref[snd_ref]->oneForthRefY;

    /*********************************
    *****                       *****
    *****  HALF-PEL REFINEMENT  *****
    *****                       *****
    *********************************/
    //===== convert search center to quarter-pel units =====
    *mv_x <<= 2;
    *mv_y <<= 2;

    //===== set function for getting pixel values =====
    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)) {
        PelY_14 = UMVPelY_14;//FastPelY_14;//xyji
    } else {
        PelY_14 = UMVPelY_14;
    }

    //===== loop over search positions =====
    for (best_pos = 0, pos = min_pos2; pos < max_pos2; pos++) {
        cand_mv_x = *mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
        cand_mv_y = *mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units

        //----- set motion vector cost -----
        mcost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

        cand_bmv_x_bid = *bmv_x_bid;
        cand_bmv_y_bid = *bmv_y_bid;
        mcost += MV_COST(lambda_factor, mv_shift, cand_bmv_x_bid, cand_bmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid);

        //----- add up SATD -----
        for (y0 = 0, abort_search = 0; y0 < blocksize_y && !abort_search; y0 += 4) {
            ry0 = ((pic_pix_y + y0) << 2) + cand_mv_y;

            ry0_sym = ((pic_pix_y + y0) << 2) + cand_bmv_y_bid;


            for (x0 = 0; x0 < blocksize_x; x0 += 4) {
                rx0 = ((pic_pix_x + x0) << 2) + cand_mv_x;

                rx0_sym = ((pic_pix_x + x0) << 2) + cand_bmv_x_bid;

                d   = diff;

                orig_line = orig_pic [y0  ];
                ry = ry0;
                ry_sym = ry0_sym;

                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 1];
                ry = ry0 + 4;
                ry_sym = ry0_sym + 4;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 2];
                ry = ry0 + 8;
                ry_sym = ry0_sym + 8;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 3];
                ry = ry0 + 12;
                ry_sym = ry0_sym + 12;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d        = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                for (yy = y0, kk = 0; yy < y0 + 4; yy++) {
                    for (xx = x0; xx < x0 + 4; xx++, kk++) {
                        curr_diff[yy][xx] = diff[kk];
                    }
                }
            }
        }

        mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);

        if (mcost < min_mcost) {
            min_mcost = mcost;
            best_pos  = pos;
        }
    }

    if (best_pos) {
        *mv_x += (spiral_search_x [best_pos] << 1);
        *mv_y += (spiral_search_y [best_pos] << 1);
    }


    /************************************
    *****                          *****
    *****  QUARTER-PEL REFINEMENT  *****
    *****                          *****
    ************************************/
    //===== set function for getting pixel values =====
    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 1) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 1)) {
        PelY_14 = UMVPelY_14;//FastPelY_14;//xyji
    } else {
        PelY_14 = UMVPelY_14;
    }

    //===== loop over search positions =====
    for (best_pos = 0, pos = 1; pos < search_pos4; pos++) {
        if (input->b_pmvr_enabled) {
            if (!pmvr_adapt_mv1(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, *mv_x, *mv_y, pos)) {
                continue;
            }
        } else {
            cand_mv_x = *mv_x + spiral_search_x[pos];    // quarter-pel units
            cand_mv_y = *mv_y + spiral_search_y[pos];    // quarter-pel units
        }
#if Mv_check_bug
		img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
		img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype);
#endif
        
        if (!img->mv_range_flag) {
            img->mv_range_flag = 1;
            continue;
        }
#if Mv_check_bug
		img->mv_range_flag = check_mv_range(bit_size, cand_bmv_x_bid, cand_bmv_y_bid, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
		img->mv_range_flag = check_mv_range(bit_size, cand_bmv_x_bid, cand_bmv_y_bid, pic_pix_x, pic_pix_y, blocktype);
#endif
        
        if (!img->mv_range_flag) {
            img->mv_range_flag = 1;
            continue;
        }
        // xiaozhen zheng, mv_rang, 20071009

        //----- set motion vector cost -----
        mcost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);


        cand_bmv_x_bid = *bmv_x_bid;
        cand_bmv_y_bid = *bmv_y_bid;
        mcost += MV_COST(lambda_factor, mv_shift, cand_bmv_x_bid, cand_bmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid);


        //----- add up SATD -----
        for (y0 = 0, abort_search = 0; y0 < blocksize_y && !abort_search; y0 += 4) {
            ry0 = ((pic_pix_y + y0) << 2) + cand_mv_y;


            ry0_sym = ((pic_pix_y + y0) << 2) + cand_bmv_y_bid;


            for (x0 = 0; x0 < blocksize_x; x0 += 4) {
                rx0 = ((pic_pix_x + x0) << 2) + cand_mv_x;


                rx0_sym = ((pic_pix_x + x0) << 2) + cand_bmv_x_bid;

                d   = diff;

                orig_line = orig_pic [y0  ];
                ry = ry0;
                ry_sym = ry0_sym;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 1];
                ry = ry0 + 4;
                ry_sym = ry0_sym + 4;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 2];
                ry = ry0 + 8;
                ry_sym = ry0_sym + 8;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d++      = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                orig_line = orig_pic [y0 + 3];
                ry = ry0 + 12;
                ry_sym = ry0_sym + 12;
                *d++      = orig_line[x0  ]  - (PelY_14(ref_pic, ry, rx0) + PelY_14(ref_pic_sym, ry_sym, rx0_sym) + 1) / 2;
                *d++      = orig_line[x0 + 1]  - (PelY_14(ref_pic, ry, rx0 + 4) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 4) + 1) / 2;
                *d++      = orig_line[x0 + 2]  - (PelY_14(ref_pic, ry, rx0 + 8) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 8) + 1) / 2;
                *d        = orig_line[x0 + 3]  - (PelY_14(ref_pic, ry, rx0 + 12) + PelY_14(ref_pic_sym, ry_sym,
                                                  rx0_sym + 12) + 1) / 2;

                for (yy = y0, kk = 0; yy < y0 + 4; yy++) {
                    for (xx = x0; xx < x0 + 4; xx++, kk++) {
                        curr_diff[yy][xx] = diff[kk];
                    }
                }
            }
        }

        mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);

        if (mcost < min_mcost) {
            min_mcost = mcost;
            best_pos  = pos;
        }
    }

    if (best_pos) {
        if (input->b_pmvr_enabled) {
            pmvr_adapt_mv1(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, *mv_x, *mv_y, best_pos);
            *mv_x = cand_mv_x;
            *mv_y = cand_mv_y;
        } else {
            *mv_x += spiral_search_x [best_pos];
            *mv_y += spiral_search_y [best_pos];
        }
    }

    //===== return minimum motion cost =====
    return min_mcost;
}

/*
*************************************************************************
* Function:Block motion search
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int BlockMotionSearch(int ref, int pic_pix_x, int pic_pix_y, int blocktype, int search_range, double lambda,
                      int mb_nr, int bit_size , int block)

{
    static pel_t  orig_pic  [MAX_CU_SIZE][MAX_CU_SIZE];

    int       pred_mv_x, pred_mv_y, mv_x, mv_y, i, j;
    int       max_value     = (1 << 20);
    int       min_mcost     = 0x7FFFFFFF;

    int       number_mb_per_row = img->width / MIN_CU_SIZE;
    int       mb_x = mb_nr % number_mb_per_row;
    int       mb_y = mb_nr / number_mb_per_row;
    int       SMBpix_x = mb_x * MIN_CU_SIZE;
    int       SMBpix_y = mb_y * MIN_CU_SIZE;
    int       mb_pix_x      = pic_pix_x - SMBpix_x;
    int       mb_pix_y      = pic_pix_y - SMBpix_y;

    int       b8_x          =  mb_pix_x == 0 ? 0 : 1;
    int       b8_y          =  mb_pix_y == 0 ? 0 : 1;


    int       bsx           = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int       bsy           = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);

    int       refframe      = (ref == -1 ? 0 : ref);
    int      *pred_mv;
    int     **ref_array     = ((img->type != B_IMG) ? hc->refFrArr : ref >= 0 ? img->fw_refFrArr : img->bw_refFrArr);
    int ***    mv_array      = ((img->type != B_IMG) ? img->tmp_mv   : ref >= 0 ? img->fw_mv    : img->bw_mv);
    int ** ***  allFwMv        = (ref < 0 ? img->allBwMv : img->allFwMv);
    byte    **imgY_org_pic  = he->imgY_org;

    int      *allIntegerPFwMv;
    int       refinx = ref + 1;

    int       incr_y = 1, off_y = 0; /*lgp*/
    int       center_x = pic_pix_x;/*lgp*/
    int       center_y = pic_pix_y;/*lgp*/
    pred_mv = ((img->type != B_IMG) ? img->mv  : ref >= 0 ? img->predBFwMv :
               img->predBBwMv) [b8_y][b8_x ][refframe][blocktype];


    allIntegerPFwMv = ((img->type != B_IMG) ? img->allIntegerPFwMv  : ref >= 0 ? img->allIntegerBFwMv :
                       img->allIntegerBBwMv) [b8_y][b8_x][refframe][blocktype];

    //==================================
    //=====   GET ORIGINAL BLOCK   =====
    //==================================
    for (j = 0; j < bsy; j++) {
        for (i = 0; i < bsx; i++) {
            orig_pic[j][i] = imgY_org_pic[pic_pix_y +/*j*/incr_y * j + off_y/*lgp*/][pic_pix_x + i];
        }
    }

    //===========================================
    //=====   GET MOTION VECTOR PREDICTOR   =====
    //===========================================
    SetMotionVectorPredictor(bit_size, mb_nr, pred_mv, ref_array, mv_array, refframe, mb_pix_x, mb_pix_y, bsx, bsy, ref,
                             0);  //Lou 1016
    pred_mv_x = pred_mv[0];
    pred_mv_y = pred_mv[1];


    //==================================
    //=====   INTEGER-PEL SEARCH   =====
    //==================================
    //--- set search center ---
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;

    //--- perform motion search ---

    min_mcost = FullPelBlockMotionSearch(orig_pic, ref, center_x/*lgp*/, center_y/*lgp*/, blocktype,
                                         pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                         min_mcost, lambda, bit_size, block);

    allIntegerPFwMv[0] = mv_x;
    allIntegerPFwMv[1] = mv_y;
    int_motion_cost[refinx] = min_mcost;


    if (!(img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) {

        //add by wuzhongmou 200612
        //==============================
        //=====   SUB-PEL SEARCH   =====
        //==============================
        if (input->hadamard) {
            min_mcost = 0x7FFFFFFF;
        }

        min_mcost =  SubPelBlockMotionSearch(orig_pic, ref, center_x/*lgp*/, center_y/*lgp*/, blocktype,
                                             pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                             min_mcost, lambda, bit_size, block);

    }


    //===============================================
    //=====   SET MV'S AND RETURN MOTION COST   =====
    //===============================================

    for (i = 0; i < ((bsx >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 2 ? 2 : 1); i++) {
        for (j = 0; j < ((bsy >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 2 ? 2 : 1); j++) {
            allFwMv[b8_y + j][b8_x + i][refframe][blocktype][0] = mv_x;
            allFwMv[b8_y + j][b8_x + i][refframe][blocktype][1] = mv_y;
        }
    }
    // xiaozhen zheng, mv_rang, 20071009
#if Mv_check_bug
	img->mv_range_flag = check_mv_range(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
	img->mv_range_flag = check_mv_range(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype);
#endif
    
    img->mv_range_flag *= check_mvd((mv_x - pred_mv_x), (mv_y - pred_mv_y));

    if (!img->mv_range_flag) {
#if RD160_FIX_BG
		min_mcost = 0x0FFFFFFF;
#else
        min_mcost = 0x7FFFFFFF;
#endif
        img->mv_range_flag = 1;
    }

    // xiaozhen zheng, mv_rang, 20071009

    return min_mcost;
}

/*
*************************************************************************
* Function:Block motion search
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


int BlockMotionSearch_sym(int ref, int pic_pix_x, int pic_pix_y, int blocktype, int search_range, double lambda,
                          int mb_nr, int bit_size, int block, int *mcost, int *mcost_bid)
{
    static pel_t  orig_pic  [MAX_CU_SIZE][MAX_CU_SIZE];

    int       pred_mv_x, pred_mv_y, mv_x, mv_y, i, j;

    int       pred_bmv_x_bid, pred_bmv_y_bid, pred_fmv_x_bid, pred_fmv_y_bid;
    int       bmv_x_bid, bmv_y_bid, fmv_x_bid, fmv_y_bid;

    int       max_value = (1 << 20);
    int       min_mcost = 0x7FFFFFFF;
    int       min_mcost_bid = 0x7FFFFFFF;

    int       number_mb_per_row = img->width / MIN_CU_SIZE;
    int       mb_x = mb_nr % number_mb_per_row;
    int       mb_y = mb_nr / number_mb_per_row;
    int       SMBpix_x = mb_x * MIN_CU_SIZE;
    int       SMBpix_y = mb_y * MIN_CU_SIZE;
    int       mb_pix_x      = pic_pix_x - SMBpix_x;
    int       mb_pix_y      = pic_pix_y - SMBpix_y;
    //sw 9.30

    int       b8_x          =  mb_pix_x == 0 ? 0 : 1;
    int       b8_y          =  mb_pix_y == 0 ? 0 : 1;

    int       bsx           = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int       bsy           = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);

    int       refframe  = (ref == -1 ? 0 : ref);
    int      *pred_mv;

    int      *pred_fmv_bid;
    int      *pred_bmv_bid;

    int     **ref_array = ((img->type != B_IMG) ? hc->refFrArr : ref >= 0 ? img->fw_refFrArr : img->bw_refFrArr);
    int ***    mv_array  = ((img->type != B_IMG) ? img->tmp_mv   : ref >= 0 ? img->fw_mv    : img->bw_mv);
    int ** ***  allFwMv    = (ref == -1 ? img->allBwMv : img->allSymMv);

    int ** ***  allBwMv   = img->allBwMv;
    int ** ***  allBidFwMv = img->allBidFwMv;
    int ** ***  allBidBwMv = img->allBidBwMv;


    int      *allIntegerPFwMv;

    byte    **imgY_org_pic = he->imgY_org;
    int       incr_y = 1, off_y = 0; /*lgp*/
    int       center_x = pic_pix_x;/*lgp*/
    int       center_y = pic_pix_y;/*lgp*/
    //sw 10.1


    pred_mv = ((img->type != B_IMG) ? img->mv  : ref >= 0 ? img->predSymMv   :
               img->predBBwMv) [b8_y][b8_x ][refframe][blocktype];

    allIntegerPFwMv = ((img->type != B_IMG) ? img->allIntegerPFwMv  : ref >= 0 ? img->allIntegerBFwMv :
                       img->allIntegerBBwMv) [b8_y][b8_x][refframe][blocktype];


    pred_fmv_bid = ((img->type != B_IMG) ? img->mv : img->predBidFwMv)[b8_y][b8_x ][refframe][blocktype];
    pred_bmv_bid = ((img->type != B_IMG) ? img->mv : img->predBidBwMv)[b8_y][b8_x ][refframe][blocktype];

    //==================================
    //=====   GET ORIGINAL BLOCK   =====
    //==================================
    for (j = 0; j < bsy; j++) {
        for (i = 0; i < bsx; i++) {
            orig_pic[j][i] = imgY_org_pic[pic_pix_y +/*j*/incr_y * j + off_y/*lgp*/][pic_pix_x + i];
        }
    }

    //===========================================
    //=====   GET MOTION VECTOR PREDICTOR   =====
    //===========================================

    pred_mv[0] =  img->predBFwMv[b8_y][b8_x][refframe][blocktype][0];
    pred_mv[1] =  img->predBFwMv[b8_y][b8_x][refframe][blocktype][1];

    pred_mv_x = pred_mv[0];
    pred_mv_y = pred_mv[1];



    pred_fmv_x_bid = pred_fmv_bid[0] = pred_mv_x;
    pred_fmv_y_bid = pred_fmv_bid[1] = pred_mv_y;



    pred_bmv_bid[0] = img->predBBwMv[b8_y][b8_x][refframe][blocktype][0];
    pred_bmv_bid[1] = img->predBBwMv[b8_y][b8_x][refframe][blocktype][1];

    pred_bmv_x_bid = pred_bmv_bid[0];
    pred_bmv_y_bid = pred_bmv_bid[1];



    //==================================
    //=====   INTEGER-PEL SEARCH   =====
    //==================================

    //--- set search center ---
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;

    bmv_x_bid = allBwMv[b8_y][b8_x][0][blocktype][0];
    bmv_y_bid = allBwMv[b8_y][b8_x][0][blocktype][1];
    fmv_x_bid = pred_fmv_x_bid / 4;
    fmv_y_bid = pred_fmv_y_bid / 4;

    //--- perform motion search ---


    mv_x = allIntegerPFwMv[0];
    mv_y = allIntegerPFwMv[1];
    min_mcost = int_motion_cost[1];



    fmv_x_bid = mv_x;
    fmv_y_bid = mv_y;
    min_mcost_bid = min_mcost;

    //==============================
    //=====   SUB-PEL SEARCH   =====
    //==============================
    if (input->hadamard) {
        min_mcost = 0x7FFFFFFF;

        min_mcost_bid = 0x7FFFFFFF;

    }

    min_mcost =  SubPelBlockMotionSearch_sym(orig_pic, ref, center_x/*lgp*/, center_y/*lgp*/, blocktype,
                 pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                 min_mcost, lambda, bit_size, block);
    min_mcost_bid =  SubPelBlockMotionSearch_bid(orig_pic, ref, center_x/*lgp*/, center_y/*lgp*/, blocktype,
                     pred_fmv_x_bid, pred_fmv_y_bid, &fmv_x_bid, &fmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid, &bmv_x_bid, &bmv_y_bid, 9, 9,
                     min_mcost, lambda, bit_size, block);



    //===============================================
    //=====   SET MV'S AND RETURN MOTION COST   =====
    //===============================================

    for (i = 0; i < ((bsx >> (3 + bit_size - MIN_BLOCK_SIZE_IN_BIT)) == 2 ? 2 : 1); i++) {
        for (j = 0; j < ((bsy >> (3 + bit_size - MIN_BLOCK_SIZE_IN_BIT)) == 2 ? 2 : 1); j++) {
            allFwMv[b8_y + j][b8_x + i][refframe][blocktype][0] = mv_x;
            allFwMv[b8_y + j][b8_x + i][refframe][blocktype][1] = mv_y;

            allBidFwMv[b8_y + j][b8_x + i][refframe][blocktype][0] = fmv_x_bid;
            allBidFwMv[b8_y + j][b8_x + i][refframe][blocktype][1] = fmv_y_bid;
            allBidBwMv[b8_y + j][b8_x + i][refframe][blocktype][0] = bmv_x_bid;
            allBidBwMv[b8_y + j][b8_x + i][refframe][blocktype][1] = bmv_y_bid;


        }
    }

#if Mv_check_bug
	img->mv_range_flag = check_mv_range(bit_size, fmv_x_bid, fmv_y_bid, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
	img->mv_range_flag = check_mv_range(bit_size, fmv_x_bid, fmv_y_bid, pic_pix_x, pic_pix_y, blocktype);
#endif
    
    img->mv_range_flag *= check_mvd((fmv_x_bid - pred_fmv_x_bid), (fmv_y_bid - pred_fmv_y_bid));
    if (!img->mv_range_flag) {
#if RD160_FIX_BG
		min_mcost_bid = 0x0FFFFFFF;
#else
        min_mcost_bid = 0x7FFFFFFF;
#endif
        img->mv_range_flag = 1;
    }
#if Mv_check_bug
	img->mv_range_flag = check_mv_range(bit_size, bmv_x_bid, bmv_y_bid, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
	img->mv_range_flag = check_mv_range(bit_size, bmv_x_bid, bmv_y_bid, pic_pix_x, pic_pix_y, blocktype);
#endif
    
    img->mv_range_flag *= check_mvd((bmv_x_bid - pred_bmv_x_bid), (bmv_y_bid - pred_bmv_y_bid));
    if (!img->mv_range_flag) {
#if RD160_FIX_BG
		min_mcost_bid = 0x0FFFFFFF;
#else
        min_mcost_bid = 0x7FFFFFFF;
#endif
        img->mv_range_flag = 1;
    }

    // xiaozhen zheng, mv_rang, 20071009
#if Mv_check_bug
	img->mv_range_flag = check_mv_range_sym(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype, ref, 0, 0);
#else
	img->mv_range_flag = check_mv_range_sym(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype, ref);
#endif
    
    img->mv_range_flag *= check_mvd((mv_x - pred_mv_x), (mv_y - pred_mv_y));

    if (!img->mv_range_flag) {
#if RD160_FIX_BG
		min_mcost = 0x0FFFFFFF;
#else
        min_mcost = 0x7FFFFFFF;
#endif
        img->mv_range_flag = 1;
    }

    // xiaozhen zheng, mv_rang, 20071009


    *mcost = min_mcost;
    *mcost_bid = min_mcost_bid;


    return min_mcost;
}

/*
*************************************************************************
* Function:Find motion vector for forward dual hypothesis prediction
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int BlockMotionSearch_dual(int ref, int pic_pix_x, int pic_pix_y, int blocktype, int search_range, double lambda,
                           int mb_nr, int bit_size, int block, int *mcost_dual)
{
    static pel_t  orig_pic  [MAX_CU_SIZE][MAX_CU_SIZE];

    int       i, j;

    int       snd_ref = ref == 0 ? 1 : 0;
    int       max_ref = img->num_of_references;
    int       DistanceIndexFw, DistanceIndexBw;
    int       pred_fst_x_dual, pred_fst_y_dual, pred_snd_x_dual, pred_snd_y_dual;
    int       fst_x_dual, fst_y_dual;

    int       max_value = (1 << 20);
    int       min_mcost_dual = 0x7FFFFFFF;

    int       number_mb_per_row = img->width / MIN_CU_SIZE;
    int       mb_x = mb_nr % number_mb_per_row;
    int       mb_y = mb_nr / number_mb_per_row;
    int       SMBpix_x = mb_x * MIN_CU_SIZE;
    int       SMBpix_y = mb_y * MIN_CU_SIZE;
    int       mb_pix_x      = pic_pix_x - SMBpix_x;
    int       mb_pix_y      = pic_pix_y - SMBpix_y;
    //sw 9.30

    int       b8_x          =  mb_pix_x == 0 ? 0 : 1;
    int       b8_y          =  mb_pix_y == 0 ? 0 : 1;

    int       bsx           = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int       bsy           = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);

    byte    **imgY_org_pic = he->imgY_org;
    int       incr_y = 1, off_y = 0; /*lgp*/
    int       center_x = pic_pix_x;/*lgp*/
    int       center_y = pic_pix_y;/*lgp*/
    //sw 10.1

    {
        //==================================
        //=====   GET ORIGINAL BLOCK   =====
        //==================================
        for (j = 0; j < bsy; j++) {
            for (i = 0; i < bsx; i++) {
                orig_pic[j][i] = imgY_org_pic[pic_pix_y +/*j*/incr_y * j + off_y/*lgp*/][pic_pix_x + i];
            }
        }

        //===========================================
        //=====   GET MOTION VECTOR PREDICTOR   =====
        //===========================================
        img->predDualFstMv [b8_y][b8_x][ref][blocktype][0] = pred_fst_x_dual = img->mv[b8_y][b8_x][ref][blocktype][0];
        img->predDualFstMv [b8_y][b8_x][ref][blocktype][1] = pred_fst_y_dual = img->mv[b8_y][b8_x][ref][blocktype][1];
        img->predDualSndMv [b8_y][b8_x][ref][blocktype][0] = pred_snd_x_dual = img->mv[b8_y][b8_x][snd_ref][blocktype][0];
        img->predDualSndMv [b8_y][b8_x][ref][blocktype][1] = pred_snd_y_dual = img->mv[b8_y][b8_x][snd_ref][blocktype][1];

        //==================================
        //=====   INTEGER-PEL SEARCH   =====
        //==================================

        //--- set search center ---
        fst_x_dual = img->allIntegerPFwMv[b8_y][b8_x][ref][blocktype][0];
        fst_y_dual = img->allIntegerPFwMv[b8_y][b8_x][ref][blocktype][1];
        min_mcost_dual = int_motion_cost[ref];

        //==============================
        //=====   SUB-PEL SEARCH   =====
        //==============================
        if (input->hadamard) {
            min_mcost_dual = 0x7FFFFFFF;
        }

        min_mcost_dual =  SubPelBlockMotionSearch_sym(orig_pic, ref, center_x, center_y, blocktype,
                          pred_fst_x_dual, pred_fst_y_dual, &fst_x_dual, &fst_y_dual, 9, 9, min_mcost_dual, lambda, bit_size, block);

        //===============================================
        //=====   SET MV'S AND RETURN MOTION COST   =====
        //===============================================
        {
            DistanceIndexFw = 2 * (hc->picture_distance - fref[ref]->imgtr_fwRefDistance);
            DistanceIndexFw = (DistanceIndexFw + 512) % 512;
            DistanceIndexBw = 2 * (hc->picture_distance - fref[snd_ref]->imgtr_fwRefDistance);
            DistanceIndexBw = (DistanceIndexBw + 512) % 512;
            if (ref == img->num_of_references - 1 && he->background_reference_enable) {
                DistanceIndexFw = 1;
            }
            if (snd_ref == img->num_of_references - 1 && he->background_reference_enable) {
                DistanceIndexBw = 1;
            }
            for (i = 0; i < ((bsx >> (3 + bit_size - MIN_BLOCK_SIZE_IN_BIT)) == 2 ? 2 : 1); i++) {
                for (j = 0; j < ((bsy >> (3 + bit_size - MIN_BLOCK_SIZE_IN_BIT)) == 2 ? 2 : 1); j++) {
                    img->allDualFstMv[b8_y + j][b8_x + i][ref][blocktype][0] = fst_x_dual;
                    img->allDualFstMv[b8_y + j][b8_x + i][ref][blocktype][1] = fst_y_dual;
#if MV_SCALE
                    img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][0] = scale_mv(fst_x_dual, DistanceIndexBw, DistanceIndexFw);
                    img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][1] = scale_mv(fst_y_dual, DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
                    if (img->is_field_sequence) {
                        img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][1] = scale_mv_y1(fst_y_dual, DistanceIndexBw, DistanceIndexFw);
                    }
#endif
#else
                    img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][0] = (fst_x_dual * DistanceIndexBw *
                            (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET ;
                    img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][1] = (fst_y_dual * DistanceIndexBw *
                            (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET ;
#if HALF_PIXEL_COMPENSATION_MVD
                    if (img->is_field_sequence) {
                        int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                        oriPOC = 2 * hc->picture_distance;
                        oriRefPOC = oriPOC - DistanceIndexFw;
                        scaledPOC = 2 * hc->picture_distance;
                        scaledRefPOC = scaledPOC - DistanceIndexBw;
                        getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                        img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][1] = (((fst_y_dual + delta) * DistanceIndexBw *
                                (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2 ;    //yb, ? +delta2
                    }
#endif
#endif
                }
            }

#if Mv_check_bug
			img->mv_range_flag = check_mv_range_sym(bit_size, fst_x_dual, fst_y_dual, pic_pix_x, pic_pix_y, blocktype, ref, 0, 0);
#else
			img->mv_range_flag = check_mv_range_sym(bit_size, fst_x_dual, fst_y_dual, pic_pix_x, pic_pix_y, blocktype, ref);
#endif
            
            img->mv_range_flag *= check_mvd((fst_x_dual - pred_fst_x_dual), (fst_y_dual - pred_fst_y_dual));
            if (!img->mv_range_flag) {
#if RD160_FIX_BG
				min_mcost_dual = 0x0FFFFFFF;
#else
                min_mcost_dual = 0x7FFFFFFF;
#endif
                img->mv_range_flag = 1;
            }
            *mcost_dual = min_mcost_dual;
        }
    }
    return *mcost_dual;
}

/*
*************************************************************************
* Function:Find motion vector for the Skip mode
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void FindSkipModeMotionVector(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int bx, by;
    int mb_nr = uiPositionInPic;
    int mb_width = img->width / MIN_CU_SIZE;
    int mb_x = mb_nr % mb_width;
    int mb_y = mb_nr / mb_width;
    int block8_x = mb_x << 1; //qyu 0830
    int block8_y = mb_y << 1;
    int **refar = hc->refFrArr;
    int ***tmpmv = img->tmp_mv;
    int ** ***allFwMv = img->allFwMv;
    int *mv  = img->mv[0][0][0][0];
    int mb_available_up;
    int mb_available_left;
    int zeroMotionAbove;
    int zeroMotionLeft;

    mb_available_up      = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           mb_width  ].slice_nr);       //jlzheng 6.23
    mb_available_left    = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           1         ].slice_nr);
    zeroMotionAbove   = !mb_available_up  ? 1 : hc->refFrArr[block8_y - 1][block8_x] == 0 &&
                        img->tmp_mv[block8_y - 1][block8_x][0]   == 0 && img->tmp_mv[block8_y - 1][block8_x][1]   == 0 ? 1 : 0;
    zeroMotionLeft    = !mb_available_left ? 1 : hc->refFrArr[block8_y][block8_x - 1] == 0 &&
                        img->tmp_mv[block8_y  ][block8_x - 1][0] == 0 && img->tmp_mv[block8_y  ][block8_x - 1][1] == 0 ? 1 : 0;

    if (zeroMotionAbove || zeroMotionLeft) {
        for (by = 0; by < 2; by++) {   //allFwMv[j][i][refindex][mode][k]
            for (bx = 0; bx < 2; bx++) {
                allFwMv [by][bx][0][0][0] = 0;
                allFwMv [by][bx][0][0][1] = 0;
            }
        }
    } else {
        SetMotionVectorPredictor(uiBitSize, uiPositionInPic, mv, refar, tmpmv, 0, 0, 0, (1 << uiBitSize),
                                 (1 << uiBitSize), 0, 0);    //qyu 0816

        for (by = 0; by < 2; by++) {   //qyu 0816
            for (bx = 0; bx < 2; bx++) {
                allFwMv [by][bx][0][0][0] = mv[0];
                allFwMv [by][bx][0][0][1] = mv[1];
            }
        }
    }
#if Mv_check_bug
	img->mv_range_flag = check_mv_range(uiBitSize, mv[0], mv[1], block8_x << MIN_BLOCK_SIZE_IN_BIT,
		block8_y << MIN_BLOCK_SIZE_IN_BIT, 1, 0, 0);
#else
	img->mv_range_flag = check_mv_range(uiBitSize, mv[0], mv[1], block8_x << MIN_BLOCK_SIZE_IN_BIT,
		block8_y << MIN_BLOCK_SIZE_IN_BIT, 1);
#endif
    

}

/*
*************************************************************************
* Function:Motion search for a partition
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void PartitionMotionSearch(int    blocktype, int    block8x8, double lambda, int mb_nr, int bit_size)
{

    static int  bx0[9][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 1}};//16x16 16x8 8x16 16x4 16x12 4x16 12x16 8x8
    static int  by0[9][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 1, 1}};


    int   ref, refinx, v, h, mcost, search_range;
    int   bframe    = (img->type == B_IMG);
    int   max_ref   = bframe ? img->num_of_references - 1 : img->num_of_references;

    int   parttype  = blocktype ;
    int   step_h, step_v;

    int   number_mb_per_row = img->width / MIN_CU_SIZE;
    int   mb_x = mb_nr % number_mb_per_row;
    int   mb_y = mb_nr / number_mb_per_row;
    int   SubMBpix_x = mb_x * MIN_CU_SIZE;
    int   SubMBpix_y = mb_y * MIN_CU_SIZE;
    int   min_ref   = (bframe ? -1 : 0);   /*lgp13*/

    int   PU_start_x;
    int   PU_start_y;
    v =   by0[parttype][block8x8];
    h =   bx0[parttype][block8x8];

    get_b8_offset(blocktype, bit_size, h, v, &PU_start_x, &PU_start_y, &step_h, &step_v);


    if (img->type == B_IMG) {
        max_ref = 1;  /*lgp*/
    }

    if (max_ref > img->buf_cycle) {
        max_ref = img->buf_cycle;
    }

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) { //only one reference frame(G\GB) for S frame
        max_ref = 1;
    }


    //===== LOOP OVER REFERENCE FRAMES =====
    for (ref = min_ref; ref < max_ref; ref++) {
        refinx    = ref + 1;



        //----- set search range ---
        search_range = input->search_range;

        //----- init motion cost -----
        motion_cost[blocktype][refinx][block8x8] = 0;

        //===== LOOP OVER BLOCKS =====

        //--- motion search for block ---

        PU_start_x = SubMBpix_x + ((h * g_blk_size[blocktype * 2][0]) << (bit_size - MIN_CU_SIZE_IN_BIT));
        PU_start_y = SubMBpix_y + ((v * g_blk_size[blocktype * 2][1]) << (bit_size - MIN_CU_SIZE_IN_BIT));
        if (input->usefme) {
            mcost = FME_BlockMotionSearch(ref, PU_start_x, PU_start_y, blocktype, search_range, lambda, mb_nr, bit_size,
                                          block8x8);
        } else {
            mcost = BlockMotionSearch(ref, PU_start_x, PU_start_y, blocktype, search_range, lambda, mb_nr , bit_size, block8x8);
        }


        motion_cost[blocktype][refinx][block8x8] += mcost;

        //--- set motion vectors and reference frame (for motion vector prediction) ---
    }
}


void ForwardPred(int *fw_mcost, int *best_fw_ref, int *best_bw_ref, int max_ref, int adjust_ref, int mode, int block,
                 double lambda_motion, int write_ref, int lambda_motion_factor, int mb_nr, int bit_size)
{
    int ref;
    int mcost;
    int max_mcost = 1 << 30;
    PartitionMotionSearch(mode, block, lambda_motion, mb_nr, bit_size);

    //--- get cost and reference frame for forward prediction ---
    if ((img->type == F_IMG) || (img->type == P_IMG)) {
        for (*fw_mcost = max_mcost, ref = 0; ref < max_ref - adjust_ref; ref++) {
            mcost  = (input->rdopt ? write_ref ? REF_COST_FWD(lambda_motion_factor,
                      ref) : 0 : (int)(2 * lambda_motion * min(ref, 1)));
            mcost += motion_cost[mode][ref + 1][block];

            if (mcost < *fw_mcost) {
                *fw_mcost    = mcost;
                *best_fw_ref = ref;
            }
        }

        *best_bw_ref = 0;//xyji
    } else {
        for (*fw_mcost = max_mcost, ref = 0; ref < max_ref - adjust_ref; ref++) {
            mcost = motion_cost[mode][ref + 1][block];
            if (mcost < *fw_mcost) {
                *fw_mcost    = mcost;
                *best_fw_ref = ref;
            }
        }

        *best_bw_ref = 0;
    }
}


void BiPred(int *best_bw_ref, int *bw_mcost, int *sym_mcost, int *sym_best_fw_ref, int *sym_best_bw_ref,
            int *bid_mcost, int *bid_best_fw_ref, int *bid_best_bw_ref, int mode, int block, double lambda_motion, int max_ref,
            int adjust_ref , int mb_nr, int bit_size)

{
    int max_mcost = 1 << 30;
    int mcost;
    int ref;
    //--- get cost for symirectional prediction ---

    PartitionMotionSearch_sym(mode, block, lambda_motion, mb_nr,  bit_size);

    *best_bw_ref = 0;
    *bw_mcost   = motion_cost[mode][0][block];


    for (*bw_mcost = max_mcost, *sym_mcost = max_mcost, *bid_mcost = max_mcost, ref = 0; ref < max_ref - adjust_ref;
         ref++)

    {
        mcost = motion_cost[mode][ref][block];

        if (mcost < *bw_mcost) {
            *bw_mcost    = mcost;
            *best_bw_ref = ref;
        }
        mcost = motion_cost_sym[mode][ref + 1][block];

        if (mcost < *sym_mcost) {
            *sym_mcost    = mcost;
            *sym_best_fw_ref = ref;
            *sym_best_bw_ref = ref;
        }

        mcost = motion_cost_sym[mode][ref][block];

        if (mcost < *bid_mcost) {
            *bid_mcost = mcost;
            *bid_best_fw_ref = ref;
            *bid_best_bw_ref = ref;
        }

    }
}


void BackwardPred(int *best_bw_ref, int *bw_mcost, int mode, int block, double lambda_motion, int max_ref,
                  int adjust_ref , int mb_nr, int bit_size)

{
    int max_mcost = 1 << 30;
    int mcost;
    int ref;


    // *best_bw_ref = 0;
    // *bw_mcost   = motion_cost[mode][0][block];


    for (*bw_mcost = max_mcost, ref = 0; ref < max_ref - adjust_ref; ref++)

    {
        // cost has been calculated in ForwardPred()
        mcost = motion_cost[mode][ref][block];

        if (mcost < *bw_mcost) {
            *bw_mcost    = mcost;
            *best_bw_ref = ref;
        }
    }
}


void DualPred(int *dual_mcost, int *dual_best_fst_ref, int *dual_best_snd_ref, int max_ref, int adjust_ref, int mode,
              int block, double lambda_motion, int write_ref, int lambda_motion_factor, int mb_nr, int bit_size)
{
    int ref;

    int mcost;
    int max_mcost = 1 << 30;

    //--- get cost for dual hypothesis prediction ---
    PartitionMotionSearch_dual(mode, block, lambda_motion, mb_nr, bit_size);

    //--- get cost and reference frame for dual prediction ---
    for (*dual_mcost = max_mcost, ref = 0; ref < max_ref - adjust_ref; ref++) {
        if (motion_cost_dual[mode][ref][block] == 0x7FFFFFFF) {
            continue;
        }
        mcost  = (input->rdopt ? write_ref ? REF_COST_FWD(lambda_motion_factor,
                  ref) : 0 : (int)(2 * lambda_motion * min(ref, 1)));
        mcost += motion_cost_dual[mode][ref][block];

        if (mcost < *dual_mcost) {
            *dual_mcost    = mcost;
            *dual_best_fst_ref = ref;
            *dual_best_snd_ref = ref == 0 ? 1 : 0;
        }
    }
}

/*
*************************************************************************
* Function:Motion search for a partition
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void PartitionMotionSearch_sym(int    blocktype, int    block8x8, double lambda, int mb_nr, int bit_size)
{

    static int  bx0[9][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 1}};//16x16 16x8 8x16 16x4 16x12 4x16 12x16 8x8
    static int  by0[9][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 1, 1}};

    int   ref, refinx, v, h, mcost, search_range;

    int mcost_bid;

    int   bframe    = (img->type == B_IMG);
    int   max_ref   = bframe ? img->num_of_references - 1 : img->num_of_references;

    int   parttype  = blocktype ;
    int   step_h    = (g_blk_size[blocktype * 2][0]   << (bit_size - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT;
    int   step_v    = (g_blk_size[blocktype * 2][1]   << (bit_size - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT;

    int   number_mb_per_row = img->width / MIN_CU_SIZE;
    int   mb_x = mb_nr % number_mb_per_row;
    int   mb_y = mb_nr / number_mb_per_row;
    int   SubMBpix_x = mb_x * MIN_CU_SIZE;
    int   SubMBpix_y = mb_y * MIN_CU_SIZE;
    int   PU_start_x;
    int   PU_start_y;
    int   PU_size_x = (g_blk_size[blocktype * 2 + block8x8][0] << (bit_size - MIN_CU_SIZE_IN_BIT)) >>
                      MIN_BLOCK_SIZE_IN_BIT;
    int   PU_size_y = (g_blk_size[blocktype * 2 + block8x8][1] << (bit_size - MIN_CU_SIZE_IN_BIT)) >>
                      MIN_BLOCK_SIZE_IN_BIT;
    PU_size_x = (PU_size_x == 0) ? 1 : PU_size_x;
    PU_size_y = (PU_size_y == 0) ? 1 : PU_size_y;
    step_h = step_h == 0 ? 1 : step_h;
    step_v = step_v == 0 ? 1 : step_v;


    if (img->type == B_IMG) {
        max_ref = 1;
    }

    if (max_ref > img->buf_cycle) {
        max_ref = img->buf_cycle;
    }



    //===== LOOP OVER REFERENCE FRAMES =====
    for (ref = 0; ref < max_ref; ref++) {
        refinx    = ref + 1;


        search_range = input->search_range;

        //----- init motion cost -----
        motion_cost_sym[blocktype][refinx][block8x8] = 0;

        motion_cost_sym[blocktype][ref][block8x8] = 0;


        //===== LOOP OVER BLOCKS =====
        v = by0[parttype][block8x8];
        h = bx0[parttype][block8x8];

        //--- motion search for block ---

        PU_start_x = SubMBpix_x + ((h * g_blk_size[blocktype * 2][0]) << (bit_size - MIN_CU_SIZE_IN_BIT));
        PU_start_y = SubMBpix_y + ((v * g_blk_size[blocktype * 2][1]) << (bit_size - MIN_CU_SIZE_IN_BIT));
        if (input->usefme) {

            FME_BlockMotionSearch_sym(ref, PU_start_x, PU_start_y, blocktype, search_range, lambda, mb_nr , bit_size, block8x8,
                                      &mcost, &mcost_bid);

        } else {

            BlockMotionSearch_sym(ref, PU_start_x, PU_start_y, blocktype, search_range, lambda, mb_nr , bit_size, block8x8,
                                  &mcost, &mcost_bid);

        }


        motion_cost_sym[blocktype][refinx][block8x8] += mcost;

        motion_cost_sym[blocktype][ref][block8x8] += mcost_bid;


        //--- set motion vectors and reference frame (for motion vector prediction) ---

    }
}

/*
*************************************************************************
* Function:Motion search for a partition
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void PartitionMotionSearch_dual(int    blocktype, int    block8x8, double lambda, int mb_nr, int bit_size)
{

    static int  bx0[9][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 1}};//16x16 16x8 8x16 16x4 16x12 4x16 12x16 8x8
    static int  by0[9][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 1, 1}};


    int   ref, v, h, search_range;
    int   mcost_dual;
    int   max_value = 0x7FFFFFFF;
    int   max_ref   = img->num_of_references;

    int   parttype  = blocktype ;
    int   step_h    = (g_blk_size[blocktype * 2][0]   << (bit_size - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT;
    int   step_v    = (g_blk_size[blocktype * 2][1]   << (bit_size - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT;

    int   number_mb_per_row = img->width / MIN_CU_SIZE;
    int   mb_x = mb_nr % number_mb_per_row;
    int   mb_y = mb_nr / number_mb_per_row;
    int   SubMBpix_x = mb_x * MIN_CU_SIZE;
    int   SubMBpix_y = mb_y * MIN_CU_SIZE;

    int   PU_start_x;
    int   PU_start_y;
    int   PU_size_x = (g_blk_size[blocktype * 2 + block8x8][0] << (bit_size - MIN_CU_SIZE_IN_BIT)) >>
                      MIN_BLOCK_SIZE_IN_BIT;
    int   PU_size_y = (g_blk_size[blocktype * 2 + block8x8][1] << (bit_size - MIN_CU_SIZE_IN_BIT)) >>
                      MIN_BLOCK_SIZE_IN_BIT;
    PU_size_x = (PU_size_x == 0) ? 1 : PU_size_x;
    PU_size_y = (PU_size_y == 0) ? 1 : PU_size_y;
    step_h = step_h == 0 ? 1 : step_h;
    step_v = step_v == 0 ? 1 : step_v;


    if (max_ref > img->buf_cycle) {
        max_ref = img->buf_cycle;
    }

    //===== LOOP OVER REFERENCE FRAMES =====
    for (ref = 0; ref < max_ref; ref++) {

        search_range = input->search_range;

        //----- init motion cost -----
        motion_cost_dual[blocktype][ref][block8x8] = 0;
        mcost_dual = max_value;

        //===== LOOP OVER BLOCKS =====
        v = by0[parttype][block8x8];
        h = bx0[parttype][block8x8];

        //--- motion search for block ---
        PU_start_x = SubMBpix_x + ((h * g_blk_size[blocktype * 2][0]) << (bit_size - MIN_CU_SIZE_IN_BIT));
        PU_start_y = SubMBpix_y + ((v * g_blk_size[blocktype * 2][1]) << (bit_size - MIN_CU_SIZE_IN_BIT));

        if (input->usefme) {
            FME_BlockMotionSearch_dual(ref, PU_start_x, PU_start_y, blocktype, search_range, lambda, mb_nr , bit_size, block8x8,
                                       &mcost_dual);
        } else {
            BlockMotionSearch_dual(ref, PU_start_x, PU_start_y, blocktype, search_range, lambda, mb_nr , bit_size, block8x8,
                                   &mcost_dual);
        }

        motion_cost_dual[blocktype][ref][block8x8] += mcost_dual;



        //--- set motion vectors and reference frame (for motion vector prediction) ---
    }
}
/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void mvd_prediction(int *mvd_U, int *mvd_L, int *mvd_C, int available_U, int available_L, int available_C, int *pmvd)
{
    int abs_U, abs_L, abs_C;
    abs_U = abs(mvd_U[0]) + abs(mvd_U[1]);
    abs_L = abs(mvd_L[0]) + abs(mvd_L[1]);
    abs_C = abs(mvd_C[0]) + abs(mvd_C[1]);

    if (!abs_L || !abs_U) {
        pmvd[0] = pmvd[1] = 0;
    } else {
        pmvd[0] = MEDIAN(mvd_U[0], mvd_L[0], mvd_C[0]);
        pmvd[1] = MEDIAN(mvd_U[1], mvd_L[1], mvd_C[1]);
    }

}

void PskipMV_COL(int uiBitSize, unsigned int uiPositionInPic, int block8_in_row, int block8_in_col,
                 int blockshape_pix_x, int blockshape_pix_y)
{
    //  int bx, by;
    int mb_nr = uiPositionInPic;
    int mb_width = img->width / MIN_CU_SIZE;
    int mb_x = mb_nr % mb_width;
    int mb_y = mb_nr / mb_width;
    int pic_block8_x = mb_x << (MIN_CU_SIZE_IN_BIT - MIN_BLOCK_SIZE_IN_BIT); //qyu 0830
    int pic_block8_y = mb_y << (MIN_CU_SIZE_IN_BIT - MIN_BLOCK_SIZE_IN_BIT);
    int pic_pix_x = mb_x << (MIN_CU_SIZE_IN_BIT);
    int pic_pix_y = mb_y << (MIN_CU_SIZE_IN_BIT);
    int blockshape_block_x = blockshape_pix_x >> MIN_BLOCK_SIZE_IN_BIT;
    int blockshape_block_y = blockshape_pix_y >> MIN_BLOCK_SIZE_IN_BIT;
    int blockshape_mb_x = blockshape_pix_x >> MIN_CU_SIZE_IN_BIT;
    int blockshape_mb_y = blockshape_pix_y >> MIN_CU_SIZE_IN_BIT;

    int **cur_ref = hc->refFrArr;

    int **col_ref = fref[0]->refbuf;
    int ***cur_mv = img->tmp_mv;
    int ***col_mv = fref[0]->mvbuf;
    int *col_pic_dist = fref[0]->ref_poc;
    int ** ***allFwMv = img->allFwMv;
    int refframe;
    int curT, colT;
    int i, j;
#if HALF_PIXEL_PSKIP
    int delta, delta2;
    delta = delta2 = 0;
#endif


    img->mv_range_flag = 1;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            int block_x = pic_block8_x + blockshape_block_x / 2 * i ;
            int block_y = pic_block8_y + blockshape_block_y / 2 * j ;
            refframe = col_ref[block_y ][block_x];

            if (refframe >= 0)

            {
                curT = (2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance) + 512) % 512;
                colT = (2 * (fref[0]->imgtr_fwRefDistance - col_pic_dist[refframe]) + 512) % 512;
                if (0 == img->num_of_references - 1 && he->background_reference_enable) {
                    curT = 1;
                    colT = 1;
                }
                if (refframe == img->num_of_references - 1 && he->background_reference_enable) {
                    colT = 1;
                }
#if HALF_PIXEL_PSKIP
                if (img->is_field_sequence) {
                    int oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                    oriPOC = 2 * fref[0]->imgtr_fwRefDistance;
                    oriRefPOC = oriPOC - colT;
                    scaledPOC = 2 * hc->picture_distance;
                    scaledRefPOC = 2 * fref[0]->imgtr_fwRefDistance;
                    getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                }

                scalingMV(&allFwMv [j][i][0][0][0], &allFwMv [j][i][0][0][1], curT, col_mv[block_y][block_x][0],
                          col_mv[block_y][block_x][1] + delta, colT, 1);
                allFwMv [j][i][0][0][1] -= delta2;
#else
                scalingMV(&allFwMv [j][i][0][0][0], &allFwMv [j][i][0][0][1], curT, col_mv[block_y][block_x][0],
                          col_mv[block_y][block_x][1], colT, 1);
#endif
                img->mv[j][i][0][0][0] = allFwMv [j][i][0][0][0] ;
                img->mv[j][i][0][0][1] = allFwMv [j][i][0][0][1] ;
                allFwMv [j][i][0][0][0] = Clip3(-32768, 32767, img->mv[j][i][0][0][0]) ;
                allFwMv [j][i][0][0][1] = Clip3(-32768, 32767, img->mv[j][i][0][0][1]) ;
            } else {
                img->mv[j][i][0][0][0] = 0;
                img->mv[j][i][0][0][1] = 0;

                allFwMv [j][i][0][0][0] = img->mv[j][i][0][0][0] ;
                allFwMv [j][i][0][0][1] = img->mv[j][i][0][0][1] ;
            }
#if Mv_check_bug
			img->mv_range_flag *= check_mv_range(uiBitSize, img->mv[j][i][0][0][0], img->mv[j][i][0][0][1],
				(pic_block8_x + blockshape_block_x / 2 * i) << MIN_BLOCK_SIZE_IN_BIT,
				(pic_block8_y + blockshape_block_y / 2 * j) << MIN_BLOCK_SIZE_IN_BIT, 1, 0, 0);
#else
			img->mv_range_flag *= check_mv_range(uiBitSize, img->mv[j][i][0][0][0], img->mv[j][i][0][0][1],
				(pic_block8_x + blockshape_block_x / 2 * i) << MIN_BLOCK_SIZE_IN_BIT,
				(pic_block8_y + blockshape_block_y / 2 * j) << MIN_BLOCK_SIZE_IN_BIT, 1);
#endif
        }
    }
    if (uiBitSize == MIN_CU_SIZE_IN_BIT) {
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 2; j++) {
                img->mv[j][i][0][0][0] = allFwMv [j][i][0][0][0] = allFwMv [0][0][0][0][0];
                img->mv[j][i][0][0][1] = allFwMv [j][i][0][0][1] = allFwMv [0][0][0][0][1];
            }
        }
    }

}

void FindSkipModeMotionVectorInCCD(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int mb_nr = uiPositionInPic;
    int mb_width = img->width / MIN_CU_SIZE;
    int mb_x = mb_nr % mb_width;
    int mb_y = mb_nr / mb_width;
    int block8_x = mb_x << 1; //qyu 0830
    int block8_y = mb_y << 1;
    int **refar = hc->refFrArr;
    int ***tmpmv = img->tmp_mv;
    int ** ***allFwMv = img->allFwMv;
    int *mv  = img->mv[0][0][0][0];
    int N8_SizeScale = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    int num_of_orgMB_in_row = N8_SizeScale;//4:1  5:2  6:4
    int num_of_orgMB_in_col = N8_SizeScale;
    int pusize = 1 << uiBitSize;
    int pix_x = mb_x << MIN_CU_SIZE_IN_BIT;
    int pix_y = mb_y << MIN_CU_SIZE_IN_BIT;
    if (pix_x + pusize >= img->width) {
        num_of_orgMB_in_row = min(N8_SizeScale, (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT);
    }

    if (pix_y + pusize >= img->height) {
        num_of_orgMB_in_col = min(N8_SizeScale, (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT);
    }
    PskipMV_COL(uiBitSize,  uiPositionInPic, num_of_orgMB_in_row, num_of_orgMB_in_col,
                num_of_orgMB_in_row << MIN_CU_SIZE_IN_BIT, num_of_orgMB_in_col << MIN_CU_SIZE_IN_BIT);

    setPSkipMotionVector(uiBitSize, uiPositionInPic, 0, 0, 0, (1 << uiBitSize), (1 << uiBitSize), 1);

}

double GetcurCanPositionCostBid_NoMV(pel_t orig_pic[64][64], int ref_f, int ref_s, int pic_pix_x, int pic_pix_y,
                                     int blocksize_x, int blocksize_y, int blocktype,  int *mv_f, int *mv_s,  double lambda)
{
    int   diff[64], *d;
    double    mcost;
    int   y0, x0, ry0, rx0, ry1, rx1;
    int   cand_mv_f_x, cand_mv_f_y;
    int   cand_mv_s_x, cand_mv_s_y;
    pel_t *orig_line;
    int   incr            = 0;//qyu 0926
    pel_t **ref_pic_f;
    pel_t **ref_pic_s;
    //int   lambda_factor   = LAMBDA_FACTOR ( lambda );
    int   mv_shift        = 0;

    int   pic4_pix_x      = (pic_pix_x << 2);
    int   pic4_pix_y      = (pic_pix_y << 2);
    int   max_pos_x4      = ((img->width - blocksize_x) << 2);
    int   max_pos_y4      = ((img->height - blocksize_y) << 2);
    int   curr_diff[64][64]; // for AVS 8x8 SATD calculation
    int   xx, yy, kk;                              // indicees for curr_diff
    pel_t (*PelY_f_14)(pel_t **, int, int);
    pel_t (*PelY_s_14)(pel_t **, int, int);
    // int  mv_x = predMv_set->pmv_cand[index][0];
    //int  mv_y = predMv_set->pmv_cand[index][1];
    int  distortion;
    if (img->type == B_IMG) {
        incr = 1;
    }
    assert(img->type == P_IMG || img->type == F_IMG);

    ref_pic_f = img->type == B_IMG ? fref[ref_f + incr]->oneForthRefY  : fref[ref_f]->oneForthRefY;
    ref_pic_s = img->type == B_IMG ? fref[ref_s + incr]->oneForthRefY : fref[ref_s]->oneForthRefY;


    //////////////////////////////////////////////////////////////////


    //===== set function for getting pixel values =====



    if ((pic4_pix_x + mv_f[0] > 0) && (pic4_pix_x + mv_f[0] < max_pos_x4) &&
        (pic4_pix_y + mv_f[1] > 0) && (pic4_pix_y + mv_f[1] < max_pos_y4)) {
        PelY_f_14 = FastPelY_14;
    } else {
        PelY_f_14 = UMVPelY_14;
    }
    if ((pic4_pix_x + mv_s[0] > 0) && (pic4_pix_x + mv_s[0] < max_pos_x4) &&
        (pic4_pix_y + mv_s[1] > 0) && (pic4_pix_y + mv_s[1] < max_pos_y4)) {
        PelY_s_14 = FastPelY_14;
    } else {
        PelY_s_14 = UMVPelY_14;
    }

    //===== loop over search positions =====

    cand_mv_f_x = mv_f[0] ;    // quarter-pel units
    cand_mv_f_y = mv_f[1] ;    // quarter-pel units

    cand_mv_s_x = mv_s[0];
    cand_mv_s_y = mv_s[1];

    //----- set motion vector cost -----
//  mcost = MV_COST ( lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y );

    // mcost = lambda * mvpidxbits[index][predMv_set->pmv_num];
    mcost = 0;
    distortion = 0;
    //----- add up SATD -----
    for (y0 = 0;  y0 < blocksize_y ; y0 += 4) {
        ry0 = ((pic_pix_y + y0) << 2) + cand_mv_f_y;
        ry1 = ((pic_pix_y + y0) << 2) + cand_mv_s_y;

        for (x0 = 0; x0 < blocksize_x; x0 += 4) {
            rx0 = ((pic_pix_x + x0) << 2) + cand_mv_f_x;
            rx1 = ((pic_pix_x + x0) << 2) + cand_mv_s_x;
            d   = diff;

            orig_line = orig_pic [y0  ];
            //ry = ry0;
            *d++      = orig_line[x0    ]  - ((PelY_f_14(ref_pic_f, ry0, rx0) +     PelY_s_14(ref_pic_s, ry1, rx1) + 1) >> 1);
            *d++      = orig_line[x0 + 1]  - ((PelY_f_14(ref_pic_f, ry0, rx0 + 4) + PelY_s_14(ref_pic_s, ry1,
                                               rx1 + 4) + 1) >> 1);
            *d++      = orig_line[x0 + 2]  - ((PelY_f_14(ref_pic_f, ry0, rx0 + 8) + PelY_s_14(ref_pic_s, ry1,
                                               rx1 + 8) + 1) >> 1);
            *d++      = orig_line[x0 + 3]  - ((PelY_f_14(ref_pic_f, ry0, rx0 + 12) + PelY_s_14(ref_pic_s, ry1,
                                               rx1 + 12) + 1) >> 1);

            orig_line = orig_pic [y0 + 1];
            //ry = ry0 + 4;
            *d++      = orig_line[x0    ]  - ((PelY_f_14(ref_pic_f, ry0 + 4, rx0) +     PelY_s_14(ref_pic_s, ry1 + 4,
                                               rx1) + 1) >> 1);
            *d++      = orig_line[x0 + 1]  - ((PelY_f_14(ref_pic_f, ry0 + 4, rx0 + 4) + PelY_s_14(ref_pic_s, ry1 + 4,
                                               rx1 + 4) + 1) >> 1);
            *d++      = orig_line[x0 + 2]  - ((PelY_f_14(ref_pic_f, ry0 + 4, rx0 + 8) + PelY_s_14(ref_pic_s, ry1 + 4,
                                               rx1 + 8) + 1) >> 1);
            *d++      = orig_line[x0 + 3]  - ((PelY_f_14(ref_pic_f, ry0 + 4, rx0 + 12) + PelY_s_14(ref_pic_s, ry1 + 4,
                                               rx1 + 12) + 1) >> 1);

            orig_line = orig_pic [y0 + 2];
            //ry = ry0 + 8;
            *d++      = orig_line[x0    ]  - ((PelY_f_14(ref_pic_f, ry0 + 8, rx0) +     PelY_s_14(ref_pic_s, ry1 + 8,
                                               rx1) + 1) >> 1);
            *d++      = orig_line[x0 + 1]  - ((PelY_f_14(ref_pic_f, ry0 + 8, rx0 + 4) + PelY_s_14(ref_pic_s, ry1 + 8,
                                               rx1 + 4) + 1) >> 1);
            *d++      = orig_line[x0 + 2]  - ((PelY_f_14(ref_pic_f, ry0 + 8, rx0 + 8) + PelY_s_14(ref_pic_s, ry1 + 8,
                                               rx1 + 8) + 1) >> 1);
            *d++      = orig_line[x0 + 3]  - ((PelY_f_14(ref_pic_f, ry0 + 8, rx0 + 12) + PelY_s_14(ref_pic_s, ry1 + 8,
                                               rx1 + 12) + 1) >> 1);

            orig_line = orig_pic [y0 + 3];
            //ry = ry0 + 12;
            *d++      = orig_line[x0    ]  - ((PelY_f_14(ref_pic_f, ry0 + 12, rx0) +     PelY_s_14(ref_pic_s, ry1 + 12,
                                               rx1) + 1) >> 1);
            *d++      = orig_line[x0 + 1]  - ((PelY_f_14(ref_pic_f, ry0 + 12, rx0 + 4) + PelY_s_14(ref_pic_s, ry1 + 12,
                                               rx1 + 4) + 1) >> 1);
            *d++      = orig_line[x0 + 2]  - ((PelY_f_14(ref_pic_f, ry0 + 12, rx0 + 8) + PelY_s_14(ref_pic_s, ry1 + 12,
                                               rx1 + 8) + 1) >> 1);
            *d        = orig_line[x0 + 3]  - ((PelY_f_14(ref_pic_f, ry0 + 12, rx0 + 12) + PelY_s_14(ref_pic_s, ry1 + 12,
                                               rx1 + 12) + 1) >> 1);

            for (yy = y0, kk = 0; yy < y0 + 4; yy++) {
                for (xx = x0; xx < x0 + 4; xx++, kk++) {
                    curr_diff[yy][xx] = diff[kk];

                }
            }
        }
    }

    mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);


    //mcost += distortion;

    return mcost;
}

/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void  Get_direct(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int  block_x, block_y;
    int  pix_x = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int  pix_y = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;
    int  pic_block_y ;
    int  pic_block_x ;
    int num_mb_inblock = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);   //qyu 0820 add 4:1 5:2 6:4

    int  refframe, bw_ref, TRb, TRp, TRd, TRp1;   // 20071009
    int  frame_no_next_P, frame_no_B;  // 20071009
#if FIX_MAX_REF
    int  delta_P[MAXREF];
#else
    int  delta_P[4];
#endif
    int i;
    int  **refarr         = fref[0]->refbuf;
    int  ***tmpmvs        = fref[0]->mvbuf;
    int *****allFwMvs     = img->allFwMv;
    int *****allBwMvs     = img->allBwMv;
    int  scale_refframe;
    int  **fwrefarr         = img->fw_refFrArr;
    int  **bwrefarr         = img->bw_refFrArr;
    int  ***tmpmvfw         = img->fw_mv;
    int  ***tmpmvbw         = img->bw_mv;
    int j;


    for (block_y = 0; block_y < 2; block_y++) {
        pic_block_y = (pix_y >> MIN_BLOCK_SIZE_IN_BIT) + block_y * num_mb_inblock;

        for (block_x = 0; block_x < 2; block_x++) {
            pic_block_x = (pix_x >> MIN_BLOCK_SIZE_IN_BIT) + block_x * num_mb_inblock;

            refframe = refarr[pic_block_y][pic_block_x];

            if (refframe == -1) {
                allFwMvs[block_y][block_x][0][0][0] = 0;
                allFwMvs[block_y][block_x][0][0][1] = 0;
                allBwMvs[block_y][block_x][0][0][0] = 0;
                allBwMvs[block_y][block_x][0][0][1] = 0;
                SetMotionVectorPredictor(uiBitSize, uiPositionInPic, allFwMvs[block_y][block_x][0][0], fwrefarr, tmpmvfw, 0, 0, 0,
                                         (1 << uiBitSize), (1 << uiBitSize), 0, 1);      //Lou 1016
                SetMotionVectorPredictor(uiBitSize, uiPositionInPic, allBwMvs[block_y][block_x][0][0], bwrefarr, tmpmvbw, 0, 0, 0,
                                         (1 << uiBitSize), (1 << uiBitSize), -1, 1);      //qyu 0820
#if Mv_check_bug
				img->mv_range_flag = check_mv_range(uiBitSize, allFwMvs[block_y][block_x][0][0][0],
					allFwMvs[block_y][block_x][0][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0, 0, 0);
				img->mv_range_flag *= check_mv_range(uiBitSize, allBwMvs[block_y][block_x][0][0][0],
					allBwMvs[block_y][block_x][0][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0, 0, 0);
#else
				img->mv_range_flag = check_mv_range(uiBitSize, allFwMvs[block_y][block_x][0][0][0],
					allFwMvs[block_y][block_x][0][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0);
				img->mv_range_flag *= check_mv_range(uiBitSize, allBwMvs[block_y][block_x][0][0][0],
					allBwMvs[block_y][block_x][0][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0);
#endif
                
            } else {
                frame_no_next_P = 2 * img->imgtr_next_P;
                frame_no_B = 2 * hc->picture_distance;
#if FIX_MAX_REF
                for (i = 0; i < MAXREF; i++) {
#else
                for (i = 0; i < 4; i++) {
#endif
                    delta_P[i] = 2 * (img->imgtr_next_P - fref[0]->ref_poc[i]);
                    delta_P[i] = (delta_P[i] + 512) % 512;
                }

                scale_refframe = 0;
                TRp = delta_P[refframe];
                TRp1 = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);


                TRd = frame_no_next_P - frame_no_B;

                TRb = TRp1 - TRd;
                TRp  = (TRp + 512) % 512;
                TRp1 = (TRp1 + 512) % 512;
                TRd  = (TRd + 512) % 512;
                TRb  = (TRb + 512) % 512;

                refframe = 0;
                bw_ref = 0;

                {
#if MV_SCALE
                    scale_mv_direct_x(tmpmvs[pic_block_y][pic_block_x][0], TRp, TRb, TRd, &allFwMvs[block_y][block_x][refframe][0][0],
                                      &allBwMvs[block_y][block_x][bw_ref][0][0]);
                    scale_mv_direct_y(tmpmvs[pic_block_y][pic_block_x][1], frame_no_next_P, TRp, frame_no_B, TRb, TRd,
                                      &allFwMvs[block_y][block_x][refframe][0][1], &allBwMvs[block_y][block_x][bw_ref][0][1]);
#else
                    if (tmpmvs[pic_block_y][pic_block_x][0] < 0) {
                        allFwMvs[block_y][block_x][refframe][0][0] = -(((MULTI / TRp) * (1 - TRb * tmpmvs[pic_block_y][pic_block_x][0])
                                - 1) >> OFFSET);
                        allBwMvs[block_y][block_x][bw_ref][0][0] = ((MULTI / TRp) * (1 - TRd * tmpmvs[pic_block_y][pic_block_x][0]) - 1)
                                >> OFFSET;
                    } else {
                        allFwMvs[block_y][block_x][refframe][0][0] = ((MULTI / TRp) * (1 + TRb * tmpmvs[pic_block_y][pic_block_x][0]) -
                                1) >> OFFSET;
                        allBwMvs[block_y][block_x][bw_ref][0][0] = -(((MULTI / TRp) * (1 + TRd * tmpmvs[pic_block_y][pic_block_x][0]) -
                                1) >> OFFSET);
                    }

                    if (tmpmvs[pic_block_y][pic_block_x][1] < 0) {
                        allFwMvs[block_y][block_x][refframe][0][1] = -(((MULTI / TRp) * (1 - TRb * tmpmvs[pic_block_y][pic_block_x][1])
                                - 1) >> OFFSET);
                        allBwMvs[block_y][block_x][bw_ref][0][1] = ((MULTI / TRp) * (1 - TRd * tmpmvs[pic_block_y][pic_block_x][1]) - 1)
                                >> OFFSET;
                    } else {
                        allFwMvs[block_y][block_x][refframe][0][1] = ((MULTI / TRp) * (1 + TRb * tmpmvs[pic_block_y][pic_block_x][1]) -
                                1) >> OFFSET;
                        allBwMvs[block_y][block_x][bw_ref][0][1] = -(((MULTI / TRp) * (1 + TRd * tmpmvs[pic_block_y][pic_block_x][1]) -
                                1) >> OFFSET);

                    }
#if HALF_PIXEL_COMPENSATION_DIRECT
                    if (img->is_field_sequence) {
                        int delta, delta2, delta_d, delta2_d;
                        int oriPOC = frame_no_next_P;
                        int oriRefPOC = frame_no_next_P - TRp;
                        int scaledPOC = frame_no_B;
                        int scaledRefPOC = frame_no_B - TRb;
                        getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);

                        scaledRefPOC = frame_no_B - TRd;
                        getDeltas(&delta_d, &delta2_d, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                        assert(delta_d == delta);

                        if (tmpmvs[pic_block_y][pic_block_x][1] + delta < 0) {
                            allFwMvs[block_y][block_x][refframe][0][1] = -(((MULTI / TRp) * (1 - TRb * (tmpmvs[pic_block_y][pic_block_x][1] +
                                    delta)) - 1) >> OFFSET) - delta2;
                            allBwMvs[block_y][block_x][bw_ref][0][1] = (((MULTI / TRp) * (1 - TRd * (tmpmvs[pic_block_y][pic_block_x][1] +
                                    delta_d)) - 1) >> OFFSET) - delta2_d;
                        } else {
                            allFwMvs[block_y][block_x][refframe][0][1] = (((MULTI / TRp) * (1 + TRb * (tmpmvs[pic_block_y][pic_block_x][1] +
                                    delta)) - 1) >> OFFSET) - delta2;;
                            allBwMvs[block_y][block_x][bw_ref][0][1] = -(((MULTI / TRp) * (1 + TRd * (tmpmvs[pic_block_y][pic_block_x][1] +
                                    delta_d)) - 1) >> OFFSET) - delta2_d;;
                        }

                    }
#endif
#endif
                }


                allFwMvs [block_y][block_x][refframe][0][0] = Clip3(-32768, 32767, allFwMvs [block_y][block_x][refframe][0][0]);
                allBwMvs [block_y][block_x][bw_ref][0][0] = Clip3(-32768, 32767, allBwMvs [block_y][block_x][bw_ref][0][0]);
                allFwMvs [block_y][block_x][refframe][0][1] = Clip3(-32768, 32767, allFwMvs [block_y][block_x][refframe][0][1]);
                allBwMvs [block_y][block_x][bw_ref][0][1] = Clip3(-32768, 32767, allBwMvs [block_y][block_x][bw_ref][0][1]);
#if Mv_check_bug
				img->mv_range_flag *= check_mv_range(uiBitSize, allFwMvs [block_y][block_x][refframe][0][0],
					allFwMvs [block_y][block_x][refframe][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0, 0, 0);
				img->mv_range_flag *= check_mv_range(uiBitSize, allBwMvs [block_y][block_x][bw_ref][0][0],
					allBwMvs [block_y][block_x][bw_ref][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0, 0, 0);
#else
				img->mv_range_flag *= check_mv_range(uiBitSize, allFwMvs [block_y][block_x][refframe][0][0],
					allFwMvs [block_y][block_x][refframe][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0);
				img->mv_range_flag *= check_mv_range(uiBitSize, allBwMvs [block_y][block_x][bw_ref][0][0],
					allBwMvs [block_y][block_x][bw_ref][0][1],
					pic_block_x << MIN_BLOCK_SIZE_IN_BIT, pic_block_y << MIN_BLOCK_SIZE_IN_BIT, 0);
#endif
                
            }
        }


        // only calculate block 0
        if (uiBitSize == MIN_CU_SIZE_IN_BIT) {
            block_y = 2;
            block_x = 2;
        }

    }


    // copy MV of block 0 to block 1/2/3
    if (uiBitSize == MIN_CU_SIZE_IN_BIT) {
        for (block_y = 0; block_y < 2; block_y++) {
            for (block_x = 0; block_x < 2; block_x++) {
                if (block_x != 0 || block_y != 0) {
                    allFwMvs[block_y][block_x][0][0][0] = allFwMvs[0][0][0][0][0];
                    allFwMvs[block_y][block_x][0][0][1] = allFwMvs[0][0][0][0][1];
                    allBwMvs[block_y][block_x][0][0][0] = allBwMvs[0][0][0][0][0];
                    allBwMvs[block_y][block_x][0][0][1] = allBwMvs[0][0][0][0][1];
                }
            }
        }
    }


    SetSkipMotionVectorPredictor(uiBitSize, uiPositionInPic, fwrefarr, bwrefarr, tmpmvfw, tmpmvbw, 0, 0, 0,
                                 (1 << uiBitSize), (1 << uiBitSize), 1);

    for (j = 0; j < 6; j++) {
#if Mv_check_bug
		img->mv_range_flag *= check_mv_range(uiBitSize, img->tmp_fwBSkipMv[j][0], img->tmp_fwBSkipMv[j][1], pix_x, pix_y, 0, 0, 0);
		img->mv_range_flag *= check_mv_range(uiBitSize, img->tmp_bwBSkipMv[j][0], img->tmp_bwBSkipMv[j][1], pix_x, pix_y, 0, 0, 0);
#else
		img->mv_range_flag *= check_mv_range(uiBitSize, img->tmp_fwBSkipMv[j][0], img->tmp_fwBSkipMv[j][1], pix_x, pix_y, 0);
		img->mv_range_flag *= check_mv_range(uiBitSize, img->tmp_bwBSkipMv[j][0], img->tmp_bwBSkipMv[j][1], pix_x, pix_y, 0);
#endif
        
    }

}

/*
*************************************************************************
* Function:control the sign of a with b
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int sign(int a, int b)
{
    int x;
    x = absm(a);

    if (b >= 0) {
        return x;
    } else {
        return -x;
    }
}


