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
* File name: codingUnit.c
* Function: Decode a codingUnit
*
*************************************************************************************
*/



#include "../../lcommon/inc/contributors.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "codingUnit.h"
#include "vlc.h"
#include "../../lcommon/inc/transform.h"
#include "../../lcommon/inc/defines.h"
#include "../../lcommon/inc/intra-prediction.h"
#include "../../lcommon/inc/inter-prediction.h"
#include "../../lcommon/inc/block_info.h"
#include "../../lcommon/inc/memalloc.h"
#include "block.h"
#include "AEC.h"

// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif

extern  short IQ_SHIFT[80];
extern  unsigned  short IQ_TAB[80];

void readMotionVector(unsigned int uiBitSize, unsigned int uiPositionInPic);
void readReferenceIndex(unsigned int uiBitSize, unsigned int uiPositionInPic);

extern int DCT_Pairs;


int dmh_pos[DMH_MODE_NUM + DMH_MODE_NUM - 1][2][2] = {
    { { 0,  0},  {0,  0 } },
    { { -1,  0 }, {1,  0 } },
    { { 0, -1 }, {0,  1 } },
    { { -1,  1 }, {1, -1 } },
    { { -1, -1 }, {1,  1 } },
    { { -2,  0 }, {2,  0 } },
    { { 0, -2 }, {0,  2 } },
    { { -2,  2 }, {2, -2 } },
    { { -2, -2 }, {2,  2 } }
};

void pmvr_mv_derivation(int mv[2], int mvd[2], int mvp[2]);

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
* Function:Set motion vector predictor
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
#define MEDIAN(a,b,c)  (a + b + c - min(a, min(b, c)) - max(a, max(b, c)));

void SetMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  pmv[2], int  **refFrArr,
                              int  ***tmp_mv, int  ref_frame, int  mb_pix_x, int  mb_pix_y, int  blockshape_x, int  blockshape_y, int  ref,
                              int  direct_mv)  //Lou 1016//qyu 0816
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

    mb_available_up = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                                         mb_width].slice_nr);      //jlzheng 6.23
    mb_available_left = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                                           1].slice_nr);      // jlzheng 6.23
    mb_available_upleft = (mb_x == 0 ||
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
    } else if (input->g_uiMaxSizeInBit == B8X8_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix8[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
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
        if ((ref_frame == img->num_of_references - 1 && rFrameL != img->num_of_references - 1 ||
             ref_frame != img->num_of_references - 1 && rFrameL == img->num_of_references - 1) && (img->type == INTER_IMG ||
                     img->type == F_IMG) && hd->background_reference_enable) {
            rFrameL = -1;
        }
        if (img->typeb == BP_IMG) {
            rFrameL = -1;
        }

    }
    if (rFrameU != -1) {
        if ((ref_frame == img->num_of_references - 1 && rFrameU != img->num_of_references - 1 ||
             ref_frame != img->num_of_references - 1 && rFrameU == img->num_of_references - 1) && (img->type == INTER_IMG ||
                     img->type == F_IMG) && hd->background_reference_enable) {
            rFrameU = -1;
        }
        if (img->typeb == BP_IMG) {
            rFrameU = -1;
        }

    }
    if (rFrameUR != -1) {
        if ((ref_frame == img->num_of_references - 1 && rFrameUR != img->num_of_references - 1 ||
             ref_frame != img->num_of_references - 1 && rFrameUR == img->num_of_references - 1) && (img->type == INTER_IMG ||
                     img->type == F_IMG) && hd->background_reference_enable) {
            rFrameUR = -1;
        }
        if (img->typeb == BP_IMG) {
            rFrameUR = -1;
        }

    }
    if (rFrameUL != -1) {
        if ((ref_frame == img->num_of_references - 1 && rFrameUL != img->num_of_references - 1 ||
             ref_frame != img->num_of_references - 1 && rFrameUL == img->num_of_references - 1) && (img->type == INTER_IMG ||
                     img->type == F_IMG) && hd->background_reference_enable) {
            rFrameUL = -1;
        }
        if (img->typeb == BP_IMG) {
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

        mva[hv] = mv_a = block_available_left    ? tmp_mv[pic_block_y - off_y][pic_block_x - 1][hv]              : 0;
        mvb[hv] = mv_b = block_available_up      ? tmp_mv[pic_block_y - y_up][pic_block_x][hv]                : 0;
        mv_d = block_available_upleft  ? tmp_mv[pic_block_y - y_upleft][pic_block_x - 1][hv]              : 0;

        mvc[hv] = mv_c = block_available_upright ? tmp_mv[pic_block_y - y_upright][pic_block_x +
                         (blockshape_x / MIN_BLOCK_SIZE) ][hv] : mv_d;

        if ((rFrameL == -1 && (img->type == INTER_IMG || img->type == F_IMG) && hd->background_reference_enable) ||
            (rFrameL == -1 && img->typeb == BP_IMG))

            mva[hv] = 0;
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

        if ((rFrameU == -1 && (img->type == INTER_IMG || img->type == F_IMG) && hd->background_reference_enable) ||
            (rFrameU == -1 && img->typeb == BP_IMG))

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
#if Mv_Clip
			mvb[hv] = scale_motion_vector(mvb[hv], ref_frame, rFrameU,
			ref,0);  //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#else
            mvb[hv] = scale_motion_vector(mvb[hv], ref_frame, rFrameU,
                                          ref);  //, smbtypecurr, smbtypeU, pic_block_y-y_up, pic_block_y, ref, direct_mv);
#endif
#endif
//#endif

        if ((rFrameUL == -1 && (img->type == INTER_IMG || img->type == F_IMG) && hd->background_reference_enable) ||
            (rFrameUL == -1 && img->typeb == BP_IMG))

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

        if ((rFrameUR == -1 && (img->type == INTER_IMG || img->type == F_IMG) && hd->background_reference_enable) ||
            (rFrameUR == -1 && img->typeb == BP_IMG))

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
            mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref,0);
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
                        mva[2] = abs(mva[0] - mvb[0]);
                        // !! for Bx
                        mvb[2] = abs(mvb[0] - mvc[0]);
                        // !! for Cx
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
                        mva[2] = abs(mva[1] - mvb[1]);
                        // !! for By
                        mvb[2] = abs(mvb[1] - mvc[1]);
                        // !! for Cy
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
void SetSkipMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  fw_pmv[2], int bw_pmv[2],
                                  int  ref_frame, int  mb_pix_x, int  mb_pix_y, int  blockshape_x, int  blockshape_y, int num_skip_dir)
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

    int tmp_bwBSkipMv[5][2];
    int tmp_fwBSkipMv[5][2];
    int mode_info[6];
    PixelPos block_L, block_U, block_UR, block_UL, block_L1, block_U1;
    codingUnit *neighborMB;
    int sizeOfNeighborBlock_x, sizeOfNeighborBlock_y;


    blockshape_x = blockshape_x % MIN_BLOCK_SIZE == 0 ? blockshape_x : MIN_BLOCK_SIZE;
    blockshape_y = blockshape_y % MIN_BLOCK_SIZE == 0 ? blockshape_y : MIN_BLOCK_SIZE;

    mb_available_up = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - mb_width].slice_nr);
    mb_available_left = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - 1].slice_nr);
    mb_available_upleft = (mb_x == 0 ||
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
    } else if (input->g_uiMaxSizeInBit == B8X8_IN_BIT) {
        upright = g_Up_Right_Avail_Matrix8[pic_block_y - img->block8_y][pic_block_x - img->block8_x + blockshape_x /
                  MIN_BLOCK_SIZE - 1];;
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

    rFrameL[0]    = block_available_left    ? img->bw_refFrArr[pic_block_y  - off_y]  [pic_block_x - 1] : -1;
    rFrameU[0]    = block_available_up      ? img->bw_refFrArr[pic_block_y - y_up][pic_block_x]   : -1;
    rFrameUR[0]   = block_available_upright ? img->bw_refFrArr[pic_block_y - y_upright][pic_block_x +
                    (blockshape_x / MIN_BLOCK_SIZE) ] :  -1;
    rFrameUL[0]   = block_available_upleft  ? img->bw_refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameL1[0] = block_available_left1 ? img->bw_refFrArr[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE -
                  1][pic_block_x - 1] : -1;
    rFrameU1[0] = block_available_up1 ? img->bw_refFrArr[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                  1] : -1;

    rFrameL[1]    = block_available_left    ? img->fw_refFrArr[pic_block_y  - off_y]  [pic_block_x - 1] : -1;
    rFrameU[1]    = block_available_up      ? img->fw_refFrArr[pic_block_y - y_up][pic_block_x]   : -1;
    rFrameUR[1]   = block_available_upright ? img->fw_refFrArr[pic_block_y - y_upright][pic_block_x +
                    (blockshape_x / MIN_BLOCK_SIZE) ] :  -1;
    rFrameUL[1]   = block_available_upleft  ? img->fw_refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameL1[1] = block_available_left1 ? img->fw_refFrArr[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE -
                  1][pic_block_x - 1] : -1;
    rFrameU1[1] = block_available_up1 ? img->fw_refFrArr[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                  1] : -1;

    for (i = 0; i < 2; i++) {
        bRefFrame[i][0] = rFrameUL[i];
        bRefFrame[i][1] = rFrameU[i];
        bRefFrame[i][2] = rFrameL[i];
        bRefFrame[i][3] = rFrameUR[i];
        bRefFrame[i][4] = rFrameU1[i];
        bRefFrame[i][5] = rFrameL1[i];
    }
    for (hv = 0; hv < 2; hv++) {
        mva[0][hv]  = block_available_left    ? img->bw_mv[pic_block_y - off_y][pic_block_x - 1][hv]              : 0;
        mva1[0][hv]  = block_available_left1 ? img->bw_mv[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x -
                       1][hv] : 0;
        mvb[0][hv] = block_available_up      ? img->bw_mv[pic_block_y - y_up][pic_block_x][hv]                : 0;
        mvb1[0][hv]  = block_available_up1 ? img->bw_mv[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                       1][hv] : 0;
        mve[0][hv] = block_available_upleft  ? img->bw_mv[pic_block_y - y_upleft][pic_block_x - 1][hv]              : 0;
        mvc[0][hv]  = block_available_upright ? img->bw_mv[pic_block_y - y_upright][pic_block_x +
                      (blockshape_x / MIN_BLOCK_SIZE) ][hv] : 0;

        mva[1][hv]  = block_available_left    ? img->fw_mv[pic_block_y - off_y][pic_block_x - 1][hv]              : 0;
        mva1[1][hv]  = block_available_left1 ? img->fw_mv[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE - 1][pic_block_x -
                       1][hv] : 0;
        mvb[1][hv] = block_available_up      ? img->fw_mv[pic_block_y - y_up][pic_block_x][hv]                : 0;
        mvb1[1][hv]  = block_available_up1 ? img->fw_mv[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                       1][hv] : 0;
        mve[1][hv] = block_available_upleft  ? img->fw_mv[pic_block_y - y_upleft][pic_block_x - 1][hv]              : 0;
        mvc[1][hv]  = block_available_upright ? img->fw_mv[pic_block_y - y_upright][pic_block_x +
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
            tmp_bwBSkipMv[dir][i] = 0;
            tmp_fwBSkipMv[dir][i] = 0;
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
        mode_info[2] = img->mb_data[block_L.mb_addr].b8pdir[block_L.x + block_L.y * 2];

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
        mode_info[3] =  img->mb_data[block_UR.mb_addr].b8pdir[block_UR.x + block_UR.y * 2];

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
        mode_info[4] =  img->mb_data[block_U1.mb_addr].b8pdir[block_U1.x + block_U1.y * 2];

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
            tmp_bwBSkipMv[DS_BID][0] = pmv[0][0][j];
            tmp_bwBSkipMv[DS_BID][1] = pmv[0][1][j];
            tmp_fwBSkipMv[DS_BID][0] = pmv[1][0][j];
            tmp_fwBSkipMv[DS_BID][1] = pmv[1][1][j];
            bid_flag++;
            if (bid_flag == 1) {
                bid2 = j;
            }
        } else if (mode_info[j] == SYM) {
            tmp_bwBSkipMv[DS_SYM][0] = pmv[0][0][j];
            tmp_bwBSkipMv[DS_SYM][1] = pmv[0][1][j];
            tmp_fwBSkipMv[DS_SYM][0] = pmv[1][0][j];
            tmp_fwBSkipMv[DS_SYM][1] = pmv[1][1][j];
            sym_flag++;
        } else if (mode_info[j] == BACKWARD) {
            tmp_bwBSkipMv[DS_BACKWARD][0] = pmv[0][0][j];
            tmp_bwBSkipMv[DS_BACKWARD][1] = pmv[0][1][j];
            bw_flag++;
        } else if (mode_info[j] == FORWARD) {
            tmp_fwBSkipMv[DS_FORWARD][0] = pmv[1][0][j];
            tmp_fwBSkipMv[DS_FORWARD][1] = pmv[1][1][j];
            fw_flag++;
        }
    }

    if (bid_flag == 0 && fw_flag != 0 && bw_flag != 0) {
        tmp_bwBSkipMv[DS_BID][0] = tmp_bwBSkipMv[DS_BACKWARD][0];
        tmp_bwBSkipMv[DS_BID][1] = tmp_bwBSkipMv[DS_BACKWARD][1];
        tmp_fwBSkipMv[DS_BID][0] = tmp_fwBSkipMv[DS_FORWARD][0];
        tmp_fwBSkipMv[DS_BID][1] = tmp_fwBSkipMv[DS_FORWARD][1];
    }

    if (sym_flag == 0 && bid_flag > 1) {
        tmp_bwBSkipMv[DS_SYM][0] = pmv[0][0][bid2];
        tmp_bwBSkipMv[DS_SYM][1] = pmv[0][1][bid2];
        tmp_fwBSkipMv[DS_SYM][0] = pmv[1][0][bid2];
        tmp_fwBSkipMv[DS_SYM][1] = pmv[1][1][bid2];
    } else if (sym_flag == 0 && bw_flag != 0) {
        tmp_bwBSkipMv[DS_SYM][0] = tmp_bwBSkipMv[DS_BACKWARD][0];
        tmp_bwBSkipMv[DS_SYM][1] = tmp_bwBSkipMv[DS_BACKWARD][1];
        tmp_fwBSkipMv[DS_SYM][0] = -tmp_bwBSkipMv[DS_BACKWARD][0];
        tmp_fwBSkipMv[DS_SYM][1] = -tmp_bwBSkipMv[DS_BACKWARD][1];
#if Mv_Clip
		 tmp_fwBSkipMv[DS_SYM][0] =Clip3(-32768, 32767,tmp_fwBSkipMv[DS_SYM][0]);
		 tmp_fwBSkipMv[DS_SYM][1] =Clip3(-32768, 32767,tmp_fwBSkipMv[DS_SYM][1]);
#endif
    } else if (sym_flag == 0 && fw_flag != 0) {
        tmp_bwBSkipMv[DS_SYM][0] = -tmp_fwBSkipMv[DS_FORWARD][0];
        tmp_bwBSkipMv[DS_SYM][1] = -tmp_fwBSkipMv[DS_FORWARD][1];
        tmp_fwBSkipMv[DS_SYM][0] = tmp_fwBSkipMv[DS_FORWARD][0];
        tmp_fwBSkipMv[DS_SYM][1] = tmp_fwBSkipMv[DS_FORWARD][1];
#if Mv_Clip
		 tmp_bwBSkipMv[DS_SYM][0] = Clip3(-32768, 32767,tmp_bwBSkipMv[DS_SYM][0]);
		 tmp_bwBSkipMv[DS_SYM][1] = Clip3(-32768, 32767,tmp_bwBSkipMv[DS_SYM][1]);
#endif
    }

    if (bw_flag == 0 && bid_flag > 1) {
        tmp_bwBSkipMv[DS_BACKWARD][0] = pmv[0][0][bid2];
        tmp_bwBSkipMv[DS_BACKWARD][1] = pmv[0][1][bid2];
    } else if (bw_flag == 0 && bid_flag != 0) {
        tmp_bwBSkipMv[DS_BACKWARD][0] = tmp_bwBSkipMv[DS_BID][0];
        tmp_bwBSkipMv[DS_BACKWARD][1] = tmp_bwBSkipMv[DS_BID][1];
    }

    if (fw_flag == 0 && bid_flag > 1) {
        tmp_fwBSkipMv[DS_FORWARD][0] = pmv[1][0][bid2];
        tmp_fwBSkipMv[DS_FORWARD][1] = pmv[1][1][bid2];
    } else if (fw_flag == 0 && bid_flag != 0) {
        tmp_fwBSkipMv[DS_FORWARD][0] = tmp_fwBSkipMv[DS_BID][0];
        tmp_fwBSkipMv[DS_FORWARD][1] = tmp_fwBSkipMv[DS_BID][1];
    }


    fw_pmv[0] = tmp_fwBSkipMv[num_skip_dir][0];
    fw_pmv[1] = tmp_fwBSkipMv[num_skip_dir][1];
    bw_pmv[0] = tmp_bwBSkipMv[num_skip_dir][0];
    bw_pmv[1] = tmp_bwBSkipMv[num_skip_dir][1];



}

void start_codingUnit(unsigned int uiBitSize)
{
    int i, j; //,k,l;

    assert(img->current_mb_nr >= 0 && img->current_mb_nr < img->max_mb_nr);

    // Update coordinates of the current codingUnit
    img->mb_x = (img->current_mb_nr) % (img->PicWidthInMbs);
    img->mb_y = (img->current_mb_nr) / (img->PicWidthInMbs);

    // Define vertical positions
    img->block8_y = img->mb_y * BLOCK_MULTIPLE;    // luma block position /
    //  img->block8_y = img->mb_y * BLOCK_SIZE/2;
    img->pix_y   = img->mb_y * MIN_CU_SIZE;   // luma codingUnit position /

    if (input->chroma_format == 1) {
        img->pix_c_y = img->mb_y * MIN_CU_SIZE / 2;  // chroma codingUnit position /
    }

    // Define horizontal positions /
    img->block8_x = img->mb_x * BLOCK_MULTIPLE;    // luma block position /
    // img->block8_x = img->mb_x * BLOCK_SIZE/2;
    img->pix_x   = img->mb_x * MIN_CU_SIZE;   // luma pixel position /
    img->pix_c_x = img->mb_x * MIN_CU_SIZE / 2; // chroma pixel position /


    // initialize img->resiY for ABT//Lou

    for (j = 0; j < (1 << (uiBitSize + 1)); j++) {
        for (i = 0; i < (1 << (uiBitSize + 1)); i++) {
            img->resiY[j][i] = 0;
        }
    }

    for (j = 0; j < (1 << uiBitSize); j++) {
        for (i = 0; i < (1 << uiBitSize); i++) {
            img->resiUV[0][j][i] = 0;
            img->resiUV[1][j][i] = 0;
        }
    }
}

void Init_SubMB(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int i, j, k, l, r, c;
    codingUnit *currMB = &img->mb_data[uiPositionInPic];  // intialization code deleted, see below, StW

    int N8_SizeScale = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    assert(uiPositionInPic >= 0 && uiPositionInPic < (unsigned int)img->max_mb_nr);

    // Reset syntax element entries in MB struct
    currMB->ui_MbBitSize = uiBitSize;
    currMB->qp          = img->qp ;

#if MB_DQP
    currMB->previouse_qp     = img->qp ;
    currMB->left_cu_qp       = img->qp ;
#endif

    currMB->cuType     = PSKIPDIRECT;
    currMB->delta_quant = 0;
    currMB->cbp         = 0;
    currMB->cbp_blk     = 0;
    currMB->c_ipred_mode = DC_PRED_C; //GB

    for (l = 0; l < 2; l++) {
        for (j = 0; j < BLOCK_MULTIPLE; j++) {
            for (i = 0; i < BLOCK_MULTIPLE; i++) {
                for (k = 0; k < 2; k++) {
                    currMB->mvd[l][j][i][k] = 0;
                }
            }
        }
    }

    currMB->cbp_bits   = 0;

    for (r = 0; r < N8_SizeScale; r++) {
        int pos = uiPositionInPic + r * img->PicWidthInMbs;

        for (c = 0; c < N8_SizeScale; c++, pos++) {
            if (r + c == 0) {
                continue;
            }

            memcpy(&img->mb_data[pos], &img->mb_data[uiPositionInPic], sizeof(codingUnit));
        }
    }
}
/*
*************************************************************************
* Function:Interpret the mb mode for P-Frames
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

void PskipMV_COL(codingUnit *currMB, unsigned int uiPositionInPic, int block8_in_row, int block8_in_col,
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
    int refframe;
    int curT, colT;
    int i, j, l, m;
#if HALF_PIXEL_PSKIP
    int delta, delta2;
    delta = delta2 = 0;
#endif

    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            int block_x = pic_block8_x + blockshape_block_x / 2 * i ;
            int block_y = pic_block8_y + blockshape_block_y / 2 * j ;
            refframe = col_ref[block_y ][block_x];

            if (refframe >= 0) {
                curT = (2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance) + 512) % 512;
                colT = (2 * (fref[0]->imgtr_fwRefDistance - col_pic_dist[refframe]) + 512) % 512;
                if (0 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                    curT = 1;
                    colT = 1;
                }
                if (refframe == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
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
                scalingMV(&cur_mv[block_y ][block_x ][0], &cur_mv[block_y ][block_x][1], curT, col_mv[block_y][block_x][0],
                          col_mv[block_y][block_x][1] + delta, colT, 1);
                cur_mv[block_y ][block_x][1] -= delta2;
#else
                scalingMV(&cur_mv[block_y ][block_x ][0], &cur_mv[block_y ][block_x][1], curT, col_mv[block_y][block_x][0],
                          col_mv[block_y][block_x][1], colT, 1);
#endif
                cur_mv[block_y ][block_x ][0] = Clip3(-32768, 32767, cur_mv[block_y ][block_x ][0]);
                cur_mv[block_y ][block_x ][1] = Clip3(-32768, 32767, cur_mv[block_y ][block_x ][1]);
            } else {
                cur_mv[block_y ][block_x ][0] = 0;
                cur_mv[block_y ][block_x ][1] = 0;
            }

            for (l = 0; l < blockshape_block_x / 2; l++) {
                for (m = 0; m < blockshape_block_y / 2; m++) {
                    cur_mv[block_y + m][block_x + l][0] = cur_mv[block_y ][block_x ][0] ;
                    cur_mv[block_y + m][block_x + l][1] = cur_mv[block_y ][block_x ][1] ;
                }
            }

        }
    }
    if (currMB->ui_MbBitSize == MIN_CU_SIZE_IN_BIT) {

        for (i = 0; i < 2; i++) {
            for (j = 0; j < 2; j++) {

                int block_x = pic_block8_x + blockshape_block_x / 2 * i ;
                int block_y = pic_block8_y + blockshape_block_y / 2 * j ;

                cur_mv[block_y ][block_x ][0] = cur_mv[pic_block8_y ][pic_block8_x ][0] ;
                cur_mv[block_y ][block_x ][1] = cur_mv[pic_block8_y ][pic_block8_x ][1] ;


                for (l = 0; l < blockshape_block_x / 2; l++) {
                    for (m = 0; m < blockshape_block_y / 2; m++) {
                        cur_mv[block_y + m][block_x + l][0] = cur_mv[block_y ][block_x ][0] ;
                        cur_mv[block_y + m][block_x + l][1] = cur_mv[block_y ][block_x ][1] ;
                    }
                }
            }
        }
    }
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

    mb_available_up = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - mb_width].slice_nr);
    mb_available_left = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr - 1].slice_nr);
    mb_available_upleft = (mb_x == 0 ||
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
    rFrameUL[1] = block_available_upleft ? hc->p_snd_refFrArr[pic_block_y - y_upleft][pic_block_x - 1] : -1;
    rFrameL1[1] = block_available_left1 ? hc->p_snd_refFrArr[pic_block_y - off_y + blockshape_y / MIN_BLOCK_SIZE -
                  1][pic_block_x - 1] : -1;
    rFrameU1[1] = block_available_up1 ? hc->p_snd_refFrArr[pic_block_y - y_up][pic_block_x + blockshape_x / MIN_BLOCK_SIZE -
                  1] : -1;


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
* Function:Interpret the mb mode for P-Frames
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void interpret_mb_mode_PF(unsigned int uiPositionInPic, int pdir)
{
    int pdir0[4] = {FORWARD, FORWARD, DUAL, DUAL};
    int pdir1[4] = {FORWARD, DUAL, FORWARD, DUAL};
    int i, j, k;

    codingUnit *currMB = &img->mb_data[uiPositionInPic];//GB current_mb_nr];
    int         mbmode = currMB->cuType;
    int N8_SizeScale = 1 << (currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT);

    int num_of_orgMB_in_row = N8_SizeScale;//4:1  5:2  6:4
    int num_of_orgMB_in_col = N8_SizeScale;
    int size = 1 << currMB->ui_MbBitSize;
    int pix_x = (uiPositionInPic % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (uiPositionInPic / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

    if (pix_x + size >= img->width) {
        num_of_orgMB_in_col = min(N8_SizeScale, (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT);
    }

    if (pix_y + size >= img->height) {
        num_of_orgMB_in_row = min(N8_SizeScale, (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT);
    }
    if (mbmode == 0) {
        for (i = 0; i < 4; i++) {
            currMB->b8mode[i] = mbmode;
            currMB->b8pdir[i] = FORWARD;
        }
    } else if (mbmode == 1) {   // 16x16
        for (i = 0; i < 4; i++) {
            currMB->b8mode[i] = mbmode;
            currMB->b8pdir[i] = pdir == 0 ? FORWARD : DUAL;
        }
    } else if (mbmode == 2 || mbmode == 4 || mbmode == 5) {   // 16x8, 16x4, 16x12
        for (i = 0; i < 4; i++) {
            currMB->b8mode[i] = mbmode;
        }
        currMB->b8pdir[0] = currMB->b8pdir[1] = pdir0 [pdir];
        currMB->b8pdir[2] = currMB->b8pdir[3] = pdir1 [pdir];
    } else if (mbmode == 3 || mbmode == 6 || mbmode == 7) {
        for (i = 0; i < 4; i++) {
            currMB->b8mode[i] = mbmode;
        }
        currMB->b8pdir[0] = currMB->b8pdir[2] = pdir0[pdir];
        currMB->b8pdir[1] = currMB->b8pdir[3] = pdir1[pdir];
    } else if (mbmode >= 9) {   //modefy by xfwang 2004.7.29
		#if !REMOVE_UNUSED
        currMB->cbp = NCBP[currMB->cuType - 9][0]; // qhg  //modefy by xfwang 2004.7.29
        #endif
		currMB->cuType = I8MB;

        for (i = 0; i < 4; i++) {
            currMB->b8mode[i] = IBLOCK;
            currMB->b8pdir[i] = INTRA;
        }
    }

    for (i = 0; i < num_of_orgMB_in_row; i++) {
        int pos = uiPositionInPic + i * img->PicWidthInMbs;

        for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
            codingUnit *tmpMB = &img->mb_data[pos];
            tmpMB->cuType = currMB->cuType;
            tmpMB->cbp = currMB->cbp;

            for (k = 0; k < 4; k++) {
                tmpMB->b8mode[k] = currMB->b8mode[k];
                tmpMB->b8pdir[k] = currMB->b8pdir[k];
            }
        }
    }
}

/*
*************************************************************************
* Function:Interpret the mb mode for I-Frames
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void interpret_mb_mode_I(unsigned int uiPositionInPic)
{
    int i, j, k;
    codingUnit *currMB   = &img->mb_data[uiPositionInPic];
    int         num      = 4;
    int N8_SizeScale = 1 << (currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT);
    int num_of_orgMB_in_row = N8_SizeScale;//4:1  5:2  6:4
    int num_of_orgMB_in_col = N8_SizeScale;
    int size = 1 << currMB->ui_MbBitSize;
    int pix_x = (uiPositionInPic % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (uiPositionInPic / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

    if (pix_x + size >= img->width) {
        num_of_orgMB_in_col = min(N8_SizeScale, (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT);
    }

    if (pix_y + size >= img->height) {
        num_of_orgMB_in_row = min(N8_SizeScale, (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT);
    }
    currMB->cuType = I8MB;


    for (i = 0; i < 4; i++) {
        currMB->b8mode[i] = IBLOCK;
        currMB->b8pdir[i] = INTRA;
    }

    for (i = num; i < 4; i++) {
        currMB->b8mode[i] = 0/*currMB->cuType_2==PNXN? 4 : currMB->cuType_2*/;
        currMB->b8pdir[i] = FORWARD;
    }

    for (i = 0; i < num_of_orgMB_in_row; i++) {
        int pos = uiPositionInPic + i * img->PicWidthInMbs;

        for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
            codingUnit *tmpMB = &img->mb_data[pos];
            tmpMB->cuType = currMB->cuType;

            for (k = 0; k < 4; k++) {
                tmpMB->b8mode[k] = currMB->b8mode[k];
                tmpMB->b8pdir[k] = currMB->b8pdir[k];
            }
        }
    }
}

/*
*************************************************************************
* Function:Interpret the mb mode for B-Frames
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void interpret_mb_mode_B(unsigned int uiPositionInPic, int pdir)
{

    int pdir0[16] = {FORWARD, BACKWARD, FORWARD, BACKWARD, FORWARD, BACKWARD, SYM, SYM, SYM, FORWARD, BACKWARD, SYM, BID, BID, BID, BID};
    int pdir1[16] = {FORWARD, BACKWARD, BACKWARD, FORWARD, SYM, SYM, FORWARD, BACKWARD, SYM, BID, BID, BID, FORWARD, BACKWARD, SYM, BID};


    codingUnit *currMB = &img->mb_data[uiPositionInPic];

    int i, mbmode, j, k;
    int mbtype  = currMB->cuType;
    int *b8mode = currMB->b8mode;
    int *b8pdir = currMB->b8pdir;
    int N8_SizeScale = 1 << (currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT);

    int num_of_orgMB_in_row = N8_SizeScale;//4:1  5:2  6:4
    int num_of_orgMB_in_col = N8_SizeScale;
    int size = 1 << currMB->ui_MbBitSize;
    int pix_x = (uiPositionInPic % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (uiPositionInPic / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

    if (pix_x + size >= img->width) {
        num_of_orgMB_in_col = min(N8_SizeScale, (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT);
    }

    if (pix_y + size >= img->height) {
        num_of_orgMB_in_row = min(N8_SizeScale, (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT);
    }

    //--- set mbtype, b8type, and b8pdir ---
    if (mbtype == 0) {   // direct
        mbmode = 0;
        switch (currMB->md_directskip_mode) {
        case 0:
        case DS_SYM:
            for (i = 0; i < 4; i++) {
                b8mode[i] = 0;
                b8pdir[i] = SYM;
            }
            break;
        case DS_BID:
            for (i = 0; i < 4; i++) {
                b8mode[i] = 0;
                b8pdir[i] = BID;
            }
            break;
        case DS_BACKWARD:
            for (i = 0; i < 4; i++) {
                b8mode[i] = 0;
                b8pdir[i] = BACKWARD;
            }
            break;
        case DS_FORWARD:
            for (i = 0; i < 4; i++) {
                b8mode[i] = 0;
                b8pdir[i] = FORWARD;
            }
            break;
        }
    } else if (mbtype == 8) {   // intra8x8
        mbmode = PNXN;     // b8mode and pdir is transmitted in additional codewords
    } else if (mbtype == 9) { // 8x8(+split)
		#if !REMOVE_UNUSED
        currMB->cbp = NCBP[mbtype - 23][0]; // qhg
		 #endif
        mbmode = I8MB;

        for (i = 0; i < 4; i++) {
            b8mode[i] = IBLOCK;
            b8pdir[i] = INTRA;
        }
    } else if (mbtype == 1) {   // 16x16
        mbmode = 1;

        for (i = 0; i < 4; i++) {
            b8mode[i] = 1;
            b8pdir[i] = pdir;
        }
    } else if (mbtype == 2 || mbtype == 4 || mbtype == 5) {   // 16x8, 16x4, 16x12
        //mbmode = 2;
        mbmode = mbtype;

        for (i = 0; i < 4; i++) {
            b8mode[i] = mbmode;//2;
        }
        b8pdir[0] = b8pdir[1] = pdir0 [pdir];
        b8pdir[2] = b8pdir[3] = pdir1 [pdir];
    } else if (mbtype == 3 || mbtype == 6 || mbtype == 7) {
        //mbmode = 3;
        mbmode = mbtype;

        for (i = 0; i < 4; i++) {
            b8mode[i] = mbmode;//3;
        }
        b8pdir[0] = b8pdir[2] = pdir0[pdir];
        b8pdir[1] = b8pdir[3] = pdir1[pdir];
    }

    currMB->cuType = mbmode;

    for (i = 0; i < num_of_orgMB_in_row; i++) {
        int pos = uiPositionInPic + i * img->PicWidthInMbs;

        for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
            codingUnit *tmpMB = &img->mb_data[pos];
            tmpMB->cuType = currMB->cuType;

            for (k = 0; k < 4; k++) {
                tmpMB->b8mode[k] = currMB->b8mode[k];
                tmpMB->b8pdir[k] = currMB->b8pdir[k];
            }
        }
    }
}

/*
*************************************************************************
* Function:init codingUnit I and P frames
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void init_codingUnit(unsigned int uiPositionInPic)
{
    int i, j;

    int r, c;
    int row, col;
    int width, height;
    codingUnit *currMB = &img->mb_data[uiPositionInPic];//GB current_mb_nr];
    int N8_SizeScale = 1 << (currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT);
    int block8_y = (uiPositionInPic / img->PicWidthInMbs) << 1;
    int block8_x = (uiPositionInPic % img->PicWidthInMbs) << 1;
    int num_of_orgMB_in_row = N8_SizeScale;//4:1  5:2  6:4
    int num_of_orgMB_in_col = N8_SizeScale;
    int size = 1 << currMB->ui_MbBitSize;
    int pix_x = (uiPositionInPic % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (uiPositionInPic / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

    if (pix_x + size >= img->width) {
        num_of_orgMB_in_col = min(N8_SizeScale, (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT);
    }

    if (pix_y + size >= img->height) {
        num_of_orgMB_in_row = min(N8_SizeScale, (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT);
    }

    for (i = 0; i < 2 * num_of_orgMB_in_col; i++) {
        for (j = 0; j < 2 * num_of_orgMB_in_row; j++) {
            img->tmp_mv[block8_y + j][block8_x + i][0] = 0;
            img->tmp_mv[block8_y + j][block8_x + i][1] = 0;
            img->tmp_mv[block8_y + j][block8_x + i][3] = 0;
            img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = 0;
            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = 0;
            img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
        }
    }

    for (i = 0; i < 2 * num_of_orgMB_in_col; i++) {
        for (j = 0; j < 2 * num_of_orgMB_in_row; j++) {
            img->ipredmode[block8_y + j + 1][block8_x + i + 1] = -1;    //by oliver 0512
        }
    }

    // Set the reference frame information for motion vector prediction
    if (IS_INTRA(currMB)) {
        for (i = 0; i < 2 * num_of_orgMB_in_col; i++) {
            for (j = 0; j < 2 * num_of_orgMB_in_row; j++) {
                hc->refFrArr[block8_y + j][block8_x + i] = -1;
                hc->p_snd_refFrArr[block8_y + j][block8_x + i] = -1;
            }
        }
    } else if (!IS_P8x8(currMB)) {
        for (j = 0; j < 2; j++) {
            for (i = 0; i < 2; i++) {
                get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i , j , &col , &row, &width, &height);
                for (r = 0; r < height; r++) {
                    for (c = 0; c < width; c++) {
                        hc->refFrArr[block8_y + row + r][block8_x + col + c] = 0;
                        hc->p_snd_refFrArr[block8_y + row + r][block8_x + col + c] = -1;
                    }
                }
            }
        }
    } else {
        for (j = 0; j < 2; j++) {
            for (i = 0; i < 2; i++) {
                get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i , j , &col , &row, &width, &height);
                for (r = 0; r < height; r++) {
                    for (c = 0; c < width; c++) {
                        hc->refFrArr[block8_y + row + r][block8_x + col + c] = (currMB->b8mode[(i / N8_SizeScale) + 2 *
                                (j / N8_SizeScale) ] == IBLOCK ? -1 : 0);
                        hc->p_snd_refFrArr[block8_y + row + r][block8_x + col + c] = -1;
                    }
                }
            }
        }
    }
}

/*
*************************************************************************
* Function:Sets mode for 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void SetB8Mode(codingUnit *currMB, int value, int i)
{
    static const int b_v2b8 [5] = {0, 8, 8, 8, 8};
    static const int b_v2pd [5] = {2, 0, 1, 2, 3};

    if (value >= 6) {
        value = (value >> 1) + (value & 0x0001);
    }


    if (img->type == B_IMG) {
        currMB->b8mode[i]   = b_v2b8[value];
        currMB->b8pdir[i]   = b_v2pd[value];
    } else {
        currMB->b8mode[i]   = 8;
        currMB->b8pdir[i]   = value == 0 ? FORWARD : DUAL;
    }

}

/*
*************************************************************************
* Function:Get the syntax elements from the NAL
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void read_MBHeader(codingUnit *currMB, unsigned int uiPositionInPic, int size, int num_of_orgMB_in_row,
                   int num_of_orgMB_in_col, int *pdir, int *real_cuType)
{
    SyntaxElement currSE;

    int weighted_skipmode_fix = 0;
    Slice *currSlice = img->currentSlice;
    DataPartition *dP;
    int i, j;
    int md_directskip_mode = 0;
    currMB->md_directskip_mode = 0;

    currSE.type = SE_MBTYPE;
    currSE.mapping = linfo_ue;
    CheckAvailabilityOfNeighbors(currMB, currMB->ui_MbBitSize, uiPositionInPic);    //need fix
    if (img->type == I_IMG) {
        dP = & (currSlice->partArr[0]);
        currSE.reading = readcuTypeInfo;
        dP->readSyntaxElement(&currSE, dP, currMB,
                              uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position

        *real_cuType = currSE.value1;
        if (currSE.value1 < 0) {
            currSE.value1 = 0;
        }
    } else if (img->type == P_IMG && img->typeb == BP_IMG) {
        dP = & (currSlice->partArr[0]);
        currSE.reading = readcuTypeInfo_SFRAME;
        dP->readSyntaxElement(&currSE, dP, currMB,
                              uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position
        *real_cuType = currSE.value1;
        if (currSE.value1 < 0) {
            currSE.value1 = 0;
        }
    } else { //qyu 0822 delete skip_mode_flag
        dP = & (currSlice->partArr[0]);
        currSE.reading = readcuTypeInfo;
        dP->readSyntaxElement(&currSE, dP, currMB,
                              uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position
        if ((img->type == F_IMG) || (img->type == P_IMG)) {
            currSE.value1++;
        }

        currSE.value1--;
        *real_cuType = currSE.value1;

        if (currSE.value1 < 0) {
            currSE.value1 = 0;
        }
    }
    for (i = 0; i < num_of_orgMB_in_row; i++) {
        int pos = uiPositionInPic + i * img->PicWidthInMbs;

        for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
            codingUnit *tmpMB = &img->mb_data[pos];
            tmpMB->cuType = currSE.value1;
        }
    }

    if (img->type == B_IMG && (currMB->cuType >= P2NX2N && currMB->cuType <= PVER_RIGHT)) {
        dP = & (currSlice->partArr[0]);
        currSE.reading = readPdir;
        dP->readSyntaxElement(&currSE, dP, currMB,
                              uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position

        *pdir = currSE.value1;
    } else if (img->type == F_IMG  && img->num_of_references > 1 && hd->dhp_enabled && (currMB->cuType >= P2NX2N &&
               currMB->cuType <= PVER_RIGHT)) {

        if (currMB->ui_MbBitSize == B8X8_IN_BIT && currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT) {
            *pdir = 0;
        } else {
            dP = & (currSlice->partArr[0]);
            currSE.reading = readPdir_dhp;
            dP->readSyntaxElement(&currSE, dP, currMB,
                                  uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position
            *pdir = currSE.value1;
        }
    } else {
        *pdir = 0;
    }
    if (IS_P_SKIP(currMB) && img->type == F_IMG && hd->wsm_enabled && img->num_of_references > 1) {
        dP = & (currSlice->partArr[0]);
        currSE.type = SE_WPM1;
        currSE.reading = readWPM;
        dP->readSyntaxElement(&currSE, dP, currMB,
                              uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position

#if TRACE
        fprintf(hc->p_trace, "weighted_skipmode1 = %3d \n", currSE.value1);
#endif
        weighted_skipmode_fix = currSE.value1;
    }


    if (IS_P_SKIP(currMB)) {
        for (i = 0; i < num_of_orgMB_in_row; i++) {
            int pos = uiPositionInPic + i * img->PicWidthInMbs;

            for (j = 0; j < num_of_orgMB_in_row; j++, pos++) {
                codingUnit *tmpMB = &img->mb_data[pos];
                tmpMB->weighted_skipmode = weighted_skipmode_fix;
            }
        }
    }


    if (IS_P_SKIP(currMB) && (weighted_skipmode_fix == 0) && hd->b_mhpskip_enabled && img->type == F_IMG) {
        dP = & (currSlice->partArr[0]);
        currSE.reading = read_p_skip_mode;
        dP->readSyntaxElement(&currSE, dP, currMB,
                              uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position

        md_directskip_mode = currSE.value1;

#if TRACE
        fprintf(hc->p_trace, "p_directskip_mode = %3d \n", currSE.value1);
#endif

        for (i = 0; i < num_of_orgMB_in_row; i++) {
            int pos = uiPositionInPic + i * img->PicWidthInMbs;

            for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
                codingUnit *tmpMB = &img->mb_data[pos];
                tmpMB->md_directskip_mode = md_directskip_mode;
            }
        }
    }



    if (!(hd->b_mhpskip_enabled && img->type == F_IMG)) {
        for (i = 0; i < num_of_orgMB_in_row; i++) {
            int pos = uiPositionInPic + i * img->PicWidthInMbs;

            for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
                codingUnit *tmpMB = &img->mb_data[pos];
                tmpMB->md_directskip_mode = 0;
            }
        }
    }



    if (IS_DIRECT(currMB)) {
        dP = & (currSlice->partArr[0]);
        currSE.reading = read_b_dir_skip;
        dP->readSyntaxElement(&currSE, dP, currMB,
                              uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position

        md_directskip_mode = currSE.value1;

#if TRACE
        fprintf(hc->p_trace, "md_directskip_mode = %3d \n", currSE.value1);
#endif

        for (i = 0; i < num_of_orgMB_in_row; i++) {
            int pos = uiPositionInPic + i * img->PicWidthInMbs;

            for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
                codingUnit *tmpMB = &img->mb_data[pos];
                tmpMB->md_directskip_mode = md_directskip_mode;
            }
        }
    }
    if ((img->type == F_IMG) || (img->type == P_IMG)) {
        interpret_mb_mode_PF(uiPositionInPic, *pdir);
    } else if (img->type == I_IMG) {                            // intra frame
        interpret_mb_mode_I(uiPositionInPic);
    } else if ((img->type == B_IMG)) {     // B frame
        interpret_mb_mode_B(uiPositionInPic, *pdir);
    }

    //====== READ 8x8 SUB-PARTITION MODES (modes of 8x8 blocks) and Intra VBST block modes ======
    if (IS_P8x8(currMB)) {
        currSE.type    = SE_MBTYPE;
        if (img->type == B_IMG) {
            for (i = 0; i < 4; i++) {
                currSE.mapping = linfo_ue;
#if TRACE
                strncpy(currSE.tracestring, "8x8 mode", TRACESTRING_SIZE);
#endif
                //mb_part_type is fix length coding(fix length equal 2)!!   jlzheng    7.22
                assert(currStream->streamBuffer != NULL);
                currSE.len = 2;
                dP = & (currSlice->partArr[0]);
                currSE.reading = readB8TypeInfo;
                dP->readSyntaxElement(&currSE, dP, currMB,
                                      uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position

                SetB8Mode(currMB, currSE.value1, i);
            }
        } else if (img->type == F_IMG && hd->dhp_enabled && img->num_of_references > 1) {
            for (i = 0; i < 4; i++) {
                currSE.mapping = linfo_ue;
#if TRACE
                strncpy(currSE.tracestring, "8x8 mode", TRACESTRING_SIZE);
#endif
                //mb_part_type is fix length coding(fix length equal 2)!!   jlzheng    7.22
                assert(currStream->streamBuffer != NULL);
                currSE.len = 2;
                dP = & (currSlice->partArr[0]);
                currSE.reading = readB8TypeInfo_dhp;
                dP->readSyntaxElement(&currSE, dP, currMB,
                                      uiPositionInPic);  //check: first_mb_nr - LCU position, need current CU's position

                SetB8Mode(currMB, currSE.value1, i);
            }
        } else {
            currSE.value1 = 0;
            for (i = 0; i < 4; i++) {
                SetB8Mode(currMB, currSE.value1, i);
            }
        }

        for (i = 0; i < num_of_orgMB_in_row; i++) {
            int pos = uiPositionInPic + i * img->PicWidthInMbs;

            for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
                codingUnit *tmpMB = &img->mb_data[pos];
                int k;

                for (k = 0; k < 4; k++) {
                    tmpMB->b8mode[k] = currMB->b8mode[k];
                    tmpMB->b8pdir[k] = currMB->b8pdir[k];
                }
            }
        }
    }
    //read currMB->trans_size
    if (IS_INTRA(currMB)) {
        currSE.value2 = (currMB->cuType == I8MB);
        dP = & (currSlice->partArr[0]);
        currSE.reading = readTrSize;
        dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);
#if TRACE
        fprintf(hc->p_trace, "Transform_Size = %3d \n", currSE.value1);
#endif
        currMB->trans_size = currSE.value1;
    }
    ///////////////////////////////////////////
    if (input->useSDIP && IS_INTRA(currMB)) {
        dP = & (currSlice->partArr[0]);
        currSE.reading = readcuTypeInfo_SDIP;
        dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);
        currMB->cuType = currSE.value1;
    }
    ///////////////////////////////////////
    for (i = 0; i < num_of_orgMB_in_row; i++) {
        int pos = uiPositionInPic + i * img->PicWidthInMbs;

        for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
            codingUnit *tmpMB = &img->mb_data[pos];
            int tmp_intra_blksize = currMB->cuType;

            tmpMB->trans_size = currMB->trans_size;
            tmpMB->cuType = ((currMB->trans_size == 0) && (tmp_intra_blksize == I8MB)) ? I16MB : currMB->cuType;
        }
    }
    //--- init codingUnit data ---
    if (img->type == B_IMG) {
        init_codingUnit_Bframe(uiPositionInPic);
    } else {
        init_codingUnit(uiPositionInPic);
    }

    if (IS_INTRA(currMB)) {
        if (currMB->cuType == I16MB) {
            read_ipred_block_modes(0, uiPositionInPic);
            read_ipred_block_modes(4, uiPositionInPic);
        } else {
            for (i = 0; i < (input->chroma_format == 1 ? 5 : 6); i++) {
                read_ipred_block_modes(i, uiPositionInPic);
            }
        }
    }

}

void read_CBP(codingUnit *currMB, unsigned int uiPositionInPic, int num_of_orgMB_in_row, int num_of_orgMB_in_col)
{
    SyntaxElement currSE;
    Slice *currSlice = img->currentSlice;
    DataPartition *dP;
    int fixqp = (hd->fixed_picture_qp || hd->fixed_slice_qp);
    int i, j;

#if TRACE
    snprintf(currSE.tracestring, TRACESTRING_SIZE, "CBP");
#endif

    dP = & (currSlice->partArr[0]);
    currSE.reading = readCBP;
    dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);    //Check: first_mb_nr
    currMB->cbp  = currSE.value1;


#if MB_DQP

    // Delta quant only if nonzero coeffs
    if (input->useDQP) {
        if (currMB->cbp) {
#if TRACE
            snprintf(currSE.tracestring, TRACESTRING_SIZE, "Delta quant ");
#endif
            dP = & (currSlice->partArr[0]);
            currSE.reading = readDquant;
            dP->readSyntaxElement(&currSE,  dP, currMB, uiPositionInPic);    //check: first_mb_nr

            currMB->delta_quant = currSE.value1;
#if Check_Bitstream
			assert(currMB->delta_quant>=(-32 - 4 * (input->sample_bit_depth - 8))&&currMB->delta_quant<=(32 + 4 * (input->sample_bit_depth - 8)));
#endif
        }
#if LEFT_PREDICTION
        currMB->qp = currMB->delta_quant + currMB->left_cu_qp;
#else
        currMB->qp = currMB->delta_quant + hd->lastQP;
#endif

        //currMB->previouse_qp = lastQP;
        hd->lastQP = currMB->qp;
    } else {
        currMB->delta_quant = 0;
        img->qp = img->qp + currMB->delta_quant;
        assert(img->qp >= 0);
        assert(img->qp <= MAX_QP + (input->sample_bit_depth - 8) * 8);
        currMB->qp = img->qp;
    }

#endif



    for (i = 0; i < num_of_orgMB_in_row; i++) {
        int pos = uiPositionInPic + i * img->PicWidthInMbs;

        for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
            codingUnit *tmpMB = &img->mb_data[pos];
            tmpMB->cbp = currMB->cbp;
#if MB_DQP

            tmpMB->delta_quant = currMB->delta_quant;
            tmpMB->qp = currMB->qp;
            tmpMB->left_cu_qp = currMB->left_cu_qp;
            tmpMB->previouse_qp = currMB->previouse_qp;

#else
            tmpMB->delta_quant = 0;
#endif
            tmpMB->trans_size = currMB->trans_size;
        }
    }
}

void fill_B_Skip_block(codingUnit *currMB, unsigned int uiPositionInPic, int num_of_orgMB_in_row,
                       int num_of_orgMB_in_col, int N8_SizeScale)
{
    int i, j;

    for (i = 0; i < num_of_orgMB_in_row; i++) {
        int pos = uiPositionInPic + i * img->PicWidthInMbs;

        for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
            codingUnit *tmpMB = &img->mb_data[pos];
            tmpMB->cbp = 0;
        }
    }

    for (i = 0; i < 4; i++) {
        if ((currMB->b8mode[i] != IBLOCK) && (currMB->b8mode[i] == 0)) {
            int i8 = ((uiPositionInPic % img->PicWidthInMbs) << 1) + (i % 2) * N8_SizeScale;
            int j8 = ((uiPositionInPic / img->PicWidthInMbs) << 1) + (i / 2) * N8_SizeScale;

            int r, c, hv;


            // store 2D index of block 0
            int i8_1st;
            int j8_1st;

            if (i == 0) {
                i8_1st = i8;
                j8_1st = j8;
            }

            //===== DIRECT PREDICTION =====
            if (currMB->md_directskip_mode == 0) {
                // 时域参考帧对应位置块的参考帧为“视间参考”，需要特殊处理
                if (fref[0]->refbuf[j8][i8] == -1) {
                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            for (hv = 0; hv < 2; hv++) {
                                img->fw_mv[j8 + r][i8 + c][hv] = img->bw_mv[j8 + r][i8 + c][hv] = 0;
                            }
                        }
                    }

                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            img->fw_refFrArr[j8 + r][i8 + c] = 0;
                            img->bw_refFrArr[j8 + r][i8 + c] = 0;
                        }
                    }

                    // only MV of block 0 is calculated, block 1/2/3 will copy the MV from block 0
                    if (i == 0 || currMB->ui_MbBitSize != MIN_CU_SIZE_IN_BIT) {

                        SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->fw_mv[j8][i8][0])
                                                 , img->fw_refFrArr, img->fw_mv, 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale, 0, 1);
                        SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->bw_mv[j8][i8][0])
                                                 , img->bw_refFrArr, img->bw_mv, 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale, -1, 1);

                    } else {
                        img->fw_mv[j8][i8][0] = img->fw_mv[j8_1st][i8_1st][0];
                        img->fw_mv[j8][i8][1] = img->fw_mv[j8_1st][i8_1st][1];

                        img->bw_mv[j8][i8][0] = img->bw_mv[j8_1st][i8_1st][0];
                        img->bw_mv[j8][i8][1] = img->bw_mv[j8_1st][i8_1st][1];
                    }


                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            for (hv = 0; hv < 2; hv++) {
                                img->fw_mv[j8 + r][i8 + c][hv] = img->fw_mv[j8][i8][hv]; // need to fix
                                img->bw_mv[j8 + r][i8 + c][hv] = img->bw_mv[j8][i8][hv];
                            }
                        }
                    }
                } else { // next P is skip or inter mode
                    int iTRp, iTRb, iTRp1, iTRd, scale_refframe;
                    int refframe = fref[0]->refbuf[j8][i8];
                    int frame_no_next_P = 2 * img->imgtr_next_P;

                    int frame_no_B = 2 * img->pic_distance;
#if FIX_MAX_REF
                    int delta_P[MAXREF];
                    for (j = 0; j < MAXREF; j++) {
#else
                    int delta_P[4];
                    for (j = 0; j < 4; j++) {
#endif
                        delta_P[j] = 2 * (img->imgtr_next_P - fref[0]->ref_poc[j]);
                        delta_P[j] = (delta_P[j] + 512) % 512;
                    }
                    iTRp = (refframe + 1) * delta_P[0];
                    iTRb = iTRp - (frame_no_next_P - frame_no_B);

                    scale_refframe = 0;

                    iTRp = delta_P[refframe];
                    iTRp1 = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);


                    iTRd = frame_no_next_P - frame_no_B;
                    iTRb = iTRp1 - (frame_no_next_P - frame_no_B);

                    iTRp  = (iTRp  + 512) % 512;
                    iTRp1 = (iTRp1 + 512) % 512;
                    iTRd  = (iTRd  + 512) % 512;
                    iTRb  = (iTRb  + 512) % 512;


                    // only MV of block 0 is calculated, block 1/2/3 will copy the MV from block 0
                    if (i == 0 || currMB->ui_MbBitSize != MIN_CU_SIZE_IN_BIT) {
#if MV_SCALE
                        scale_mv_direct_x(fref[0]->mvbuf[j8][i8][0], iTRp, iTRb, iTRd, &img->fw_mv[j8][i8][0], &img->bw_mv[j8][i8][0]);
                        scale_mv_direct_y(fref[0]->mvbuf[j8][i8][1], frame_no_next_P, iTRp, frame_no_B, iTRb, iTRd,
                                          &img->fw_mv[j8][i8][1], &img->bw_mv[j8][i8][1]);
#else
                        if (fref[0]->mvbuf[j8][i8][0] < 0) {
                            img->fw_mv[j8][i8][0] = - (((MULTI / iTRp) * (1 - iTRb * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET);
                            img->bw_mv[j8][i8][0] = ((MULTI / iTRp) * (1 - iTRd * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET;
                        } else {
                            img->fw_mv[j8][i8][0] = ((MULTI / iTRp) * (1 + iTRb * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET;
                            img->bw_mv[j8][i8][0] = -(((MULTI / iTRp) * (1 + iTRd * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET);
                        }

                        if (fref[0]->mvbuf[j8][i8][1] < 0) {
                            img->fw_mv[j8][i8][1] = -(((MULTI / iTRp) * (1 - iTRb * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET);
                            img->bw_mv[j8][i8][1] = ((MULTI / iTRp) * (1 - iTRd * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET;
                        } else {
                            img->fw_mv[j8][i8][1] = ((MULTI / iTRp) * (1 + iTRb * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET;
                            img->bw_mv[j8][i8][1] = -(((MULTI / iTRp) * (1 + iTRd * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET);
                        }
#if HALF_PIXEL_COMPENSATION_DIRECT
                        if (img->is_field_sequence) {
                            int delta, delta2, delta_d, delta2_d;
                            int oriPOC = frame_no_next_P;
                            int oriRefPOC = frame_no_next_P - iTRp;
                            int scaledPOC = frame_no_B;
                            int scaledRefPOC = frame_no_B - iTRb;
                            getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                            scaledRefPOC = frame_no_B - iTRd;
                            getDeltas(&delta_d, &delta2_d, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                            assert(delta == delta_d);

                            if (fref[0]->mvbuf[j8][i8][1] + delta < 0) {
                                img->fw_mv[j8][i8][1] = - (((MULTI / iTRp) * (1 - iTRb * (fref[0]->mvbuf[j8][i8][1] + delta)) - 1) >> OFFSET) -
                                                        delta2;
                                img->bw_mv[j8][i8][1] = (((MULTI / iTRp) * (1 - iTRd * (fref[0]->mvbuf[j8][i8][1] + delta_d)) - 1) >> OFFSET) -
                                                        delta2_d;
                            } else {
                                img->fw_mv[j8][i8][1] = (((MULTI / iTRp) * (1 + iTRb * (fref[0]->mvbuf[j8][i8][1] + delta)) - 1) >> OFFSET) - delta2;
                                img->bw_mv[j8][i8][1] = - (((MULTI / iTRp) * (1 + iTRd * (fref[0]->mvbuf[j8][i8][1] + delta_d)) - 1) >> OFFSET) -
                                                        delta2_d;
                            }
                        }
#endif
#endif
                    } else {
                        img->fw_mv[j8][i8][0] = img->fw_mv[j8_1st][i8_1st][0];
                        img->bw_mv[j8][i8][0] = img->bw_mv[j8_1st][i8_1st][0];

                        img->fw_mv[j8][i8][1] = img->fw_mv[j8_1st][i8_1st][1];
                        img->bw_mv[j8][i8][1] = img->bw_mv[j8_1st][i8_1st][1];
                    }

                    img->fw_mv[j8][i8][0] = Clip3(-32768, 32767, img->fw_mv[j8][i8][0]);
                    img->bw_mv[j8][i8][0] = Clip3(-32768, 32767, img->bw_mv[j8][i8][0]);
                    img->fw_mv[j8][i8][1] = Clip3(-32768, 32767, img->fw_mv[j8][i8][1]);
                    img->bw_mv[j8][i8][1] = Clip3(-32768, 32767, img->bw_mv[j8][i8][1]);

                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            if (r + c == 0) {
                                continue;
                            }

                            img->fw_mv[j8 + r][i8 + c][0] = img->fw_mv[j8][i8][0];
                            img->bw_mv[j8 + r][i8 + c][0] = img->bw_mv[j8][i8][0];
                            img->fw_mv[j8 + r][i8 + c][1] = img->fw_mv[j8][i8][1];
                            img->bw_mv[j8 + r][i8 + c][1] = img->bw_mv[j8][i8][1];
                        }
                    }

                    img->fw_refFrArr[j8][i8] = 0;
                    img->bw_refFrArr[j8][i8] = 0;

                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            img->bw_refFrArr[j8 + r][i8 + c] = 0; //img->bw_refFrArr[j8][i8];
                            img->fw_refFrArr[j8 + r][i8 + c] = 0; //img->fw_refFrArr[j8][i8];
                        }
                    }
                }
            }//end of md_directskip_mode == 0;
            else if (currMB->md_directskip_mode != 0) {
                for (r = 0; r < num_of_orgMB_in_row; r++) {
                    for (c = 0; c < num_of_orgMB_in_col; c++) {
                        for (hv = 0; hv < 2; hv++) {
                            img->fw_mv[j8 + r][i8 + c][hv] = img->bw_mv[j8 + r][i8 + c][hv] = 0;
                        }
                    }
                }

                for (r = 0; r < num_of_orgMB_in_row; r++) {
                    for (c = 0; c < num_of_orgMB_in_col; c++) {
                        img->fw_refFrArr[j8 + r][i8 + c] = 0;
                        img->bw_refFrArr[j8 + r][i8 + c] = 0;
                    }
                }

                for (r = 0; r < num_of_orgMB_in_row; r++) {
                    for (c = 0; c < num_of_orgMB_in_col; c++) {
                        switch (currMB->md_directskip_mode) {
                        case 0:
                        case DS_SYM:
                        case DS_BID:
                            img->fw_refFrArr[j8 + r][i8 + c] = 0;
                            img->bw_refFrArr[j8 + r][i8 + c] = 0;
                            break;
                        case DS_BACKWARD:
                            img->fw_refFrArr[j8 + r][i8 + c] = -1;
                            img->bw_refFrArr[j8 + r][i8 + c] = 0;
                            break;
                        case DS_FORWARD:
                            img->fw_refFrArr[j8 + r][i8 + c] = 0;
                            img->bw_refFrArr[j8 + r][i8 + c] = -1;
                            break;

                        }
                    }
                }

                SetSkipMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->fw_mv[j8][i8][0]),
                                             & (img->bw_mv[j8][i8][0]), 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale,
                                             currMB->md_directskip_mode);

                for (r = 0; r < num_of_orgMB_in_row; r++) {
                    for (c = 0; c < num_of_orgMB_in_col; c++) {
                        for (hv = 0; hv < 2; hv++) {
                            img->fw_mv[j8 + r][i8 + c][hv] = img->fw_mv[j8][i8][hv]; // need to fix
                            img->bw_mv[j8 + r][i8 + c][hv] = img->bw_mv[j8][i8][hv];
                        }
                    }
                }
            }
        }
    }
}


#if FIX_MAX_REF
static void get_p_reference_distances(int *delta_P)
{
    int k;

    for (k = 0; k < img->num_of_references; k++) {
        delta_P[k] = (2 * (hc->picture_distance - fref[k]->imgtr_fwRefDistance) + 512) % 512;
        if (k == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
            delta_P[k] = 1;
        }
    }
}
#endif


void fill_P_Skip_for_MH(codingUnit *currMB, unsigned int uiPositionInPic, int num_of_orgMB_in_row,
                        int num_of_orgMB_in_col)
{
    int ***tmp_mv         = img->tmp_mv;
    int mb_y = uiPositionInPic / img->PicWidthInMbs;
    int mb_x = uiPositionInPic % img->PicWidthInMbs;
    int block8_y = mb_y << 1;
    int block8_x = mb_x << 1;
    int i, j;
#if FIX_MAX_REF
    int delta[MAXREF];
#else
    int delta[4];
#endif

    int N8_SizeScale = 1 << (currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT);
    {
        if (currMB->md_directskip_mode == 0) {
            PskipMV_COL(currMB, uiPositionInPic, num_of_orgMB_in_row, num_of_orgMB_in_col,
                        num_of_orgMB_in_row << MIN_CU_SIZE_IN_BIT, num_of_orgMB_in_col << MIN_CU_SIZE_IN_BIT);

            for (i = 0; i < 2 * num_of_orgMB_in_row; i++) {
                for (j = 0; j < 2 * num_of_orgMB_in_col; j++) {
                    if (currMB->weighted_skipmode != 0) {
#if FIX_MAX_REF
                        get_p_reference_distances(delta);
#else
                        delta[0] = (2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance) + 512) % 512;
                        delta[1] = (2 * (hc->picture_distance - fref[1]->imgtr_fwRefDistance) + 512) % 512;
                        delta[2] = (2 * (hc->picture_distance - fref[2]->imgtr_fwRefDistance) + 512) % 512;
                        delta[3] = (2 * (hc->picture_distance - fref[3]->imgtr_fwRefDistance) + 512) % 512;
                        if (0 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                            delta[0] = 1;
                        }
                        if (1 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                            delta[1] = 1;
                        }
                        if (2 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                            delta[2] = 1;
                        }
                        if (3 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                            delta[3] = 1;
                        }
#endif
                        hc->p_snd_refFrArr[block8_y + j][block8_x + i] = currMB->weighted_skipmode;
#if MV_SCALE
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = scale_mv(img->tmp_mv[block8_y + j][block8_x + i][0],
                                delta[currMB->weighted_skipmode], delta[0]);
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = scale_mv(img->tmp_mv[block8_y + j][block8_x + i][1],
                                delta[currMB->weighted_skipmode], delta[0]);
#if HALF_PIXEL_COMPENSATION_M1
                        assert(hd->is_field_sequence == img->is_field_sequence);
                        if (img->is_field_sequence) {
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = scale_mv_y1(img->tmp_mv[block8_y + j][block8_x + i][1],
                                    delta[currMB->weighted_skipmode], delta[0]);
                        }
#endif
#else
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = (int)(delta[currMB->weighted_skipmode] * img->tmp_mv[block8_y +
                                j][block8_x + i][0] * (MULTI / delta[0]) + HALF_MULTI >> OFFSET);
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = (int)(delta[currMB->weighted_skipmode] * img->tmp_mv[block8_y +
                                j][block8_x + i][1] * (MULTI / delta[0]) + HALF_MULTI >> OFFSET);
#if HALF_PIXEL_COMPENSATION_M1
                        assert(hd->is_field_sequence == img->is_field_sequence);
                        if (img->is_field_sequence) {
                            int deltaT, delta2;
                            int oriPOC = 2 * hc->picture_distance;
                            int oriRefPOC = oriPOC - delta[0];
                            int scaledPOC = 2 * hc->picture_distance;
                            int scaledRefPOC = scaledPOC - delta[currMB->weighted_skipmode];
                            getDeltas(&deltaT, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = (int)((delta[currMB->weighted_skipmode] *
                                    (img->tmp_mv[block8_y + j][block8_x + i][1] + deltaT) * (16384 / delta[0]) + 8192 >> 14) - delta2);
                        }
#endif
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = Clip3(-32768, 32767,
                                img->p_snd_tmp_mv[block8_y + j][block8_x + i][0]);
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = Clip3(-32768, 32767,
                                img->p_snd_tmp_mv[block8_y + j][block8_x + i][1]);
#endif
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                    } else {
                        hc->p_snd_refFrArr[block8_y + j][block8_x + i] = -1;
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = 0;
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = 0;
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                    }
                }
            }
        } else {
            setPSkipMotionVector(currMB->ui_MbBitSize, uiPositionInPic, 0, 0, 0, (1 << currMB->ui_MbBitSize),
                                 (1 << currMB->ui_MbBitSize), 1);
            if (currMB->md_directskip_mode == BID_P_FST || currMB->md_directskip_mode == BID_P_SND) {
                for (i = 0; i < 2 * num_of_orgMB_in_row; i++) {
                    for (j = 0; j < 2 * num_of_orgMB_in_col; j++) {
                        hc->refFrArr[block8_y + j][block8_x + i] = img->tmp_pref_fst[currMB->md_directskip_mode];
                        hc->p_snd_refFrArr[block8_y + j][block8_x + i] = img->tmp_pref_snd[currMB->md_directskip_mode];
                        img->tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][0];
                        img->tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][1];
                        img->tmp_mv[block8_y + j][block8_x + i][3] = 0;
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][0];
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][1];
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                    }
                }

            } else if (currMB->md_directskip_mode == FW_P_FST || currMB->md_directskip_mode == FW_P_SND) {
                for (i = 0; i < 2 * num_of_orgMB_in_row; i++) {
                    for (j = 0; j < 2 * num_of_orgMB_in_col; j++) {
                        hc->refFrArr[block8_y + j][block8_x + i] = img->tmp_pref_fst[currMB->md_directskip_mode];
                        hc->p_snd_refFrArr[block8_y + j][block8_x + i] = -1;
                        img->tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][0];
                        img->tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][1];
                        img->tmp_mv[block8_y + j][block8_x + i][3] = 0;
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][0];
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][1];
                        img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                    }
                }
            }
        }
    }
}

void fill_blockMV(codingUnit *currMB, unsigned int uiPositionInPic, int num_of_orgMB_in_row, int num_of_orgMB_in_col,
                  int N8_SizeScale)
{
    int i, j;

    int mb_y = uiPositionInPic / img->PicWidthInMbs;
    int mb_x = uiPositionInPic % img->PicWidthInMbs;
    int block8_y = mb_y << 1;
    int block8_x = mb_x << 1;
#if FIX_MAX_REF
    int delta[MAXREF];
#else
    int delta[4];
#endif

    if (((img->type == F_IMG) || (img->type == P_IMG)) && currMB->b8pdir[0] == FORWARD && currMB->b8mode[0] == 0) {
        {
            if (currMB->md_directskip_mode == 0) {
                PskipMV_COL(currMB, uiPositionInPic, num_of_orgMB_in_row, num_of_orgMB_in_col,
                            num_of_orgMB_in_row << MIN_CU_SIZE_IN_BIT, num_of_orgMB_in_col << MIN_CU_SIZE_IN_BIT);
                for (i = 0; i < 2 * num_of_orgMB_in_row; i++) {
                    for (j = 0; j < 2 * num_of_orgMB_in_col; j++) {
                        if (currMB->weighted_skipmode != 0) {
#if FIX_MAX_REF
                            get_p_reference_distances(delta);
#else
                            delta[0] = (2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance) + 512) % 512;
                            delta[1] = (2 * (hc->picture_distance - fref[1]->imgtr_fwRefDistance) + 512) % 512;
                            delta[2] = (2 * (hc->picture_distance - fref[2]->imgtr_fwRefDistance) + 512) % 512;
                            delta[3] = (2 * (hc->picture_distance - fref[3]->imgtr_fwRefDistance) + 512) % 512;
                            if (0 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                                delta[0] = 1;
                            }
                            if (1 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                                delta[1] = 1;
                            }
                            if (2 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                                delta[2] = 1;
                            }
                            if (3 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                                delta[3] = 1;
                            }
#endif
                            hc->p_snd_refFrArr[block8_y + j][block8_x + i] = currMB->weighted_skipmode;
#if MV_SCALE
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = scale_mv(img->tmp_mv[block8_y + j][block8_x + i][0],
                                    delta[currMB->weighted_skipmode], delta[0]);
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = scale_mv(img->tmp_mv[block8_y + j][block8_x + i][1],
                                    delta[currMB->weighted_skipmode], delta[0]);
#if HALF_PIXEL_COMPENSATION_M1
                            assert(hd->is_field_sequence == img->is_field_sequence);
                            if (img->is_field_sequence) {
                                img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = scale_mv_y1(img->tmp_mv[block8_y + j][block8_x + i][1],
                                        delta[currMB->weighted_skipmode], delta[0]);
                            }
#endif
#else
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = (int)(delta[currMB->weighted_skipmode] * img->tmp_mv[block8_y +
                                    j][block8_x + i][0] * (MULTI / delta[0]) + HALF_MULTI >> OFFSET);
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = (int)(delta[currMB->weighted_skipmode] * img->tmp_mv[block8_y +
                                    j][block8_x + i][1] * (MULTI / delta[0]) + HALF_MULTI >> OFFSET);
#if HALF_PIXEL_COMPENSATION_M1
                            assert(hd->is_field_sequence == img->is_field_sequence);
                            if (img->is_field_sequence) {
                                int deltaT, delta2;
                                int oriPOC = 2 * hc->picture_distance;
                                int oriRefPOC = oriPOC - delta[0];
                                int scaledPOC = 2 * hc->picture_distance;
                                int scaledRefPOC = scaledPOC - delta[currMB->weighted_skipmode];
                                getDeltas(&deltaT, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                                img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = (int)(delta[currMB->weighted_skipmode] *
                                        (img->tmp_mv[block8_y + j][block8_x + i][1] + deltaT) * (16384 / delta[0]) + 8192 >> 14) - delta2;
                            }
#endif
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = Clip3(-32768, 32767,
                                    img->p_snd_tmp_mv[block8_y + j][block8_x + i][0]);
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = Clip3(-32768, 32767,
                                    img->p_snd_tmp_mv[block8_y + j][block8_x + i][1]);
#endif
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                        } else {
                            hc->p_snd_refFrArr[block8_y + j][block8_x + i] = -1;
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = 0;
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = 0;
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                        }
                    }
                }
            } else {
                setPSkipMotionVector(currMB->ui_MbBitSize, uiPositionInPic, 0, 0, 0, (1 << currMB->ui_MbBitSize),
                                     (1 << currMB->ui_MbBitSize), 1);
                if (currMB->md_directskip_mode == BID_P_FST || currMB->md_directskip_mode == BID_P_SND) {
                    for (i = 0; i < 2 * num_of_orgMB_in_row; i++) {
                        for (j = 0; j < 2 * num_of_orgMB_in_col; j++) {
                            hc->refFrArr[block8_y + j][block8_x + i] = img->tmp_pref_fst[currMB->md_directskip_mode];
                            hc->p_snd_refFrArr[block8_y + j][block8_x + i] = img->tmp_pref_snd[currMB->md_directskip_mode];
                            img->tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][0];
                            img->tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][1];
                            img->tmp_mv[block8_y + j][block8_x + i][3] = 0;
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][0];
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][1];
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                        }
                    }

                } else if (currMB->md_directskip_mode == FW_P_FST || currMB->md_directskip_mode == FW_P_SND) {
                    for (i = 0; i < 2 * num_of_orgMB_in_row; i++) {
                        for (j = 0; j < 2 * num_of_orgMB_in_col; j++) {
                            hc->refFrArr[block8_y + j][block8_x + i] = img->tmp_pref_fst[currMB->md_directskip_mode];
                            hc->p_snd_refFrArr[block8_y + j][block8_x + i] = -1;
                            img->tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][0];
                            img->tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][1];
                            img->tmp_mv[block8_y + j][block8_x + i][3] = 0;
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][0] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][0];
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][1] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][1];
                            img->p_snd_tmp_mv[block8_y + j][block8_x + i][3] = 0;
                        }
                    }
                }
            }
        }
    }

    if (img->type == B_IMG) {
        for (i = 0; i < 4; i++) {
            if ((currMB->b8mode[i] != IBLOCK) && (currMB->b8mode[i] == 0)) {
                int i8 = ((uiPositionInPic % img->PicWidthInMbs) << 1) + (i % 2) * N8_SizeScale;
                int j8 = ((uiPositionInPic / img->PicWidthInMbs) << 1) + (i / 2) * N8_SizeScale;
                int r, c, hv;


                int i8_1st;
                int j8_1st;

                if (i == 0) {
                    i8_1st = i8;
                    j8_1st = j8;
                }


                //===== DIRECT PREDICTION =====
                if (currMB->md_directskip_mode == 0) {
                    if (fref[0]->refbuf[j8][i8] == -1) {
                        for (r = 0; r < num_of_orgMB_in_row; r++) {
                            for (c = 0; c < num_of_orgMB_in_col; c++) {
                                for (hv = 0; hv < 2; hv++) {
                                    img->fw_mv[j8 + r][i8 + c][hv] = img->bw_mv[j8 + r][i8 + c][hv] = 0;
                                }
                            }
                        }

                        for (r = 0; r < num_of_orgMB_in_row; r++) {
                            for (c = 0; c < num_of_orgMB_in_col; c++) {
                                img->fw_refFrArr[j8 + r][i8 + c] = 0;
                                img->bw_refFrArr[j8 + r][i8 + c] = 0;
                            }
                        }


                        if (i == 0 || img->type != B_IMG || currMB->ui_MbBitSize != MIN_CU_SIZE_IN_BIT) {

                            SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->fw_mv[j8][i8][0])
                                                     , img->fw_refFrArr, img->fw_mv, 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale, 0, 1);
                            SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->bw_mv[j8][i8][0])
                                                     , img->bw_refFrArr, img->bw_mv, 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale, -1, 1);


                        } else {
                            img->fw_mv[j8][i8][0] = img->fw_mv[j8_1st][i8_1st][0];
                            img->fw_mv[j8][i8][1] = img->fw_mv[j8_1st][i8_1st][1];

                            img->bw_mv[j8][i8][0] = img->bw_mv[j8_1st][i8_1st][0];
                            img->bw_mv[j8][i8][1] = img->bw_mv[j8_1st][i8_1st][1];
                        }



                        for (r = 0; r < num_of_orgMB_in_row; r++) {
                            for (c = 0; c < num_of_orgMB_in_col; c++) {
                                for (hv = 0; hv < 2; hv++) {
                                    img->fw_mv[j8 + r][i8 + c][hv] = img->fw_mv[j8][i8][hv]; // need to fix
                                    img->bw_mv[j8 + r][i8 + c][hv] = img->bw_mv[j8][i8][hv];
                                }
                            }
                        }
                    } else { // next P is skip or inter mode
                        int iTRp, iTRb, iTRp1, iTRd, scale_refframe;
                        int refframe = fref[0]->refbuf[j8][i8];
                        int frame_no_next_P = 2 * img->imgtr_next_P;

                        int frame_no_B = 2 * img->pic_distance;
#if FIX_MAX_REF
                        int delta_P[MAXREF];
                        for (j = 0; j < MAXREF; j++) {
#else
                        int delta_P[4];
                        for (j = 0; j < 4; j++) {
#endif
                            delta_P[j] = 2 * (img->imgtr_next_P - fref[0]->ref_poc[j]);
                            delta_P[j] = (delta_P[j] + 512) % 512;
                        }
                        iTRp = (refframe + 1) * delta_P[0];
                        iTRb = iTRp - (frame_no_next_P - frame_no_B);

                        scale_refframe = 0;

                        iTRp = delta_P[refframe];
                        iTRp1 = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);

                        iTRd = frame_no_next_P - frame_no_B;
                        iTRb = iTRp1 - (frame_no_next_P - frame_no_B);

                        iTRp  = (iTRp  + 512) % 512;
                        iTRp1 = (iTRp1 + 512) % 512;
                        iTRd  = (iTRd  + 512) % 512;
                        iTRb  = (iTRb  + 512) % 512;

                        if (i == 0 || img->type != B_IMG || currMB->ui_MbBitSize != MIN_CU_SIZE_IN_BIT) {
#if MV_SCALE
                            scale_mv_direct_x(fref[0]->mvbuf[j8][i8][0], iTRp, iTRb, iTRd, &img->fw_mv[j8][i8][0], &img->bw_mv[j8][i8][0]);
                            scale_mv_direct_y(fref[0]->mvbuf[j8][i8][1], frame_no_next_P, iTRp, frame_no_B, iTRb, iTRd,
                                              &img->fw_mv[j8][i8][1],  &img->bw_mv[j8][i8][1]);
#else
                            if (fref[0]->mvbuf[j8][i8][0] < 0) {
                                img->fw_mv[j8][i8][0] = - (((MULTI / iTRp) * (1 - iTRb * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET);
                                img->bw_mv[j8][i8][0] = ((MULTI / iTRp) * (1 - iTRd * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET;
                            } else {
                                img->fw_mv[j8][i8][0] = ((MULTI / iTRp) * (1 + iTRb * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET;
                                img->bw_mv[j8][i8][0] = - (((MULTI / iTRp) * (1 + iTRd * fref[0]->mvbuf[j8][i8][0]) - 1) >> OFFSET);
                            }

                            if (fref[0]->mvbuf[j8][i8][1] < 0) {
                                img->fw_mv[j8][i8][1] = - (((MULTI / iTRp) * (1 - iTRb * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET);
                                img->bw_mv[j8][i8][1] = ((MULTI / iTRp) * (1 - iTRd * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET;
                            } else {
                                img->fw_mv[j8][i8][1] = ((MULTI / iTRp) * (1 + iTRb * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET;
                                img->bw_mv[j8][i8][1] = - (((MULTI / iTRp) * (1 + iTRd * fref[0]->mvbuf[j8][i8][1]) - 1) >> OFFSET);
                            }
#if HALF_PIXEL_COMPENSATION_DIRECT
                            //assert( img->is_field_sequence == 1 );
                            //assert(is_field_sequence == 1);
                            if (img->is_field_sequence) {
                                int delta, delta2, delta_d, delta2_d;
                                int oriPOC = frame_no_next_P;
                                int oriRefPOC = frame_no_next_P - iTRp;
                                int scaledPOC = frame_no_B;
                                int scaledRefPOC = frame_no_B - iTRb;
                                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                                scaledRefPOC = frame_no_B - iTRd;
                                getDeltas(&delta_d, &delta2_d, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                                assert(delta == delta_d);

                                if (fref[0]->mvbuf[j8][i8][1] + delta < 0) {
                                    img->fw_mv[j8][i8][1] = - (((MULTI / iTRp) * (1 - iTRb * (fref[0]->mvbuf[j8][i8][1] + delta)) - 1) >> OFFSET) -
                                                            delta2;
                                    img->bw_mv[j8][i8][1] = (((MULTI / iTRp) * (1 - iTRd * (fref[0]->mvbuf[j8][i8][1] + delta_d)) - 1) >> OFFSET) -
                                                            delta2_d;
                                } else {
                                    img->fw_mv[j8][i8][1] = (((MULTI / iTRp) * (1 + iTRb * (fref[0]->mvbuf[j8][i8][1] + delta)) - 1) >> OFFSET) - delta2;
                                    img->bw_mv[j8][i8][1] = - (((MULTI / iTRp) * (1 + iTRd * (fref[0]->mvbuf[j8][i8][1] + delta_d)) - 1) >> OFFSET) -
                                                            delta2_d;
                                }
                            }
#endif
#endif
                        } else {
                            img->fw_mv[j8][i8][0] = img->fw_mv[j8_1st][i8_1st][0];
                            img->bw_mv[j8][i8][0] = img->bw_mv[j8_1st][i8_1st][0];

                            img->fw_mv[j8][i8][1] = img->fw_mv[j8_1st][i8_1st][1];
                            img->bw_mv[j8][i8][1] = img->bw_mv[j8_1st][i8_1st][1];
                        }
                        img->fw_mv[j8][i8][0] = Clip3(-32768, 32767, img->fw_mv[j8][i8][0]);
                        img->bw_mv[j8][i8][0] = Clip3(-32768, 32767, img->bw_mv[j8][i8][0]);
                        img->fw_mv[j8][i8][1] = Clip3(-32768, 32767, img->fw_mv[j8][i8][1]);
                        img->bw_mv[j8][i8][1] = Clip3(-32768, 32767, img->bw_mv[j8][i8][1]);


                        for (r = 0; r < num_of_orgMB_in_row; r++) {
                            for (c = 0; c < num_of_orgMB_in_col; c++) {
                                if (r + c == 0) {
                                    continue;
                                }

                                img->fw_mv[j8 + r][i8 + c][0] = img->fw_mv[j8][i8][0];
                                img->bw_mv[j8 + r][i8 + c][0] = img->bw_mv[j8][i8][0];
                                img->fw_mv[j8 + r][i8 + c][1] = img->fw_mv[j8][i8][1];
                                img->bw_mv[j8 + r][i8 + c][1] = img->bw_mv[j8][i8][1];
                            }
                        }

                        img->fw_refFrArr[j8][i8] = 0;
                        img->bw_refFrArr[j8][i8] = 0;

                        for (r = 0; r < num_of_orgMB_in_row; r++) {
                            for (c = 0; c < num_of_orgMB_in_col; c++) {
                                img->bw_refFrArr[j8 + r][i8 + c] = 0; //img->bw_refFrArr[j8][i8];
                                img->fw_refFrArr[j8 + r][i8 + c] = 0; //img->fw_refFrArr[j8][i8];
                            }
                        }
                    }
                } else if (currMB->md_directskip_mode != 0) {
                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            for (hv = 0; hv < 2; hv++) {
                                img->fw_mv[j8 + c][i8 + r][hv] = img->bw_mv[j8 + c][i8 + r][hv] = 0;
                            }
                        }
                    }

                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            switch (currMB->md_directskip_mode) {
                            case 0:
                            case DS_SYM:
                            case DS_BID:
                                img->fw_refFrArr[j8 + r][i8 + c] = 0;
                                img->bw_refFrArr[j8 + r][i8 + c] = 0;
                                break;
                            case DS_BACKWARD:
                                img->fw_refFrArr[j8 + r][i8 + c] = -1;
                                img->bw_refFrArr[j8 + r][i8 + c] = 0;
                                break;
                            case DS_FORWARD:
                                img->fw_refFrArr[j8 + r][i8 + c] = 0;
                                img->bw_refFrArr[j8 + r][i8 + c] = -1;
                                break;
                            }
                        }
                    }

                    SetSkipMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->fw_mv[j8][i8][0]),
                                                 & (img->bw_mv[j8][i8][0]), 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale,
                                                 currMB->md_directskip_mode);

                    for (r = 0; r < num_of_orgMB_in_row; r++) {
                        for (c = 0; c < num_of_orgMB_in_col; c++) {
                            for (hv = 0; hv < 2; hv++) {
                                img->fw_mv[j8 + c][i8 + r][hv] = img->fw_mv[j8][i8][hv]; // need to fix
                                img->bw_mv[j8 + c][i8 + r][hv] = img->bw_mv[j8][i8][hv];
                            }
                        }
                    }
                }
            }
        }
    }
}

int read_SubMB(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    codingUnit *currMB = &img->mb_data[uiPositionInPic];//GB current_mb_nr];
    int N8_SizeScale = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    int mb_y = (uiPositionInPic / img->PicWidthInMbs);
    int mb_x = uiPositionInPic % img->PicWidthInMbs;

    int num_of_orgMB_in_row = N8_SizeScale;//4:1  5:2  6:4
    int num_of_orgMB_in_col = N8_SizeScale;
    int size = 1 << uiBitSize;
    int pix_x = (img->current_mb_nr % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (img->current_mb_nr / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pdir = 0;
    int real_cuType;

#if MB_DQP
	
    int i, j;
    int PositionInPic = uiPositionInPic;
    int mb_width;
    int pix_a;
    int remove_prediction;

    int    pix_x_InSMB = (mb_x % 4) << MIN_CU_SIZE_IN_BIT;
    int    pix_y_InSMB = (mb_y % 4) << MIN_CU_SIZE_IN_BIT;
    int    currMB_pix_height = 1 << uiBitSize;

#endif

    pix_x = (img->current_mb_nr % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    pix_y = (img->current_mb_nr / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

    if (pix_x + size >= img->width) {
        num_of_orgMB_in_col = min(N8_SizeScale, (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT);
    }

    if (pix_y + size >= img->height) {
        num_of_orgMB_in_row = min(N8_SizeScale, (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT);
    }

    Init_SubMB(uiBitSize, uiPositionInPic);

    //  currMB->cuType_2= 0;
    read_MBHeader(currMB, uiPositionInPic, size, num_of_orgMB_in_row, num_of_orgMB_in_col, &pdir, &real_cuType);

    //qyu 0822 delete skip_mode_flag

#if MB_DQP

    mb_width = img->width / MIN_CU_SIZE;
    pix_a   = (PositionInPic % mb_width) * MIN_CU_SIZE;
    if (pix_a >= MIN_CU_SIZE) {
        //currMB->qp = currMB->delta_quant + img->qp;
        remove_prediction = (currMB->slice_nr != img->mb_data[PositionInPic - 1].slice_nr);
        if (!remove_prediction) {
            currMB->left_cu_qp = currMB->mb_available_left->qp;  //mb_data[PositionInPic - 1].qp;
        } else {
            currMB->left_cu_qp = img->qp;
        }
    } else {
        currMB->left_cu_qp = img->qp;
    }

    if (currMB->cbp == 0) {

#if LEFT_PREDICTION
        currMB->delta_quant = 0;
        currMB->qp = currMB->left_cu_qp;
#else
        currMB->delta_quant = 0;
        currMB->qp = hd->lastQP;
#endif
        for (i = 0; i < num_of_orgMB_in_row; i++) {
            int pos = uiPositionInPic + i * img->PicWidthInMbs;

            for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
                codingUnit *tmpMB = &img->mb_data[pos];
                tmpMB->delta_quant = currMB->delta_quant;
                tmpMB->qp = currMB->qp;
                tmpMB->previouse_qp = currMB->previouse_qp;
                tmpMB->left_cu_qp = currMB->left_cu_qp;
            }
        }

    }

#endif



    if (img->type == B_IMG && real_cuType < 0) {
        fill_B_Skip_block(currMB, uiPositionInPic, num_of_orgMB_in_row, num_of_orgMB_in_col, N8_SizeScale);
        return DECODE_MB;
    }

    readReferenceIndex(currMB->ui_MbBitSize, uiPositionInPic);
    readMotionVector(currMB->ui_MbBitSize, uiPositionInPic);

    if (((img->type == F_IMG) || (img->type == P_IMG)) && real_cuType < 0) {
        fill_P_Skip_for_MH(currMB, uiPositionInPic, num_of_orgMB_in_row, num_of_orgMB_in_col);
        return DECODE_MB;
    }

    fill_blockMV(currMB, uiPositionInPic, num_of_orgMB_in_row, num_of_orgMB_in_col, N8_SizeScale);

    // read CBP if not new intra mode
    read_CBP(currMB, uiPositionInPic, num_of_orgMB_in_row, num_of_orgMB_in_col);

    // read CBP and Coeffs  ***************************************************************
    readBlockCoeffs(uiPositionInPic);    //check
    return DECODE_MB;
}

int decode_SMB(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int i, split_flag = 1;
    int N8_SizeScale = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);

    int pix_x_InPic_start = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_y_InPic_start = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_x_InPic_end = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);     //Liwr 0915
    int pix_y_InPic_end = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);     //Liwr 0915
    int iBoundary_start = (pix_x_InPic_start >= img->width) || (pix_y_InPic_start >= img->height);
    int iBoundary_end = (pix_x_InPic_end > img->width) || (pix_y_InPic_end > img->height);
    int pos_x, pos_y, pos_InPic;

    int tmp_pos;


    if (uiBitSize == MIN_CU_SIZE_IN_BIT) {   //Liwr 0915
        split_flag = 0;
    }

    if (uiBitSize > MIN_CU_SIZE_IN_BIT && !iBoundary_end) {
        split_flag = readSplitFlag(uiBitSize);
    }

    if (split_flag) {
        for (i = 0; i < 4; i++) {
            int uiBitSize_Next = uiBitSize - 1;
            int pos = uiPositionInPic + (i / 2) * img->PicWidthInMbs * (N8_SizeScale / 2) + (i % 2) * (N8_SizeScale / 2);
            //Liwr 0915
            pos_x = pix_x_InPic_start + (((i % 2) * N8_SizeScale / 2) << MIN_CU_SIZE_IN_BIT);
            pos_y = pix_y_InPic_start + (((i / 2) * N8_SizeScale / 2) << MIN_CU_SIZE_IN_BIT);
            pos_InPic = (pos_x >= img->width || pos_y >= img->height);

            if (pos_InPic) {
                continue;
            }

            decode_SMB(uiBitSize_Next, pos);
        }

        return DECODE_MB;
    } else {
        if (!iBoundary_start) {
            tmp_pos = img->current_mb_nr;
            img->mb_x = (img->current_mb_nr) % (img->width / MIN_CU_SIZE);
            img->mb_y = (img->current_mb_nr) / (img->width / MIN_CU_SIZE);

            /* Define vertical positions */
            img->block8_y = img->mb_y * BLOCK_MULTIPLE;    /* luma block position */
            /* Define horizontal positions */
            img->block8_x = img->mb_x * BLOCK_MULTIPLE;    /* luma block position */
            read_SubMB(uiBitSize, uiPositionInPic);
            img->current_mb_nr = uiPositionInPic;
            img->mb_x = (img->current_mb_nr) % (img->width / MIN_CU_SIZE);
            img->mb_y = (img->current_mb_nr) / (img->width / MIN_CU_SIZE);

            /* Define vertical positions */
            img->block8_y = img->mb_y * BLOCK_MULTIPLE;    /* luma block position */
            img->pix_y   = img->mb_y * MIN_CU_SIZE;   /* luma codingUnit position */
            img->pix_c_y = img->mb_y * MIN_CU_SIZE / 2; /* chroma codingUnit position */

            /* Define horizontal positions */
            img->block8_x = img->mb_x * BLOCK_MULTIPLE;    /* luma block position */
            img->pix_x   = img->mb_x * MIN_CU_SIZE;   /* luma pixel position */
            img->pix_c_x = img->mb_x * MIN_CU_SIZE / 2; /* chroma pixel position */
            decode_SubMB();
            img->current_mb_nr = tmp_pos;
        }
    }

    return DECODE_MB;
}


void read_ipred_block_modes(int b8, unsigned int uiPositionInPic)
{
    int bi, bj, dec, i, j;
    SyntaxElement currSE;
    codingUnit *currMB = img->mb_data + uiPositionInPic; //current_mb_nr
    int mostProbableIntraPredMode[2];
    int upIntraPredMode;
    int leftIntraPredMode;
    Slice *currSlice = img->currentSlice;
    DataPartition *dP;
    int N8_SizeScale = 1 << (currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT);
    int block8_y = (uiPositionInPic / img->PicWidthInMbs) << 1;
    int block8_x = (uiPositionInPic % img->PicWidthInMbs) << 1;

    currSE.type = SE_INTRAPREDMODE;
#if TRACE
    strncpy(currSE.tracestring, "Ipred Mode", TRACESTRING_SIZE);
#endif

    if (b8 < 4) {
        if (currMB->b8mode[b8] == IBLOCK) {
            dP = & (currSlice->partArr[0]);
            currSE.reading = readIntraPredMode;
            dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);
            if (currMB->cuType == InNxNMB) {
                bi = block8_x ;
                bj = block8_y + b8 * (1 << (currMB->ui_MbBitSize - 4)); ;
            } else if (currMB->cuType == INxnNMB) {
                bi = block8_x + b8 * (1 << (currMB->ui_MbBitSize - 4)); ;
                bj = block8_y  ;
            }
            //get from array and decode
            else {
                //get from array and decode
                bi = block8_x + (b8 & 1) * N8_SizeScale;
                bj = block8_y + (b8 / 2) * N8_SizeScale;
            }

            upIntraPredMode            = img->ipredmode[bj][bi + 1];
            leftIntraPredMode          = img->ipredmode[bj + 1][bi];

            upIntraPredMode  = (upIntraPredMode < 0) ? DC_PRED : upIntraPredMode ;
            leftIntraPredMode  = (leftIntraPredMode < 0) ? DC_PRED : leftIntraPredMode ;
            mostProbableIntraPredMode[0] = min(upIntraPredMode, leftIntraPredMode);
            mostProbableIntraPredMode[1] = max(upIntraPredMode, leftIntraPredMode);
            if (mostProbableIntraPredMode[0] == mostProbableIntraPredMode[1]) {
                mostProbableIntraPredMode[0] = DC_PRED;
                mostProbableIntraPredMode[1] = (mostProbableIntraPredMode[1] == DC_PRED) ? BI_PRED : mostProbableIntraPredMode[1];
            }
            dec = (currSE.value1 < 0) ? mostProbableIntraPredMode[currSE.value1 + 2] : currSE.value1 +
                  (currSE.value1 >= mostProbableIntraPredMode[0]) + (currSE.value1 + 1 >= mostProbableIntraPredMode[1]);

            //set
            if (currMB->trans_size == 0) {
                int k = 0;
                for (k = 0; k < 4; k++) {
                    bi = block8_x + (k & 1) * N8_SizeScale;
                    bj = block8_y + (k / 2) * N8_SizeScale;
                    for (i = 0; i < N8_SizeScale; i++) {
                        for (j = 0; j < N8_SizeScale; j++) {
                            img->ipredmode[1 + bj + j][1 + bi + i] = dec;
                        }
                    }
                }
            } else {
                if (currMB->cuType == InNxNMB) {

                    i = b8 * (1 << (currMB->ui_MbBitSize - 4));
                    for (j = 0; j < (1 << (currMB->ui_MbBitSize + 1 - MIN_CU_SIZE_IN_BIT)); j++) {
                        if (currMB->ui_MbBitSize == 4) {
                            img->ipredmode[1 + block8_y + i][1 + block8_x + j] = dec;
                        } else {
                            img->ipredmode[1 + block8_y + i][1 + block8_x + j] = dec;
                            img->ipredmode[1 + block8_y + i + 1][1 + block8_x + j] = dec;

                        }
                    }


                } else if (currMB->cuType == INxnNMB) {
                    i = b8 * (1 << (currMB->ui_MbBitSize - 4));
                    for (j = 0; j < (1 << (currMB->ui_MbBitSize + 1 - MIN_CU_SIZE_IN_BIT)); j++) {
                        if (currMB->ui_MbBitSize == 4) {
                            img->ipredmode[1 + block8_y + j][1 + block8_x + i] = dec;
                        } else {
                            img->ipredmode[1 + block8_y + j][1 + block8_x + i] = dec;
                            img->ipredmode[1 + block8_y + j][1 + block8_x + i + 1] = dec;

                        }
                    }
                } else {
                    for (i = 0; i < N8_SizeScale; i++) {
                        for (j = 0; j < N8_SizeScale; j++) {
                            img->ipredmode[1 + bj + j][1 + bi + i] = dec;
                        }
                    }

                }

            }
            currMB->intra_pred_modes[b8] = dec;
#if Check_Bitstream
			assert(currMB->intra_pred_modes[b8]>=0&&currMB->intra_pred_modes[b8]<=32);
#endif
        }
    } else if (b8 == 4 && currMB->b8mode[b8 - 3] == IBLOCK) {

        currSE.type = SE_INTRAPREDMODE;
#if TRACE
        strncpy(currSE.tracestring, "Chroma intra pred mode", TRACESTRING_SIZE);
#endif

        dP = & (currSlice->partArr[0]);
        currSE.reading = readCIPredMode;
        dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);

        for (i = 0; i < N8_SizeScale; i++) {
            int pos = uiPositionInPic + i * img->PicWidthInMbs;

            for (j = 0; j < N8_SizeScale; j++, pos++) {
                codingUnit *tmpMB = &img->mb_data[pos];
                tmpMB->c_ipred_mode = currSE.value1;
            }
        }

        if (currSE.value1 < 0 || currSE.value1 >= NUM_INTRA_PMODE_CHROMA) {
            printf("%d\n", uiPositionInPic);
            error("illegal chroma intra pred mode!\n", 600);
        }
    }
}

/*
*************************************************************************
* Function:Set context for reference frames
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int BType2CtxRef(int btype)
{
    if (btype < 4) {
        return 0;
    } else {
        return 1;
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
void readReferenceIndex(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int k;
    codingUnit *currMB  = &img->mb_data[uiPositionInPic];
    SyntaxElement currSE;
    int bframe          = (img->type == B_IMG);
    int partmode        = currMB->cuType;
    int step_h0         = BLOCK_STEP [partmode][0];
    int step_v0         = BLOCK_STEP [partmode][1];

    int i0, j0, refframe;

    Slice *currSlice    = img->currentSlice;
    DataPartition *dP;

    int r, c;
    int N8_SizeScale = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    int block8_y = (uiPositionInPic / img->PicWidthInMbs) << 1;
    int block8_x = (uiPositionInPic % img->PicWidthInMbs) << 1;

    //  If multiple ref. frames, read reference frame for the MB *********************************
    if (img->typeb == BP_IMG)
    { return; }


    currSE.type = SE_REFFRAME;
    currSE.mapping = linfo_ue;

    for (j0 = 0; j0 < 2;) {
        if ((currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) &&
            j0 == 0) {
            j0 += 1;
            continue;
        }

        for (i0 = 0; i0 < 2;) {
            int b8_x, b8_y;

            k = 2 * j0 + i0;
            if ((currMB->b8pdir[k] == FORWARD || currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID ||
                 currMB->b8pdir[k] == DUAL) && currMB->b8mode[k] != 0) {   //non skip (direct)
                int start_x, start_y, tmp_step_h, tmp_step_v;
                get_b8_offset(currMB->cuType, uiBitSize, i0, j0, &start_x, &start_y, &tmp_step_h, &tmp_step_v);
                img->subblock_x = start_x;
                img->subblock_y = start_y;

                if (img->num_of_references > 1 && ((img->type == F_IMG) || (img->type == P_IMG)) && img->types != I_IMG)

                {
#if TRACE
                    strncpy(currSE.tracestring,  "Fwd ref frame no ", TRACESTRING_SIZE);
#endif
                    currSE.context = BType2CtxRef(currMB->b8mode[k]);

                    currSE.len = 1;

                    currSE.type = SE_REFFRAME;
                    dP = & (currSlice->partArr[0]);
                    currSE.reading = readRefFrame;
                    currSE.value2 = 0;
                    dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);
#if TRACE
                    fprintf(hc->p_trace, "Fwd Ref frame no  = %3d \n", currSE.value1);
#endif
                    refframe = currSE.value1;
                } else {
                    refframe = 0;
                }

                if (!bframe) {
                    b8_x = block8_x + start_x;
                    b8_y = block8_y + start_y;
                    for (r = 0; r < step_v0 * tmp_step_v; r++) {
                        for (c = 0; c < step_h0 * tmp_step_h; c++) {
                            hc->refFrArr[b8_y + r][b8_x + c] = refframe;
                            hc->p_snd_refFrArr[b8_y + r][b8_x + c] = -1;
                        }
                    }
                } else { // !! for B frame shenyanfei
                    b8_x = block8_x + start_x;
                    b8_y = block8_y + start_y;
                    for (r = 0; r < step_v0 * tmp_step_v; r++) {
                        for (c = 0; c < step_h0 * tmp_step_h; c++) {
                            img->fw_refFrArr[b8_y + r][b8_x + c] = refframe;
                        }
                    }
                }
            }

            i0 += max(1, step_h0);
        }

        j0 += max(1, step_v0);
    }

    for (j0 = 0; j0 < 2;) {
        if ((currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) &&
            j0 == 0) {
            j0 += 1;
            continue;
        }

        for (i0 = 0; i0 < 2;) {
            int b8_x, b8_y;
            k = 2 * j0 + i0;

            if (currMB->b8pdir[k] == DUAL && currMB->b8mode[k] != 0) {   //non skip (direct)
                int start_x, start_y, tmp_step_h, tmp_step_v;
                get_b8_offset(currMB->cuType, uiBitSize, i0, j0, &start_x, &start_y, &tmp_step_h, &tmp_step_v);
                img->subblock_x = start_x;
                img->subblock_y = start_y;

                if (img->num_of_references > 1 && currMB->b8pdir[k] != DUAL && hd->dhp_enabled && img->type == F_IMG &&
                    img->types != I_IMG) {
#if TRACE
                    strncpy(currSE.tracestring,  "Fwd snd ref frame no ", TRACESTRING_SIZE);
#endif
                    currSE.context = BType2CtxRef(currMB->b8mode[k]);

                    currSE.len = 1;

                    currSE.type = SE_REFFRAME;
                    dP = & (currSlice->partArr[0]);
                    currSE.reading = readRefFrame;
                    currSE.value2 = 1;
                    dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);
#if TRACE
                    fprintf(hc->p_trace, "Fwd Snd Ref frame no  = %3d \n", currSE.value1);
#endif
                    refframe = currSE.value1;
                } else {
                    refframe = 0;
                }
                b8_x = block8_x + start_x;
                b8_y = block8_y + start_y;
                for (r = 0; r < step_v0 * tmp_step_v; r++) {
                    for (c = 0; c < step_h0 * tmp_step_h; c++) {
                        hc->p_snd_refFrArr[b8_y + r][b8_x + c] = hc->refFrArr[b8_y + r][b8_x + c] == 0 ? 1 : 0;
                    }
                }
            }

            if ((currMB->b8pdir[k] == BACKWARD || currMB->b8pdir[k] == BID) &&
                currMB->b8mode[k] != 0) {  // non skip(direct) - backward
                int start_x, start_y, tmp_step_h, tmp_step_v;
                get_b8_offset(currMB->cuType, uiBitSize, i0, j0, &start_x, &start_y, &tmp_step_h, &tmp_step_v);
                img->subblock_x = start_x;
                img->subblock_y = start_y;

                refframe = 0;
                b8_x = block8_x + start_x;
                b8_y = block8_y + start_y;
                for (r = 0; r < step_v0 * tmp_step_v; r++) {
                    for (c = 0; c < step_h0 * tmp_step_h; c++) {
                        img->bw_refFrArr[b8_y + r][b8_x + c] = refframe;
                    }
                }
            }

            i0 += max(1, step_h0);
        }

        j0 += max(1, step_v0);
    }
}

static __inline int pmvr_sign(int val)
{
    if (val > 0) {
        return 1;
    } else if (val < 0) {
        return -1;
    } else {
        return 0;
    }
}
void pmvr_mv_derivation(int mv[2], int mvd[2], int mvp[2])
{
    int ctrd[2];
    ctrd[0] = ((mvp[0] >> 1) << 1) - mvp[0];
    ctrd[1] = ((mvp[1] >> 1) << 1) - mvp[1];
    if (abs(mvd[0] - ctrd[0]) > TH) {
        mv[0] = mvp[0] + (mvd[0] << 1) - ctrd[0] - pmvr_sign(mvd[0] - ctrd[0]) * TH;
        mv[1] = mvp[1] + (mvd[1] << 1) + ctrd[1];
    } else if (abs(mvd[1] - ctrd[1]) > TH) {
        mv[0] = mvp[0] + (mvd[0] << 1) + ctrd[0];
        mv[1] = mvp[1] + (mvd[1] << 1) - ctrd[1] - pmvr_sign(mvd[1] - ctrd[1]) * TH;
    } else {
        mv[0] = mvd[0] + mvp[0];
        mv[1] = mvd[1] + mvp[1];
    }
    mv[0] = Clip3(-32768, 32767, mv[0]);
    mv[1] = Clip3(-32768, 32767, mv[1]);
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
void readMotionVector(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int i, j, k, l, m, n;
    int curr_mvd;
    int dmh_mode = 0;
    codingUnit *currMB  = &img->mb_data[uiPositionInPic];
    SyntaxElement currSE;
    int bframe          = (img->type == B_IMG);
    int partmode = currMB->cuType;
    int step_h0         = BLOCK_STEP [partmode][0];
    int step_v0         = BLOCK_STEP [partmode][1];
    Slice *currSlice    = img->currentSlice;
    DataPartition *dP;

    int i0, j0, refframe;
    int pmv[2];
    int j8, i8, ii, jj;
    int vec;

    int iTRb, iTRp, iTRd;
    int frame_no_next_P, frame_no_B;
#if FIX_MAX_REF
    int  delta_P[MAXREF];
#else
    int  delta_P[4];
#endif
    int ref;
    int fw_refframe;

    int scale_refframe, iTRp1;
    int block8_y = (uiPositionInPic / img->PicWidthInMbs) << 1;
    int block8_x = (uiPositionInPic % img->PicWidthInMbs) << 1;
    int N8_SizeScale = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);

    int num_of_orgMB_in_row = N8_SizeScale;//4:1  5:2  6:4
    int num_of_orgMB_in_col = N8_SizeScale;
    int size = 1 << currMB->ui_MbBitSize;
    int pix_x = (uiPositionInPic % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (uiPositionInPic / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int start_x, start_y, step_h, step_v;
    int pmvr_mvd[2];
    int pmvr_mv[2];

    if (pix_x + size >= img->width) {
        num_of_orgMB_in_col = min(N8_SizeScale, (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT);
    }

    if (pix_y + size >= img->height) {
        num_of_orgMB_in_row = min(N8_SizeScale, (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT);
    }

    if (bframe && IS_P8x8(currMB)) {
        for (i = 0; i < 4; i++) {
            if (currMB->b8mode[i] == 0) {
                int pos_x, pos_y;

                pos_x = (i % 2) * N8_SizeScale;
                pos_y = (i / 2) * N8_SizeScale;
                for (j = 0; j < num_of_orgMB_in_row; j++) {
                    for (k = 0; k < num_of_orgMB_in_col; k++) {
                        img->fw_refFrArr[block8_y + pos_y + j][block8_x + pos_x + k] = 0;
                        img->bw_refFrArr[block8_y + pos_y + j][block8_x + pos_x + k] = 0;
                    }
                }
            }
        }
    }

    //=====  READ FORWARD MOTION VECTORS =====
    currSE.type = SE_MVD;

    currSE.mapping = linfo_se;

    for (j0 = 0; j0 < 2;) {
        if ((currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) &&
            j0 == 0) { // for  by jhzheng [2004/08/02]
            j0 += 1;
            continue;
        }

        for (i0 = 0; i0 < 2;) {
            k = 2 * j0 + i0;
            if ((currMB->b8pdir[k] == FORWARD || currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID ||
                 currMB->b8pdir[k] == DUAL) && (currMB->b8mode[k] != 0)) {     //has forward vector
                get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &start_x, &start_y, &step_h, &step_v);

                j8 = block8_y + start_y ;
                i8 = block8_x + start_x ;
                if (!bframe) {
                    refframe = hc->refFrArr        [ j8 ][ i8 ];
                } else {
                    refframe = img->fw_refFrArr[ j8 ][ i8 ];
                }

                // first make mv-prediction
                if (!bframe) {
                    int pix_start_x, pix_start_y, pix_step_h, pix_step_v;
                    get_pix_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &pix_start_x, &pix_start_y, &pix_step_h, &pix_step_v);
                    pix_step_h = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 2 || currMB->cuType == 4 ||
                                  currMB->cuType == 5) ? pix_step_h * 2 : pix_step_h;
                    pix_step_v = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 3 || currMB->cuType == 6 ||
                                  currMB->cuType == 7) ? pix_step_v * 2 : pix_step_v;

                    SetMotionVectorPredictor(uiBitSize, uiPositionInPic, pmv, hc->refFrArr, img->tmp_mv, refframe, pix_start_x, pix_start_y,
                                             pix_step_h , pix_step_v, 0, 0);  //Lou 1016
                } else {
                    int pix_start_x, pix_start_y, pix_step_h, pix_step_v;
                    get_pix_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &pix_start_x, &pix_start_y, &pix_step_h, &pix_step_v);
                    pix_step_h = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 2 || currMB->cuType == 4 ||
                                  currMB->cuType == 5) ? pix_step_h * 2 : pix_step_h;
                    pix_step_v = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 3 || currMB->cuType == 6 ||
                                  currMB->cuType == 7) ? pix_step_v * 2 : pix_step_v;

                    SetMotionVectorPredictor(uiBitSize, uiPositionInPic, pmv, img->fw_refFrArr, img->fw_mv, refframe, pix_start_x,
                                             pix_start_y, pix_step_h , pix_step_v, 0, 0);  //Lou 1016
                }

                if (img->type == F_IMG && hd->b_dmh_enabled && currMB->b8pdir[0] == FORWARD && currMB->b8pdir[1] == FORWARD &&
                    currMB->b8pdir[2] == FORWARD && currMB->b8pdir[3] == FORWARD && img->typeb != BP_IMG)

                {
                    if (k == 0 && (!(uiBitSize == B8X8_IN_BIT && currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT))) {
                        dP = & (currSlice->partArr[0]);
                        currSE.type = SE_DMH;
                        currSE.reading = readDmhMode;
                        currSE.value2 = currMB->ui_MbBitSize; // only used for context determination
                        dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);
                        dmh_mode = currSE.value1;
#if TRACE
                        fprintf(hc->p_trace, "dmh_mode = %3d\n", dmh_mode);
#endif
                    }
                    for (ii = 0; ii < step_h0 * step_h; ii++) {
                        for (jj = 0; jj < step_v0 * step_v; jj++) {
                            if (!bframe) {
                                img->tmp_mv[j8 + jj][i8 + ii][3] = dmh_mode;
                            } else {
                                img->fw_mv[j8 + jj][i8 + ii][3] = dmh_mode;
                            }
                        }
                    }
                } else {
                    dmh_mode = 0;
                }
                if (!(img->typeb == BP_IMG)) {  //no mvd for S frame, just set it to 0

                    for (n = 0; n < 2; n++) {
#if TRACE
                        snprintf(currSE.tracestring, TRACESTRING_SIZE, "FMVD (pred %3d)", pmv[n]);
#endif
                        currSE.value2 = (!bframe ? n : 2 * n);   // identifies the component; only used for context determination

                        img->subblock_x = start_x;
                        img->subblock_y = start_y;

                        dP = & (currSlice->partArr[0]);
                        currSE.reading = readMVD;
                        currSE.value2 = (n << 1) + 0;   // identifies the component; only used for context determination
                        dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);

                        pmvr_mvd[n] = currSE.value1;
                    }
                }
                if (img->typeb == BP_IMG) {
                    pmvr_mvd[0] = 0;
                    pmvr_mvd[1] = 0;
                }


                if (hd->b_pmvr_enabled) {
                    pmvr_mv_derivation(pmvr_mv, pmvr_mvd, pmv);
                } else {
                    pmvr_mv[0] = pmvr_mvd[0] + pmv[0];
                    pmvr_mv[1] = pmvr_mvd[1] + pmv[1];
                    pmvr_mv[0] = Clip3(-32768, 32767, pmvr_mv[0]);
                    pmvr_mv[1] = Clip3(-32768, 32767, pmvr_mv[1]);
                }
                for (n = 0; n < 2; n++) {
                    vec = pmvr_mv[n];
                    curr_mvd = pmvr_mvd[n];

                    // need B support
                    if (!bframe) {
                        for (ii = 0; ii < step_h0 * step_h; ii++) {   //qyu 0903
                            for (jj = 0; jj < step_v0 * step_v; jj++) {
                                img->tmp_mv[j8 + jj][i8 + ii][n] = vec;
                                img->p_snd_tmp_mv[j8 + jj][i8 + ii][n] = 0;
                            }
                        }
                    } else {  // B frame
                        for (ii = 0; ii < step_h0 * step_h; ii++) {   //qyu 0903
                            for (jj = 0; jj < step_v0 * step_v; jj++) {
                                img->fw_mv[j8 + jj][i8 + ii][n] = vec;
                            }
                        }
                    }

                    /* store (oversampled) mvd */
                    for (jj = 0; jj < num_of_orgMB_in_row; jj++) {
                        int pos = uiPositionInPic + jj * img->PicWidthInMbs;

                        for (ii = 0; ii < num_of_orgMB_in_col; ii++, pos++) {
                            codingUnit *tmpMB = &img->mb_data[pos];

                            for (l = 0; l < step_v0; l++) {
                                for (m = 0; m < step_h0; m++) {
                                    tmpMB->mvd[0][j0 + l][i0 + m][n] =  curr_mvd;
                                }
                            }
                        }
                    }
                }
            } else if (currMB->b8mode[k = 2 * j0 + i0] == 0) {
                // direct mode
                for (j = j0; j < j0 + step_v0; j++) {
                    for (i = i0; i < i0 + step_h0; i++) {
                        j8 = block8_y + j * N8_SizeScale;
                        i8 = block8_x + i * N8_SizeScale; //qyu 0904
                        ref = fref[0]->refbuf[j8][i8];

                        if (ref == -1) {
                            for (l = 0; l < num_of_orgMB_in_row; l++) {
                                for (m = 0; m < num_of_orgMB_in_col; m++) {
                                    img->fw_refFrArr[block8_y + j * N8_SizeScale + l][block8_x + i * N8_SizeScale + m] = 0;
                                    img->bw_refFrArr[block8_y + j * N8_SizeScale + l][block8_x + i * N8_SizeScale + m] = 0;
                                }
                            }

                            j8 = block8_y + j * N8_SizeScale;
                            i8 = block8_x + i * N8_SizeScale;

                            for (ii = 0; ii < 2; ii++) {
                                img->fw_mv[j8][i8][ii] = 0;
                                img->bw_mv[j8][i8][ii] = 0;
                            }


                            SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->fw_mv[j8][i8][0]),
                                                     img->fw_refFrArr, img->fw_mv, 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale, 0, 1);
                            SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, & (img->bw_mv[j8][i8][0]),
                                                     img->bw_refFrArr, img->bw_mv, 0, 0, 0, MIN_CU_SIZE * N8_SizeScale, MIN_CU_SIZE * N8_SizeScale, -1, 1);

                            for (l = 0; l < num_of_orgMB_in_row; l++) {
                                for (m = 0; m < num_of_orgMB_in_col; m++) {
                                    for (ii = 0; ii < 2; ii++) {
                                        img->fw_mv[j8 + l][i8 + m][ii] = img->fw_mv[j8][i8][ii];
                                        img->bw_mv[j8 + l][i8 + m][ii] = img->bw_mv[j8][i8][ii];
                                    }
                                }
                            }
                        } else {
                            frame_no_next_P = 2 * img->imgtr_next_P;
                            frame_no_B = 2 * img->pic_distance;

#if FIX_MAX_REF
                            for (n = 0; n < MAXREF; n++) {
#else
                            for (n = 0; n < 4; n++) {
#endif
                                delta_P[n] = 2 * (img->imgtr_next_P - fref[0]->ref_poc[n]);
                                delta_P[n] = (delta_P[n] + 512) % 512;
                            }

                            scale_refframe = 0;

                            iTRp = delta_P[ref];
                            iTRp1 = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);

                            iTRd = frame_no_next_P - frame_no_B;
                            iTRb = iTRp1 - iTRd;

                            iTRp  = (iTRp  + 512) % 512;
                            iTRp1 = (iTRp1 + 512) % 512;
                            iTRd  = (iTRd  + 512) % 512;
                            iTRb  = (iTRb  + 512) % 512;
                            if (ref == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                                iTRp = 1;
                            }
                            if (0 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                                iTRp1 = 1;
                                iTRb = 1;
                            }
                            for (l = 0; l < num_of_orgMB_in_row; l++) {
                                for (m = 0; m < num_of_orgMB_in_col; m++) {
                                    img->fw_refFrArr[block8_y + j * N8_SizeScale + l][block8_x + i * N8_SizeScale + m] = 0;
                                    img->bw_refFrArr[block8_y + j * N8_SizeScale + l][block8_x + i * N8_SizeScale + m] = 0;
                                }
                            }

                            j8 = block8_y + j * N8_SizeScale;
                            i8 = block8_x + i * N8_SizeScale;

                            for (ii = 0; ii < 2; ii++) {
                                for (l = 0; l < num_of_orgMB_in_row; l++) {
                                    for (m = 0; m < num_of_orgMB_in_col; m++) {
#if MV_SCALE
#if FIELD_HORI_MV_NO_SCALE_FIX
										if(ii==0) {
											scale_mv_direct_x(fref[0]->mvbuf[block8_y + j * N8_SizeScale][block8_x + i * N8_SizeScale][ii], iTRp, iTRb, iTRd,
												&img->fw_mv[j8 + l][i8 + m][ii], &img->bw_mv[j8 + l][i8 + m][ii]);
										}
										else {
											scale_mv_direct_y(fref[0]->mvbuf[block8_y + j * N8_SizeScale][block8_x + i * N8_SizeScale][ii], frame_no_next_P, iTRp,
												frame_no_B, iTRb, iTRd,
												&img->fw_mv[j8 + l][i8 + m][ii], &img->bw_mv[j8 + l][i8 + m][ii]);
										}

#else


                                        scale_mv_direct_y(fref[0]->mvbuf[block8_y + j * N8_SizeScale][block8_x + i * N8_SizeScale][ii], frame_no_next_P, iTRp,
                                                          frame_no_B, iTRb, iTRd,
                                                          &img->fw_mv[j8 + l][i8 + m][ii], &img->bw_mv[j8 + l][i8 + m][ii]);
#endif
#else
                                        if (fref[0]->mvbuf[block8_y + j * N8_SizeScale][block8_x + i * N8_SizeScale][ii] < 0) {
                                            img->fw_mv[j8 + l][i8 + m][ii] =  - (((MULTI / iTRp) * (1 - iTRb * fref[0]->mvbuf[j8][i8][ii]) - 1) >> OFFSET);
                                            img->bw_mv[j8 + l][i8 + m][ii] = ((MULTI / iTRp) * (1 - iTRd * fref[0]->mvbuf[j8][i8][ii]) - 1) >> OFFSET;
                                        } else {
                                            img->fw_mv[j8 + l][i8 + m][ii] = ((MULTI / iTRp) * (1 + iTRb * fref[0]->mvbuf[j8][i8][ii]) - 1) >> OFFSET;
                                            img->bw_mv[j8 + l][i8 + m][ii] = - (((MULTI / iTRp) * (1 + iTRd * fref[0]->mvbuf[j8][i8][ii]) - 1) >> OFFSET);
                                        }
#if HALF_PIXEL_COMPENSATION_DIRECT
                                        if (img->is_field_sequence && ii == 1) {
                                            int delta, delta2, delta_d, delta2_d;
                                            int oriPOC = frame_no_next_P;
                                            int oriRefPOC = frame_no_next_P - iTRp;
                                            int scaledPOC = frame_no_B;
                                            int scaledRefPOC = frame_no_B - iTRb;
                                            getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                                            scaledRefPOC = frame_no_B - iTRd;
                                            getDeltas(&delta_d, &delta2_d, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                                            assert(delta == delta_d);

                                            if (fref[0]->mvbuf[block8_y + j * N8_SizeScale][block8_x + i * N8_SizeScale][ii] + delta < 0) {
                                                img->fw_mv[j8 + l][i8 + m][ii] =  - (((MULTI / iTRp) * (1 - iTRb * (fref[0]->mvbuf[j8][i8][ii] + delta)) - 1) >>
                                                                                     OFFSET) - delta2;
                                                img->bw_mv[j8 + l][i8 + m][ii] = (((MULTI / iTRp) * (1 - iTRd * (fref[0]->mvbuf[j8][i8][ii] + delta_d)) - 1) >>
                                                                                  OFFSET) - delta2_d;
                                            } else {
                                                img->fw_mv[j8 + l][i8 + m][ii] = (((MULTI / iTRp) * (1 + iTRb * (fref[0]->mvbuf[j8][i8][ii] + delta)) - 1) >> OFFSET)
                                                                                 - delta2;
                                                img->bw_mv[j8 + l][i8 + m][ii] = - (((MULTI / iTRp) * (1 + iTRd * (fref[0]->mvbuf[j8][i8][ii] + delta_d)) - 1) >>
                                                                                    OFFSET) - delta2_d;
                                            }
                                        }
#endif
#endif
                                        img->fw_mv[j8 + l][i8 + m][ii] = Clip3(-32768, 32767, img->fw_mv[j8 + l][i8 + m][ii]);
                                        img->bw_mv[j8 + l][i8 + m][ii] = Clip3(-32768, 32767, img->bw_mv[j8 + l][i8 + m][ii]);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            i0 += max(1, step_h0);
        }

        j0 += max(1, step_v0);
    }


    //=====  READ BACKWARD MOTION VECTORS =====
    currSE.type = SE_MVD;

    currSE.mapping = linfo_se;


    for (j0 = 0; j0 < 2;) {
        if ((currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) &&
            j0 == 0) { // for  by jhzheng [2004/08/02]
            j0 += 1;
            continue;
        }


        for (i0 = 0; i0 < 2;) {
            k = 2 * j0 + i0;
            if ((currMB->b8pdir[k] == BACKWARD || currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID ||
                 currMB->b8pdir[k] == DUAL) && (currMB->b8mode[k] != 0)) {     //has backward vector
                get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &start_x, &start_y, &step_h, &step_v);

                j8 = block8_y + start_y ;
                i8 = block8_x + start_x ;
                refframe = (bframe ? img->bw_refFrArr : hc->p_snd_refFrArr)[j8][i8];
                if (currMB->b8pdir[k] == SYM) {
                    fw_refframe = img->fw_refFrArr[ j8 ][ i8 ];
                }
                if (!bframe) {
                    int pix_start_x, pix_start_y, pix_step_h, pix_step_v;
                    get_pix_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &pix_start_x, &pix_start_y, &pix_step_h, &pix_step_v);
                    pix_step_h = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 2 || currMB->cuType == 4 ||
                                  currMB->cuType == 5) ? pix_step_h * 2 : pix_step_h;
                    pix_step_v = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 3 || currMB->cuType == 6 ||
                                  currMB->cuType == 7) ? pix_step_v * 2 : pix_step_v;

                    SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, pmv, hc->refFrArr, img->tmp_mv, refframe, pix_start_x,
                                             pix_start_y, pix_step_h , pix_step_v, 0, 0);  //?????
                } else {
                    int pix_start_x, pix_start_y, pix_step_h, pix_step_v;
                    get_pix_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &pix_start_x, &pix_start_y, &pix_step_h, &pix_step_v);
                    pix_step_h = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 2 || currMB->cuType == 4 ||
                                  currMB->cuType == 5) ? pix_step_h * 2 : pix_step_h;
                    pix_step_v = (currMB->cuType == 0 || currMB->cuType == 1 || currMB->cuType == 3 || currMB->cuType == 6 ||
                                  currMB->cuType == 7) ? pix_step_v * 2 : pix_step_v;

                    SetMotionVectorPredictor(currMB->ui_MbBitSize, uiPositionInPic, pmv,  img->bw_refFrArr, img->bw_mv, refframe,
                                             pix_start_x, pix_start_y, pix_step_h, pix_step_v, -1, 0);  //Lou 1016

                }

                for (k = 0; k < 2; k++) {
#if TRACE
                    snprintf(currSE.tracestring, TRACESTRING_SIZE, "BMVD (pred %3d)", pmv[k]);
#endif

                    currSE.value2   = 2 * k + 1; // identifies the component; only used for context determination

                    if (currMB->b8pdir[2 * j0 + i0] == SYM) {
                        int delta_P, iTRp, DistanceIndexFw, DistanceIndexBw, refframe, delta_PB;
                        refframe = fw_refframe;
                        delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
                        delta_P = (delta_P + 512) % 512;

                        iTRp = (refframe + 1) * delta_P;    //the lates backward reference

                        delta_PB = 2 * (img->pic_distance - fref[0]->imgtr_fwRefDistance);
                        delta_PB = (delta_PB + 512) % 512;
                        iTRp     = (iTRp + 512) % 512;

                        DistanceIndexFw = delta_PB;

                        DistanceIndexBw    = (iTRp - DistanceIndexFw + 512) % 512;
#if MV_SCALE
                        curr_mvd = -scale_mv(img->fw_mv[j8][i8][k], DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
                        if (img->is_field_sequence && k == 1) {
                            curr_mvd = -scale_mv_y2(img->fw_mv[j8][i8][k], DistanceIndexBw, DistanceIndexFw);
                        }
#endif
#else
                        curr_mvd = - ((img->fw_mv[j8][i8][k] * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
#if HALF_PIXEL_COMPENSATION_MVD
                        if (img->is_field_sequence && k == 1) {
                            int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                            oriPOC = 2 * hc->picture_distance;
                            oriRefPOC = oriPOC - DistanceIndexFw;
                            scaledPOC = 2 * hc->picture_distance;
                            scaledRefPOC = scaledPOC - DistanceIndexBw;
                            getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                            curr_mvd = - (((img->fw_mv[j8][i8][k] + delta) * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                                          OFFSET);
                            curr_mvd -= delta2;
                        }
#endif
#endif
                        pmvr_mv[k] = curr_mvd;
                        pmvr_mv[k] = Clip3(-32768, 32767, pmvr_mv[k]);
                        pmvr_mvd[k] = curr_mvd - pmv[k];
                        pmvr_mvd[k] = Clip3(-32768, 32767, pmvr_mvd[k]);
                    } else if (currMB->b8pdir[2 * j0 + i0] == DUAL) {
                        int DistanceIndexFw, DistanceIndexBw;
                        fw_refframe = hc->refFrArr[j8][i8];

                        DistanceIndexFw = 2 * (img->pic_distance - fref[fw_refframe]->imgtr_fwRefDistance);
                        DistanceIndexFw = (DistanceIndexFw + 512) % 512;
                        DistanceIndexBw = 2 * (img->pic_distance - fref[refframe]->imgtr_fwRefDistance);
                        DistanceIndexBw = (DistanceIndexBw + 512) % 512;
                        if (fw_refframe == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                            DistanceIndexFw = 1;
                        }
                        if (refframe == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
                            DistanceIndexBw = 1;
                        }

#if MV_SCALE
                        curr_mvd = scale_mv(img->tmp_mv[j8][i8][k], DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
                        if (img->is_field_sequence && k == 1) {
                            curr_mvd = scale_mv_y1(img->tmp_mv[j8][i8][k], DistanceIndexBw, DistanceIndexFw);
                        }
#endif
#else
                        curr_mvd = (img->tmp_mv[j8][i8][k] * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET ;
#if HALF_PIXEL_COMPENSATION_MVD
                        if (img->is_field_sequence && k == 1) {
                            int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                            oriPOC = 2 * hc->picture_distance;
                            oriRefPOC = oriPOC - DistanceIndexFw;
                            scaledPOC = 2 * hc->picture_distance;
                            scaledRefPOC = scaledPOC - DistanceIndexBw;
                            getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                            curr_mvd = ((img->tmp_mv[j8][i8][k] + delta) * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET
                                       ;
                            curr_mvd -= delta2;
                        }
#endif
#endif
                        pmvr_mv[k] = curr_mvd;
                        pmvr_mv[k] = Clip3(-32768, 32767, pmvr_mv[k]);
                        pmvr_mvd[k] = curr_mvd - pmv[k];
                        pmvr_mvd[k] = Clip3(-32768, 32767, pmvr_mvd[k]);
                    } else {
                        img->subblock_x = start_x;
                        img->subblock_y = start_y;
                        dP = & (currSlice->partArr[0]);
                        currSE.reading = readMVD;


                        currSE.value2   = (k << 1) + 1;   // identifies the component; only used for context determination

                        dP->readSyntaxElement(&currSE, dP, currMB, uiPositionInPic);

                        pmvr_mvd[k] = currSE.value1;
                        pmvr_mv[k] = pmvr_mvd[k] + pmv[k];
#if Mv_Clip
						pmvr_mv[k] =Clip3(-32768, 32767, pmvr_mv[k]);
#endif
                        pmvr_mvd[k] = Clip3(-32768, 32767, pmvr_mvd[k]);
                    }

                }
                if (hd->b_pmvr_enabled && currMB->b8pdir[2 * j0 + i0] != SYM && currMB->b8pdir[2 * j0 + i0] != DUAL) {
                    pmvr_mv_derivation(pmvr_mv, pmvr_mvd, pmv);
                }
                for (k = 0; k < 2; k++) {   //jcma
                    vec = pmvr_mv[k];
                    curr_mvd = pmvr_mvd[k];
                    for (ii = 0; ii < step_h0 * step_h; ii++) {   //qyu 0903
                        for (jj = 0; jj < step_v0 * step_v; jj++) {
                            (bframe ? img->bw_mv : img->p_snd_tmp_mv) [j8 + jj][i8 + ii][k] = vec;
                        }
                    }

                    // set all org MB in the current MB
                    for (jj = 0; jj < num_of_orgMB_in_row; jj++) {
                        int pos = uiPositionInPic + jj * img->PicWidthInMbs;

                        for (ii = 0; ii < num_of_orgMB_in_col; ii++, pos++) {
                            codingUnit *tmpMB = &img->mb_data[pos];

                            for (l = 0; l < step_v0; l++) {
                                for (m = 0; m < step_h0; m++) {
                                    tmpMB->mvd[1][j0 + l][i0 + m][k] =  curr_mvd;
                                }
                            }
                        }
                    }
                }
            }

            i0 += max(1, step_h0);
        }

        j0 += max(1, step_v0);

    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////
///read PMV Index
    /////////////////////////////////////////////////////////////////////////////////////////////////////
}


/*
*************************************************************************
* Function:Get coded block pattern and coefficients (run/level)
from the bitstream
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void readBlockCoeffs(unsigned int uiPosition)
{
    int i, j;
    codingUnit *currMB = &img->mb_data[uiPosition];
    int b8;

    int offset_x = ((uiPosition % img->PicWidthInMbs) - (img->current_mb_nr % img->PicWidthInMbs)) * MIN_CU_SIZE;
    int offset_y = ((uiPosition / img->PicWidthInMbs) - (img->current_mb_nr / img->PicWidthInMbs)) * MIN_CU_SIZE;
    int bit_size = currMB->ui_MbBitSize;

    int block_x, block_y;
    Slice *currSlice = img->currentSlice;
    int     level, run, coef_ctr, k , i0, j0, uv, val, qp;
    int     shift, QPI, sum;
    int     start_scan;
    SyntaxElement currSE;          //lzhang
    DataPartition *dP;

    int **SCAN;

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    int wqm_coef;
    int WQMSizeId, WQMSize, iStride;
    int wqm_shift;
#endif


    extern int DCT_CGFlag[CG_SIZE * CG_SIZE];
    extern int DCT_PairsInCG[CG_SIZE * CG_SIZE];
    extern int DCT_CGNum;
    int iCG = 0;
    int pairs = 0;
    extern int g_intraModeClassified[NUM_INTRA_PMODE];

    int iVer = 0, iHor = 0;
    int iStartX, iStartY;
    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->cuType == P2NXN || currMB->cuType == PHOR_UP ||
            currMB->cuType == PHOR_DOWN) && currMB->trans_size == 1) {
        iHor = 1;
    } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->cuType == PNX2N ||
               currMB->cuType == PVER_LEFT || currMB->cuType == PVER_RIGHT) && currMB->trans_size == 1) {
        iVer = 1;
    } else if (input->useSDIP && (currMB->trans_size == 1) && (currMB->cuType == InNxNMB)) { //add yuqh 20130825
        iHor = 1;
    } else if (input->useSDIP && (currMB->trans_size == 1) && (currMB->cuType == INxnNMB)) { //add yuqh 20130825
        iVer = 1;
    }
    DCT_Pairs = -1;

#if MB_DQP
#else
    currMB->qp = img->qp;
#endif

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    if (WeightQuantEnable) {
#if !WQ_MATRIX_FCD
        wqm_shift = (hd->pic_weight_quant_data_index == 1) ? 3 : 0;
#else
        wqm_shift = 2;
#endif

    }
#endif

    qp  = currMB->qp;
    // luma coefficients

    if (currMB->cuType == I16MB || currMB->trans_size == 0) {
#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
            SCAN = AVS_CG_SCAN8x8;
            WQMSizeId = 1;
        } else if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
            SCAN = AVS_CG_SCAN16x16;
            WQMSizeId = 2;
        } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
            SCAN = AVS_CG_SCAN32x32;
            WQMSizeId = 3;
        } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
            SCAN = AVS_CG_SCAN32x32;
            WQMSizeId = 3;
        }
#else
        if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
            SCAN = AVS_CG_SCAN8x8;
        } else if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
            SCAN = AVS_CG_SCAN16x16;
        } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
            SCAN = AVS_CG_SCAN32x32;
        } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
            SCAN = AVS_CG_SCAN32x32;
        }
#endif
        if (currMB->cbp & 0xF) {
            // === set offset in current codingUnit ===
            int coded_bitsize ;
            if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
                coded_bitsize = currMB->ui_MbBitSize - 1;
            } else {
                coded_bitsize = currMB->ui_MbBitSize;
            }

            start_scan = 0; // take all coeffs
            coef_ctr = start_scan - 1;
            level    = 1;
            iCG = 0;
            pairs = 0;
            for (k = start_scan; (k < (1 << coded_bitsize) * (1 << coded_bitsize) + 1) && (level != 0); k++) {
                //============ read =============
                currSE.context      = LUMA_8x8;
                currSE.type         = (IS_INTRA(currMB)) ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER;
#if TRACE
                fprintf(hc->p_trace, "  Luma8x8 sng");
#endif
                currMB->l_ipred_mode = currMB->intra_pred_modes[0];
                dP = & (currSlice->partArr[0]);
                currSE.reading = readRunLevelRef;
                dP->readSyntaxElement(&currSE, dP, currMB, uiPosition);
                level = currSE.value1;
                run   = currSE.value2;
#if TRACE
                fprintf(hc->p_trace, "(%2d) level =%3d run =%2d\n", k, level, run);
                fflush(hc->p_trace);
#endif

                //============ decode =============
                if (level != 0) {   /* leave if len=1 */
                    while (DCT_CGFlag[ DCT_CGNum - iCG - 1 ] == 0) {
                        coef_ctr += 16;
                        iCG ++;
                    }

                    pairs ++;
                    coef_ctr += run + 1;

                    i = SCAN[coef_ctr][0];
                    j = SCAN[coef_ctr][1];

                    if (currSE.type == SE_LUM_AC_INTRA && g_intraModeClassified[currMB->l_ipred_mode] == INTRA_PRED_HOR) {
                        SWAP(i, j);
                    }

                    shift = IQ_SHIFT[qp] + (input->sample_bit_depth + 1) + currMB->ui_MbBitSize - LIMIT_BIT + 1;
                    shift ++;
                    shift --;

//Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
                    if (WeightQuantEnable) {
                        WQMSize = 1 << (WQMSizeId + 2);
                        if ((WQMSizeId == 0) || (WQMSizeId == 1)) {
                            iStride = WQMSize;
                            wqm_coef = cur_wq_matrix[WQMSizeId][(j & (iStride - 1)) * iStride + (i & (iStride - 1))];
                        }
#if AWQ_LARGE_BLOCK_EXT_MAPPING
                        else if (WQMSizeId == 2) {
                            iStride = WQMSize >> 1;
                            wqm_coef = cur_wq_matrix[WQMSizeId][((j >> 1) & (iStride - 1)) * iStride + ((i >> 1) & (iStride - 1))];
                        } else if (WQMSizeId == 3) {
                            iStride = WQMSize >> 2;
                            wqm_coef = cur_wq_matrix[WQMSizeId][((j >> 2) & (iStride - 1)) * iStride + ((i >> 2) & (iStride - 1))];
                        }
#endif
                    }

#if QuantMatrixClipFix
                    level = Clip3(0 - (1 << 15), (1 << 15) - 1, level);
#endif

                    QPI   = IQ_TAB[qp];
                    val   = level;
                    if (WeightQuantEnable) {
#if AWQ_WEIGHTING
                        sum = ((((((long long int)val * wqm_coef) >> wqm_shift) * QPI) >> 4) + (long long int)(1 << (shift - 2))) >>
                              (shift - 1); // M2239, N1466
#else
                        sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
#endif
                    } else {
                        sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
                    }
#if QuantClip
                    sum = Clip3(-32768, 32767, sum);
#endif
#else
                    QPI   = IQ_TAB[qp];
                    val =   level;
                    sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
#endif

                    sum = Clip3(0 - (1 << 15), (1 << 15) - 1,  sum);

                    img->resiY[offset_y + j][offset_x + i] = sum;
                }
                if (pairs == DCT_PairsInCG[DCT_CGNum - iCG - 1]) {
                    coef_ctr |= 0xf;
                    pairs = 0;
                    iCG ++;
                }
            }
        }
    } else {
        if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->cuType < 9 && (iVer || iHor))) {
            if (iHor) {
                if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                    SCAN = AVS_SCAN4x16;
                } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
                    SCAN = AVS_SCAN8x32;
                } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
                    SCAN = AVS_SCAN8x32;
                    bit_size -= 1;
                }
            }
            if (iVer) {
                if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                    SCAN = AVS_SCAN16x4;
                } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
                    SCAN = AVS_SCAN32x8;
                } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
                    SCAN = AVS_SCAN32x8;
                    bit_size -= 1;
                }
            }
#if FREQUENCY_WEIGHTING_QUANTIZATION
            if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                WQMSizeId = 2;
            } else if ((currMB->ui_MbBitSize == B32X32_IN_BIT) || (currMB->ui_MbBitSize == B64X64_IN_BIT)) {
                WQMSizeId = 3;
            }
#endif
        } else if (input->useSDIP && (currMB->cuType > 11 && (iVer || iHor))) {   //inter MB
            if (iHor) {
                if (currMB->ui_MbBitSize  == B16X16_IN_BIT) {
                    SCAN = AVS_SCAN4x16;
                } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
                    SCAN = AVS_SCAN8x32;
                } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
                    SCAN = AVS_SCAN8x32;
                    bit_size -= 1;
                }
            }
            if (iVer) {
                if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                    SCAN = AVS_SCAN16x4;
                } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
                    SCAN = AVS_SCAN32x8;
                } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
                    SCAN = AVS_SCAN32x8;
                    bit_size -= 1;
                }
            }
#if FREQUENCY_WEIGHTING_QUANTIZATION
            if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                WQMSizeId = 2;
            } else if ((currMB->ui_MbBitSize == B32X32_IN_BIT) || (currMB->ui_MbBitSize == B64X64_IN_BIT)) {
                WQMSizeId = 3;
            }
#endif
        } else {
//Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
            if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
                SCAN = AVS_SCAN4x4;
                WQMSizeId = 0;
            } else if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                SCAN = AVS_CG_SCAN8x8;
                WQMSizeId = 1;
            } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
                SCAN = AVS_CG_SCAN16x16;
                WQMSizeId = 2;
            } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
                SCAN = AVS_CG_SCAN32x32;
                WQMSizeId = 3;
            }
#else //FREQUENCY_WEIGHTING_QUANTIZATION
            if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
                SCAN = AVS_SCAN4x4;
            } else if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                SCAN = AVS_CG_SCAN8x8;
            } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
                SCAN = AVS_CG_SCAN16x16;
            } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
                SCAN = AVS_CG_SCAN32x32;
            }
#endif //FREQUENCY_WEIGHTING_QUANTIZATION
        }

        for (block_y = 0; block_y < 4; block_y += 2) {   /* all modes */
            for (block_x = 0; block_x < 4; block_x += 2) {
                b8 = 2 * (block_y / 2) + block_x / 2;
                if (iHor == 1) {
                    iStartX = 0;
                    iStartY = b8 * (1 << (bit_size - 2));
                } else if (iVer == 1) {
                    iStartX = b8 * (1 << (bit_size - 2));
                    iStartY = 0;
                }
                if (currMB->cbp & (1 << b8)) {
                    start_scan = 0; // take all coeffs
                    coef_ctr = start_scan - 1;
                    level    = 1;
                    iCG = 0;
                    pairs = 0;

                    for (k = start_scan; (k < (1 << (bit_size - 1)) * (1 << (bit_size - 1)) + 1) && (level != 0); k++) {
                        //============ read =============
                        currSE.context      = LUMA_8x8;
                        currSE.type         = (IS_INTRA(currMB)) ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER;
#if TRACE
                        fprintf(hc->p_trace, "  Luma8x8 sng");
#endif
                        currMB->l_ipred_mode = currMB->intra_pred_modes[b8];
                        dP = & (currSlice->partArr[0]);
                        currSE.reading = readRunLevelRef;
                        dP->readSyntaxElement(&currSE, dP, currMB, uiPosition);
                        level = currSE.value1;
                        run   = currSE.value2;
#if TRACE
                        fprintf(hc->p_trace, "(%2d) level =%3d run =%2d\n", k, level, run);
                        fflush(hc->p_trace);
#endif

                        //============ decode =============
                        if (level != 0) {   /* leave if len=1 */
                            while (DCT_CGFlag[ DCT_CGNum - iCG - 1 ] == 0) {
                                coef_ctr += 16;
                                iCG ++;
                            }

                            pairs ++;
                            coef_ctr += run + 1;

                            i = SCAN[coef_ctr][0];
                            j = SCAN[coef_ctr][1];

                            if (currSE.type == SE_LUM_AC_INTRA && g_intraModeClassified[currMB->l_ipred_mode] == INTRA_PRED_HOR &&
                                currMB->cuType != InNxNMB && currMB->cuType != INxnNMB) {
                                SWAP(i, j);
                            }
                            if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (iHor || iVer) && currMB->ui_MbBitSize == 6) {
                                shift = IQ_SHIFT[qp] + (input->sample_bit_depth + 1) + (bit_size) - LIMIT_BIT + 1;
                            } else {
                                shift = IQ_SHIFT[qp] + (input->sample_bit_depth + 1) + (bit_size - 1) - LIMIT_BIT + 1;
                            }
                            shift ++;
                            shift --;

                            // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
                            if (WeightQuantEnable) {
                                wqm_coef = cur_wq_matrix[1][(j & 7) * 8 + (i & 7)];
                                WQMSize = 1 << (WQMSizeId + 2);
                                if ((WQMSizeId == 0) || (WQMSizeId == 1)) {
                                    iStride = WQMSize;
                                    wqm_coef = cur_wq_matrix[WQMSizeId][(j & (iStride - 1)) * iStride + (i & (iStride - 1))];
                                }
#if AWQ_LARGE_BLOCK_EXT_MAPPING
                                else if (WQMSizeId == 2) {
                                    iStride = WQMSize >> 1;
                                    wqm_coef = cur_wq_matrix[WQMSizeId][((j >> 1) & (iStride - 1)) * iStride + ((i >> 1) & (iStride - 1))];
                                } else if (WQMSizeId == 3) {
                                    iStride = WQMSize >> 2;
                                    wqm_coef = cur_wq_matrix[WQMSizeId][((j >> 2) & (iStride - 1)) * iStride + ((i >> 2) & (iStride - 1))];
                                }
#endif
                            }

#if QuantMatrixClipFix
                            level = Clip3(0 - (1 << 15), (1 << 15) - 1, level);
#endif

                            QPI   = IQ_TAB[qp];
                            val   = level;
                            if (WeightQuantEnable) {
#if AWQ_WEIGHTING
                                sum = ((((((long long int)val * wqm_coef) >> wqm_shift) * QPI) >> 4) + (long long int)(1 << (shift - 2))) >>
                                      (shift - 1); // M2239, N1466
#else
                                sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
#endif
                            } else {
                                sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
                            }
#if QuantClip
                            sum = Clip3(-32768, 32767, sum);
#endif
#else  // Adaptive frequency weighting quantization
                            QPI   = IQ_TAB[qp];
                            val =   level;
                            sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
#endif  // Adaptive frequency weighting quantization


                            sum = Clip3(0 - (1 << 15), (1 << 15) - 1,  sum);
                            if ((input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (iVer || iHor))) {
                                img->resiY[offset_y + iStartY + j][offset_x + iStartX + i] = sum;
                            } else if (input->useSDIP && (iVer || iHor)) {
                                img->resiY[offset_y + iStartY + j][offset_x + iStartX + i] = sum;
                            } else {
                                img->resiY[offset_y + ((b8 / 2) << (bit_size - 1)) + j][offset_x + ((b8 % 2) << (bit_size - 1)) + i] = sum;
                            }

                        }
                        if (pairs == DCT_PairsInCG[DCT_CGNum - iCG - 1]) {
                            coef_ctr |= 0xf;
                            pairs = 0;
                            iCG ++;
                        }
                    }
                }
            }
        }
    }

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
        SCAN = AVS_SCAN4x4;
        WQMSizeId = 0;
    } else if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
        SCAN = AVS_CG_SCAN8x8;
        WQMSizeId = 1;
    } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
        SCAN = AVS_CG_SCAN16x16;
        WQMSizeId = 2;
    } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
        SCAN = AVS_CG_SCAN32x32;
        WQMSizeId = 3;
    }
#else // Adaptive frequency weighting quantization
    if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
        SCAN = AVS_SCAN4x4;
    } else if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
        SCAN = AVS_CG_SCAN8x8;
    } else if (currMB->ui_MbBitSize == B32X32_IN_BIT) {
        SCAN = AVS_CG_SCAN16x16;
    } else if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
        SCAN = AVS_CG_SCAN32x32;
    }
#endif  // Adaptive frequency weighting quantization


    uv = -1;
    block_y = 4;

    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && ((iHor || iVer) &&
            (currMB->ui_MbBitSize == B64X64_IN_BIT))) {
        bit_size++;
    }

    for (block_x = 0; block_x < 4; block_x += 2) {
        uv++;
        if (input->sample_bit_depth > 8) {
            qp = currMB->qp - 8 * (input->sample_bit_depth - 8) + (uv == 0 ? hd->chroma_quant_param_delta_u :
                    hd->chroma_quant_param_delta_v);
#if Check_Bitstream
		    assert(qp>=-16&&qp<=63);
#endif
            qp = qp < 0 ? qp : QP_SCALE_CR[qp];

            qp = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), qp + 8 * (input->sample_bit_depth - 8));

        } else {
            qp = currMB->qp - 8 * (input->sample_bit_depth - 8) + (uv == 0 ? hd->chroma_quant_param_delta_u :
                    hd->chroma_quant_param_delta_v);
#if Check_Bitstream
		   assert(qp>=-16&&qp<=63);
#endif
            qp = qp < 0 ? qp : QP_SCALE_CR[qp];

            qp = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), qp + 8 * (input->sample_bit_depth - 8));

        }

        if ((currMB->cbp >> (uv + 4)) & 0x1) {
            iCG = 0;
            pairs = 0;
            coef_ctr = -1;
            level = 1;

            currSE.context     = CHROMA;
            currSE.type        = (IS_INTRA(currMB) ? SE_CHR_AC_INTRA : SE_CHR_AC_INTER);      // element is of type DC
            dP = & (currSlice->partArr[0]);
            currSE.reading = readRunLevelRef;

            for (k = 0; (k < ((1 << (bit_size - 1)) * (1 << (bit_size - 1)) + 1)) && (level != 0); k++) {
#if TRACE
                fprintf(hc->p_trace, "  AC chroma 8X8 ");
#endif
                dP->readSyntaxElement(&currSE, dP, currMB, uiPosition);
                level = currSE.value1;
                run   = currSE.value2;
#if TRACE
                fprintf(hc->p_trace, "%2d: level =%3d run =%2d\n", k, level, run);
                fflush(hc->p_trace);
#endif

                if (level != 0) {                   // leave if len=1
                    while (DCT_CGFlag[ DCT_CGNum - iCG - 1 ] == 0) {
                        coef_ctr += 16;
                        iCG ++;
                    }

                    pairs ++;
                    coef_ctr = coef_ctr + run + 1;

                    i0 = SCAN[coef_ctr][0];
                    j0 = SCAN[coef_ctr][1];
                    shift = IQ_SHIFT[qp] + (input->sample_bit_depth + 1) + (bit_size - 1) - LIMIT_BIT + 1;
                    shift ++;
                    shift --;

                    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
                    if (WeightQuantEnable) {
                        WQMSize = 1 << (WQMSizeId + 2);
                        if ((WQMSizeId == 0) || (WQMSizeId == 1)) {
                            iStride = WQMSize;
                            wqm_coef = cur_wq_matrix[WQMSizeId][(j0 & (iStride - 1)) * iStride + (i0 & (iStride - 1))];
                        }

#if AWQ_LARGE_BLOCK_EXT_MAPPING
                        else if (WQMSizeId == 2) {
                            iStride = WQMSize >> 1;
                            wqm_coef = cur_wq_matrix[WQMSizeId][((j0 >> 1) & (iStride - 1)) * iStride + ((i0 >> 1) & (iStride - 1))];
                        } else if (WQMSizeId == 3) {
                            iStride = WQMSize >> 2;
                            wqm_coef = cur_wq_matrix[WQMSizeId][((j0 >> 2) & (iStride - 1)) * iStride + ((i0 >> 2) & (iStride - 1))];
                        }
#endif
                    }

#if QuantMatrixClipFix
                    level = Clip3(0 - (1 << 15), (1 << 15) - 1, level);
#endif

                    QPI   = IQ_TAB[qp];
                    val   = level;
                    if (WeightQuantEnable) {
#if AWQ_WEIGHTING
                        //sum = ((((((int)val*wqm_coef)>>3)*QPI)>>4) + (1<<(shift-2)) )>>(shift-1);   // M2239, N1466
                        sum = ((((((long long int)val * wqm_coef) >> wqm_shift) * QPI) >> 4) + (long long int)(1 << (shift - 2))) >>
                              (shift - 1); // M2239, N1466
#else
                        sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
#endif
                    } else {
                        sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
                    }
#if QuantClip
                    sum = Clip3(-32768, 32767, sum);
#endif
#else // Adaptive frequency weighting quantization
                    QPI   = IQ_TAB[qp];
                    val   = level;
                    sum = (val * QPI + (1 << (shift - 2))) >> (shift - 1);
#endif // Adaptive frequency weighting quantization 


                    sum = Clip3(0 - (1 << 15), (1 << 15) - 1,  sum);
                    img->resiUV[uv][offset_y / 2 + j0][offset_x / 2 + i0] = sum;
                } //level!=0
                if (pairs == DCT_PairsInCG[DCT_CGNum - iCG - 1]) {
                    coef_ctr |= 0xf;
                    pairs = 0;
                    iCG ++;
                }
            } //end k=0~64
        }
    }
}

/*
*************************************************************************
* Function:decode one codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void decode_one_InterLumaBlock(int block8Nx8N)
{
    codingUnit *currMB   = &img->mb_data[img->current_mb_nr];//GB current_mb_nr];

    int tmp_block[MAX_CU_SIZE][MAX_CU_SIZE];
    int tmp_blockbw[MAX_CU_SIZE][MAX_CU_SIZE];
    int tmp_block_wgt[MAX_CU_SIZE][MAX_CU_SIZE];
    int i = 0, j = 0, ii = 0, jj = 0, j8 = 0, i8 = 0;

    int vec1_x = 0, vec1_y = 0, vec2_x = 0, vec2_y = 0;
    int ioff, joff;

    int mv_mul = 4;

    int refframe, fw_refframe, bw_refframe, mv_mode, pred_dir; // = currMB->ref_frame;
    int bw_ref_idx;
    int *** mv_array, * **fw_mv_array, * **bw_mv_array;
    int bframe = (img->type == B_IMG);

    int mb_nr             = img->current_mb_nr;//GBimg->current_mb_nr;

    int mb_BitSize = img->mb_data[mb_nr].ui_MbBitSize;
    int start_x, start_y, step_h, step_v;

    int vec_wgt_x, vec_wgt_y;
#if FIX_MAX_REF
    int delta[MAXREF];
#else
    int delta[4];
#endif

    int first_x, first_y, second_x, second_y;
    int dmh_mode = 0;
#if Mv_Rang
	int mv0_x,mv0_y;
	int mv1_x,mv1_y;
#endif

#if FIX_MAX_REF
    get_p_reference_distances(delta);
#else
    delta[0] = (2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance) + 512) % 512;
    delta[1] = (2 * (hc->picture_distance - fref[1]->imgtr_fwRefDistance) + 512) % 512;
    delta[2] = (2 * (hc->picture_distance - fref[2]->imgtr_fwRefDistance) + 512) % 512;
    delta[3] = (2 * (hc->picture_distance - fref[3]->imgtr_fwRefDistance) + 512) % 512;
    if (0 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        delta[0] = 1;
    }
    if (1 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        delta[1] = 1;
    }
    if (2 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        delta[2] = 1;
    }
    if (3 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        delta[3] = 1;
    }
#endif

    i = block8Nx8N % 2;
    j = block8Nx8N / 2;

    {
        get_b8_offset(currMB->cuType, mb_BitSize, i, j, &start_x, &start_y, &step_h, &step_v);
        i8 = img->block8_x + start_x;
        j8 = img->block8_y + start_y;
        get_pix_offset(currMB->cuType, mb_BitSize, i, j, &start_x, &start_y, &step_h, &step_v);
        ioff = start_x;
        joff = start_y;
    }


    mv_mode  = currMB->b8mode[block8Nx8N];
    pred_dir = currMB->b8pdir[block8Nx8N];

    if (pred_dir != SYM && pred_dir != BID) {
        //===== FORWARD/BACKWARD PREDICTION =====
        if (!bframe) {   // !! P MB shenyanfei
            refframe = hc->refFrArr[j8][i8];
            mv_array = img->tmp_mv;
        } else if (!pred_dir) {   // !! B forward shenyanfei
            refframe = img->fw_refFrArr[j8][i8];// fwd_ref_idx_to_refframe(img->fw_refFrArr[j8][i8]);
            mv_array = img->fw_mv;
        } else { // !! B backward shenyanfei
            refframe = img->bw_refFrArr[j8][i8];// bwd_ref_idx_to_refframe(img->bw_refFrArr[j8][i8]);
            bw_ref_idx = img->bw_refFrArr[j8][i8];
            mv_array = img->bw_mv;
        }
#if INTERLACE_CODING
        if (bframe)
#else
        if ((hd->progressive_sequence || hd->progressive_frame) && bframe && img->picture_structure)
#endif
        {
            refframe = 0;
        }

        vec1_x = (img->pix_x + ioff) * mv_mul + mv_array[j8][i8][0];
        vec1_y = (img->pix_y + joff) * mv_mul + mv_array[j8][i8][1];

        if (img->num_of_references > 1 && (!mv_mode) &&  img->type == F_IMG && currMB->weighted_skipmode) {
#if MV_SCALE
            vec_wgt_x = (img->pix_x + ioff) * mv_mul + scale_mv(mv_array[j8][i8][0], delta[currMB->weighted_skipmode ], delta[0]);
            vec_wgt_y = (img->pix_y + joff) * mv_mul + scale_mv(mv_array[j8][i8][1], delta[currMB->weighted_skipmode], delta[0]);
#if HALF_PIXEL_COMPENSATION_M1
            assert(hd->is_field_sequence == img->is_field_sequence);
            if (img->is_field_sequence) {
                vec_wgt_y = (img->pix_y + joff) * mv_mul + scale_mv_y1(mv_array[j8][i8][1], delta[currMB->weighted_skipmode], delta[0]);
            }
#endif
#else
            vec_wgt_x = (img->pix_x + ioff) * mv_mul + (int)(delta[currMB->weighted_skipmode ] * mv_array[j8][i8][0] *
                        (MULTI / delta[0]) + HALF_MULTI >> OFFSET);
            vec_wgt_y = (img->pix_y + joff) * mv_mul + (int)(delta[currMB->weighted_skipmode ] * mv_array[j8][i8][1] *
                        (MULTI / delta[0]) + HALF_MULTI >> OFFSET);
#if HALF_PIXEL_COMPENSATION_M1
            assert(hd->is_field_sequence == img->is_field_sequence);
            if (img->is_field_sequence) {
                int deltaT, delta2;
                int oriPOC = 2 * hc->picture_distance;
                int oriRefPOC = oriPOC - delta[0];
                int scaledPOC = 2 * hc->picture_distance;
                int scaledRefPOC = scaledPOC - delta[currMB->weighted_skipmode ];
                getDeltas(&deltaT, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                vec_wgt_y = (img->pix_y + joff) * mv_mul + (int)(delta[currMB->weighted_skipmode ] *
                            (mv_array[j8][i8][1] + deltaT) * (16384 / delta[0]) + 8192 >> 14) - delta2;
            }
#endif
#endif
#if !RD1601_FIX_BG
            vec_wgt_x = Clip3(-32768, 32767, vec_wgt_x);
            vec_wgt_y = Clip3(-32768, 32767, vec_wgt_y);
#endif
        }
        if (img->type == F_IMG && hd->b_dmh_enabled && img->typeb != BP_IMG)

        {
            if (mv_mode == 0) {
                dmh_mode = 0;
            } else {
                dmh_mode = mv_array[j8][i8][3];

                first_x  = dmh_pos[dmh_mode][0][0];
                first_y  = dmh_pos[dmh_mode][0][1];
                second_x = dmh_pos[dmh_mode][1][0];
                second_y = dmh_pos[dmh_mode][1][1];
            }
        }

        if (!bframe) {
            //CHECKMOTIONRANGE
            if (dmh_mode == 0 || !hd->b_dmh_enabled || img->type != F_IMG || img->typeb == BP_IMG)


            {
                get_block(refframe, vec1_x, vec1_y, step_h, step_v, tmp_block, hd->integerRefY[refframe]
                          , ioff, joff

                         ); // need fix
            } else {

                first_x  = dmh_pos[dmh_mode][0][0];
                first_y  = dmh_pos[dmh_mode][0][1];
                second_x = dmh_pos[dmh_mode][1][0];
                second_y = dmh_pos[dmh_mode][1][1];
#if Mv_Rang
#if RD1601_FIX_BG
				mv0_x=(img->pix_x + ioff) * mv_mul + Clip3(-32768, 32767,mv_array[j8][i8][0] + first_x);
				mv0_y=(img->pix_y + joff) * mv_mul + Clip3(-32768, 32767,mv_array[j8][i8][1] + first_y);
				mv1_x=(img->pix_x + ioff) * mv_mul + Clip3(-32768, 32767,mv_array[j8][i8][0] + second_x);
				mv1_y=(img->pix_y + joff) * mv_mul + Clip3(-32768, 32767,mv_array[j8][i8][1] + second_y);
#else
				 mv0_x=Clip3(-32768, 32767,vec1_x + first_x);
			     mv0_y=Clip3(-32768, 32767,vec1_y + first_y);
				 mv1_x=Clip3(-32768, 32767,vec1_x + second_x);
				 mv1_y=Clip3(-32768, 32767,vec1_y + second_y);
#endif
				 get_block(refframe, mv0_x, mv0_y, step_h, step_v, tmp_block, hd->integerRefY[refframe]
				 , ioff, joff

					 ); // need fix
				 get_block(refframe, mv1_x, mv1_y, step_h, step_v, tmp_blockbw, hd->integerRefY[refframe]
				 , ioff, joff

					 ); // need fix // re-use of tmp_blockbw for memory saving
#else
                get_block(refframe, vec1_x + first_x, vec1_y + first_y, step_h, step_v, tmp_block, hd->integerRefY[refframe]
                          , ioff, joff

                         ); // need fix
                get_block(refframe, vec1_x + second_x, vec1_y + second_y, step_h, step_v, tmp_blockbw, hd->integerRefY[refframe]
                          , ioff, joff

                         ); // need fix // re-use of tmp_blockbw for memory saving
#endif
                for (i = 0; i < MAX_CU_SIZE; i++) {
                    for (j = 0; j < MAX_CU_SIZE; j++) {
                        tmp_block[j][i] = (tmp_block[j][i] + tmp_blockbw[j][i] + 1) / 2;
                    }
                }
            }

            if (img->num_of_references > 1 && img->type == F_IMG && currMB->weighted_skipmode  && (mv_mode == 0)) {
                get_block(currMB->weighted_skipmode, vec_wgt_x, vec_wgt_y, step_h, step_v, tmp_block_wgt,
                          hd->integerRefY[currMB->weighted_skipmode]
                          , ioff, joff

                         );

                for (i = 0; i < MAX_CU_SIZE; i++) {
                    for (j = 0; j < MAX_CU_SIZE; j++) {
                        tmp_block[j][i] = (tmp_block[j][i] + tmp_block_wgt[j][i] + 1) / 2;
                    }
                }
            }

            if (img->type == F_IMG && pred_dir == DUAL) {
                vec_wgt_x = (img->pix_x + ioff) * mv_mul + img->p_snd_tmp_mv[j8][i8][0];
                vec_wgt_y = (img->pix_y + joff) * mv_mul + img->p_snd_tmp_mv[j8][i8][1];

                get_block(hc->p_snd_refFrArr[j8][i8], vec_wgt_x, vec_wgt_y, step_h, step_v, tmp_block_wgt,
                          hd->integerRefY[hc->p_snd_refFrArr[j8][i8]]
                          , ioff, joff

                         );

                for (i = 0; i < MAX_CU_SIZE; i++) {
                    for (j = 0; j < MAX_CU_SIZE; j++) {
                        tmp_block[j][i] = (tmp_block[j][i] + tmp_block_wgt[j][i] + 1) / 2;
                    }
                }
            }

            if (img->type == F_IMG && (currMB->md_directskip_mode == BID_P_FST || currMB->md_directskip_mode == BID_P_SND)) {
                vec_wgt_x = (img->pix_x + ioff) * mv_mul + img->p_snd_tmp_mv[j8][i8][0];
                vec_wgt_y = (img->pix_y + joff) * mv_mul + img->p_snd_tmp_mv[j8][i8][1];

                get_block(hc->p_snd_refFrArr[j8][i8], vec_wgt_x, vec_wgt_y, step_h, step_v, tmp_block_wgt,
                          hd->integerRefY[hc->p_snd_refFrArr[j8][i8]]
                          , ioff, joff

                         );

                for (i = 0; i < MAX_CU_SIZE; i++) {
                    for (j = 0; j < MAX_CU_SIZE; j++) {
                        tmp_block[j][i] = (tmp_block[j][i] + tmp_block_wgt[j][i] + 1) / 2;
                    }
                }
            }

        } else { // !! for B MB one direction  shenyanfei cjw
            if (!pred_dir) {   // !! forward shenyanfei
                //CHECKMOTIONRANGE
                //CHECKMOTIONDIRECTION

                get_block(refframe, vec1_x, vec1_y, step_h, step_v, tmp_block, hd->integerRefY_fref[refframe]
                          , ioff, joff

                         );
            } else { // !! backward shenyanfei
                //CHECKMOTIONRANGE
                get_block(refframe, vec1_x, vec1_y, step_h, step_v, tmp_block, hd->integerRefY_bref[refframe]
                          , ioff, joff

                         );
            }
        }

        for (ii = 0; ii < step_h; ii++) {
            for (jj = 0; jj < step_v; jj++) {
                img->predBlock[jj + joff][ii + ioff] = tmp_block[jj][ii];
            }
        }
    } else { // !! pred_dir == 2
        if (mv_mode != 0) {
            //===== BI-DIRECTIONAL PREDICTION =====

            fw_mv_array = img->fw_mv;
            bw_mv_array = img->bw_mv;

            fw_refframe = img->fw_refFrArr[j8][i8];// fwd_ref_idx_to_refframe(img->fw_refFrArr[j8][i8]);
            bw_refframe = img->bw_refFrArr[j8][i8];// bwd_ref_idx_to_refframe(img->bw_refFrArr[j8][i8]);
            bw_ref_idx = img->bw_refFrArr[j8][i8];

        } else {
            //===== DIRECT PREDICTION =====
            fw_mv_array = img->fw_mv;
            bw_mv_array = img->bw_mv;
            bw_refframe = 0;


            if (hc->refFrArr[j8][i8] == -1) {   // next P is intra mode
                fw_refframe = 0;
            } else { // next P is skip or inter mode
                refframe = hc->refFrArr[j8][i8];

                fw_refframe = 0;  // DIRECT
                img->fw_refFrArr[j8][i8] = 0;
                img->bw_refFrArr[j8][i8] = 0;
            }
        }

        vec1_x = (img->pix_x + start_x) * mv_mul + fw_mv_array[j8][i8][0];
        vec1_y = (img->pix_y + start_y) * mv_mul + fw_mv_array[j8][i8][1];

        //CHECKMOTIONRANGE
        //CHECKMOTIONDIRECTION
        //rm52k

        vec2_x = (img->pix_x + start_x) * mv_mul + bw_mv_array[j8][i8][0];
        vec2_y = (img->pix_y + start_y) * mv_mul + bw_mv_array[j8][i8][1];

        //CHECKMOTIONRANGE

        // !! symirection prediction shenyanfei

        get_block(fw_refframe, vec1_x, vec1_y, step_h, step_v, tmp_block, hd->integerRefY_fref[fw_refframe]
                  , ioff, joff

                 );
        get_block(bw_refframe, vec2_x, vec2_y, step_h, step_v, tmp_blockbw, hd->integerRefY_bref[bw_refframe]
                  , ioff, joff

                 );
        for (ii = 0; ii < step_h; ii++) {
            for (jj = 0; jj < step_v; jj++) {
                img->predBlock[jj + joff][ii + ioff] = (tmp_block[jj][ii] + tmp_blockbw[jj][ii] + 1) / 2;
            }
        }

    }
}
void decode_one_IntraChromaBlock(int uv)
{
    codingUnit *currMB   = &img->mb_data[img->current_mb_nr];//GB current_mb_nr];
    //short * edgepixels = ( short * ) malloc ( ( ( 1 << currMB->ui_MbBitSize ) * 4 + 1 ) * sizeof ( short ) );
    short edgepixels[4 * MAX_CU_SIZE + 1];
#define EP ( edgepixels + ( ( 1 << currMB->ui_MbBitSize ) * 2 ) )
    int x, y;
    int bs_x;
    int bs_y;
    int i = 0, j = 0;
    int **piPredBuf;
    int img_x = (img->mb_x << MIN_CU_SIZE_IN_BIT);
    int img_y = (img->mb_y << MIN_CU_SIZE_IN_BIT);
    int predLmode = img->ipredmode[img_y / MIN_BLOCK_SIZE + 1][img_x / MIN_BLOCK_SIZE + 1];
    int mb_available_left_down;
    int p_avai[5];
    int mb_nr             = img->current_mb_nr;//GBimg->current_mb_nr;
    int mb_available_up;
    int mb_available_left;
    int  mb_available_up_left;
    int mb_available_up_right;
    int mb_BitSize = img->mb_data[mb_nr].ui_MbBitSize;
    int N8_SizeScale;
    int mb_x, mb_y;
    get_mb_pos(img->current_mb_nr, &mb_x, &mb_y, input->g_uiMaxSizeInBit);
    N8_SizeScale = (1 << mb_BitSize) >> MIN_CU_SIZE_IN_BIT;

    bs_x = MIN_BLOCK_SIZE * N8_SizeScale;
    bs_y = MIN_BLOCK_SIZE * N8_SizeScale;

    getIntraNeighborAvailabilities(currMB, input->g_uiMaxSizeInBit, img_x, img_y, (1 << mb_BitSize), (1 << mb_BitSize),
                                   p_avai);

    mb_available_left_down = p_avai[NEIGHBOR_INTRA_LEFT_DOWN];
    mb_available_left      = p_avai[NEIGHBOR_INTRA_LEFT];
    mb_available_up_left   = p_avai[NEIGHBOR_INTRA_UP_LEFT];
    mb_available_up        = p_avai[NEIGHBOR_INTRA_UP];
    mb_available_up_right  = p_avai[NEIGHBOR_INTRA_UP_RIGHT];


    for (i = -2 * bs_y; i <= 2 * bs_x; i++) {
        EP[i] = 1 << (input->sample_bit_depth - 1);
    }

    if (mb_available_up) {
        for (x = 0; x < bs_x; x++) {
            EP[x + 1] = hc->imgUV[uv][img->pix_c_y - 1][img->pix_c_x + x];
        }

    }

    if (mb_available_up_right) {
        for (x = 0; x < bs_x; x++) {
            if (img->pix_c_x + bs_x + x >= img->width_cr) {
                EP[1 + x + bs_x] = hc->imgUV[uv][img->pix_c_y - 1][img->width_cr - 1];
            } else {
                EP[1 + x + bs_x] = hc->imgUV[uv][img->pix_c_y - 1][img->pix_c_x + bs_x + x];
            }
        }
    } else {
        for (x = 0; x < bs_x; x++) {
            EP[1 + x + bs_x] = EP[bs_x];
        }
    }

    if (mb_available_left) {
        for (y = 0; y < bs_y; y++) {
            EP[-1 - y] = hc->imgUV[uv][img->pix_c_y + y][img->pix_c_x - 1];
        }

    }

    if (mb_available_left_down) {
        for (y = 0; y < bs_y; y++) {
            if (img->pix_c_y + bs_y + y >= img->height_cr) {
                EP[-1 - y - bs_y] = hc->imgUV[uv][img->height_cr - 1][img->pix_c_x - 1];
            } else {
                EP[-1 - y - bs_y] = hc->imgUV[uv][img->pix_c_y + bs_y + y][img->pix_c_x - 1];
            }
        }
    } else {
        for (y = 0; y < bs_y; y++) {
            EP[-1 - y - bs_y] = EP[-bs_y];
        }
    }

    {
        if (mb_available_up_left) {
            EP[0] = hc->imgUV[uv][img->pix_c_y - 1][img->pix_c_x - 1];
        } else if (mb_available_up) {
            EP[0] = hc->imgUV[uv][img->pix_c_y - 1][img->pix_c_x];
        } else if (mb_available_left) {
            EP[0] = hc->imgUV[uv][img->pix_c_y][img->pix_c_x - 1];
        }
    }

    get_mem2Dint(&piPredBuf, bs_y, bs_x);

    for (y = 0; y < bs_y; y++) {
        for (x = 0; x < bs_x; x++) {
            piPredBuf[y][x] = 0;
        }
    }
    predIntraChromaAdi(EP, piPredBuf, currMB->c_ipred_mode, mb_BitSize - 1, mb_available_up, mb_available_left, predLmode,
                       input->sample_bit_depth);
    for (y = 0; y < bs_y; y++) {
        for (x = 0; x < bs_x; x++) {
            img->predBlock[y][x] = piPredBuf[y][x];
        }
    }
    free_mem2Dint(piPredBuf);
    //free(edgepixels);
}

#define  IClip( Min, Max, Val) (((Val)<(Min))? (Min):(((Val)>(Max))? (Max):(Val)))

int get_chroma_subpix_blk(int **dst, int xoffs, int yoffs, int width, int height, int curx, int cury, int posx,
                          int posy, byte **refpic)
{
    int i, j;
    int x0, x1, x2, x3;
    int y0, y1, y2, y3;
    int val;

    int shift1 = input->sample_bit_depth - 8;
    int add1 = (1 << (shift1)) >> 1;
    int shift2 = 20 - input->sample_bit_depth;
    int add2 = 1 << (19 - input->sample_bit_depth);

    int max_pel_value = (1 << input->sample_bit_depth) - 1;
    static const int COEF[8][4] = {
        { 0, 64, 0, 0 },
        { -4, 62, 6, 0 },
        { -6, 56, 15, -1 },
        { -5, 47, 25, -3 },
        { -4, 36, 36, -4 },
        { -3, 25, 47, -5 },
        { -1, 15, 56, -6 },
        { 0, 6, 62, -4 }
    };


    if (posy == 0) {
        for (i = 0; i < height; i++) {
            for (j = 0; j < width; j++) {
                int base_x = curx - 1 + j;
                x0 = max(0, min(img->width_cr - 1, base_x + 0));
                x1 = max(0, min(img->width_cr - 1, base_x + 1));
                x2 = max(0, min(img->width_cr - 1, base_x + 2));
                x3 = max(0, min(img->width_cr - 1, base_x + 3));
                y0 = y1 = y2 = y3 = max(0, min(img->height_cr - 1, cury + i));
                val = refpic[y0][x0] * COEF[posx][0] + refpic[y1][x1] * COEF[posx][1] + refpic[y2][x2] * COEF[posx][2] + refpic[y3][x3]
                      * COEF[posx][3];
                dst[yoffs + i][xoffs + j] = IClip(0, max_pel_value, (val + 32) >> 6);
            }
        }
    } else if (posx == 0) {
        for (i = 0; i < height; i++) {
            for (j = 0; j < width; j++) {
                int base_y = cury - 1 + i;
                y0 = max(0, min(img->height_cr - 1, base_y + 0));
                y1 = max(0, min(img->height_cr - 1, base_y + 1));
                y2 = max(0, min(img->height_cr - 1, base_y + 2));
                y3 = max(0, min(img->height_cr - 1, base_y + 3));
                x0 = x1 = x2 = x3 = max(0, min(img->width_cr - 1, curx + j));
                val = refpic[y0][x0] * COEF[posy][0] + refpic[y1][x1] * COEF[posy][1] + refpic[y2][x2] * COEF[posy][2] + refpic[y3][x3]
                      * COEF[posy][3];
                dst[yoffs + i][xoffs + j] = IClip(0, max_pel_value, (val + 32) >> 6);
            }
        }
    } else {
        int tmpbuf[36][32];

        for (i = -1; i < height + 3; i++) {
            for (j = 0; j < width; j++) {
                int base_x = curx - 1 + j;
                x0 = max(0, min(img->width_cr - 1, base_x + 0));
                x1 = max(0, min(img->width_cr - 1, base_x + 1));
                x2 = max(0, min(img->width_cr - 1, base_x + 2));
                x3 = max(0, min(img->width_cr - 1, base_x + 3));
                y0 = y1 = y2 = y3 = max(0, min(img->height_cr - 1, cury + i));
                tmpbuf[i + 1][j] = refpic[y0][x0] * COEF[posx][0] + refpic[y1][x1] * COEF[posx][1] + refpic[y2][x2] * COEF[posx][2] +
                                   refpic[y3][x3] * COEF[posx][3];
                tmpbuf[i + 1][j] = (tmpbuf[i + 1][j] + add1) >> shift1;
            }
        }

        for (i = 0; i < height; i++) {
            for (j = 0; j < width; j++) {
                val = tmpbuf[i    ][j] * COEF[posy][0] + tmpbuf[i + 1][j] * COEF[posy][1] +
                      tmpbuf[i + 2][j] * COEF[posy][2] + tmpbuf[i + 3][j] * COEF[posy][3];

                dst[yoffs + i][xoffs + j] = IClip(0, max_pel_value, (val + add2) >> shift2);
            }
        }
    }

    return val;
}


void decode_one_InterChromaBlock(int uv)
{
    codingUnit *currMB = &img->mb_data[img->current_mb_nr];//GB current_mb_nr];
    int i = 0, j = 0, ii = 0, jj = 0, i1 = 0, j1 = 0;
    int jf = 0;

    int ioff, joff, i8, j8;

    int  ifx;
    int f1, f2, f3, f4;
    int if1;
    int refframe, fw_refframe, bw_refframe, mv_mode, pred_dir; // = currMB->ref_frame;
    int bw_ref_idx;
    int *** mv_array, * **fw_mv_array, * **bw_mv_array;
    int bframe = (img->type == B_IMG);

    int mb_nr = img->current_mb_nr;//GBimg->current_mb_nr;

    int mb_BitSize = img->mb_data[mb_nr].ui_MbBitSize;
    int N8_SizeScale;
    int start_x, start_y, step_h, step_v;
    int start_b8x, start_b8y, step_b8h, step_b8v;
    int dmh_mode = 0;
    int center_x, center_y;
    int delta = 0;
    int delta2 = 0;
    byte **p_ref;
    byte **p_ref1;
    byte **p_ref2;

    int center_x1, center_y1;
    int psndrefframe;

#if FIX_MAX_REF
    int distanceDelta[MAXREF];

    get_p_reference_distances(distanceDelta);
#else
    int distanceDelta[4];
    distanceDelta[0] = (2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance) + 512) % 512;
    distanceDelta[1] = (2 * (hc->picture_distance - fref[1]->imgtr_fwRefDistance) + 512) % 512;
    distanceDelta[2] = (2 * (hc->picture_distance - fref[2]->imgtr_fwRefDistance) + 512) % 512;
    distanceDelta[3] = (2 * (hc->picture_distance - fref[3]->imgtr_fwRefDistance) + 512) % 512;
    if (0 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        distanceDelta[0] = 1;
    }
    if (1 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        distanceDelta[1] = 1;
    }
    if (2 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        distanceDelta[2] = 1;
    }
    if (3 == hd->curr_RPS.num_of_ref - 1 && hd->background_reference_enable) {
        distanceDelta[3] = 1;
    }
#endif

    N8_SizeScale = (1 << mb_BitSize) >> MIN_CU_SIZE_IN_BIT;

    ifx = if1 = 0;

    f1 = 8;
    f2 = 7;

    f3 = f1 * f1;
    f4 = f3 / 2;

    for (j = 4; j < 6; j++) {
        joff = (j - 4) * MIN_BLOCK_SIZE / 2 * N8_SizeScale;

        for (i = 0; i < 2; i++) {
            ioff = i * MIN_BLOCK_SIZE / 2 * N8_SizeScale;
            get_pix_offset(currMB->cuType, mb_BitSize, i, j - 4, &start_x, &start_y, &step_h, &step_v);
            ioff = start_x = start_x >> 1;
            joff = start_y = start_y >> 1;
            step_h = step_h >> 1;
            step_v = step_v >> 1;
            i8 = img->pix_c_x + start_x;
            j8 = img->pix_c_y + start_y;

            mv_mode = currMB->b8mode[2 * (j - 4) + i];
            pred_dir = currMB->b8pdir[2 * (j - 4) + i];
            if (pred_dir != SYM && pred_dir != BID) {
                //--- FORWARD/BACKWARD PREDICTION ---
                if (!bframe) {
                    mv_array = img->tmp_mv;
                } else if (!pred_dir) {
                    mv_array = img->fw_mv;
                } else {
                    mv_array = img->bw_mv;
                }

                get_b8_offset(currMB->cuType, mb_BitSize, i, j - 4, &start_b8x, &start_b8y, &step_b8h, &step_b8v);

                jf = img->block8_y + start_b8y;
                if1 = img->block8_x + start_b8x;

                if (!bframe) {
                    refframe = hc->refFrArr[jf][if1];
                } else if (!pred_dir) {
                    refframe = img->fw_refFrArr[jf][if1];// fwd_ref_idx_to_refframe(img->fw_refFrArr[jf][if1]);
                } else {
                    refframe = img->bw_refFrArr[jf][if1];// bwd_ref_idx_to_refframe(img->bw_refFrArr[jf][if1]);
                    bw_ref_idx = img->bw_refFrArr[jf][if1];
                }

                if (img->type == F_IMG && hd->b_dmh_enabled && img->typeb != BP_IMG)


                {
                    dmh_mode = mv_array[jf][if1][3];
                } else {
                    dmh_mode = 0;
                }

#if HALF_PIXEL_CHROMA
                if (img->is_field_sequence && input->chroma_format == 1) {
                    int fw_bw = bframe && pred_dir ? -1 : 0;
                    int distance = calculate_distance(refframe, fw_bw);
                    delta = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                }
#endif
                psndrefframe = hc->p_snd_refFrArr[jf][if1];

                if (img->type == F_IMG && pred_dir == DUAL) {
#if HALF_PIXEL_CHROMA
                    if (img->is_field_sequence && input->chroma_format == 1) {
                        int fw_bw = 0; //fw
                        int distance = calculate_distance(psndrefframe, fw_bw);
                        delta2 = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                    }
#endif
                }

                if (img->type == F_IMG && (currMB->md_directskip_mode == BID_P_FST || currMB->md_directskip_mode == BID_P_SND)) {
#if HALF_PIXEL_CHROMA
                    if (img->is_field_sequence && input->chroma_format == 1) {
                        int fw_bw = 0; //fw
                        int distance = calculate_distance(psndrefframe, fw_bw);
                        delta2 = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                    }
#endif
                }
#if RD1601_FIX_BG
				center_y = (img->pix_c_y + joff) * f1 + Clip3(-32768, 32767, mv_array[jf][if1][1] - delta);
#else
                center_y = (img->pix_c_y + joff) * f1 + mv_array[jf][if1][1] - delta;
#endif
                center_x = (img->pix_c_x + ioff) * f1 + mv_array[jf][if1][0];
#if RD1601_FIX_BG
				center_y1 = (img->pix_c_y + joff) * f1 + Clip3(-32768, 32767, img->p_snd_tmp_mv[jf][if1][1] - delta2);
#else
                center_y1 = (img->pix_c_y + joff) * f1 + img->p_snd_tmp_mv[jf][if1][1] - delta2;
#endif
                center_x1 = (img->pix_c_x + ioff) * f1 + img->p_snd_tmp_mv[jf][if1][0];

                if (img->num_of_references > 1 && (!mv_mode) &&  img->type == F_IMG && currMB->weighted_skipmode) {
#if HALF_PIXEL_CHROMA
                    if (img->is_field_sequence && input->chroma_format == 1) {
                        int fw_bw = 0; //fw
                        int distance = calculate_distance(currMB->weighted_skipmode, fw_bw);
                        delta2 = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;

                        distance = calculate_distance(refframe, fw_bw);
                        delta = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                    }
#endif
                    //center_y1 = (img->pix_c_y + joff) * f1 + img->p_snd_tmp_mv[jf][if1][1] - delta2;
#if MV_SCALE
                 
#if RD1601_FIX_BG
					center_y1 = Clip3(-32768, 32767,(((long long int)(mv_array[jf][if1][1] + delta) * distanceDelta[currMB->weighted_skipmode ] * (MULTI / distanceDelta[0]) + HALF_MULTI) >> OFFSET) - delta2);
					center_y1 = (img->pix_c_y + joff) * f1 + Clip3(-32768, 32767,  center_y1 - delta2);
#else
					   center_y1 = (img->pix_c_y + joff) * f1 + scale_mv(mv_array[jf][if1][1] + delta,
                                distanceDelta[currMB->weighted_skipmode ], distanceDelta[0]) - delta2;
#endif
#else
                    center_y1 = (img->pix_c_y + joff) * f1 + (int)(distanceDelta[currMB->weighted_skipmode ] *
                                (mv_array[jf][if1][1] + delta) * (MULTI / distanceDelta[0]) + HALF_MULTI >> OFFSET) - delta2;
					 center_y1 -= delta2;
#endif
                  
                    center_x1 = (img->pix_c_x + ioff) * f1 + img->p_snd_tmp_mv[jf][if1][0];
                }

                if (!bframe) {
                    if (hd->background_reference_enable && refframe == img->num_of_references - 1 &&
                        (img->type == P_IMG || img->type == F_IMG) && img->typeb != BP_IMG) {
                        p_ref = hd->background_frame[uv + 1];
                    } else if (img->typeb == BP_IMG) {
                        p_ref = hd->background_frame[uv + 1];
                    }

                    else {
                        p_ref = hd->integerRefUV[refframe][uv];
                    }


                } else {
                    refframe = 0;

                    if (!pred_dir) {
                        p_ref = hd->integerRefUV_fref[refframe][uv];
                    } else {
                        p_ref = hd->integerRefUV_bref[refframe][uv];
                    }
                }
                if (hd->background_reference_enable && psndrefframe == img->num_of_references - 1 &&
                    (img->type == P_IMG || img->type == F_IMG)/* && img->typeb != BP_IMG*/) {
                    p_ref1 = hd->background_frame[uv + 1];
                } else if (img->typeb == BP_IMG) {
                    p_ref1 = hd->background_frame[uv + 1];
                }

                else if (img->num_of_references > 1 && (!mv_mode) &&  img->type == F_IMG && currMB->weighted_skipmode) {
                    p_ref1 = hd->integerRefUV[currMB->weighted_skipmode][uv];
                } else {
                    p_ref1 = hd->integerRefUV[psndrefframe][uv];
                }
                if (hd->background_reference_enable && psndrefframe == img->num_of_references - 1 &&
                    (img->type == P_IMG || img->type == F_IMG)/* && img->typeb != BP_IMG*/) {
                    p_ref2 = hd->background_frame[uv + 1];
                } else if (img->typeb == BP_IMG) {
                    p_ref2 = hd->background_frame[uv + 1];
                }

                else if (img->num_of_references > 1 && (!mv_mode) &&  img->type == F_IMG && currMB->weighted_skipmode) {
                    p_ref1 = hd->integerRefUV[currMB->weighted_skipmode][uv];
                } else {
                    p_ref2 = hd->integerRefUV[psndrefframe][uv];
                }

                if (dmh_mode) {
#if Mv_Rang
#if RD1601_FIX_BG
			
					i1 =   (img->pix_c_x + ioff) * f1+Clip3(-32768, 32767,mv_array[jf][if1][0] + dmh_pos[dmh_mode][0][0]);
					j1 = (img->pix_c_y + joff) * f1+Clip3(-32768, 32767,mv_array[jf][if1][1] + dmh_pos[dmh_mode][0][1])- delta;
					get_chroma_subpix_blk(img->predBlock, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref);

					i1 = (img->pix_c_x + ioff) * f1+Clip3(-32768, 32767, mv_array[jf][if1][0] + dmh_pos[dmh_mode][1][0]);
					j1 =(img->pix_c_y + joff) * f1 + Clip3(-32768, 32767,mv_array[jf][if1][1] + dmh_pos[dmh_mode][1][1])- delta;

					get_chroma_subpix_blk(img->predBlockTmp, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref);
#else
					i1 = Clip3(-32768, 32767,center_x + dmh_pos[dmh_mode][0][0]);
					j1 = Clip3(-32768, 32767,center_y + dmh_pos[dmh_mode][0][1]);
					get_chroma_subpix_blk(img->predBlock, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref);

					i1 = Clip3(-32768, 32767,center_x + dmh_pos[dmh_mode][1][0]);
					j1 = Clip3(-32768, 32767,center_y + dmh_pos[dmh_mode][1][1]);
					get_chroma_subpix_blk(img->predBlockTmp, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref);

#endif

#else
                    i1 = center_x + dmh_pos[dmh_mode][0][0];
                    j1 = center_y + dmh_pos[dmh_mode][0][1];
                    get_chroma_subpix_blk(img->predBlock, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref);

                    i1 = center_x + dmh_pos[dmh_mode][1][0];
                    j1 = center_y + dmh_pos[dmh_mode][1][1];
                    get_chroma_subpix_blk(img->predBlockTmp, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref);
#endif
                    for (jj = 0; jj < step_v; jj++) {
                        for (ii = 0; ii < step_h; ii++) {
                            img->predBlock[jj + joff][ii + ioff] =
                                (img->predBlock[jj + joff][ii + ioff] + img->predBlockTmp[jj + joff][ii + ioff] + 1) >> 1;
                        }
                    }
                } else {
                    i1 = center_x;
                    j1 = center_y;
                    get_chroma_subpix_blk(img->predBlock, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref);
                    if (img->num_of_references > 1 && (!mv_mode) &&  img->type == F_IMG && currMB->weighted_skipmode) {
                        i1 = center_x1;
                        j1 = center_y1;
                        get_chroma_subpix_blk(img->predBlockTmp, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref1);
                        for (jj = 0; jj < step_v; jj++) {
                            for (ii = 0; ii < step_h; ii++) {
                                img->predBlock[jj + joff][ii + ioff] =
                                    (img->predBlock[jj + joff][ii + ioff] + img->predBlockTmp[jj + joff][ii + ioff] + 1) >> 1;
                            }
                        }
                    }
                    if (img->type == F_IMG && pred_dir == DUAL) {
                        center_x1 = (img->pix_c_x + ioff) * f1 + img->p_snd_tmp_mv[jf][if1][0];
                        i1 = center_x1;
                        j1 = center_y1;
                        get_chroma_subpix_blk(img->predBlockTmp, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref1);
                        for (jj = 0; jj < step_v; jj++) {
                            for (ii = 0; ii < step_h; ii++) {
                                img->predBlock[jj + joff][ii + ioff] =
                                    (img->predBlock[jj + joff][ii + ioff] + img->predBlockTmp[jj + joff][ii + ioff] + 1) >> 1;
                            }
                        }
                    }
                    if (img->type == F_IMG && (currMB->md_directskip_mode == BID_P_FST || currMB->md_directskip_mode == BID_P_SND)) {
                        center_x1 = (img->pix_c_x + ioff) * f1 + img->p_snd_tmp_mv[jf][if1][0];
                        i1 = center_x1;
                        j1 = center_y1;

                        get_chroma_subpix_blk(img->predBlockTmp, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7, p_ref2);
                        for (jj = 0; jj < step_v; jj++) {
                            for (ii = 0; ii < step_h; ii++) {
                                img->predBlock[jj + joff][ii + ioff] =
                                    (img->predBlock[jj + joff][ii + ioff] + img->predBlockTmp[jj + joff][ii + ioff] + 1) >> 1;
                            }
                        }
                    }
                }
            } else {
                if (mv_mode != 0) {
                    //===== BI-DIRECTIONAL PREDICTION =====
                    fw_mv_array = img->fw_mv;
                    bw_mv_array = img->bw_mv;
                } else {
                    //===== DIRECT PREDICTION =====
                    fw_mv_array = img->fw_mv;
                    bw_mv_array = img->bw_mv;
                }

                get_b8_offset(currMB->cuType, mb_BitSize, i, j - 4, &start_b8x, &start_b8y, &step_b8h, &step_b8v);
                jf = img->block8_y + start_b8y;
                ifx = img->block8_x + start_b8x;

                if (mv_mode != 0) {
                    fw_refframe = img->fw_refFrArr[jf][ifx];
                    bw_refframe = img->bw_refFrArr[jf][ifx];
                    bw_ref_idx = img->bw_refFrArr[jf][ifx];
                    bw_ref_idx = bw_ref_idx;
                } else {
                    fw_refframe = 0;
                    bw_refframe = 0;
                }

#if HALF_PIXEL_CHROMA
                if (img->is_field_sequence && input->chroma_format == 1) {
                    int fw_bw = 0;  //fw
                    int distance = calculate_distance(fw_refframe, fw_bw);
                    delta = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                }
#endif
#if HALF_PIXEL_CHROMA
                if (img->is_field_sequence && input->chroma_format == 1) {
                    int fw_bw = -1;  //bw
                    int distance = calculate_distance(bw_refframe, fw_bw);
                    delta2 = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                }
#endif
                i1 = (img->pix_c_x + ioff) * f1 + fw_mv_array[jf][ifx][0];
                j1 = (img->pix_c_y + joff) * f1 + fw_mv_array[jf][ifx][1] - delta;
                get_chroma_subpix_blk(img->predBlock, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7,
                                      hd->integerRefUV_fref[fw_refframe][uv]);

                i1 = (img->pix_c_x + ioff) * f1 + bw_mv_array[jf][ifx][0];
                j1 = (img->pix_c_y + joff) * f1 + bw_mv_array[jf][ifx][1] - delta2;
                get_chroma_subpix_blk(img->predBlockTmp, ioff, joff, step_h, step_v, i1 >> 3, j1 >> 3, i1 & 7, j1 & 7,
                                      hd->integerRefUV_bref[fw_refframe][uv]);

                for (jj = 0; jj < step_v; jj++) {
                    for (ii = 0; ii < step_h; ii++) {
                        img->predBlock[jj + joff][ii + ioff] =
                            (img->predBlock[jj + joff][ii + ioff] + img->predBlockTmp[jj + joff][ii + ioff] + 1) / 2;
                    }
                }
            }
        }
    }
}

int decode_SubMB()
{
    codingUnit *currMB   = &img->mb_data[img->current_mb_nr];//GB current_mb_nr];

    int uv;//, hv;
    int tmp;
    int **curr_blk;
    //  int curr_blk[MAX_CU_SIZE][MAX_CU_SIZE];  //SW for AVS   //liweiran
    int block8Nx8N;

    int mb_BitSize = img->mb_data[img->current_mb_nr].ui_MbBitSize;
    int N8_SizeScale = 1 << (mb_BitSize - MIN_CU_SIZE_IN_BIT);

    get_mem2Dint(&curr_blk, 1 << mb_BitSize, 1 << mb_BitSize);
    if (currMB->trans_size == 0) {
        if (currMB->cuType != I16MB) {
            {
                for (block8Nx8N = 0; block8Nx8N < 4; block8Nx8N++) {
                    if (currMB->b8mode[block8Nx8N] != IBLOCK) {
                        decode_one_InterLumaBlock(block8Nx8N);
                    }
                }
            }
            get_curr_blk(0, curr_blk, mb_BitSize);
            idct_dequant_B8(0, currMB->qp - MIN_QP, curr_blk, mb_BitSize);
        } else {
            get_curr_blk(0, curr_blk, mb_BitSize);
            tmp = intrapred((img->mb_x << MIN_CU_SIZE_IN_BIT), (img->mb_y << MIN_CU_SIZE_IN_BIT), mb_BitSize);

            if (tmp == SEARCH_SYNC) {   /* make 4x4 prediction block mpr from given prediction img->mb_mode */
                return SEARCH_SYNC;  /* bit error */
            }

            idct_dequant_B8(0, currMB->qp - MIN_QP, curr_blk, mb_BitSize);
        }
    } else {
        // luma decoding **************************************************
        {
            for (block8Nx8N = 0; block8Nx8N < 4; block8Nx8N++) {
                if (currMB->b8mode[block8Nx8N] != IBLOCK) {
                    decode_one_InterLumaBlock(block8Nx8N);
                } else {
                    get_curr_blk(block8Nx8N, curr_blk, mb_BitSize - 1);
                    if (currMB->cuType == InNxNMB) {
                        tmp = intrapred((img->mb_x << MIN_CU_SIZE_IN_BIT),
                                        (img->mb_y << MIN_CU_SIZE_IN_BIT) + block8Nx8N * (1 << (mb_BitSize - 2)), mb_BitSize);
                        if (tmp == SEARCH_SYNC) {   /* make 4x4 prediction block mpr from given prediction img->mb_mode */
                            return SEARCH_SYNC;  /* bit error */
                        }
                        idct_dequant_B8_NSQT(block8Nx8N, curr_blk, mb_BitSize - 1);
                    } else if (currMB->cuType == INxnNMB) {
                        tmp = intrapred((img->mb_x << MIN_CU_SIZE_IN_BIT) + block8Nx8N * (1 << (mb_BitSize - 2)),
                                        (img->mb_y << MIN_CU_SIZE_IN_BIT), mb_BitSize);
                        if (tmp == SEARCH_SYNC) {   /* make 4x4 prediction block mpr from given prediction img->mb_mode */
                            return SEARCH_SYNC;  /* bit error */
                        }
                        idct_dequant_B8_NSQT(block8Nx8N, curr_blk, mb_BitSize - 1);
                    } else {

                        ///////////////////
                        tmp = intrapred((img->mb_x << MIN_CU_SIZE_IN_BIT) + ((block8Nx8N & 1) << MIN_BLOCK_SIZE_IN_BIT) * N8_SizeScale,
                                        (img->mb_y << MIN_CU_SIZE_IN_BIT) + ((block8Nx8N & 2) << (MIN_BLOCK_SIZE_IN_BIT - 1)) * N8_SizeScale,
                                        mb_BitSize - 1);
                        if (tmp == SEARCH_SYNC) {   /* make 4x4 prediction block mpr from given prediction img->mb_mode */
                            return SEARCH_SYNC;  /* bit error */
                        }

                        idct_dequant_B8(block8Nx8N, currMB->qp - MIN_QP, curr_blk, mb_BitSize - 1);
                        ///////////////////
                    }

                }
            }
        }
        for (block8Nx8N = 0; block8Nx8N < 4; block8Nx8N++) {
            if (currMB->b8mode[block8Nx8N] != IBLOCK) {
                get_curr_blk(block8Nx8N, curr_blk, mb_BitSize - 1);
                if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->cuType == P2NXN || currMB->cuType == PHOR_UP ||
                        currMB->cuType == PHOR_DOWN || currMB->cuType == PNX2N || currMB->cuType == PVER_LEFT ||
                        currMB->cuType == PVER_RIGHT)) {
                    idct_dequant_B8_NSQT(block8Nx8N, curr_blk, mb_BitSize - 1);
                } else {
                    idct_dequant_B8(block8Nx8N, currMB->qp - MIN_QP, curr_blk, mb_BitSize - 1);
                }
            }
        }
    }


    if (input->chroma_format == 1) {
        for (uv = 0; uv < 2; uv++) {
            if (IS_INTRA(currMB)) {
                decode_one_IntraChromaBlock(uv);
            } else {
                decode_one_InterChromaBlock(uv);
            }
            get_curr_blk(4 + uv, curr_blk, mb_BitSize - 1);
            if (input->sample_bit_depth > 8) {
                idct_dequant_B8(4 + uv, QP_SCALE_CR[Clip3(0, 63,
                                                    (currMB->qp - (8 * (input->sample_bit_depth - 8)) - MIN_QP))] + (8 * (input->sample_bit_depth - 8)), curr_blk,
                                mb_BitSize - 1);
            } else {
                idct_dequant_B8(4 + uv, QP_SCALE_CR[currMB->qp - MIN_QP], curr_blk, mb_BitSize - 1);
            }

        }
    }

    free_mem2Dint(curr_blk);
    return 0;
}

