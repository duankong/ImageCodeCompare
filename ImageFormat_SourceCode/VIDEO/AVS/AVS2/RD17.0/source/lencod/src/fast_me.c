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
Fast integer pel motion estimation and fractional pel motion estimation
algorithms are described in this file.
1. get_mem_FME() and free_mem_FME() are functions for allocation and release
of memories about motion estimation
2. FME_BlockMotionSearch() is the function for fast integer pel motion
estimation and fractional pel motion estimation
3. DefineThreshold() defined thresholds for early termination
*
*************************************************************************************
*/

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#include "../../lcommon/inc/memalloc.h"
#include "../../lcommon/inc/inter-prediction.h"
#include "fast_me.h"
#include "refbuf.h"
#include "block.h"
#include "../../lcommon/inc/block_info.h"


extern  int   *byte_abs;
extern  int   *mvbits;
extern  int   *spiral_search_x;
extern  int   *spiral_search_y;

extern  int   *int_motion_cost;

extern  int check_mvd(int mvd_x, int mvd_y);

#if Mv_check_bug
	extern  int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype, int dmh_x, int dmh_y);
	extern  int check_mv_range_sym(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype,
		int ref, int dmh_x, int dmh_y);
#else
	extern  int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype);
	extern  int check_mv_range_sym(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y, int blocktype,
		int ref);
#endif

extern int g_Left_Down_Avail_Matrix64[16][16];
extern int g_Left_Down_Avail_Matrix32[8][8];
extern int g_Left_Down_Avail_Matrix16[4][4];
extern int g_Left_Down_Avail_Matrix8[2][2];
extern int g_Up_Right_Avail_Matrix64[16][16];
extern int g_Up_Right_Avail_Matrix32[8][8];
extern int g_Up_Right_Avail_Matrix16[4][4];
extern int g_Up_Right_Avail_Matrix8[2][2];

static pel_t (*PelY_14)(pel_t **, int, int);
unsigned short MEQ_TAB[64] = {
    32768, 29775, 27554, 25268, 23170, 21247, 19369, 17770,
    16302, 15024, 13777, 12634, 11626, 10624, 9742, 8958,
    8192, 7512, 6889, 6305, 5793, 5303, 4878, 4467,
    4091, 3756, 3444, 3161, 2894, 2654, 2435, 2235,
    2048, 1878, 1722, 1579, 1449, 1329, 1218, 1117,
    1024, 939, 861, 790, 724, 664, 609, 558,
    512, 470, 430, 395, 362, 332, 304, 279,
    256, 235, 215, 197, 181, 166, 152, 140
};

/*
*************************************************************************
* Function:Dynamic memory allocation of all infomation needed for Fast ME
* Input:
* Output:
* Return: Number of allocated bytes
* Attention:
*************************************************************************
*/

static const int quant_coef[6][4][4] = {
    {{13107, 8066, 13107, 8066}, { 8066, 5243, 8066, 5243}, {13107, 8066, 13107, 8066}, { 8066, 5243, 8066, 5243}},
    {{11916, 7490, 11916, 7490}, { 7490, 4660, 7490, 4660}, {11916, 7490, 11916, 7490}, { 7490, 4660, 7490, 4660}},
    {{10082, 6554, 10082, 6554}, { 6554, 4194, 6554, 4194}, {10082, 6554, 10082, 6554}, { 6554, 4194, 6554, 4194}},
    {{ 9362, 5825, 9362, 5825}, { 5825, 3647, 5825, 3647}, { 9362, 5825, 9362, 5825}, { 5825, 3647, 5825, 3647}},
    {{ 8192, 5243, 8192, 5243}, { 5243, 3355, 5243, 3355}, { 8192, 5243, 8192, 5243}, { 5243, 3355, 5243, 3355}},
    {{ 7282, 4559, 7282, 4559}, { 4559, 2893, 4559, 2893}, { 7282, 4559, 7282, 4559}, { 4559, 2893, 4559, 2893}}
};
void DefineThreshold()
{
    AlphaSec[1] = 0.01f;
    AlphaSec[2] = 0.01f;
    AlphaSec[3] = 0.01f;
    AlphaSec[4] = 0.02f;
    AlphaSec[5] = 0.03f;
    AlphaSec[6] = 0.03f;
    AlphaSec[7] = 0.04f;
    AlphaSec[8] = 0.05f;
    AlphaThird[1] = 0.06f;
    AlphaThird[2] = 0.07f;
    AlphaThird[3] = 0.07f;
    AlphaThird[4] = 0.08f;
    AlphaThird[5] = 0.12f;
    AlphaThird[6] = 0.11f;
    AlphaThird[7] = 0.15f;
    AlphaThird[8] = 0.16f;
    DefineThresholdMB();
    return;
}
void DefineThresholdMB()
{
#if TH_ME
    int gb_qp_per    = (img->qp - MIN_QP) / 6;
    int gb_qp_rem    = (img->qp - MIN_QP) % 6;
#else
    int gb_qp_per    = (input->qpP - MIN_QP) / 6;
    int gb_qp_rem    = (input->qpP - MIN_QP) % 6;
#endif
    int gb_q_bits    = 15 + gb_qp_per;
    int gb_qp_const, Thresh4x4;
    if (img->type == INTRA_IMG) {
        gb_qp_const = (1 << gb_q_bits) / 3;    // intra
    } else {
        gb_qp_const = (1 << gb_q_bits) / 6;    // inter
    }
    Thresh4x4 = ((1 << gb_q_bits) - gb_qp_const) / quant_coef[gb_qp_rem][0][0];
    Quantize_step = Thresh4x4 / (4 * 5.61f);
    Bsize[8] = (16 * 16) * Quantize_step;
    Bsize[7] = (16 * 16) * Quantize_step;
    Bsize[6] = Bsize[7] * 4;
    Bsize[5] = Bsize[7] * 4;
    Bsize[4] = Bsize[5] * 4;
    Bsize[3] = Bsize[4] * 4;
    Bsize[2] = Bsize[4] * 4;
    Bsize[1] = Bsize[2] * 4;
}

int get_mem_mincost(int **** **mv)
{
    int i, j, k, l;
    int *m;

    if ((*mv = (int **** *) calloc(img->height / MIN_BLOCK_SIZE, sizeof(int ** **))) == NULL) {
        no_mem_exit("get_mem_mv: mv");
    }

    for (i = 0; i < img->height / MIN_BLOCK_SIZE; i++) {
        if (((*mv) [i] = (int ** **) calloc(img->width / MIN_BLOCK_SIZE, sizeof(int ** *))) == NULL) {
            no_mem_exit("get_mem_mv: mv");
        }

        for (j = 0; j < img->width / MIN_BLOCK_SIZE; j++) {
            if (((*mv) [i][j] = (int ** *) calloc(BUF_CYCLE, sizeof(int **))) == NULL) {
                no_mem_exit("get_mem_mv: mv");
            }
            for (k = 0; k < BUF_CYCLE; k++) {
                if (((*mv) [i][j][k] = (int **) calloc(9, sizeof(int *))) == NULL) {
                    no_mem_exit("get_mem_mv: mv");
                }
            }
        }
    }

    (*mv) [0][0][0][0] = (int *) calloc(img->width / MIN_BLOCK_SIZE * img->height / MIN_BLOCK_SIZE * BUF_CYCLE * 9 *
                                        5, sizeof(int));

    m = (*mv) [0][0][0][0];

    for (i = 0; i < img->height / MIN_BLOCK_SIZE; i++) {
        for (j = 0; j < img->width / MIN_BLOCK_SIZE; j++) {
            for (k = 0; k < BUF_CYCLE; k++) {
                for (l = 0; l < 9; l++) {
                    (*mv) [i][j][k][l] = m;
                    m += 5;
                }
            }
        }
    }

    return img->width / MIN_BLOCK_SIZE * img->height / MIN_BLOCK_SIZE * 2 * 9 * 5 * sizeof(int);

}
int get_mem_FME()
{
    int memory_size = 0;
    memory_size += get_mem2Dint(&McostState, 2 * input->search_range + 1, 2 * input->search_range + 1);
    memory_size += get_mem_mincost(&(he->all_mincost));
    memory_size += get_mem_mincost(&(he->all_bwmincost));
    memory_size += get_mem2D(&SearchState, 7, 7);

    return memory_size;
}

/*
*************************************************************************
* Function:free the memory allocated for of all infomation needed for Fast ME
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_mem_mincost(int ** *** mv)
{
    int i, j, k;

    free(mv[0][0][0][0]);

    for (i = 0; i < img->height / MIN_BLOCK_SIZE; i++) {   //Liwr 0915
        for (j = 0; j < img->width / MIN_BLOCK_SIZE; j++) {
            for (k = 0; k < BUF_CYCLE; k++) {
                free(mv[i][j][k]);
            }

            free(mv[i][j]);
        }

        free(mv[i]);
    }

    free(mv);
}




void free_mem_FME()
{
    free_mem2Dint(McostState);
    free_mem_mincost(he->all_mincost);
    free_mem_mincost(he->all_bwmincost);
    free_mem2D(SearchState);
}




void FME_SetMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  pmv[2], int  **refFrArr,
                                  int  ***tmp_mv, int  ref_frame, int  mb_pix_x, int  mb_pix_y, int  blockshape_x, int  blockshape_y, int blocktype,
                                  int  ref, int  direct_mv)
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
    int upright = 0;
    int SAD_a, SAD_b, SAD_c, SAD_d;
    int temp_pred_SAD[2];
    int mvc_temp;


    blockshape_x = blockshape_x % MIN_BLOCK_SIZE == 0 ? blockshape_x : MIN_BLOCK_SIZE;
    blockshape_y = blockshape_y % MIN_BLOCK_SIZE == 0 ? blockshape_y : MIN_BLOCK_SIZE;

    pred_SAD_space = 0;
    mb_available_up      = (mb_y == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           mb_width  ].slice_nr);       //jlzheng 6.23
    mb_available_left    = (mb_x == 0) ? 0 : (img->mb_data[mb_nr].slice_nr == img->mb_data[mb_nr -
                           1         ].slice_nr);       // jlzheng 6.23
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
        if (((ref_frame == img->num_of_references - 1) && (rFrameL != img->num_of_references - 1) ||
             (ref_frame != img->num_of_references - 1) && (rFrameL == img->num_of_references - 1)) && ((img->type == INTER_IMG) ||
                     (img->type == F_IMG)) && (input->bg_enable)) {
            rFrameL = -1;
        }

        if ((img->type == P_IMG) && (img->typeb == BP_IMG) && (input->bg_enable)) {
            rFrameL = -1;
        }

    }
    if (rFrameU != -1) {
        if (((ref_frame == img->num_of_references - 1) && (rFrameU != img->num_of_references - 1) ||
             (ref_frame != img->num_of_references - 1) && (rFrameU == img->num_of_references - 1)) && ((img->type == P_IMG) ||
                     (img->type == F_IMG)) && (input->bg_enable)) {
            rFrameU = -1;
        }

        if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
            rFrameU = -1;
        }

    }
    if (rFrameUR != -1) {
        if (((ref_frame == img->num_of_references - 1) && (rFrameUR != img->num_of_references - 1) ||
             (ref_frame != img->num_of_references - 1) && (rFrameUR == img->num_of_references - 1)) && ((img->type == P_IMG) ||
                     (img->type == F_IMG)) && (input->bg_enable)) {
            rFrameUR = -1;
        }

        if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
            rFrameUR = -1;
        }

    }
    if (rFrameUL != -1) {
        if (((ref_frame == img->num_of_references - 1) && (rFrameUL != img->num_of_references - 1) ||
             (ref_frame != img->num_of_references - 1) && (rFrameUL == img->num_of_references - 1)) && ((img->type == P_IMG) ||
                     (img->type == F_IMG)) && (input->bg_enable)) {
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
    }

    else if (blockshape_x < blockshape_y) {
        if (mb_pix_x == 0) {
            if (rFrameL == ref_frame) {
                mvPredType = MVPRED_L;
            }
        } else {
            if (rFrameUR == ref_frame) {
                mvPredType = MVPRED_UR;
            }
        }
    }

    else if (blockshape_x > blockshape_y) {
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
#define MEDIAN(a,b,c)  (a + b + c - min(a, min(b, c)) - max(a, max(b, c)));
    for (hv = 0; hv < 2; hv++) {
        mva[hv] = mv_a = block_available_left ? tmp_mv[pic_block_y - off_y][pic_block_x - 1][hv] : 0;
        mvb[hv] = mv_b = block_available_up ? tmp_mv[pic_block_y - y_up][pic_block_x][hv] : 0;
        mv_d = block_available_upleft ? tmp_mv[pic_block_y - y_upleft][pic_block_x - 1][hv] : 0;


        mvc[hv] = mv_c = block_available_upright ? tmp_mv[pic_block_y - y_upright][pic_block_x +
                         (blockshape_x / MIN_BLOCK_SIZE) ][hv] : mv_d;

        if (((rFrameL == -1) && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) || ((rFrameL == -1) &&
                img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) {
            mva[hv] = 0;
        }
        else
#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1 && (rFrameL != -1)) {
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
            if (img->is_field_sequence && hv == 1  && (rFrameL != -1)) {
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

        if (((rFrameU == -1) && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) || ((rFrameU == -1) &&
                img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) {
            mvb[hv] = 0;
        }
        else
#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1 && (rFrameU != -1)) {
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
            if (img->is_field_sequence && hv == 1  && (rFrameU != -1)) {
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
        if (((rFrameUL == -1) && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) || ((rFrameUL == -1) &&
                img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) {
            mv_d = 0;
        }
        else


#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1  && (rFrameUL != -1)) {
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
            if (img->is_field_sequence && hv == 1  && (rFrameUL != -1)) {
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

        if (((rFrameUR == -1) && (img->type == P_IMG || img->type == F_IMG) && input->bg_enable) || ((rFrameUR == -1) &&
                img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) {
            mvc_temp = 0;
        }
        else

#if HALF_PIXEL_COMPENSATION_PMV
        {
#if MV_SCALE
            if (img->is_field_sequence && hv == 1 && (rFrameUR != -1)) {
                mvc_temp = scale_motion_vector_y1(mvc[hv], ref_frame, rFrameUR, ref);
            } else {
#if Mv_Clip
				mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref,0);
#else
                mvc_temp = scale_motion_vector(mvc[hv], ref_frame, rFrameUR, ref);
#endif
            }
#else
            if (img->is_field_sequence && hv == 1  && (rFrameUR != -1)) {
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
            SAD_a = block_available_left ? ((ref == -1) ? he->all_bwmincost[pic_block_y][pic_block_x - 1][0][blocktype][0] :
                                            he->all_mincost[pic_block_y][pic_block_x - 1][0][blocktype][0]) : 0;
            SAD_b = block_available_up ? ((ref == -1) ? he->all_bwmincost[pic_block_y - 1][pic_block_x][0][blocktype][0] :
                                          he->all_mincost[pic_block_y - 1][pic_block_x][0][blocktype][0]) : 0;
            SAD_d = block_available_upleft ? ((ref == -1) ? he->all_bwmincost[pic_block_y - 1][pic_block_x - 1][0][blocktype][0] :
                                              he->all_mincost[pic_block_y - 1][pic_block_x - 1][0][blocktype][0]) : 0;
            SAD_c = block_available_upright ? ((ref == -1) ? he->all_bwmincost[pic_block_y - 1][pic_block_x + 1][0][blocktype][0] :
                                               he->all_mincost[pic_block_y - 1][pic_block_x + 1][0][blocktype][0]) : SAD_d;
            switch (mvPredType) {
#if XY_MIN_PMV
            case MVPRED_xy_MIN:
                if (hv == 1) {
                    //for x component
                    if (((mva[0] < 0) && (mvb[0] > 0) && (mvc[0] > 0)) || (mva[0] > 0) && (mvb[0] < 0) && (mvc[0] < 0)) {
                        pmv[0] = (mvb[0] + mvc[0]) / 2;
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_b;
                    } else if (((mvb[0] < 0) && (mva[0] > 0) && (mvc[0] > 0)) || ((mvb[0] > 0) && (mva[0] < 0) && (mvc[0] < 0))) {
                        pmv[0] = (mvc[0] + mva[0]) / 2;
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_c;
                    } else if (((mvc[0] < 0) && (mva[0] > 0) && (mvb[0] > 0)) || ((mvc[0] > 0) && (mva[0] < 0) && (mvb[0] < 0))) {
                        pmv[0] = (mva[0] + mvb[0]) / 2;
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_a;
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
                            temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_a;
                        } else if (pred_vec == mvb[2]) {
                            pmv[0] = (mvb[0] + mvc[0]) / 2;
                            temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_b;
                        } else {
                            pmv[0] = (mvc[0] + mva[0]) / 2;
                            temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_c;
                        }
                    }

                    //for y component
                    if (((mva[1] < 0) && (mvb[1] > 0) && (mvc[1] > 0)) || (mva[1] > 0) && (mvb[1] < 0) && (mvc[1] < 0)) {
                        pmv[1] = (mvb[1] + mvc[1]) / 2;
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_b;
                    } else if (((mvb[1] < 0) && (mva[1] > 0) && (mvc[1] > 0)) || ((mvb[1] > 0) && (mva[1] < 0) && (mvc[1] < 0))) {
                        pmv[1] = (mvc[1] + mva[1]) / 2;
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_c;
                    } else if (((mvc[1] < 0) && (mva[1] > 0) && (mvb[1] > 0)) || ((mvc[1] > 0) && (mva[1] < 0) && (mvb[1] < 0))) {
                        pmv[1] = (mva[1] + mvb[1]) / 2;
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_a;
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
                            temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_a;
                        } else if (pred_vec == mvb[2]) {
                            pmv[1] = (mvb[1] + mvc[1]) / 2;
                            temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_b;
                        } else {
                            pmv[1] = (mvc[1] + mva[1]) / 2;
                            temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_c;
                        }
                    }

                }

                break;
#else
            case MVPRED_MEDIAN:
                if (hv == 1) {
                    mva[2] = abs(mva[0] - mvb[0]) + abs(mva[1] - mvb[1]) ;
                    mvb[2] = abs(mvb[0] - mvc[0]) + abs(mvb[1] - mvc[1]);
                    mvc[2] = abs(mvc[0] - mva[0]) + abs(mvc[1] - mva[1]) ;
                    pred_vec = MEDIAN(mva[2], mvb[2], mvc[2]);
                    if (pred_vec == mva[2]) {
                        pmv[0] = mvc[0];
                        pmv[1] = mvc[1];
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_a;
                    } else if (pred_vec == mvb[2]) {
                        pmv[0] = mva[0];
                        pmv[1] = mva[1];
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_b;
                    } else {
                        pmv[0] = mvb[0];
                        pmv[1] = mvb[1];
                        temp_pred_SAD[1] = temp_pred_SAD[0] = SAD_c;
                    }// END
                }
                break;
#endif
            case MVPRED_L:
                pred_vec = mva[hv];
                temp_pred_SAD[hv] = SAD_a;
                break;
            case MVPRED_U:
                pred_vec = mvb[hv];
                temp_pred_SAD[hv] = SAD_b;
                break;
            case MVPRED_UR:
                pred_vec = mvc[hv];
                temp_pred_SAD[hv] = SAD_c;
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
            pred_SAD_space = temp_pred_SAD[0] > temp_pred_SAD[1] ? temp_pred_SAD[1] : temp_pred_SAD[0];
        }
    }
#undef  MEDIAN
}



int FME_BlockMotionSearch(int  ref, int pic_pix_x, int pic_pix_y, int blocktype, int search_range, double lambda,
                          int mb_nr, int bit_size , int block)

{
    static pel_t  orig_pic  [MAX_CU_SIZE][MAX_CU_SIZE];

    int       pred_mv_x, pred_mv_y, mv_x, mv_y, i, j;
    int       max_value = (1 << 20);
    int       min_mcost = 0x7FFFFFFF;
    int       number_mb_per_row = img->width / MIN_CU_SIZE;
    int       mb_x = mb_nr % number_mb_per_row;
    int       mb_y = mb_nr / number_mb_per_row;
    int       SubMBpix_x = mb_x * MIN_CU_SIZE;
    int       SubMBpix_y = mb_y * MIN_CU_SIZE;
    int       mb_pix_x, mb_pix_y, b8_x, b8_y, b8_pix_x, b8_pix_y, center_x, center_y, bsx, bsy; //,blocksize_y,blocksize_x;

    int       refframe  = (ref == -1 ? 0 : ref);
    int      *pred_mv;
    int     **ref_array = ((img->type != B_IMG) ? hc->refFrArr : ref >= 0 ? img->fw_refFrArr : img->bw_refFrArr);
    int ***    mv_array  = ((img->type != B_IMG) ? img->tmp_mv   : ref >= 0 ? img->fw_mv    : img->bw_mv);
    int ** ***  allFwMv    = (ref < 0 ? img->allBwMv : img->allFwMv);
    byte    **imgY_org_pic = he->imgY_org;

    int      *allIntegerPFwMv;
    int       refinx = ref + 1;

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    int       N_Bframe = input->successive_Bframe,
              n_Bframe = (N_Bframe) ? ((hc->Bframe_ctr % N_Bframe) ? hc->Bframe_ctr % N_Bframe : N_Bframe) : 0 ;

    int       incr_y = 1, off_y = 0; /*lgp*/
    int m, n;
    mb_pix_x      = pic_pix_x - SubMBpix_x;//start position of PU
    mb_pix_y      = pic_pix_y - SubMBpix_y;
    b8_x          = mb_pix_x == 0 ? 0 : 1; // PU index
    b8_y          = mb_pix_y == 0 ? 0 : 1;
    b8_pix_x      = (mb_pix_x >> MIN_BLOCK_SIZE_IN_BIT);  // PU 8x8
    b8_pix_y      = (mb_pix_y >> MIN_BLOCK_SIZE_IN_BIT);
    bsx           = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    bsy           = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);

    if (bit_size == MIN_CU_SIZE_IN_BIT && (blocktype >= 4 || blocktype <= 7)) {
        //if 16x4 then  set to 16x8
        b8_pix_x = b8_x;
        b8_pix_y = b8_y;
    }
    center_x = pic_pix_x;
    center_y = pic_pix_y;

    pred_mv = ((img->type != B_IMG) ? img->mv  : ref >= 0 ? img->predBFwMv :
               img->predBBwMv) [b8_y][b8_x][refframe][blocktype];

    allIntegerPFwMv = ((img->type != B_IMG) ? img->allIntegerPFwMv  : ref >= 0 ? img->allIntegerBFwMv :
                       img->allIntegerBBwMv) [b8_y][b8_x][refframe][blocktype];

    //==================================
    //=====   GET ORIGINAL BLOCK   =====
    //==================================
    for (j = 0; j < bsy; j++) {
        for (i = 0; i < bsx; i++) {
            orig_pic[j][i] = imgY_org_pic[pic_pix_y + incr_y * j + off_y][pic_pix_x + i];
        }
    }

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3

    if (blocktype == PNXN)

    {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][2][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][2][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][2][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][2][0]);
        pred_SAD_uplayer   /= 2;
    } else if (blocktype == PHOR_UP || blocktype == PHOR_DOWN) {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][2][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][2][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][2][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][2][0]);
        pred_SAD_uplayer   /= 2;

    } else if (blocktype == PVER_LEFT || blocktype == PVER_RIGHT) {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][3][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][3][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][3][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][3][0]);
        pred_SAD_uplayer   /= 2;

    }

    else if (blocktype > P2NX2N) {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][1][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][1][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][1][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][1][0]);
        pred_SAD_uplayer   /= 2;
    }

    pred_SAD_uplayer = flag_intra_SAD ? 0 : pred_SAD_uplayer;// for irregular motion
    //coordinate prediction
    if (img->number > refframe + 1) {
        pred_MV_time[0] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                          MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][0][3];
        pred_MV_time[1] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                          MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][0][4];
    }

    if (ref == -1 && hc->Bframe_ctr > 1) {
        pred_MV_time[0] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][0][3] * ((n_Bframe == 1) ? (N_Bframe) : (N_Bframe - n_Bframe + 1.0) /
                                        (N_Bframe - n_Bframe + 2.0)));     //should add a factor
        pred_MV_time[1] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][0][4] * ((n_Bframe == 1) ? (N_Bframe) : (N_Bframe - n_Bframe + 1.0) /
                                        (N_Bframe - n_Bframe + 2.0)));     //should add a factor
    }

    {
        if (refframe > 0) {

            pred_SAD_ref = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                           b8_pix_x][(refframe - 1)][blocktype][0];
            pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
            pred_MV_ref[0] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT)
                             + b8_pix_x ][(refframe - 1) ][blocktype][1];
            pred_MV_ref[0] = (int)(pred_MV_ref[0] * (refframe + 1) / (double)(refframe));
            pred_MV_ref[1] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT)
                             + b8_pix_x ][(refframe - 1) ][blocktype][2];
            pred_MV_ref[1] = (int)(pred_MV_ref[1] * (refframe + 1) / (double)(refframe));
        }

        if (img->type == B_IMG && ref == 0) {
            pred_SAD_ref = he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT)
                           + b8_pix_x][0][blocktype][0];
            pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
            pred_MV_ref[0] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                   MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][blocktype][1] * (-n_Bframe) / (N_Bframe - n_Bframe +
                                           1.0f));   //should add a factor
            pred_MV_ref[1] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                   MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][blocktype][2] * (-n_Bframe) / (N_Bframe - n_Bframe + 1.0f));
        }
    }


    //===========================================
    //=====   GET MOTION VECTOR PREDICTOR   =====
    //===========================================

    FME_SetMotionVectorPredictor(bit_size, mb_nr, pred_mv, ref_array, mv_array, refframe, mb_pix_x, mb_pix_y, bsx, bsy,
                                 blocktype,  ref, 0);

    pred_mv_x = pred_mv[0];
    pred_mv_y = pred_mv[1];


    //==================================
    //=====   INTEGER-PEL SEARCH   =====
    //==================================

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;

    min_mcost = FastIntegerPelBlockMotionSearch(orig_pic, ref, center_x, center_y, blocktype,
                pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                min_mcost, lambda, mb_nr, bit_size, block);



    allIntegerPFwMv[0] = mv_x;
    allIntegerPFwMv[1] = mv_y;
    int_motion_cost[refinx] = min_mcost;


    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3

    m = (bsx >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsx >> MIN_BLOCK_SIZE_IN_BIT);
    n = (bsy >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsy >> MIN_BLOCK_SIZE_IN_BIT);
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {

            if (ref > -1) {
                he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x +
                        i ][refframe][blocktype][0] = min_mcost;
            } else {
                he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][0][blocktype][0] = min_mcost;
            }
        }
    }

    if (!(img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) { //no motion search for S frame

        //==============================
        //=====   SUB-PEL SEARCH   =====
        //==============================
        if (input->hadamard) {
            min_mcost = 0x7FFFFFFF;
        }



        if (blocktype > 3) {

            min_mcost =  FastSubPelBlockMotionSearch(orig_pic, ref, center_x, center_y, blocktype,
                         pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                         min_mcost, lambda, 0, mb_nr , bit_size, block);

        } else {

            min_mcost =  SubPelBlockMotionSearch(orig_pic, ref, center_x, center_y, blocktype,
                                                 pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                                 min_mcost, lambda, bit_size, block);

        }

    }



    m = (bsx >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsx >> MIN_BLOCK_SIZE_IN_BIT);
    n = (bsy >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsy >> MIN_BLOCK_SIZE_IN_BIT);
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {

            if (ref > -1) {
                he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x
                        + i ][refframe][blocktype][1] = mv_x;
                he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x
                        + i ][refframe][blocktype][2] = mv_y;
            } else {
                he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][0][blocktype][1] = mv_x;
                he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][0][blocktype][2] = mv_y;
            }
        }
    }


    //===============================================
    //=====   SET MV'S AND RETURN MOTION COST   =====
    //===============================================

    m = (bsx >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 0 ? 1 : (bsx >>
            (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT));
    n = (bsy >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 0 ? 1 : (bsy >>
            (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT));
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {

            allFwMv[b8_y + j][b8_x + i][refframe][blocktype][0] = mv_x;
            allFwMv[b8_y + j][b8_x + i][refframe][blocktype][1] = mv_y;
        }
    }
#if Mv_check_bug
	img->mv_range_flag = check_mv_range(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
	img->mv_range_flag = check_mv_range(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype);
#endif
    
    img->mv_range_flag *= check_mvd((mv_x - pred_mv_x), (mv_y - pred_mv_y));

    if (!img->mv_range_flag) {
#if RD160_FIX_BG
		 min_mcost = 0x0EEEEEEE;
#else
        min_mcost = 0x7FFFFFFF;
#endif
        img->mv_range_flag = 1;
    }

    return min_mcost;
}

int PartCalMad(pel_t *ref_pic, pel_t orig_pic[MAX_CU_SIZE][MAX_CU_SIZE], pel_t * (*get_ref_line)(int,
               pel_t *, int, int), int blocksize_y, int blocksize_x, int blocksize_x4, int mcost, int min_mcost, int cand_x,
               int cand_y)
{
    int y, x4;
    pel_t *orig_line, *ref_line;

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

    return mcost;
}


/*
*************************************************************************
* Function: FastIntegerPelBlockMotionSearch: fast pixel block motion search
this algrithm is called UMHexagonS(see JVT-D016),which includes
four steps with different kinds of search patterns
* Input:
pel_t**   orig_pic,     // <--  original picture
int       ref,          // <--  reference frame (0... or -1 (backward))
int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
int*      mv_x,         //  --> motion vector (x) - in pel units
int*      mv_y,         //  --> motion vector (y) - in pel units
int       search_range, // <--  1-d search range in pel units
int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
double    lambda        // <--  lagrangian parameter for determining motion cost
* Output:
* Return:
* Attention: in this function, three macro definitions is gives,
EARLY_TERMINATION: early termination algrithm, refer to JVT-D016.doc
SEARCH_ONE_PIXEL: search one pixel in search range
SEARCH_ONE_PIXEL1(value_iAbort): search one pixel in search range,
but give a parameter to show if mincost refeshed
*************************************************************************
*/


int                                     //  ==> minimum motion cost after search
FastIntegerPelBlockMotionSearch(pel_t     orig_pic[MAX_CU_SIZE][MAX_CU_SIZE],       // <--  not used
                                int       ref,          // <--  reference frame (0... or -1 (backward))
                                int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                                int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                                int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                                int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                                int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                                int      *mv_x,         //  --> motion vector (x) - in pel units
                                int      *mv_y,         //  --> motion vector (y) - in pel units
                                int       search_range, // <--  1-d search range in pel units
                                int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                                double    lambda,
                                int mb_nr ,
                                int bit_size,
                                int block)       // <--  lagrangian parameter for determining motion cost

{
    static int Diamond_x[4] = { -1, 0, 1, 0};
    static int Diamond_y[4] = {0, 1, 0, -1};
    static int Hexagon_x[6] = {2, 1, -1, -2, -1, 1};
    static int Hexagon_y[6] = {0, -2, -2, 0,  2, 2};
    static int Big_Hexagon_x[16] = {0, -2, -4, -4, -4, -4, -4, -2,  0,  2,  4,  4, 4, 4, 4, 2};
    static int Big_Hexagon_y[16] = {4, 3, 2,  1, 0, -1, -2, -3, -4, -3, -2, -1, 0, 1, 2, 3};

    int   pos, cand_x, cand_y,  mcost;
    pel_t * (*get_ref_line)(int, pel_t *, int, int);
    pel_t  *ref_pic     = img->type == B_IMG ? he->Refbuf11 [ref + 1] : he->Refbuf11[ref];
    int   lambda_factor = LAMBDA_FACTOR(lambda);                    // factor for determining lagragian motion cost
    int   mvshift       = 2;                  // motion vector shift for getting sub-pel units
    int   blocksize_y   = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_x   = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_x4  = blocksize_x >> 2;                         // horizontal block size in 4-pel units
    int   pred_x        = (pic_pix_x << mvshift) + pred_mv_x;       // predicted position x (in sub-pel units)
    int   pred_y        = (pic_pix_y << mvshift) + pred_mv_y;       // predicted position y (in sub-pel units)
    int   center_x      = pic_pix_x + *mv_x;                        // center position x (in pel units)
    int   center_y      = pic_pix_y + *mv_y;                        // center position y (in pel units)
    int   best_x = 0, best_y = 0;

    int   search_step, iYMinNow, iXMinNow;
    int   i, m;
    int   iAbort;

    int   best_pos      = 0;                                        // position with minimum motion cost
    int   max_pos       = (2 * search_range + 1) * (2 * search_range + 1); // number of search positions
    double betaSec, betaThird;
    int   best_pos_x, best_pos_y;


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
        get_ref_line = FastLineX;
    } else {
        get_ref_line = UMVLineX;
    }
    memset(McostState[0], 0, (2 * search_range + 1) * (2 * search_range + 1) * 4);

    ///////////////////////////////////////////////////////////////

    if (ref > 0) {
        if (pred_SAD_ref != 0) {
            betaSec = Bsize[blocktype] / (pred_SAD_ref * pred_SAD_ref) - AlphaSec[blocktype];
            betaThird = Bsize[blocktype] / (pred_SAD_ref * pred_SAD_ref) - AlphaThird[blocktype];
        } else {
            betaSec = 0;
            betaThird = 0;
        }
    } else {
        if (blocktype == 1) {
            if (pred_SAD_space != 0) {
                betaSec = Bsize[blocktype] / (pred_SAD_space * pred_SAD_space) - AlphaSec[blocktype];
                betaThird = Bsize[blocktype] / (pred_SAD_space * pred_SAD_space) - AlphaThird[blocktype];
            } else {
                betaSec = 0;
                betaThird = 0;
            }
        } else {
            if (pred_SAD_uplayer != 0) {
                betaSec = Bsize[blocktype] / (pred_SAD_uplayer * pred_SAD_uplayer) - AlphaSec[blocktype];
                betaThird = Bsize[blocktype] / (pred_SAD_uplayer * pred_SAD_uplayer) - AlphaThird[blocktype];
            } else {
                betaSec = 0;
                betaThird = 0;
            }
        }
    }
    /*****************************/

    ////////////search around the predictor and (0,0)
    //check the center median predictor
    cand_x = center_x ;
    cand_y = center_y ;
    mcost = MV_COST(lambda_factor, mvshift, cand_x, cand_y, pred_x, pred_y);
    mcost = PartCalMad(ref_pic, orig_pic, get_ref_line, blocksize_y, blocksize_x, blocksize_x4, mcost, min_mcost, cand_x,
                       cand_y);
    McostState[search_range][search_range] = mcost;

    if (mcost < min_mcost) {
        min_mcost = mcost;
        best_x = cand_x;
        best_y = cand_y;
    }

    if (!(img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) { //no motion search for S frame

        iXMinNow = best_x;
        iYMinNow = best_y;
        best_pos_x = best_x;
        best_pos_y = best_y;

        for (m = 0; m < 4; m++) {
            cand_x = iXMinNow + Diamond_x[m];
            cand_y = iYMinNow + Diamond_y[m];
            SEARCH_ONE_PIXEL
        }

        if (center_x != pic_pix_x || center_y != pic_pix_y) {
            cand_x = pic_pix_x ;
            cand_y = pic_pix_y ;
            SEARCH_ONE_PIXEL


            if (best_x != best_pos_x || best_y != best_pos_y) {
                iXMinNow = best_x;
                iYMinNow = best_y;
                best_pos_x = best_x;
                best_pos_y = best_y;
                for (m = 0; m < 4; m++) {
                    cand_x = iXMinNow + Diamond_x[m];
                    cand_y = iYMinNow + Diamond_y[m];
                    SEARCH_ONE_PIXEL
                }
            }
        }

        if (blocktype > 1) {
            cand_x = pic_pix_x + (pred_MV_uplayer[0] / 4);
            cand_y = pic_pix_y + (pred_MV_uplayer[1] / 4);
            SEARCH_ONE_PIXEL

            if ((min_mcost - pred_SAD_uplayer) < pred_SAD_uplayer * betaThird) {
                goto third_step;
            } else if ((min_mcost - pred_SAD_uplayer) < pred_SAD_uplayer * betaSec) {
                goto sec_step;
            }
        }

        //coordinate position prediction
        if ((img->number > 1 + ref && ref != -1) || (ref == -1 && hc->Bframe_ctr > 1)) {
            cand_x = pic_pix_x + pred_MV_time[0] / 4;
            cand_y = pic_pix_y + pred_MV_time[1] / 4;
            SEARCH_ONE_PIXEL
        }

        ////prediciton using mV of last ref moiton vector
        if ((ref > 0) || (img->type == B_IMG && ref == 0)) {
            cand_x = pic_pix_x + pred_MV_ref[0] / 4;
            cand_y = pic_pix_y + pred_MV_ref[1] / 4;
            SEARCH_ONE_PIXEL
        }


        if (best_x != best_pos_x || best_y != best_pos_y) {
            iXMinNow = best_x;
            iYMinNow = best_y;
            best_pos_x = best_x;
            best_pos_y = best_y;
            for (m = 0; m < 4; m++) {
                cand_x = iXMinNow + Diamond_x[m];
                cand_y = iYMinNow + Diamond_y[m];
                SEARCH_ONE_PIXEL
            }
        }

        //early termination algrithm, refer to JVT-D016
        EARLY_TERMINATION

        if (blocktype > 7) { //PNXN
            goto sec_step;
        } else {
            goto first_step;
        }
first_step:

        //Unsymmetrical-cross search
        iXMinNow = best_x;
        iYMinNow = best_y;

        for (i = 1; i <= search_range / 2; i++) {
            search_step = 2 * i - 1;
            cand_x = iXMinNow + search_step;
            cand_y = iYMinNow ;
            SEARCH_ONE_PIXEL
            cand_x = iXMinNow - search_step;
            cand_y = iYMinNow ;
            SEARCH_ONE_PIXEL
        }

        for (i = 1; i <= search_range / 4; i++) {
            search_step = 2 * i - 1;
            cand_x = iXMinNow ;
            cand_y = iYMinNow + search_step;
            SEARCH_ONE_PIXEL
            cand_x = iXMinNow ;
            cand_y = iYMinNow - search_step;
            SEARCH_ONE_PIXEL
        }

        //early termination algrithm, refer to JVT-D016

        EARLY_TERMINATION
        iXMinNow = best_x;
        iYMinNow = best_y;

        // Uneven Multi-Hexagon-grid Search
        for (pos = 1; pos < 25; pos++) {
            cand_x = iXMinNow + spiral_search_x[pos];
            cand_y = iYMinNow + spiral_search_y[pos];
            SEARCH_ONE_PIXEL
        }

        //early termination algrithm, refer to JVT-D016

        EARLY_TERMINATION

        for (i = 1; i <= search_range / 4; i++) {
            iAbort = 0;

            for (m = 0; m < 16; m++) {
                cand_x = iXMinNow + Big_Hexagon_x[m] * i;
                cand_y = iYMinNow + Big_Hexagon_y[m] * i;
                SEARCH_ONE_PIXEL1(1)
            }
            if (iAbort) {
                EARLY_TERMINATION
            }

        }

sec_step:
        //Extended Hexagon-based Search
        iXMinNow = best_x;
        iYMinNow = best_y;

        for (i = 0; i < search_range; i++) {
            iAbort = 1;

            for (m = 0; m < 6; m++) {
                cand_x = iXMinNow + Hexagon_x[m];
                cand_y = iYMinNow + Hexagon_y[m];
                SEARCH_ONE_PIXEL1(0)
            }

            if (iAbort) {
                break;
            }

            iXMinNow = best_x;
            iYMinNow = best_y;
        }

third_step:
        // the third step with a small search pattern
        iXMinNow = best_x;
        iYMinNow = best_y;

        for (i = 0; i < search_range; i++) {
            iAbort = 1;

            for (m = 0; m < 4; m++) {
                cand_x = iXMinNow + Diamond_x[m];
                cand_y = iYMinNow + Diamond_y[m];
                SEARCH_ONE_PIXEL1(0)
            }

            if (iAbort) {
                break;
            }

            iXMinNow = best_x;
            iYMinNow = best_y;
        }
    }


    *mv_x = best_x - pic_pix_x;
    *mv_y = best_y - pic_pix_y;
    return min_mcost;
}

int pmvr_adapt_mv2(int *cand_mv_x, int *cand_mv_y, int ctr_x, int ctr_y,
                   int iXMinNow, int iYMinNow, int step_x, int step_y)
{
    if (abs(iXMinNow - ctr_x) > TH || abs(iYMinNow - ctr_y) > TH) {
        *cand_mv_x = iXMinNow + step_x * 2;
        *cand_mv_y = iYMinNow + step_y * 2;
        return (abs(*cand_mv_x - ctr_x) > TH || abs(*cand_mv_y - ctr_y) > TH);
    } else {
        *cand_mv_x = iXMinNow + step_x;
        *cand_mv_y = iYMinNow + step_y;
        return (abs(*cand_mv_x - ctr_x) <= TH && abs(*cand_mv_y - ctr_y) <= TH);
    }
}

/*
*************************************************************************
* Function: Functions for fast fractional pel motion estimation.
1. int AddUpSADQuarter() returns SADT of a fractiona pel MV
2. int FastSubPelBlockMotionSearch () proceed the fast fractional pel ME
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


int AddUpSADQuarter(int pic_pix_x, int pic_pix_y, int blocksize_x, int blocksize_y,
                    int cand_mv_x, int cand_mv_y, pel_t **ref_pic, pel_t   orig_pic[MAX_CU_SIZE][MAX_CU_SIZE], int Mvmcost, int min_mcost,
                    int useABT)
{
    int abort_search, y0, x0, rx0, ry0, ry;
    pel_t *orig_line;
    int   diff[16], *d;
    int  mcost = Mvmcost;
    int yy, kk, xx;
    int   curr_diff[64][64]; // for ABT SATD calculation

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

    return mcost;//mcost;


}

int                                                   //  ==> minimum motion cost after search
FastSubPelBlockMotionSearch(pel_t
                            orig_pic[MAX_CU_SIZE][MAX_CU_SIZE],     // <--  original pixel values for the AxB block
                            int       ref,           // <--  reference frame (0... or -1 (backward))
                            int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                            int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                            int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                            int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                            int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                            int      *mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                            int      *mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                            int       search_pos2,   // <--  search positions for    half-pel search  (default: 9)
                            int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                            int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                            double    lambda,
                            int  useABT,
                            int mb_nr,
                            int bit_size,
                            int block)        // <--  lagrangian parameter for determining motion cost

{
    static int Diamond_x[4] = { -1, 0, 1, 0};
    static int Diamond_y[4] = {0, 1, 0, -1};
    int   mcost;
    int   cand_mv_x, cand_mv_y;

    int   incr            = 0 ;//qyu 0926
    pel_t **ref_pic = img->type == B_IMG ? fref[ref + 1 + incr]->oneForthRefY : fref[ref]->oneForthRefY; //Liwr 0915

    int   lambda_factor   = LAMBDA_FACTOR(lambda);
    int   mv_shift        = 0;
    int   blocksize_x     = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_y     = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   pic4_pix_x      = (pic_pix_x << 2);
    int   pic4_pix_y      = (pic_pix_y << 2);
    int   max_pos_x4      = ((img->width - blocksize_x) << 2);
    int   max_pos_y4      = ((img->height - blocksize_y) << 2);
    int   search_range_dynamic, iXMinNow, iYMinNow, i;
    int   m, currmv_x = 0, currmv_y = 0, iCurrSearchRange;
    int   pred_frac_mv_x, pred_frac_mv_y, abort_search;
    int   mv_cost;
    int   ctr_x = (pred_mv_x >> 1) << 1;
    int   ctr_y = (pred_mv_y >> 1) << 1;

    if (img->type == B_IMG) {
        incr = 1;
    }

    ref_pic       = img->type == B_IMG ? fref[ref + incr]->oneForthRefY : fref[ref]->oneForthRefY;

    if (input->bg_enable && he->background_reference_enable &&
        ref == img->num_of_references - 1 &&
        (img->type == P_IMG || img->type == F_IMG) && img->typeb != BP_IMG) {
        ref_pic = he->background_frame_quarter;
    }

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        ref_pic = he->background_frame_quarter;
    }


    *mv_x <<= 2;
    *mv_y <<= 2;

    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)) {
        PelY_14 = FastPelY_14;
    } else {
        PelY_14 = UMVPelY_14;
    }

    search_range_dynamic = 3;
    pred_frac_mv_x = (pred_mv_x - *mv_x) % 4;
    pred_frac_mv_y = (pred_mv_y - *mv_y) % 4;
    memset(SearchState[0], 0, (2 * search_range_dynamic + 1) * (2 * search_range_dynamic + 1) * sizeof(byte));

    if (input->hadamard) {
        cand_mv_x = *mv_x;
        cand_mv_y = *mv_y;
        mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
        mcost = AddUpSADQuarter(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic, orig_pic,
                                mv_cost, min_mcost, useABT);
        SearchState[search_range_dynamic][search_range_dynamic] = 1;
#if Mv_check_bug
		img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
		img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype);
#endif
        
        if (!img->mv_range_flag) {
#if RD160_FIX_BG
			min_mcost = 0x0EEEEEEE;
#else
			min_mcost = 0x7FFFFFFF;
#endif
            img->mv_range_flag = 1;
        }

        if (mcost < min_mcost) {
            min_mcost = mcost;
            currmv_x = cand_mv_x;
            currmv_y = cand_mv_y;
        }
    } else {
        SearchState[search_range_dynamic][search_range_dynamic] = 1;
        currmv_x = *mv_x;
        currmv_y = *mv_y;
    }

    if (pred_frac_mv_x != 0 || pred_frac_mv_y != 0) {
        cand_mv_x = *mv_x + pred_frac_mv_x;
        cand_mv_y = *mv_y + pred_frac_mv_y;
        if (!input->b_pmvr_enabled || abs(cand_mv_x - ctr_x) <= TH && abs(cand_mv_y - ctr_y) <= TH) { //jcma
            mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
            mcost = AddUpSADQuarter(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic, orig_pic,
                                    mv_cost, min_mcost, useABT);
            SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;
#if Mv_check_bug
			img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, 0, 0);
#else
			img->mv_range_flag = check_mv_range(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype);
#endif
            
            if (!img->mv_range_flag) {
#if RD160_FIX_BG
				min_mcost = 0x0EEEEEEE;
#else
				min_mcost = 0x7FFFFFFF;
#endif
               
                img->mv_range_flag = 1;
            }

            if (mcost < min_mcost) {
                min_mcost = mcost;
                currmv_x = cand_mv_x;
                currmv_y = cand_mv_y;
            }
        }
    }


    iXMinNow = currmv_x;
    iYMinNow = currmv_y;
    iCurrSearchRange = 2 * search_range_dynamic + 1;

    for (i = 0; i < iCurrSearchRange; i++) {
        abort_search = 1;

        for (m = 0; m < 4; m++) {
            if (input->b_pmvr_enabled) {
                if (!pmvr_adapt_mv2(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, iXMinNow, iYMinNow, Diamond_x[m], Diamond_y[m])) {
                    continue;
                }
            } else {
                cand_mv_x = iXMinNow + Diamond_x[m];
                cand_mv_y = iYMinNow + Diamond_y[m];
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

            if (abs(cand_mv_x - *mv_x) <= search_range_dynamic && abs(cand_mv_y - *mv_y) <= search_range_dynamic) {
                if (!SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic]) {
                    mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
                    mcost = AddUpSADQuarter(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic, orig_pic,
                                            mv_cost, min_mcost, useABT);
                    SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;

                    if (mcost < min_mcost) {
                        min_mcost = mcost;
                        currmv_x = cand_mv_x;
                        currmv_y = cand_mv_y;
                        abort_search = 0;

                    }
                }
            }
        }

        iXMinNow = currmv_x;
        iYMinNow = currmv_y;

        if (abort_search) {
            break;
        }
    }

    *mv_x = currmv_x;
    *mv_y = currmv_y;

    //===== return minimum motion cost =====
    return min_mcost;
}
/*
*************************************************************************
* Function:Functions for SAD prediction of intra block cases.
1. void   decide_intrabk_SAD() judges the block coding type(intra/inter)
of neibouring blocks
2. void skip_intrabk_SAD() set the SAD to zero if neigouring block coding
type is intra
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void   decide_intrabk_SAD(int mb_nr)
{
    int       number_mb_per_row = img->width / MIN_CU_SIZE;
    int       mb_x = mb_nr % number_mb_per_row;
    int       SubMBpix_x = mb_x * MIN_CU_SIZE;
    if (img->type != 0) {
        if (img->pix_x == 0 && img->pix_y == 0) {
            flag_intra_SAD = 0;
        } else if (img->pix_x == 0) {
            flag_intra_SAD = flag_intra[SubMBpix_x >> MIN_CU_SIZE_IN_BIT];
        } else if (img->pix_y == 0) {
            flag_intra_SAD = flag_intra[(SubMBpix_x >> MIN_CU_SIZE_IN_BIT) - 1];
        } else {
            flag_intra_SAD = ((flag_intra[(SubMBpix_x) >> MIN_CU_SIZE_IN_BIT]) ||
                              (flag_intra[((SubMBpix_x) >> MIN_CU_SIZE_IN_BIT) - 1]) || (flag_intra[((SubMBpix_x) >> MIN_CU_SIZE_IN_BIT) + 1])) ;
        }
    }
    return;
}
void skip_intrabk_SAD(int best_mode, int ref_max, int mb_nr, int uiBitSize)
{
    int i, j, k, ref;
    int       number_mb_per_row = img->width / MIN_CU_SIZE;
    int       mb_x = mb_nr % number_mb_per_row;
    int       mb_y = mb_nr / number_mb_per_row;
    int       SubMBpix_x = mb_x * MIN_CU_SIZE;
    int       SubMBpix_y = mb_y * MIN_CU_SIZE;
    int       N8_SizeScale = 1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT);
    if (img->number > 0) {
        flag_intra[SubMBpix_x >> MIN_CU_SIZE_IN_BIT] = (best_mode == 9 || best_mode == 10) ? 1 : 0;
    }
    if (img->type != 0  && (best_mode == 9 || best_mode == 10)) {
        for (i = 0; i < N8_SizeScale; i++) {
            for (j = 0; j < N8_SizeScale; j++) {
                for (k = 1; k < 8; k++) {
                    for (ref = 0; ref < ref_max; ref++) {
                        he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + i][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + j][ref][k][0] = 0;
                    }
                }
            }
        }
    }
    return;
}



int                                         //  ==> minimum motion cost after search
FME_BlockMotionSearch_sym(int       ref,            // <--  reference frame (0... or -1 (backward))
                          int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                          int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                          int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                          int       search_range,  // <--  1-d search range for integer-position search
                          double    lambda ,// <--  lagrangian parameter for determining motion cost
                          int mb_nr,
                          int bit_size,
                          int block,
                          int *mcost,
                          int *mcost_bid
                         )

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
    int       SubMBpix_x = mb_x * MIN_CU_SIZE;
    int       SubMBpix_y = mb_y * MIN_CU_SIZE;


    int       mb_pix_x, mb_pix_y, b8_x, b8_y, b8_pix_x, b8_pix_y, center_x, center_y, bsx, bsy; //,blocksize_y,blocksize_x;

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

    byte    **imgY_org_pic = he->imgY_org;

    int      *allIntegerPFwMv;

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    int       N_Bframe = input->successive_Bframe,
              n_Bframe = (N_Bframe) ? ((hc->Bframe_ctr % N_Bframe) ? hc->Bframe_ctr % N_Bframe : N_Bframe) : 0 ;

    int       incr_y = 1, off_y = 0; /*lgp*/
    int m, n;


    mb_pix_x      = pic_pix_x - SubMBpix_x;//start position of PU
    mb_pix_y      = pic_pix_y - SubMBpix_y;
    b8_x          = mb_pix_x == 0 ? 0 : 1; // PU index
    b8_y          = mb_pix_y == 0 ? 0 : 1;
    b8_pix_x      = (mb_pix_x >> MIN_BLOCK_SIZE_IN_BIT);   // PU 8x8
    b8_pix_y      = (mb_pix_y >> MIN_BLOCK_SIZE_IN_BIT);
    bsx           = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    bsy           = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);
    //  blocksize_y   = g_blk_size[blocktype][1] << ( bit_size - 4 );        // vertical block size
    //  blocksize_x   = g_blk_size[blocktype][0] << ( bit_size - 4 );

    if (bit_size == 4 && (blocktype >= 4 || blocktype <= 7)) {
        //if 16x4 then  set to 16x8
        b8_pix_x = b8_x;
        b8_pix_y = b8_y;
    }
    center_x = pic_pix_x;
    center_y = pic_pix_y;

    pred_mv = ((img->type != B_IMG) ? img->mv  : ref >= 0 ? img->predSymMv   :
               img->predBBwMv) [b8_y][b8_x][refframe][blocktype];

    allIntegerPFwMv = ((img->type != B_IMG) ? img->allIntegerPFwMv  : ref >= 0 ? img->allIntegerBFwMv :
                       img->allIntegerBBwMv) [b8_y][b8_x][refframe][blocktype];


    pred_fmv_bid = ((img->type != B_IMG) ? img->mv : img->predBidFwMv) [b8_y][b8_x][refframe][blocktype];;
    pred_bmv_bid = ((img->type != B_IMG) ? img->mv : img->predBidBwMv) [b8_y][b8_x][refframe][blocktype];

    //==================================
    //=====   GET ORIGINAL BLOCK   =====
    //==================================
    for (j = 0; j < bsy; j++) {
        for (i = 0; i < bsx; i++) {
            orig_pic[j][i] = imgY_org_pic[pic_pix_y +/*j*/incr_y * j + off_y/*lgp*/][pic_pix_x + i];
            //      orig_pic[j][i] = imgY_org_pic[pic_pix_y+j][pic_pix_x+i];
        }
    }

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3

    if (blocktype == PNXN)

    {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][2][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][2][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y ][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][2][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y ][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][2][0]);
        pred_SAD_uplayer   /= 2;

    }

    else if (blocktype == 4 || blocktype == 5) {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][2][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][2][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][2][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][2][0]);
        pred_SAD_uplayer   /= 2;
    } else if (blocktype == 6 || blocktype == 7) {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][3][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][3][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][3][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][3][0]);
        pred_SAD_uplayer   /= 2;
    } else if (blocktype > 1) {
        pred_MV_uplayer[0] = allFwMv[b8_y][b8_x][refframe][1][0];
        pred_MV_uplayer[1] = allFwMv[b8_y][b8_x][refframe][1][1];
        pred_SAD_uplayer    = (ref == -1) ? (he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y ][(SubMBpix_x >>
                                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][1][0]) : (he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) +
                                                     b8_pix_y ][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][refframe][1][0]);
        pred_SAD_uplayer   /= 2;
    }

    pred_SAD_uplayer = flag_intra_SAD ? 0 : pred_SAD_uplayer;// for irregular motion
    //coordinate prediction
    if (img->number > refframe + 1) {
        pred_MV_time[0] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                          MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][0][3];
        pred_MV_time[1] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                          MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][0][4];
    }

    if (ref == -1 && hc->Bframe_ctr > 1) {
        pred_MV_time[0] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][0][3] * ((n_Bframe == 1) ? (N_Bframe) : (N_Bframe - n_Bframe + 1.0) /
                                        (N_Bframe - n_Bframe + 2.0)));     //should add a factor
        pred_MV_time[1] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][0][4] * ((n_Bframe == 1) ? (N_Bframe) : (N_Bframe - n_Bframe + 1.0) /
                                        (N_Bframe - n_Bframe + 2.0)));     //should add a factor
    }

    {
        if (refframe > 0) {

            pred_SAD_ref = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                           MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][(refframe - 1)][blocktype][0];
            pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
            pred_MV_ref[0] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][(refframe - 1) ][blocktype][1];
            pred_MV_ref[0] = (int)(pred_MV_ref[0] * (refframe + 1) / (double)(refframe));
            pred_MV_ref[1] = he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                             MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][(refframe - 1) ][blocktype][2];
            pred_MV_ref[1] = (int)(pred_MV_ref[1] * (refframe + 1) / (double)(refframe));
        }

        if (img->type == B_IMG && ref == 0) {

            pred_SAD_ref = he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                           MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][blocktype][0];
            pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
            pred_MV_ref[0] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                   MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x][0][blocktype][1] * (-n_Bframe) / (N_Bframe - n_Bframe +
                                           1.0f));   //should add a factor
            pred_MV_ref[1] = (int)(he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y][(SubMBpix_x >>
                                   MIN_BLOCK_SIZE_IN_BIT) + b8_pix_x ][0][blocktype][2] * (-n_Bframe) / (N_Bframe - n_Bframe + 1.0f));
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

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;

    bmv_x_bid = allBwMv[b8_y][b8_x][0][blocktype][0];
    bmv_y_bid = allBwMv[b8_y][b8_x][0][blocktype][1];
    fmv_x_bid = pred_fmv_x_bid / 4;
    fmv_y_bid = pred_fmv_y_bid / 4;



    mv_x = allIntegerPFwMv[0];
    mv_y = allIntegerPFwMv[1];
    min_mcost = int_motion_cost[1];

    fmv_x_bid = mv_x;
    fmv_y_bid = mv_y;
    min_mcost_bid = min_mcost;



    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3

    m = (bsx >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsx >> MIN_BLOCK_SIZE_IN_BIT);
    n = (bsy >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsy >> MIN_BLOCK_SIZE_IN_BIT);
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {

            if (ref > -1) {
                he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][refframe][blocktype][0] = min_mcost;
            } else {
                he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][0][blocktype][0] = min_mcost;
            }
        }
    }


    //==============================
    //=====   SUB-PEL SEARCH   =====
    //==============================
    if (input->hadamard) {
        min_mcost = 0x7FFFFFFF;

        min_mcost_bid = 0x7FFFFFFF;

    }


    if (blocktype > 3) {

        min_mcost =  FastSubPelBlockMotionSearch_sym(orig_pic, ref, center_x, center_y, blocktype,
                     pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9, min_mcost, lambda, 0, mb_nr, bit_size, block);

        min_mcost_bid =  FastSubPelBlockMotionSearch_bid(orig_pic, ref, center_x, center_y, blocktype,
                         pred_fmv_x_bid, pred_fmv_y_bid, &fmv_x_bid, &fmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid, &bmv_x_bid, &bmv_y_bid, 9, 9,
                         min_mcost_bid, lambda, 0, mb_nr, bit_size, block);

    } else {

        min_mcost =  SubPelBlockMotionSearch_sym(orig_pic, ref, center_x/*lgp*/, center_y/*lgp*/, blocktype,
                     pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9, min_mcost, lambda, bit_size, block);

        min_mcost_bid =  SubPelBlockMotionSearch_bid(orig_pic, ref, center_x, center_y, blocktype,
                         pred_fmv_x_bid, pred_fmv_y_bid, &fmv_x_bid, &fmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid, &bmv_x_bid, &bmv_y_bid, 9, 9,
                         min_mcost_bid, lambda, bit_size, block);

    }

    m = (bsx >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsx >> MIN_BLOCK_SIZE_IN_BIT);
    n = (bsy >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 : (bsy >> MIN_BLOCK_SIZE_IN_BIT);
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {

            if (ref > -1) {
                he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][refframe][blocktype][1] = mv_x;
                he->all_mincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][refframe][blocktype][2] = mv_y;
            } else {
                he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][0][blocktype][1] = mv_x;
                he->all_bwmincost[(SubMBpix_y >> MIN_BLOCK_SIZE_IN_BIT) + b8_pix_y + j][(SubMBpix_x >> MIN_BLOCK_SIZE_IN_BIT) +
                        b8_pix_x + i ][0][blocktype][2] = mv_y;
            }
        }
    }

    //===============================================
    //=====   SET MV'S AND RETURN MOTION COST   =====
    //===============================================


    m = (bsx >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 0 ? 1 : (bsx >>
            (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT));
    n = (bsy >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 0 ? 1 : (bsy >>
            (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT));
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {

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
#if RD160_FIX_BG //jie1222.chen@samsung.com
		min_mcost_bid = 0x0EEEEEEE;
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
		min_mcost_bid = 0x0EEEEEEE;
#else
		min_mcost_bid = 0x7FFFFFFF;
#endif
        img->mv_range_flag = 1;
    }
#if Mv_check_bug
	img->mv_range_flag = check_mv_range_sym(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype, ref, 0, 0);
#else
	img->mv_range_flag = check_mv_range_sym(bit_size, mv_x, mv_y, pic_pix_x, pic_pix_y, blocktype, ref);
#endif

    
    img->mv_range_flag *= check_mvd((mv_x - pred_mv_x), (mv_y - pred_mv_y));

    if (!img->mv_range_flag) {
#if RD160_FIX_BG
		min_mcost = 0x0EEEEEEE;
#else
		min_mcost = 0x7FFFFFFF;
#endif
      //  min_mcost = 0x7FFFFFFF;
        img->mv_range_flag = 1;
    }


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
int                                         //  ==> minimum motion cost after search
FME_BlockMotionSearch_dual(int       ref,            // <--  reference frame (0... or -1 (backward))
                           int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                           int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                           int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                           int       search_range,  // <--  1-d search range for integer-position search
                           double    lambda ,// <--  lagrangian parameter for determining motion cost
                           int mb_nr,
                           int bit_size,
                           int block,
                           int *mcost_dual
                          )
{

    static pel_t  orig_pic  [MAX_CU_SIZE][MAX_CU_SIZE];

    int       i, j;
    int       snd_ref = ref == 0 ? 1 : 0;
    int       max_ref = img->num_of_references;
    int       DistanceIndexFw = 0, DistanceIndexBw = 0;
    int       pred_fst_x_dual, pred_fst_y_dual, pred_snd_x_dual, pred_snd_y_dual;
    int       fst_x_dual, fst_y_dual;

    int       max_value = (1 << 20);
    int       min_mcost_dual = 0x7FFFFFFF;

    int       number_mb_per_row = img->width / MIN_CU_SIZE;
    int       mb_x = mb_nr % number_mb_per_row;
    int       mb_y = mb_nr / number_mb_per_row;
    int       SubMBpix_x = mb_x * MIN_CU_SIZE;
    int       SubMBpix_y = mb_y * MIN_CU_SIZE;

    int       mb_pix_x, mb_pix_y, b8_x, b8_y, b8_pix_x, b8_pix_y, center_x, center_y, bsx, bsy; //,blocksize_y,blocksize_x;

    byte    **imgY_org_pic = he->imgY_org;

    int       incr_y = 1, off_y = 0; /*lgp*/

    int m, n;


    mb_pix_x      = pic_pix_x - SubMBpix_x;//start position of PU
    mb_pix_y      = pic_pix_y - SubMBpix_y;
    b8_x          = mb_pix_x == 0 ? 0 : 1; // PU index
    b8_y          = mb_pix_y == 0 ? 0 : 1;
    b8_pix_x      = (mb_pix_x >> MIN_BLOCK_SIZE_IN_BIT);   // PU 8x8
    b8_pix_y      = (mb_pix_y >> MIN_BLOCK_SIZE_IN_BIT);
    bsx           = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    bsy           = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);
    //  blocksize_y   = g_blk_size[blocktype][1] << ( bit_size - 4 );        // vertical block size
    //  blocksize_x   = g_blk_size[blocktype][0] << ( bit_size - 4 );

    if (bit_size == 4 && (blocktype >= 4 || blocktype <= 7)) {
        //if 16x4 then  set to 16x8
        b8_pix_x = b8_x;
        b8_pix_y = b8_y;
    }
    center_x = pic_pix_x;
    center_y = pic_pix_y;

    {
        //==================================
        //=====   get original block   =====
        //==================================
        for (j = 0; j < bsy; j++) {
            for (i = 0; i < bsx; i++) {
                orig_pic[j][i] = imgY_org_pic[pic_pix_y +/*j*/incr_y * j + off_y/*lgp*/][pic_pix_x + i];
                //      orig_pic[j][i] = imgy_org_pic[pic_pix_y+j][pic_pix_x+i];
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

        //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
        fst_x_dual = img->allIntegerPFwMv[b8_y][b8_x][ref][blocktype][0];
        fst_y_dual = img->allIntegerPFwMv[b8_y][b8_x][ref][blocktype][1];
        min_mcost_dual = int_motion_cost[ref];

        //==============================
        //=====   SUB-PEL SEARCH   =====
        //==============================
        if (input->hadamard) {
            min_mcost_dual = 0x7FFFFFFF;
        }

        if (blocktype > 3) {
            min_mcost_dual =  FastSubPelBlockMotionSearch_sym(orig_pic, ref, center_x, center_y, blocktype,
                              pred_fst_x_dual, pred_fst_y_dual, &fst_x_dual, &fst_y_dual, 9, 9, min_mcost_dual, lambda, 0, mb_nr, bit_size, block);
        } else {
            min_mcost_dual =  SubPelBlockMotionSearch_sym(orig_pic, ref, center_x, center_y, blocktype,
                              pred_fst_x_dual, pred_fst_y_dual, &fst_x_dual, &fst_y_dual, 9, 9, min_mcost_dual, lambda, bit_size, block);
        }

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

            m = (bsx >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 0 ? 1 : (bsx >>
                    (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT));
            n = (bsy >> (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT)) == 0 ? 1 : (bsy >>
                    (MIN_BLOCK_SIZE_IN_BIT + bit_size - MIN_CU_SIZE_IN_BIT));
            for (i = 0; i < m; i++) {
                for (j = 0; j < n; j++) {
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
                                (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                    }
#endif
#endif
                    img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][0] = Clip3(-32768, 32767,
                            img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][0]);
                    img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][1] = Clip3(-32768, 32767,
                            img->allDualSndMv[b8_y + j][b8_x + i][ref][blocktype][1]);
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
				min_mcost_dual = 0x0EEEEEEE;
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


int                                                   //  ==> minimum motion cost after search
FastSubPelBlockMotionSearch_sym(pel_t
                                orig_pic[MAX_CU_SIZE][MAX_CU_SIZE],     // <--  original pixel values for the AxB block
                                int       ref,           // <--  reference frame (0... or -1 (backward))
                                int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                                int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                                int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                                int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                                int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                                int      *mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                                int      *mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                                int       search_pos2,   // <--  search positions for    half-pel search  (default: 9)
                                int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                                int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                double    lambda,
                                int  useABT,
                                int mb_nr,
                                int bit_size,
                                int block)         // <--  lagrangian parameter for determining motion cost

{
    static int Diamond_x[4] = { -1, 0, 1, 0};
    static int Diamond_y[4] = {0, 1, 0, -1};
    int   mcost;
    int   cand_mv_x = 0, cand_mv_y = 0;
    int   snd_ref = ref == 0 ? 1 : 0;
    int   incr            = 0 ;//qyu 0926
    pel_t **ref_pic       = img->type == B_IMG ? fref[ref + 1 + incr]->oneForthRefY  : fref[ref]->oneForthRefY; //Liwr 0915
    pel_t **ref_pic_sym = NULL; //lgp

    int   lambda_factor   = LAMBDA_FACTOR(lambda);
    int   mv_shift        = 0;
    int   blocksize_x     = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_y     = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   pic4_pix_x      = (pic_pix_x << 2);
    int   pic4_pix_y      = (pic_pix_y << 2);
    int   max_pos_x4      = ((img->width - blocksize_x) << 2);
    int   max_pos_y4      = ((img->height - blocksize_y) << 2);
    int   search_range_dynamic, iXMinNow, iYMinNow, i;
    int   m, currmv_x = 0, currmv_y = 0, iCurrSearchRange;
    int   pred_frac_mv_x, pred_frac_mv_y, abort_search;
    int   mv_cost;
    int delta_P, TRp = 0, DistanceIndexFw = 0, DistanceIndexBw = 0, refframe, delta_PB;
    int   ctr_x = (pred_mv_x >> 1) << 1;
    int   ctr_y = (pred_mv_y >> 1) << 1;

    //xyji
    if (img->type == B_IMG) {
        ref_pic_sym = img->type == B_IMG ? fref[0]->oneForthRefY : fref[ref]->oneForthRefY ;

        refframe = ref;// + incr;
        delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
        delta_P = (delta_P + 512) % 512;   // Added by Xiaozhen ZHENG, 2007.05.05


        TRp = (refframe + 1) * delta_P;
        delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);   // Tsinghua 200701
        TRp  = (TRp + 512) % 512;
        delta_PB = (delta_PB + 512) % 512;   // Added by Xiaozhen ZHENG, 2007.05.05

        DistanceIndexFw = delta_PB;
        //DistanceIndexBw    = TRp - DistanceIndexFw;
        DistanceIndexBw    = (TRp - DistanceIndexFw + 512) % 512;   // Added by Zhijie Yang, 20070419, Broadcom
    } else if (img->type == F_IMG) {
        ref_pic_sym = fref[snd_ref]->oneForthRefY;

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
    }
    *mv_x <<= 2;
    *mv_y <<= 2;

    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)) {
        PelY_14 = UMVPelY_14;//FastPelY_14;//lgp
    } else {
        PelY_14 = UMVPelY_14;
    }

    search_range_dynamic = 3;
    pred_frac_mv_x = (pred_mv_x - *mv_x) % 4;
    pred_frac_mv_y = (pred_mv_y - *mv_y) % 4;
    memset(SearchState[0], 0, (2 * search_range_dynamic + 1) * (2 * search_range_dynamic + 1) * sizeof(byte));

    if (input->hadamard) {
        cand_mv_x = *mv_x;
        cand_mv_y = *mv_y;
        mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
        mcost = AddUpSADQuarter_sym(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic,
                                    ref_pic_sym, orig_pic, mv_cost, min_mcost, useABT, DistanceIndexFw, DistanceIndexBw);
        SearchState[search_range_dynamic][search_range_dynamic] = 1;

        if (mcost < min_mcost) {
            min_mcost = mcost;
            currmv_x = cand_mv_x;
            currmv_y = cand_mv_y;
        }
    } else {
        SearchState[search_range_dynamic][search_range_dynamic] = 1;
        currmv_x = *mv_x;
        currmv_y = *mv_y;
    }

    if (pred_frac_mv_x != 0 || pred_frac_mv_y != 0) {
        cand_mv_x = *mv_x + pred_frac_mv_x;
        cand_mv_y = *mv_y + pred_frac_mv_y;
        if (!input->b_pmvr_enabled || abs(cand_mv_x - ctr_x) <= TH && abs(cand_mv_y - ctr_y) <= TH) { //jcma
            mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
            mcost = AddUpSADQuarter_sym(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic,
                                        ref_pic_sym, orig_pic, mv_cost, min_mcost, useABT, DistanceIndexFw, DistanceIndexBw);
            SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;

            if (mcost < min_mcost) {
                min_mcost = mcost;
                currmv_x = cand_mv_x;
                currmv_y = cand_mv_y;
            }
        }
    }


    iXMinNow = currmv_x;
    iYMinNow = currmv_y;
    iCurrSearchRange = 2 * search_range_dynamic + 1;

    for (i = 0; i < iCurrSearchRange; i++) {
        abort_search = 1;

        for (m = 0; m < 4; m++) {
            if (input->b_pmvr_enabled) {
                if (!pmvr_adapt_mv2(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, iXMinNow, iYMinNow, Diamond_x[m], Diamond_y[m])) {
                    continue;
                }
            } else {
                cand_mv_x = iXMinNow + Diamond_x[m];
                cand_mv_y = iYMinNow + Diamond_y[m];
            }
#if Mv_check_bug
			img->mv_range_flag = check_mv_range_sym(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, ref, 0, 0);
#else
			img->mv_range_flag = check_mv_range_sym(bit_size, cand_mv_x, cand_mv_y, pic_pix_x, pic_pix_y, blocktype, ref);
#endif
            
            if (!img->mv_range_flag) {
                img->mv_range_flag = 1;
                continue;
            }

            if (abs(cand_mv_x - *mv_x) <= search_range_dynamic && abs(cand_mv_y - *mv_y) <= search_range_dynamic) {
                if (!SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic]) {
                    mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
                    mcost = AddUpSADQuarter_sym(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic,
                                                ref_pic_sym, orig_pic, mv_cost, min_mcost, useABT, DistanceIndexFw, DistanceIndexBw);
                    SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;

                    if (mcost < min_mcost) {
                        min_mcost = mcost;
                        currmv_x = cand_mv_x;
                        currmv_y = cand_mv_y;
                        abort_search = 0;

                    }
                }
            }
        }

        iXMinNow = currmv_x;
        iYMinNow = currmv_y;

        if (abort_search) {
            break;
        }
    }

    *mv_x = currmv_x;
    *mv_y = currmv_y;

    //===== return minimum motion cost =====
    return min_mcost;
}


int                                                   //  ==> minimum motion cost after search
FastSubPelBlockMotionSearch_bid(pel_t
                                orig_pic[MAX_CU_SIZE][MAX_CU_SIZE],     // <--  original pixel values for the AxB block
                                int       ref,           // <--  reference frame (0... or -1 (backward))
                                int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                                int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                                int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                                int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                                int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                                int      *mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                                int      *mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                                int       pred_bmv_x_bid,
                                int       pred_bmv_y_bid,
                                int      *bmv_x_bid,
                                int      *bmv_y_bid,
                                int       search_pos2,   // <--  search positions for    half-pel search  (default: 9)
                                int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                                int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                                double    lambda,
                                int  useABT,
                                int mb_nr,
                                int bit_size,
                                int block)         // <--  lagrangian parameter for determining motion cost

{
    static int Diamond_x[4] = { -1, 0, 1, 0};
    static int Diamond_y[4] = {0, 1, 0, -1};
    int   mcost;
    int   cand_mv_x, cand_mv_y;

    int   cand_bmv_x_bid = 1, cand_bmv_y_bid = 1;
    int   snd_ref = 0;
    int   incr            = 0 ;//qyu 0926
    pel_t **ref_pic = img->type == B_IMG ? fref[ref + 1 + incr]->oneForthRefY : fref[ref]->oneForthRefY; //Liwr 0915
    pel_t **ref_pic_sym = NULL; //lgp

    int   lambda_factor   = LAMBDA_FACTOR(lambda);
    int   mv_shift        = 0;
    int   blocksize_x     = g_blk_size[blocktype * 2 + block][0] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   blocksize_y     = g_blk_size[blocktype * 2 + block][1] << (bit_size - MIN_CU_SIZE_IN_BIT);
    int   pic4_pix_x      = (pic_pix_x << 2);
    int   pic4_pix_y      = (pic_pix_y << 2);
    int   max_pos_x4      = ((img->width - blocksize_x) << 2);
    int   max_pos_y4      = ((img->height - blocksize_y) << 2);
    int   search_range_dynamic, iXMinNow, iYMinNow, i;
    int   m, currmv_x = 0, currmv_y = 0, iCurrSearchRange;
    int   pred_frac_mv_x, pred_frac_mv_y, abort_search;
    int   mv_cost;
    int   refframe;
    int   ctr_x = (pred_mv_x >> 1) << 1;
    int   ctr_y = (pred_mv_y >> 1) << 1;

    //xyji
    ref_pic_sym = img->type == B_IMG ? fref[0]->oneForthRefY : fref[snd_ref]->oneForthRefY;

    refframe = ref;// + incr;

    *mv_x <<= 2;
    *mv_y <<= 2;

    //*bmv_x_bid <<= 2;
    //*bmv_y_bid <<= 2;

    if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
        (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)) {
        PelY_14 = UMVPelY_14;//FastPelY_14;//lgp
    } else {
        PelY_14 = UMVPelY_14;
    }

    search_range_dynamic = 3;
    pred_frac_mv_x = (pred_mv_x - *mv_x) % 4;
    pred_frac_mv_y = (pred_mv_y - *mv_y) % 4;
    memset(SearchState[0], 0, (2 * search_range_dynamic + 1) * (2 * search_range_dynamic + 1) * sizeof(byte));

    if (input->hadamard) {
        cand_mv_x = *mv_x;
        cand_mv_y = *mv_y;
        mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

        cand_bmv_x_bid = *bmv_x_bid;
        cand_bmv_y_bid = *bmv_y_bid;
        mv_cost += MV_COST(lambda_factor, mv_shift, cand_bmv_x_bid, cand_bmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid);


        mcost = AddUpSADQuarter_bid(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic,
                                    ref_pic_sym, orig_pic, mv_cost, min_mcost, useABT, cand_bmv_x_bid, cand_bmv_y_bid);

        SearchState[search_range_dynamic][search_range_dynamic] = 1;

        if (mcost < min_mcost) {
            min_mcost = mcost;
            currmv_x = cand_mv_x;
            currmv_y = cand_mv_y;
        }
    } else {
        SearchState[search_range_dynamic][search_range_dynamic] = 1;
        currmv_x = *mv_x;
        currmv_y = *mv_y;
    }

    if (pred_frac_mv_x != 0 || pred_frac_mv_y != 0) {
        cand_mv_x = *mv_x + pred_frac_mv_x;
        cand_mv_y = *mv_y + pred_frac_mv_y;
        if (!input->b_pmvr_enabled || abs(cand_mv_x - ctr_x) <= TH && abs(cand_mv_y - ctr_y) <= TH) { //jcma
            mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

            cand_bmv_x_bid = *bmv_x_bid;
            cand_bmv_y_bid = *bmv_y_bid;
            mv_cost += MV_COST(lambda_factor, mv_shift, cand_bmv_x_bid, cand_bmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid);


            mcost = AddUpSADQuarter_bid(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic,
                                        ref_pic_sym, orig_pic, mv_cost, min_mcost, useABT, cand_bmv_x_bid, cand_bmv_y_bid);


            SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;

            if (mcost < min_mcost) {
                min_mcost = mcost;
                currmv_x = cand_mv_x;
                currmv_y = cand_mv_y;
            }
        }
    }


    iXMinNow = currmv_x;
    iYMinNow = currmv_y;
    iCurrSearchRange = 2 * search_range_dynamic + 1;

    for (i = 0; i < iCurrSearchRange; i++) {
        abort_search = 1;

        for (m = 0; m < 4; m++) {
            if (input->b_pmvr_enabled) {
                if (!pmvr_adapt_mv2(&cand_mv_x, &cand_mv_y, ctr_x, ctr_y, iXMinNow, iYMinNow, Diamond_x[m], Diamond_y[m])) {
                    continue;
                }
            } else {
                cand_mv_x = iXMinNow + Diamond_x[m];
                cand_mv_y = iYMinNow + Diamond_y[m];
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
            if (abs(cand_mv_x - *mv_x) <= search_range_dynamic && abs(cand_mv_y - *mv_y) <= search_range_dynamic) {
                if (!SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic]) {
                    mv_cost = MV_COST(lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

                    cand_bmv_x_bid = *bmv_x_bid;
                    cand_bmv_y_bid = *bmv_y_bid;
                    mv_cost += MV_COST(lambda_factor, mv_shift, cand_bmv_x_bid, cand_bmv_y_bid, pred_bmv_x_bid, pred_bmv_y_bid);


                    mcost = AddUpSADQuarter_bid(pic_pix_x, pic_pix_y, blocksize_x, blocksize_y, cand_mv_x, cand_mv_y, ref_pic,
                                                ref_pic_sym, orig_pic, mv_cost, min_mcost, useABT, cand_bmv_x_bid, cand_bmv_y_bid);

                    SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;

                    if (mcost < min_mcost) {
                        min_mcost = mcost;
                        currmv_x = cand_mv_x;
                        currmv_y = cand_mv_y;
                        abort_search = 0;

                    }
                }
            }
        }

        iXMinNow = currmv_x;
        iYMinNow = currmv_y;

        if (abort_search) {
            break;
        }
    }

    *mv_x = currmv_x;
    *mv_y = currmv_y;

    //===== return minimum motion cost =====
    return min_mcost;
}

/*
*************************************************************************
* Function:Functions for fast fractional pel motion estimation.
1. int AddUpSADQuarter() returns SADT of a fractiona pel MV
2. int FastSubPelBlockMotionSearch () proceed the fast fractional pel ME
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int AddUpSADQuarter_sym(int pic_pix_x, int pic_pix_y, int blocksize_x, int blocksize_y,
                        int cand_mv_x, int cand_mv_y, pel_t **ref_pic, pel_t **ref_pic_sym, pel_t   orig_pic[MAX_CU_SIZE][MAX_CU_SIZE],
                        int Mvmcost, int min_mcost, int useABT, int DistanceIndexFw, int DistanceIndexBw)
{

    int abort_search, y0, x0, rx0, ry0, ry;
    pel_t *orig_line;
    int   diff[16], *d;
    int  mcost = Mvmcost;
    int yy, kk, xx;
    int   curr_diff[MAX_CU_SIZE][MAX_CU_SIZE];

    int ry0_sym = 0, rx0_sym = 0, ry_sym = 0; //lgp

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
                                                      (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2 ;
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
            ry_sym = ry0_sym;//lgp
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

    return mcost;

}


int AddUpSADQuarter_bid(int pic_pix_x, int pic_pix_y, int blocksize_x, int blocksize_y,
                        int cand_mv_x, int cand_mv_y, pel_t **ref_pic, pel_t **ref_pic_sym, pel_t   orig_pic[MAX_CU_SIZE][MAX_CU_SIZE],
                        int Mvmcost, int min_mcost, int useABT, int cand_bmv_x_bid, int cand_bmv_y_bid)
{
    int abort_search, y0, x0, rx0, ry0, ry;
    pel_t *orig_line;
    int   diff[16], *d;
    int  mcost = Mvmcost;
    int yy, kk, xx;
    int   curr_diff[MAX_CU_SIZE][MAX_CU_SIZE];
    int ry0_sym = 0, rx0_sym = 0, ry_sym = 0; //lgp
    for (y0 = 0, abort_search = 0; y0 < blocksize_y && !abort_search; y0 += 4) {
        ry0 = ((pic_pix_y + y0) << 2) + cand_mv_y;

        ry0_sym = ((pic_pix_y + y0) << 2) + cand_bmv_y_bid;

        for (x0 = 0; x0 < blocksize_x; x0 += 4) {
            rx0 = ((pic_pix_x + x0) << 2) + cand_mv_x;

            rx0_sym = ((pic_pix_x + x0) << 2) + cand_bmv_x_bid;

            d   = diff;
            orig_line = orig_pic [y0  ];
            ry = ry0;
            ry_sym = ry0_sym;//lgp
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
            for (yy = y0, kk = 0; yy < y0 + 4; yy++)
                for (xx = x0; xx < x0 + 4; xx++, kk++) {
                    curr_diff[yy][xx] = diff[kk];
                }
        }
    }
    mcost += find_sad_8x8(input->hadamard, blocksize_x, blocksize_y, 0, 0, curr_diff);

    return mcost;
}

