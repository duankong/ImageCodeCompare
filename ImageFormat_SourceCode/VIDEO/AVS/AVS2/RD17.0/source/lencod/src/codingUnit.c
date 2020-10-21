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
* Function: Process one codingUnit
*
*************************************************************************************
*/


#include "../../lcommon/inc/contributors.h"

#include <memory.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "codingUnit.h"
#include "refbuf.h"
#include "vlc.h"
#include "block.h"
#include "header.h"
#include "global.h"
#include "AEC.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/commonStructures.h"
#include "../../lcommon/inc/memalloc.h"
#include "../../lcommon/inc/block_info.h"
#include "../../lcommon/inc/transform.h"
#include "../../lcommon/inc/intra-prediction.h"
#include "../../lcommon/inc/inter-prediction.h"

#if RATECONTROL
#include "ratecontrol.h"
#endif


static const int IF_CHROMA_COEF[8][4] = {
    { 0, 64, 0, 0 },
    { -4, 62, 6, 0 },
    { -6, 56, 15, -1 },
    { -5, 47, 25, -3 },
    { -4, 36, 36, -4 },
    { -3, 25, 47, -5 },
    { -1, 15, 56, -6 },
    { 0, 6, 62, -4 }
};

const byte QP_SCALE_CR[64] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 42, 43, 43, 44, 44, 45, 45,
    46, 46, 47, 47, 48, 48, 48, 49, 49, 49,
    50, 50, 50, 51,
};
extern const int NCBP[64][2];

#if Mv_check_bug
	extern  int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y,
		int blocktype, int dmh_x, int dmh_y);  // mv_range, 20071009
#else
	extern  int check_mv_range(unsigned int uiBitSize, int mv_x, int mv_y, int pix_x, int pix_y,
		int blocktype);  // mv_range, 20071009
#endif

extern void SetMotionVectorPredictor(unsigned int uiBitSize, unsigned int uiPositionInPic, int  pmv[2],
                                     int  **refFrArr, int  ***tmp_mv, int  ref_frame, int  mb_pix_x,
                                     int  mb_pix_y, int  blockshape_x, int  blockshape_y, int  ref, int  direct_mv);



int **curr_blk_PRED;
int **curr_blk_MPR;
int fw_mode_2N;
int bw_mode_2N;

void pmvr_mvd_derivation(int mvd[2], int mv[2], int mvp[2]);

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

#if HALF_PIXEL_CHROMA
extern int calculate_distance(int blkref, int fw_bw);
#endif

/*
*************************************************************************
* Function:Update the coordinates for the next codingUnit to be processed
* Input:mb: MB address in scan order
* Output:
* Return:
* Attention:
*************************************************************************
*/

void set_MB_parameters(int mb)
{
    const int number_mb_per_row = img->width / MIN_CU_SIZE ;  //add by wuzhongmou 0610

    img->current_mb_nr = mb;
    img->mb_x = mb % number_mb_per_row;
    img->mb_y = mb / number_mb_per_row;

    // Define vertical positions
    img->block8_y = img->mb_y * BLOCK_MULTIPLE;
    img->pix_y   = img->mb_y * MIN_CU_SIZE;   // vertical luma codingUnit position

    if (input->chroma_format == 1) {
        img->pix_c_y = img->mb_y * MIN_CU_SIZE / 2;  // vertical chroma codingUnit position
    }

    // Define horizontal positions
    img->block8_x = img->mb_x * BLOCK_MULTIPLE; /*lgp*/
    img->pix_x   = img->mb_x * MIN_CU_SIZE;     // luma pixel
    img->pix_c_x   = img->mb_x * MIN_CU_SIZE / 2; // chroma pixel
}

/*
*************************************************************************
* Function:Update the coordinates and statistics parameter for the
next codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void proceed2nextcodingUnit()
{
    codingUnit *currMB = &img->mb_data[img->current_mb_nr];
    int        *bitCount = currMB->bitcounter;

    // Update the statistics
    stat->bit_use_cuType[img->type]      += bitCount[BITS_MB_MODE];
    stat->bit_use_coeffY[img->type]       += bitCount[BITS_COEFF_Y_MB] ;
    stat->tmp_bit_use_cbp[img->type]      += bitCount[BITS_CBP_MB];
    stat->bit_use_coeffC[img->type]       += bitCount[BITS_COEFF_UV_MB];
    stat->bit_use_delta_quant[img->type]  += bitCount[BITS_DELTA_QUANT_MB];

    if (img->type == INTRA_IMG) {
        ++stat->mode_use_intra[currMB->cuType];
    } else if (img->type != B_IMG) {
        ++stat->mode_use_inter[0][currMB->cuType];
        stat->bit_use_mode_inter[0][currMB->cuType] += bitCount[BITS_INTER_MB];

    } else {
        stat->bit_use_mode_inter[1][currMB->cuType] += bitCount[BITS_INTER_MB];
        ++stat->mode_use_inter[1][currMB->cuType];
    }

    // Statistics
    if ((img->type == F_IMG) || (img->type == P_IMG)) {
        ++stat->quant0;
#if MB_DQP
        stat->quant1 += currMB->qp;
#else
        stat->quant1 += img->qp;      // to find average quant for inter frames
#endif
    }
}

/*
*************************************************************************
* Function:initializes the current codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void start_codingUnit()
{
    int num_of_orgMB_in_row;
    int num_of_orgMB_in_col;//4:1  5:2  6:4
    int row, col, pos;
    int pix_x_InPic_end = (img->current_mb_nr % img->PicWidthInMbs) * MIN_CU_SIZE + (1 << input->g_uiMaxSizeInBit);
    int pix_y_InPic_end = (img->current_mb_nr / img->PicWidthInMbs) * MIN_CU_SIZE + (1 << input->g_uiMaxSizeInBit);

    if (pix_x_InPic_end > img->width) {
        num_of_orgMB_in_col = (img->width - img->pix_x) >> MIN_CU_SIZE_IN_BIT;
    } else {
        num_of_orgMB_in_col = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);   //Liwr 0915
    }

    if (pix_y_InPic_end > img->height) {
        num_of_orgMB_in_row = (img->height - img->pix_y) >> MIN_CU_SIZE_IN_BIT;
    } else {
        num_of_orgMB_in_row = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);   //Liwr 0915
    }

    for (row = 0; row < num_of_orgMB_in_row; row++) {
        pos = img->current_mb_nr + row * (img->width / MIN_CU_SIZE);

        for (col = 0; col < num_of_orgMB_in_col; col++, pos++) {   //Liwr 0915
            int i, j, k, l;
            codingUnit *currMB = &img->mb_data[pos];
            currMB->slice_nr = img->current_slice_nr;
            currMB->ui_MbBitSize = MIN_CU_SIZE_IN_BIT;
            currMB->delta_qp = 0;
            currMB->qp       = img->qp;       // needed in loop filter (even if constant QP is used)

            // Initialize counter for MB symbols
            currMB->currSEnr = 0;

#if MB_DQP
            currMB->previouse_qp     = img->qp;
            currMB->left_cu_qp       = img->qp;
#endif

            // Reset syntax element entries in MB struct
            currMB->cuType    = PSKIPDIRECT;
            currMB->cbp_blk   = 0;
            currMB->cbp       = 0;
            for (l = 0; l < 2; l++) {
                for (j = 0; j < BLOCK_MULTIPLE; j++) {
                    for (i = 0; i < BLOCK_MULTIPLE; i++) {
                        for (k = 0; k < 2; k++) {
                            currMB->mvd[l][j][i][k] = 0;
                        }
                    }
                }
            }

            currMB->c_ipred_mode = DC_PRED_C; //GB

            for (i = 0; i < (BLOCK_MULTIPLE * BLOCK_MULTIPLE); i++) {
                currMB->intra_pred_modes[i] = DC_PRED;
            }

            // store filtering parameters for this MB; For now, we are using the
            // same offset throughout the sequence

            // Initialize bitcounters for this codingUnit
            if (img->current_mb_nr == 0) {   // No slice header to account for  //qyu 0817
                currMB->bitcounter[BITS_HEADER] = 0;
            } else if (currMB->slice_nr == img->mb_data[img->current_mb_nr -
                       1].slice_nr) {  // current MB belongs to the same slice as the last MB
                currMB->bitcounter[BITS_HEADER] = 0;
            }

            currMB->bitcounter[BITS_MB_MODE] = 0;
            currMB->bitcounter[BITS_COEFF_Y_MB] = 0;
            currMB->bitcounter[BITS_INTER_MB] = 0;
            currMB->bitcounter[BITS_CBP_MB] = 0;
            currMB->bitcounter[BITS_DELTA_QUANT_MB] = 0;
            currMB->bitcounter[BITS_COEFF_UV_MB] = 0;

            // Reset vectors before doing motion search in motion_search().

            if (input->usefme) {
                const int number_mb_per_row = img->width / MIN_CU_SIZE ;
                int mb_pos_x = (pos % number_mb_per_row) << 1;
                int mb_pos_y = (pos / number_mb_per_row) << 1;

                if (img->type != B_IMG) {
                    for (j = 0; j < BLOCK_MULTIPLE; j++) {
                        for (i = 0; i < BLOCK_MULTIPLE; i++) {
                            he->all_mincost[mb_pos_y + j][mb_pos_x + i][0][0][3] = img->tmp_mv[mb_pos_y + j][mb_pos_x + i][0];
                            he->all_mincost[mb_pos_y + j][mb_pos_x + i][0][0][4] = img->tmp_mv[mb_pos_y + j][mb_pos_x + i][1];
                        }
                    }
                } else {
                    for (j = 0; j < BLOCK_MULTIPLE; j++) {
                        for (i = 0; i < BLOCK_MULTIPLE; i++) {

                            he->all_mincost[mb_pos_y + j][mb_pos_x + i][0][0][3] = img->fw_mv[mb_pos_y + j][mb_pos_x + i][0];
                            he->all_mincost[mb_pos_y + j][mb_pos_x + i][0][0][4] = img->fw_mv[mb_pos_y + j][mb_pos_x + i][1];
                            he->all_bwmincost[mb_pos_y + j][mb_pos_x + i][0][0][3] = img->bw_mv[mb_pos_y + j][mb_pos_x + i][0];
                            he->all_bwmincost[mb_pos_y + j][mb_pos_x + i][0][0][4] = img->bw_mv[mb_pos_y + j][mb_pos_x + i][1];
                        }
                    }
                }
            }

            if (img->type != B_IMG) {   //qyu 0817
                const int number_mb_per_row = img->width / MIN_CU_SIZE ;
                int mb_pos_x = (pos % number_mb_per_row) << 1;
                int mb_pos_y = (pos / number_mb_per_row) << 1;

                for (k = 0; k < 2; k++) {
                    for (j = 0; j < BLOCK_MULTIPLE; j++) {
                        for (i = 0; i < BLOCK_MULTIPLE; i++) {
                            img->tmp_mv[mb_pos_y + j][mb_pos_x + i][k] = 0;
                            img->p_snd_tmp_mv[mb_pos_y + j][mb_pos_x + i][k] = 0;
                        }
                    }
                }
            }
        }
    }
}
/*
*************************************************************************
* Function:Terminate processing of the current codingUnit depending
on the chosen slice mode
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void Init_Curr_codingUnit(codingUnit *currMB, unsigned int uiBitSize,
                          unsigned int PositionInPic)  //,int     **ipredmodes)
{
    int i, j, k, l;

#if MB_DQP
    int mb_width, pix_a, remove_prediction;
#endif

    currMB->ui_MbBitSize = uiBitSize;

    currMB->slice_nr = img->current_slice_nr;
    currMB->delta_qp = 0;
    currMB->qp       = img->qp;       // needed in loop filter (even if constant QP is used)

#if RATECONTROL
    if (input->EncControl && input->useDQP) {
        currMB->qp = pRC->RcMBQP;
    }
#endif

#if MB_DQP
    if (input->useDQP) {
        mb_width = img->width / MIN_CU_SIZE;
        pix_a   = (PositionInPic % mb_width) * MIN_CU_SIZE;
        if (pix_a >= MIN_CU_SIZE) {
            remove_prediction = currMB->slice_nr != img->mb_data[PositionInPic - 1].slice_nr;
            if (!remove_prediction) {
                currMB->left_cu_qp = img->mb_data[PositionInPic - 1].qp;
            } else {
                currMB->left_cu_qp = img->qp;
            }
        } else {
            currMB->left_cu_qp = img->qp;
        }
#if LEFT_PREDICTION      //left prediction of QP
        currMB->delta_qp = currMB->qp - currMB->left_cu_qp;

        //currMB->delta_qp = 0;
        //currMB->qp = img->qp;
#else
        currMB->delta_qp = 0;
#endif
    } else {
        currMB->delta_qp = 0;
        currMB->qp       = img->qp;
    }

#else
    currMB->delta_qp = 0;
    currMB->qp       = img->qp;       // needed in loop filter (even if constant QP is used)
#endif
    // Initialize counter for MB symbols
    currMB->currSEnr = 0;

    // Reset syntax element entries in MB struct
    currMB->cuType   = PSKIPDIRECT;
    currMB->cbp_blk   = 0;
    currMB->cbp       = 0;

    for (l = 0; l < 2; l++) {
        for (j = 0; j < BLOCK_MULTIPLE; j++) {
            for (i = 0; i < BLOCK_MULTIPLE; i++) {
                for (k = 0; k < 2; k++) {
                    currMB->mvd[l][j][i][k] = 0;
                }
            }
        }
    }


    currMB->c_ipred_mode = DC_PRED_C; //GB

    for (i = 0; i < (BLOCK_MULTIPLE * BLOCK_MULTIPLE); i++) {
        currMB->intra_pred_modes[i] = DC_PRED;
    }

    // store filtering parameters for this MB; For now, we are using the
    // same offset throughout the sequence

    // Initialize bitcounters for this codingUnit
    if (img->current_mb_nr == 0) {   // No slice header to account for
        currMB->bitcounter[BITS_HEADER] = 0;
    } else if (currMB->slice_nr == img->mb_data[img->current_mb_nr - 1].slice_nr) {   // current MB belongs to the
        currMB->bitcounter[BITS_HEADER] = 0;
    }

    currMB->bitcounter[BITS_MB_MODE] = 0;
    currMB->bitcounter[BITS_COEFF_Y_MB] = 0;
    currMB->bitcounter[BITS_INTER_MB] = 0;
    currMB->bitcounter[BITS_CBP_MB] = 0;
    currMB->bitcounter[BITS_DELTA_QUANT_MB] = 0;
    currMB->bitcounter[BITS_COEFF_UV_MB] = 0;
}

void terminate_codingUnit(Boolean *end_of_picture, int lcuIndex)
{
    codingUnit    *currMB    = &img->mb_data[img->current_mb_nr];

    int mb_width = img->width / MIN_CU_SIZE;
    int i, j, pos, pos_next;
    int num_of_orgMB_in_row = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);   //4:1  5:2  6:4
    int num_of_orgMB_in_col = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);
    int size = 1 << input->g_uiMaxSizeInBit;
    int pix_x = (img->current_mb_nr % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (img->current_mb_nr / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

    if (pix_x + size >= img->width) {
        num_of_orgMB_in_col = (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT;
    }

    if (pix_y + size >= img->height) {
        num_of_orgMB_in_row = (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT;
    }

    img->coded_mb_nr += (num_of_orgMB_in_col * num_of_orgMB_in_row) ;
    if (num_of_orgMB_in_col < (1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT))) {
        img->slice_offset += num_of_orgMB_in_row * num_of_orgMB_in_col;
    }

    if (/*input->slice_row_nr &&*/ (img->coded_mb_nr != img->PicSizeInMbs)) {
        if ((img->current_mb_nr + num_of_orgMB_in_col) % img->PicWidthInMbs == 0) {
            pos_next = img->current_mb_nr + ((1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT)) - 1) * img->PicWidthInMbs +
                       num_of_orgMB_in_col;
        } else {
            pos_next = img->current_mb_nr + num_of_orgMB_in_col;
        }

        pix_x = (pos_next % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
        pix_y = (pos_next / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

        if (pix_x + size >= img->width) {
            num_of_orgMB_in_col = (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT;
        }

        if (pix_y + size >= img->height) {
            num_of_orgMB_in_row = (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT;
        }
        if (lcuIndex % input->slice_row_nr == 0) {
            img->current_slice_nr++;
//        img->mb_no_currSliceLastMB =  min ( img->mb_no_currSliceLastMB + slice_mb, img->PicSizeInMbs - 1 ) ;
        } else {
            for (j = 0; j < num_of_orgMB_in_row; j++) {
                pos = pos_next + j * mb_width;

                for (i = 0; i < num_of_orgMB_in_col; i++, pos++) {
                    img->mb_data[pos].slice_nr =
                        img->mb_data[img->current_mb_nr].slice_nr;  // last MB of slice, begin next slice   by jlzheng  6.30
                }
            }
        }
    }


    if (img->coded_mb_nr == img->PicSizeInMbs) {   // maximum number of MBs reached
        *end_of_picture = TRUE;
        img->coded_mb_nr = 0;
        img->slice_offset = 0;
    }
}

/*
*************************************************************************
* Function:Predict one component of a 8x8 Luma block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
#if Mv_Rang
void OneComponentLumaPrediction8x8(unsigned int uiBitSize, int   *mpred, int    pic_pix_x, int    pic_pix_y,
                                   int mode, int PU_size_x, int PU_size_y, int   *mv, int    ref)  // <--  reference frame (0.. / -1:backward)
#else
void OneComponentLumaPrediction8x8(unsigned int uiBitSize, int   *mpred, int    pic_pix_x, int    pic_pix_y,
                                   int PU_size_x, int PU_size_y, int   *mv, int    ref)  // <--  reference frame (0.. / -1:backward)
#endif
{
    int incr = 1; /*lgp*13*/
    pel_t **ref_pic;
    int     pix_add = 4;
    int     j0      = (pic_pix_y << 2) + mv[1];   //, j1=j0+pix_add, j2=j1+pix_add, j3=j2+pix_add;
    int     i0      = (pic_pix_x << 2) + mv[0];   //, i1=i0+pix_add, i2=i1+pix_add, i3=i2+pix_add;
    int     i, j;
    int first_x, first_y, second_x, second_y, dmh_mode;
#if Mv_Rang
	int mv0_x,mv0_y;
	int mv1_x,mv1_y;
#endif
    pel_t (*get_pel)(pel_t **, int, int) = UMVPelY_14;

    if (img->type == F_IMG && input->b_dmh_enabled && img->typeb != BP_IMG)

    {
        dmh_mode = mv[2];
    } else {
        dmh_mode = 0;
    }

#if Mv_check_bug
	img->mv_range_flag *= check_mv_range(uiBitSize + 1, mv[0], mv[1], pic_pix_x, pic_pix_y, mode,dmh_pos[dmh_mode][0][0],dmh_pos[dmh_mode][0][1]);    // qyu 5->4
	img->mv_range_flag *= check_mv_range(uiBitSize + 1, mv[0], mv[1], pic_pix_x, pic_pix_y, mode,dmh_pos[dmh_mode][1][0],dmh_pos[dmh_mode][1][1]); 
#else
	#if Mv_Rang
		img->mv_range_flag *= check_mv_range(uiBitSize + 1, mv[0], mv[1], pic_pix_x, pic_pix_y, mode);    // qyu 5->4
	#else
		img->mv_range_flag *= check_mv_range(uiBitSize + 1, mv[0], mv[1], pic_pix_x, pic_pix_y, 4);    // qyu 5->4
	#endif
#endif

    ref_pic = img->type == B_IMG ? fref[ref + incr]->oneForthRefY : fref[ref]->oneForthRefY;
    if (input->bg_enable && he->background_reference_enable &&
        ref == img->num_of_references - 1 &&
        (img->type == P_IMG || img->type == F_IMG) && img->typeb != BP_IMG) {
        ref_pic = he->background_frame_quarter;
    }

    if (img->type ==  P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        ref_pic = he->background_frame_quarter;
    }


    first_x  = dmh_pos[dmh_mode][0][0];
    first_y  = dmh_pos[dmh_mode][0][1];
    second_x = dmh_pos[dmh_mode][1][0];
    second_y = dmh_pos[dmh_mode][1][1];


    for (j = 0; j < PU_size_y; j++) {
        for (i = 0; i < PU_size_x; i++)

        {
            if (dmh_mode == 0) {
                *mpred++ = get_pel(ref_pic, j0 + pix_add * j, i0 + pix_add * i);
            } else {
#if Mv_Rang
				mv0_x=Clip3(-32768, 32767,i0 + pix_add * i + first_x);
				mv0_y=Clip3(-32768, 32767,j0 + pix_add * j + first_y);
				mv1_x=Clip3(-32768, 32767,i0 + pix_add * i + second_x);
				mv1_y=Clip3(-32768, 32767,j0 + pix_add * j + second_y);
				*mpred++ = (get_pel(ref_pic, mv0_y, mv0_x) + get_pel(ref_pic,
					mv1_y, mv1_x) + 1) / 2;
#else
                *mpred++ = (get_pel(ref_pic, j0 + pix_add * j + first_y, i0 + pix_add * i + first_x) + get_pel(ref_pic,
                            j0 + pix_add * j + second_y, i0 + pix_add * i + second_x) + 1) / 2;
#endif
            }
        }
    }
}

/*
*************************************************************************
* Function:Predict one 8x8 Luma block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
#if Mv_Rang
void LumaPrediction8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int  block_x,
                       int  block_y, int mode, int  fw_mode, int  bw_mode, int  fw_ref, int  bw_ref, MCParam *pMCParam)
#else
void LumaPrediction8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int  block_x,
                       int  block_y, int  fw_mode, int  bw_mode, int  fw_ref, int  bw_ref, MCParam *pMCParam)
#endif
{
    int *fw_pred = (int *) malloc((1 << (uiBitSize + 1)) * (1 << (uiBitSize + 1)) * sizeof(int));
    int *bw_pred = (int *) malloc((1 << (uiBitSize + 1)) * (1 << (uiBitSize + 1)) * sizeof(int));

    int *fw_pred_wgt = (int *) malloc((1 << (uiBitSize + 1)) * (1 << (uiBitSize + 1)) * sizeof(int));
    int  i, j;

    int block_x8, block_y8, step_h, step_v, start_x, start_y, bx, by;

    int  pic_pix_x = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + block_x;
    int  pic_pix_y = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + block_y;

    int *fpred     = fw_pred;
    int *bpred     = bw_pred;
    int *fpred_wgt  = fw_pred_wgt;
    int  direct    = (fw_mode == 0 && bw_mode == 0 && (img->type == B_IMG));
    int  skipped   = (fw_mode == 0 && bw_mode == 0 && (img->type != B_IMG));
    int                ** ***fmv_array = (fw_mode < 0) ? img->allBidFwMv : (fw_mode > 0  &&
                                         bw_mode > 0) ? img->allSymMv : (fw_mode > 0  && bw_mode < 0) ? img->allDualFstMv : img->allFwMv;
    int                ** ***bmv_array = (fw_mode < 0) ? img->allBidBwMv : (fw_mode > 0  &&
                                         bw_mode < 0) ? img->allDualSndMv : img->allBwMv;
    int  dual = (fw_mode > 0 && bw_mode < 0) ? 1 : 0;
    int bid = (fw_mode < 0 && bw_mode < 0) ? 1 : 0;

#if FIX_MAX_REF
    int delta_P[MAXREF];
#else
    int delta_P[4];
#endif

    int num_skipmode = pMCParam->num_skipmode;

#if FIX_MAX_REF
    for (i = 0; i < img->num_of_references; i++) {
#else
    for (i = 0; i < 4; i++) {
#endif
        delta_P[i] = (2 * (hc->picture_distance - fref[i]->imgtr_fwRefDistance) + 512) % 512;
        if (i == img->num_of_references - 1 && he->background_reference_enable) {
            delta_P[i] = 1;
        }
    }

    if (fw_mode < 0) {
        fw_mode = -fw_mode;
    }
    if (bw_mode < 0) {
        bw_mode = -bw_mode;
    }

    bx = block_x != 0;
    by = block_y != 0;
    get_pix_offset(currMB->cuType, uiBitSize + 1, bx, by, &start_x, &start_y, &step_h, &step_v);
    block_x8 = block_x + step_h;
    block_y8 = block_y + step_v;


    if (direct && pMCParam->dir == DS_BACKWARD) {
        fw_ref = -1;
    } else if (direct && pMCParam->dir == DS_FORWARD) {
        bw_ref = -1;
    }

    //direct_mode = direct;

    if (fw_mode || (direct && (fw_ref != -1)) || skipped) {
        fmv_array [by][bx][fw_ref][fw_mode][2] = pMCParam->dmh_mode;

#if Mv_Rang
        if (pMCParam->dir != 0) {
            if (img->type == P_IMG || img->type == F_IMG) {
                OneComponentLumaPrediction8x8(uiBitSize, fw_pred, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                              img->tmp_fstPSkipMv[pMCParam->dir], fw_ref);
            } else {
                OneComponentLumaPrediction8x8(uiBitSize, fw_pred, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                              img->tmp_fwBSkipMv[pMCParam->dir], (direct ? 0 : fw_ref));
            }
        } else {
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                          fmv_array [by][bx][fw_ref][fw_mode], (direct ? 0 : fw_ref));    //qyu 4x4->8x8
        }
#else
        if (pMCParam->dir != 0) {
            if (img->type == P_IMG || img->type == F_IMG) {
                OneComponentLumaPrediction8x8(uiBitSize, fw_pred, pic_pix_x, pic_pix_y, step_h, step_v,
                                              img->tmp_fstPSkipMv[pMCParam->dir], fw_ref);
            } else {
                OneComponentLumaPrediction8x8(uiBitSize, fw_pred, pic_pix_x, pic_pix_y, step_h, step_v,
                                              img->tmp_fwBSkipMv[pMCParam->dir], (direct ? 0 : fw_ref));
            }
        } else {
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred, pic_pix_x, pic_pix_y, step_h, step_v,
                                          fmv_array [by][bx][fw_ref][fw_mode], (direct ? 0 : fw_ref));    //qyu 4x4->8x8
        }
#endif
        if (img->type == F_IMG && num_skipmode && fw_mode == 0) {
            int mv_wgt[3];

#if MV_SCALE
            mv_wgt[0] = scale_mv(fmv_array[by][bx][fw_ref][fw_mode][0], delta_P[num_skipmode], delta_P[0]);
            mv_wgt[1] = scale_mv(fmv_array[by][bx][fw_ref][fw_mode][1], delta_P[num_skipmode], delta_P[0]);
#if HALF_PIXEL_COMPENSATION_M1
            //assert(is_field_sequence == img->is_field_sequence);
            if (img->is_field_sequence) {
                mv_wgt[1] = scale_mv_y1(fmv_array[by][bx][fw_ref][fw_mode][1], delta_P[num_skipmode], delta_P[0]);
            }
#endif
#else
            mv_wgt[0] = (delta_P[num_skipmode] * fmv_array[by] [bx][fw_ref][fw_mode][0] * (MULTI / delta_P[0]) + HALF_MULTI) >>
                        OFFSET ;
            mv_wgt[1] = (delta_P[num_skipmode] * fmv_array[by] [bx][fw_ref][fw_mode][1] * (MULTI / delta_P[0]) + HALF_MULTI) >>
                        OFFSET;
#if HALF_PIXEL_COMPENSATION_M1
            //assert(is_field_sequence == img->is_field_sequence);
            if (img->is_field_sequence) {
                int delta, delta2;
                int oriPOC = 2 * hc->picture_distance;
                int oriRefPOC = oriPOC - delta_P[0];
                int scaledPOC = 2 * hc->picture_distance;
                int scaledRefPOC = scaledPOC - delta_P[num_skipmode];
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                mv_wgt[1] = ((delta_P[num_skipmode] * (fmv_array[by] [bx][fw_ref][fw_mode][1] + delta) *
                              (16384 / delta_P[0]) + 8192) >> 14) - delta2;
            }
#endif
#endif
            mv_wgt[2] = 0;
#if Mv_Rang
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred_wgt, pic_pix_x, pic_pix_y, mode, step_h, step_v, mv_wgt, num_skipmode);
#else
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred_wgt, pic_pix_x, pic_pix_y, step_h, step_v, mv_wgt, num_skipmode);
#endif

            for (j = block_y; j < block_y8; j++) {
                for (i = block_x; i < block_x8; i++) {
                    //img->mpr[i][j] = (*fpred + *fpred_yb + 1) / 2;
                    *fpred = (*fpred + *fpred_wgt + 1) / 2;
                    fpred++;
                    fpred_wgt++;
                    //bpred++;
                }
            }

            fpred = fw_pred;
        }

        if (img->type == F_IMG && input->dhp_enabled && dual) {
            bmv_array [by][bx][fw_ref][bw_mode][2] = 0;
#if Mv_Rang
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred_wgt, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                          bmv_array [by][bx][fw_ref][bw_mode], bw_ref);
#else
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred_wgt, pic_pix_x, pic_pix_y, step_h, step_v,
                                          bmv_array [by][bx][fw_ref][bw_mode], bw_ref);
#endif
            for (j = block_y; j < block_y8; j++) {
                for (i = block_x; i < block_x8; i++) {
                    *fpred = (*fpred + *fpred_wgt + 1) / 2;
                    fpred++;
                    fpred_wgt++;
                }
            }
            fpred = fw_pred;
        }

        if ((img->type == P_IMG || img->type == F_IMG) && (pMCParam->dir == BID_P_FST || pMCParam->dir == BID_P_SND)) {
#if Mv_Rang            
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred_wgt, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                          img->tmp_sndPSkipMv[pMCParam->dir], bw_ref);
#else
            OneComponentLumaPrediction8x8(uiBitSize, fw_pred_wgt, pic_pix_x, pic_pix_y, step_h, step_v,
                                          img->tmp_sndPSkipMv[pMCParam->dir], bw_ref);
#endif
            for (j = block_y; j < block_y8; j++) {
                for (i = block_x; i < block_x8; i++) {
                    *fpred = (*fpred + *fpred_wgt + 1) / 2;
                    fpred++;
                    fpred_wgt++;
                }
            }
            fpred = fw_pred;
        }
    }

    if (img->type == B_IMG && (bw_mode || (direct && (bw_ref != -1)))) {
        int delta_P, TRp, DistanceIndexFw, DistanceIndexBw, refframe, delta_PB;
        int mv[2];
        refframe = fw_ref;
        delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
        delta_P = (delta_P + 512) % 512;

        TRp = (refframe + 1) * delta_P;    //the lates backward reference

        delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);
        TRp  = (TRp + 512) % 512;
        delta_PB = (delta_PB + 512) % 512;
        DistanceIndexFw = delta_PB;
        DistanceIndexBw    = (TRp - DistanceIndexFw + 512) % 512;


        if (fw_ref != -1) {
#if MV_SCALE
            mv[0] = -scale_mv(fmv_array[by][bx][fw_ref][fw_mode][0], DistanceIndexBw, DistanceIndexFw);
            mv[1] = -scale_mv(fmv_array[by][bx][fw_ref][fw_mode][1], DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
            if (img->is_field_sequence) {
                mv[1] = -scale_mv_y2(fmv_array[by][bx][fw_ref][fw_mode][1], DistanceIndexBw, DistanceIndexFw);
            }
#endif
#else
            mv[0] = - ((fmv_array[by][bx][fw_ref][fw_mode][0] * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                       OFFSET);
            mv[1] = - ((fmv_array[by][bx][fw_ref][fw_mode][1] * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >>
                       OFFSET);
#if HALF_PIXEL_COMPENSATION_MVD
            if (img->is_field_sequence) {
                int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                oriPOC = 2 * hc->picture_distance;
                oriRefPOC = oriPOC - DistanceIndexFw;
                scaledPOC = 2 * hc->picture_distance;
                scaledRefPOC = scaledPOC - DistanceIndexBw;
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                mv[1] = - (((fmv_array[by][bx][fw_ref][fw_mode][1] + delta) * DistanceIndexBw * (MULTI / DistanceIndexFw) +
                            HALF_MULTI) >> OFFSET) - delta2;
            }
#endif
#endif
        }
        mv[0] = Clip3(-32768, 32767, mv[0]);
        mv[1] = Clip3(-32768, 32767, mv[1]);

        if (bid) {
            mv[0] = bmv_array[by][bx][bw_ref][bw_mode][0] ;
            mv[1] = bmv_array[by][bx][bw_ref][bw_mode][1] ;
        }

#if Mv_Rang            
        if (fw_mode && bw_mode) {
            if (pMCParam->dir != 0) {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                              img->tmp_bwBSkipMv[pMCParam->dir], (-1 - bw_ref));
            } else {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, mode, step_h, step_v, mv, (-1 - bw_ref));
            }
        } else {
            if (pMCParam->dir != 0) {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                              img->tmp_bwBSkipMv[pMCParam->dir], (-1 - bw_ref));
            } else {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, mode, step_h, step_v,
                                              bmv_array[by][bx][bw_ref][bw_mode], (-1 - bw_ref));
            }
        }
#else
        if (fw_mode && bw_mode) {
            if (pMCParam->dir != 0) {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, step_h, step_v,
                                              img->tmp_bwBSkipMv[pMCParam->dir], (-1 - bw_ref));
            } else {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, step_h, step_v, mv, (-1 - bw_ref));
            }
        } else {
            if (pMCParam->dir != 0) {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, step_h, step_v,
                                              img->tmp_bwBSkipMv[pMCParam->dir], (-1 - bw_ref));
            } else {
                OneComponentLumaPrediction8x8(uiBitSize, bw_pred, pic_pix_x, pic_pix_y, step_h, step_v,
                                              bmv_array[by][bx][bw_ref][bw_mode], (-1 - bw_ref));
            }
        }
#endif
    }

    // !! start shenyanfei
    if (direct || (fw_mode && bw_mode && img->type == B_IMG)) {
        if (pMCParam->dir == 0 || pMCParam->dir == DS_BID || pMCParam->dir == DS_SYM) {
            for (j = block_y; j < block_y8; j++) {
                for (i = block_x; i < block_x8; i++) {
                    img->predBlock[j][i] = (*fpred + *bpred + 1) >> 1;

                    fpred++;
                    bpred++;
                }
            }
        } else if (pMCParam->dir == DS_BACKWARD) {
            for (j = block_y; j < block_y8; j++) {
                for (i = block_x; i < block_x8; i++) {
                    img->predBlock[j][i] = *bpred;
                    bpred++;
                }
            }
        } else if (pMCParam->dir == DS_FORWARD) {
            for (j = block_y; j < block_y8; j++) {
                for (i = block_x; i < block_x8; i++) {
                    img->predBlock[j][i] = *fpred;
                    fpred++;
                }
            }
        }
    } else if (fw_mode || skipped) {   //P fw and B one direction fw
        for (j = block_y; j < block_y8; j++) {
            for (i = block_x; i < block_x8; i++) {
                img->predBlock[j][i] = *fpred;
                fpred++;
            }
        }
    } else { //B one direction bw
        for (j = block_y; j < block_y8; j++) {
            for (i = block_x; i < block_x8; i++) {
                img->predBlock[j][i] = *bpred;
                bpred++;
            }
        }
    }

    // !! end shenyanfei
    free(fw_pred);
    free(bw_pred);
    free(fw_pred_wgt);
}

/*
*************************************************************************
* Function:Residual Coding of an 8x8 Luma block (not for intra)
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
#if Mv_Rang
void LumaResidualCoding8x8_PRED(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cbp,
                                int *cbp_blk, int block8x8, int mode, int fw_mode, int bw_mode, int fw_refframe, int bw_refframe, MCParam *pMCParam,
                                int **curr_blk)
#else
void LumaResidualCoding8x8_PRED(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cbp,
                                int *cbp_blk, int block8x8, int fw_mode, int bw_mode, int fw_refframe, int bw_refframe, MCParam *pMCParam,
                                int **curr_blk)
#endif
{
    int    pic_pix_y, pic_pix_x, i, j;

    int mb_x, mb_y, step_h, step_v;

    byte **imgY_original = he->imgY_org;
    int  pix_x    = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int  pix_y    = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;

    get_pix_offset(currMB->cuType, uiBitSize + 1, (block8x8 % 2), (block8x8 / 2), &mb_x, &mb_y, &step_h, &step_v);

    pic_pix_y = pix_y + mb_y;
    pic_pix_x = pix_x + mb_x;
#if Mv_Rang
    LumaPrediction8x8(currMB, uiBitSize, uiPositionInPic, mb_x, mb_y, mode, fw_mode, bw_mode, fw_refframe, bw_refframe,
                      pMCParam);
#else
    LumaPrediction8x8(currMB, uiBitSize, uiPositionInPic, mb_x, mb_y, fw_mode, bw_mode, fw_refframe, bw_refframe,
                      pMCParam);
#endif
    // !! end shenyanfei

    //===== get displaced frame difference ======

    for (j = 0; j < step_v; j++)
        for (i = 0; i < step_h; i++)

        {
            img->resiY[j][i] = curr_blk[j][i] =
                                   imgY_original[pic_pix_y + j][pic_pix_x + i] - img->predBlock[j + mb_y][i + mb_x];
        }
}
int LumaResidualCoding8x8_TRANS(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cbp,
                                int *cbp_blk, int block8x8, int fw_mode, int bw_mode, int fw_refframe, int bw_refframe, int **curr_blk)
{
    int    i, j;
    int    coeff_cost = 0;
    int    iStartX, iStartY;
    int    iSizeX, iSizeY;
    int    cbp_mask   = 1 << block8x8;
    int    scrFlag = 0;                // 0=noSCR, 1=strongSCR, 2=jmSCR
    int  pix_x    = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int  pix_y    = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;

    int    skipped   = (fw_mode == 0 && bw_mode == 0 && (img->type != B_IMG));

    if (!img->NoResidueDirect) {
        int intraPredMode = DC_PRED;
        if ((currMB->cuType == PSKIPDIRECT || currMB->cuType == P2NX2N || currMB->cuType == PNXN || currMB->trans_size == 0 ||
             IS_INTRA(currMB)) || (input->useNSQT == 0)  || currMB->ui_MbBitSize == B8X8_IN_BIT) {
            transform_B8(curr_blk, uiBitSize, currMB, 0, input->b_secT_enabled, input->sample_bit_depth);
        } else {
            transform_NSQT(curr_blk, uiBitSize, currMB, 0, input->b_secT_enabled, input->sample_bit_depth);
        }
        he->g_block8x8 = block8x8;

        he->g_intra_mode = 0;

#if MB_DQP

        if (IS_INTRA(currMB)) {
            intraPredMode = currMB->l_ipred_mode;
        }
        quantization(currMB->qp - MIN_QP, 0, block8x8, curr_blk, uiBitSize, uiPositionInPic, currMB, 0, intraPredMode);
        coeff_cost = inverse_quantization(currMB->qp - MIN_QP, 0, block8x8, curr_blk, scrFlag, cbp,  uiBitSize, uiPositionInPic,
                                          currMB, 0);
        inverse_transform(block8x8, curr_blk, uiBitSize, uiPositionInPic, currMB, 0);
#else

        if (IS_INTRA(currMB)) {
            intraPredMode = currMB->l_ipred_mode;
        }
        quantization(img->qp - MIN_QP, 0, block8x8, curr_blk,  uiBitSize, uiPositionInPic, currMB, 0, intraPredMode);
        coeff_cost = inverse_quantization(img->qp - MIN_QP, 0, block8x8, curr_blk, scrFlag, cbp,  uiBitSize, uiPositionInPic,
                                          currMB, 0);
        inverse_transform(block8x8, curr_blk, uiBitSize, uiPositionInPic, currMB, 0);

#endif
    }

    if (coeff_cost <= _LUMA_COEFF_COST_) {
        coeff_cost  = 0;
        (*cbp)     &= (63 - cbp_mask);
        (*cbp_blk) &= ~(51 << (4 * block8x8 - 2 * (block8x8 % 2)));

        if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->trans_size == 1)) {
            switch (currMB->cuType) {
            case PSKIPDIRECT:   //SKIP
            case P2NX2N:   //2Nx2N
            case PNXN: { //NxN
                iStartX = (block8x8 % 2) << uiBitSize;
                iStartY = (block8x8 / 2) << uiBitSize;
                iSizeX  = (1 << uiBitSize);
                iSizeY  = (1 << uiBitSize);
                break;
            }
            case P2NXN:   //2NxN
            case PHOR_UP:   //2N_HU
            case PHOR_DOWN: { //2N_HD
                iStartX = 0;
                iStartY = block8x8 * (1 << (uiBitSize - 1));
                iSizeX  = (1 << (uiBitSize + 1));
                iSizeY  = (1 << (uiBitSize - 1));
                break;
            }
            case PNX2N:   //Nx2N
            case PVER_LEFT:   //2N_VL
            case PVER_RIGHT: { //2N_VR
                iStartX = block8x8 * (1 << (uiBitSize - 1));
                iStartY = 0;
                iSizeX  = (1 << (uiBitSize - 1));
                iSizeY  = (1 << (uiBitSize + 1));
                break;
            }
            }
        } else {
            iStartX = (block8x8 % 2) << uiBitSize;
            iStartY = (block8x8 / 2) << uiBitSize;
            iSizeX  = (1 << uiBitSize);
            iSizeY  = (1 << uiBitSize);
        }
        for (j = iStartY; j < iSizeY + iStartY; j++) {
            for (i = iStartX; i < iSizeX + iStartX; i++) {
                hc->imgY[pix_y + j][pix_x + i] = img->predBlock[j][i];
            }
        }

    }

    return coeff_cost;
}


#if Mv_Rang
int LumaResidualCoding8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cbp,
                          int *cbp_blk, int block8x8, int mode, int fw_mode, int bw_mode, int fw_refframe, int bw_refframe, MCParam *pMCParam)
#else
int LumaResidualCoding8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cbp,
                          int *cbp_blk, int block8x8, int fw_mode, int bw_mode, int fw_refframe, int bw_refframe, MCParam *pMCParam)
#endif
{
    int    pic_pix_y, pic_pix_x, i, j;
    int    coeff_cost = 0;
    int    mb_y       = (block8x8 / 2) << uiBitSize;
    int    mb_x       = (block8x8 % 2) << uiBitSize;
    int    cbp_mask   = 1 << block8x8;
    int    scrFlag = 0;                // 0=noSCR, 1=strongSCR, 2=jmSCR
    byte **imgY_original = he->imgY_org;
    int  pix_x    = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int  pix_y    = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;

    int    skipped   = (fw_mode == 0 && bw_mode == 0 && (img->type != B_IMG));
    int    **curr_blk ; // AVS 8x8 pred.error buffer
    int  incr_y = 1, off_y = 0; /*lgp*/

    get_mem2Dint(&curr_blk, 1 << uiBitSize, 1 << uiBitSize);

    if (img->type == B_IMG) {
        scrFlag = 1;
    }

    pic_pix_y = pix_y + mb_y;
    pic_pix_x = pix_x + mb_x;
#if Mv_Rang
    LumaPrediction8x8(currMB, uiBitSize, uiPositionInPic, mb_x, mb_y, mode, fw_mode, bw_mode, fw_refframe, bw_refframe,
                      pMCParam);
#else
    LumaPrediction8x8(currMB, uiBitSize, uiPositionInPic, mb_x, mb_y,f w_mode, bw_mode, fw_refframe, bw_refframe,
                      pMCParam);
#endif
    // !! end shenyanfei

    //===== get displaced frame difference ======
    for (j = 0; j < (1 << uiBitSize); j++) {
        for (i = 0; i < (1 << uiBitSize); i++) {
            img->resiY[j][i] = curr_blk[j][i] =
                                   imgY_original[pic_pix_y + incr_y * j + off_y][pic_pix_x + i] - img->predBlock[j + mb_y][i + mb_x];
        }
    }


    if (!skipped) {
        int intraPredMode = DC_PRED;
        transform_B8(curr_blk, uiBitSize, currMB, 0, input->b_secT_enabled, input->sample_bit_depth);
        he->g_block8x8 = block8x8;

        he->g_intra_mode = 0;

#if MB_DQP

        if (IS_INTRA(currMB)) {
            intraPredMode = currMB->l_ipred_mode;
        }
        quantization(currMB->qp - MIN_QP, 0, block8x8, curr_blk, uiBitSize, uiPositionInPic, currMB, 0, intraPredMode);
        coeff_cost = inverse_quantization(currMB->qp - MIN_QP, 0, block8x8, curr_blk, scrFlag, cbp,  uiBitSize, uiPositionInPic,
                                          currMB, 0);
        inverse_transform(block8x8, curr_blk, uiBitSize, uiPositionInPic, currMB, 0);
#else

        if (IS_INTRA(currMB)) {
            intraPredMode = currMB->l_ipred_mode;
        }
        quantization(img->qp - MIN_QP, 0, block8x8, curr_blk, uiBitSize, uiPositionInPic, currMB, 0, intraPredMode);
        coeff_cost = inverse_quantization(img->qp - MIN_QP, 0, block8x8, curr_blk, scrFlag, cbp,  uiBitSize, uiPositionInPic,
                                          currMB, 0);
        inverse_transform(block8x8, curr_blk, uiBitSize, uiPositionInPic, currMB, 0);

#endif
    }

    if (!skipped && coeff_cost <= _LUMA_COEFF_COST_) {
        coeff_cost  = 0;
        (*cbp)     &= (63 - cbp_mask);
        (*cbp_blk) &= ~(51 << (4 * block8x8 - 2 * (block8x8 % 2)));

        for (i = mb_x; i < mb_x + (1 << uiBitSize); i++) {
            for (j = mb_y; j < mb_y + (1 << uiBitSize); j++) {
                hc->imgY[pix_y + j][pix_x + i] = img->predBlock[j][i];
            }
        }
    }
    free_mem2Dint(curr_blk);
    return coeff_cost;
}

/*
*************************************************************************
* Function:Set mode parameters and reference frames for an 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void SetModesAndRefframe(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int b8,
                         int *fw_mode, int *bw_mode, int *fw_ref, int *bw_ref)
{
    int     **frefarr = hc->refFrArr;
    int     **fw_refarr = img->fw_refFrArr;
    int     **bw_refarr = img->bw_refFrArr;
    int     block_8x = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;
    int     block_8y = (uiPositionInPic / img->PicWidthInMbs) << 1;

    int i, j, m, n;
    get_b8_offset(currMB->cuType, uiBitSize, (b8 % 2), (b8 / 2), &i, &j, &m, &n);

    *fw_mode = *bw_mode = *fw_ref = *bw_ref = -1;

    if (img->type != B_IMG) {
        if (currMB->b8pdir[b8] == DUAL) {
            *fw_ref = frefarr[block_8y + j][block_8x + i];
            *bw_ref = hc->p_snd_refFrArr[block_8y + j][block_8x + i];
            *bw_mode  = -currMB->b8mode[b8];
            *fw_mode  = currMB->b8mode[b8];
        } else if (currMB->md_directskip_mode == BID_P_FST || currMB->md_directskip_mode == BID_P_SND) {
            *fw_ref = frefarr[block_8y + j][block_8x + i];
            *bw_ref = hc->p_snd_refFrArr[block_8y + j][block_8x + i];
            *bw_mode  = 0;
            *fw_mode  = currMB->b8mode[b8];
#if CJ
            assert(currMB->b8mode[b8] == 0);
#endif
        } else {
            *fw_ref = frefarr[block_8y + j][block_8x + i];
            *bw_ref = 0;
            *bw_mode  = 0;
            *fw_mode  = currMB->b8mode[b8];
        }
    } else {
        if (currMB->b8pdir[b8] == INTRA) {
            *fw_ref   = -1;
            *bw_ref   = -1;
            *fw_mode  =  0;
            *bw_mode  =  0;
        } else if (currMB->b8pdir[b8] == FORWARD) {
            *fw_ref   = fw_refarr[block_8y + j][block_8x + i];
            *bw_ref   = 0;
            *fw_mode  = currMB->b8mode[b8];
            *bw_mode  = 0;
        } else if (currMB->b8pdir[b8] == BACKWARD) {
            *fw_ref   = 0;
            *bw_ref   = bw_refarr[block_8y + j][block_8x + i];
            *fw_mode  = 0;
            *bw_mode  = currMB->b8mode[b8];
        } else {

            if (currMB->b8pdir[b8] == BID) {
                *fw_ref   = fw_refarr[block_8y + j][block_8x + i];
                *bw_ref   = bw_refarr[block_8y + j][block_8x + i];
                *fw_mode  = -currMB->b8mode[b8];
                *bw_mode  = -currMB->b8mode[b8];
            } else {
                *fw_ref   = fw_refarr[block_8y + j][block_8x + i];
                *bw_ref   = bw_refarr[block_8y + j][block_8x + i];
                *fw_mode  = currMB->b8mode[b8];
                *bw_mode  = currMB->b8mode[b8];
            }

        }
    }
}
/*
*************************************************************************
* Function:Residual Coding of a Luma codingUnit (not for intra)
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
#if Mv_Rang
void LumaResidualCoding(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, MCParam *pMCParam, int mode)
#else
void LumaResidualCoding(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, MCParam *pMCParam)
#endif
{
    int i, j, block8x8;
    int fw_mode[4], bw_mode[4], refframe;
    int sum_cnt_nonz;
    int stage_block8x8_pos = 0; /*lgp*/
    int pixInPic_x = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int pixInPic_y = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;
    int **curr_blk;
    int bw_ref;

    get_mem2Dint(&curr_blk, 1 << uiBitSize, 1 << uiBitSize);

    currMB->cbp     = 0 ;
    currMB->cbp_blk = 0 ;
    sum_cnt_nonz    = 0 ;
    if (currMB->trans_size == 1) {
        fw_mode_2N = 1;
        bw_mode_2N = 1;
        get_mem2Dint(&curr_blk_PRED, 1 << uiBitSize, 1 << uiBitSize);
        get_mem2Dint(&curr_blk_MPR, 1 << uiBitSize, 1 << uiBitSize);
        {
            for (block8x8 = stage_block8x8_pos; block8x8 < 4; block8x8++) {
                SetModesAndRefframe(currMB, uiBitSize, uiPositionInPic, block8x8, &fw_mode[block8x8], &bw_mode[block8x8], &refframe,
                                    &bw_ref);  //qyu -1
#if Mv_Rang
                LumaResidualCoding8x8_PRED(currMB, uiBitSize - 1, uiPositionInPic, &(currMB->cbp), &(currMB->cbp_blk), block8x8, mode,
                                           fw_mode[block8x8], bw_mode[block8x8], refframe, bw_ref, pMCParam, curr_blk);
#else
                LumaResidualCoding8x8_PRED(currMB, uiBitSize - 1, uiPositionInPic, &(currMB->cbp), &(currMB->cbp_blk), block8x8,
                                           fw_mode[block8x8], bw_mode[block8x8], refframe, bw_ref, pMCParam, curr_blk);
#endif

                {
                    int start_x, start_y, width, height;
                    get_pix_offset(currMB->cuType, uiBitSize, block8x8 % 2, block8x8 / 2, &start_x, &start_y, &width, &height);
                    for (j = 0; j < height; j++) {
                        for (i = 0; i < width; i++) {
                            curr_blk_PRED[j + start_y][i + start_x] = curr_blk[j][i];
                            curr_blk_MPR[j + start_y][i + start_x] = img->predBlock[j + start_y][i + start_x];
                        }
                    }
                }

                fw_mode_2N = fw_mode_2N && fw_mode[block8x8];
                bw_mode_2N = bw_mode_2N && bw_mode[block8x8];
            }
        }
        for (block8x8 = stage_block8x8_pos; block8x8 < 4; block8x8++) {
            if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT) {
                switch (currMB->cuType) {
                case PSKIPDIRECT:   //SKIP
                case P2NX2N:   //2Nx2N
                case PNXN: { //NxN
                    for (j = 0; j < (1 << (uiBitSize - 1)); j++) {
                        for (i = 0; i < (1 << (uiBitSize - 1)); i++) {
                            curr_blk[j][i] = curr_blk_PRED[j + (block8x8 / 2) * (1 << (uiBitSize - 1))][i + (block8x8 % 2) * (1 <<
                                             (uiBitSize - 1))];
                        }
                    }
                    {
                        sum_cnt_nonz += LumaResidualCoding8x8_TRANS(currMB, uiBitSize - 1, uiPositionInPic, &(currMB->cbp),
                                        &(currMB->cbp_blk), block8x8, fw_mode[block8x8], bw_mode[block8x8], refframe, bw_ref, curr_blk);
                    }
                    break;
                }
                case P2NXN:   //2NxN
                case PHOR_UP:   //2N_HU
                case PHOR_DOWN: { //2N_HD
                    int iStartX, iStartY;
                    iStartX = 0;
                    iStartY = block8x8 * (1 << (uiBitSize - 2));
                    for (j = 0; j < (1 << (uiBitSize - 2)); j++) {
                        for (i = 0; i < (1 << uiBitSize); i++) {
                            curr_blk[j][i] = curr_blk_PRED[j + iStartY][i + iStartX];
                        }
                    }
                    sum_cnt_nonz += LumaResidualCoding8x8_TRANS(currMB, uiBitSize - 1, uiPositionInPic, & (currMB->cbp),
                                    & (currMB->cbp_blk), block8x8, fw_mode[block8x8], bw_mode[block8x8], refframe, bw_ref, curr_blk);
                    break;
                }
                case PNX2N:   //Nx2N
                case PVER_LEFT:   //2N_VL
                case PVER_RIGHT: { //2N_VR
                    int iStartX, iStartY;
                    iStartX = block8x8 * (1 << (uiBitSize - 2));
                    iStartY = 0;
                    for (j = 0; j < (1 << uiBitSize); j++) {
                        for (i = 0; i < (1 << (uiBitSize - 2)); i++) {
                            curr_blk[j][i] = curr_blk_PRED[j + iStartY][i + iStartX];
                        }
                    }
                    sum_cnt_nonz += LumaResidualCoding8x8_TRANS(currMB, uiBitSize - 1, uiPositionInPic, & (currMB->cbp),
                                    & (currMB->cbp_blk), block8x8, fw_mode[block8x8], bw_mode[block8x8], refframe, bw_ref, curr_blk);
                    break;
                }
                }
            } else {
                for (j = 0; j < (1 << (uiBitSize - 1)); j++) {
                    for (i = 0; i < (1 << (uiBitSize - 1)); i++) {
                        curr_blk[j][i] = curr_blk_PRED[j + (block8x8 / 2) * (1 << (uiBitSize - 1))][i + (block8x8 % 2) * (1 <<
                                         (uiBitSize - 1))];
                    }
                }
                {
                    sum_cnt_nonz += LumaResidualCoding8x8_TRANS(currMB, uiBitSize - 1, uiPositionInPic, &(currMB->cbp),
                                    &(currMB->cbp_blk), block8x8, fw_mode[block8x8], bw_mode[block8x8], refframe, bw_ref, curr_blk);
                }
            }
        }
    }
    if (currMB->trans_size == 0) {
        for (i = 0; i < (1 << uiBitSize); i++) {
            for (j = 0; j < (1 << uiBitSize); j++) {
                img->predBlock[j][i] = curr_blk_MPR[j][i];
            }
        }
        sum_cnt_nonz += LumaResidualCoding8x8_TRANS(currMB, uiBitSize, uiPositionInPic, & (currMB->cbp),
                        & (currMB->cbp_blk), 0, fw_mode_2N, bw_mode_2N, 0, 0, curr_blk_PRED);
        if (currMB->cbp != 0) {
            currMB->cbp = 15;
        }
        free_mem2Dint(curr_blk_PRED);
        free_mem2Dint(curr_blk_MPR);
    }
    free_mem2Dint(curr_blk);
    if (sum_cnt_nonz <= 5) {
        currMB->cbp     &= 0xfffff0 ;
        currMB->cbp_blk &= 0xff0000 ;
        for (i = 0; i < (1 << uiBitSize); i++) {
            for (j = 0; j < (1 << uiBitSize); j++) {
                hc->imgY[pixInPic_y + j][pixInPic_x + i] = img->predBlock[j][i];
            }
        }
        if ((img->type == INTER_IMG || img->type == F_IMG) && pMCParam->dmh_mode == 0) {
            he->bypass_all_dmh = 1;
        }
    }
}

/*
*************************************************************************
* Function: Predict one component of a chroma 4x4 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
#define  IClip( Min, Max, Val) (((Val)<(Min))? (Min):(((Val)>(Max))? (Max):(Val)))

void OneComponentChromaPrediction4x4(int uiPositionInPic, int b8, int *mpred, int pix_c_x, int pix_c_y, int step_h,
                                     int step_v, int *mv, int ref, int blocktype, int uv, int directforword)
{
    int     i, j, ii, jj;
    int     incr;
    int    *mvb;
    int     refframe  = (ref < 0 ?      0 :    ref);
    pel_t **refimage;
    int     f1        = 8 , f2 = f1 - 1, f3 = f1 * f1, f4 = f3 >> 1;
    int     s1        = 3;
    int     scale   = 1;
    int     mpred_tmp [2];
    int     k;
    int     dmh_mode = 0;

    int     je        = pix_c_y + step_v;
    int     ie        = pix_c_x + step_h;
    int shift1 = input->sample_bit_depth - 8;
    int add1 = (1 << (shift1)) >> 1;
    int shift2 = 20 - input->sample_bit_depth;
    int add2 = 1 << (19 - input->sample_bit_depth);
    int max_pel_value = (1 << input->sample_bit_depth) - 1;
#if HALF_PIXEL_CHROMA
    int ref_bak = ref;
#endif
    incr = 1;

    ref = (img->type == B_IMG) ? ref + incr : ref;

    refimage = he->integerRefUV[ref][uv];

    if (input->bg_enable && he->background_reference_enable &&
        ref == img->num_of_references - 1 &&
        (img->type == P_IMG || img->type == F_IMG) && img->typeb != BP_IMG) {
        refimage = he->background_frame[uv + 1];
    }

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        refimage = he->background_frame[uv + 1];
    }


    for (j = pix_c_y; j < je; j++) {
        for (i = pix_c_x; i < ie; i++) {
            if (input->chroma_format == 1) {
                mvb  = mv;
                ii   = (i << s1) + mvb[0];   //s1 1/8 interpolation
                jj   = (j << s1) + mvb[1];
#if HALF_PIXEL_CHROMA
                if (img->is_field_sequence && input->chroma_format == 1) {
                    //int fw_bw = directforword == 0 ? -1 : 0;
                    int fw_bw = ref_bak;
                    int distance = calculate_distance(ref, fw_bw);
                    int delta = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                    jj   -= delta;
                }
#endif

                if (img->type == F_IMG && input->b_dmh_enabled && img->typeb != BP_IMG)

                {
                    dmh_mode = mvb[2];
                } else {
                    dmh_mode = 0;
                }
            }

            for (k = 0; k < 2; k++) {
                int posx, posy;
                int curx, cury;
                int x0, x1, x2, x3;
                int y0, y1, y2, y3;

#if Mv_Rang
				ii=Clip3(-32768, 32767,(i << 3) + mvb[0] + dmh_pos[dmh_mode][k][0]);
				jj=Clip3(-32768, 32767,(j << 3) + mvb[1] + dmh_pos[dmh_mode][k][1]);
#else
                ii = (i << 3) + mvb[0] + dmh_pos[dmh_mode][k][0];
                jj = (j << 3) + mvb[1] + dmh_pos[dmh_mode][k][1];
#endif
				
#if HALF_PIXEL_CHROMA
                if (img->is_field_sequence && input->chroma_format == 1) {
                    //int fw_bw = directforword == 0 ? -1 : 0;
                    int fw_bw = ref_bak;
                    int distance = calculate_distance(ref, fw_bw);
                    int delta = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                    jj -= delta;
                }
#endif
                posx = (ii & 7);
                posy = (jj & 7);
                curx = ii >> 3;
                cury = jj >> 3;
                // get mpred_tmp[k]
                if (posy == 0) {
                    x0 = max(0, min(img->width_cr - 1, curx - 1));
                    x1 = max(0, min(img->width_cr - 1, curx - 0));
                    x2 = max(0, min(img->width_cr - 1, curx + 1));
                    x3 = max(0, min(img->width_cr - 1, curx + 2));
                    y0 = y1 = y2 = y3 = max(0, min(img->height_cr - 1, cury));
                    mpred_tmp[k] = refimage[y0][x0] * IF_CHROMA_COEF[posx][0] +
                                   refimage[y1][x1] * IF_CHROMA_COEF[posx][1] +
                                   refimage[y2][x2] * IF_CHROMA_COEF[posx][2] +
                                   refimage[y3][x3] * IF_CHROMA_COEF[posx][3];
                    mpred_tmp[k] = IClip(0, max_pel_value, (mpred_tmp[k] + 32) >> 6);
                } else if (posx == 0) {
                    y0 = max(0, min(img->height_cr - 1, cury - 1));
                    y1 = max(0, min(img->height_cr - 1, cury - 0));
                    y2 = max(0, min(img->height_cr - 1, cury + 1));
                    y3 = max(0, min(img->height_cr - 1, cury + 2));
                    x0 = x1 = x2 = x3 = max(0, min(img->width_cr - 1, curx));
                    mpred_tmp[k] = refimage[y0][x0] * IF_CHROMA_COEF[posy][0] +
                                   refimage[y1][x1] * IF_CHROMA_COEF[posy][1] +
                                   refimage[y2][x2] * IF_CHROMA_COEF[posy][2] +
                                   refimage[y3][x3] * IF_CHROMA_COEF[posy][3];
                    mpred_tmp[k] = IClip(0, max_pel_value, (mpred_tmp[k] + 32) >> 6);
                } else {
                    int tmp[4], line;
                    for (line = -1; line < 3; line++) {
                        x0 = max(0, min(img->width_cr - 1, curx - 1));
                        x1 = max(0, min(img->width_cr - 1, curx - 0));
                        x2 = max(0, min(img->width_cr - 1, curx + 1));
                        x3 = max(0, min(img->width_cr - 1, curx + 2));
                        y0 = y1 = y2 = y3 = max(0, min(img->height_cr - 1, cury + line));
                        tmp[line + 1] = refimage[y0][x0] * IF_CHROMA_COEF[posx][0] +
                                        refimage[y1][x1] * IF_CHROMA_COEF[posx][1] +
                                        refimage[y2][x2] * IF_CHROMA_COEF[posx][2] +
                                        refimage[y3][x3] * IF_CHROMA_COEF[posx][3];
                        tmp[line + 1] = (tmp[line + 1] + add1) >> shift1;
                    }
                    mpred_tmp[k] = tmp[0] * IF_CHROMA_COEF[posy][0] +
                                   tmp[1] * IF_CHROMA_COEF[posy][1] +
                                   tmp[2] * IF_CHROMA_COEF[posy][2] +
                                   tmp[3] * IF_CHROMA_COEF[posy][3];
                    mpred_tmp[k] = IClip(0, max_pel_value, (mpred_tmp[k] + add2) >> shift2);
                }
            }
            *mpred++ = (mpred_tmp[0] + mpred_tmp[1] + 1) >> 1;
        }
    }
}

/*
*************************************************************************
* Function:Predict one component of a chroma 4x4 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void OneComponentChromaPrediction4x4_dir(int uiPositionInPic, int b8, int *mpred, int pix_c_x, int pix_c_y, int step_h,
        int step_v, int *mv, int ref, int blocktype, int uv, int refframe)
{
    int     i, j, ii, jj;
    int     incr;
    int    *mvb;
    pel_t **refimage;
    int     f1        = 8 , f2 = f1 - 1, f3 = f1 * f1, f4 = f3 >> 1;
    int     s1        = 3;
    int   scale   = 1;


    int     je        = pix_c_y + step_v;
    int     ie        = pix_c_x + step_h;

    int shift1 = input->sample_bit_depth - 8;
    int add1 = (1 << (shift1)) >> 1;
    int shift2 = 20 - input->sample_bit_depth;
    int add2 = 1 << (19 - input->sample_bit_depth);
    int max_pel_value = (1 << input->sample_bit_depth) - 1;
    int posx, posy;
    int curx, cury;
    int x0, x1, x2, x3;
    int y0, y1, y2, y3;

    incr = 1;

    ref = (img->type == B_IMG) ? ref + incr : ref;

    refimage = he->integerRefUV[ref][uv];


    for (j = pix_c_y; j < je; j++) {
        for (i = pix_c_x; i < ie; i++) {
            if (input->chroma_format == 1) {
                mvb  = mv;
            }

            {
                int delta_P, TRp, DistanceIndexFw, DistanceIndexBw, delta_PB;
                delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
                delta_P = (delta_P + 512) % 512;

                TRp = refframe == 0 ? delta_P : (2 * (img->imgtr_next_P - fref[1]->imgtr_fwRefDistance) + 512) % 512;
                delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);
                TRp = (TRp + 512) % 512;
                delta_PB = (delta_PB + 512) % 512;
                DistanceIndexFw = delta_PB;

                DistanceIndexBw = (TRp - DistanceIndexFw + 512) % 512;

#if MV_SCALE
                ii = (i << s1) - scale_mv(mvb[0], DistanceIndexBw, DistanceIndexFw);
#else
                ii = (i << s1) - ((mvb[0] * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
#endif
                ii = Clip3(-32768, 32768, ii);

                if (input->chroma_format == 1) {
#if MV_SCALE
                    jj = (j << s1) - scale_mv(mvb[1], DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
                    if (img->is_field_sequence) {
                        jj = (j << s1) - scale_mv_y2(mvb[1], DistanceIndexBw, DistanceIndexFw);
                    }
#endif
#else
                    jj = (j << s1) - ((mvb[1] * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
#if HALF_PIXEL_COMPENSATION_MVD
                    if (img->is_field_sequence) {
                        int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                        oriPOC = 2 * hc->picture_distance;
                        oriRefPOC = oriPOC - DistanceIndexFw;
                        scaledPOC = 2 * hc->picture_distance;
                        scaledRefPOC = scaledPOC - DistanceIndexBw;
                        getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                        jj = (j << s1) - (((mvb[1] + delta) * DistanceIndexBw * (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                    }
#endif
#endif
#if HALF_PIXEL_CHROMA
                    if (img->is_field_sequence && input->chroma_format == 1) {
                        int fw_bw = -1;
                        int distance2 = calculate_distance(ref, fw_bw);
                        int distance = DistanceIndexBw;
                        int delta = (distance % 4) == 0 ? 0 : img->is_top_field ? 2 : -2;
                        assert(distance2 == distance);
                        jj   -= delta;
                    }
#endif
                    jj = Clip3(-32768, 32768, jj);
                }
            }

            posx = (ii & 7);
            posy = (jj & 7);
            curx = ii >> 3;
            cury = jj >> 3;
            // get mpred_tmp[k]
            if (posy == 0) {
                x0 = max(0, min(img->width_cr - 1, curx - 1));
                x1 = max(0, min(img->width_cr - 1, curx - 0));
                x2 = max(0, min(img->width_cr - 1, curx + 1));
                x3 = max(0, min(img->width_cr - 1, curx + 2));
                y0 = y1 = y2 = y3 = max(0, min(img->height_cr - 1, cury));
                *mpred = refimage[y0][x0] * IF_CHROMA_COEF[posx][0] +
                         refimage[y1][x1] * IF_CHROMA_COEF[posx][1] +
                         refimage[y2][x2] * IF_CHROMA_COEF[posx][2] +
                         refimage[y3][x3] * IF_CHROMA_COEF[posx][3];
                *mpred = IClip(0, max_pel_value, (*mpred + 32) >> 6);
            } else if (posx == 0) {
                y0 = max(0, min(img->height_cr - 1, cury - 1));
                y1 = max(0, min(img->height_cr - 1, cury - 0));
                y2 = max(0, min(img->height_cr - 1, cury + 1));
                y3 = max(0, min(img->height_cr - 1, cury + 2));
                x0 = x1 = x2 = x3 = max(0, min(img->width_cr - 1, curx));
                *mpred = refimage[y0][x0] * IF_CHROMA_COEF[posy][0] +
                         refimage[y1][x1] * IF_CHROMA_COEF[posy][1] +
                         refimage[y2][x2] * IF_CHROMA_COEF[posy][2] +
                         refimage[y3][x3] * IF_CHROMA_COEF[posy][3];
                *mpred = IClip(0, max_pel_value, (*mpred + 32) >> 6);
            } else {
                int tmp[4], line;
                for (line = -1; line < 3; line++) {
                    x0 = max(0, min(img->width_cr - 1, curx - 1));
                    x1 = max(0, min(img->width_cr - 1, curx - 0));
                    x2 = max(0, min(img->width_cr - 1, curx + 1));
                    x3 = max(0, min(img->width_cr - 1, curx + 2));
                    y0 = y1 = y2 = y3 = max(0, min(img->height_cr - 1, cury + line));
                    tmp[line + 1] = refimage[y0][x0] * IF_CHROMA_COEF[posx][0] +
                                    refimage[y1][x1] * IF_CHROMA_COEF[posx][1] +
                                    refimage[y2][x2] * IF_CHROMA_COEF[posx][2] +
                                    refimage[y3][x3] * IF_CHROMA_COEF[posx][3];
                    tmp[line + 1] = (tmp[line + 1] + add1) >> shift1;
                }
                *mpred = tmp[0] * IF_CHROMA_COEF[posy][0] +
                         tmp[1] * IF_CHROMA_COEF[posy][1] +
                         tmp[2] * IF_CHROMA_COEF[posy][2] +
                         tmp[3] * IF_CHROMA_COEF[posy][3];
                *mpred = IClip(0, max_pel_value, (*mpred + add2) >> shift2);
            }
            mpred++;
        }
    }
}
/*
*************************************************************************
* Function:Predict an intra chroma 4x4 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void IntraChromaPrediction4x4(codingUnit *currMB, unsigned int uiBitSize, int  uv, int  block_x, int  block_y)
{
    int mode = currMB->c_ipred_mode;
    int i, j;

    //===== prediction =====
    for (j = block_y; j < block_y + (1 << uiBitSize); j++) {
        for (i = block_x; i < block_x + (1 << uiBitSize); i++) {
            img->predBlock[j][i] = img->predBlockUV[uv][mode][j][i];
        }
    }
}

/*
*************************************************************************
* Function:Predict one chroma 4x4 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void ChromaPrediction4x4(codingUnit *currMB, int uiBitSize, int uiPositionInPic, int b8, int uv, int block_x,
                         int block_y, int step_h, int step_v, int fw_mode, int bw_mode, int fw_ref_frame, int bw_ref_frame, MCParam *pMCParam)
{

    int *fw_pred = (int *) malloc((1 << (uiBitSize + 1)) * (1 << (uiBitSize + 1)) * sizeof(int));
    int *bw_pred = (int *) malloc((1 << (uiBitSize + 1)) * (1 << (uiBitSize + 1)) * sizeof(int));

    int *fw_pred_wgt = (int *) malloc((1 << (uiBitSize + 1)) * (1 << (uiBitSize + 1)) * sizeof(int));

    int  i, j;

    int  block_x4  = block_x + step_h;
    int  block_y4  = block_y + step_v;

    int  picC_pix_y = input->chroma_format == 1 ? ((uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE / 2) : ((
                          uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE);
    int  picC_pix_x = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE / 2;
    int  pic_pix_x = picC_pix_x + block_x;
    int  pic_pix_y = picC_pix_y + block_y;
    int *fpred     = fw_pred;
    int *bpred     = bw_pred;
    int *fpred_wgt = fw_pred_wgt;
    //     int  by;
    //     int  bx = ( block_x >> uiBitSize );

    int  direct    = (fw_mode == 0 && bw_mode == 0 && (img->type == B_IMG));

    int  skipped   = (fw_mode == 0 && bw_mode == 0 && (img->type != B_IMG));
    int ** ***fmv_array = (fw_mode < 0) ? img->allBidFwMv : (fw_mode > 0  &&
                          bw_mode > 0) ? img->allSymMv : (fw_mode > 0  && bw_mode < 0) ? img->allDualFstMv : img->allFwMv;
    int ** ***bmv_array = (fw_mode < 0) ? img->allBidBwMv : (fw_mode > 0  &&
                          bw_mode < 0) ? img->allDualSndMv : img->allBwMv;
    int  dual = (fw_mode > 0 && bw_mode < 0) ? 1 : 0;
    int bid = (fw_mode < 0 && bw_mode < 0) ? 1 : 0;

    int *mv;
    int ref;

    int directforward = (img->type == B_IMG && fw_mode == 0);

#if FIX_MAX_REF
    int delta_P[MAXREF];
#else
    int delta_P[4];
#endif

    int num_skipmode = pMCParam->num_skipmode;

#if FIX_MAX_REF
    for (i = 0; i < img->num_of_references; i++) {
#else
    for (i = 0; i < 4; i++) {
#endif
        delta_P[i] = (2 * (hc->picture_distance - fref[i]->imgtr_fwRefDistance) + 512) % 512;
        if (i == img->num_of_references - 1 && he->background_reference_enable) {
            delta_P[i] = 1;
        }
    }

    if (fw_mode < 0 && bw_mode < 0) {
        fw_mode = -fw_mode;
        bw_mode = -bw_mode;
    }

    if (fw_mode > 0 && bw_mode < 0) {
        bw_mode = -bw_mode;
    }
    //===== INTRA PREDICTION =====
    if (IS_INTRA(currMB)) {

        int mode = currMB->c_ipred_mode;
        for (j = block_y; j < block_y + step_h; j++) {
            for (i = block_x; i < block_x + step_v; i++) {
                img->predBlock[j][i] = img->predBlockUV[uv][mode][j][i];
            }
        }

        free(fw_pred);
        free(bw_pred);
        free(fw_pred_wgt);
        return;
    }
    //===== INTER PREDICTION =====
    if (fw_mode || (direct && (fw_ref_frame != -1)) || skipped) {
        if (pMCParam->dir != 0) {
            if (img->type == P_IMG || img->type == F_IMG) {
                mv = img->tmp_fstPSkipMv[pMCParam->dir];
            } else {
                mv = img->tmp_fwBSkipMv[pMCParam->dir];
            }
        } else {
            ref = (directforward ? 0 : fw_ref_frame);
            ref = (ref < 0 ?      0 :    ref);
            mv = fmv_array [ b8 / 2 ][ b8 % 2 ][ref][fw_mode];
        }
        OneComponentChromaPrediction4x4(uiPositionInPic, b8, fw_pred, pic_pix_x, pic_pix_y, step_h, step_v, mv ,
                                        (directforward ? 0 : fw_ref_frame), fw_mode, uv, (directforward ? 1 : 0));

        if (img->type == F_IMG && num_skipmode && fw_mode == 0) {
            int mv_wgt[3];
#if MV_SCALE
            mv_wgt[0] = scale_mv(fmv_array[ b8 / 2 ][ b8 % 2 ][ref][fw_mode][0], delta_P[num_skipmode], delta_P[0]);
            mv_wgt[1] = scale_mv(fmv_array[b8 / 2][b8 % 2][ref][fw_mode][1], delta_P[num_skipmode], delta_P[0]);
#if HALF_PIXEL_COMPENSATION_M1
            //assert(is_field_sequence == img->is_field_sequence);
            if (img->is_field_sequence) {
                mv_wgt[1] = scale_mv_y1(fmv_array[b8 / 2][b8 % 2][ref][fw_mode][1], delta_P[num_skipmode], delta_P[0]);
            }
#endif
#else
            mv_wgt[0] = (delta_P[num_skipmode] * fmv_array[ b8 / 2 ][ b8 % 2 ][ref][fw_mode][0] *
                         (MULTI / delta_P[0]) + HALF_MULTI) >> OFFSET;
            mv_wgt[1] = (delta_P[num_skipmode] * fmv_array[ b8 / 2 ][ b8 % 2 ][ref][fw_mode][1] *
                         (MULTI / delta_P[0]) + HALF_MULTI) >> OFFSET;
#if HALF_PIXEL_COMPENSATION_M1
            //assert(is_field_sequence == img->is_field_sequence);
            if (img->is_field_sequence) {
                int delta, delta2;
                int oriPOC = 2 * hc->picture_distance;
                int oriRefPOC = oriPOC - delta_P[0];
                int scaledPOC = 2 * hc->picture_distance;
                int scaledRefPOC = scaledPOC - delta_P[num_skipmode];
                getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                mv_wgt[1] = ((delta_P[num_skipmode] * (fmv_array[ b8 / 2 ][ b8 % 2 ][ref][fw_mode][1] + delta) *
                              (MULTI / delta_P[0]) + HALF_MULTI) >> OFFSET) - delta2;
            }
#endif
#endif

            mv_wgt[2] = 0;
            OneComponentChromaPrediction4x4(uiPositionInPic, b8, fw_pred_wgt, pic_pix_x, pic_pix_y, step_h, step_v, mv_wgt ,
                                            num_skipmode, fw_mode, uv, (directforward ? 1 : 0));
            for (j = block_y; j < block_y4; j++) {
                for (i = block_x; i < block_x4; i++) {
                    *fpred = (*fpred + *fpred_wgt + 1) / 2;
                    fpred++;
                    fpred_wgt++;
                }
            }
            fpred = fw_pred;
        }

        if (img->type == F_IMG && input->dhp_enabled && dual) {
            mv = bmv_array [ b8 / 2 ][ b8 % 2 ][fw_ref_frame][bw_mode];;
            OneComponentChromaPrediction4x4(uiPositionInPic, b8, fw_pred_wgt, pic_pix_x, pic_pix_y, step_h, step_v, mv ,
                                            (directforward ? 0 : bw_ref_frame), bw_mode, uv, (directforward ? 1 : 0));

            for (j = block_y; j < block_y4; j++) {
                for (i = block_x; i < block_x4; i++) {
                    *fpred = (*fpred + *fpred_wgt + 1) / 2;
                    fpred++;
                    fpred_wgt++;
                }
            }
            fpred = fw_pred;
        }

        if (img->type == F_IMG && (pMCParam->dir == BID_P_FST || pMCParam->dir == BID_P_SND)) {
            mv = img->tmp_sndPSkipMv[pMCParam->dir];
            OneComponentChromaPrediction4x4(uiPositionInPic, b8, fw_pred_wgt, pic_pix_x, pic_pix_y, step_h, step_v, mv ,
                                            (directforward ? 0 : bw_ref_frame), fw_mode, uv, (directforward ? 1 : 0));

            for (j = block_y; j < block_y4; j++) {
                for (i = block_x; i < block_x4; i++) {
                    *fpred = (*fpred + *fpred_wgt + 1) / 2;
                    fpred++;
                    fpred_wgt++;
                }
            }
            fpred = fw_pred;
        }
    }

    if (img->type == B_IMG && (bw_mode || (direct && (bw_ref_frame != -1)))) {
        if (fw_mode && bw_mode && !bid) {
            if (pMCParam->dir != 0) {
                mv = img->tmp_fwBSkipMv[pMCParam->dir];
            } else {
                mv = fmv_array [ b8 / 2 ][ b8 % 2 ][fw_ref_frame][bw_mode];
            }
            OneComponentChromaPrediction4x4_dir(uiPositionInPic, b8, bw_pred, pic_pix_x, pic_pix_y, step_h, step_v, mv,
                                                -1, bw_mode, uv, fw_ref_frame);
        } else {

            if (pMCParam->dir != 0) {
                mv = img->tmp_bwBSkipMv[pMCParam->dir];
            } else {
                ref = -1;
                ref = (ref < 0 ?      0 :    ref);
                mv = bmv_array [ b8 / 2 ][ b8 % 2 ][ref][bw_mode];
            }
            OneComponentChromaPrediction4x4(uiPositionInPic, b8, bw_pred, pic_pix_x, pic_pix_y, step_h, step_v, mv,
                                            -1, bw_mode, uv, 0);
        }
    }

    if (direct || (fw_mode && bw_mode && img->type == B_IMG)) {
        if (pMCParam->dir == 0 || pMCParam->dir == DS_BID || pMCParam->dir == DS_SYM) {
            for (j = block_y; j < block_y4; j++) {
                for (i = block_x; i < block_x4; i++) {
                    img->predBlock[j][i] = (*fpred + *bpred + 1) / 2;
                    fpred++ ;
                    bpred++ ;
                }
            }
        } else if (pMCParam->dir == DS_BACKWARD) {
            for (j = block_y; j < block_y4; j++) {
                for (i = block_x; i < block_x4; i++) {
                    img->predBlock[j][i] = *bpred;
                    bpred++ ;
                }
            }
        } else if (pMCParam->dir == DS_FORWARD) {
            for (j = block_y; j < block_y4; j++) {
                for (i = block_x; i < block_x4; i++) {
                    img->predBlock[j][i] = *fpred;
                    fpred++ ;
                }
            }
        }
    } else if (fw_mode || skipped) {   //P fw and B one direction fw
        for (j = block_y; j < block_y4; j++) {
            for (i = block_x; i < block_x4; i++) {
                img->predBlock[j][i] = *fpred;
                fpred++;
            }
        }
    } else { //B bw
        for (j = block_y; j < block_y4; j++) {
            for (i = block_x; i < block_x4; i++) {
                img->predBlock[j][i] = *bpred;
                bpred++;
            }
        }
    }

    free(fw_pred);
    free(bw_pred);
    free(fw_pred_wgt);
}

/*
*************************************************************************
* Function:Chroma residual coding for an codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void ChromaResidualCoding(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cr_cbp,
                          MCParam *pMCparam)
{
    int   uv, block8, block_y, block_x, j, i;
    int   fw_mode, bw_mode, refframe;
    int   skipped = (currMB->cuType == PSKIPDIRECT && ((img->type == F_IMG) || (img->type == P_IMG)));
    int   bw_ref;
    int   **tmp_block_88;
    int   ***tmp_block;//[2][8][8];/*lgp*/
    int   ***tmp_mpr;//[2][8][8];/*lgp*/
    int i0, j0;

    // Adaptive frequency weighting quantization
#if (MB_DQP||FREQUENCY_WEIGHTING_QUANTIZATION)
    int   CurrentQP, CurrentChromaQP;
#endif

    int IntraPrediction = IS_INTRA(currMB);
    int pix_c_y = input->chroma_format == 1 ? ((uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE / 2) : ((
                      uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE);
    int pix_c_x = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE / 2;

    int step_h, step_v;

    get_mem2Dint(&tmp_block_88, (1 << (uiBitSize - 1)), (1 << (uiBitSize - 1)));
    get_mem3Dint(&tmp_block, 2, (1 << (uiBitSize - 1)), (1 << (uiBitSize - 1)));
    get_mem3Dint(&tmp_mpr, 2, (1 << (uiBitSize - 1)), (1 << (uiBitSize - 1)));

    if (input->chroma_format == 1) {   //qyu 0824
        for (*cr_cbp = 0, uv = 0; uv < 2; uv++) {
            //===== prediction of chrominance blocks =====
            block8 = 0;

            {
                for (j0 = 0; j0 < 2; j0++) {
                    for (i0 = 0; i0 < 2; i0++, block8++) {
                        if (!IS_INTRA(currMB)) {
                            get_pix_offset(currMB->cuType, uiBitSize, i0, j0, &block_x, &block_y, &step_h, &step_v);
                        } else {
                            block_x = i0 << (uiBitSize - 1);
                            block_y = j0 << (uiBitSize - 1);
                            step_h = step_v = 1 << (uiBitSize - 1);
                        }
                        block_x /= 2;
                        block_y = input->chroma_format == 1 ? block_y / 2 : block_y;
                        step_h /= 2;
                        step_v = input->chroma_format == 1 ? step_v / 2 : step_v;
                        SetModesAndRefframe(currMB, uiBitSize, uiPositionInPic, block8, &fw_mode, &bw_mode, &refframe, &bw_ref);
                        ChromaPrediction4x4(currMB, uiBitSize - 2, uiPositionInPic, block8, uv, block_x, block_y, step_h, step_v, fw_mode,
                                            bw_mode, refframe, bw_ref, pMCparam); //qyu 0824
                        if (img->NoResidueDirect) {
                            for (j = block_y; j < block_y + step_v; j++) {
                                for (i = block_x; i < block_x + step_h; i++) {
                                    hc->imgUV[uv][pix_c_y + j][pix_c_x + i] = img->predBlock[j][i];
                                }
                            }
                        } else {
                            for (j = block_y; j < block_y + step_v; j++) {
                                for (i = block_x; i < block_x + step_h; i++) {
                                    tmp_mpr[uv][j][i] = img->predBlock[j][i];
                                    tmp_block[uv][j][i] = he->imgUV_org[uv][pix_c_y + j][pix_c_x + i] - img->predBlock[j][i];
                                    img->resiY[j][i] = he->imgUV_org[uv][pix_c_y + j][pix_c_x + i] - img->predBlock[j][i];
                                }
                            }
                        }
                    }
                }
            }


            //===== DCT, Quantization, inverse Quantization, IDCT, and Reconstruction =====

            if (!img->NoResidueDirect) {
                for (j = 0; j < (1 << (uiBitSize - 1)); j++) {
                    for (i = 0; i < (1 << (uiBitSize - 1)); i++) {
                        tmp_block_88[j][i] = img->resiY[j][i];
                    }
                }
                he->g_block8x8 = uv + 4;

                he->g_intra_mode = IntraPrediction;
                transform_B8(tmp_block_88, (uiBitSize - 1), currMB, 1, input->b_secT_enabled, input->sample_bit_depth);

#if MB_DQP
                CurrentQP = currMB->qp;
#if CHROMA_DELTA_QP
                if (!input->chroma_quant_param_disable)
                { CurrentQP = CurrentQP + (uv == 0 ? input->chroma_quant_param_delta_u : input->chroma_quant_param_delta_v); }

#endif
                if (input->sample_bit_depth > 8) {
                    CurrentChromaQP = CurrentQP - 8 * (input->sample_bit_depth - 8);
                    CurrentChromaQP = CurrentChromaQP < 0 ? CurrentChromaQP : QP_SCALE_CR[CurrentChromaQP];
                    CurrentChromaQP = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), CurrentChromaQP + 8 * (input->sample_bit_depth - 8));
                } else {
                    CurrentChromaQP = QP_SCALE_CR[Clip3(0, 63, CurrentQP - MIN_QP)];
                }
#else
                if (input->sample_bit_depth > 8) {
                    CurrentQP = currMB->qp;
#if CHROMA_DELTA_QP
                    if (!input->chroma_quant_param_disable)
                    { CurrentQP = CurrentQP + (uv == 0 ? input->chroma_quant_param_delta_u : input->chroma_quant_param_delta_v); }

#endif
                    CurrentChromaQP = QP_SCALE_CR[ Clip3(0, 63,
                                                         (CurrentQP  - (8 * (input->sample_bit_depth - 8)) - MIN_QP))] + (8 * (input->sample_bit_depth - 8));
                } else {
                    CurrentQP = img->qp;
#if CHROMA_DELTA_QP
                    if (!input->chroma_quant_param_disable)
                    { CurrentQP = CurrentQP + (uv == 0 ? input->chroma_quant_param_delta_u : input->chroma_quant_param_delta_v); }

#endif
                    CurrentChromaQP = QP_SCALE_CR[ CurrentQP - MIN_QP];
                }
#endif
                quantization(CurrentChromaQP, IntraPrediction ? 4 : 0, 4 + uv, tmp_block_88, (uiBitSize - 1), uiPositionInPic,
                             currMB, 1, DC_PRED);
                inverse_quantization(CurrentChromaQP, IntraPrediction ? 4 : 0, 4 + uv, tmp_block_88, 0, cr_cbp, (uiBitSize - 1),
                                     uiPositionInPic, currMB, 1);
                inverse_transform(4 + uv, tmp_block_88, (uiBitSize - 1), uiPositionInPic, currMB, 1);

            }
        }

        //===== update currMB->cbp =====
        currMB->cbp += (*cr_cbp);   //((*cr_cbp)<<4); /*lgp*dct*/
    }

    free_mem2Dint(tmp_block_88);
    free_mem3Dint(tmp_block, 2);
    free_mem3Dint(tmp_mpr, 2);
}
/*
*************************************************************************
* Function:Predict an intra chroma 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void IntraChromaPrediction8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *mb_up,
                              int *mb_left, int *mb_up_left)
{
    //short * edgepixels = ( short * ) malloc ( ( ( 1 << uiBitSize ) * 4 + 1 ) * sizeof ( short ) );
    short edgepixels[MAX_CU_SIZE * 4 + 1];
#define EP ( edgepixels + ( ( 1 << uiBitSize ) * 2 ) )
    int     bs_x = (1 << uiBitSize);
    int     bs_y = (1 << uiBitSize);
    int     x, y;
    int     PicWidthInMB          = img->width / MIN_CU_SIZE;
    int     i;
    int block8_y = (uiPositionInPic / img->PicWidthInMbs) << 1;
    int block8_x = (uiPositionInPic % img->PicWidthInMbs) << 1;
    int predLmode = img->ipredmode[1 + block8_y][1 + block8_x];
    int mb_available_left_down;
    int p_avai[5];
    int mb_pix_x;
    int     img_cx            = ((uiPositionInPic % PicWidthInMB) * MIN_CU_SIZE) >> 1;
    int     img_cy            = input->chroma_format == 1 ? (((uiPositionInPic / PicWidthInMB) * MIN_CU_SIZE) >> 1)
                                : ((uiPositionInPic / PicWidthInMB) * MIN_CU_SIZE);     //img->pix_c_y;
    int     img_cx_1          = img_cx - 1;
    int     img_cy_1          = img_cy - 1;
    int     b8_x              = img_cx / (MIN_BLOCK_SIZE >> 1);
    int     b8_y              = img_cy / (MIN_BLOCK_SIZE >> 1);
    int     mb_available_up_right;
    int     mb_available_up;
    int     mb_available_left;
    int     mb_available_up_left;
    int     uv;
    int numPUWidthInSMB = (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT));
    int HalfPixWidth = 1 << (uiBitSize - 1);
    int mb_x, mb_y;
    int mb_pix_y;
    int **piPredBuf;

    get_mem2Dint(&piPredBuf, bs_y, bs_x);
    get_mb_pos(uiPositionInPic, &mb_x, &mb_y, input->g_uiMaxSizeInBit);
    mb_pix_y = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE - mb_y;
    mb_pix_x = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE - mb_x;

    getIntraNeighborAvailabilities(currMB, input->g_uiMaxSizeInBit, (uiPositionInPic % PicWidthInMB) * MIN_CU_SIZE,
                                   (uiPositionInPic / PicWidthInMB) * MIN_CU_SIZE,
                                   (1 << (uiBitSize + 1)), (1 << (uiBitSize + 1)), p_avai);

    mb_available_left_down = p_avai[NEIGHBOR_INTRA_LEFT_DOWN];
    mb_available_left      = p_avai[NEIGHBOR_INTRA_LEFT];
    mb_available_up_left   = p_avai[NEIGHBOR_INTRA_UP_LEFT];
    mb_available_up        = p_avai[NEIGHBOR_INTRA_UP];
    mb_available_up_right  = p_avai[NEIGHBOR_INTRA_UP_RIGHT];

    if (mb_up) {
        *mb_up = mb_available_up;
    }

    if (mb_left) {
        *mb_left = mb_available_left;
    }

    if (mb_up_left) {
        *mb_up_left = mb_available_up_left;
    }

    // compute all chroma intra prediction modes for both U and V
    for (uv = 0; uv < 2; uv++) {
        for (i = -2 * bs_y; i <= 2 * bs_x; i++) {
            EP[i] = 1 << (input->sample_bit_depth - 1);
        }

        if (mb_available_up) {
            for (x = 0; x < bs_x; x++) {
                EP[x + 1] = hc->imgUV[uv][img_cy - 1][img_cx + x];
            }

        }

        if (mb_available_up_right) {
            for (x = 0; x < bs_x; x++) {
                if (img_cx + bs_x + x >= img->width_cr) {
                    EP[1 + x + bs_x] = hc->imgUV[uv][img_cy - 1][img->width_cr - 1];
                } else {
                    EP[1 + x + bs_x] = hc->imgUV[uv][img_cy - 1][img_cx + bs_x + x];
                }
            }
        } else {
            for (x = 0; x < bs_x; x++) {
                EP[1 + x + bs_x] = EP[bs_x];
            }
        }

        if (mb_available_left) {
            for (y = 0; y < bs_y; y++) {
                EP[-1 - y] = hc->imgUV[uv][img_cy + y][img_cx - 1];
            }

        }

        if (mb_available_left_down) {
            for (y = 0; y < bs_y; y++) {
                if (img_cy + bs_y + y >= img->height_cr) {
                    EP[-1 - y - bs_y] = hc->imgUV[uv][img->height_cr - 1][img_cx - 1];
                } else {
                    EP[-1 - y - bs_y] = hc->imgUV[uv][img_cy + bs_y + y][img_cx - 1];
                }
            }
        } else {
            for (y = 0; y < bs_y; y++) {
                EP[-1 - y - bs_y] = EP[-bs_y];
            }
        }

        {
            if (mb_available_up_left) {
                EP[0] = hc->imgUV[uv][img_cy - 1][img_cx - 1];
            } else if (mb_available_up) {
                EP[0] = hc->imgUV[uv][img_cy - 1][img_cx];
            } else if (mb_available_left) {
                EP[0] = hc->imgUV[uv][img_cy][img_cx - 1];
            }
        }

        for (i = 0; i < NUM_INTRA_PMODE_CHROMA; i++) {
            for (y = 0; y < bs_y; y++) {
                for (x = 0; x < bs_x; x++) {
                    piPredBuf[y][x] = 0;
                }
            }
            predIntraChromaAdi(EP, piPredBuf, i, uiBitSize, mb_available_up, mb_available_left, predLmode,
                               input->sample_bit_depth);
            for (y = 0; y < bs_y; y++) {
                for (x = 0; x < bs_x; x++) {
                    img->predBlockUV[uv][i][y][x] = piPredBuf[y][x];
                }
            }
        }
    }
    free_mem2Dint(piPredBuf);
    //free ( edgepixels );
}

//!EDIT START <added by lzhang AEC

int MBType2Value_NoCBP(codingUnit *currMB)
{
    if (img->type == B_IMG || img->type == F_IMG  || img->type == P_IMG) {

        if (currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
            return (img->type == INTRA_IMG ? 0 : 5);
        } else if (currMB->cuType == PNXN) {
            return 4;
        } else if (currMB->cuType == P2NXN || currMB->cuType == PHOR_UP || currMB->cuType == PHOR_DOWN) { //16x8, 16x4, 16x12
            return 2;
        } else if (currMB->cuType == PNX2N || currMB->cuType == PVER_LEFT ||
                   currMB->cuType == PVER_RIGHT) {  //8x16, 4x16, 12x16
            return 3;
        } else {
            return currMB->cuType;
        }
    } else {
        return 0;  //yuquanhe@hisilicon.com
    }
}


/*
*************************************************************************
* Function: Codes codingUnit header
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


int writeMBHeader(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int rdopt)
{
    int             i;
    SyntaxElement   *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    int            *bitCount  = currMB->bitcounter;
    int             no_bits   = 0;
    DataPartition  *dataPart;
    Slice          *currSlice = img->currentSlice ;

    CheckAvailabilityOfNeighbors(currMB, uiBitSize, uiPositionInPic);
    //=====  BITS FOR MACROBLOCK MODE =====
    if (img->type != INTRA_IMG) {   //qyu 0822 delete  img->skip_mode_flag == 1

        // Put out mb mode
        if (!(img->type == P_IMG && img->typeb == BP_IMG)) {
            currSE->value1  = MBType2Value_NoCBP(currMB);

            if (currMB->cuType == PSKIPDIRECT &&  currMB->cbp == 0) {    //rm52k
                currSE->value1 = 0;
            } else {
                currSE->value1++;
            }

            currSE->type    = SE_MBTYPE;

            dataPart = & (currSlice->partArr[0]);
            currSE->writing = writeCuTypeInfo;
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        } else {

            if (currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
                currSE->value1 = 2;
            } else if (currMB->cbp == 0) {   //rm52k
                currSE->value1 = 0;
            }

            else {
                currSE->value1 = 1;
            }
            currSE->type    = SE_MBTYPE;

            dataPart = & (currSlice->partArr[0]);
            currSE->writing = writeCuTypeInfo_SFRAME;
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        }

#if TRACE

        if (img->type == B_IMG) {
            if (he->AEC_writting) {
                fprintf(hc->p_trace,  "B_MB mode(%2d,%2d) = %3d\n", img->mb_x, img->mb_y, currMB->cuType);
            }
        } else {
            if (he->AEC_writting) {
                fprintf(hc->p_trace,   "MB mode(%2d,%2d) = %3d\n", img->mb_x, img->mb_y, currMB->cuType);
            }
        }

#endif

        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                += currSE->len;

        if (img->type == B_IMG && (currMB->cuType >= P2NX2N && currMB->cuType <= PVER_RIGHT)) {
            dataPart = & (currSlice->partArr[0]);
            currSE->writing = writePdir;
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE

            if (img->type == B_IMG) {
                if (he->AEC_writting && currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT) {
                    fprintf(hc->p_trace,  "B_Pred_Dir0 (%2d,%2d) = (%2d)\n", img->mb_x, img->mb_y, currMB->b8pdir[0]);
                    fprintf(hc->p_trace,  "B_Pred_Dir1 (%2d,%2d) = (%2d)\n", img->mb_x, img->mb_y, currMB->b8pdir[3]);
                } else if (he->AEC_writting && currMB->cuType == P2NX2N) {
                    fprintf(hc->p_trace,  "B_Pred_Dir (%2d,%2d) = (%2d)\n", img->mb_x, img->mb_y, currMB->b8pdir[0]);
                }
            }
            fflush(hc->p_trace);
#endif

            bitCount[BITS_MB_MODE] += currSE->len;
            no_bits                += currSE->len;
        } else if (img->type == F_IMG && input->dhp_enabled && (img->num_of_references > 1) &&
                   ((currMB->cuType >= P2NX2N && currMB->cuType <= PVER_RIGHT && uiBitSize >  B8X8_IN_BIT) ||
                    (currMB->cuType == P2NX2N                                 && uiBitSize == B8X8_IN_BIT))) {
            dataPart = & (currSlice->partArr[0]);
            currSE->writing = writePdir_dhp;
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE
            if (he->AEC_writting && currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT) {
                fprintf(hc->p_trace,  "P_Pred_Dir0 (%2d,%2d) = (%2d)\n", img->mb_x, img->mb_y, currMB->b8pdir[0]);
                fprintf(hc->p_trace,  "P_Pred_Dir1 (%2d,%2d) = (%2d)\n", img->mb_x, img->mb_y, currMB->b8pdir[3]);
            } else if (he->AEC_writting && currMB->cuType == P2NX2N) {
                fprintf(hc->p_trace,  "P_Pred_Dir (%2d,%2d) = (%2d)\n", img->mb_x, img->mb_y, currMB->b8pdir[0]);
            }
            fflush(hc->p_trace);
#endif

            bitCount[BITS_MB_MODE] += currSE->len;
            no_bits                += currSE->len;
        }

        if (IS_P_SKIP(currMB) && img->type == F_IMG && input->wsm_enabled && img->num_of_references > 1) {
            currSE->value1 = currMB->weighted_skipmode;
            currSE->type   = SE_WPM1;
            currSE->bitpattern = currSE->value1;
            currSE->len = 1;

            currSE->writing = writeWPM;
            dataPart = & (currSlice->partArr[0]);
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
#if TRACE
            if (he->AEC_writting) {
                fprintf(hc->p_trace, "weighted_skipmode1 = %3d \n", currSE->value1);
            }
#endif
            bitCount[BITS_INTER_MB] += currSE->len;
            no_bits                += currSE->len;

        }
    }

    if (img->type == F_IMG && input->b_mhpskip_enabled && IS_P_SKIP(currMB) && (currMB->weighted_skipmode == 0)) {
        dataPart = & (currSlice->partArr[0]);
        currSE->writing = write_p_skip_mode;
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                += currSE->len;
        currSE++;
        currMB->currSEnr++;
#if TRACE
        if (he->AEC_writting) {
            fprintf(hc->p_trace, "mh_p_skip_mode = %3d \n", currMB->md_directskip_mode);
        }
#endif

    }



    if (IS_DIRECT(currMB)) {
        dataPart = & (currSlice->partArr[0]);
        currSE->writing = write_b_dir_skip;
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                += currSE->len;
        currSE++;
        currMB->currSEnr++;
#if TRACE
        if (he->AEC_writting) {
            fprintf(hc->p_trace, "md_directskip_mode = %3d \n", currMB->md_directskip_mode);
        }
#endif
    }

    //===== BITS FOR 8x8 SUB-PARTITION MODES =====
    if (IS_P8x8(currMB)) {
        if (img->type == B_IMG) {
            for (i = 0; i < 4; i++) {
                //mb_part_type is fix length coding(fix length equal 2)!!   jlzheng 7.22

                currSE->value1  =  currSE->bitpattern = (currMB->b8mode[i] == PNXN) ? (1 + (currMB->b8pdir[i] == BID ? 6 :
                                                        currMB->b8pdir[i])) : 0;
                currSE->type    = SE_MBTYPE;
                currSE->len     = 2;
                currSE->value2  = 0;

                dataPart = & (currSlice->partArr[0]);
                currSE->writing = writeB8TypeInfo;
                dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE
                if (he->AEC_writting) {
                    fprintf(hc->p_trace, "8x8 mode/pdir(%2d) = %3d/%d\n", i, currMB->b8mode[i], currMB->b8pdir[i]);
                }
#endif

                bitCount[BITS_MB_MODE] += currSE->len;
                no_bits               += currSE->len;

            }
        } else if (img->type == F_IMG && input->dhp_enabled && img->num_of_references > 1) {
            for (i = 0; i < 4; i++) {
                //mb_part_type is fix length coding(fix length equal 2)!!   jlzheng 7.22

                currSE->value1  =  currSE->bitpattern = currMB->b8pdir[i];
                currSE->type    = SE_MBTYPE;
                currSE->len     = 2;
                currSE->value2  = 0;

                dataPart = & (currSlice->partArr[0]);
                currSE->writing = writeB8TypeInfo_dhp;
                dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE
                if (he->AEC_writting) {
                    fprintf(hc->p_trace, "8x8 mode/pdir(%2d) = %3d/%d\n", i, currMB->b8mode[i], currMB->b8pdir[i]);
                }
#endif

                bitCount[BITS_MB_MODE] += currSE->len;
                no_bits               += currSE->len;

            }
        }
    }

    //qyu  write TR size
    if (IS_INTRA(currMB)) {
        currSE->value1  = currMB->trans_size;
        currSE->type    = SE_MBTYPE;
        currSE->len     = 1;
        if (currMB->ui_MbBitSize == B64X64_IN_BIT) {
            currSE->len = 0;
        }
        currSE->value2  = 0;
        currSE->value2  = (currMB->cuType == I8MB ||  currMB->cuType == I16MB || currMB->cuType == InNxNMB ||
                           currMB->cuType == INxnNMB);
        currSE->writing = writeTrSize;
        dataPart = & (currSlice->partArr[0]);
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
#if TRACE
        if (he->AEC_writting) {
            fprintf(hc->p_trace, "Transform_Size = %d\n", currMB->trans_size);
        }
#endif
        bitCount[BITS_HEADER] += currSE->len;
        no_bits               += currSE->len;
    }
    //////////////////////////
    if (input->useSDIP && IS_INTRA(currMB)) {
        currSE->type    = SE_MBTYPE;
        currSE->value1 = 1;
        if (currMB->ui_MbBitSize == B64X64_IN_BIT || currMB->ui_MbBitSize == B8X8_IN_BIT) {
            currSE->len = 0;
        }
        dataPart = & (currSlice->partArr[0]);
        currSE->writing = writeCuTypeInfo_SDIP;
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                += currSE->len;
    }

    if (IS_INTRA(currMB)) {
        int num_of_intra_block = (currMB->cuType == I8MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) ? 4 : 1;
        for (i = 0; i < num_of_intra_block; i++) {
            currSE->context = i;//need modification???
            currSE->value1  = currMB->intra_pred_modes[i];

#if TRACE

            if (he->AEC_writting) {
                fprintf(hc->p_trace, "Intra mode     = %3d %d\n", currSE->value1, currSE->context);
            }

#endif

            currSE->type = SE_INTRAPREDMODE;

            dataPart = & (currSlice->partArr[0]);
            currSE->writing = writeIntraPredMode;
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

            bitCount[BITS_MB_MODE] += currSE->len;
            no_bits += currSE->len;
        }


        //!EDIT START <added by lzhang AEC
        currSE->value1 = currMB->c_ipred_mode;
        currSE->type = SE_INTRAPREDMODE;

        dataPart = & (currSlice->partArr[0]);
        currSE->writing = writeCIPredMode;
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                    += currSE->len;
#if TRACE

        if (he->AEC_writting) {
            fprintf(hc->p_trace, "Chroma intra pred mode %d\n", currMB->c_ipred_mode);
        }

#endif
    }

    return no_bits;
}

/*
*************************************************************************
* Function:Writes motion vectors of an 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int writeMotionVector8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int  i0, int  j0,
                         int  i1, int  j1, int  refframe, int  dmv_flag, int  fwd_flag, int  mv_mode)
{
    int            i, j, k, l, m;
    int            curr_mvd;
    int            bwflag      = ((refframe < 0 || (!fwd_flag)) ? 1 : 0);
    int            rate        = 0;

    int            step_h     = g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[mv_mode * 2][0] >>
                                MIN_BLOCK_SIZE_IN_BIT;
    int            step_v     = g_blk_size[mv_mode * 2][1] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[mv_mode * 2][1] >>
                                MIN_BLOCK_SIZE_IN_BIT ;

    //  codingUnit*    currMB      = &img->mb_data[img->current_mb_nr];
    SyntaxElement *currSE      = &img->MB_SyntaxElements[currMB->currSEnr];
    int           *bitCount    = currMB->bitcounter;
    int            refindex    = (refframe < 0 ? 0 : refframe);
    int ** ***       allFwMv      = (fwd_flag ? img->allFwMv : img->allBwMv);
    int ** ***       pred_mv     = ((img->type != B_IMG) ? img->mv : (fwd_flag ? img->predBFwMv : img->predBBwMv));
    Slice         *currSlice  = img->currentSlice;
    DataPartition *dataPart;
    int pmvr_mvd[2];

    if (!fwd_flag) {
        bwflag = 1;
    }


    for (j = j0; j < j1; j += step_v) {
        for (i = i0; i < i1; i += step_h) {
            if (input->b_pmvr_enabled) {
                pmvr_mvd_derivation(pmvr_mvd, allFwMv[j][i][refindex][mv_mode], pred_mv[j][i][refindex][mv_mode]);
            } else {
                pmvr_mvd[0] = allFwMv[j][i][refindex][mv_mode][0] - pred_mv[j][i][refindex][mv_mode][0];
                pmvr_mvd[1] = allFwMv[j][i][refindex][mv_mode][1] - pred_mv[j][i][refindex][mv_mode][1];
            }
            for (k = 0; k < 2; k++) {
                curr_mvd = pmvr_mvd[k];

                //--- store (oversampled) mvd ---
                for (l = 0; l < step_v; l++) {
                    for (m = 0; m < step_h; m++) {
                        currMB->mvd[bwflag][j + l][i + m][k] = curr_mvd;

                    }
                }

                currSE->value1 = curr_mvd;
                currSE->type   = ((img->type == B_IMG) ? SE_BFRAME : SE_MVD);
                currSE->value2  = 2 * k + bwflag; // identifies the component and the direction; only used for context determination
                currSE->writing = writeMVD_AEC;
                dataPart = & (currSlice->partArr[0]);
                dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE

                if (he->AEC_writting) {
                    if (fwd_flag) {
                        fprintf(hc->p_trace, "FMVD(%d) = %3d  (org_mv %3d pred_mv %3d) %d\n", k, curr_mvd, allFwMv[i][j][refindex][mv_mode][k],
                                pred_mv[j][i][refindex][mv_mode][k], currSE->value2);
                    } else {
                        fprintf(hc->p_trace, "BMVD(%d) = %3d  (org_mv %3d pred_mv %3d)\n", k, curr_mvd, allFwMv[i][j][refindex][mv_mode][k],
                                pred_mv[j][i][refindex][mv_mode][k]);
                    }
                }

#endif

                bitCount[BITS_INTER_MB] += currSE->len;
                rate                    += currSE->len;
            }
        }
    }

    return rate;
}

/*
*************************************************************************
* Function:Writes motion vectors of an 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int writeMotionVector8x8_sym(int  i0,  int  j0,  int  i1, int  j1, int  refframe, int  dmv_flag, int  fwd_flag,
                             int  mv_mode, int  pdir, codingUnit *currMB, int  uiPositionInPic)
{
    int            i, j, k, l, m;
    int            curr_mvd;
    int            bwflag     = ((refframe < 0 || (!fwd_flag)) ? 1 : 0);
    int            rate       = 0;

    int            step_h     = (g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                                (g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT) ;
    int            step_v     = (g_blk_size[mv_mode * 2][1] >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                                (g_blk_size[mv_mode * 2][1] >> MIN_BLOCK_SIZE_IN_BIT) ;


    SyntaxElement *currSE     = &img->MB_SyntaxElements[currMB->currSEnr];
    int           *bitCount   = currMB->bitcounter;
    int            refindex   = (refframe < 0 ? 0 : refframe);
    int ** ***       allFwMv     = (fwd_flag ? img->allFwMv : img->allBwMv);
    int ** ***       pred_mv    = ((img->type != B_IMG) ? img->mv : (fwd_flag ? img->predBFwMv : img->predBBwMv));
    Slice         *currSlice  = img->currentSlice;
    DataPartition *dataPart;

    if (!fwd_flag) {
        bwflag = 1;
    }

    if (pdir == SYM && mv_mode != 0) {
        //sw 10.1
        allFwMv = img->allSymMv;
        pred_mv = img->predSymMv  ;
    }

    if (pdir == BID && mv_mode != 0) {
        //sw 10.1
        if (fwd_flag) {
            allFwMv = img->allBidFwMv;
            pred_mv = img->predBidFwMv;
            //pred_mv = img->predSymMv  ;
        } else {
            allFwMv = img->allBidBwMv;
            pred_mv = img->predBidBwMv;
        }

    }

    for (j = j0; j < j1; j += step_v) {
        for (i = i0; i < i1; i += step_h) {
            for (k = 0; k < 2; k++) {

                //      curr_mvd = allFwMv[i][j][refindex][mv_mode][k] - pred_mv[i][j][refindex][mv_mode][k];
                curr_mvd = currMB->mvd[bwflag][j][i][k];

                //--- store (oversampled) mvd ---
                for (l = 0; l < step_v; l++) {
                    for (m = 0; m < step_h; m++) {
                        currMB->mvd[bwflag][j + l][i + m][k] = curr_mvd;

                    }
                }

                currSE->value1 = curr_mvd;
                currSE->type   = (img->type == B_IMG ? SE_BFRAME : SE_MVD);

                currSE->value2  = 2 * k + (1 -
                                           fwd_flag);  // identifies the component and the direction; only used for context determination

                currSE->writing = writeMVD_AEC;
                dataPart = & (currSlice->partArr[0]);
                dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE

                if (fwd_flag) {
                    if (he->AEC_writting) {

                        fprintf(hc->p_trace, "FMVD(%d) = %3d  (org_mv %3d pred_mv %3d) %d\n", k, curr_mvd, allFwMv[j][i][refindex][mv_mode][k],
                                pred_mv[j][i][refindex][mv_mode][k], currSE->value2);

                    }
                } else {
                    if (he->AEC_writting) {

                        fprintf(hc->p_trace, "BMVD(%d) = %3d  (org_mv %3d pred_mv %3d)\n", k, curr_mvd, allFwMv[j][i][refindex][mv_mode][k],
                                pred_mv[j][i][refindex][mv_mode][k]);

                    }
                }

#endif
                bitCount[BITS_INTER_MB] += currSE->len;
                rate                    += currSE->len;
            }
        }
    }

    return rate;
}

void copyMBInfo(codingUnit *srcMB, codingUnit *dstMB)
{
    int i, j;

    dstMB->trans_size   = srcMB->trans_size;
    dstMB->ui_MbBitSize = srcMB->ui_MbBitSize;
    dstMB->currSEnr     = srcMB->currSEnr;
    dstMB->slice_nr     = srcMB->slice_nr;
    dstMB->delta_qp     = srcMB->delta_qp;
    dstMB->qp           = srcMB->qp;
    dstMB->cuType      = srcMB->cuType;
    dstMB->cbp          = srcMB->cbp;
    dstMB->cbp_blk      = srcMB->cbp_blk;

#if MB_DQP
    dstMB->left_cu_qp          = srcMB->left_cu_qp;
    dstMB->previouse_qp      = srcMB->previouse_qp;
#endif

    dstMB->c_ipred_mode = srcMB->c_ipred_mode;


    dstMB->mbAddrA        = srcMB->mbAddrA;
    dstMB->mbAddrB        = srcMB->mbAddrB;
    dstMB->mbAddrC        = srcMB->mbAddrC;
    dstMB->mbAddrD        = srcMB->mbAddrD;

    dstMB->slice_set_index    = srcMB->slice_set_index;
    dstMB->slice_header_flag  = srcMB->slice_header_flag;
    dstMB->sliceqp            = srcMB->sliceqp;

    memcpy(dstMB->bitcounter,       srcMB->bitcounter,        sizeof(int) * MAX_BITCOUNTER_MB);
    memcpy(dstMB->mvd,              srcMB->mvd,               sizeof(int) * 2 * 2 * BLOCK_MULTIPLE * BLOCK_MULTIPLE);
    memcpy(dstMB->intra_pred_modes, srcMB->intra_pred_modes,  sizeof(int) * BLOCK_MULTIPLE * BLOCK_MULTIPLE);
    memcpy(dstMB->b8mode,           srcMB->b8mode,            sizeof(int) * 4);
    memcpy(dstMB->b8pdir,           srcMB->b8pdir,            sizeof(int) * 4);

    //mb_available
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            dstMB->mb_available[i][j] = srcMB->mb_available[i][j];
        }
    }
    dstMB->mb_available_left  = srcMB->mb_available_left;
    dstMB->mb_available_up    = srcMB->mb_available_up;
}

int estLumaCoeff8x8(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, int intra_mode,
                    int uiPositionInPic)
{
    int             rate      = 0;
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];

    int level, run;
    int   k;
    int curr_val;
    int ipos = 0, xx, icoef, yy;
    int **AVS_SCAN;


    for (xx = 0; xx <= ((1 << uiBitSize) * (1 << uiBitSize)); xx++) {         //qyu 0821
        he->ACRun_RDOQ[xx] = he->ACLevel_RDOQ[xx] = 0;
    }

    run  = -1;
    if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->trans_size == 1 && IS_INTER(currMB) &&
            (currMB->cuType == P2NXN || currMB->cuType == PHOR_UP || currMB->cuType == PHOR_DOWN))) {
        if (uiBitSize == 3) {
            AVS_SCAN = AVS_SCAN4x16;
        } else if (uiBitSize == 4) {
            AVS_SCAN = AVS_SCAN8x32;
        } else if (uiBitSize == 5) {
            AVS_SCAN = AVS_SCAN8x32;
            uiBitSize--;
        }
    } else if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->trans_size == 1 && IS_INTER(currMB) &&
               (currMB->cuType == PNX2N || currMB->cuType == PVER_LEFT || currMB->cuType == PVER_RIGHT))) {
        if (uiBitSize == 3) {
            AVS_SCAN = AVS_SCAN16x4;
        } else if (uiBitSize == 4) {
            AVS_SCAN = AVS_SCAN32x8;
        } else if (uiBitSize == 5) {
            AVS_SCAN = AVS_SCAN32x8;
            uiBitSize--;
        }
    } else {
        if (uiBitSize == B4X4_IN_BIT) {
            AVS_SCAN = AVS_SCAN4x4;
        } else if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_SCAN8x8;

        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_SCAN16x16;

        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_SCAN32x32;
        }
    }
    for (icoef = 0; icoef < ((1 << uiBitSize) * (1 << uiBitSize)); icoef++) {
        run++;
        xx = AVS_SCAN[icoef][0];
        yy = AVS_SCAN[icoef][1];

        curr_val = Quant_Coeff[yy][xx];

        if (curr_val != 0) {
            he->ACLevel_RDOQ[ipos] = curr_val;
            he->ACRun_RDOQ[ipos] = run;
            run = -1;
            ipos++;
        }
    }
    //added 1.6
    level = 1; // get inside loop
    for (k = 0; k <= ((1 << uiBitSize) * (1 << uiBitSize)) && level != 0; k++) {
        he->alllevel[k] = he->ACLevel_RDOQ[k];
        he->allrun[k] = he->ACRun_RDOQ[k];
    }
    rate += estRunLevelRef(currMB, LUMA_8x8);



    return rate;
}

int estChromaCoeff8x8(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, int b8 , int uiPositionInPic)
{
    int             rate      = 0;
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    int            *bitCount  = currMB->bitcounter;
    Slice          *currSlice = img->currentSlice;
    int             cbp       = currMB->cbp;
    DataPartition  *dataPart;
    int             level, run;//, ipos=-1;
    int             k;
    int             curr_val;
    int             ipos = 0, xx, icoef, yy;
    //  int block_x = (b8%2)<<3;
    //  int block_y = (b8/2)<<3;
    int **AVS_SCAN;

    for (xx = 0; xx < ((1 << uiBitSize) * (1 << uiBitSize) + 1); xx++) {
        he->ACLevel_RDOQ[xx] = he->ACRun_RDOQ[xx] = 0;
    }

    run  = -1;
    if (uiBitSize == B4X4_IN_BIT) {
        AVS_SCAN = AVS_SCAN4x4;

    } else if (uiBitSize == B8X8_IN_BIT) {
        AVS_SCAN = AVS_SCAN8x8;

    } else if (uiBitSize == B16X16_IN_BIT) {
        AVS_SCAN = AVS_SCAN16x16;

    } else if (uiBitSize == B32X32_IN_BIT) {
        AVS_SCAN = AVS_SCAN32x32;

    }
    for (icoef = 0; icoef < ((1 << uiBitSize) * (1 << uiBitSize)); icoef++) {
        run++;
        xx = AVS_SCAN[icoef][0];
        yy = AVS_SCAN[icoef][1];

        curr_val = Quant_Coeff[yy][xx];

        if (curr_val != 0) {
            he->ACLevel_RDOQ[ipos] = curr_val;
            he->ACRun_RDOQ[ipos] = run;
            run = -1;
            ipos++;
        }
    }
    //added by lzhang for AEC
    if ((cbp >> b8) & 0x01) {
        level = 1; // get inside loop
        for (k = 0; k < ((1 << uiBitSize) * (1 << uiBitSize) + 1) && level != 0; k++) {
            level = he->alllevel[k] = he->ACLevel_RDOQ[k];
            run = he->allrun[k] = he->ACRun_RDOQ[k];
#if TRACE
            if (he->AEC_writting) {
                fprintf(hc->p_trace, "AC Chroma8x8 %2d: level =%3d run =%2d\n", k, level, run);
            }
#endif
        }
        currSE->writing = writeRunLevelRef;//qyu 0825 ???
        currSE->context     =  CHROMA;
        currSE->type        = (IS_INTRA(currMB) ? SE_CHR_AC_INTRA : SE_CHR_AC_INTER);
        // choose the appropriate data partition
        dataPart = &(currSlice->partArr[0]);
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        bitCount[BITS_COEFF_UV_MB] += currSE->len;
        rate                       += currSE->len;

    }
    return rate;
}

int estCGLastRate(int iCG, BiContextTypePtr pCTX, int bitSize, int *CGLastX, int *CGLastY, int isChroma, int ctxmode,
                  codingUnit *currMB, int intraPredMode)
{
    int rate = 0;
    int count = 0;
    int numCGminus1 = (1 << (bitSize + 1)) - 1;
    int offset;
    int numCGminus1X;
    int numCGminus1Y;
    if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
            (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                          (currMB->cuType == PHOR_DOWN)))) {
        numCGminus1X = (bitSize == 2) ? (1 << (bitSize + 1)) - 1 : (1 << (bitSize + 2)) - 1;
        numCGminus1Y = (bitSize == 2) ? (1 << (bitSize - 1)) - 1 : (1 << (bitSize)) - 1;
    } else if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
               (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                       (currMB->cuType == PVER_RIGHT)))) {
        numCGminus1X = (bitSize == 2) ? (1 << (bitSize - 1)) - 1 : (1 << (bitSize)) - 1;
        numCGminus1Y = (bitSize == 2) ? (1 << (bitSize + 1)) - 1 : (1 << (bitSize + 2)) - 1;
    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                  (currMB->cuType == InNxNMB))) {
        numCGminus1X = (bitSize == 2) ? (1 << (bitSize + 1)) - 1 : (1 << (bitSize + 2)) - 1;
        numCGminus1Y = (bitSize == 2) ? (1 << (bitSize - 1)) - 1 : (1 << (bitSize)) - 1;
    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                  (currMB->cuType == INxnNMB))) {
        numCGminus1X = (bitSize == 2) ? (1 << (bitSize - 1)) - 1 : (1 << (bitSize)) - 1;
        numCGminus1Y = (bitSize == 2) ? (1 << (bitSize + 1)) - 1 : (1 << (bitSize + 2)) - 1;
    } else {
        numCGminus1X = numCGminus1Y = numCGminus1;
    }

    if (bitSize == -1) { //4x4
        rate = 0;
    } else

        if (bitSize == 0) { //8x8
            *CGLastX = iCG & 1;
            *CGLastY = iCG / 2;

            count = 0;
            while (count < iCG) {
                rate += biari_encode_symbol_est(0, pCTX + count);
                count++;
            }
            if (iCG < 3) {
                rate += biari_encode_symbol_est(1, pCTX + iCG);
            }
        } else {
            if (bitSize == 1) { //16x16
                if ((input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && IS_INTER(currMB) && !(isChroma) &&
                     (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                   (currMB->cuType == PHOR_DOWN)))) {
                    *CGLastX = AVS_SCAN2x8[iCG][0];
                    *CGLastY = AVS_SCAN2x8[iCG][1];
                } else if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                           (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                   (currMB->cuType == PVER_RIGHT)))) {
                    *CGLastX = AVS_SCAN8x2[iCG][0];
                    *CGLastY = AVS_SCAN8x2[iCG][1];
                } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                              (currMB->cuType == InNxNMB))) {
                    *CGLastX = AVS_SCAN2x8[iCG][0];
                    *CGLastY = AVS_SCAN2x8[iCG][1];
                } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                              (currMB->cuType == INxnNMB))) {
                    *CGLastX = AVS_SCAN8x2[iCG][0];
                    *CGLastY = AVS_SCAN8x2[iCG][1];
                }

                else {
                    *CGLastX = AVS_SCAN4x4[iCG][0];
                    *CGLastY = AVS_SCAN4x4[iCG][1];
                }
            } else if (bitSize == 2) { //32x32
                if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                        (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                      (currMB->cuType == PHOR_DOWN)))) {
                    *CGLastX = AVS_SCAN2x8[iCG][0];
                    *CGLastY = AVS_SCAN2x8[iCG][1];
                } else if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                           (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                   (currMB->cuType == PVER_RIGHT)))) {
                    *CGLastX = AVS_SCAN8x2[iCG][0];
                    *CGLastY = AVS_SCAN8x2[iCG][1];
                } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                              (currMB->cuType == InNxNMB))) {
                    *CGLastX = AVS_SCAN2x8[iCG][0];
                    *CGLastY = AVS_SCAN2x8[iCG][1];
                } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                              (currMB->cuType == INxnNMB))) {
                    *CGLastX = AVS_SCAN8x2[iCG][0];
                    *CGLastY = AVS_SCAN8x2[iCG][1];
                } else {
                    *CGLastX = AVS_SCAN8x8[iCG][0];
                    *CGLastY = AVS_SCAN8x8[iCG][1];
                }
            }
            if (!isChroma && intraPredMode == INTRA_PRED_DC_DIAG) {
                SWAP(*CGLastX, *CGLastY);
            }
            offset = isChroma ? 3 : 9;
            if (*CGLastX == 0 && *CGLastY == 0) {
                rate += biari_encode_symbol_est(0, pCTX + offset);
            } else {
                rate += biari_encode_symbol_est(1, pCTX + offset);

                count = 0;
                offset = isChroma ? 4 : 10;
                while (count < *CGLastX) {
                    rate += biari_encode_symbol_est(0, pCTX + offset);
                    count++;
                }
                if (*CGLastX < numCGminus1X) {
                    rate += biari_encode_symbol_est(1, pCTX + offset);
                }
                offset = isChroma ? 5 : 11;
                if (*CGLastX == 0) {
                    count = 0;
                    while (count < *CGLastY - 1) {
                        rate += biari_encode_symbol_est(0, pCTX + offset);
                        count++;
                    }
                    if (*CGLastY < numCGminus1Y) {
                        rate += biari_encode_symbol_est(1, pCTX + offset);
                    }
                } else {
                    count = 0;
                    while (count < *CGLastY) {
                        rate += biari_encode_symbol_est(0, pCTX + offset);
                        count++;
                    }
                    if (*CGLastY < numCGminus1Y) {
                        rate += biari_encode_symbol_est(1, pCTX + offset);
                    }
                }

            }
            if (!isChroma && intraPredMode == INTRA_PRED_DC_DIAG) {
                SWAP(*CGLastX, *CGLastY);
            }
        }

    return rate;
}

int estSigCGFlagRate(int sigCGFlag, BiContextTypePtr pCTX, int ctx)
{
    int rate = biari_encode_symbol_est(sigCGFlag, pCTX + ctx);

    return rate;
}

int estLastPosRate(int lastPosX, int lastPosY, BiContextTypePtr pCTX, int isLastCG, int CGLastX, int CGLastY ,
                   int iCG, int ctxmode, unsigned int uiBitSize, int isChroma)
{

    int rate = 0;
    int symbol;
    int ctx;
    int xx = lastPosX;
    int yy = lastPosY;
    int offset;
    int CGx = CGLastX;
    int CGy = CGLastY;

    if (!isLastCG) {
        if (ctxmode != 1) {
            xx = 3 - xx;
        }
        if (ctxmode != 0) {
            yy = 3 - yy;
        }
    }

    if ((CGx == 0 && CGy > 0 && ctxmode == 2) /*|| (ctxmode == 1)*/) {
        SWAP(xx, yy);
    }

    if (!isChroma) {
        offset = (uiBitSize == B4X4_IN_BIT) ? (ctxmode / 2) * 4 : ((CGx > 0 &&
                 CGy > 0) ? 0 : ((ctxmode / 2) * 4 + ((iCG == 0) ? 4 : 12)) + 8);
    } else {
        offset = (uiBitSize == B4X4_IN_BIT) ? 0 : 4;
    }
    if (!isLastCG) {
        offset += (isChroma ? NUM_LAST_POS_CTX_CHROMA / 2 : NUM_LAST_POS_CTX_LUMA / 2);
    }

    symbol = xx;
    ctx = 0;
    while (symbol >= 1) {
        symbol -= 1;
        rate += biari_encode_symbol_est(0, pCTX + offset + ctx);

        ctx ++;
        if (ctx >= 2) {
            ctx = 2;
        }
        if (ctx >= 1) {
            ctx = 1;
        }
    }
    if (xx != 3) {
        rate += biari_encode_symbol_est(1, pCTX + offset + ctx);
    }

    symbol = yy;
    ctx = 0;
    while (symbol >= 1) {
        symbol -= 1;
        rate += biari_encode_symbol_est(0, pCTX + offset + 2 + ctx);
        ctx ++;
        if (ctx >= 2) {
            ctx = 2;
        }
        if (ctx >= 1) {
            ctx = 1;
        }
    }
    if (yy != 3) {
        rate += biari_encode_symbol_est(1, pCTX + offset + 2 + ctx);
    }

    return rate;
}


int estLastCGLastPosRate(int lastPosX, int lastPosY, BiContextTypePtr pCTX, int offset, int CGLastX, int CGLastY)
{
    int rate = 0;
    int symbol;
    int ctx;
    int xx = lastPosX;
    int yy = lastPosY;

    symbol = xx;
    ctx = 0;
    while (symbol >= 1) {
        symbol -= 1;
        rate += biari_encode_symbol_est(0, pCTX + offset + ctx);

        ctx ++;
        if (ctx >= 2) {
            ctx = 2;
        }
    }
    if (xx != 3) {
        rate += biari_encode_symbol_est(1, pCTX + offset + ctx);
    }

    symbol = yy;
    ctx = 0;
    while (symbol >= 1) {
        symbol -= 1;
        rate += biari_encode_symbol_est(0, pCTX + offset + 3 + ctx);

        ctx ++;
        if (ctx >= 2) {
            ctx = 2;
        }
    }
    if (yy != 3) {
        rate += biari_encode_symbol_est(1, pCTX + offset + 3 + ctx);
    }

    return rate;
}

int estLastRunRate(int lastRun, BiContextTypePtr pCTX, int rank, int iCG, int isChroma, int ctxmode)
{
    int rate = 0;
    int symbol = lastRun;
    int ctxpos = 0;
    int moddiv;
    int px, py;
    int offset = ((isChroma ? 0 : (min(rank - 1, 1) << 1)) + (iCG == 0)) << 1;

    while (symbol >= 1) {
        symbol -= 1;

        px = AVS_SCAN4x4[15 - ctxpos][0];
        py = AVS_SCAN4x4[15 - ctxpos][1];
        moddiv = (ctxmode == INTRA_PRED_VER) ? (py >> 1) : ((ctxmode == INTRA_PRED_HOR) ? (px >> 1) : (ctxpos <= 9));
        ctxpos++;
        rate += biari_encode_symbol_est(0, pCTX + offset + moddiv);
    }

    if (((iCG == 0) && lastRun < 16) || ((iCG != 0) && lastRun < 15)) {
        px = AVS_SCAN4x4[15 - ctxpos][0];
        py = AVS_SCAN4x4[15 - ctxpos][1];
        moddiv = (ctxmode == INTRA_PRED_VER) ? (py >> 1) : ((ctxmode == INTRA_PRED_HOR) ? (px >> 1) : (ctxpos <= 9));
        rate += biari_encode_symbol_est(1, pCTX + offset + moddiv);
    }

    return rate;
}

int estLevelRate(int absLevel, BiContextTypePtr pCTX, int rank, int pairsInCG, int iCG, int pos, int isChroma,
                 int bitSize)
{
    int rate = 0;
    int symbol = absLevel - 1;
    int exp_golomb_order = 0;
    int bins = 0;

    int indiv = min(2, (pairsInCG + 1) / 2);
    int offset = ((iCG == 0 && pos > 12) ? 0 : 3) + indiv + 8;
    if (!isChroma) {
        offset += 3;
    }
    if (symbol > 31) {
        exp_golomb_order = 0;
        rate += biari_encode_symbol_final_est(1);
        symbol = symbol - 32;
        while (1) {
            if ((unsigned int)symbol >= (unsigned int)(1 << exp_golomb_order)) {
                rate += biari_encode_symbol_eq_prob_est(0);
                symbol = symbol - (1 << exp_golomb_order);
                exp_golomb_order++;
            } else {
                rate += biari_encode_symbol_eq_prob_est(1);

                while (exp_golomb_order--) {   //next binary part
                    rate += biari_encode_symbol_eq_prob_est((unsigned char)((symbol >> exp_golomb_order) & 1));
                }

                break;
            }
        }
    } else {
        rate += biari_encode_symbol_final_est(0);
        bins = 0;
        while (symbol >= 1) {
            symbol -= 1;
            rate += biari_encode_symbol_est(0, pCTX + offset);
            bins++;
        }
        if (bins < 31) {
            rate += biari_encode_symbol_est(1, pCTX + offset);
        }


    }

    return rate;
}

int estSignRate(int level)
{
    int rate = 0;

    if (level < 0) {
        rate += biari_encode_symbol_eq_prob_est(1);
    } else {
        rate += biari_encode_symbol_eq_prob_est(0);
    }

    return rate;
}

int estRunRate(int run, BiContextTypePtr pCTX, int pos, int iCG, int remainingPos, int isChroma, int ctxmode,
               int bitSize)
{
    int rate = 0;
    int symbol = run;
    int ctxpos = 0;
    int moddiv;
    int px, py;
    int offset;

    if (15 - pos > 0) {
        px = AVS_SCAN4x4[15 - pos - 1 - ctxpos][0];
        py = AVS_SCAN4x4[15 - pos - 1 - ctxpos][1];
        moddiv = (ctxmode == INTRA_PRED_VER) ? (py >> 1) : ((ctxmode == INTRA_PRED_HOR) ? (px >> 1) : (pos + ctxpos <= 9));
        offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv)) : (isChroma ? 2 : 3 + moddiv)) +
                 (bitSize == B4X4_IN_BIT ? 0 : 3);
        if (!isChroma) {
            moddiv = (ctxmode == INTRA_PRED_VER) ? ((py + 1) / 2) : ((ctxmode == INTRA_PRED_HOR) ? (((px + 1) / 2) + 3) : ((
                         pos + ctxpos) > 11 ? 6 : ((pos + ctxpos) > 4 ? 7 : 8)));
            offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv % 3)) : (4 + moddiv % 3)) +
                     (bitSize == B4X4_IN_BIT ? 0 : 4);
        }
    }

    while (symbol >= 1) {
        symbol -= 1;

        rate += biari_encode_symbol_est(0, pCTX + offset);

        ctxpos ++;
        if ((15 - pos - 1 - ctxpos) >= 0) {
            px = AVS_SCAN4x4[15 - pos - 1 - ctxpos][0];
            py = AVS_SCAN4x4[15 - pos - 1 - ctxpos][1];
            moddiv = (ctxmode == INTRA_PRED_VER) ? (py >> 1) : ((ctxmode == INTRA_PRED_HOR) ? (px >> 1) : (pos + ctxpos <= 9));
            offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv)) : (isChroma ? 2 : 3 + moddiv)) +
                     (bitSize == B4X4_IN_BIT ? 0 : 3);
            if (!isChroma) {
                moddiv = (ctxmode == INTRA_PRED_VER) ? ((py + 1) / 2) : ((ctxmode == INTRA_PRED_HOR) ? (((px + 1) / 2) + 3) : ((
                             pos + ctxpos) > 11 ? 6 : ((pos + ctxpos) > 4 ? 7 : 8)));
                offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv % 3)) : (4 + moddiv % 3)) +
                         (bitSize == B4X4_IN_BIT ? 0 : 4);
            }
        }
    }

    if (run < remainingPos) {
        rate += biari_encode_symbol_est(1, pCTX + offset);
    }

    return rate;
}
int writeLumaCoeff8x8_AEC(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                          unsigned int uiPixInSMB_y, int intra_mode, int uiPositionInPic)
{
    //qyu 0824 Quantized_Coeff=img->Coeff_all or img->Coeff_all_to_write
    int             rate      = 0;
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    Slice          *currSlice = img->currentSlice;
    int            *bitCount  = currMB->bitcounter;
    DataPartition  *dataPart;

#if MB_DQP
    int qp = currMB->qp;
    int shift = IQ_SHIFT[qp];
    int QPI   = IQ_TAB[qp];
#endif

    int level, run;
    int   k;
    int  ACLevel[MAX_CU_SIZE * MAX_CU_SIZE + 1]  ; //qyu 0821
    int  ACRun[MAX_CU_SIZE * MAX_CU_SIZE + 1]    ; //qyu 0821

    int curr_val;
    int ipos = 0, xx, icoef, yy;
    int **AVS_SCAN;
    int iCG;
    extern int DCT_CGFlag[CG_SIZE * CG_SIZE];
    extern int DCT_PairsInCG[CG_SIZE * CG_SIZE];
    extern int DCT_CGLastRun[CG_SIZE * CG_SIZE];

    extern int g_intraModeClassified[NUM_INTRA_PMODE];

    int iVer = 0, iHor = 0;
    if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && ((currMB->trans_size == 1) && (currMB->cuType == P2NXN ||
            currMB->cuType == PHOR_UP || currMB->cuType == PHOR_DOWN))) {
        iHor = 1;
    } else if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && ((currMB->trans_size == 1) &&
               (currMB->cuType == PNX2N || currMB->cuType == PVER_LEFT || currMB->cuType == PVER_RIGHT))) {
        iVer = 1;
    } else if (input->useSDIP && (currMB->trans_size == 1) && (currMB->cuType == InNxNMB)) {
        iHor = 1;
    } else if (input->useSDIP && (currMB->trans_size == 1) && (currMB->cuType == INxnNMB)) {
        iVer = 1;
    }

    if (currMB->trans_size == 0 || currMB->cuType == I16MB) {
        if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN8x8;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN16x16;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN32x32;
        } else if (uiBitSize == B64X64_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN32x32;
            uiBitSize -= 1;
        }
    } else {
        if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->cuType < 9 && (iVer || iHor))) {   //inter MB
            if (iHor) {
                if (uiBitSize == B8X8_IN_BIT) {
                    AVS_SCAN = AVS_SCAN4x16;
                } else if (uiBitSize == B16X16_IN_BIT) {
                    AVS_SCAN = AVS_SCAN8x32;
                } else if (uiBitSize == B32X32_IN_BIT) {
                    AVS_SCAN = AVS_SCAN8x32;
                    uiBitSize -= 1;
                }
            }
            if (iVer) {
                if (uiBitSize == B8X8_IN_BIT) {
                    AVS_SCAN = AVS_SCAN16x4;
                } else if (uiBitSize == B16X16_IN_BIT) {
                    AVS_SCAN = AVS_SCAN32x8;
                } else if (uiBitSize == B32X32_IN_BIT) {
                    AVS_SCAN = AVS_SCAN32x8;
                    uiBitSize -= 1;
                }
            }
        } else if (input->useSDIP && ((currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) && (iVer ||
                                      iHor))) {  //inter MB
            if (iHor) {
                if (uiBitSize == B8X8_IN_BIT) {
                    AVS_SCAN = AVS_SCAN4x16;
                } else if (uiBitSize == B16X16_IN_BIT) {
                    AVS_SCAN = AVS_SCAN8x32;
                } else if (uiBitSize == B32X32_IN_BIT) {
                    AVS_SCAN = AVS_SCAN8x32;
                    uiBitSize -= 1;
                }
            }
            if (iVer) {
                if (uiBitSize == B8X8_IN_BIT) {
                    AVS_SCAN = AVS_SCAN16x4;
                } else if (uiBitSize == B16X16_IN_BIT) {
                    AVS_SCAN = AVS_SCAN32x8;
                } else if (uiBitSize == B32X32_IN_BIT) {
                    AVS_SCAN = AVS_SCAN32x8;
                    uiBitSize -= 1;
                }
            }
        } else {
            if (uiBitSize == B4X4_IN_BIT) {
                AVS_SCAN = AVS_SCAN4x4;
            } else if (uiBitSize == B8X8_IN_BIT) {
                AVS_SCAN = AVS_CG_SCAN8x8;
            } else if (uiBitSize == B16X16_IN_BIT) {
                AVS_SCAN = AVS_CG_SCAN16x16;
            } else if (uiBitSize == B32X32_IN_BIT) {
                AVS_SCAN = AVS_CG_SCAN32x32;
            }
        }
    }

    //ACLevel = ( int * ) malloc ( ( ( 1 << uiBitSize ) * ( 1 << uiBitSize ) + 1 ) * sizeof ( int ) );
    //ACRun = ( int * ) malloc ( ( ( 1 << uiBitSize ) * ( 1 << uiBitSize ) + 1 ) * sizeof ( int ) );
    for (xx = 0; xx <= ((1 << uiBitSize) * (1 << uiBitSize)); xx++) {         //qyu 0821
        ACRun[xx] = ACLevel[xx] = 0;
    }

    for (iCG = 0; iCG < CG_SIZE * CG_SIZE; iCG++) {
        DCT_CGFlag[ iCG ] = 0;
        DCT_PairsInCG[ iCG ] = 0;
    }

    run  = -1;
    for (icoef = 0; icoef < ((1 << uiBitSize) * (1 << uiBitSize)); icoef++) {
        iCG = icoef >> 4;
        if (icoef % 16 == 0) {
            DCT_CGFlag[ iCG ] = 0;
            DCT_PairsInCG[ iCG ] = 0;
            run = -1;
        }
        run++;
        xx = AVS_SCAN[icoef][0] + uiPixInSMB_x;
        yy = AVS_SCAN[icoef][1] + uiPixInSMB_y;

        if (intra_mode && g_intraModeClassified[currMB->l_ipred_mode] == INTRA_PRED_HOR && currMB->cuType != InNxNMB &&
            currMB->cuType != INxnNMB) {
            xx = AVS_SCAN[icoef][1] + uiPixInSMB_x;
            yy = AVS_SCAN[icoef][0] + uiPixInSMB_y;
        }
        curr_val = Quant_Coeff[yy][xx];

        if (icoef % 16 == 15) {
            if (curr_val == 0) {
                DCT_CGLastRun[ iCG ] = run + 1;
            } else {
                DCT_CGLastRun[ iCG ] = 0;
            }
        }
        if (curr_val != 0) {
            ACLevel[ipos] = curr_val;
            ACRun[ipos]   = run;
            run = -1;
            ipos++;
            DCT_CGFlag[ iCG ] = 1;
            DCT_PairsInCG[ iCG ] ++;
        }
    }

    //added 1.6
    level = 1; // get inside loop


    for (k = 0; k <= ((1 << uiBitSize) * (1 << uiBitSize)) && level != 0; k++) {
        level = he->alllevel[k] = ACLevel[k]; // level
        run = he->allrun[k] = ACRun[k]; // run
#if TRACE
        if (he->AEC_writting) {
            fprintf(hc->p_trace, "Luma8x8 sng(%2d) level =%3d run =%2d\n", k, level, run);
        }

#endif
    }
    currSE->writing = writeRunLevelRef;
    currSE->context     = LUMA_8x8;
    currSE->type        = ((intra_mode ? SE_LUM_AC_INTRA : SE_LUM_AC_INTER));
    dataPart = & (currSlice->partArr[0]);
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
    bitCount[BITS_COEFF_Y_MB] += currSE->len;
    rate                      += currSE->len;

    //free ( ACLevel );
    //free ( ACRun );
    return rate;
}

int writeChromaCoeff8x8_AEC(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                            unsigned int uiPixInSMB_y, int b8 , int uiPositionInPic)
{
    //qyu 0824 Quantized_Coeff=img->Coeff_all or img->Coeff_all_to_write
    int             rate      = 0;
    //  codingUnit*     currMB    = &img->mb_data[img->current_mb_nr];
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    int            *bitCount  = currMB->bitcounter;
    Slice          *currSlice = img->currentSlice;
    int             cbp       = currMB->cbp;
    DataPartition  *dataPart;
    int       level, run;//, ipos=-1;
    int       k;
    //int  *ACLevel = ( int * ) malloc ( ( ( 1 << uiBitSize ) * ( 1 << uiBitSize ) + 1 ) * sizeof ( int ) ); //qyu 0821
    //int  *ACRun   = ( int * ) malloc ( ( ( 1 << uiBitSize ) * ( 1 << uiBitSize ) + 1 ) * sizeof ( int ) ); //qyu 0821
    int ACLevel[MAX_CU_SIZE * MAX_CU_SIZE + 1];
    int ACRun[MAX_CU_SIZE * MAX_CU_SIZE + 1];

    int curr_val;
    int ipos = 0, xx, icoef, yy;

    int **AVS_SCAN;
    int iCG;
    extern int DCT_CGFlag[CG_SIZE * CG_SIZE];
    extern int DCT_PairsInCG[CG_SIZE * CG_SIZE];
    extern int DCT_CGLastRun[CG_SIZE * CG_SIZE];

    for (xx = 0; xx < ((1 << uiBitSize) * (1 << uiBitSize) + 1); xx++) {
        ACRun[xx] = ACLevel[xx] = 0;
    }
    for (iCG = 0; iCG < CG_SIZE * CG_SIZE; iCG++) {
        DCT_CGFlag[ iCG ] = 0;
        DCT_PairsInCG[ iCG ] = 0;
    }

    run  = -1;
    if (uiBitSize == B4X4_IN_BIT) {
        AVS_SCAN = AVS_SCAN4x4;
    }

    if (uiBitSize == B8X8_IN_BIT) {
        AVS_SCAN = AVS_CG_SCAN8x8;
    } else if (uiBitSize == B16X16_IN_BIT) {
        AVS_SCAN = AVS_CG_SCAN16x16;
    } else if (uiBitSize == B32X32_IN_BIT) {
        AVS_SCAN = AVS_CG_SCAN32x32;
    }

    for (icoef = 0; icoef < ((1 << uiBitSize) * (1 << uiBitSize)); icoef++) {
        iCG = icoef >> 4;
        if (icoef % 16 == 0) {
            DCT_CGFlag[ iCG ] = 0;
            DCT_PairsInCG[ iCG ] = 0;
            run = -1;
        }
        run++;
        xx = AVS_SCAN[icoef][0] + uiPixInSMB_x;
        yy = AVS_SCAN[icoef][1] + uiPixInSMB_y;

        curr_val = Quant_Coeff[yy][xx];
        if (icoef % 16 == 15) {
            if (curr_val == 0) {
                DCT_CGLastRun[ iCG ] = run + 1;
            } else {
                DCT_CGLastRun[ iCG ] = 0;
            }
        }
        if (curr_val != 0) {
            ACLevel[ipos] = curr_val;
            ACRun[ipos]   = run;
            run = -1;
            ipos++;
            DCT_CGFlag[ iCG ] = 1;
            DCT_PairsInCG[ iCG ] ++;
        }
    }

    //added by lzhang for AEC
    if ((cbp >> b8) & 0x01) {
        level = 1; // get inside loop

        for (k = 0; k < ((1 << uiBitSize) * (1 << uiBitSize) + 1) && level != 0; k++) {
            level = he->alllevel[k] = ACLevel[k]; // level
            run = he->allrun[k] = ACRun[k]; // run
#if TRACE

            if (he->AEC_writting) {
                fprintf(hc->p_trace, "AC Chroma8x8 %2d: level =%3d run =%2d\n", k, level, run);
            }

#endif
        }
        currSE->writing = writeRunLevelRef;//qyu 0825 ???
        currSE->context     =  CHROMA;
        currSE->type        = (IS_INTRA(currMB) ? SE_CHR_AC_INTRA : SE_CHR_AC_INTER);
        // choose the appropriate data partition
        dataPart = & (currSlice->partArr[0]);
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        bitCount[BITS_COEFF_UV_MB] += currSE->len;
        rate                       += currSE->len;

    }

    //free ( ACLevel );
    //free ( ACRun );
    return rate;
}

/*
*************************************************************************
* Function:Writes Luma Coeff of an 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int writeLumaCoeff8x8(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                      unsigned int uiPixInSMB_y, int block8x8, int intra8x8mode, int uiPositionInPic)
{
    int  rate = 0;

    rate  += writeLumaCoeff8x8_AEC(Quant_Coeff, currMB, uiBitSize, uiPixInSMB_x, uiPixInSMB_y, intra8x8mode,
                                   uiPositionInPic);

    return rate;
}
int writeChromaCoeff8x8(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                        unsigned int uiPixInSMB_y, int block8x8, int intramode, int uiPositionInPic)
{
    int  rate = 0;

    rate += writeChromaCoeff8x8_AEC(Quant_Coeff, currMB, uiBitSize, uiPixInSMB_x, uiPixInSMB_y, block8x8,
                                    uiPositionInPic);

    return rate;
}


/*
*************************************************************************
* Function:Writes CBP, DQUANT, and Luma Coefficients of an codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int StoreMotionVector8x8(codingUnit *currMB, int i0, int j0, int i1, int j1, int refframe, int dmv_flag, int fwd_flag,
                         int mv_mode, int dmh_mode)
{
    int            i, j, k, l, m;
    int            curr_mvd;
    int            bwflag     = ((refframe < 0 || (!fwd_flag)) ? 1 : 0);

    int            step_h     = g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[mv_mode * 2][0] >>
                                MIN_BLOCK_SIZE_IN_BIT;
    int            step_v     = g_blk_size[mv_mode * 2][1] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[mv_mode * 2][1] >>
                                MIN_BLOCK_SIZE_IN_BIT ;

    int            refindex   = (refframe < 0 ? 0 : refframe);
    int ** ***       allFwMv     = (fwd_flag ? img->allFwMv : img->allBwMv);
    int ** ***       pred_mv    = ((img->type != B_IMG) ? img->mv : (fwd_flag ? img->predBFwMv : img->predBBwMv));
    int            pmvr_mvd[2];

    if (!fwd_flag) {
        bwflag = 1;
    }

    for (j = j0; j < j1; j += step_v) {
        for (i = i0; i < i1; i += step_h) {
            if (input->b_pmvr_enabled) {
                pmvr_mvd_derivation(pmvr_mvd, allFwMv[j][i][refindex][mv_mode], pred_mv[j][i][refindex][mv_mode]);
            } else {
                pmvr_mvd[0] = allFwMv[j][i][refindex][mv_mode][0] - pred_mv[j][i][refindex][mv_mode][0];
                pmvr_mvd[1] = allFwMv[j][i][refindex][mv_mode][1] - pred_mv[j][i][refindex][mv_mode][1];
            }
            //--- store (oversampled) dmh mode ---
            for (l = 0; l < step_v; l++) {
                for (m = 0; m < step_h; m++) {
                    currMB->mvd[bwflag][j + l][i + m][2] = dmh_mode;
                }
            }
            for (k = 0; k < 2; k++) {
                curr_mvd = pmvr_mvd[k];

                //--- store (oversampled) mvd ---
                for (l = 0; l < step_v; l++) {
                    for (m = 0; m < step_h; m++) {
                        currMB->mvd[bwflag][j + l][i + m][k] = curr_mvd;

                    }
                }
            }
        }
    }

    return 0;
}
int StoreMotionVector8x8_sym(codingUnit *currMB, int i0, int j0, int i1, int j1, int refframe, int dmv_flag,
                             int fwd_flag, int mv_mode, int pdir)
{
    int            i, j, k, l, m;
    int            curr_mvd;
    int            bwflag     = ((refframe < 0 || (!fwd_flag)) ? 1 : 0);

    int            step_h     = g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[mv_mode * 2][0] >>
                                MIN_BLOCK_SIZE_IN_BIT;
    int            step_v     = g_blk_size[mv_mode * 2][1] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[mv_mode * 2][1] >>
                                MIN_BLOCK_SIZE_IN_BIT ;

    int            refindex   = (refframe < 0 ? 0 : refframe);
    int ** ***       allFwMv     = (fwd_flag ? img->allFwMv : img->allBwMv);
    int ** ***       pred_mv    = ((img->type != B_IMG) ? img->mv : (fwd_flag ? img->predBFwMv : img->predBBwMv));
    int   pmvr_mvd[2];

    if (!fwd_flag) {
        bwflag = 1;
    }

    if (pdir == SYM && mv_mode != 0) {
        allFwMv = img->allSymMv;
        pred_mv = img->predSymMv  ;
    }
    if (pdir == BID && mv_mode != 0) {
        //sw 10.1
        if (fwd_flag) {
            allFwMv = img->allBidFwMv;
            pred_mv = img->predBidFwMv;
            //pred_mv = img->predSymMv  ;
        } else {
            allFwMv = img->allBidBwMv;
            pred_mv = img->predBidBwMv;
        }

    }
    if (pdir == DUAL && mv_mode != 0) {
        //sw 10.1
        if (fwd_flag) {
            allFwMv = img->allDualFstMv;
            pred_mv = img->predDualFstMv;
            //pred_mv = img->predSymMv  ;
        } else {
            allFwMv = img->allDualSndMv;
            pred_mv = img->predDualSndMv;
        }

    }


    for (j = j0; j < j1; j += step_v) {
        for (i = i0; i < i1; i += step_h) {
            if (input->b_pmvr_enabled)
                pmvr_mvd_derivation(pmvr_mvd, allFwMv[j][i][refindex][mv_mode],
                                    pred_mv[j][i][refindex][mv_mode]);
            else {
                pmvr_mvd[0] = allFwMv[j][i][refindex][mv_mode][0] - pred_mv[j][i][refindex][mv_mode][0];
                pmvr_mvd[1] = allFwMv[j][i][refindex][mv_mode][1] - pred_mv[j][i][refindex][mv_mode][1];
            }
            for (k = 0; k < 2; k++) {
                curr_mvd = pmvr_mvd[k];

                //--- store (oversampled) mvd ---
                for (l = 0; l < step_v; l++) {
                    for (m = 0; m < step_h; m++) {
                        currMB->mvd[bwflag][j + l][i + m][k] = curr_mvd;

                    }
                }
            }
        }
    }

    return 0;
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
void pmvr_mvd_derivation(int mvd[2], int mv[2], int mvp[2])
{
    int ctr[2];
    ctr[0] = (mvp[0] >> 1) << 1;
    ctr[1] = (mvp[1] >> 1) << 1;
    if (abs(mv[0] - ctr[0]) > TH) {
        mvd[0] = (mv[0] + ctr[0] + pmvr_sign(mv[0] - ctr[0]) * TH) / 2 - mvp[0];
        mvd[1] = (mv[1] - ctr[1]) >> 1;
    } else if (abs(mv[1] - ctr[1]) > TH) {
        mvd[0] = (mv[0] - ctr[0]) >> 1;
        mvd[1] = (mv[1] + ctr[1] + pmvr_sign(mv[1] - ctr[1]) * TH) / 2 - mvp[1];
    } else {
        mvd[0] = mv[0] - mvp[0];
        mvd[1] = mv[1] - mvp[1];
    }
}
/*
*************************************************************************
* Function:Writes motion vectors of an 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int writePMVIdx8x8(int  i0,  int  j0,  int  i1, int  j1, int  refframe, int  dmv_flag, int  fwd_flag, int  mv_mode,
                   int  pdir, codingUnit *currMB, int  uiPositionInPic)
{
    int            i, j;

    int            bwflag     = ((refframe < 0 || (!fwd_flag)) ? 1 : 0);
    int            rate       = 0;

    int            step_h     = (g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                                (g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT) ;
    int            step_v     = (g_blk_size[mv_mode * 2][1] >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                                (g_blk_size[mv_mode * 2][1] >> MIN_BLOCK_SIZE_IN_BIT) ;

    SyntaxElement *currSE     = &img->MB_SyntaxElements[currMB->currSEnr];
    int           *bitCount   = currMB->bitcounter;
    int            refindex   = (refframe < 0 ? 0 : refframe);

    Slice         *currSlice  = img->currentSlice;
    DataPartition *dataPart;

    //assert(mv_mode != 0);

    if (!fwd_flag) {
        bwflag = 1;
    }
#if TRACE
    if (he->AEC_writting) {
        if (uiPositionInPic == 89 && img->tr == 2) {
            uiPositionInPic = uiPositionInPic;
        }
        fprintf_AMVPset(&(currMB->predMv_set[bwflag][j0][i0]));
        fprintf(hc->p_trace, "Position in Pic= %d, Size = %d,  type = %d Position in cu (%d, %d)\n", uiPositionInPic,
                currMB->ui_MbBitSize, currMB->cuType, i0, j0);
    }

#endif

    for (j = j0; j < j1; j += step_v) {
        for (i = i0; i < i1; i += step_h) {
            currSE->value2 = bwflag;
            currSE->type   = (img->type == B_IMG ? SE_BFRAME : SE_MVD);

            currSE->writing = writePMVindex_AEC;
            dataPart = & (currSlice->partArr[0]);
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE

            if (fwd_flag) {
                if (he->AEC_writting) {
#if TRACE
                    fprintf(hc->p_trace, "PMV_idx_fw = %3d  \n", currSE->value1);
#endif

                }
            } else {
                if (he->AEC_writting) {
#if TRACE
                    fprintf(hc->p_trace, "PMV_idx_bw = %3d  \n",  currSE->value1);
#endif

                }
            }

#endif
            bitCount[BITS_INTER_MB] += currSE->len;
            rate                    += currSE->len;
            currSE++;
            currMB->currSEnr++;

        }
    }

    return rate;
}

/*
*************************************************************************
* Function:Writes motion vectors of an 8x8 block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int writeMVD8x8(int  i0, int  j0, int  i1, int  j1, int  refframe, int  dmv_flag, int  fwd_flag, int  mv_mode,
                codingUnit *currMB, int  uiPositionInPic)
{
    int            i, j, k;
    int            curr_mvd;
    int            dmh_mode;
    int            bwflag     = ((refframe < 0 || (!fwd_flag)) ? 1 : 0);
    int            rate       = 0;

    int            step_h     = (g_blk_size[mv_mode * 2][0] >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                                (g_blk_size[mv_mode * 2][0] >>  MIN_BLOCK_SIZE_IN_BIT);
    int            step_v     = (g_blk_size[mv_mode * 2][1] >>  MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                                (g_blk_size[mv_mode * 2][1] >>  MIN_BLOCK_SIZE_IN_BIT) ;

    SyntaxElement *currSE     = &img->MB_SyntaxElements[currMB->currSEnr];
    int           *bitCount   = currMB->bitcounter;

    int            refindex   = (refframe < 0 ? 0 : refframe);
    int ** ***       allFwMv     = (fwd_flag ? img->allFwMv : img->allBwMv);
    int ** ***       pred_mv    = ((img->type != B_IMG) ? img->mv : (fwd_flag ? img->predBFwMv : img->predBBwMv));
    //!<added by lzhang for AEC
    Slice         *currSlice  = img->currentSlice;
    DataPartition *dataPart;

    if (!fwd_flag) {
        bwflag = 1;
    }


    for (j = j0; j < j1; j += step_v) {
        for (i = i0; i < i1; i += step_h) {
            if (img->type == F_IMG && input->b_dmh_enabled && currMB->b8pdir[0] == FORWARD && currMB->b8pdir[1] == FORWARD &&
                currMB->b8pdir[2] == FORWARD && currMB->b8pdir[3] == FORWARD && img->typeb != BP_IMG)

            {
                if (i == 0 && j == 0 && (!(currMB->ui_MbBitSize == B8X8_IN_BIT && currMB->cuType >= P2NXN &&
                                           currMB->cuType <= PVER_RIGHT))) {
                    dmh_mode = currMB->mvd[bwflag][j][i][2];
                    currSE->value1  = dmh_mode;
                    currSE->value2  = currMB->ui_MbBitSize; // only used for context determination
                    currSE->type    = ((img->type == B_IMG) ? SE_BFRAME : SE_DMH);
                    currSE->writing = writeDmhMode;
                    dataPart        = & (currSlice->partArr[0]);
                    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
#if TRACE
                    if (he->AEC_writting) {
                        fprintf(hc->p_trace, "dmh_mode = %3d\n", dmh_mode);
                    }
#endif
                    bitCount[BITS_INTER_MB] += currSE->len;
                    rate                    += currSE->len;
                    currSE++;
                    currMB->currSEnr++;
                }
            }

            if (!(img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)) {

                for (k = 0; k < 2; k++) {
                    curr_mvd = currMB->mvd[bwflag][j][i][k];

                    currSE->value1 = curr_mvd;
                    currSE->type   = ((img->type == B_IMG) ? SE_BFRAME : SE_MVD);


                    currSE->value2  = 2 * k + bwflag; // identifies the component and the direction; only used for context determination
                    currSE->writing = writeMVD_AEC;
                    dataPart = & (currSlice->partArr[0]);
                    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

#if TRACE
                    {
                        if (fwd_flag) {
                            if (he->AEC_writting) {

                                fprintf(hc->p_trace, "FMVD(%d) = %3d  (org_mv %3d pred_mv %3d) %d\n", k, curr_mvd, allFwMv[j][i][refindex][mv_mode][k],
                                        pred_mv[j][i][refindex][mv_mode][k], currSE->value2);

                            }
                        } else {
                            if (AEC_writting) {

                                fprintf(hc->p_trace, "BMVD(%d) = %3d  (org_mv %3d pred_mv %3d)\n", k, curr_mvd, allFwMv[j][i][refindex][mv_mode][k],
                                        pred_mv[j][i][refindex][mv_mode][k]);

                            }
                        }
                    }
#endif
                    bitCount[BITS_INTER_MB] += currSE->len;
                    rate                    += currSE->len;
                }
            }

        }
    }

    return rate;
}


int writeCBPandDqp(codingUnit *currMB, int pos, int *rate_top, int *rate_bot, int uiPositionInPic)
{
    int             rate      = 0;
    int            *bitCount  = currMB->bitcounter;
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    DataPartition  *dataPart;
    Slice          *currSlice = img->currentSlice;

    currSE->value1 = currMB->cbp; //CBP

    currSE->writing = writeCBP;
    dataPart = & (currSlice->partArr[0]);
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

    bitCount[BITS_CBP_MB] += currSE->len;
    rate                   = currSE->len;
#if TRACE

    if (he->AEC_writting) {
        fprintf(hc->p_trace, "CBP (%2d,%2d) = %3d\n", img->mb_x, img->mb_y, currMB->cbp);
    }

#endif

#if MB_DQP         ///////////// added by KY, write Dqp information

    if (currMB->cbp != 0 && input->useDQP) {
        //if (IS_INTER (currMB))  currSE->type = SE_DELTA_QUANT_INTER;
        //else                    currSE->type = SE_DELTA_QUANT_INTRA;

        currSE->value1 = currMB->delta_qp;
        currSE->writing = writeDqp;//  (eep_dp, (unsigned char) currMB->delta_qp, ctx->tu_contexts[0]) ;// write DQP;
        dataPart = & (currSlice->partArr[0]);   // this sentence, exist or not does not affect the total bits
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

        bitCount[BITS_DELTA_QUANT_MB] += currSE->len;
        rate += currSE->len;

#if TRACE
        snprintf(currSE->tracestring, TRACESTRING_SIZE, "Delta QP (%2d,%2d) = %3d", img->mb_x, img->mb_y, currMB->delta_qp);
#endif

        currSE++;
        currMB->currSEnr++;
    }

#endif

    *rate_top += rate / 2;
    *rate_bot += rate / 2;

    return 0;

}

/*lgp*/
/*
*************************************************************************
* Function:Writes motion info
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int storeMotionInfo(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int pos, int dmh_mode)
{
    int k, j0, i0, refframe;
    int             no_bits   = 0;

    int   bframe          = (img->type == B_IMG);
    int **refframe_array = ((img->type == B_IMG) ? img->fw_refFrArr : hc->refFrArr);
    int **bw_refframe_array = (img->type == B_IMG) ? img->bw_refFrArr : hc->p_snd_refFrArr;

    int   step_h0         = (g_blk_size[  currMB->cuType * 2][0] >>  MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                            (g_blk_size[  currMB->cuType * 2][0] >>  MIN_BLOCK_SIZE_IN_BIT);
    int   step_v0         = (g_blk_size[  currMB->cuType * 2][1] >>  MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                            (g_blk_size[  currMB->cuType * 2][1] >>  MIN_BLOCK_SIZE_IN_BIT);

    int   b8_y     = (uiPositionInPic / img->PicWidthInMbs) << 1;   //img->block8_y;
    int   b8_x     = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;

    int start_x, start_y, widht, height;

    int l, m;
    //AMVP_SET***** predFwMv_set =  ( img->type != B_IMG ) ? img->predPMv_set  : img->predBFwMv_set ;
    //===== write forward motion vectors =====
    if (!IS_INTRA(currMB)) {
        for (j0 = pos; j0 < 2; j0 += step_v0) {
            for (i0 = 0; i0 < 2; i0 += step_h0) {
                k = j0 * 2 + i0;
                if ((currMB->b8pdir[k] == FORWARD || currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID ||
                     currMB->b8pdir[k] == DUAL) && currMB->b8mode[k] != 0) {

                    get_b8_offset(currMB->b8mode[k], uiBitSize, i0, j0, &start_x, &start_y, &widht, &height);
                    refframe  = refframe_array[b8_y + start_y ][b8_x + start_x ];
                    if (currMB->b8pdir[k] == SYM) {
                        StoreMotionVector8x8_sym(currMB, i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 1, currMB->b8mode[k],
                                                 currMB->b8pdir[k]);
                    }

                    else if (currMB->b8pdir[k] == BID) {
                        StoreMotionVector8x8_sym(currMB, i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 1, currMB->b8mode[k],
                                                 currMB->b8pdir[k]);
                    } else if (currMB->b8pdir[k] == DUAL) {
                        StoreMotionVector8x8_sym(currMB, i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 1, currMB->b8mode[k],
                                                 currMB->b8pdir[k]);
                    } else {
                        StoreMotionVector8x8(currMB, i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 1, currMB->b8mode[k], dmh_mode);
                    }
                } else if (currMB->b8mode[k] == 0) {
                    for (l = 0; l < step_v0; l++) {
                        for (m = 0; m < step_h0; m++) {
                            /*if ( currMB->cuType == 0)
                            {
                                CopyPMVset(  predFwMv_set[i0][j0][0][0], &(currMB->predMv_set[0][j0 + l][i0 + m]) );
                            }
                            else
                            {
                                assert(currMB->cuType == PNXN);
                                CopyPMVset(  predFwMv_set[i0][j0][1][0], &(currMB->predMv_set[0][j0 + l][i0 + m]) );
                            }*/
                            currMB->mvd[0][j0 + l][i0 + m][0] = currMB->mvd[0][j0 + l][i0 + m][1] = 0;
                            currMB->mvd[1][j0 + l][i0 + m][0] = currMB->mvd[1][j0 + l][i0 + m][1] = 0;

                        }
                    }
                }
            }
        }
    }

    //===== write backward motion vectors =====
    if (IS_INTERMV(currMB) && bframe) {
        for (j0 = pos; j0 < 2; j0 += step_v0) {
            for (i0 = 0; i0 < 2; i0 += step_h0) {
                k = j0 * 2 + i0;
                if ((currMB->b8pdir[k] == BACKWARD || currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID) &&
                    currMB->b8mode[k] != 0) {  //has backward vector

                    get_b8_offset(currMB->b8mode[k], uiBitSize, i0, j0, &start_x, &start_y, &widht, &height);
                    refframe  = bw_refframe_array[b8_y + start_y ][b8_x + start_x ];

                    if (currMB->b8pdir[k] == BID) {
                        StoreMotionVector8x8_sym(currMB, i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 0, currMB->b8mode[k] ,
                                                 currMB->b8pdir[k]);
                    } else {
                        StoreMotionVector8x8(currMB, i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 0, currMB->b8mode[k], dmh_mode);
                    }
                }
            }
        }
    }

    return no_bits;
}

int writeFrameRef(codingUnit *currMB, int  mode, int  i, int  j, int  fwd_flag, int  ref, int uiPositionInPic)
{
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    int            *bitCount  = currMB->bitcounter;
    int             rate      = 0;
    int             num_ref   = (img->type == F_IMG || img->type == P_IMG) ? 2 : 1;
    Slice          *currSlice = img->currentSlice;
    DataPartition  *dataPart  ;

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable)
    { return 0; }


    if (num_ref == 1) {
        return 0;
    }

    currSE->value1 = ref;

    currSE->type   = ((img->type == B_IMG) ? SE_BFRAME : SE_REFFRAME);

    currSE->bitpattern = currSE->value1;

    dataPart = & (currSlice->partArr[0]);
    currSE->writing = writeRefFrame;
    currSE->value2 = (fwd_flag) ? 0 : 1;
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

    bitCount[BITS_INTER_MB] += currSE->len;
    rate                    += currSE->len;

#if TRACE

    if (fwd_flag) {
        if (he->AEC_writting) {
            fprintf(hc->p_trace, "Fwd Ref frame no %d\n", currSE->bitpattern);
        }
    } else {
        if (he->AEC_writting) {
            fprintf(hc->p_trace, "Bwd Ref frame no %d\n", currSE->bitpattern);
        }
    }

#endif


    return rate;
}

/*
*************************************************************************
* Function:Writes motion info
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int writeReferenceIndex(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int pos,
                        int *rate_top, int *rate_bot)
{
    int k, j0, i0, refframe;
    int             no_bits   = 0;

    int   bframe          = (img->type == B_IMG);
    int **refframe_array = ((img->type == B_IMG) ? img->fw_refFrArr : hc->refFrArr);
    int **bw_refframe_array = (img->type == B_IMG) ? img->bw_refFrArr : hc->p_snd_refFrArr;
    int   step_h0  = 0       ;
    int   step_v0  = 0       ;
    int   b8_y     = (uiPositionInPic / img->PicWidthInMbs) << 1;   //img->block8_y;
    int   b8_x     = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;
    int start_x, start_y, widht, height;

    if (currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
        return 0;
    } else {

        step_h0     = g_blk_size[currMB->cuType * 2][0] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[currMB->cuType * 2][0] >>
                      MIN_BLOCK_SIZE_IN_BIT;
        step_v0     = g_blk_size[currMB->cuType * 2][1] >> MIN_BLOCK_SIZE_IN_BIT == 0 ? 1 : g_blk_size[currMB->cuType * 2][1] >>
                      MIN_BLOCK_SIZE_IN_BIT;

    }

    //forward reference
    for (j0 = pos; j0 < 2;) {
        for (i0 = 0; i0 < 2;) {
            k = j0 * 2 + i0;
            no_bits = 0;
            if ((currMB->b8pdir[k] == FORWARD || currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID ||
                 currMB->b8pdir[k] == DUAL) && currMB->b8mode[k] != 0 && img->type != INTRA_IMG) {

                get_b8_offset(currMB->b8mode[k], uiBitSize, i0, j0, &start_x, &start_y, &widht, &height);
                refframe  = refframe_array[b8_y + start_y ][b8_x + start_x ];
                img->subblock_x = start_x; // position used for context determination
                img->subblock_y = start_y; // position used for context determination

                if (img->num_of_references > 1) {
                    no_bits   = writeFrameRef(currMB, currMB->b8mode[k], i0, j0, 1, refframe, uiPositionInPic);
                }
            }

            if (j0 < 1) {
                *rate_top += no_bits;
            } else {
                *rate_bot += no_bits;
            }

            i0 += max(1, step_h0);
        }

        j0 += max(1, step_v0);
    }

    //backward reference
    if (bframe) {
        for (j0 = pos; j0 < 2;) {
            if ((currMB->cuType == I8MB && j0 == 0)) {
                j0 += 1;
                continue;
            }

            for (i0 = 0; i0 < 2;) {
                k = j0 * 2 + i0;
                no_bits = 0;
                if ((currMB->b8pdir[k] == BACKWARD || currMB->b8pdir[k] == BID) && currMB->b8mode[k] != 0) {     //has backward vector

                    get_b8_offset(currMB->b8mode[k], uiBitSize, i0, j0, &start_x, &start_y, &widht, &height);
                    refframe  = bw_refframe_array[b8_y + start_y ][b8_x + start_x ];
                    img->subblock_x = start_x; // position used for context determination
                    img->subblock_y = start_y; // position used for context determination



                    if (img->num_of_references > 1) {
                        no_bits   = writeFrameRef(currMB, currMB->b8mode[k], i0, j0, 0, refframe, uiPositionInPic);
                    }
                }

                if (j0 < 1) {
                    *rate_top += no_bits;
                } else {
                    *rate_bot += no_bits;
                }

                i0 += max(1, step_h0);
            }

            j0 += max(1, step_v0);
        }
    }

    return 0;
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

int writeMVD(codingUnit *currMB, int pos, int *rate_top, int *rate_bot, int uiPositionInPic)
{
    int k, j0, i0, refframe;

    int             no_bits   = 0;

    int   bframe          = (img->type == B_IMG);
    int **refframe_array = ((img->type == B_IMG) ? img->fw_refFrArr : hc->refFrArr);
    int **bw_refframe_array = (img->type == B_IMG) ? img->bw_refFrArr : hc->p_snd_refFrArr;

    int   step_h0         = (g_blk_size[currMB->cuType * 2][0] >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                            (g_blk_size[currMB->cuType * 2][0] >> MIN_BLOCK_SIZE_IN_BIT);
    int   step_v0         = (g_blk_size[currMB->cuType * 2][1] >> MIN_BLOCK_SIZE_IN_BIT) == 0 ? 1 :
                            (g_blk_size[currMB->cuType * 2][1] >> MIN_BLOCK_SIZE_IN_BIT);


    int   b8_x     = (uiPositionInPic % img->PicWidthInMbs) << 1;
    int   b8_y     = (uiPositionInPic / img->PicWidthInMbs) << 1;
    int width, height, start_x, start_y;

    if (currMB->cuType == I8MB || currMB->cuType == I16MB || currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
        return 0;
    }

    for (j0 = pos; j0 < 2;) {
        for (i0 = 0; i0 < 2;) {
            k = j0 * 2 + i0;

            no_bits = 0;
            if ((currMB->b8pdir[k] == FORWARD || currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID ||
                 currMB->b8pdir[k] == DUAL) && currMB->b8mode[k] != 0) {

                get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &start_x, &start_y, &width, &height);
                refframe  = refframe_array[b8_y + start_y ][b8_x + start_x ];

#if INTERLACE_CODING
                if ((currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID || currMB->b8pdir[k] == DUAL))
#else
                if ((currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID || currMB->b8pdir[k] == DUAL) &&
                    input->InterlaceCodingOption == FRAME_CODING)
#endif
                {

                    get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &start_x, &start_y, &width, &height);
                    refframe  = refframe_array[b8_y + start_y ][b8_x + start_x ];

                    img->subblock_x = start_x; // position used for context determination
                    img->subblock_y = start_y; // position used for context determination

                    no_bits  += writeMotionVector8x8_sym(i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 1, currMB->b8mode[k],
                                                         currMB->b8pdir[k], currMB,  uiPositionInPic);
                } else

                {
                    get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &start_x, &start_y, &width, &height);
                    img->subblock_x = start_x; // position used for context determination
                    img->subblock_y = start_y; // position used for context determination
                    no_bits   = writeMVD8x8(i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 1, currMB->b8mode[k], currMB,
                                            uiPositionInPic);
                }
            }

            if (j0 < 1) {
                *rate_top += no_bits;
            } else {
                *rate_bot += no_bits;
            }

            i0 += max(1, step_h0);
        }

        j0 += max(1, step_v0);

    }

    if (bframe) {
        for (j0 = pos; j0 < 2;) {
            if ((currMB->cuType == I8MB && j0 == 0)) {
                j0 += 1;
                continue;
            }

            for (i0 = 0; i0 < 2;) {
                k = j0 * 2 + i0;
                /*lgp*/
                no_bits = 0;

                if ((currMB->b8pdir[k] == BACKWARD ||  currMB->b8pdir[k] == BID) && currMB->b8mode[k] != 0) {     //has backward vector


                    get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i0, j0, &start_x, &start_y, &width, &height);
                    refframe  = refframe_array[b8_y + start_y ][b8_x + start_x ];
                    img->subblock_x = start_x; // position used for context determination
                    img->subblock_y = start_y; // position used for context determination

#if INTERLACE_CODING
                    if (currMB->b8pdir[k] == BID)
#else
                    if ((currMB->b8pdir[k] == BID) && input->InterlaceCodingOption == FRAME_CODING)
#endif
                    {
                        no_bits  = writeMotionVector8x8_sym(i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 0, currMB->b8mode[k],
                                                            currMB->b8pdir[k], currMB,  uiPositionInPic);
                    } else {
                        no_bits   = writeMVD8x8(i0, j0, i0 + step_h0, j0 + step_v0, refframe, 0, 0, currMB->b8mode[k], currMB,
                                                uiPositionInPic);
                    }
                }

                if (j0 < 1) {
                    *rate_top += no_bits;
                } else {
                    *rate_bot += no_bits;
                }

                i0 += max(1, step_h0);
            }

            j0 += max(1, step_v0);
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////write PMV index
//////////////////////////////////////////////////////////////////////////////////////////////////////////
    return 0;
}


/*
*************************************************************************
* Function:Writes CBP, DQUANT, and Luma Coefficients of an codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int writeBlockCoeff(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                    unsigned int uiPixInSMB_y, int block8x8, int uiPositionInPic)
{
    int             rate      = 0;

    //=====  L U M I N A N C E   =====
    //--------------------------------
    if (block8x8 < 6) {
        if (currMB->cbp & (1 << block8x8)) {
            if (block8x8 < 4) {
                currMB->l_ipred_mode = currMB->real_intra_pred_modes[block8x8];
                rate  += writeLumaCoeff8x8(Quant_Coeff, currMB, uiBitSize, uiPixInSMB_x, uiPixInSMB_y, block8x8,
                                           (currMB->b8mode[block8x8] == IBLOCK), uiPositionInPic);    //changed by lzhang
            } else {
                if (currMB->trans_size == 0) {
                    rate += writeChromaCoeff8x8(Quant_Coeff, currMB, uiBitSize - 1, uiPixInSMB_x, uiPixInSMB_y, block8x8,
                                                IS_INTRA(currMB), uiPositionInPic);     //changed by lzhang
                } else {
                    rate += writeChromaCoeff8x8(Quant_Coeff, currMB, uiBitSize, uiPixInSMB_x, uiPixInSMB_y, block8x8, IS_INTRA(currMB),
                                                uiPositionInPic);  //changed by lzhang
                }
            }
        }
    }

    return rate;
}


void write_one_subMB(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int eos_bit)
{
    int        *bitCount = currMB->bitcounter;
    int i;
    int dummy;
    extern int AEC_encoding;
    int iStartX, iStartY;
    int WidthInLCU  = ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0));
    int HeightInLCU = ((img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 :
                       0));
    int LCUNumInPic = WidthInLCU * HeightInLCU;
    int currSMB_nr = (img->pix_y >> input->g_uiMaxSizeInBit) * WidthInLCU + (img->pix_x >> input->g_uiMaxSizeInBit);
    AEC_encoding = 1;

    //--- write header ---
    writeMBHeader(currMB, uiBitSize, uiPositionInPic, 0);    //qyu 0825 uiBitSize,uiPositionInPic

#if MB_DQP
#if LEFT_PREDICTION

#else
    if (input->useDQP) {
        if (currMB->cbp != 0) {
            currMB->delta_qp = currMB->qp - he->lastQP;
            he->lastQP = currMB->qp;
        } else {
            currMB->delta_qp = 0;
            currMB->qp = he->lastQP;
            for (j = 0; j < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
                for (i = 0; i < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); i++) {
                    tmpMB = &img->mb_data[uiPositionInPic + i + j * img->PicWidthInMbs]; //save data to mb_data
                    tmpMB->delta_qp      = 0;
                    tmpMB->qp     = he->lastQP;
                }
            }

        }
    } else {
        currMB->delta_qp = 0;
        he->lastQP = img->qp;
    }
#endif
#endif

    //  Do nothing more if copy and inter mode
    if (input->chroma_format == 1) {
        unsigned int uiPixInSMB_x_luma, uiPixInSMB_y_luma, uiPixInSMB_x, uiPixInSMB_y;

        //      unsigned int uiPixInSMB_x_luma, uiPixInSMB_y_luma, uiPixInSMB_x, uiPixInSMB_y; //moved
        writeReferenceIndex(currMB, uiBitSize, uiPositionInPic, 0, &dummy, &dummy);
        writeMVD(currMB, 0, &dummy, &dummy, uiPositionInPic);

        if ((IS_INTERMV(currMB)  || IS_INTRA(currMB)) ||
            (currMB->cbp != 0)) {
            writeCBPandDqp(currMB, 0, &dummy, &dummy, uiPositionInPic);

            uiPixInSMB_x_luma = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE - img->pix_x;
            uiPixInSMB_y_luma = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE - img->pix_y;

            for (i = 0; i < 6;) {
                if (i < 4) {
                    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->trans_size == 1 && IS_INTER(currMB) &&
                            (currMB->cuType == P2NXN || currMB->cuType == PHOR_UP || currMB->cuType == PHOR_DOWN))) {
                        iStartX = 0;
                        iStartY = (uiBitSize == B64X64_IN_BIT) ? (i * (1 << (uiBitSize - 3))) : (i * (1 << (uiBitSize - 2)));
                    } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->trans_size == 1 && IS_INTER(currMB) &&
                               (currMB->cuType == PNX2N || currMB->cuType == PVER_LEFT || currMB->cuType == PVER_RIGHT))) {
                        iStartX = (uiBitSize == B64X64_IN_BIT) ? (i * (1 << (uiBitSize - 3))) : (i * (1 << (uiBitSize - 2)));
                        iStartY = 0;
                    } else if (input->useSDIP && (currMB->trans_size == 1 && IS_INTRA(currMB) && (currMB->cuType == InNxNMB ||
                                                  currMB->cuType == INxnNMB))) {
                        iStartX = currMB->cuType == InNxNMB ? 0 : (1 << (uiBitSize - 2)) * i;
                        iStartY = currMB->cuType == InNxNMB ? (1 << (uiBitSize - 2)) * i : 0;
                    } else {
                        iStartX = (i % 2) * (1 << (uiBitSize - 1));
                        iStartY = (i / 2) * (1 << (uiBitSize - 1));
                    }
                    uiPixInSMB_x = uiPixInSMB_x_luma + iStartX ;
                    uiPixInSMB_y = uiPixInSMB_y_luma + iStartY ;
                } else if (i == 4) {
                    uiPixInSMB_x = (uiPixInSMB_x_luma >> 1) ;
                    uiPixInSMB_y = (uiPixInSMB_y_luma >> 1) + (1 << input->g_uiMaxSizeInBit);
                } else {
                    uiPixInSMB_x = (uiPixInSMB_x_luma >> 1) + (1 << (input->g_uiMaxSizeInBit - 1));
                    uiPixInSMB_y = (uiPixInSMB_y_luma >> 1) + (1 << input->g_uiMaxSizeInBit);
                }

                writeBlockCoeff(img->Coeff_all_to_write_ALF[currSMB_nr], currMB, uiBitSize - currMB->trans_size/*1*/, uiPixInSMB_x,
                                uiPixInSMB_y, i, uiPositionInPic);
                if (currMB->trans_size == 0 && i == 0) {
                    i += 4;
                } else {
                    i++;
                }
            }
        }

        //--- set total bit-counter ---
        bitCount[BITS_TOTAL_MB] = bitCount[BITS_MB_MODE] + bitCount[BITS_COEFF_Y_MB]     + bitCount[BITS_INTER_MB]
                                  + bitCount[BITS_CBP_MB]  + bitCount[BITS_DELTA_QUANT_MB] + bitCount[BITS_COEFF_UV_MB];
        stat->bit_slice += bitCount[BITS_TOTAL_MB];
    }
}
void write_one_SMB(unsigned int uiBitSize, unsigned int uiPositionInPic, int eos_bit)
{
    int i;
    codingUnit *currMB;
    int pix_x_InPic_start = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_y_InPic_start = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_x_InPic_end = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);     //Liwr 0915
    int pix_y_InPic_end = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);     //Liwr 0915
    int  iBoundary_start = (pix_x_InPic_start >= img->width) || (pix_y_InPic_start >= img->height);
    int  iBoundary_end = (pix_x_InPic_end > img->width) || (pix_y_InPic_end > img->height);
    int pos_x, pos_y, pos_InPic;

    if (!iBoundary_start) {
#if TRACE

        if (uiPositionInPic == img->current_mb_nr && uiBitSize == input->g_uiMaxSizeInBit) {
            fprintf(hc->, "\n*********** Pic: %i (I/P) MB: %i Slice: %i **********\n\n", img->tr, uiPositionInPic,
                    img->current_slice_nr);
        }

#endif
        currMB = &img->mb_data[uiPositionInPic];

        if (currMB->ui_MbBitSize < uiBitSize) {
            if (!iBoundary_end) {
                writeSplitFlag(1, currMB, uiBitSize);
            }

            for (i = 0; i < 4; i++) {
                int mb_x = (i % 2) << (uiBitSize - MIN_CU_SIZE_IN_BIT - 1);     //uiBitSize 5:1 ; 6:2
                int mb_y = (i / 2) << (uiBitSize - MIN_CU_SIZE_IN_BIT - 1);     //uiBitSize 5:1 ; 6:2
                int pos = uiPositionInPic + mb_y * img->PicWidthInMbs + mb_x;
                //Liwr 0915
                pos_x = pix_x_InPic_start + (mb_x << MIN_CU_SIZE_IN_BIT);
                pos_y = pix_y_InPic_start + (mb_y << MIN_CU_SIZE_IN_BIT);
                pos_InPic = (pos_x >= img->width || pos_y >= img->height);

                if (pos_InPic) {
                    continue;
                }

                write_one_SMB(uiBitSize - 1, pos, eos_bit);
            }

            return;
        }

        if (uiBitSize > MIN_CU_SIZE_IN_BIT && !iBoundary_end) {
            writeSplitFlag(0, currMB, uiBitSize);
        }

        if (!iBoundary_end) {
            write_one_subMB(currMB, uiBitSize, uiPositionInPic, eos_bit);
#if MB_DQP
            he->DqpCount++;
            if (currMB->delta_qp == 0) {
                he->zeroDqpCount++;    // added by KY, calculate the number of zero DQP
            }
#endif
        } else {
            return;
        }
    } else {
        return;
    }
}



