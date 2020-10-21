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

#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <assert.h>
#include "rdopt_coding_state.h"
#include "../../lcommon/inc/memalloc.h"
#include "refbuf.h"
#include "block.h"
#include "vlc.h"
#include "codingUnit.h"

#include "global.h"
#include "AEC.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/transform.h"
#include "../../lcommon/inc/block_info.h"
#include "../../lcommon/inc/inter-prediction.h"

#include "fast_me.h"
#include "mv-search.h"

#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif


//==== MODULE PARAMETERS ====
int   best_mode;
//CXR SDIP fast
double modeRDcost[14];

int weighted_skipmode_fix;
int best_md_directskip_mode;
int best_trans_size;
int   rec_mbY[MAX_CU_SIZE][MAX_CU_SIZE], rec_mbU[MAX_CU_SIZE][MAX_CU_SIZE / 2], rec_mbV[MAX_CU_SIZE][MAX_CU_SIZE / 2];
int **Coeff_all_tmp_best = NULL;

#if MB_DQP
int   dQP;
int QP;
int left_cu_qp;
int previouse_qp;
#endif

int   cbp;
int   cbp_blk;
int   frefframe[2][2], brefframe[2][2], b8mode[4], b8pdir[4];
int   p_snd_frefframe[2][2];
int   best8x8mode [4];                // [block]
int   best8x8pdir [MAXMODE][4];       // [mode][block]
int   best8x8ref  [MAXMODE][4];       // [mode][block]
CSptr cs_submb_best, cs_next_best;
CSptr cs_submb[4];
CSptr cs_mb = NULL, cs_b8 = NULL, cs_cm = NULL;

CSptr cs_tmp = NULL;

int   best_c_imode;


int   best8x8bwref     [MAXMODE][4];       // [mode][block]
int   best8x8syoneForthRefY     [MAXMODE][4][2];       // [mode][block]

int   best8x8bidref     [MAXMODE][4][2];       // [mode][block]
int   best8x8dualref     [MAXMODE][4][2];

int   best_intra_pred_modes_tmp[4];
int   best_real_intra_pred_modes_tmp[4];
int   best_ipredmode_tmp[2][2];

int g_best_dmh_mode = 0;

int best_brp;



#define IS_FW ((best8x8pdir[mode][k]==FORWARD || best8x8pdir[mode][k]==SYM) && (mode!=PNXN || best8x8mode[k]!=0 || !bframe))
#define IS_BW ((best8x8pdir[mode][k]==BACKWARD || best8x8pdir[mode][k]==SYM) && (mode!=PNXN || best8x8mode[k]!=0))
#define  IS_bid ((best8x8pdir[mode][k]==BID) && (mode!=PNXN || best8x8mode[k]!=0))

/*
*************************************************************************
* Function:delete structure for RD-optimized mode decision
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void clear_rdopt()
{
    unsigned int i = 0;
    for (i = 0; i < (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT); i++) {
        delete_coding_state(cs_submb[i]);
    }
    delete_coding_state(cs_submb_best);
    delete_coding_state(cs_next_best);

    free_mem2Dint(Coeff_all_tmp_best);    //qyu 0821

    // structure for saving the coding state
    delete_coding_state(cs_mb);
    delete_coding_state(cs_b8);
    delete_coding_state(cs_cm);
    delete_coding_state(cs_tmp);

}

/*
*************************************************************************
* Function:create structure for RD-optimized mode decision
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void init_rdopt()
{
    unsigned int i = 0;
    for (i = 0; i < (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT); i++) {
        cs_submb[i] = create_coding_state();
    }
    cs_submb_best = create_coding_state();
    cs_next_best = create_coding_state();

    get_mem2Dint(&Coeff_all_tmp_best, (1 << input->g_uiMaxSizeInBit) << 1, 1 << input->g_uiMaxSizeInBit);

    // structure for saving the coding state
    cs_mb  = create_coding_state();
    cs_b8  = create_coding_state();
    cs_cm  = create_coding_state();
    cs_tmp = create_coding_state();
}

/*
*************************************************************************
* Function:Get RD cost for AVS intra block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
double RDCost_for_AVSIntraBlocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic,
                                 int *nonzero, int b8, int ipmode, double lambda, double  min_rdcost, int *mostProbableMode, int isChroma)
{
    //qyu 0824 uiBitSize
    int x, y, rate, tmp_cbp, tmp_cbp_blk;
    int distortion;
    int block_x;
    int block_y;
    int pic_pix_x;
    int pic_pix_y;

    SyntaxElement *currSE;
    int incr_y = 1, off_y = 0;

    Slice          *currSlice    =  img->currentSlice;
    DataPartition  *dataPart;

    block_x    = (b8 % 2) << uiBitSize;
    block_y    = (b8 / 2) << uiBitSize;
    pic_pix_x  = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + block_x;   //img->pix_x+block_x;
    pic_pix_y  = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + block_y;   //img->pix_y+block_y;
    currSE   = &img->MB_SyntaxElements[currMB->currSEnr];
    if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {  //add yuqh20130824
        block_x = currMB->cuType == InNxNMB ? 0 : b8 << (uiBitSize - 1);
        block_y = currMB->cuType == InNxNMB ? b8 << (uiBitSize - 1) : 0;
        pic_pix_x  = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + block_x;
        pic_pix_y  = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + block_y;
    }
    //===== perform DCT, Q, IQ, IDCT, Reconstruction =====

    if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB)
        for (y = 0; y < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); y++) {
            for (x = 0; x < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); x++) {
                hc->tmp_block_88_inv[y][x] =
                    img->resiY[y][x];      //the subblock to be processed is in the top left corner of img->resiY[][].

            }
        }
    else for (y = 0; y < (1 << uiBitSize); y++) {
            for (x = 0; x < (1 << uiBitSize); x++) {
                hc->tmp_block_88_inv[y][x] =
                    img->resiY[y][x];      //the subblock to be processed is in the top left corner of img->resiY[][].
            }
        }
    tmp_cbp = tmp_cbp_blk = 0;

    if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {//add yu 20130824
        transform_NSQT(hc->tmp_block_88_inv, uiBitSize, currMB, 0, input->b_secT_enabled, input->sample_bit_depth);
    } else {
        transform_B8(hc->tmp_block_88_inv, uiBitSize, currMB, 0, input->b_secT_enabled, input->sample_bit_depth);
    }
    he->g_block8x8 = b8;

    he->g_intra_mode = 1;


#if MB_DQP
    quantization(currMB->qp - MIN_QP, 4, b8, hc->tmp_block_88_inv, uiBitSize, uiPositionInPic, currMB, 0,
                 currMB->l_ipred_mode);
    inverse_quantization(currMB->qp - MIN_QP, 4, b8, hc->tmp_block_88_inv, 0, &tmp_cbp, uiBitSize, uiPositionInPic, currMB,
                         0);
    inverse_transform(b8, hc->tmp_block_88_inv, uiBitSize, uiPositionInPic, currMB, 0);
#else
    quantization(img->qp - MIN_QP, 4, b8, hc->tmp_block_88_inv, uiBitSize, uiPositionInPic, currMB, 0,
                 currMB->l_ipred_mode);
    inverse_quantization(img->qp - MIN_QP, 4, b8, hc->tmp_block_88_inv, 0, &tmp_cbp, uiBitSize, uiPositionInPic, currMB, 0);
    inverse_transform(b8, hc->tmp_block_88_inv, uiBitSize, uiPositionInPic, currMB, 0);

#endif

    *nonzero = (tmp_cbp != 0);

    //===== get distortion (SSD) of 8x8 block =====
    distortion = 0;

    /*lgp*/
    if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB)
        for (y = 0; y < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); y++) {
            for (x = 0; x < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); x++) {
                distortion += img->quad[he->imgY_org[pic_pix_y + incr_y * y + off_y][pic_pix_x + x] - hc->imgY[pic_pix_y + incr_y * y +
                                        off_y][pic_pix_x + x]];
            }
        }
    else
        for (y = 0; y < (1 << uiBitSize); y++) {
            for (x = 0; x < (1 << uiBitSize); x++) {
                distortion += img->quad[he->imgY_org[pic_pix_y + incr_y * y + off_y][pic_pix_x + x] - hc->imgY[pic_pix_y + incr_y * y +
                                        off_y][pic_pix_x + x]];
            }
        }

    //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO UVLC) =====
    currSE->value1 = (mostProbableMode[0] == ipmode) ? -2 : ((mostProbableMode[1] == ipmode) ? -1 :
                     (ipmode < mostProbableMode[0] ? ipmode : (ipmode < mostProbableMode[1] ? ipmode - 1 : ipmode - 2)));


    //--- set position and type ---
    currSE->context = 4 * b8;
    currSE->type    = SE_INTRAPREDMODE;

    //--- encode and update rate ---
    currSE->writing = writeIntraPredMode;
    //--- choose data partition ---
    dataPart = & (currSlice->partArr[0]);
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

    rate = currSE->len;

    //===== RATE for LUMINANCE COEFFICIENTS =====

    x = currMB->cbp;
    currMB->cbp = tmp_cbp;

    rate  += writeLumaCoeff8x8_AEC(img->Coeff_all, currMB, uiBitSize, block_x, block_y, 1, uiPositionInPic);

    currMB->cbp = x;
    //calc RD and return it.
    return (double) distortion + lambda * (double) rate;
}



/*
*************************************************************************
* Function:Mode Decision for AVS intra blocks
This might also be placed in rdopt.c behind Mode_Decision_for_8x8IntraBlocks().
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int Mode_Decision_for_AVS_IntraBlocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic,
                                      int b8, double lambda, int *min_cost)
{
    int ipmode, best_ipmode, i, j, x, y;
    int c_nz, nonzero;
    double rdcost;
    int upMode;
    int leftMode;
    int mostProbableMode[2];

    double min_rdcost ;
    int incr_y = 1, off_y = 0;
    int **Coeff_for_intra_luma;//qyu 0823
    int **rec8x8;

    int block_x = (b8 & 1) << uiBitSize ;
    int block_y = (b8 >> 1) << uiBitSize;
    int pic_pix_x  = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + block_x;
    int pic_pix_y  = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + block_y;

    int pic_block_x = pic_pix_x >> MIN_BLOCK_SIZE_IN_BIT;
    int pic_block_y = pic_pix_y >> MIN_BLOCK_SIZE_IN_BIT;
    int    CandModeList[ NUM_MODE_FULL_RD ];
    double CandCostList[ NUM_MODE_FULL_RD ];
    int uiSad;
    int uiModebits;
    double uicost;
    int imode;
    if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {  //add yuqh20130824
        block_x = currMB->cuType == InNxNMB ? 0 : b8 << (uiBitSize - 1);
        block_y = currMB->cuType == InNxNMB ? b8 << (uiBitSize - 1) : 0;
        pic_pix_x  = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + block_x;
        pic_pix_y  = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + block_y;
        pic_block_x = pic_pix_x >> MIN_BLOCK_SIZE_IN_BIT;
        pic_block_y = pic_pix_y >> MIN_BLOCK_SIZE_IN_BIT;
        get_mem2Dint(&rec8x8, currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1)),
                     currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1)));      //qyu 0823
        get_mem2Dint(&Coeff_for_intra_luma, currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1)),
                     currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1)));      //qyu 0823
    } else {
        get_mem2Dint(&rec8x8, (1 << uiBitSize), (1 << uiBitSize));        //qyu 0823
        get_mem2Dint(&Coeff_for_intra_luma, (1 << uiBitSize), (1 << uiBitSize));        //qyu 0823
    }

    min_rdcost = 1e30;
    *min_cost = (1 << 20);

    upMode           = img->ipredmode[pic_block_y][  pic_block_x + 1];
    leftMode         = img->ipredmode[ pic_block_y + 1][pic_block_x ];
    upMode  = (upMode < 0) ? DC_PRED : upMode ;
    leftMode  = (leftMode < 0) ? DC_PRED : leftMode ;
    mostProbableMode[0] = min(upMode, leftMode);
    mostProbableMode[1] = max(upMode, leftMode);
    if (mostProbableMode[0] == mostProbableMode[1]) {
        mostProbableMode[0] = DC_PRED;
        mostProbableMode[1] = (mostProbableMode[1] == DC_PRED) ? BI_PRED : mostProbableMode[1];
    }



    //===== INTRA PREDICTION FOR 8x8 BLOCK =====
    intrapred_luma_AVS(currMB, uiBitSize, uiPositionInPic, pic_pix_x, pic_pix_y);

    //===== LOOP OVER ALL INTRA PREDICTION MODES =====
    for (i = 0; i < NUM_MODE_FULL_RD; i++) {
        CandModeList[ i ] = DC_PRED;
        CandCostList[ i ] = 1e30;
    }

    for (imode = 0; imode < NUM_INTRA_PMODE; imode++) {
        if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
            for (j = 0; j < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); j++) {
                for (i = 0; i < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); i++) {
                    img->resiY[j][i] = he->imgY_org[pic_pix_y + incr_y * j + off_y][pic_pix_x + i] - img->predBlockY[imode][j][i];
                }
            }

            if (uiBitSize == 2) {
                uiSad = currMB->cuType == INxnNMB ? calcHAD8x2(img->resiY, 2, 8) : calcHAD2x8(img->resiY, 8, 2);
            } else if (uiBitSize == 3) {
                uiSad = currMB->cuType == INxnNMB ? calcHAD16x4(img->resiY, 4, 16) : calcHAD4x16(img->resiY, 16, 4);
            } else if (uiBitSize == 4) {
                uiSad = currMB->cuType == INxnNMB ? calcHAD16x4(img->resiY, 8, 32) : calcHAD4x16(img->resiY, 32, 8);
            }

        } else {
            for (j = 0; j < (1 << uiBitSize); j++) {
                for (i = 0; i < (1 << uiBitSize); i++) {
                    img->resiY[j][i] = he->imgY_org[pic_pix_y + incr_y * j + off_y][pic_pix_x + i] - img->predBlockY[imode][j][i];
                }
            }
            if (uiBitSize == 2) {
                uiSad = calcHAD4x4(img->resiY, 1 << uiBitSize, 1 << uiBitSize);
            } else {
                uiSad = calcHAD(img->resiY, 1 << uiBitSize, 1 << uiBitSize);
            }
        }

        uiModebits = (mostProbableMode[0] == imode || mostProbableMode[1] == imode) ? 2 : 6;


        uicost = (double)uiSad + (double)uiModebits * lambda;
        xUpdateCandList(imode, uicost, NUM_MODE_FULL_RD, CandModeList, CandCostList);
    }

    for (imode = 0; imode < NUM_MODE_FULL_RD; imode++) {
        ipmode = CandModeList[imode];
        if (input->rdopt) {
            // get prediction and prediction error
            if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB)
                for (j = 0; j < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); j++) {
                    for (i = 0; i < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); i++) {
                        img->predBlock[block_y + j][block_x + i] = img->predBlockY[ipmode][j][i];

                        img->resiY[j][i] = he->imgY_org[pic_pix_y + incr_y * j + off_y][pic_pix_x + i] - img->predBlock[block_y + j][block_x +
                                           i];
                    }
                }
            else
                for (j = 0; j < (1 << uiBitSize); j++) {
                    for (i = 0; i < (1 << uiBitSize); i++) {
                        img->predBlock[block_y + j][block_x + i] = img->predBlockY[ipmode][j][i];

                        img->resiY[j][i] = he->imgY_org[pic_pix_y + incr_y * j + off_y][pic_pix_x + i] - img->predBlock[block_y + j][block_x +
                                           i];
                    }
                }

            //===== store the coding state =====
            store_coding_state(cs_cm);
            currMB->l_ipred_mode = currMB->real_intra_pred_modes[b8] = ipmode;
            // get and check rate-distortion cost
            if ((rdcost = RDCost_for_AVSIntraBlocks(currMB, uiBitSize, uiPositionInPic, &c_nz, b8, ipmode, lambda, min_rdcost,
                                                    mostProbableMode, 0)) < min_rdcost) {
                //--- set coefficients ---
                if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB)
                    for (y = 0; y < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); y++) {
                        for (x = 0; x < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); x++) {
                            Coeff_for_intra_luma[y][x] = img->Coeff_all[y + block_y][x + block_x];
                        }
                    }
                else
                    for (y = 0; y < (1 << uiBitSize); y++) {
                        for (x = 0; x < (1 << uiBitSize); x++) {
                            Coeff_for_intra_luma[y][x] = img->Coeff_all[y + block_y][x + block_x];
                        }
                    }

                //--- set reconstruction ---
                if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
                    for (y = 0; y < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); y++) {
                        for (x = 0; x < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); x++) {
                            rec8x8[y][x] = hc->imgY[pic_pix_y + incr_y * y + off_y][pic_pix_x + x];
                        }
                    }

                } else
                    for (y = 0; y < (1 << uiBitSize); y++) {
                        for (x = 0; x < (1 << uiBitSize); x++) {
                            rec8x8[y][x] = hc->imgY[pic_pix_y + incr_y * y + off_y][pic_pix_x + x];
                        }
                    }

                //--- flag if dct-coefficients must be coded ---
                nonzero = c_nz;

                //--- set best mode update minimum cost ---
                min_rdcost  = rdcost;
                *min_cost = (int) min_rdcost;
                best_ipmode = ipmode;
            }

            reset_coding_state(cs_cm);
        }
    }

    //===== set intra mode prediction =====

    if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
        for (j = 0;
             j < (currMB->cuType == InNxNMB ?  1 << (uiBitSize - 1 - MIN_BLOCK_SIZE_IN_BIT) : 1 <<
                  (uiBitSize + 1 - MIN_BLOCK_SIZE_IN_BIT)); j++) {
            for (i = 0;
                 i < (currMB->cuType == INxnNMB ?  1 << (uiBitSize - 1 - MIN_BLOCK_SIZE_IN_BIT)  : 1 <<
                      (uiBitSize + 1 - MIN_BLOCK_SIZE_IN_BIT)); i++) {
                img->ipredmode[pic_block_y + 1 + j][pic_block_x + 1 + i] = best_ipmode; //qyu 0829
            }
        }
    } else
        for (j = 0; j < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); j++) {
            for (i = 0; i < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); i++) {
                img->ipredmode[pic_block_y + 1 + j][pic_block_x + 1 + i] = best_ipmode; //qyu 0829
            }
        }



    currMB->intra_pred_modes[b8] = (mostProbableMode[0] == best_ipmode) ? -2 : ((mostProbableMode[1] == best_ipmode) ?
                                   -1 : (best_ipmode < mostProbableMode[0] ? best_ipmode : (best_ipmode < mostProbableMode[1] ? best_ipmode - 1 :
                                           best_ipmode - 2)));

    currMB->real_intra_pred_modes[b8] = best_ipmode;
    /*lgp*/
    if (input->rdopt) {
        /*lgp*/
        //===== restore coefficients =====
        if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
            for (j = 0; j < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); j++) {
                for (i = 0; i < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); i++) {
                    img->Coeff_all[j + block_y][i + block_x] = Coeff_for_intra_luma[j][i];
                }
            }
        } else
            for (j = 0; j < (1 << uiBitSize); j++) {
                for (i = 0; i < (1 << uiBitSize); i++) {
                    img->Coeff_all[j + block_y][i + block_x] = Coeff_for_intra_luma[j][i];
                }
            }

        //--- set reconstruction ---
        if (currMB->cuType == InNxNMB || currMB->cuType == INxnNMB) {
            for (y = 0; y < (currMB->cuType == InNxNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); y++) {
                for (x = 0; x < (currMB->cuType == INxnNMB ? (1 << (uiBitSize - 1)) : (1 << (uiBitSize + 1))); x++) {
                    hc->imgY[pic_pix_y + incr_y * y + off_y][pic_pix_x + x] = rec8x8[y][x];
                }
            }
        } else
            for (y = 0; y < (1 <<  uiBitSize); y++) {
                for (x = 0; x < (1 << uiBitSize); x++) {
                    hc->imgY[pic_pix_y + incr_y * y + off_y][pic_pix_x + x] = rec8x8[y][x];
                }
            }
    }/*lgp*/

    free_mem2Dint(rec8x8);
    free_mem2Dint(Coeff_for_intra_luma);
    return nonzero;
}

int calcHAD(int **pi, int iWidth, int iHeight)
{
    int uiSum = 0, sad, sum_sad = 0;
    int x, y, i, j, k;
    int diff[64], m1[8][8], m2[8][8], m3[8][8];

    // Hadamard8x8
    for (y = 0; y < iHeight; y += 8) {
        for (x = 0; x < iWidth; x += 8) {
            sad = 0;

            for (j = 0; j < 8; j++) {
                for (i = 0; i < 8; i++) {
                    diff[j * 8 + i] = pi[y + j][x + i];
                }
            }

            //horizontal
            for (j = 0; j < 8; j++) {
                k = j << 3;
                m2[j][0] = diff[k  ] + diff[k + 4];
                m2[j][1] = diff[k + 1] + diff[k + 5];
                m2[j][2] = diff[k + 2] + diff[k + 6];
                m2[j][3] = diff[k + 3] + diff[k + 7];
                m2[j][4] = diff[k  ] - diff[k + 4];
                m2[j][5] = diff[k + 1] - diff[k + 5];
                m2[j][6] = diff[k + 2] - diff[k + 6];
                m2[j][7] = diff[k + 3] - diff[k + 7];

                m1[j][0] = m2[j][0] + m2[j][2];
                m1[j][1] = m2[j][1] + m2[j][3];
                m1[j][2] = m2[j][0] - m2[j][2];
                m1[j][3] = m2[j][1] - m2[j][3];
                m1[j][4] = m2[j][4] + m2[j][6];
                m1[j][5] = m2[j][5] + m2[j][7];
                m1[j][6] = m2[j][4] - m2[j][6];
                m1[j][7] = m2[j][5] - m2[j][7];

                m2[j][0] = m1[j][0] + m1[j][1];
                m2[j][1] = m1[j][0] - m1[j][1];
                m2[j][2] = m1[j][2] + m1[j][3];
                m2[j][3] = m1[j][2] - m1[j][3];
                m2[j][4] = m1[j][4] + m1[j][5];
                m2[j][5] = m1[j][4] - m1[j][5];
                m2[j][6] = m1[j][6] + m1[j][7];
                m2[j][7] = m1[j][6] - m1[j][7];
            }

            //vertical
            for (i = 0; i < 8; i++) {
                m3[0][i] = m2[0][i] + m2[4][i];
                m3[1][i] = m2[1][i] + m2[5][i];
                m3[2][i] = m2[2][i] + m2[6][i];
                m3[3][i] = m2[3][i] + m2[7][i];
                m3[4][i] = m2[0][i] - m2[4][i];
                m3[5][i] = m2[1][i] - m2[5][i];
                m3[6][i] = m2[2][i] - m2[6][i];
                m3[7][i] = m2[3][i] - m2[7][i];

                m1[0][i] = m3[0][i] + m3[2][i];
                m1[1][i] = m3[1][i] + m3[3][i];
                m1[2][i] = m3[0][i] - m3[2][i];
                m1[3][i] = m3[1][i] - m3[3][i];
                m1[4][i] = m3[4][i] + m3[6][i];
                m1[5][i] = m3[5][i] + m3[7][i];
                m1[6][i] = m3[4][i] - m3[6][i];
                m1[7][i] = m3[5][i] - m3[7][i];

                m2[0][i] = m1[0][i] + m1[1][i];
                m2[1][i] = m1[0][i] - m1[1][i];
                m2[2][i] = m1[2][i] + m1[3][i];
                m2[3][i] = m1[2][i] - m1[3][i];
                m2[4][i] = m1[4][i] + m1[5][i];
                m2[5][i] = m1[4][i] - m1[5][i];
                m2[6][i] = m1[6][i] + m1[7][i];
                m2[7][i] = m1[6][i] - m1[7][i];
            }

            for (j = 0; j < 8; j++) {
                for (i = 0; i < 8; i++) {
                    sad += (abs(m2[j][i]));
                }
            }

            sad = ((sad + 2) >> 2);

            sum_sad += sad;
        }
    }
    return sum_sad;
}
int calcHAD4x4(int **pi, int iWidth, int iHeight)
{
    int uiSum = 0, sad, sum_sad = 0;
    int x, y, i, j;
    int  diff[16], m[16], d[16];

    // Hadamard8x8
    for (y = 0; y < iHeight; y += 4) {
        for (x = 0; x < iWidth; x += 4) {
            sad = 0;

            for (j = 0; j < 4; j++) {
                for (i = 0; i < 4; i++) {
                    diff[j * 4 + i] = pi[y + j][x + i];
                }
            }
            /*===== hadamard transform =====*/
            m[ 0] = diff[ 0] + diff[12];
            m[ 1] = diff[ 1] + diff[13];
            m[ 2] = diff[ 2] + diff[14];
            m[ 3] = diff[ 3] + diff[15];
            m[ 4] = diff[ 4] + diff[ 8];
            m[ 5] = diff[ 5] + diff[ 9];
            m[ 6] = diff[ 6] + diff[10];
            m[ 7] = diff[ 7] + diff[11];
            m[ 8] = diff[ 4] - diff[ 8];
            m[ 9] = diff[ 5] - diff[ 9];
            m[10] = diff[ 6] - diff[10];
            m[11] = diff[ 7] - diff[11];
            m[12] = diff[ 0] - diff[12];
            m[13] = diff[ 1] - diff[13];
            m[14] = diff[ 2] - diff[14];
            m[15] = diff[ 3] - diff[15];

            d[ 0] = m[ 0] + m[ 4];
            d[ 1] = m[ 1] + m[ 5];
            d[ 2] = m[ 2] + m[ 6];
            d[ 3] = m[ 3] + m[ 7];
            d[ 4] = m[ 8] + m[12];
            d[ 5] = m[ 9] + m[13];
            d[ 6] = m[10] + m[14];
            d[ 7] = m[11] + m[15];
            d[ 8] = m[ 0] - m[ 4];
            d[ 9] = m[ 1] - m[ 5];
            d[10] = m[ 2] - m[ 6];
            d[11] = m[ 3] - m[ 7];
            d[12] = m[12] - m[ 8];
            d[13] = m[13] - m[ 9];
            d[14] = m[14] - m[10];
            d[15] = m[15] - m[11];

            m[ 0] = d[ 0] + d[ 3];
            m[ 1] = d[ 1] + d[ 2];
            m[ 2] = d[ 1] - d[ 2];
            m[ 3] = d[ 0] - d[ 3];
            m[ 4] = d[ 4] + d[ 7];
            m[ 5] = d[ 5] + d[ 6];
            m[ 6] = d[ 5] - d[ 6];
            m[ 7] = d[ 4] - d[ 7];
            m[ 8] = d[ 8] + d[11];
            m[ 9] = d[ 9] + d[10];
            m[10] = d[ 9] - d[10];
            m[11] = d[ 8] - d[11];
            m[12] = d[12] + d[15];
            m[13] = d[13] + d[14];
            m[14] = d[13] - d[14];
            m[15] = d[12] - d[15];

            d[ 0] = m[ 0] + m[ 1];
            d[ 1] = m[ 0] - m[ 1];
            d[ 2] = m[ 2] + m[ 3];
            d[ 3] = m[ 3] - m[ 2];
            d[ 4] = m[ 4] + m[ 5];
            d[ 5] = m[ 4] - m[ 5];
            d[ 6] = m[ 6] + m[ 7];
            d[ 7] = m[ 7] - m[ 6];
            d[ 8] = m[ 8] + m[ 9];
            d[ 9] = m[ 8] - m[ 9];
            d[10] = m[10] + m[11];
            d[11] = m[11] - m[10];
            d[12] = m[12] + m[13];
            d[13] = m[12] - m[13];
            d[14] = m[14] + m[15];
            d[15] = m[15] - m[14];


            for (j = 0; j < 16; j++) {
                sad += (abs(d[j]));
            }

            sad = ((sad + 1) >> 1);

            sum_sad += sad;
        }
    }
    return sum_sad;
}
int calcHAD8x2(int **pi, int iWidth, int iHeight)
{
    int uiSum = 0, sad, sum_sad = 0;
    int x, y, i, j, jj;
    int diff[257], m1[2][8], m2[2][8];

    // Hadamard8x8
    for (y = 0; y < iHeight; y += 8) {
        for (x = 0; x < iWidth; x += 2) {
            sad = 0;

            for (j = 0; j < 8; j++) {
                for (i = 0; i < 2; i++) {
                    diff[j * 2 + i] = pi[y + j][x + i];
                }
            }

            //horizontal
            for (j = 0; j < 2; j++) {
                jj = j << 3;
                m2[j][0] = diff[jj  ] + diff[jj + 4];
                m2[j][1] = diff[jj + 1] + diff[jj + 5];
                m2[j][2] = diff[jj + 2] + diff[jj + 6];
                m2[j][3] = diff[jj + 3] + diff[jj + 7];
                m2[j][4] = diff[jj  ] - diff[jj + 4];
                m2[j][5] = diff[jj + 1] - diff[jj + 5];
                m2[j][6] = diff[jj + 2] - diff[jj + 6];
                m2[j][7] = diff[jj + 3] - diff[jj + 7];

                m1[j][0] = m2[j][0] + m2[j][2];
                m1[j][1] = m2[j][1] + m2[j][3];
                m1[j][2] = m2[j][0] - m2[j][2];
                m1[j][3] = m2[j][1] - m2[j][3];
                m1[j][4] = m2[j][4] + m2[j][6];
                m1[j][5] = m2[j][5] + m2[j][7];
                m1[j][6] = m2[j][4] - m2[j][6];
                m1[j][7] = m2[j][5] - m2[j][7];

                m2[j][0] = m1[j][0] + m1[j][1];
                m2[j][1] = m1[j][0] - m1[j][1];
                m2[j][2] = m1[j][2] + m1[j][3];
                m2[j][3] = m1[j][2] - m1[j][3];
                m2[j][4] = m1[j][4] + m1[j][5];
                m2[j][5] = m1[j][4] - m1[j][5];
                m2[j][6] = m1[j][6] + m1[j][7];
                m2[j][7] = m1[j][6] - m1[j][7];
            }

            //vertical
            for (i = 0; i < 8; i++) {
                m1[0][i] = m2[0][i] + m2[1][i];
                m1[1][i] = m2[0][i] - m2[1][i];
            }

            for (j = 0; j < 2; j++) {
                for (i = 0; i < 8; i++) {
                    sad += (abs(m1[j][i]));
                }
            }

            sad = ((sad + 1) >> 1);

            sum_sad += sad;
        }
    }
    return sum_sad;
}
int calcHAD2x8(int **pi, int iWidth, int iHeight)
{
    int uiSum = 0, sad, sum_sad = 0;
    int x, y, i, j, jj;
    int diff[257], m1[8][2], m2[8][2];

    // Hadamard8x8
    for (y = 0; y < iHeight; y += 2) {
        for (x = 0; x < iWidth; x += 8) {
            sad = 0;

            for (j = 0; j < 2; j++) {
                for (i = 0; i < 8; i++) {
                    diff[j * 8 + i] = pi[y + j][x + i];
                }
            }


            //horizontal
            for (j = 0; j < 8; j++) {
                jj = j << 1;
                m1[j][0] = diff[jj] + diff[jj + 1];
                m1[j][1] = diff[jj] - diff[jj + 1];
            }

            //vertical
            for (i = 0; i < 2; i++) {
                m2[0][i] = m1[0][i] + m1[4][i];
                m2[1][i] = m1[1][i] + m1[5][i];
                m2[2][i] = m1[2][i] + m1[6][i];
                m2[3][i] = m1[3][i] + m1[7][i];
                m2[4][i] = m1[0][i] - m1[4][i];
                m2[5][i] = m1[1][i] - m1[5][i];
                m2[6][i] = m1[2][i] - m1[6][i];
                m2[7][i] = m1[3][i] - m1[7][i];

                m1[0][i] = m2[0][i] + m2[2][i];
                m1[1][i] = m2[1][i] + m2[3][i];
                m1[2][i] = m2[0][i] - m2[2][i];
                m1[3][i] = m2[1][i] - m2[3][i];
                m1[4][i] = m2[4][i] + m2[6][i];
                m1[5][i] = m2[5][i] + m2[7][i];
                m1[6][i] = m2[4][i] - m2[6][i];
                m1[7][i] = m2[5][i] - m2[7][i];

                m2[0][i] = m1[0][i] + m1[1][i];
                m2[1][i] = m1[0][i] - m1[1][i];
                m2[2][i] = m1[2][i] + m1[3][i];
                m2[3][i] = m1[2][i] - m1[3][i];
                m2[4][i] = m1[4][i] + m1[5][i];
                m2[5][i] = m1[4][i] - m1[5][i];
                m2[6][i] = m1[6][i] + m1[7][i];
                m2[7][i] = m1[6][i] - m1[7][i];
            }

            for (j = 0; j < 8; j++) {
                for (i = 0; i < 2; i++) {
                    sad += (abs(m2[j][i]));
                }
            }

            sad = ((sad + 1) >> 1);

            sum_sad += sad;
        }
    }
    return sum_sad;
}

int calcHAD16x4(int **pi, int iWidth, int iHeight)
{
    int uiSum = 0, sad, sum_sad = 0;
    int x, y, i, j, jj;
    int diff[257], m1[4][16], m2[4][16];

    // Hadamard8x8
    for (y = 0; y < iHeight; y += 16) {
        for (x = 0; x < iWidth; x += 4) {
            sad = 0;

            for (j = 0; j < 16; j++) {
                for (i = 0; i < 4; i++) {
                    diff[j * 4 + i] = pi[y + j][x + i];
                }
            }


            //horizontal
            for (j = 0; j < 4; j++) {
                jj = j << 4;

                m2[j][0]  = diff[jj  ] + diff[jj + 8];
                m2[j][1]  = diff[jj + 1] + diff[jj + 9];
                m2[j][2]  = diff[jj + 2] + diff[jj + 10];
                m2[j][3]  = diff[jj + 3] + diff[jj + 11];
                m2[j][4]  = diff[jj + 4] + diff[jj + 12];
                m2[j][5]  = diff[jj + 5] + diff[jj + 13];
                m2[j][6]  = diff[jj + 6] + diff[jj + 14];
                m2[j][7]  = diff[jj + 7] + diff[jj + 15];
                m2[j][8]  = diff[jj  ] - diff[jj + 8];
                m2[j][9]  = diff[jj + 1] - diff[jj + 9];
                m2[j][10] = diff[jj + 2] - diff[jj + 10];
                m2[j][11] = diff[jj + 3] - diff[jj + 11];
                m2[j][12] = diff[jj + 4] - diff[jj + 12];
                m2[j][13] = diff[jj + 5] - diff[jj + 13];
                m2[j][14] = diff[jj + 6] - diff[jj + 14];
                m2[j][15] = diff[jj + 7] - diff[jj + 15];

                m1[j][0]  = m2[j][0]  + m2[j][4];
                m1[j][1]  = m2[j][1]  + m2[j][5];
                m1[j][2]  = m2[j][2]  + m2[j][6];
                m1[j][3]  = m2[j][3]  + m2[j][7];
                m1[j][4]  = m2[j][0]  - m2[j][4];
                m1[j][5]  = m2[j][1]  - m2[j][5];
                m1[j][6]  = m2[j][2]  - m2[j][6];
                m1[j][7]  = m2[j][3]  - m2[j][7];
                m1[j][8]  = m2[j][8]  + m2[j][12];
                m1[j][9]  = m2[j][9]  + m2[j][13];
                m1[j][10] = m2[j][10] + m2[j][14];
                m1[j][11] = m2[j][11] + m2[j][15];
                m1[j][12] = m2[j][8]  - m2[j][12];
                m1[j][13] = m2[j][9]  - m2[j][13];
                m1[j][14] = m2[j][10] - m2[j][14];
                m1[j][15] = m2[j][11] - m2[j][15];

                m2[j][0]  = m1[j][0]  + m1[j][2];
                m2[j][1]  = m1[j][1]  + m1[j][3];
                m2[j][2]  = m1[j][0]  - m1[j][2];
                m2[j][3]  = m1[j][1]  - m1[j][3];
                m2[j][4]  = m1[j][4]  + m1[j][6];
                m2[j][5]  = m1[j][5]  + m1[j][7];
                m2[j][6]  = m1[j][4]  - m1[j][6];
                m2[j][7]  = m1[j][5]  - m1[j][7];
                m2[j][8]  = m1[j][8]  + m1[j][10];
                m2[j][9]  = m1[j][9]  + m1[j][11];
                m2[j][10] = m1[j][8]  - m1[j][10];
                m2[j][11] = m1[j][9]  - m1[j][11];
                m2[j][12] = m1[j][12] + m1[j][14];
                m2[j][13] = m1[j][13] + m1[j][15];
                m2[j][14] = m1[j][12] - m1[j][14];
                m2[j][15] = m1[j][13] - m1[j][15];

                m1[j][0]  = m2[j][0]  + m2[j][1];
                m1[j][1]  = m2[j][0]  - m2[j][1];
                m1[j][2]  = m2[j][2]  + m2[j][3];
                m1[j][3]  = m2[j][2]  - m2[j][3];
                m1[j][4]  = m2[j][4]  + m2[j][5];
                m1[j][5]  = m2[j][4]  - m2[j][5];
                m1[j][6]  = m2[j][6]  + m2[j][7];
                m1[j][7]  = m2[j][6]  - m2[j][7];
                m1[j][8]  = m2[j][8]  + m2[j][9];
                m1[j][9]  = m2[j][8]  - m2[j][9];
                m1[j][10] = m2[j][10] + m2[j][11];
                m1[j][11] = m2[j][10] - m2[j][11];
                m1[j][12] = m2[j][12] + m2[j][13];
                m1[j][13] = m2[j][12] - m2[j][13];
                m1[j][14] = m2[j][14] + m2[j][15];
                m1[j][15] = m2[j][14] - m2[j][15];
            }

            //vertical
            for (i = 0; i < 16; i++) {
                m2[0][i] = m1[0][i] + m1[2][i];
                m2[1][i] = m1[1][i] + m1[3][i];
                m2[2][i] = m1[0][i] - m1[2][i];
                m2[3][i] = m1[1][i] - m1[3][i];

                m1[0][i] = m2[0][i] + m2[1][i];
                m1[1][i] = m2[0][i] - m2[1][i];
                m1[2][i] = m2[2][i] + m2[3][i];
                m1[3][i] = m2[2][i] - m2[3][i];
            }


            for (j = 0; j < 4; j++) {
                for (i = 0; i < 16; i++) {
                    sad += (abs(m1[j][i]));
                }
            }

            sad = ((sad + 2) >> 2);

            sum_sad += sad;
        }
    }
    return sum_sad;
}
int calcHAD4x16(int **pi, int iWidth, int iHeight)
{
    int uiSum = 0, sad, sum_sad = 0;
    int x, y, i, j, jj;
    int diff[257], m1[16][4], m2[16][4], m3[16][4];

    // Hadamard8x8
    for (y = 0; y < iHeight; y += 4) {
        for (x = 0; x < iWidth; x += 16) {
            sad = 0;

            for (j = 0; j < 4; j++) {
                for (i = 0; i < 16; i++) {
                    diff[j * 16 + i] = pi[y + j][x + i];
                }
            }

            //horizontal
            for (j = 0; j < 16; j++) {
                jj = j << 2;
                m2[j][0] = diff[jj  ] + diff[jj + 2];
                m2[j][1] = diff[jj + 1] + diff[jj + 3];
                m2[j][2] = diff[jj  ] - diff[jj + 2];
                m2[j][3] = diff[jj + 1] - diff[jj + 3];

                m1[j][0] = m2[j][0] + m2[j][1];
                m1[j][1] = m2[j][0] - m2[j][1];
                m1[j][2] = m2[j][2] + m2[j][3];
                m1[j][3] = m2[j][2] - m2[j][3];
            }

            //vertical
            for (i = 0; i < 4; i++) {
                m2[0][i]  = m1[0][i] + m1[8][i];
                m2[1][i]  = m1[1][i] + m1[9][i];
                m2[2][i]  = m1[2][i] + m1[10][i];
                m2[3][i]  = m1[3][i] + m1[11][i];
                m2[4][i]  = m1[4][i] + m1[12][i];
                m2[5][i]  = m1[5][i] + m1[13][i];
                m2[6][i]  = m1[6][i] + m1[14][i];
                m2[7][i]  = m1[7][i] + m1[15][i];
                m2[8][i]  = m1[0][i] - m1[8][i];
                m2[9][i]  = m1[1][i] - m1[9][i];
                m2[10][i] = m1[2][i] - m1[10][i];
                m2[11][i] = m1[3][i] - m1[11][i];
                m2[12][i] = m1[4][i] - m1[12][i];
                m2[13][i] = m1[5][i] - m1[13][i];
                m2[14][i] = m1[6][i] - m1[14][i];
                m2[15][i] = m1[7][i] - m1[15][i];

                m3[0][i]  = m2[0][i]  + m2[4][i];
                m3[1][i]  = m2[1][i]  + m2[5][i];
                m3[2][i]  = m2[2][i]  + m2[6][i];
                m3[3][i]  = m2[3][i]  + m2[7][i];
                m3[4][i]  = m2[0][i]  - m2[4][i];
                m3[5][i]  = m2[1][i]  - m2[5][i];
                m3[6][i]  = m2[2][i]  - m2[6][i];
                m3[7][i]  = m2[3][i]  - m2[7][i];
                m3[8][i]  = m2[8][i]  + m2[12][i];
                m3[9][i]  = m2[9][i]  + m2[13][i];
                m3[10][i] = m2[10][i] + m2[14][i];
                m3[11][i] = m2[11][i] + m2[15][i];
                m3[12][i] = m2[8][i]  - m2[12][i];
                m3[13][i] = m2[9][i]  - m2[13][i];
                m3[14][i] = m2[10][i] - m2[14][i];
                m3[15][i] = m2[11][i] - m2[15][i];

                m1[0][i]  = m3[0][i]  + m3[2][i];
                m1[1][i]  = m3[1][i]  + m3[3][i];
                m1[2][i]  = m3[0][i]  - m3[2][i];
                m1[3][i]  = m3[1][i]  - m3[3][i];
                m1[4][i]  = m3[4][i]  + m3[6][i];
                m1[5][i]  = m3[5][i]  + m3[7][i];
                m1[6][i]  = m3[4][i]  - m3[6][i];
                m1[7][i]  = m3[5][i]  - m3[7][i];
                m1[8][i]  = m3[8][i]  + m3[10][i];
                m1[9][i]  = m3[9][i]  + m3[11][i];
                m1[10][i] = m3[8][i]  - m3[10][i];
                m1[11][i] = m3[9][i]  - m3[11][i];
                m1[12][i] = m3[12][i] + m3[14][i];
                m1[13][i] = m3[13][i] + m3[15][i];
                m1[14][i] = m3[12][i] - m3[14][i];
                m1[15][i] = m3[13][i] - m3[15][i];

                m2[0][i]  = m1[0][i]  + m1[1][i];
                m2[1][i]  = m1[0][i]  - m1[1][i];
                m2[2][i]  = m1[2][i]  + m1[3][i];
                m2[3][i]  = m1[2][i]  - m1[3][i];
                m2[4][i]  = m1[4][i]  + m1[5][i];
                m2[5][i]  = m1[4][i]  - m1[5][i];
                m2[6][i]  = m1[6][i]  + m1[7][i];
                m2[7][i]  = m1[6][i]  - m1[7][i];
                m2[8][i]  = m1[8][i]  + m1[9][i];
                m2[9][i]  = m1[8][i]  - m1[9][i];
                m2[10][i] = m1[10][i] + m1[11][i];
                m2[11][i] = m1[10][i] - m1[11][i];
                m2[12][i] = m1[12][i] + m1[13][i];
                m2[13][i] = m1[12][i] - m1[13][i];
                m2[14][i] = m1[14][i] + m1[15][i];
                m2[15][i] = m1[14][i] - m1[15][i];
            }


            for (j = 0; j < 16; j++) {
                for (i = 0; i < 4; i++) {
                    sad += (abs(m2[j][i]));
                }
            }

            sad = ((sad + 2) >> 2);

            sum_sad += sad;
        }
    }
    return sum_sad;
}

void xUpdateCandList(int uiMode, double uiCost, int uiFullCandNum, int *CandModeList, double *CandCostList)
{
    int i;
    int shift = 0;

    while (shift < uiFullCandNum && uiCost < CandCostList[ uiFullCandNum - 1 - shift ]) {
        shift++;
    }

    if (shift != 0) {
        for (i = 1; i < shift; i++) {
            CandModeList[ uiFullCandNum - i ] = CandModeList[ uiFullCandNum - 1 - i ];
            CandCostList[ uiFullCandNum - i ] = CandCostList[ uiFullCandNum - 1 - i ];
        }
        CandModeList[ uiFullCandNum - shift ] = uiMode;
        CandCostList[ uiFullCandNum - shift ] = uiCost;
        return ;
    }
    return ;
}
/*
*************************************************************************
* Function:Mode Decision for an 8x8 Intra block
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int Mode_Decision_for_NxNIntraBlocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int b8,
                                     double lambda, int *cost)
{
    int  nonzero = 0;
    int  cost8x8;
    int loc_cost, best_AVS_cost;
    int best_nonzero = 0;

    best_AVS_cost = 0x7FFFFFFFL; //max int value

    loc_cost = (int) floor(6.0 * lambda + 0.4999);

    if (Mode_Decision_for_AVS_IntraBlocks(currMB, uiBitSize, uiPositionInPic, b8, lambda, &cost8x8)) {
        nonzero = 1;
    }

    loc_cost += cost8x8;

    if (loc_cost < best_AVS_cost) {
        //is better than last (or ist first try).
        best_AVS_cost = loc_cost;
        best_nonzero = nonzero;
    }

    *cost = best_AVS_cost;

    return best_nonzero;
}


/*
*************************************************************************
* Function:8x8 Intra mode decision for an codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int Mode_Decision_for_IntracodingUnit(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic,
                                      double lambda,  int *cost)
{
    int  cbp = 0, b8, cost8x8;
    int  stage_block8x8_pos = 0;
    *cost = 0;
    if (currMB->trans_size == 1) {
        for (b8 = stage_block8x8_pos; b8 < 4; b8++) {
            if (Mode_Decision_for_NxNIntraBlocks(currMB, uiBitSize - 1, uiPositionInPic, b8, lambda, &cost8x8)) {
                cbp |= (1 << b8);
            }
            *cost += cost8x8;
        }
    } else {
        if (Mode_Decision_for_NxNIntraBlocks(currMB, uiBitSize, uiPositionInPic, 0, lambda, &cost8x8)) {
            cbp = 15;
        }
        *cost += cost8x8;
    }

    return cbp;
}


/*
*************************************************************************
* Function:R-D Cost for an 8x8 Partition
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

double RDCost_for_8x8blocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cnt_nonz,
                            int *cbp_blk, double lambda, int block, int mode, int pdir, int ref, int bwd_ref)  // <-- abp type
{
    int  i, j;
    int  rate = 0, distortion = 0;
    int  fw_mode, bw_mode;
    int  cbp     = 0;
    int  pax     = (block % 2) << uiBitSize;
    int  pay     = (block / 2) << uiBitSize;
    int  i0      = pax >> uiBitSize;
    int  j0      = pay >> uiBitSize;
    int  bframe  = (img->type == B_IMG);
    int  direct  = (bframe && mode == PSKIPDIRECT);
    int  b8value = mode == PNXN ? (1 + (pdir == BID ? 6 : pdir)) : 0;


    SyntaxElement *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    int  **frefarr = hc->refFrArr;  // For MB level field/frame
    int  block_8y = (uiPositionInPic / img->PicWidthInMbs) << 1;
    int  block_8x = (uiPositionInPic % img->PicWidthInMbs) << 1;
    int pix_y = block_8y << MIN_BLOCK_SIZE_IN_BIT;
    int pix_x = block_8x << MIN_BLOCK_SIZE_IN_BIT;
    byte **imgY_original = he->imgY_org;
    int  incr_y = 1, off_y = 0; /*lgp*/

    Slice         *currSlice  = img->currentSlice;
    DataPartition *dataPart;

    MCParam cMCParam;
    cMCParam.num_skipmode = 0;
    cMCParam.dmh_mode = 0;
    cMCParam.dir = 0;
    currMB->md_directskip_mode = 0;
    currMB->cuType = PNXN;
    if (img->type == INTER_IMG || img->type == F_IMG) {
        assert(mode != 0);
    }
    //=====  GET COEFFICIENTS, RECONSTRUCTIONS, CBP
    if (direct) {
#if Mv_Rang
        *cnt_nonz = LumaResidualCoding8x8(currMB, uiBitSize, uiPositionInPic, &cbp, cbp_blk, block, mode, 0, 0,
                                          max(0, frefarr[block_8y + (pay >> MIN_BLOCK_SIZE_IN_BIT) ][block_8x + (pax >> MIN_BLOCK_SIZE_IN_BIT) ]), 0,
                                          &cMCParam);  /////////qyu now
#else
        *cnt_nonz = LumaResidualCoding8x8(currMB, uiBitSize, uiPositionInPic, &cbp, cbp_blk, block, 0, 0,
                                          max(0, frefarr[block_8y + (pay >> MIN_BLOCK_SIZE_IN_BIT) ][block_8x + (pax >> MIN_BLOCK_SIZE_IN_BIT) ]), 0,
                                          &cMCParam);  /////////qyu now
#endif
    } else {
        fw_mode   = (pdir == FORWARD || pdir == SYM ? mode : 0);
        bw_mode   = (pdir == BACKWARD || pdir == SYM ? mode : 0);

        fw_mode = (pdir == BID ? -mode : fw_mode);
        bw_mode = (pdir == BID ? -mode : bw_mode);
#if Mv_Rang
        *cnt_nonz = LumaResidualCoding8x8(currMB, uiBitSize, uiPositionInPic, &cbp, cbp_blk, block, mode, fw_mode, bw_mode, ref,
                                          bwd_ref, &cMCParam);
#else
        *cnt_nonz = LumaResidualCoding8x8(currMB, uiBitSize, uiPositionInPic, &cbp, cbp_blk, block, fw_mode, bw_mode, ref,
                                          bwd_ref, &cMCParam);
#endif
    }

    for (j = 0; j < (1 << uiBitSize); j++) {     /*lgp*/
        for (i = pax; i < pax + (1 << uiBitSize); i++) {
            distortion += img->quad[imgY_original[pix_y + pay + incr_y * j + off_y][pix_x + i] - hc->imgY[pix_y + pay + incr_y * j +
                                    off_y][pix_x + i]];
        }
    }

    //=====   GET RATE
    //----- block 8x8 mode -----

    currSE->value1 = currSE->bitpattern = b8value;
    dataPart = & (currSlice->partArr[0]);
    currSE->writing = writeB8TypeInfo;
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
    rate += currSE->len;

    //----- motion information -----
    if (!direct) {
        /*  if ( pdir == 0 || pdir == 2 )
          {
            rate  += writeMotionVector8x8 ( currMB, uiBitSize + 1, uiPositionInPic, i0, j0, i0 + 1, j0 + 1, ref, 0, 1, mode );
          }

          if ( pdir == 1 || pdir == 2 )
          {
            rate  += writeMotionVector8x8 ( currMB, uiBitSize + 1, uiPositionInPic, i0, j0, i0 + 1, j0 + 1, bwd_ref, 0, 0, mode );
          }*/
        if (pdir == FORWARD) {
            rate  += writeMotionVector8x8(currMB, uiBitSize + 1, uiPositionInPic, i0, j0, i0 + 1, j0 + 1, ref, 0, 1, mode);
        }
        if (pdir == BACKWARD) {
            rate  += writeMotionVector8x8(currMB, uiBitSize + 1, uiPositionInPic, i0, j0, i0 + 1, j0 + 1, bwd_ref, 0, 0, mode);
        }
        if (pdir == SYM) {
            rate  += writeMotionVector8x8_sym(i0, j0, i0 + 1, j0 + 1, ref, 0, 1, mode, pdir, currMB, uiPositionInPic);
        }

        if (pdir == BID) {
            rate  += writeMotionVector8x8_sym(i0, j0, i0 + 1, j0 + 1, ref, 0, 1, mode, pdir, currMB, uiPositionInPic);
            rate  += writeMotionVector8x8_sym(i0, j0, i0 + 1, j0 + 1, bwd_ref, 0, 0, mode, pdir, currMB, uiPositionInPic);
        }

    }

    //----- luminance coefficients -----
    if (*cnt_nonz) {
        currMB->cbp = cbp;
        currMB->l_ipred_mode = currMB->real_intra_pred_modes[block];
        rate += writeLumaCoeff8x8(img->Coeff_all, currMB, uiBitSize, pax, pay, block, 0,
                                  uiPositionInPic);  //block_8x,block_8y

    }

    return (double) distortion + lambda * (double) rate;
}


/*
*************************************************************************
* Function:Sets modes and reference frames for an codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void SetModesAndRefframeForBlocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int mode,
                                  MCParam *pMCParam)
{
    int i, j, k;
    int  bframe  = (img->type == B_IMG);
    int  **fwrefarr = img->fw_refFrArr;
    int  **bwrefarr = img->bw_refFrArr;
    int  **frefarr = hc->refFrArr;
    int  block_8y = (uiPositionInPic / img->PicWidthInMbs) << 1;   //img->block8_y;
    int  block_8x = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;
    int r, c, row, col, step_h, step_v;
    int  stage_block8x8_pos = 0; /*lgp*/

    //--- codingUnit type ---
    currMB->cuType = mode;

    //--- block 8x8 mode and prediction direction ---
    switch (mode) {
    case PSKIPDIRECT:
        if (pMCParam->dir == 0 || pMCParam->dir == DS_SYM) {
            for (i = stage_block8x8_pos; i < 4; i++) {
                currMB->b8mode[i] = 0;
                currMB->b8pdir[i] = (bframe ? SYM : 0);
            }
        } else if (pMCParam->dir == DS_BID) {
            for (i = stage_block8x8_pos; i < 4; i++) {
                currMB->b8mode[i] = 0;
                currMB->b8pdir[i] = (bframe ? BID : 0);
            }
        } else if (pMCParam->dir == DS_BACKWARD) {
            for (i = stage_block8x8_pos; i < 4; i++) {
                currMB->b8mode[i] = 0;
                currMB->b8pdir[i] = (bframe ? BACKWARD : 0);
            }
        } else if (pMCParam->dir == DS_FORWARD) {
            for (i = stage_block8x8_pos; i < 4; i++) {
                currMB->b8mode[i] = 0;
                currMB->b8pdir[i] = (bframe ? FORWARD : 0);
            }
        }

        break;
    case P2NX2N:
    case P2NXN:
    case PNX2N:
    case PHOR_UP:
    case PHOR_DOWN:
    case PVER_LEFT:
    case PVER_RIGHT:


        for (i = stage_block8x8_pos/*lgp*/; i < 4; i++) {
            currMB->b8mode[i] = mode;
            currMB->b8pdir[i] = best8x8pdir[mode][i];
        }

        break;
    //case 4:
    case PNXN:

        for (i = stage_block8x8_pos/*lgp*/; i < 4; i++) {
            currMB->b8mode[i]   = best8x8mode[i];
            currMB->b8pdir[i]   = best8x8pdir[mode][i];
        }

        break;
    case I8MB:
    case I16MB:
    case InNxNMB:
    case INxnNMB:
        for (i = stage_block8x8_pos/*lgp*/; i < 4; i++) {
            currMB->b8mode[i] = IBLOCK;
            currMB->b8pdir[i] = INTRA;
        }

        break;
    default:
        printf("Unsupported mode in SetModesAndRefframeForBlocks!\n");
        exit(1);
    }

    //--- reference frame arrays ---
    if (mode == PSKIPDIRECT || mode == I8MB || mode == I16MB || mode == InNxNMB || mode == INxnNMB) {
        if (bframe) {
            if (pMCParam->dir == 0) {
                for (j = stage_block8x8_pos / 2; j < 2; j++) {
                    for (i = 0; i < 2; i++) {
                        get_b8_offset(mode, uiBitSize, i, j, &col, &row, &step_h, &step_v);

                        for (r = 0; r < step_v; r++) {
                            for (c = 0; c < step_h; c++) {
                                k = 2 * j + i;

                                if (!mode) {
                                    fwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                                    bwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                                } else {
                                    fwrefarr[block_8y + row + r][block_8x + col + c] = -1;
                                    bwrefarr[block_8y + row + r][block_8x + col + c] = -1;
                                }
                            }
                        }
                    }
                }
            } else {
                for (j = stage_block8x8_pos / 2; j < 2; j++) {
                    for (i = 0; i < 2; i++) {
                        get_b8_offset(mode, uiBitSize, i, j, &col, &row, &step_h, &step_v);

                        for (r = 0; r < step_v; r++) {
                            for (c = 0; c < step_h; c++) {
                                k = 2 * j + i;

                                if (!mode) {
                                    if (pMCParam->dir == DS_BID || pMCParam->dir == DS_SYM) {
                                        fwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                                        bwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                                    } else if (pMCParam->dir == DS_BACKWARD) {
                                        fwrefarr[block_8y + row + r][block_8x + col + c] = -1;
                                        bwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                                    } else if (pMCParam->dir == DS_FORWARD) {
                                        fwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                                        bwrefarr[block_8y + row + r][block_8x + col + c] = -1;
                                    }

                                } else {
                                    fwrefarr[block_8y + row + r][block_8x + col + c] = -1;
                                    bwrefarr[block_8y + row + r][block_8x + col + c] = -1;
                                }
                            }
                        }
                    }
                }
            }
        } else {
            for (j = stage_block8x8_pos / 2; j < 2; j++) {
                for (i = 0; i < 2; i++) {
                    get_b8_offset(mode, uiBitSize, i, j, &col, &row, &step_h, &step_v);


                    for (r = 0; r < step_v; r++) {
                        for (c = 0; c < step_h; c++) {
                            {
                                if (pMCParam->dir == 0) {
                                    frefarr[block_8y + row + r][block_8x + col + c] = (mode == PSKIPDIRECT ? 0 : -1);
                                    hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = -1;
                                } else if (pMCParam->dir == BID_P_FST || pMCParam->dir == BID_P_SND) {
                                    frefarr[block_8y + row + r][block_8x + col + c] = img->tmp_pref_fst[pMCParam->dir];
                                    hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = img->tmp_pref_snd[pMCParam->dir];
                                } else if (pMCParam->dir == FW_P_FST) {
                                    frefarr[block_8y + row + r][block_8x + col + c] = img->tmp_pref_fst[pMCParam->dir];
                                    hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = -1;
                                } else if (pMCParam->dir == FW_P_SND) {
                                    frefarr[block_8y + row + r][block_8x + col + c] = img->tmp_pref_fst[pMCParam->dir];
                                    hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = -1;
                                }

                                if (pMCParam->num_skipmode != 0) {
                                    hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = pMCParam->num_skipmode;
                                }
                            }
                        }
                    }
                }
            }
        }
    } else {
        if (bframe) {
            for (j = stage_block8x8_pos / 2; j < 2; j++) {
                for (i = 0; i < 2; i++) {
                    get_b8_offset(mode, uiBitSize, i, j, &col, &row, &step_h, &step_v);


                    for (r = 0; r < step_v; r++) {
                        for (c = 0; c < step_h; c++) {
                            k = 2 * j + i;

                            if ((mode == PNXN) && (best8x8mode[k] == 0)) {
                                fwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                                bwrefarr[block_8y + row + r][block_8x + col + c] = 0;
                            } else {
                                if (IS_FW && IS_BW) {
                                    fwrefarr[block_8y + row + r][block_8x + col + c] = best8x8syoneForthRefY[mode][k][0];
                                    bwrefarr[block_8y + row + r][block_8x + col + c] = best8x8syoneForthRefY[mode][k][1];
                                } else if (IS_bid) {
                                    fwrefarr[block_8y + row + r][block_8x + col + c] = best8x8bidref[mode][k][0];
                                    bwrefarr[block_8y + row + r][block_8x + col + c] = best8x8bidref[mode][k][1];
                                } else {
                                    fwrefarr[block_8y + row + r][block_8x + col + c] = (IS_FW ? best8x8ref[mode][k] : -1);
                                    bwrefarr[block_8y + row + r][block_8x + col + c] = (IS_BW ? best8x8bwref[mode][k] : -1);
                                }
                            }
                        }
                    }
                }
            }
        } else {
            for (j = stage_block8x8_pos / 2/*lgp*/; j < 2; j++) {
                for (i = 0; i < 2; i++) {
                    get_b8_offset(mode, uiBitSize, i, j, &col, &row, &step_h, &step_v);


                    for (r = 0; r < step_v; r++) {
                        for (c = 0; c < step_h; c++) {
                            k = 2 * j + i;
                            frefarr [block_8y + row + r][block_8x + col + c] = ((currMB->b8pdir[k] == FORWARD && (mode != PNXN ||
                                    best8x8mode[k] != 0)) ? best8x8ref[mode][k] : (currMB->b8pdir[k] == DUAL && (mode != PNXN ||
                                            best8x8mode[k] != 0)) ? best8x8dualref[mode][k][0] : -1);
                            hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = (currMB->b8pdir[k] == DUAL && (mode != PNXN ||
                                    best8x8mode[k] != 0)) ? best8x8dualref[mode][k][1] : -1;
                        }
                    }
                }
            }
        }
    }
}

/*
*************************************************************************
* Function:Sets motion vectors for an codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void SetMotionVectorsMB(codingUnit *currMB, int bframe, unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int i, j, k, mode8, pdir8, ref, by, bx, bxr, dref;
    int ***mv        = img->tmp_mv;
    int ** ***allFwMv  = img->allFwMv;
    int ** ***allBwMv = img->allBwMv;
    int ***fwMV      = img->fw_mv;
    int ***bwMV      = img->bw_mv;
    int **refar = hc->refFrArr;
    int **p_snd_refar = hc->p_snd_refFrArr;
    int **frefar     = img->fw_refFrArr;
    int block_y      = (uiPositionInPic / img->PicWidthInMbs) << 1;   //img->block8_y;
    int block_x      = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;
    int **brefar     = img->bw_refFrArr;
    int  bw_ref;
    int stage_block8x8_pos = 0; /*lgp*/
    int width, height;

    int ***pSndmv        = img->p_snd_tmp_mv;
#if FIX_MAX_REF
    int delta_P[MAXREF];
#else
    int delta_P[4];
#endif

    int r, c;
    int ii, jj;

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

    for (j = stage_block8x8_pos / 2; j < 2; j++)
        for (i = 0; i < 2; i++) {
            mode8 = currMB->b8mode[k = 2 * j + i];
            pdir8 = currMB->b8pdir[k];
            if (pdir8 == BID && mode8 != 0) {
                allFwMv = img->allBidFwMv;
                //  predBFwMv=img->predBidFwMv;

                allBwMv = img->allBidBwMv;
                //predBBwMv=img->predBidBwMv;
            } else {
                if (pdir8 == SYM && mode8 != 0) {   //mode8 != 0 added by xyji 07.11
                    allFwMv = img->allSymMv;
                } else if (pdir8 == DUAL && mode8 != 0) {
                    allFwMv = img->allDualFstMv;
                    allBwMv = img->allDualSndMv;
                } else {
                    allFwMv = img->allFwMv;
                    allBwMv = img->allBwMv;
                }
            }

            if (img->type == B_IMG && currMB->md_directskip_mode != 0) {
                for (ii = 0; ii < 2; ii ++)
                    for (jj = 0; jj < 2; jj ++) {
                        allFwMv[ii][jj][0][0][0] = img->tmp_fwBSkipMv[currMB->md_directskip_mode][0];
                        allFwMv[ii][jj][0][0][1] = img->tmp_fwBSkipMv[currMB->md_directskip_mode][1];
                        allBwMv[ii][jj][0][0][0] = img->tmp_bwBSkipMv[currMB->md_directskip_mode][0];
                        allBwMv[ii][jj][0][0][1] = img->tmp_bwBSkipMv[currMB->md_directskip_mode][1];
                    }
            }

            get_b8_offset(mode8, uiBitSize, i , j , &bx , &by, &width, &height);
            by = block_y + by;
            bx = block_x + bx;
            bxr = bx;

            ref   = (bframe ? frefar : refar) [by][bxr];
            bw_ref = (bframe ? brefar : p_snd_refar) [by][bxr];

            if (img->type == F_IMG  && input->dhp_enabled && mode8 >= P2NX2N && mode8 <= PNXN && pdir8 == FORWARD) {
                img->mv[j][i][ref][mode8][0] = img->forwardpred_mv[j][i][ref][mode8][0];
                img->mv[j][i][ref][mode8][1] = img->forwardpred_mv[j][i][ref][mode8][1];
                img->allFwMv[j][i][ref][mode8][0] = img->forwardpred_allFwMv[j][i][ref][mode8][0];
                img->allFwMv[j][i][ref][mode8][1] = img->forwardpred_allFwMv[j][i][ref][mode8][1];
                img->allFwMv[j][i][ref][mode8][2] = img->forwardpred_allFwMv[j][i][ref][mode8][2];
            }

            if (img->type == F_IMG && currMB->md_directskip_mode != 0) {
                for (ii = 0; ii < 2; ii ++)
                    for (jj = 0; jj < 2; jj ++) {
                        if (ref != -1) {
                            allFwMv[ii][jj][ref][0][0] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][0];
                            allFwMv[ii][jj][ref][0][1] = img->tmp_fstPSkipMv[currMB->md_directskip_mode][1];
                            allFwMv[ii][jj][ref][0][2] = 0;
                            allBwMv[ii][jj][0][0][0] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][0];
                            allBwMv[ii][jj][0][0][1] = img->tmp_sndPSkipMv[currMB->md_directskip_mode][1];
                            allBwMv[ii][jj][0][0][2] = 0;
                        }
                    }
            }

            if (img->type == F_IMG && mode8 == 0 && currMB->weighted_skipmode != 0) {
                for (ii = 0; ii < 2; ii ++)
                    for (jj = 0; jj < 2; jj ++) {
#if MV_SCALE
                        allBwMv[ii][jj][0][0][0] = scale_mv(allFwMv[ii][jj][0][0][0], delta_P[currMB->weighted_skipmode], delta_P[0]);
                        allBwMv[ii][jj][0][0][1] = scale_mv(allFwMv[ii][jj][0][0][1], delta_P[currMB->weighted_skipmode], delta_P[0]);
#if HALF_PIXEL_COMPENSATION_M1
                        //assert(is_field_sequence == img->is_field_sequence);
                        if (img->is_field_sequence) {
                            allBwMv[ii][jj][0][0][1] = scale_mv_y1(allFwMv[ii][jj][0][0][1], delta_P[currMB->weighted_skipmode], delta_P[0]);
                        }
#endif
#else
                        allBwMv[ii][jj][0][0][0] = (int)((delta_P[currMB->weighted_skipmode] * allFwMv[ii][jj][0][0][0] *
                                                          (MULTI / delta_P[0]) + HALF_MULTI) >> OFFSET);
                        allBwMv[ii][jj][0][0][1] = (int)((delta_P[currMB->weighted_skipmode] * allFwMv[ii][jj][0][0][1] *
                                                          (MULTI / delta_P[0]) + HALF_MULTI) >> OFFSET);
#if HALF_PIXEL_COMPENSATION_M1
                        //assert(is_field_sequence == img->is_field_sequence);
                        if (img->is_field_sequence) {
                            int deltaT, delta2;
                            int oriPOC = 2 * hc->picture_distance;
                            int oriRefPOC = oriPOC - delta_P[0];
                            int scaledPOC = 2 * hc->picture_distance;
                            int scaledRefPOC = scaledPOC - delta_P[currMB->weighted_skipmode];
                            getDeltas(&deltaT, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                            allBwMv[ii][jj][0][0][1] = (int)(((delta_P[currMB->weighted_skipmode] * (allFwMv[ii][jj][0][0][1] + deltaT) *
                                                               (16384 / delta_P[0]) + 8192) >> 14) - delta2);
                        }
#endif
#endif
                        allBwMv[ii][jj][0][0][0] = Clip3(-32768, 32767, allBwMv[ii][jj][0][0][0]);
                        allBwMv[ii][jj][0][0][1] = Clip3(-32768, 32767, allBwMv[ii][jj][0][0][1]);

                        allBwMv[ii][jj][0][0][2] = 0;
                    }
            }

            for (r = 0; r < height; r++) {
                for (c = 0; c < width; c++) {
                    if (!bframe) {
                        {
                            if (mode8 != IBLOCK && ref != -1) {
                                mv[by + r][bx + c][0] = allFwMv[j][i][ref][mode8][0];
                                mv[by + r][bx + c][1] = allFwMv[j][i][ref][mode8][1];
                                mv[by + r][bx + c][2] = allFwMv[j][i][ref][mode8][2];
                                if (pdir8 == DUAL) {
                                    pSndmv[by + r][bx + c][0] = allBwMv[j][i][ref][mode8][0];
                                    pSndmv[by + r][bx + c][1] = allBwMv[j][i][ref][mode8][1];
                                    pSndmv[by + r][bx + c][2] = allBwMv[j][i][ref][mode8][2];
                                } else if (mode8 == PSKIPDIRECT) {
                                    pSndmv[by + r][bx + c][0] = allBwMv[j][i][0][mode8][0];
                                    pSndmv[by + r][bx + c][1] = allBwMv[j][i][0][mode8][1];
                                    pSndmv[by + r][bx + c][2] = allBwMv[j][i][0][mode8][2];
                                } else {
                                    pSndmv[by + r][bx + c][0] = 0;
                                    pSndmv[by + r][bx + c][1] = 0;
                                    pSndmv[by + r][bx + c][2] = 0;
                                }
                            } else {
                                mv[by + r][bx + c][0] = 0;
                                mv[by + r][bx + c][1] = 0;
                                mv[by + r][bx + c][2] = 0;
                                pSndmv[by + r][bx + c][0] = 0;
                                pSndmv[by + r][bx + c][1] = 0;
                                pSndmv[by + r][bx + c][2] = 0;
                            }
                        }
                    } else {
                        if (pdir8 == INTRA) {   // intra
                            fwMV [by + r][bx + c][0]     = 0;
                            fwMV [by + r][bx + c][1]     = 0;
                            fwMV [by + r][bx + c][2]     = 0;
                            bwMV [by + r][bx + c][0]     = 0;
                            bwMV [by + r][bx + c][1]     = 0;
                            bwMV [by + r][bx + c][2]     = 0;
                        } else if (pdir8 == FORWARD) {   // forward
                            fwMV [by + r][bx + c][0]     = allFwMv [j][i][ ref][mode8][0];
                            fwMV [by + r][bx + c][1]     = allFwMv [j][i][ ref][mode8][1];
                            fwMV [by + r][bx + c][2]     = allFwMv [j][i][ ref][mode8][2];
                            bwMV [by + r][bx + c][0]     = 0;
                            bwMV [by + r][bx + c][1]     = 0;
                            bwMV [by + r][bx + c][2]     = 0;
                        } else if (pdir8 == BACKWARD) {   // backward
                            fwMV [by + r][bx + c][0] = 0;
                            fwMV [by + r][bx + c][1] = 0;
                            fwMV [by + r][bx + c][2] = 0;
                            {
                                bwMV [by + r][bx + c][0] = allBwMv[j][i][   bw_ref][mode8][0]; /*lgp*13*/
                                bwMV [by + r][bx + c][1] = allBwMv[j][i][   bw_ref][mode8][1]; /*lgp*13*/
                                bwMV [by + r][bx + c][2] = allBwMv[j][i][   bw_ref][mode8][2]; /*lgp*13*/
                            }
                        } else if (mode8 != 0) {   // symirect
                            fwMV [by + r][bx + c][0] = allFwMv [j][i][ ref][mode8][0];
                            fwMV [by + r][bx + c][1] = allFwMv [j][i][ ref][mode8][1];
                            fwMV [by + r][bx + c][2] = allFwMv [j][i][ ref][mode8][2];
                            if (pdir8 == BID) {
                                bwMV [by + r][bx + c][0] = img->allBwMv [j][i][ ref][mode8][0];
                                bwMV [by + r][bx + c][1] = img->allBwMv [j][i][ ref][mode8][1];
                                bwMV [by + r][bx + c][2] = img->allBwMv [j][i][ ref][mode8][2];

                            } else {
                                int delta_P, TRp, DistanceIndexFw, DistanceIndexBw, refframe, delta_PB;
                                refframe = ref;
                                delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
                                delta_P = (delta_P + 512) % 512;

                                TRp = (refframe + 1) * delta_P;    //the lates backward reference

                                delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);
                                TRp = (TRp + 512) % 512;
                                delta_PB = (delta_PB + 512) % 512;

                                DistanceIndexFw = delta_PB;

                                DistanceIndexBw    = (TRp - DistanceIndexFw + 512) % 512;

#if MV_SCALE
                                bwMV [by + r][bx + c][0] = - scale_mv(allFwMv[j][i][ref][mode8][0], DistanceIndexBw, DistanceIndexFw);
                                bwMV [by + r][bx + c][1] = - scale_mv(allFwMv[j][i][ref][mode8][1], DistanceIndexBw, DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
                                if (img->is_field_sequence) {
                                    bwMV[by + r][bx + c][1] = -scale_mv_y2(allFwMv[j][i][ref][mode8][1], DistanceIndexBw, DistanceIndexFw);
                                }
#endif
#else
                                bwMV [by + r][bx + c][0] = - ((allFwMv[j][i][ref][mode8][0] * DistanceIndexBw * (MULTI / DistanceIndexFw) +
                                                               HALF_MULTI) >> OFFSET);
                                bwMV [by + r][bx + c][1] = - ((allFwMv[j][i][ref][mode8][1] * DistanceIndexBw * (MULTI / DistanceIndexFw) +
                                                               HALF_MULTI) >> OFFSET);
#if HALF_PIXEL_COMPENSATION_MVD
                                if (img->is_field_sequence) {
                                    int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                                    oriPOC = 2 * hc->picture_distance;
                                    oriRefPOC = oriPOC - DistanceIndexFw;
                                    scaledPOC = 2 * hc->picture_distance;
                                    scaledRefPOC = scaledPOC - DistanceIndexBw;
                                    getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                                    bwMV [by + r][bx + c][1] = - (((allFwMv[j][i][ref][mode8][1] + delta) * DistanceIndexBw *
                                                                   (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                                }
#endif
#endif
                                bwMV [by + r][bx + c][0] = Clip3(-32768, 32767, bwMV [by + r][bx + c][0]);
                                bwMV [by + r][bx + c][1] = Clip3(-32768, 32767, bwMV [by + r][bx + c][1]);
                                bwMV [by + r][bx + c][2] = 0;
                            }
                        } else { // direct
                            dref = 0;

                            fwMV [by + r][bx + c][0] = allFwMv [j][i][dref][    0][0]; //qyu delete
                            fwMV [by + r][bx + c][1] = allFwMv [j][i][dref][    0][1]; //qyu delete
                            fwMV [by + r][bx + c][2] = allFwMv [j][i][dref][    0][2]; //qyu delete

                            bwMV [by + r][bx + c][0] = allBwMv[j][i][   0][    0][0]; //qyu delete
                            bwMV [by + r][bx + c][1] = allBwMv[j][i][   0][    0][1]; //qyu delete
                            bwMV [by + r][bx + c][2] = allBwMv[j][i][   0][    0][2]; //qyu delete
                        }
                    }
                }
            }
        }
}


/*
*************************************************************************
* Function:Store codingUnit parameters
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void store_codingUnit_parameters(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int mode)
{
    int    i, j;
    int    bframe   = (img->type == B_IMG);
    int    **frefar = ((img->type == B_IMG) ? img->fw_refFrArr : hc->refFrArr);
    int    **brefar = img->bw_refFrArr;
    int    block_8x  = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;
    int    block_8y  = (uiPositionInPic / img->PicWidthInMbs) << 1;   //img->block8_y;

    int    pix_x    = block_8x << MIN_BLOCK_SIZE_IN_BIT;
    int    pix_c_x  = pix_x >> 1;
    int    pix_y    = block_8y << MIN_BLOCK_SIZE_IN_BIT;
    int    pix_c_y  = input->chroma_format == 1 ? (pix_y >> 1) : (pix_y);
    int    stage_block8x8_pos = 0; /*lgp*/
    int **tmp;
    int ref;
    int start_x, start_y, width, height;


    //--- store best mode ---
    best_mode = mode;

    best_trans_size = currMB->trans_size;
    if (mode == I8MB || mode == I16MB || mode == InNxNMB || mode == INxnNMB) {
        best_c_imode = currMB->c_ipred_mode;
    } else {
        best_c_imode = DC_PRED_C;
    }

    for (i = stage_block8x8_pos; i < 4; i++) {
        b8mode[i]   = currMB->b8mode[i];
        b8pdir[i]   = currMB->b8pdir[i];
    }

    //--- reconstructed blocks ----
    for (j = 0; j < (1 << uiBitSize); j++) {
        for (i = 0; i < (1 << uiBitSize); i++) {
            rec_mbY[j][i] = hc->imgY[pix_y + j][pix_x + i];
        }
    }

    if (input->chroma_format == 1) {
        {
            for (j = 0; j < (1 << (uiBitSize - 1)); j++) {
                for (i = 0; i < (1 << (uiBitSize - 1)); i++) {
                    rec_mbU[j][i] = hc->imgUV[0][pix_c_y + j][pix_c_x + i];
                    rec_mbV[j][i] = hc->imgUV[1][pix_c_y + j][pix_c_x + i];
                }
            }
        }
    }

    if (best_mode == I8MB || best_mode == I16MB || best_mode == InNxNMB || best_mode == INxnNMB) {
        for (i = stage_block8x8_pos; i < 4; i++) {
            best_intra_pred_modes_tmp[i] = currMB->intra_pred_modes[i];
            best_real_intra_pred_modes_tmp[i] = currMB->real_intra_pred_modes[i];
        }

        if (best_mode == InNxNMB) {
            for (j = stage_block8x8_pos / 2; j < 2; j++) {
                for (i = 0; i < 2; i++) {

                    best_ipredmode_tmp[j][i] = img->ipredmode[  1 + block_8y + ((j << 1) + i) * (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT -
                                               1))  ][ 1 + block_8x ];    //qyu 0829

                }
            }
        } else if (best_mode == INxnNMB) {
            for (j = stage_block8x8_pos / 2; j < 2; j++) {
                for (i = 0; i < 2; i++) {

                    best_ipredmode_tmp[j][i] =  img->ipredmode[  1 + block_8y   ][ 1 + block_8x + ((j << 1) + i) * (1 <<
                                                (uiBitSize - MIN_CU_SIZE_IN_BIT - 1)) ];    //qyu 0829

                }
            }
        } else
            for (j = stage_block8x8_pos / 2; j < 2; j++) {
                for (i = 0; i < 2; i++) {
                    best_ipredmode_tmp[j][i] = img->ipredmode[  1 + block_8y + j * (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)) ][ 1 +
                                               block_8x + i * (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)) ];      //qyu 0829
                }
            }
    }


    //--- coeff, cbp, kac ---
    if (1) {
        tmp = Coeff_all_tmp_best;
        Coeff_all_tmp_best = img->Coeff_all;
        img->Coeff_all = tmp;
        cbp     = currMB->cbp;
        cbp_blk = currMB->cbp_blk;
    } else {
        cbp = cbp_blk = 0;
    }

#if MB_DQP
    if (cbp != 0) {
        dQP      = currMB->delta_qp;
        QP = currMB->qp;
        left_cu_qp = currMB->left_cu_qp;
        previouse_qp = currMB->previouse_qp;
    } else {
        dQP      = 0;
        QP = currMB->left_cu_qp;
        left_cu_qp = currMB->left_cu_qp;
        previouse_qp = currMB->previouse_qp;
    }
#endif

    for (j = stage_block8x8_pos / 2; j < 2; j++) {
        for (i = 0; i < 2; i++) {
            get_b8_offset(mode, uiBitSize, i, j, &start_x, &start_y, &width, &height);



            frefframe[j][i] = frefar[block_8y + start_y  ][block_8x + start_x ]; //qyu 0829
            if (img->type == F_IMG) {
                p_snd_frefframe[j][i] = hc->p_snd_refFrArr[block_8y + start_y][block_8x + start_x];
            }
            if (img->type == F_IMG && input->dhp_enabled && mode >= P2NX2N && mode <= PNXN) {
                ref = frefframe[j][i];
                img->forwardpred_mv[j][i][ref][mode][0] = img->mv[j][i][ref][mode][0];
                img->forwardpred_mv[j][i][ref][mode][1] = img->mv[j][i][ref][mode][1];
                img->forwardpred_allFwMv[j][i][ref][mode][0] = img->allFwMv[j][i][ref][mode][0];
                img->forwardpred_allFwMv[j][i][ref][mode][1] = img->allFwMv[j][i][ref][mode][1];
                img->forwardpred_allFwMv[j][i][ref][mode][2] = img->allFwMv[j][i][ref][mode][2];
            }

            if (bframe) {

                brefframe[j][i] = brefar[block_8y + start_y ][block_8x + start_x ];
            }
        }
    }

    store_coding_state(cs_submb_best);    //[uiBitSize - 4] );
    reset_coding_state(cs_cm);
}



/*
*************************************************************************
* Function:R-D Cost for a codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int RDCost_for_codingUnits(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, double   lambda,
                           int mode, double  *min_rdcost, MCParam *pMCParam)  // <-> minimum rate-distortion cost
{
    int        i, j; //, k, ****ip4;
    double      rdcost;
    int         bframe        = (img->type == B_IMG);
    int         dummy;//cc_rate,
    byte        **imgY_orig   = he->imgY_org;
    byte        ***imgUV_orig = he->imgUV_org;
    int         pix_y         = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;   //img->pix_y;//qyu 0823
    int         pix_c_y       = input->chroma_format == 1 ? (pix_y >> 1) : pix_y;   //qyu 0823
    int         pix_x         = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;   //img->pix_x;//qyu 0823
    int         pix_c_x       = pix_x >> 1; //qyu 0823
    int         rate_tmp, rate_top = 0, rate_bot = 0, distortion_blk, distortion_top = 0, distortion_bot = 0;
    double      rdcost_top, rdcost_bot;
    int         incr_y = 1, off_y = 0;
    int         block8x8, block_x, block_y;
    int         stage_block8x8_pos = 0;
    int         cp_table[2][8] = {{1, 1, 1, 1, 1, 1, 1, 1}, {0, 0, 1, 1, 0, 1, 0, 1}};
    int         tmp_cbp;
    int         tmp_cbp_luma;
    int         mb_available_up;
    int         mb_available_left;
    int         mb_available_up_left;
    int         k;
    int         **tmp_Coeff_luma = NULL;
    int lmode = 0;
    Boolean is_redundant = FALSE;
    int         **tmp_Rec_Y = NULL;
    get_mem2Dint(&tmp_Rec_Y, 1 << input->g_uiMaxSizeInBit, 1 << input->g_uiMaxSizeInBit);
    get_mem2Dint(&tmp_Coeff_luma, (1 << uiBitSize), (1 << uiBitSize));
    img->mv_range_flag = 1;

    //=====  SET REFERENCE FRAMES AND BLOCK MODES
    SetModesAndRefframeForBlocks(currMB, uiBitSize, uiPositionInPic, mode,
                                 pMCParam);  //uiBitSize,uiPositionInPic//qyu 0823
    //=====  GET COEFFICIENTS, RECONSTRUCTIONS, CBP
    if (mode == I8MB || mode == I16MB || mode == InNxNMB || mode == INxnNMB) {
        currMB->trans_size = (mode == I16MB) ? 0 : 1;
        currMB->cbp = Mode_Decision_for_IntracodingUnit(currMB, uiBitSize, uiPositionInPic, lambda, &dummy);
        IntraChromaPrediction8x8(currMB, uiBitSize - 1, uiPositionInPic, &mb_available_up, &mb_available_left,
                                 &mb_available_up_left);

        for (j = 0; j < (1 << uiBitSize); j++) {     //qyu 0823
            for (i = 0; i < (1 << uiBitSize); i++) {
                tmp_Coeff_luma[j][i] = img->Coeff_all[j][i];
            }
        }
    } else {
        currMB->trans_size = 1;
#if Mv_Rang
        LumaResidualCoding(currMB, uiBitSize, uiPositionInPic,
                           pMCParam, mode);  //qyu 0823 .. quantilized coefficient: img->Coeff_all
#else
        LumaResidualCoding(currMB, uiBitSize, uiPositionInPic,
                           pMCParam);  //qyu 0823 .. quantilized coefficient: img->Coeff_all
#endif
    }

    if (IS_INTRA(currMB)) {
        int block8_y = (uiPositionInPic / img->PicWidthInMbs) << 1;
        int block8_x = (uiPositionInPic % img->PicWidthInMbs) << 1;
        int LumaMode = img->ipredmode[1 + block8_y][1 + block8_x];

        if (LumaMode == VERT_PRED || LumaMode == HOR_PRED || LumaMode == DC_PRED || LumaMode == BI_PRED) {
            lmode = LumaMode == VERT_PRED ? VERT_PRED_C : (LumaMode == HOR_PRED ? HOR_PRED_C :
                    (LumaMode == DC_PRED ?  DC_PRED_C : BI_PRED_C));
            is_redundant = TRUE;
        }
    }

    for (tmp_cbp_luma = currMB->cbp, currMB->c_ipred_mode = 0; currMB->c_ipred_mode < NUM_INTRA_PMODE_CHROMA;
         currMB->c_ipred_mode++) {  //
        if (IS_INTRA(currMB)) {
            if (currMB->c_ipred_mode != DM_PRED_C && is_redundant && currMB->c_ipred_mode == lmode) {
                continue;
            }
        }

        if (currMB->c_ipred_mode == DC_PRED_C || (IS_INTRA(currMB))) {
            if (IS_INTRA(currMB)) {
                currMB->cbp = tmp_cbp_luma;

                for (j = 0; j < (1 << uiBitSize); j++) {     //qyu 0823
                    for (i = 0; i < (1 << uiBitSize); i++) {
                        img->Coeff_all[j][i] = tmp_Coeff_luma[j][i];
                    }
                }
            }

            dummy = 0;
            ChromaResidualCoding(currMB, uiBitSize, uiPositionInPic, &dummy, pMCParam);    //quantilized coefficient: img->Coeff_all
            distortion_top = distortion_bot = 0; //=c_distortion=0;//qyu add
            //=====   GET DISTORTION
            // LUMA

            /*lgp*/
            for (block8x8 = stage_block8x8_pos; block8x8 < 4; block8x8++) {
                block_y = (block8x8 / 2) * (1 << (uiBitSize - 1));       //qyu 0823
                block_x = (block8x8 % 2) * (1 << (uiBitSize - 1));

                distortion_blk = 0;
                if (currMB->cuType == InNxNMB) {
                    block_y = (1 << (uiBitSize - 2)) * block8x8;
                    block_x = 0;
                    for (j = 0; j < (1 << (uiBitSize - 2)); j++) {
                        for (i = 0; i < (1 << ((uiBitSize))); i++) {
                            distortion_blk += img->quad[imgY_orig[pix_y + block_y + incr_y * j + off_y][pix_x + block_x + i] - hc->imgY[pix_y +
                                                        block_y + incr_y * j + off_y][pix_x + block_x + i]];
                        }
                    }
                } else if (currMB->cuType == INxnNMB) {
                    block_x = (1 << (uiBitSize - 2)) * block8x8;
                    block_y = 0;
                    for (j = 0; j < (1 << ((uiBitSize))); j++) {
                        for (i = 0; i < (1 << (uiBitSize - 2)); i++) {
                            distortion_blk += img->quad[imgY_orig[pix_y + block_y + incr_y * j + off_y][pix_x + block_x + i] - hc->imgY[pix_y +
                                                        block_y + incr_y * j + off_y][pix_x + block_x + i]];
                        }
                    }
                } else {
                    for (j = 0; j < (1 << (uiBitSize - 1)); j++) {
                        for (i = 0; i < (1 << (uiBitSize - 1)); i++) {
                            distortion_blk += img->quad[imgY_orig[pix_y + block_y + incr_y * j + off_y][pix_x + block_x + i] - hc->imgY[pix_y +
                                                        block_y + incr_y * j + off_y][pix_x + block_x + i]];
                        }
                    }
                }

                block_y = (block8x8 / 2) * (1 << (uiBitSize - 2));
                block_x = (block8x8 % 2) * (1 << (uiBitSize - 2));

                // CHROMA
                if (input->chroma_format == 1) {
                    k = (1 << (uiBitSize - 2));
                }

                {
                    for (j = 0; j < k; j++) {
                        for (i = 0; i < (1 << (uiBitSize - 2)); i++) {
                            distortion_blk += img->quad[imgUV_orig[0][block_y + incr_y * j + off_y + pix_c_y][block_x + i + pix_c_x] -
                                                        hc->imgUV[0][pix_c_y + block_y + incr_y * j + off_y][pix_c_x + block_x + i]];
                            distortion_blk += img->quad[imgUV_orig[1][block_y + incr_y * j + off_y + pix_c_y][block_x + i + pix_c_x] -
                                                        hc->imgUV[1][pix_c_y + block_y + incr_y * j + off_y][pix_c_x + block_x + i]];
                        }
                    }
                }

                if (block8x8 < 2) {
                    distortion_top += distortion_blk;
                } else {
                    distortion_bot += distortion_blk;
                }

            }

            //=====   S T O R E   C O D I N G   S T A T E   =====
            store_coding_state(cs_cm);

            //=====   GET RATE
            //----- codingUnit header -----
            rate_top = rate_bot = 0; //qyu add

            if (input->chroma_format == 1) {
                tmp_cbp =  currMB->cbp;
            }

            if (img->type != INTRA_IMG) {
                rate_top   = writeMBHeader(currMB, uiBitSize, uiPositionInPic, 1);
            } else if (!(stage_block8x8_pos / 2)) {      /*lgp*/
                rate_top = writeMBHeader(currMB, uiBitSize, uiPositionInPic, 1);    /*lgp*/
            }

            if (mode) {   //qyu 0823 to be modified
                //----- motion information -----
                storeMotionInfo(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, pMCParam->dmh_mode);
                writeReferenceIndex(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, &rate_top, &rate_bot);
                writeMVD(currMB, stage_block8x8_pos / 2, &rate_top, &rate_bot, uiPositionInPic);
                writeCBPandDqp(currMB, 0, &rate_top, &rate_bot, uiPositionInPic);
            }
            if (mode == 0) {
                storeMotionInfo(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, pMCParam->dmh_mode);
                writeReferenceIndex(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, &rate_top, &rate_bot);
                writeMVD(currMB, stage_block8x8_pos / 2, &rate_top, &rate_bot, uiPositionInPic);
            }
            if (mode == 0 && tmp_cbp != 0) {
                writeCBPandDqp(currMB, 0, &rate_top, &rate_bot, uiPositionInPic);
            }
            if (mode || tmp_cbp) {
                for (block8x8 = stage_block8x8_pos; block8x8 < (input->chroma_format == 1 ? 6 : 8);) {
                    if (cp_table[stage_block8x8_pos / 2][block8x8]) {
                        int tmp_uiBitSize = currMB->trans_size == 0 ? uiBitSize : (uiBitSize - 1);
                        if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (block8x8 < 4 && currMB->trans_size == 1 &&
                                (currMB->cuType == P2NXN || currMB->cuType == PHOR_UP || currMB->cuType == PHOR_DOWN))) {
                            if (uiBitSize == B64X64_IN_BIT) {
                                tmp_uiBitSize--;
                            }
                            block_y = block8x8 * (1 << (tmp_uiBitSize - 1));
                            block_x = 0;
                        } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (block8x8 < 4 && currMB->trans_size == 1 &&
                                   (currMB->cuType == PNX2N || currMB->cuType == PVER_LEFT || currMB->cuType == PVER_RIGHT))) {
                            if (uiBitSize == B64X64_IN_BIT) {
                                tmp_uiBitSize--;
                            }
                            block_y = 0;
                            block_x = block8x8 * (1 << (tmp_uiBitSize - 1));
                        } else if (input->useSDIP && (block8x8 < 4 && currMB->trans_size == 1 && (currMB->cuType == InNxNMB))) {
                            block_y = block8x8 * (1 << (tmp_uiBitSize - 1));
                            block_x = 0;
                        }

                        else if (input->useSDIP && (block8x8 < 4 && currMB->trans_size == 1 && (currMB->cuType == INxnNMB))) {
                            block_y = 0;
                            block_x = block8x8 * (1 << (tmp_uiBitSize - 1));
                        } else {
                            block_y = (block8x8 / 2) * (1 << (uiBitSize - 1));       //qyu 0823
                            block_x = (block8x8 % 2) * (1 << (uiBitSize - 1));
                        }

                        rate_tmp = writeBlockCoeff(img->Coeff_all, currMB, tmp_uiBitSize, block_x, block_y, block8x8, uiPositionInPic);

                        if (block8x8 < 2) {
                            rate_top += rate_tmp;
                        } else if (block8x8 < 4) {
                            rate_bot += rate_tmp;
                        }
                        if (block8x8 == 4) {
                            rate_top += rate_tmp;
                        }
                        if (block8x8 == 5) {
                            rate_bot += rate_tmp;
                        }
                    }
                    if (currMB->trans_size == 0 && block8x8 == stage_block8x8_pos) {
                        block8x8 += 4;
                    } else {
                        block8x8++;
                    }
                }
            }


            //=====   R E S T O R E   C O D I N G   S T A T E   =====

            rdcost_top = (double) distortion_top + lambda * (double) rate_top;
            rdcost_bot = (double) distortion_bot + lambda * (double) rate_bot;

            rdcost = rdcost_top + rdcost_bot;

            if (!(IS_INTRA(currMB))) {
                store_coding_state(cs_tmp);
                reset_coding_state(cs_cm);
                for (j = 0; j < (1 << uiBitSize); j++) {     //qyu 0823
                    for (i = 0; i < (1 << uiBitSize); i++) {
                        tmp_Coeff_luma[j][i] = img->Coeff_all[j][i];
                        tmp_Rec_Y[j][i] = hc->imgY[pix_y + j][pix_x + i];
                    }
                }

                currMB->trans_size = 0;
#if Mv_Rang
                LumaResidualCoding(currMB, uiBitSize, uiPositionInPic, pMCParam, mode);
#else
                LumaResidualCoding(currMB, uiBitSize, uiPositionInPic, pMCParam);
#endif
                dummy = 0;
                {
                    ChromaResidualCoding(currMB, uiBitSize, uiPositionInPic, &dummy, pMCParam);    //quantilized coefficient: img->Coeff_all
                }
                dummy = 0;

                distortion_top = distortion_bot = 0; //=c_distortion=0;//qyu add
                //=====   GET DISTORTION
                // LUMA

                /*lgp*/
                for (block8x8 = stage_block8x8_pos; block8x8 < 4; block8x8++) {
                    block_y = (block8x8 / 2) * (1 << (uiBitSize - 1));       //qyu 0823
                    block_x = (block8x8 % 2) * (1 << (uiBitSize - 1));

                    distortion_blk = 0;

                    for (j = 0; j < (1 << (uiBitSize - 1)); j++) {
                        for (i = 0; i < (1 << (uiBitSize - 1)); i++) {
                            distortion_blk += img->quad[imgY_orig[pix_y + block_y + incr_y * j + off_y][pix_x + block_x + i] - hc->imgY[pix_y +
                                                        block_y
                                                        + incr_y * j + off_y][pix_x + block_x + i]];
                        }
                    }

                    block_y = (block8x8 / 2) * (1 << (uiBitSize - 2));
                    block_x = (block8x8 % 2) * (1 << (uiBitSize - 2));

                    // CHROMA
                    if (input->chroma_format == 1) {
                        k = (1 << (uiBitSize - 2));
                    }
                    {
                        for (j = 0; j < k; j++) {
                            for (i = 0; i < (1 << (uiBitSize - 2)); i++) {
                                distortion_blk += img->quad[imgUV_orig[0][block_y + incr_y * j + off_y + pix_c_y][block_x + i + pix_c_x] -
                                                            hc->imgUV[0][pix_c_y + block_y + incr_y * j + off_y][pix_c_x + block_x + i]];
                                distortion_blk += img->quad[imgUV_orig[1][block_y + incr_y * j + off_y + pix_c_y][block_x + i + pix_c_x] -
                                                            hc->imgUV[1][pix_c_y + block_y + incr_y * j + off_y][pix_c_x + block_x + i]];
                            }
                        }
                    }

                    if (block8x8 < 2) {
                        distortion_top += distortion_blk;
                    } else {
                        distortion_bot += distortion_blk;
                    }

                }


                //=====   GET RATE
                //----- codingUnit header -----
                rate_top = rate_bot = 0; //qyu add

                if (img->type != INTRA_IMG) {
                    rate_top   = writeMBHeader(currMB, uiBitSize, uiPositionInPic, 1);
                } else if (!(stage_block8x8_pos / 2)) {      /*lgp*/
                    rate_top = writeMBHeader(currMB, uiBitSize, uiPositionInPic, 1);    /*lgp*/
                }

                if (mode) {   //qyu 0823 to be modified
                    //----- motion information -----
                    storeMotionInfo(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, pMCParam->dmh_mode);
                    writeReferenceIndex(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, &rate_top, &rate_bot);
                    writeMVD(currMB, stage_block8x8_pos / 2, &rate_top, &rate_bot, uiPositionInPic);
                    writeCBPandDqp(currMB, 0, &rate_top, &rate_bot, uiPositionInPic);
                }
                if (mode == 0) {
                    storeMotionInfo(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, pMCParam->dmh_mode);
                    writeReferenceIndex(currMB, uiBitSize, uiPositionInPic, stage_block8x8_pos / 2, &rate_top, &rate_bot);
                    writeMVD(currMB, stage_block8x8_pos / 2, &rate_top, &rate_bot, uiPositionInPic);
                }

                if (mode == 0 && currMB->cbp != 0) {
                    writeCBPandDqp(currMB, 0, &rate_top, &rate_bot, uiPositionInPic);
                }
                if (mode || (currMB->cbp)) {
                    for (block8x8 = stage_block8x8_pos; block8x8 < (input->chroma_format == 1 ? 6 : 8);) {
                        if (cp_table[stage_block8x8_pos / 2][block8x8]) {
                            int tmp_uiBitSize = currMB->trans_size == 0 ? uiBitSize : (uiBitSize - 1);
                            block_y = (block8x8 / 2) * (1 << (uiBitSize - 1));       //qyu 0823
                            block_x = (block8x8 % 2) * (1 << (uiBitSize - 1));

                            rate_tmp = writeBlockCoeff(img->Coeff_all, currMB, tmp_uiBitSize, block_x, block_y, block8x8, uiPositionInPic);

                            if (block8x8 < 2) {
                                rate_top += rate_tmp;
                            } else if (block8x8 < 4) {
                                rate_bot += rate_tmp;
                            }
                            if (block8x8 == 4) {
                                rate_top += rate_tmp;
                            }
                            if (block8x8 == 5) {
                                rate_bot += rate_tmp;
                            }

                        }
                        if (currMB->trans_size == 0 && block8x8 == stage_block8x8_pos) {
                            block8x8 += 4;
                        } else {
                            block8x8++;
                        }
                    }
                }

                //=====   R E S T O R E   C O D I N G   S T A T E   =====

                //      reset_coding_state ( cs_cm );
                rdcost_top = (double) distortion_top + lambda * (double) rate_top;
                rdcost_bot = (double) distortion_bot + lambda * (double) rate_bot;

                if (rdcost_top + rdcost_bot < rdcost) {
                    rdcost = rdcost_top + rdcost_bot;
                    currMB->trans_size = 0;
                } else {
                    for (j = 0; j < (1 << uiBitSize); j++) {     //qyu 0823
                        for (i = 0; i < (1 << uiBitSize); i++) {
                            img->Coeff_all[j][i] = tmp_Coeff_luma[j][i];
                            hc->imgY[pix_y + j][pix_x + i] = tmp_Rec_Y[j][i];
                        }
                    }
                    reset_coding_state(cs_tmp);
                    currMB->trans_size = 1;
                    currMB->cbp = tmp_cbp;
                }
            }

            if (mode == I8MB || mode == I16MB || mode == InNxNMB || mode == INxnNMB) {
                if (rdcost < *min_rdcost) {
                    *min_rdcost = rdcost;
                    store_codingUnit_parameters(currMB, uiBitSize, uiPositionInPic, mode);
                } else {
                    reset_coding_state(cs_cm);
                }
            }
        }
    }

    free_mem2Dint(tmp_Coeff_luma);
    free_mem2Dint(tmp_Rec_Y);

    //CXR SDIP fast
    modeRDcost[mode] = rdcost;

    if (rdcost >= *min_rdcost) {
        return 0;
    }

    if (!img->mv_range_flag) {
        return 0;   //return 0 means it is not the best mode
    }

    //=====   U P D A T E   M I N I M U M   C O S T   =====
    *min_rdcost = rdcost;

    return 1;
}

/*
*************************************************************************
* Function:Set stored codingUnit parameters
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void set_stored_codingUnit_parameters(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic,
                                      double RD_largeBlk, double RD_smallBlk)
{
    int    i, j, k, r, c, row, col ;
    int    mode     = best_mode;
    int    bframe   = (img->type == B_IMG);
    int    **frefar = ((img->type == B_IMG) ? img->fw_refFrArr : hc->refFrArr);
    int    **brefar = img->bw_refFrArr;
    int    **tmp;
    int    block_8x  = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;
    int    block_8y  = (uiPositionInPic / img->PicWidthInMbs) << 1;   //img->block8_y;
    int    block8_InSMB_x = block_8x - img->block8_x;
    int    block8_InSMB_y = block_8y - img->block8_y;

    int    pix_x    = block_8x << MIN_BLOCK_SIZE_IN_BIT;
    int    pix_c_x  = pix_x >> 1;
    int    pix_y    = block_8y << MIN_BLOCK_SIZE_IN_BIT;
    int    pix_c_y  = input->chroma_format == 1 ? (pix_y >> 1) : (pix_y);
    int    pix_x_InSMB = pix_x - img->pix_x;
    int    pix_y_InSMB = pix_y - img->pix_y;
    int    pix_cx_InSMB = pix_c_x - img->pix_c_x;
    int    pix_cy_InSMB = pix_c_y - img->pix_c_y;
    int    SMB_pix_height = 1 << input->g_uiMaxSizeInBit;
    int    currMB_pix_height = 1 << uiBitSize;

    int    **frefar_curr = ((img->type == B_IMG) ? img->currFwRef : img->currRef);
    int    **brefar_curr = img->currBwRef;
    int    **p_snd_frefar_curr = img->currPSndRef;

    int    stage_block8x8_pos = 0; /*lgp*/

    int width, height;

    codingUnit *tmpMB;
    int WidthInLCU  = ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0));
    int HeightInLCU = ((img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 :
                       0));
    int LCUNumInPic = WidthInLCU * HeightInLCU;
    int currSMB_nr = (pix_y >> input->g_uiMaxSizeInBit) * WidthInLCU + (pix_x >> input->g_uiMaxSizeInBit);
    if (RD_largeBlk < RD_smallBlk) {
        currMB->cuType = mode;
        currMB->md_directskip_mode = best_md_directskip_mode;
        currMB->weighted_skipmode = weighted_skipmode_fix;
        tmp = Coeff_all_tmp_best;
        Coeff_all_tmp_best = img->Coeff_all;
        img->Coeff_all = tmp; ////////qyu 0823 store Coeff_all_tmp_best to img->Coeff_all_to_write

        //===== reconstruction values =====
        for (j = 0; j < currMB_pix_height; j++) {
            for (i = 0; i < currMB_pix_height; i++) {
                hc->imgY[pix_y + j][pix_x + i] = rec_mbY[j][i]; //save mode tmp best to IMG
                img->recon_currY[pix_y_InSMB + j][pix_x_InSMB + i] = rec_mbY[j][i]; //save mode tmp best to SMBtmp_best
                img->Coeff_all_to_write[pix_y_InSMB + j][pix_x_InSMB + i] = img->Coeff_all[j][i]; //save mode tmp best to SMB tmp_best
                img->Coeff_all_to_write_ALF[currSMB_nr][pix_y_InSMB + j][pix_x_InSMB + i] = img->Coeff_all[j][i];
            }
        }

        if (input->chroma_format == 1) {
            for (j = 0; j < (currMB_pix_height >> 1); j++) {
                for (i = 0; i < (currMB_pix_height >> 1); i++) {
                    hc->imgUV[0][pix_c_y + j][pix_c_x + i] = rec_mbU[j][i];
                    hc->imgUV[1][pix_c_y + j][pix_c_x + i] = rec_mbV[j][i];
                    img->recon_currU[pix_cy_InSMB + j][pix_cx_InSMB + i] = rec_mbU[j][i];
                    img->recon_currV[pix_cy_InSMB + j][pix_cx_InSMB + i] = rec_mbV[j][i];
                    img->Coeff_all_to_write[pix_cy_InSMB + SMB_pix_height + j][pix_cx_InSMB + i] = img->Coeff_all[j +
                            currMB_pix_height][i]; //U
                    img->Coeff_all_to_write[pix_cy_InSMB + SMB_pix_height + j][pix_cx_InSMB + (SMB_pix_height >> 1) + i] =
                        img->Coeff_all[j + currMB_pix_height][i + (currMB_pix_height >> 1) ];   //V
                    img->Coeff_all_to_write_ALF[currSMB_nr][pix_cy_InSMB + SMB_pix_height + j][pix_cx_InSMB + i] = img->Coeff_all[j +
                            currMB_pix_height][i]; //U
                    img->Coeff_all_to_write_ALF[currSMB_nr][pix_cy_InSMB + SMB_pix_height + j][pix_cx_InSMB +
                            (SMB_pix_height >> 1) + i] = img->Coeff_all[j + currMB_pix_height][i + (currMB_pix_height >> 1) ];     //V
                }
            }
        }

        //===============   cbp and mode   ===============
        for (j = 0; j < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
            for (i = 0; i < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); i++) {
                tmpMB = &img->mb_data[uiPositionInPic + i + j * img->PicWidthInMbs]; //save data to mb_data

#if MB_DQP
                if (input->useDQP) {
                    tmpMB->delta_qp      = dQP;
                    tmpMB->qp     = QP;
                    tmpMB->left_cu_qp = left_cu_qp;
                    tmpMB->previouse_qp = previouse_qp;
                } else {
                    tmpMB->delta_qp      = 0;
                    tmpMB->qp     = img->qp;
                }
#endif

                tmpMB->cbp      = cbp;
                tmpMB->cbp_blk = cbp_blk;
                tmpMB->cuType = mode;
                tmpMB->trans_size = best_trans_size;
                tmpMB->md_directskip_mode = best_md_directskip_mode;
                tmpMB->weighted_skipmode = weighted_skipmode_fix;
                tmpMB->ui_MbBitSize = uiBitSize;

                for (k = stage_block8x8_pos; k < 4; k++) {
                    tmpMB->b8mode[k]   = b8mode[k];
                    tmpMB->b8pdir[k]   = b8pdir[k];
                }
            }
        }

        //============== reference frames =========================
        for (j = stage_block8x8_pos / 2; j < 2; j++) {
            for (i = 0; i < 2; i++) {

                get_b8_offset(mode, uiBitSize, i , j , &col , &row, &width, &height);


                for (r = 0; r < height; r++) {
                    for (c = 0; c < width; c++) {
                        {
                            frefar[block_8y + row + r][block_8x + col + c] = frefframe[j][i];
                            frefar_curr[block8_InSMB_y + row + r][block8_InSMB_x + col + c] = frefframe[j][i]; //qyu 0830
                            if (img->type == F_IMG) {
                                hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = p_snd_frefframe[j][i];
                                p_snd_frefar_curr[block8_InSMB_y + row + r][block8_InSMB_x + col + c] = p_snd_frefframe[j][i]; //qyu 0830
                            }
                        }
                    }
                }
            }
        }

        if (bframe) {
            for (j = stage_block8x8_pos / 2; j < 2; j++) {
                for (i = 0; i < 2; i++) {
                    get_b8_offset(mode, uiBitSize, i , j , &col , &row, &width, &height);

                    for (r = 0; r < height; r++) {
                        for (c = 0; c < width; c++) {
                            brefar[block_8y + row + r][block_8x + col + c] = brefframe[j][i];
                            brefar_curr[block8_InSMB_y + row + r ][block8_InSMB_x + col + c ] =  brefframe[j][i];
                        }
                    }
                }
            }
        }

        //===============   intra pred mode   ===============
        if (img->mb_data[uiPositionInPic].cuType == I8MB || img->mb_data[uiPositionInPic].cuType == I16MB ||
            img->mb_data[uiPositionInPic].cuType == InNxNMB || img->mb_data[uiPositionInPic].cuType == INxnNMB) {
            for (j = 0; j < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
                for (i = 0; i < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); i++) {
                    tmpMB = &img->mb_data[uiPositionInPic + i + j * img->PicWidthInMbs]; //save data to mb_data

                    for (k = stage_block8x8_pos; k < 4; k++) {
                        tmpMB->intra_pred_modes[k] = best_intra_pred_modes_tmp[k];
                        tmpMB->real_intra_pred_modes[k] = best_real_intra_pred_modes_tmp[k];
                    }
                }
            }
            if (img->mb_data[uiPositionInPic].cuType == InNxNMB) {
                for (j = stage_block8x8_pos / 2; j < 2; j++) {
                    for (i = 0; i < 2; i++) {
                        row = ((j << 1) + i) * (1 << (uiBitSize - 4));
                        for (c = 0; c < (1 << (uiBitSize + 1 - MIN_CU_SIZE_IN_BIT)); c++) {
                            if (uiBitSize == 4) {
                                img->ipredmode[1 + block_8y + row][1 + block_8x + c] = best_ipredmode_tmp[j][i];
                                img->ipredmode_curr[block8_InSMB_y + row ][block8_InSMB_x + c ] = best_ipredmode_tmp[j][i];
                            } else {
                                img->ipredmode[1 + block_8y + row][1 + block_8x + c] = best_ipredmode_tmp[j][i];
                                img->ipredmode[1 + block_8y + row + 1][1 + block_8x + c] = best_ipredmode_tmp[j][i];
                                img->ipredmode_curr[block8_InSMB_y + row ][block8_InSMB_x + c ] = best_ipredmode_tmp[j][i];
                                img->ipredmode_curr[block8_InSMB_y + row + 1 ][block8_InSMB_x + c ] = best_ipredmode_tmp[j][i];

                            }
                        }
                    }
                }
            } else if (img->mb_data[uiPositionInPic].cuType == INxnNMB) {
                for (j = stage_block8x8_pos / 2; j < 2; j++) {
                    for (i = 0; i < 2; i++) {
                        row = ((j << 1) + i) * (1 << (uiBitSize - 4));
                        for (c = 0; c < (1 << (uiBitSize + 1 - MIN_CU_SIZE_IN_BIT)); c++) {
                            if (uiBitSize == 4) {
                                img->ipredmode[1 + block_8y + c][1 + block_8x + row] = best_ipredmode_tmp[j][i];
                                img->ipredmode_curr[block8_InSMB_y + c ][block8_InSMB_x + row ] = best_ipredmode_tmp[j][i];
                            } else {
                                img->ipredmode[1 + block_8y + c][1 + block_8x + row] = best_ipredmode_tmp[j][i];
                                img->ipredmode[1 + block_8y + c][1 + block_8x + row + 1] = best_ipredmode_tmp[j][i];
                                img->ipredmode_curr[block8_InSMB_y + c ][block8_InSMB_x + row ] = best_ipredmode_tmp[j][i];
                                img->ipredmode_curr[block8_InSMB_y + c ][block8_InSMB_x + row + 1 ] = best_ipredmode_tmp[j][i];

                            }
                        }
                    }
                }
            } else {
                for (j = stage_block8x8_pos / 2; j < 2; j++) {
                    for (i = 0; i < 2; i++) {
                        row = (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)) * j;
                        col = (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)) * i;

                        for (r = 0; r < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); r++) {
                            for (c = 0; c < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); c++) {
                                img->ipredmode[1 + block_8y + row + r][1 + block_8x + col + c] = best_ipredmode_tmp[j][i];
                                img->ipredmode_curr[block8_InSMB_y + row + r ][block8_InSMB_x + col + c ] =  best_ipredmode_tmp[j][i]; //qyu 0830
                            }
                        }
                    }
                }
            }
        } else {
            for (j = 0; j < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); j++) {
                for (i = 0; i < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); i++) {
                    img->ipredmode[block_8y + j + 1][block_8x + i + 1] = -1;
                    img->ipredmode_curr[block8_InSMB_y + j][block8_InSMB_x + i] = -1;
                }
            }

            for (j = 0; j < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); j++) {
                for (i = 0; i < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); i++) {
                    tmpMB = &img->mb_data[uiPositionInPic + i + j * img->PicWidthInMbs]; //save data to mb_data
                    tmpMB->intra_pred_modes[0] = DC_PRED;
                    tmpMB->intra_pred_modes[1] = DC_PRED;
                    tmpMB->intra_pred_modes[2] = DC_PRED;
                    tmpMB->intra_pred_modes[3] = DC_PRED;
                }
            }
        }

        //==== intra prediction modes ====
        for (j = 0; j < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
            for (i = 0; i < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); i++) {
                tmpMB = &img->mb_data[uiPositionInPic + i + j * img->PicWidthInMbs]; //save data to mb_data
                tmpMB->c_ipred_mode   = best_c_imode;
            }
        }

        //==== motion vectors =====
        SetMotionVectorsMB(&img->mb_data[uiPositionInPic], bframe, uiBitSize, uiPositionInPic);    //store mv

        if (img->type != INTRA_IMG) {
            for (j = 0; j < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
                for (i = 0; i < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); i++) {
                    img->currMv[block8_InSMB_y + j][block8_InSMB_x + i][0] = img->tmp_mv[block_8y + j][block_8x + i][0];
                    img->currMv[block8_InSMB_y + j][block8_InSMB_x + i][1] = img->tmp_mv[block_8y + j][block_8x + i][1];
                    img->currMv[block8_InSMB_y + j][block8_InSMB_x + i][2] = img->tmp_mv[block_8y + j][block_8x + i][2];
                    if (img->type == F_IMG) {
                        img->currPSndMv[block8_InSMB_y + j][block8_InSMB_x + i][0] = img->p_snd_tmp_mv[block_8y + j][block_8x + i][0];
                        img->currPSndMv[block8_InSMB_y + j][block8_InSMB_x + i][1] = img->p_snd_tmp_mv[block_8y + j][block_8x + i][1];
                        img->currPSndMv[block8_InSMB_y + j][block8_InSMB_x + i][2] = img->p_snd_tmp_mv[block_8y + j][block_8x + i][2];
                    }

                    if (input->successive_Bframe != 0) {
                        img->currFwMv[block8_InSMB_y + j][block8_InSMB_x + i][1] = img->fw_mv[block_8y + j][block_8x + i][1];
                        img->currFwMv[block8_InSMB_y + j][block8_InSMB_x + i][0] = img->fw_mv[block_8y + j][block_8x + i][0];
                        img->currFwMv[block8_InSMB_y + j][block8_InSMB_x + i][2] = img->fw_mv[block_8y + j][block_8x + i][2];
                        img->currBwMv[block8_InSMB_y + j][block8_InSMB_x + i][0] = img->bw_mv[block_8y + j][block_8x + i][0];
                        img->currBwMv[block8_InSMB_y + j][block8_InSMB_x + i][1] = img->bw_mv[block_8y + j][block_8x + i][1];
                        img->currBwMv[block8_InSMB_y + j][block8_InSMB_x + i][2] = img->bw_mv[block_8y + j][block_8x + i][2];
                    }
                }
            }
        }
        //if (  IS_INTERMV ( currMB )||(currMB->cuType==0 && img->type==P_IMG))
        if (!IS_INTRA(currMB)) {
            for (j = 0; j < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
                for (i = 0; i < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); i++) {
                    tmpMB = &img->mb_data[uiPositionInPic + i + j * img->PicWidthInMbs]; //save data to mb_data
                    storeMotionInfo(tmpMB, uiBitSize, uiPositionInPic , stage_block8x8_pos / 2, g_best_dmh_mode);    // store mvd
                }
            }
        } else {
            for (j = 0; j < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
                for (i = 0; i < (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)); i++) {
                    tmpMB = &img->mb_data[uiPositionInPic + i + j * img->PicWidthInMbs]; //save data to mb_data

                    for (r = 0; r < 2; r++) {
                        for (c = 0; c < 2; c++) {
                            for (k = 0; k < 2; k++) {
                                tmpMB->mvd[r][c][k][0] = tmpMB->mvd[r][c][k][1] = tmpMB->mvd[r][c][k][2] = 0;
                            }
                        }
                    }
                }
            }
        }

        reset_coding_state(cs_submb_best);    //[uiBitSize - 4] );
        store_coding_state(cs_next_best);
    } else {
        //===== reconstruction values =====
        for (j = 0; j < (1 << uiBitSize); j++) {
            for (i = 0; i < (1 << uiBitSize); i++) {
                hc->imgY[pix_y + j][pix_x + i] = img->recon_currY[pix_y_InSMB + j][pix_x_InSMB + i];
            }
        }

        if (input->chroma_format == 1) {
            for (j = 0; j < (1 << (uiBitSize - 1)); j++) {
                for (i = 0; i < (1 << (uiBitSize - 1)); i++) {
                    hc->imgUV[0][pix_c_y + j][pix_c_x + i] = img->recon_currU[pix_cy_InSMB + j][pix_cx_InSMB + i];
                    hc->imgUV[1][pix_c_y + j][pix_c_x + i] = img->recon_currV[pix_cy_InSMB + j][pix_cx_InSMB + i];
                }
            }
        }

        //============== reference frames =========================
        for (j = stage_block8x8_pos / 2; j < 2; j++) {
            for (i = 0; i < 2; i++) {
                get_b8_offset(mode, uiBitSize, i , j , &col , &row, &width, &height);

                for (r = 0; r < height; r++) {
                    for (c = 0; c < width; c++) {
                        frefar[block_8y + row + r][block_8x + col + c] = frefar_curr[block8_InSMB_y + row + r ][block8_InSMB_x + col + c ];
                        if (img->type == F_IMG) {
                            hc->p_snd_refFrArr[block_8y + row + r][block_8x + col + c] = p_snd_frefar_curr[block8_InSMB_y + row + r][block8_InSMB_x
                                    +
                                    col + c ];
                        }
                    }
                }
            }
        }

        if (bframe) {
            for (j = stage_block8x8_pos / 2; j < 2; j++) {
                for (i = 0; i < 2; i++) {

                    get_b8_offset(mode, uiBitSize, i , j , &col , &row, &width, &height);


                    for (r = 0; r < height; r++) {
                        for (c = 0; c < width; c++) {
                            brefar[block_8y + row + r][block_8x + col + c] = brefar_curr[block8_InSMB_y + row + r ][block8_InSMB_x + col + c ];
                        }
                    }
                }
            }
        }

        //===============   intra pred mode   ===============

        for (j = 0; j < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); j++) {
            for (i = 0; i < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); i++) {
                img->ipredmode[block_8y + j + 1][block_8x + i + 1] = img->ipredmode_curr[block8_InSMB_y + j][block8_InSMB_x + i];
            }
        }

        //===============   motion vector   ===============
        if (img->type != INTRA_IMG) {
            for (j = 0; j < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); j++) {       //uiBitSize 4:1 5:2x2 6 4x4
                for (i = 0; i < (1 << (uiBitSize - MIN_BLOCK_SIZE_IN_BIT)); i++) {
                    img->tmp_mv[block_8y + j][block_8x + i][0] = img->currMv[block8_InSMB_y + j][block8_InSMB_x + i][0];
                    img->tmp_mv[block_8y + j][block_8x + i][1] = img->currMv[block8_InSMB_y + j][block8_InSMB_x + i][1];
                    img->tmp_mv[block_8y + j][block_8x + i][2] = img->currMv[block8_InSMB_y + j][block8_InSMB_x + i][2];
                    if (img->type == F_IMG) {
                        img->p_snd_tmp_mv[block_8y + j][block_8x + i][0] = img->currPSndMv[block8_InSMB_y + j][block8_InSMB_x + i][0];
                        img->p_snd_tmp_mv[block_8y + j][block_8x + i][1] = img->currPSndMv[block8_InSMB_y + j][block8_InSMB_x + i][1];
                        img->p_snd_tmp_mv[block_8y + j][block_8x + i][2] = img->currPSndMv[block8_InSMB_y + j][block8_InSMB_x + i][2];
                    }

                    if (input->successive_Bframe != 0) {
                        img->fw_mv[block_8y + j][block_8x + i][0] = img->currFwMv[block8_InSMB_y + j][block8_InSMB_x + i][0];
                        img->bw_mv[block_8y + j][block_8x + i][0] = img->currBwMv[block8_InSMB_y + j][block8_InSMB_x + i][0];
                        img->fw_mv[block_8y + j][block_8x + i][1] = img->currFwMv[block8_InSMB_y + j][block8_InSMB_x + i][1];
                        img->bw_mv[block_8y + j][block_8x + i][1] = img->currBwMv[block8_InSMB_y + j][block8_InSMB_x + i][1];
                        img->fw_mv[block_8y + j][block_8x + i][2] = img->currFwMv[block8_InSMB_y + j][block8_InSMB_x + i][2];
                        img->bw_mv[block_8y + j][block_8x + i][2] = img->currBwMv[block8_InSMB_y + j][block8_InSMB_x + i][2];
                    }
                }
            }
        }

        reset_coding_state(cs_next_best);
    }
}

/*
*************************************************************************
* Function:Set reference frames and motion vectors
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void SetRefAndMotionVectors(unsigned int uiBitSize, unsigned int uiPositionInPic, int block, int mode, int ref,
                            int bw_ref, int pdir)  /*lgp*13*/
{
    int i, j;
    int     bframe       = (img->type == B_IMG);

    int     pmode         = (mode >= P2NX2N ? mode : PNXN);


    int   **frefArr      = (bframe ? img->fw_refFrArr : hc->refFrArr);
    int   **brefArr      = (bframe ? img->bw_refFrArr : hc->p_snd_refFrArr);
    int ***  bmvArr       = (bframe ? img->bw_mv : img->p_snd_tmp_mv);
    int ** *** allBwMv_arr = (pdir == BID &&
                              mode != PSKIPDIRECT) ? img->allBidBwMv : pdir == DUAL ? img->allDualSndMv : img->allBwMv;
    int ***  fmvArr  = (bframe ? img->fw_mv    : img->tmp_mv);
    int ** *** allFwMv_arr = (pdir == SYM && mode != PSKIPDIRECT) ? img->allSymMv : (pdir == BID &&
                             mode != 0) ? img->allBidFwMv : pdir == DUAL ? img->allDualFstMv : img->allFwMv;

    int   block_x        = (uiPositionInPic % img->PicWidthInMbs) << 1;   //img->block8_x;
    int   block_y        = (uiPositionInPic / img->PicWidthInMbs) << 1;

    int     i0, j0, i1, j1, h, v;
    h = (g_blk_size[pmode * 2][0] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT  ;
    v = (g_blk_size[pmode * 2][1] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT ;

    h = (h == 0) ? 1 : h;
    v = (v == 0) ? 1 : v;
    i0 = (block % 2) * h;
    j0 = (block / 2) * v;

    h = (g_blk_size[pmode * 2 + block][0] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT  ;
    v = (g_blk_size[pmode * 2 + block][1] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT ;
    h = (h == 0) ? 1 : h;
    v = (v == 0) ? 1 : v;
    i1 = i0 + h;
    j1 = j0 + v;

    if (h > v) {
        h = h / 2;
    } else if (v > h) {
        v = v / 2;
    }


    if ((pdir == FORWARD || pdir == SYM || pdir == BID || pdir == DUAL) && (mode != IBLOCK && mode != PSKIPDIRECT)) {
        for (j = j0; j < j1; j++) {
            for (i = i0; i < i1; i++) {
                fmvArr[block_y + j][block_x + i][0] = allFwMv_arr[ j / v ][ i / h ][ref][mode][0];
                fmvArr[block_y + j][block_x + i][1] = allFwMv_arr[ j / v ][ i / h ][ref][mode][1];
                frefArr  [block_y + j][block_x + i  ] = ref;

            }
        }
    } else {
        if (!mode && bframe) {
            for (j = j0; j < j1; j++) {
                for (i = i0; i < i1; i++) {
                    ref = 0;

                    if (ref == -1) {
                        fmvArr[block_y + j][block_x + i][0] = 0;
                        fmvArr[block_y + j][block_x + i][1] = 0;
                        frefArr  [block_y + j][block_x + i  ] = -1;
                    } else {
                        fmvArr[block_y + j][block_x + i][0] = allFwMv_arr[ j / v ][ i / h ][ref][mode][0];
                        fmvArr[block_y + j][block_x + i][1] = allFwMv_arr[ j / v ][ i / h ][ref][mode][1];

                        ref = 0;
                        frefArr  [block_y + j][block_x + i  ] = ref;
                    }
                }
            }
        } else {
            for (j = j0; j < j1; j++) {
                for (i = i0; i < i1; i++) {
                    fmvArr[block_y + j][block_x + i][0] = 0;
                    fmvArr[block_y + j][block_x + i][1] = 0;
                    frefArr  [block_y + j][block_x + i  ] = -1;
                }
            }
        }
    }

    if ((pdir == BACKWARD || pdir == SYM || pdir == BID || pdir == DUAL) && (mode != IBLOCK && mode != PSKIPDIRECT)) {
        for (j = j0; j < j1; j++) {
            for (i = i0; i < i1; i++) {
                if (pdir == SYM) {
                    int delta_P, TRp, DistanceIndexFw, DistanceIndexBw, refframe, delta_PB;
                    refframe = ref;
                    delta_P = 2 * (img->imgtr_next_P - fref[0]->imgtr_fwRefDistance);
                    delta_P = (delta_P + 512) % 512;

                    TRp = (refframe + 1) * delta_P;    //the lates backward reference

                    delta_PB = 2 * (hc->picture_distance - fref[0]->imgtr_fwRefDistance);
                    TRp = (TRp + 512) % 512;
                    delta_PB = (delta_PB + 512) % 512;
                    DistanceIndexFw = delta_PB;

                    DistanceIndexBw    = (TRp - DistanceIndexFw + 512) % 512;

#if MV_SCALE
                    bmvArr[block_y + j][block_x + i][0] = - scale_mv(allFwMv_arr[ j / v ][ i / h ][ref][mode][0], DistanceIndexBw,
                                                          DistanceIndexFw);
                    bmvArr[block_y + j][block_x + i][1] = - scale_mv(allFwMv_arr[ j / v ][ i / h ][ref][mode][1], DistanceIndexBw,
                                                          DistanceIndexFw);
#if HALF_PIXEL_COMPENSATION_MVD
                    if (img->is_field_sequence) {
                        bmvArr[block_y + j][block_x + i][1] = -scale_mv_y2(allFwMv_arr[j / v][i / h][ref][mode][1], DistanceIndexBw,
                                                              DistanceIndexFw);
                    }
#endif
#else
                    bmvArr[block_y + j][block_x + i][0] = - ((allFwMv_arr[ j / v ][ i / h ][ref][mode][0] * DistanceIndexBw *
                                                          (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
                    bmvArr[block_y + j][block_x + i][1] = - ((allFwMv_arr[ j / v ][ i / h ][ref][mode][1] * DistanceIndexBw *
                                                          (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET);
#if HALF_PIXEL_COMPENSATION_MVD
                    if (img->is_field_sequence) {
                        int delta, delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC;
                        oriPOC = 2 * hc->picture_distance;
                        oriRefPOC = oriPOC - DistanceIndexFw;
                        scaledPOC = 2 * hc->picture_distance;
                        scaledRefPOC = scaledPOC - DistanceIndexBw;
                        getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
                        bmvArr[block_y + j][block_x + i][1] = - (((allFwMv_arr[ j / v ][ i / h ][ref][mode][1] + delta) * DistanceIndexBw *
                                                              (MULTI / DistanceIndexFw) + HALF_MULTI) >> OFFSET) - delta2;
                    }
#endif
#endif
                    bmvArr[block_y + j][block_x + i][0] = Clip3(-32768, 32767, bmvArr[block_y + j][block_x + i][0]);
                    bmvArr[block_y + j][block_x + i][1] = Clip3(-32768, 32767, bmvArr[block_y + j][block_x + i][1]);
                } else if (pdir == DUAL) {
                    bmvArr[block_y + j][block_x + i][0] = allBwMv_arr[ j / v ][ i / h ][ref][mode][0];
                    bmvArr[block_y + j][block_x + i][1] = allBwMv_arr[ j / v ][ i / h ][ref][mode][1];
                } else {
                    bmvArr[block_y + j][block_x + i][0] = allBwMv_arr[ j / v ][ i / h ][bw_ref][mode][0];
                    bmvArr[block_y + j][block_x + i][1] = allBwMv_arr[ j / v ][ i / h ][bw_ref][mode][1];
                }

                brefArr  [block_y + j][block_x + i  ] = bw_ref; /*lgp*13*/
            }
        }
    } else if (bframe) {
        if (!mode) {
            for (j = j0; j < j1; j++) {
                for (i = i0; i < i1; i++) {
                    bmvArr[block_y + j][block_x + i][0] = allBwMv_arr[ j / v ][ i / h ][0][mode][0];
                    bmvArr[block_y + j][block_x + i][1] = allBwMv_arr[ j / v ][ i / h ][0][mode][1];
                    ref = hc->refFrArr[block_y + j][block_x + i];
                    brefArr  [block_y + j][block_x + i  ] = 0;  // min(ref,0);
                }
            }
        } else {
            for (j = j0; j < j1; j++) {
                for (i = i0; i < i1; i++) {
                    bmvArr[block_y + j][block_x + i][0] = 0;
                    bmvArr[block_y + j][block_x + i][1] = 0;
                    brefArr  [block_y + j][block_x + i  ] = -1;
                }
            }
        }
    } else if (img->type == F_IMG) {
        for (j = j0; j < j1; j++) {
            for (i = i0; i < i1; i++) {
                bmvArr[block_y + j][block_x + i][0] = 0;
                bmvArr[block_y + j][block_x + i][1] = 0;
                brefArr  [block_y + j][block_x + i  ] = -1;
            }
        }
    }
}


void SetRef(int *adjust_ref, int *max_ref)
{
    *adjust_ref  = (img->type == B_IMG ?  1 : 0);
    *adjust_ref  = min(*adjust_ref, *max_ref - 1);

    *max_ref = min(img->num_of_references, img->buf_cycle);
    *adjust_ref = 0;

    if (*max_ref > 1 && img->type == B_IMG) {
        *max_ref = 1;
    }
}
void ME(unsigned int uiBitSize, unsigned int uiPositionInPic, int mode, int bframe, int max_ref, int adjust_ref,
        double lambda_motion, int write_ref, int lambda_motion_factor, int *valid,    double lambda_mode, codingUnit *currMB ,
        int dualpred_enabled)  //,int **ipredmodes)
{
    int  fw_mcost, bw_mcost, sym_mcost;

    int bid_mcost, bid_best_fw_ref = 0, bid_best_bw_ref = 0;

    double  min_rdcost, rdcost;
    int  stage_block8x8_pos = 0;
    int  best_fw_ref = 0, best_bw_ref = 0, sym_best_fw_ref = 0, sym_best_bw_ref = 0, best_pdir = FORWARD;
    int  dual_best_fst_ref = 0, dual_best_snd_ref = 0, dual_mcost = 1 << 30;
    int  block;
    int  index;
    int  curr_cbp_blk = 0, cnt_nonz = 0;
    static const int  b8_mode_table[2]  = {PSKIPDIRECT, PNXN};         // DO NOT CHANGE ORDER !!!



    sym_mcost = bid_mcost = 1 << 30;


    //===== set direct motion vectors =====


    //===== MOTION ESTIMATION FOR 16x16, 16x8, 8x16, AMP BLOCKS =====
    if (mode > PSKIPDIRECT && mode < PNXN)

    {
        if (valid[mode]) {
            for (block = stage_block8x8_pos / 2; block < (mode == 1 ? 1 : 2); block++) {
                ForwardPred(&fw_mcost, & best_fw_ref, &best_bw_ref,  max_ref,  adjust_ref, mode,  block,   lambda_motion,  write_ref,
                            lambda_motion_factor, uiPositionInPic, uiBitSize);

                if (img->type == F_IMG && input->dhp_enabled  && dualpred_enabled && max_ref > 1 && (!(uiBitSize == B8X8_IN_BIT &&
                        mode >= P2NXN && mode <= PVER_RIGHT))) {
                    DualPred(&dual_mcost, & dual_best_fst_ref, &dual_best_snd_ref, max_ref,  adjust_ref, mode,  block,   lambda_motion,
                             write_ref, lambda_motion_factor, uiPositionInPic, uiBitSize);
                }
                if (bframe) {

                    if (uiBitSize == B8X8_IN_BIT && mode >= P2NXN && mode <= PVER_RIGHT) {
                        BackwardPred(&best_bw_ref, &bw_mcost, mode,  block,  lambda_motion, max_ref,  adjust_ref, uiPositionInPic,
                                     uiBitSize);
                    } else {
                        BiPred(&best_bw_ref, &bw_mcost, &sym_mcost, &sym_best_fw_ref, &sym_best_bw_ref, &bid_mcost, &bid_best_fw_ref,
                               &bid_best_bw_ref, mode,  block,  lambda_motion, max_ref,  adjust_ref, uiPositionInPic, uiBitSize);
                    }


                    if (fw_mcost <= bw_mcost && fw_mcost <= sym_mcost && fw_mcost <= bid_mcost) {
                        best_pdir = FORWARD;
                        best_bw_ref = 0;
                    } else if (bw_mcost <= fw_mcost && bw_mcost <= sym_mcost && bw_mcost <= bid_mcost) {
                        best_pdir = BACKWARD;
                        best_fw_ref = 0;
                    } else if (sym_mcost <= fw_mcost && sym_mcost <= bw_mcost && sym_mcost <= bid_mcost) {
                        best_pdir = SYM;
                    } else {
                        best_pdir = BID;
                    }


                } else { // if (bframe)
                    if (fw_mcost <= dual_mcost) {
                        best_pdir = FORWARD;
                    } else {
                        best_pdir = DUAL;
                    }
                }

                //----- set reference frame and direction parameters -----

                if ((mode == PNX2N) || (mode == PVER_LEFT) || (mode == PVER_RIGHT)) {
                    best8x8ref [mode][  block] = best8x8ref [mode][  block + 2] = best_fw_ref;
                    best8x8pdir[mode][  block] = best8x8pdir[mode][  block + 2] = best_pdir;
                    best8x8bwref   [mode][block] = best8x8bwref   [mode][block + 2] = best_bw_ref;
                    best8x8syoneForthRefY  [mode][block][0] = best8x8syoneForthRefY   [mode][block + 2][0] = sym_best_fw_ref;
                    best8x8syoneForthRefY  [mode][block][1] = best8x8syoneForthRefY   [mode][block + 2][1] = sym_best_bw_ref;

                    best8x8bidref  [mode][block][0] = best8x8bidref   [mode][block + 2][0] = bid_best_fw_ref;
                    best8x8bidref  [mode][block][1] = best8x8bidref   [mode][block + 2][1] = bid_best_bw_ref;
                    best8x8dualref  [mode][block][0] = best8x8dualref   [mode][block + 2][0] = dual_best_fst_ref;
                    best8x8dualref  [mode][block][1] = best8x8dualref   [mode][block + 2][1] = dual_best_snd_ref;
                } else if ((mode == P2NXN) || (mode == PHOR_UP) || (mode == PHOR_DOWN)) {
                    best8x8ref [mode][2 * block] = best8x8ref [mode][2 * block + 1] = best_fw_ref;
                    best8x8pdir[mode][2 * block] = best8x8pdir[mode][2 * block + 1] = best_pdir;
                    best8x8bwref   [mode][2 * block] = best8x8bwref   [mode][2 * block + 1] = best_bw_ref;
                    best8x8syoneForthRefY   [mode][2 * block][0] = best8x8syoneForthRefY   [mode][2 * block + 1][0] = sym_best_fw_ref;
                    best8x8syoneForthRefY   [mode][2 * block][1] = best8x8syoneForthRefY   [mode][2 * block + 1][1] = sym_best_bw_ref;

                    best8x8bidref  [mode][2 * block][0] = best8x8bidref   [mode][2 * block + 1][0] = bid_best_fw_ref;
                    best8x8bidref  [mode][2 * block][1] = best8x8bidref   [mode][2 * block + 1][1] = bid_best_bw_ref;
                    best8x8dualref  [mode][2 * block][0] = best8x8dualref   [mode][2 * block + 1][0] = dual_best_fst_ref;
                    best8x8dualref  [mode][2 * block][1] = best8x8dualref   [mode][2 * block + 1][1] = dual_best_snd_ref;
                } else if (mode == P2NX2N) {
                    best8x8ref [mode][0] = best8x8ref [mode][1] = best8x8ref [mode][2] = best8x8ref [mode][3] = best_fw_ref;
                    best8x8pdir[mode][0] = best8x8pdir[mode][1] = best8x8pdir[mode][2] = best8x8pdir[mode][3] = best_pdir;
                    best8x8bwref   [mode][0] = best8x8bwref   [mode][1] = best8x8bwref   [mode][2] = best8x8bwref   [mode][3] = best_bw_ref;
                    best8x8syoneForthRefY [mode][0][0] = best8x8syoneForthRefY [mode][1][0] = best8x8syoneForthRefY [mode][2][0] =
                            best8x8syoneForthRefY [mode][3][0] = sym_best_fw_ref;
                    best8x8syoneForthRefY [mode][0][1] = best8x8syoneForthRefY [mode][1][1] = best8x8syoneForthRefY [mode][2][1] =
                            best8x8syoneForthRefY [mode][3][1] = sym_best_bw_ref;

                    best8x8bidref  [mode][0][0] = best8x8bidref   [mode][1][0] = best8x8bidref  [mode][2][0] = best8x8bidref   [mode][3][0]
                                                  = bid_best_fw_ref;
                    best8x8bidref  [mode][0][1] = best8x8bidref   [mode][1][1] = best8x8bidref  [mode][2][1] = best8x8bidref   [mode][3][1]
                                                  = bid_best_bw_ref;
                    best8x8dualref  [mode][0][0] = best8x8dualref   [mode][1][0] = best8x8dualref  [mode][2][0] =
                                                       best8x8dualref   [mode][3][0] = dual_best_fst_ref;
                    best8x8dualref  [mode][0][1] = best8x8dualref   [mode][1][1] = best8x8dualref  [mode][2][1] =
                                                       best8x8dualref   [mode][3][1] = dual_best_snd_ref;
                }

                //--- set reference frames and motion vectors ---
                if (mode > P2NX2N && block == 0) {
                    SetRefAndMotionVectors(uiBitSize, uiPositionInPic, block, mode,
                                           best_pdir == SYM ? sym_best_fw_ref : best_pdir == BID ?  bid_best_fw_ref : best_pdir == DUAL ? dual_best_fst_ref :
                                           best_fw_ref,
                                           best_pdir == SYM ? sym_best_bw_ref : best_pdir == BID ? bid_best_bw_ref : best_pdir == DUAL ? dual_best_snd_ref :
                                           best_bw_ref, best_pdir);
                }
            }
        } // if (valid[mode])
    } // for (mode=3; mode>0; mode--)*/


    if ((mode == PNXN) && (valid[PSKIPDIRECT] || valid[PNXN]))

    {
        //===== store coding state of codingUnit =====
        store_coding_state(cs_mb);


        currMB->trans_size = 1;
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (block = stage_block8x8_pos; block < 4; block++) {
            //--- set coordinates ---
            //=====  LOOP OVER POSSIBLE CODING MODES FOR 8x8 SUB-PARTITION  =====
            for (min_rdcost = 1e30,  index = (bframe ? 0 : 1); index < 2; index++) {     // mv_range, 20071009
                if (valid[mode = b8_mode_table[index]]) {
                    curr_cbp_blk = 0;

                    if (mode == PSKIPDIRECT) {
                        best_fw_ref = -1;
                        best_pdir   =  SYM;
                    } // if (mode==0)
                    else {
                        ForwardPred(&fw_mcost, & best_fw_ref, &best_bw_ref,  max_ref,  adjust_ref, mode,  block,   lambda_motion,  write_ref,
                                    lambda_motion_factor, uiPositionInPic, uiBitSize);
                        if (img->type == F_IMG && input->dhp_enabled  && dualpred_enabled && max_ref > 1 && (!(uiBitSize == B8X8_IN_BIT &&
                                mode >= P2NXN && mode <= PVER_RIGHT))) {
                            DualPred(&dual_mcost, & dual_best_fst_ref, &dual_best_snd_ref, max_ref,  adjust_ref, mode,  block,   lambda_motion,
                                     write_ref, lambda_motion_factor, uiPositionInPic, uiBitSize);
                        }
                        if (bframe) {

                            if (uiBitSize == B8X8_IN_BIT && mode >= P2NXN && mode <= PVER_RIGHT) {
                                BackwardPred(&best_bw_ref, &bw_mcost, mode,  block,  lambda_motion, max_ref,  adjust_ref, uiPositionInPic,
                                             uiBitSize);
                            } else {
                                BiPred(&best_bw_ref, &bw_mcost, &sym_mcost, &sym_best_fw_ref, &sym_best_bw_ref, &bid_mcost, &bid_best_fw_ref,
                                       &bid_best_bw_ref, mode,  block,  lambda_motion, max_ref,  adjust_ref, uiPositionInPic, uiBitSize);
                            }


                            if (fw_mcost <= bw_mcost && fw_mcost <= sym_mcost && fw_mcost <= bid_mcost) {
                                best_pdir = FORWARD;
                                best_bw_ref = 0;
                            } else if (bw_mcost <= fw_mcost && bw_mcost <= sym_mcost && bw_mcost <= bid_mcost) {
                                best_pdir = BACKWARD;
                                best_fw_ref = 0;
                            } else if (sym_mcost <= fw_mcost && sym_mcost <= bw_mcost && sym_mcost <= bid_mcost) {
                                best_pdir = SYM;
                            } else {
                                best_pdir = BID;

                            }

                        } else { // if (bframe)
                            if (fw_mcost <= dual_mcost) {
                                best_pdir = FORWARD;
                            } else {
                                best_pdir = DUAL;
                            }

                        }
                    } // if (mode1!=0)

                    //--- store coding state before coding with current mode1 ---
                    store_coding_state(cs_cm);

                    if (bframe) {

                        rdcost = RDCost_for_8x8blocks(currMB, uiBitSize - 1, uiPositionInPic, &cnt_nonz, &curr_cbp_blk, lambda_mode, block,
                                                      mode, best_pdir,
                                                      best_pdir == SYM ? sym_best_fw_ref : best_pdir == BID ? bid_best_fw_ref : best_fw_ref,
                                                      best_pdir == SYM ? sym_best_bw_ref : best_pdir == BID ? bid_best_bw_ref : best_bw_ref);  /*lgp*13*/


                    } else {
                        rdcost = 1 << 29;
                    }

                    if (!img->mv_range_flag && !mode && input->rdopt) {
                        rdcost = (1 << 30);
                        img->mv_range_flag = 1;
                    }

                    //--- set variables if best mode1 has changed ---
                    if ((input->rdopt && rdcost < min_rdcost)) {
                        min_rdcost                 = rdcost;
                        best8x8mode        [block] = mode;
                        best8x8pdir  [PNXN][block] = best_pdir;
                        best8x8ref   [PNXN][block] = best_fw_ref;
                        best8x8bwref   [PNXN][block] = best_bw_ref;
                        best8x8syoneForthRefY[PNXN][block][0] = sym_best_fw_ref;
                        best8x8syoneForthRefY[PNXN][block][1] = sym_best_bw_ref;

                        best8x8bidref[PNXN][block][0] = bid_best_fw_ref;
                        best8x8bidref[PNXN][block][1] = bid_best_bw_ref;
                        best8x8dualref[PNXN][block][0] = dual_best_fst_ref;
                        best8x8dualref[PNXN][block][1] = dual_best_snd_ref;

                        //--- store number of nonzero coefficients ---

                        //--- store coding state ---
                        store_coding_state(cs_b8);
                    } // if (rdcost <= min_rdcost)

                    //--- re-set coding state as it was before coding with current mode1 was performed ---
                    reset_coding_state(cs_cm);
                } // if (valid[mode1=b8_mode_table[index]])
            } // for (min_rdcost=1e30, index=(bframe?0:1); index<6; index++)

            mode = best8x8mode[block];

            if (block < 3) {
                //===== set motion vectors and reference frames (prediction) =====
                SetRefAndMotionVectors(uiBitSize, uiPositionInPic, block, mode,
                                       best8x8pdir[PNXN][block] == SYM ? best8x8syoneForthRefY[PNXN][block][0] :
                                       best8x8pdir[PNXN][block] == BID ? best8x8bidref[PNXN][block][0] : best8x8pdir[PNXN][block] == DUAL ?
                                       best8x8dualref[PNXN][block][0] : best8x8ref[PNXN][block], best8x8pdir[PNXN][block] == SYM ?
                                       best8x8syoneForthRefY[PNXN][block][1] :  best8x8pdir[PNXN][block] == BID ? best8x8bidref[PNXN][block][1] :
                                       best8x8pdir[PNXN][block] == DUAL ? best8x8dualref[PNXN][block][1] : best8x8bwref[PNXN][block],
                                       best8x8pdir[PNXN][block]);
            } // if (block<3)

            //===== set the coding state after current block =====
            reset_coding_state(cs_b8);
        } // for ( block=0; block<4; block++)

        //--- re-set coding state (as it was before 8x8 block coding) ---
        reset_coding_state(cs_mb);
    }
}
/*
*************************************************************************
* Function:Mode Decision for a codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
double encode_SubMB(unsigned int uiBitSize, unsigned int uiPositionInPic, double minRdCostD, double split_cost_down,
                    double split_cost_up, double splitRdCost)
{
    int         valid[MAXMODE];
    double      lambda_motion, min_rdcost, max_rdcost = 1e30;
    int         lambda_motion_factor;
    int         write_ref   = input->no_multpred > 1;
    int         max_ref     = img->num_of_references;
    int         adjust_ref;

    int         dmh_mode, max_dmh_mode;
    MCParam     cMCParam;

    int         dir;

#if AVS2_S2_FASTMODEDECISION
    int         temp = 0, sum = 0;
    int         sum_diff = 0;
    int         sum_diff_8x16_left = 0, sum_diff_8x16_right = 0;
    int         sum_diff_16x8_up = 0, sum_diff_16x8_down = 0;
    int         BGflag_8x8 = 0, BGflag_8x16_left = 0, BGflag_8x16_right = 0;
    int         BGflag_16x8_up = 0, BGflag_16x8_down = 0;
    int         pix_y         = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;
    int         pix_c_y       = input->chroma_format == 1 ? (pix_y >> 1) : pix_y;
    int         pix_x         = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int         pix_c_x       = pix_x >> 1;
    int         block8x8, block_x, block_y;
    int         o, p;
#endif

    int         dir_num;
    int         pframe      = img->type == P_IMG;

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#endif

    int         intra       = img->type == INTRA_IMG;
    int         bframe      = img->type == B_IMG;

    //     static const int  b8_mode_table[2]  = {0, 4};         // DO NOT CHANGE ORDER !!!
    static const int  mb_mode_table[13]  = {PSKIPDIRECT, P2NX2N, P2NXN, PNX2N, PHOR_UP, PHOR_DOWN, PVER_LEFT, PVER_RIGHT, PNXN, I16MB, I8MB, InNxNMB, INxnNMB}; // DO NOT CHANGE ORDER !!!


    codingUnit *currMB = (codingUnit *) malloc(sizeof(codingUnit));

    SetRef(&adjust_ref, &max_ref);

#if !RD1510_FIX_BG
    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) { //only one reference frame(G\GB) for S frame
        adjust_ref = 0;
        max_ref = 1;
    }
#endif


    img->mv_range_flag = 1;
    Init_Curr_codingUnit(currMB, uiBitSize, uiPositionInPic);
    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3

    //===== SET VALID MODES =====
    valid[I8MB]   = 1;
    if (uiBitSize > MIN_CU_SIZE_IN_BIT) {
        valid[I8MB] = 0;
    }

    valid[I16MB]  = 1;

    if ((input->useSDIP && uiBitSize == 5 && (img->pix_x + 32 <= img->width) && (img->pix_y + 32 <= img->height)) ||
        (input->useSDIP && uiBitSize == 4 && (img->pix_x + 16 <= img->width) &&
         (img->pix_y + 16 <= img->height))) { //只对32x8和8x32和16x4和4x16
        valid[InNxNMB]  = 1;
        valid[INxnNMB]  = 1;
    } else {
        valid[InNxNMB]  = 0;
        valid[INxnNMB]  = 0;
    }
    valid[PSKIPDIRECT]      = (!intra);
    valid[P2NX2N]      = (!intra && input->InterSearch16x16);
    valid[P2NXN]      = (!intra && input->InterSearch16x8);
    valid[PNX2N]      = (!intra && input->InterSearch8x16);

    valid[PHOR_UP]         = uiBitSize >= B16X16_IN_BIT ? (!intra && input->InterSearchAMP) : 0;   // 16*4
    valid[PHOR_DOWN]       = uiBitSize >= B16X16_IN_BIT ? valid[4] : 0; //16*12
    valid[PVER_LEFT]       = uiBitSize >= B16X16_IN_BIT ? valid[4] : 0; //4*16
    valid[PVER_RIGHT]      = uiBitSize >= B16X16_IN_BIT ? valid[4] : 0; //12*16
    valid[PNXN]   = (!intra && input->InterSearch8x8);
    if (uiBitSize > MIN_CU_SIZE_IN_BIT || uiBitSize == B8X8_IN_BIT) { //qyu 0906
        valid[PNXN] = 0;
    }


    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) { //mode restriction for S frame
        valid[PSKIPDIRECT]        = 1;
        valid[P2NX2N]             = 0;
        valid[P2NXN]              = 0;
        valid[PNX2N]              = 0;
        valid[PHOR_UP]            = 0;
        valid[PHOR_DOWN]          = 0;
        valid[PVER_LEFT]          = 0;
        valid[PVER_RIGHT]         = 0;
        valid[PNXN]               = 0;
    }



    //===== SET LAGRANGE PARAMETERS =====

    if (input->rdopt) {


        he->global_lambda = (he->lambda_mode * input->lambda_factor_rdoq) / 100;
        if ((img->type == F_IMG) || (img->type == P_IMG)) {
            he->global_lambda = (he->global_lambda * input->lambda_factor_rdoq_p) / 100;
        } else if (img->type == B_IMG) {
            he->global_lambda = (he->global_lambda * input->lambda_factor_rdoq_b) / 100;
        }
        lambda_motion = sqrt(he->lambda_mode);
    }

    lambda_motion_factor = LAMBDA_FACTOR(lambda_motion);

    // reset chroma intra predictor to default
    currMB->c_ipred_mode = DC_PRED_C;

#if AVS2_S2_FASTMODEDECISION
    if (input->bg_input_number == 1 && he->bg_flag && img->type != INTRA && img->typeb != BP_IMG &&
        img->typeb != BACKGROUND_IMG) {
        sum = (1 << uiBitSize) * (1 << uiBitSize);
        //luma
        if (uiBitSize == 4) {
            for (block8x8 = 0; block8x8 < 4; block8x8++) {
                he->sum_diff_8x8 = 0;
                block_y = (block8x8 / 2) * (1 << (uiBitSize - 1));
                block_x = (block8x8 % 2) * (1 << (uiBitSize - 1));
                for (o = 0; o < (1 << (uiBitSize - 1)); o++) {
                    for (p = 0; p < (1 << (uiBitSize - 1)); p++) {
                        temp = hc->imgY[pix_y + block_y + o][pix_x + block_x + p] - he->background_frame[0][pix_y + block_y + o][pix_x + block_x
                                + p];
                        he->sum_diff_8x8 += (abs(temp) > 3);
                    }
                    if (block8x8 == 0) {
                        he->sum_diff_16x16 = he->sum_diff_8x8;
                    } else {
                        he->sum_diff_16x16 += he->sum_diff_8x8;
                    }
                }
            }
            sum_diff = he->sum_diff_16x16;
            he->sum_diff_32x32 += he->sum_diff_16x16;


        }
        if (uiBitSize == 5) {
            sum_diff = he->sum_diff_32x32;
            he->sum_diff_64x64 += he->sum_diff_32x32;
        }
        if (uiBitSize == 6) {
            sum_diff = he->sum_diff_64x64;
        }

        if (sum_diff * 16 < sum) { //背景块
            valid[PHOR_UP]      = 0;
            valid[PHOR_DOWN]    = 0;
            valid[PVER_LEFT]    = 0;
            valid[PVER_RIGHT]   = 0;
            valid[I16MB]        = 0;
            valid[I8MB]         = 0;
        }
        if (uiBitSize == 4) {
            he->sum_diff_16x16 = 0;
        } else if (uiBitSize == 5) {
            he->sum_diff_32x32 = 0;
        } else if (uiBitSize == 6) {
            he->sum_diff_64x64 = 0;
        }
    }
#endif

    if (input->rdopt) {
        int ctr16x16, index, mode;
        int bestMode = 0;
        int secBestMode = 0;
        min_rdcost = max_rdcost;

        g_best_dmh_mode = 0;

        //===== GET BEST MACROBLOCK MODE =====
        for (ctr16x16 = 0, index = 0; index < 13; index++) {
            mode = mb_mode_table[index];

            //--- Find a motion vector for each mode ---
            if (mode == PSKIPDIRECT) {
                if ((img->type == F_IMG) || (img->type == P_IMG)) {
                    FindSkipModeMotionVectorInCCD(uiBitSize, uiPositionInPic);
                } else if (bframe) {
                    Get_direct(uiBitSize, uiPositionInPic);    // uiBitSize,uiPositionInPic
                }
            } else if (!intra && ((mode == 1 && ctr16x16 == 0) || (mode > P2NX2N && mode < I8MB))) {
                //
                if ((uiBitSize == 5 || uiBitSize == 6) && (mode == PHOR_UP || mode == PHOR_DOWN) && !(bestMode == P2NXN)) {
                    continue;
                } else if ((uiBitSize == 5 || uiBitSize == 6) && (mode == PVER_LEFT || mode == PVER_RIGHT) && !(bestMode == PNX2N)) {
                    continue;
                }
                ME(uiBitSize, uiPositionInPic, mode, bframe, max_ref, adjust_ref, lambda_motion, write_ref, lambda_motion_factor,
                   valid, he->lambda_mode, currMB, 1);
            }

            if (((img->type == F_IMG) || (img->type == P_IMG)) && !mode && !img->mv_range_flag) {
                img->mv_range_flag = 1;
                continue;
            }

            //--- for INTER16x16 check all prediction directions ---
            if (mode == 1 && img->type == B_IMG) {
                best8x8pdir[1][0] = best8x8pdir[1][1] = best8x8pdir[1][2] = best8x8pdir[1][3] = ctr16x16;

                if (ctr16x16 < 3) {
                    index--;
                }

                ctr16x16++;
            }
            //CXR SDIP fast
            if ((mode == InNxNMB || mode == INxnNMB) && uiBitSize == 4) {
                if (uiBitSize == MIN_CU_SIZE_IN_BIT) {
                    if (modeRDcost[I8MB] > modeRDcost[I16MB] * 1.06 || modeRDcost[I8MB] < modeRDcost[I16MB] * 0.7) {
                        continue;
                    }
                    if (mode ==  INxnNMB && modeRDcost[InNxNMB] < modeRDcost[I16MB] * 0.9 && modeRDcost[InNxNMB] < modeRDcost[I8MB] * 0.9) {
                        continue;
                    }
                } else {
                    if (mode ==  INxnNMB && modeRDcost[InNxNMB] < modeRDcost[I16MB] * 0.9) {
                        continue;
                    }
                }
            }
            if ((mode == InNxNMB || mode == INxnNMB) && uiBitSize == 5) {
                if (uiBitSize == MIN_CU_SIZE_IN_BIT) {
                    if (modeRDcost[I16MB] < minRdCostD * 0.94 || modeRDcost[I8MB] < minRdCostD * 0.96) {
                        continue;
                    }
                    if (modeRDcost[I16MB] < minRdCostD && modeRDcost[I8MB] < minRdCostD) {
                        continue;
                    }
                    if (modeRDcost[I8MB] > minRdCostD * 1.1 && modeRDcost[I16MB] > minRdCostD * 1.1) {
                        continue;
                    }

                    if (modeRDcost[I8MB] < modeRDcost[I16MB] * 0.9 || modeRDcost[I8MB] > modeRDcost[I16MB] * 1.1) {
                        continue;
                    }
                    if (mode ==  INxnNMB && (modeRDcost[InNxNMB] < modeRDcost[I16MB] && modeRDcost[InNxNMB] < modeRDcost[I8MB] ||
                                             modeRDcost[InNxNMB] < minRdCostD)) {
                        continue;
                    }
                } else {
                    if (mode ==  INxnNMB && (modeRDcost[InNxNMB] < modeRDcost[I16MB] || modeRDcost[InNxNMB] < minRdCostD)) {
                        continue;
                    }
                }
            }

            img->NoResidueDirect = 0;

            if (valid[mode]) {
                int i, max_num_skipmode = 1;
                if (!(img->type == F_IMG  && input->wsm_enabled && !mode && img->num_of_references > 1)) {
                    max_num_skipmode = 1;
                } else {
                    max_num_skipmode = img->num_of_references;
                }
                for (i = 0; i < max_num_skipmode; i++) {
                    currMB->weighted_skipmode = i;
                    if (img->type == F_IMG && mode != 0 && mode != I8MB && mode != I16MB && mode != InNxNMB  && mode != INxnNMB  &&
                        mode != IBLOCK && input->b_dmh_enabled && best8x8pdir[mode][0] == FORWARD && best8x8pdir[mode][1] == FORWARD &&
                        best8x8pdir[mode][2] == FORWARD && best8x8pdir[mode][3] == FORWARD && img->typeb != BP_IMG)

                    {
                        max_dmh_mode = DMH_MODE_NUM + DMH_MODE_NUM - 1;
                        if (uiBitSize == B8X8_IN_BIT && mode >= P2NXN && mode <= PVER_RIGHT) { // disable 8x4 or 4x8 2MVs/PU mode
                            max_dmh_mode = 1;
                        }
                        if (!input->dmh_enabled_encoder) {
                            max_dmh_mode = 1;
                        }
                    } else {
                        max_dmh_mode = 1;
                    }
                    he->bypass_all_dmh = 0;
                    for (dmh_mode = 0; dmh_mode < max_dmh_mode; dmh_mode++) {
                        if (dmh_mode > 0 && he->bypass_all_dmh) {
                            continue;
                        }

                        if (dmh_mode > (DMH_MODE_NUM - 1)) {
                            if (g_best_dmh_mode != (dmh_mode - (DMH_MODE_NUM - 1))) {
                                continue;
                            }
                        }
                        cMCParam.num_skipmode = i;
                        cMCParam.dmh_mode = dmh_mode;
                        cMCParam.dir = 0;
                        currMB->md_directskip_mode = 0;
                        if (RDCost_for_codingUnits(currMB, uiBitSize, uiPositionInPic, he->lambda_mode, mode, &min_rdcost, &cMCParam)) {
                            weighted_skipmode_fix = i;
                            store_codingUnit_parameters(currMB, uiBitSize, uiPositionInPic, mode);
                            secBestMode = bestMode;
                            bestMode = mode;
                            g_best_dmh_mode = dmh_mode;

                            best_md_directskip_mode = 0;
                        } else {
                            reset_coding_state(cs_cm);
                        }
                    }
                }

            }


            if (img->type == F_IMG && input->dhp_enabled && (best8x8pdir[mode][0] != FORWARD || best8x8pdir[mode][1] != FORWARD ||
                    best8x8pdir[mode][2] != FORWARD || best8x8pdir[mode][3] != FORWARD) && ((mode == 1 && ctr16x16 == 0) ||
                            (mode > P2NX2N && mode < I8MB)) && max_ref > 1) {
                if ((uiBitSize == 5 || uiBitSize == 6) && (mode == PHOR_UP || mode == PHOR_DOWN) && !(bestMode == P2NXN)) {
                    continue;
                } else if ((uiBitSize == 5 || uiBitSize == 6) && (mode == PVER_LEFT || mode == PVER_RIGHT) && !(bestMode == PNX2N)) {
                    continue;
                }
                ME(uiBitSize, uiPositionInPic, mode, bframe, max_ref, adjust_ref, lambda_motion, write_ref, lambda_motion_factor,
                   valid, he->lambda_mode, currMB, 0);
                if (valid[mode]) {
                    if (input->b_dmh_enabled) {
                        max_dmh_mode = DMH_MODE_NUM + DMH_MODE_NUM - 1;
                        if (uiBitSize == B8X8_IN_BIT && mode >= P2NXN && mode <= PVER_RIGHT) { // disable 8x4 or 4x8 2MVs/PU mode
                            max_dmh_mode = 1;
                        }
                    }
                    if (!input->dmh_enabled_encoder) {
                        max_dmh_mode = 1;
                    }

                    he->bypass_all_dmh = 0;
                    for (dmh_mode = 0; dmh_mode < max_dmh_mode; dmh_mode++) {
                        if (dmh_mode > 0 && he->bypass_all_dmh) {
                            continue;
                        }

                        if (dmh_mode > (DMH_MODE_NUM - 1)) {
                            if (g_best_dmh_mode != (dmh_mode - (DMH_MODE_NUM - 1))) {
                                continue;
                            }
                        }
                        cMCParam.num_skipmode = 0;
                        cMCParam.dmh_mode = dmh_mode;
                        cMCParam.dir = 0;
                        currMB->md_directskip_mode = 0;

                        if (RDCost_for_codingUnits(currMB, uiBitSize, uiPositionInPic, he->lambda_mode, mode, &min_rdcost, &cMCParam)) {
                            weighted_skipmode_fix = 0;
                            store_codingUnit_parameters(currMB, uiBitSize, uiPositionInPic, mode);
                            secBestMode = bestMode;
                            bestMode = mode;
                            g_best_dmh_mode = dmh_mode;

                            best_md_directskip_mode = 0;
                        } else {
                            reset_coding_state(cs_cm);
                        }
                    }
                }
            }

            if (((img->type == F_IMG && input->b_mhpskip_enabled) || bframe) && mode == 0) {
                dir_num = (pframe || img->type == F_IMG) ? MH_PSKIP_NUM : DIRECTION;
                for (dir = 1; dir <= dir_num; dir++) {
                    cMCParam.num_skipmode = 0;
                    cMCParam.dmh_mode = 0;
                    cMCParam.dir = dir;
                    currMB->md_directskip_mode = dir;
                    currMB->weighted_skipmode = 0;
                    if (RDCost_for_codingUnits(currMB, uiBitSize, uiPositionInPic, he->lambda_mode, mode, &min_rdcost, &cMCParam)) {
                        best_md_directskip_mode = dir;
                        store_codingUnit_parameters(currMB, uiBitSize, uiPositionInPic, mode);
                        secBestMode = bestMode;
                        bestMode = mode;
                        g_best_dmh_mode = 0;
                        weighted_skipmode_fix = 0;
                    } else {
                        reset_coding_state(cs_cm);
                    }
                }
            }

            if ((bframe && valid[mode] && !IS_INTRA(currMB) && currMB->cbp) || ((img->type == P_IMG || img->type == F_IMG) &&
                    mode == 0 && currMB->cbp && valid[0])) {
                img->NoResidueDirect = 1;

                if (valid[mode]) {
                    int i, max_num_skipmode = 1;
                    if (!(img->type == F_IMG && input->wsm_enabled && !mode && img->num_of_references > 1)) {
                        max_num_skipmode = 1;
                    } else {
                        max_num_skipmode = img->num_of_references;
                    }
                    for (i = 0; i < max_num_skipmode; i++) {
                        currMB->weighted_skipmode = i;
                        cMCParam.num_skipmode = i;
                        cMCParam.dmh_mode = 0;
                        cMCParam.dir = 0;
                        currMB->md_directskip_mode = 0;
                        if (RDCost_for_codingUnits(currMB, uiBitSize, uiPositionInPic, he->lambda_mode, mode, &min_rdcost,
                                                   &cMCParam)) { // qyu 0825
                            weighted_skipmode_fix = i;
                            store_codingUnit_parameters(currMB, uiBitSize, uiPositionInPic, mode);
                            secBestMode = bestMode;
                            bestMode = mode;
                            g_best_dmh_mode = 0;
                            best_md_directskip_mode = 0;
                        } else {

                            reset_coding_state(cs_cm);
                        }
                    }
                }

                if (((img->type == F_IMG && input->b_mhpskip_enabled) || bframe) && mode == 0) {
                    img->NoResidueDirect = 1;
                    dir_num = (pframe || img->type == F_IMG) ? MH_PSKIP_NUM : DIRECTION;
                    for (dir = 1; dir <= dir_num; dir++) {
                        cMCParam.num_skipmode = 0;
                        cMCParam.dmh_mode = 0;
                        cMCParam.dir = dir;
                        currMB->md_directskip_mode = dir;
                        currMB->weighted_skipmode = 0;
                        if (RDCost_for_codingUnits(currMB, uiBitSize, uiPositionInPic, he->lambda_mode, mode, &min_rdcost, &cMCParam)) {
                            best_md_directskip_mode = dir;
                            store_codingUnit_parameters(currMB, uiBitSize, uiPositionInPic, mode);
                            secBestMode = bestMode;
                            bestMode = mode;
                            g_best_dmh_mode = 0;
                            weighted_skipmode_fix = 0;
                        } else {
                            reset_coding_state(cs_cm);
                        }
                    }
                }
                img->NoResidueDirect = 0;
            }
        } //for(ctr16x16 = 0 ...)
        set_stored_codingUnit_parameters(currMB, uiBitSize, uiPositionInPic, min_rdcost + splitRdCost + split_cost_up,
                                         minRdCostD + splitRdCost + split_cost_down);
    }

    if (img->current_mb_nr == 0) {
        he->intras = 0;
    }
    if (((img->type == F_IMG) || (img->type == P_IMG)) && IS_INTRA(currMB)) {
        he->intras++;
    }

    free(currMB);
    if (min_rdcost + splitRdCost + split_cost_up < minRdCostD + splitRdCost + split_cost_down) {
        return min_rdcost + split_cost_up;
    } else {
        return minRdCostD + split_cost_down;
    }
}

double encode_one_SMB(unsigned int uiBitSize, unsigned int uiPositionInPic, double split_rd_cost)
{
    int i;

    int pix_x_InPic_start = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_y_InPic_start = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_x_InPic_end = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);     //Liwr 0915
    int pix_y_InPic_end = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);     //Liwr 0915
    int iBoundary_end = (pix_x_InPic_end > img->width) || (pix_y_InPic_end > img->height);
    int pos_x, pos_y, pos_InPic;

    double min_rd_cost = 1 << 30;
    codingUnit tmpMB;
    double split_cost = 0;
    Init_Curr_codingUnit(&tmpMB, uiBitSize, uiPositionInPic);

    store_coding_state(cs_submb[uiBitSize - MIN_CU_SIZE_IN_BIT]);

    if (uiBitSize > MIN_CU_SIZE_IN_BIT) {
        min_rd_cost = 0;
        if (!iBoundary_end) {
            split_cost = he->lambda_mode * writeSplitFlag(1, &tmpMB, uiBitSize);
        }


        for (i = 0; i < 4; i++) {
            int mb_x = (i % 2) << (uiBitSize - MIN_CU_SIZE_IN_BIT - 1);     //uiBitSize 5:1 ; 6:2
            int mb_y = (i / 2) << (uiBitSize - MIN_CU_SIZE_IN_BIT - 1);     //uiBitSize 5:1 ; 6:2
            int pos = uiPositionInPic + mb_y * img->PicWidthInMbs + mb_x;

            pos_x = pix_x_InPic_start + (mb_x << MIN_CU_SIZE_IN_BIT);
            pos_y = pix_y_InPic_start + (mb_y << MIN_CU_SIZE_IN_BIT);
            pos_InPic = (pos_x >= img->width || pos_y >= img->height);

            if (pos_InPic) {
                continue;
            }
            min_rd_cost += encode_one_SMB(uiBitSize - 1, pos, split_rd_cost + split_cost);
        }
    }

    reset_coding_state(cs_submb[uiBitSize - MIN_CU_SIZE_IN_BIT]);

    if (!iBoundary_end) {
        double split_cost_up = 0;
        if (uiBitSize > MIN_CU_SIZE_IN_BIT) {
            split_cost_up = he->lambda_mode * writeSplitFlag(0, &tmpMB, uiBitSize);
        }
        min_rd_cost = encode_SubMB(uiBitSize, uiPositionInPic, min_rd_cost, split_cost, split_cost_up,
                                   split_rd_cost);  // down layer CU cost, down layer split cost, up layer split cost, previouse split cost
    }


    if (uiBitSize == input->g_uiMaxSizeInBit) {   // reset coding state when SMB coding is done
        reset_coding_state(cs_submb[uiBitSize - MIN_CU_SIZE_IN_BIT]);
    }

    return min_rd_cost;
}
