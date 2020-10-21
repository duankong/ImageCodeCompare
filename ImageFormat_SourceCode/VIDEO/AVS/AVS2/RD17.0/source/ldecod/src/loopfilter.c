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
#include <string.h>
#include <math.h>
#include "global.h"
#include <assert.h>
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/loop-filter.h"
#include "AEC.h"
#define  IClip( Min, Max, Val) (((Val)<(Min))? (Min):(((Val)>(Max))? (Max):(Val)))

extern const byte QP_SCALE_CR[64] ;

byte ALPHA_TABLE[64] = {
    0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 1, 2, 2, 2, 3, 3,
    4, 4, 5, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 15, 16, 18, 20,
    22, 24, 26, 28, 30, 33, 33, 35,
    35, 36, 37, 37, 39, 39, 42, 44,
    46, 48, 50, 52, 53, 54, 55, 56,
    57, 58, 59, 60, 61, 62, 63, 64
};
byte  BETA_TABLE[64]  = {
    0, 0, 0, 0, 0, 0, 1, 1,
    1, 1, 1, 1, 1, 2, 2, 2,
    2, 2, 3, 3, 3, 3, 4, 4,
    4, 4, 5, 5, 5, 5, 6, 6,
    6, 7, 7, 7, 8, 8, 8, 9,
    9, 10, 10, 11, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22,
    23, 23, 24, 24, 25, 25, 26, 27
};
byte CLIP_TAB[64] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 2, 2,
    2, 2, 2, 2, 2, 2, 3, 3,
    3, 3, 3, 3, 3, 4, 4, 4,
    5, 5, 5, 6, 6, 6, 7, 7,
    7, 7, 8, 8, 8, 9, 9, 9
} ;
int EO_OFFSET_INV__MAP[] = {1, 0, 2, -1, 3, 4, 5, 6};
////////////////////////////ADD BY HJQ 2012-11-1//////////////////////////////////////////////
typedef Boolean bool;
int **ppbEdgeFilter[2];

#define LOOPFILTER_SIZE 3   //8X8

void GetStrength(byte Strength[2], codingUnit *MbP, codingUnit *MbQ, int dir, int edge, int block_y, int block_x);
void EdgeLoop(byte *SrcPtr, byte Strength[2], int QP,  int dir, int width, int Chro);
void DeblockMb(byte **imgY, byte ***imgUV, int blk_y, int blk_x, int EdgeDir) ;
void EdgeLoopX(byte *SrcPtr, int QP, int dir, int width, int Chro, codingUnit *MbP, codingUnit *MbQ, int block_y,
               int block_x);
int SkipFiltering(codingUnit *MbP, codingUnit *MbQ, int dir, int edge, int block_y, int block_x);
//void read_SAOParameter(int* slice_sao_on);
void read_sao_smb(int smb_index, int pix_y, int pix_x, int smb_pix_width, int smb_pix_height, int *slice_sao_on,
                  SAOBlkParam *sao_cur_param, SAOBlkParam *rec_sao_cur_param);
void xSetEdgeFilterParam(unsigned int uiBitSize, unsigned int b8_x_start, unsigned int b8_y_start, int idir,
                         int edgecondition)
{
    unsigned int i;

    if (idir == 0) {
        if (b8_x_start == 0) {
            return;
        }
        for (i = 0; i < (unsigned int)(1 << (uiBitSize - LOOPFILTER_SIZE)) ; i++) {
            if ((b8_y_start + i) < (unsigned int)(img->height >> LOOPFILTER_SIZE)) {
                if (ppbEdgeFilter[idir][b8_y_start + i][b8_x_start]) {
                    break;
                }
                ppbEdgeFilter[idir][b8_y_start + i][b8_x_start] = edgecondition;
            } else {
                break;
            }
        }
    } else {
        if (b8_y_start == 0) {
            return;
        }
        for (i = 0; i < (unsigned int)(1 << (uiBitSize - LOOPFILTER_SIZE)) ; i++) {
            if ((b8_x_start + i) < (unsigned int)(img->width) >> LOOPFILTER_SIZE) {
                if (ppbEdgeFilter[idir][b8_y_start][b8_x_start + i]) {
                    break;
                }
                ppbEdgeFilter[idir][b8_y_start][b8_x_start + i] = edgecondition;
            } else {
                break;
            }
        }
    }

}


void xSetEdgeFilter_One_SMB(unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    int i;
    int edge_type_onlyluma = 1;
    int edge_type_all = 2;    //EdgeCondition 0:not boundary 1:PU/TU boundary 2:CU boundary

    int pix_x_InPic_start = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_y_InPic_start = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE;
    int pix_x_InPic_end = (uiPositionInPic % img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);
    int pix_y_InPic_end = (uiPositionInPic / img->PicWidthInMbs) * MIN_CU_SIZE + (1 << uiBitSize);
    //   int  iBoundary_start = (pix_x_InPic_start>=img->width)||(pix_y_InPic_start>=img->height);
    int  iBoundary_end = (pix_x_InPic_end > img->width) || (pix_y_InPic_end > img->height);
    int pos_x, pos_y, pos_InPic;
    unsigned int b8_x_start = (uiPositionInPic % img->PicWidthInMbs) * (1 << (MIN_CU_SIZE_IN_BIT - LOOPFILTER_SIZE));
    unsigned int b8_y_start = (uiPositionInPic / img->PicWidthInMbs) * (1 << (MIN_CU_SIZE_IN_BIT - LOOPFILTER_SIZE)) ;

    codingUnit *currMB = &img->mb_data[uiPositionInPic];

    if (currMB->ui_MbBitSize < uiBitSize) {
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

            xSetEdgeFilter_One_SMB(uiBitSize - 1, pos);
        }
        return;

    }
    //////////////////////////////////////////////////////////////////////////
    //if (currMB->ui_MbBitSize > B8X8_IN_BIT)
    {
        xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start, 0, edge_type_all);  /// LEFT
        xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start, 1, edge_type_all);  ///UP
    }

    //xSetEdgeFilter_TU()
    i = (uiBitSize - LOOPFILTER_SIZE - 1) ; /// b8


    //xSetEdgeFilter_PU()
    if (currMB->ui_MbBitSize > B8X8_IN_BIT)
        switch (currMB->cuType) {
        case  2:  ///2NXN
            xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)), 1, edge_type_all);
            break;
        case 3: ///NX2N
            xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)), b8_y_start, 0, edge_type_all);
            break;
        case 8:  ///NXN_inter
            xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)), b8_y_start, 0, edge_type_all);
            xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)), 1, edge_type_all);
            break;
        case 9: ///NXN_intra
            xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)), b8_y_start, 0, edge_type_onlyluma);
            xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)), 1, edge_type_onlyluma);
            break;
        case InNxNMB:
            if (i - 1 >= 0) {
                xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i - 1)), 1,
                                    edge_type_onlyluma);   ///?¨²32???¦Ì¨ª3?? 1<<-1=0  ???¨²64???¦Ì¨ª3??2?¨º?
                xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i - 1)) * 2, 1, edge_type_onlyluma);
                xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i - 1)) * 3, 1, edge_type_onlyluma);
            } else {
                xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)), 1, edge_type_onlyluma);
            }
            break;
        case INxnNMB:
            if (i - 1 >= 0) {
                xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i - 1)), b8_y_start, 0, edge_type_onlyluma);
                xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i - 1)) * 2, b8_y_start, 0, edge_type_onlyluma);
                xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i - 1)) * 3, b8_y_start, 0, edge_type_onlyluma);
            } else {
                xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)), b8_y_start, 0, edge_type_onlyluma);
            }
            break;
        case 4:  ///2NxnU
            if (i - 1 >= 0) {
                xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i - 1)), 1,
                                    edge_type_all);   ///?¨²32???¦Ì¨ª3?? 1<<-1=0  ???¨²64???¦Ì¨ª3??2?¨º?
            }
            break;
        case 5:  ///2NxnD
            if (i - 1 >= 0) {
                xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i - 1)) * 3, 1, edge_type_all);
            }
            break;
        case 6:  ///nLx2N
            if (i - 1 >= 0) {
                xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i - 1)), b8_y_start, 0, edge_type_all);
            }
            break;
        case 7:  ///nRx2N
            if (i - 1 >= 0) {
                xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i - 1)) * 3, b8_y_start, 0, edge_type_all);
            }
            break;
        default:
            //    case 0: ///direct/skip
            //    case 1: ///size_2Nx2N_inter
            //    case 10: ///2Nx2N_intra
            ///currMB->trans_size = 0;
            break;
        }
    //////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////TU////////////////////////////////////////////////////
    if (currMB->ui_MbBitSize > B8X8_IN_BIT)
        if (currMB->cuType != 8 && currMB->cuType != 9 && currMB->trans_size == 1 && currMB->cbp != 0) {
            if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->cuType == P2NXN || currMB->cuType == PHOR_UP ||
                    currMB->cuType == PHOR_DOWN || currMB->cuType == InNxNMB)) {
                if (currMB->ui_MbBitSize == B16X16_IN_BIT) {
                    xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)), 1, edge_type_onlyluma);
                } else {
                    xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i - 1)), 1, edge_type_onlyluma);
                    xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)), 1, edge_type_onlyluma);
                    xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)) + (1 << (i - 1)), 1, edge_type_onlyluma);
                }
            } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (currMB->cuType == PNX2N ||
                       currMB->cuType == PVER_LEFT || currMB->cuType == PVER_RIGHT || currMB->cuType == INxnNMB)) {
                if (currMB->ui_MbBitSize == 4) {
                    xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)), b8_y_start, 0, edge_type_onlyluma);
                } else {
                    xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i - 1)), b8_y_start, 0, edge_type_onlyluma);
                    xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)), b8_y_start, 0, edge_type_onlyluma);
                    xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)) + (1 << (i - 1)), b8_y_start, 0, edge_type_onlyluma);
                }
            } else if (currMB->ui_MbBitSize > B8X8_IN_BIT) {
                xSetEdgeFilterParam(uiBitSize, b8_x_start + (1 << (i)), b8_y_start, 0, edge_type_onlyluma);
                xSetEdgeFilterParam(uiBitSize, b8_x_start, b8_y_start + (1 << (i)), 1, edge_type_onlyluma);
            }
        }
    //////////////////////////////////////////////////////////////////////////
}

void SetEdgeFilter()
{
    Boolean end_of_picture = FALSE;
    int CurrentMbNumber = 0;
    int MBRowSize = img->width / MIN_CU_SIZE;

    int num_of_orgMB_in_col ;
    int num_of_orgMB_in_row ;
    int size = 1 << input->g_uiMaxSizeInBit;  //input->
    int pix_x;
    int pix_y;
    int num_of_orgMB;

    while (end_of_picture == FALSE) {   // loop over codingUnits
        pix_x = (CurrentMbNumber % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
        pix_y = (CurrentMbNumber / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

        if (pix_x + size >= img->width) {
            num_of_orgMB_in_row = (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT;
        } else {
            num_of_orgMB_in_row = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);    //
        }

        if (pix_y + size >= img->height) {
            num_of_orgMB_in_col = (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT;
        } else {
            num_of_orgMB_in_col = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);   //
        }
        num_of_orgMB = num_of_orgMB_in_col * num_of_orgMB_in_row;

        xSetEdgeFilter_One_SMB(input->g_uiMaxSizeInBit, CurrentMbNumber);

        if ((CurrentMbNumber + num_of_orgMB_in_row) % MBRowSize == 0) {     //end of the row
            CurrentMbNumber += (num_of_orgMB_in_row + MBRowSize * ((1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT)) -
                                1));   //eg. 64 +: (4-1)*num_mb_inWidth+4
        } else {
            CurrentMbNumber += num_of_orgMB_in_row;
        }

        if (CurrentMbNumber >= img->PicSizeInMbs) { //img->total_number_mb=( img->width * img->height)/(MIN_CU_SIZE*MIN_CU_SIZE)
            end_of_picture = TRUE;
        }
    }

}

void CreateEdgeFilter()
{
    int b8_x, b8_y;
    int i, j;

    //////////////////////////////////////////////////////////////////////////initialize
    for (i = 0; i < 2; i++) {
        ppbEdgeFilter[i] = (int **)calloc((img->height >> LOOPFILTER_SIZE), sizeof(int *));
        for (j = 0; j < (img->height >> LOOPFILTER_SIZE) ; j++) {
            ppbEdgeFilter[i][j] = (int *)calloc((img->width >> LOOPFILTER_SIZE), sizeof(int));
        }
    }

    for (i = 0; i < 2; i++) {
        for (b8_y = 0 ; b8_y < (img->height >> LOOPFILTER_SIZE) ; b8_y++)
            for (b8_x = 0 ; b8_x < (img->width >> LOOPFILTER_SIZE) ; b8_x++) {
                ppbEdgeFilter[i][b8_y][b8_x] = 0;
            }
    }
    //////////////////////////////////////////////////////////////////////////
}

void DeleteEdgeFilter(int height)
{
    int i, j;

    for (i = 0; i < 2; i++) {
        for (j = 0; j < (height >> LOOPFILTER_SIZE) ; j++) {
            free(ppbEdgeFilter[i][j]);
        }
        free(ppbEdgeFilter[i]);
    }

}

//////////////////////////////////////////////////////////////////////////
/*
*************************************************************************
* Function:The main MB-filtering function
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void DeblockFrame(byte **imgY, byte ***imgUV)
{
    int       mb_x, mb_y ;
    img->current_mb_nr = -1;    // jlzheng  7.18
    for (mb_y = 0 ; mb_y < (img->height >> MIN_CU_SIZE_IN_BIT) ; mb_y++) {
        for (mb_x = 0 ; mb_x < (img->width >> MIN_CU_SIZE_IN_BIT) ; mb_x++) {
            img->current_mb_nr++;   // jlzheng 7.18
            if (input->chroma_format == 1) {
                DeblockMb(imgY, imgUV, mb_y, mb_x, 0) ;
            }
        }
    }

    img->current_mb_nr = -1;    // jlzheng  7.18
    for (mb_y = 0 ; mb_y < (img->height >> MIN_CU_SIZE_IN_BIT) ; mb_y++) {
        for (mb_x = 0 ; mb_x < (img->width >> MIN_CU_SIZE_IN_BIT) ; mb_x++) {
            img->current_mb_nr++;   // jlzheng 7.18
            if (input->chroma_format == 1) {
                DeblockMb(imgY, imgUV, mb_y, mb_x, 1) ;
            }
        }
    }

    DeleteEdgeFilter(img->height);
}

/*
*************************************************************************
* Function: Deblocks one codingUnit
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void DeblockMb(byte **imgY, byte ***imgUV, int mb_y, int mb_x, int EdgeDir)
{
    int           EdgeCondition;
    int           dir, QP ;
    unsigned int b8_x_start ;
    unsigned int b8_y_start ;
    byte          *SrcY, *SrcU = NULL, *SrcV = NULL;
    codingUnit    *MbP, *MbQ ;
    const int     mb_width = img->width / MIN_CU_SIZE;
    codingUnit    *currMB = &img->mb_data[img->current_mb_nr];
    int           iFlagSkipFiltering;

    int           flag = 0;
    SrcY = imgY   [mb_y << MIN_CU_SIZE_IN_BIT] + (mb_x << MIN_CU_SIZE_IN_BIT) ;

    if (imgUV != NULL) {
        SrcU = imgUV[0][mb_y << (MIN_CU_SIZE_IN_BIT - 1)] + (mb_x << (MIN_CU_SIZE_IN_BIT - 1)) ;
        SrcV = imgUV[1][mb_y << (MIN_CU_SIZE_IN_BIT - 1)] + (mb_x << (MIN_CU_SIZE_IN_BIT - 1)) ;
    }

    MbQ  = &img->mb_data[mb_y * (img->width >> MIN_CU_SIZE_IN_BIT) + mb_x]
           ;                                           // current Mb
    dir = EdgeDir;
    {
        EdgeCondition = (dir && mb_y) || (!dir && mb_x)  ;     // can not filter beyond frame boundaries

        if (!dir && mb_x && !input->crossSliceLoopFilter) {
            EdgeCondition = (currMB->slice_nr == img->mb_data[img->current_mb_nr - 1].slice_nr) ? EdgeCondition :
                            0; //  can not filter beyond slice boundaries   jlzheng 7.8
        }

        if (dir && mb_y && !input->crossSliceLoopFilter) {
            EdgeCondition = (currMB->slice_nr == img->mb_data[img->current_mb_nr - mb_width].slice_nr) ? EdgeCondition :
                            0; //  can not filter beyond slice boundaries   jlzheng 7.8
        }

        ///
        //flag = 0;
        b8_x_start = mb_x;
        b8_y_start = mb_y;
        /*#if M3198_CU8
        if (flag == 0)
        {
        #endif*/
        EdgeCondition = (ppbEdgeFilter[dir][b8_y_start][b8_x_start] &&
                         EdgeCondition) ? ppbEdgeFilter[dir][b8_y_start][b8_x_start] : 0;
        // then  4 horicontal
        if (EdgeCondition) {
            MbP = (dir) ? (MbQ - (img->width >> MIN_CU_SIZE_IN_BIT))  : (MbQ - 1);         // MbP = Mb of the remote 4x4 block
            QP = (MbP->qp + MbQ->qp + 1) >> 1 ;                               // Average QP of the two blocks

            iFlagSkipFiltering = 0;
            if (!iFlagSkipFiltering) {
                EdgeLoopX(SrcY, QP, dir, img->width, 0, MbP, MbQ, mb_y << 1, mb_x << 1);

                if ((EdgeCondition == 2) && (imgUV != NULL)/* && ( !edge )*/) {
                    if (input->sample_bit_depth > 8) {
                        if ((mb_y % 2 == 0 && dir) || (mb_x % 2 == 0) && (!dir)) {
#if DBFIX_10bit && CHROMA_DELTA_QP
                            int cQPu, cQPv;
                            int cpQPu, cqQPu, cpQPv, cqQPv;

                            cpQPu = MbP->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_u;

                            cpQPu = cpQPu < 0 ? cpQPu : QP_SCALE_CR[cpQPu];
                            cpQPu = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cpQPu + 8 * (input->sample_bit_depth - 8));
                            cqQPu = MbQ->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_u;

                            cqQPu = cqQPu < 0 ? cqQPu : QP_SCALE_CR[cqQPu];
                            cqQPu = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cqQPu + 8 * (input->sample_bit_depth - 8));
                            cQPu = (cpQPu + cqQPu + 1) >> 1;

                            cpQPv = MbP->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_v;

                            cpQPv = cpQPv < 0 ? cpQPv : QP_SCALE_CR[cpQPv];
                            cpQPv = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cpQPv + 8 * (input->sample_bit_depth - 8));
                            cqQPv = MbQ->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_v;

                            cqQPv = cqQPv < 0 ? cqQPv : QP_SCALE_CR[cqQPv];
                            cqQPv = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cqQPv + 8 * (input->sample_bit_depth - 8));
                            cQPv = (cpQPv + cqQPv + 1) >> 1;
                            EdgeLoopX(SrcU, cQPu, dir, img->width_cr, 1, MbP, MbQ, mb_y << 1, mb_x << 1) ;
                            EdgeLoopX(SrcV, cQPv, dir, img->width_cr, 1, MbP, MbQ, mb_y << 1, mb_x << 1) ;
#else
                            EdgeLoopX(SrcU, QP_SCALE_CR[Clip3(0, 63,
                                                              (QP - (8 * (input->sample_bit_depth - 8))))] + (8 * (input->sample_bit_depth - 8)), dir, img->width_cr, 1, MbP, MbQ,
                                      mb_y << 1, mb_x << 1) ;
                            EdgeLoopX(SrcV, QP_SCALE_CR[Clip3(0, 63,
                                                              (QP - (8 * (input->sample_bit_depth - 8))))] + (8 * (input->sample_bit_depth - 8)), dir, img->width_cr, 1, MbP, MbQ,
                                      mb_y << 1, mb_x << 1) ;
#endif

                        }
                        //EdgeLoopX ( SrcU, Clip3(0, 51, (QP_SCALE_CR[Clip3(0, 63, (QP - 16))] + 16)), dir, img->width_cr, 1 ) ;
                        //EdgeLoopX ( SrcV, Clip3(0, 51, (QP_SCALE_CR[Clip3(0, 63, (QP - 16))] + 16)), dir, img->width_cr, 1 ) ;
                    } else {
                        if ((mb_y % 2 == 0 && dir) || (mb_x % 2 == 0) && (!dir)) {

#if DBFIX_10bit && CHROMA_DELTA_QP
                            int cQPu, cQPv;
                            int cpQPu, cqQPu, cpQPv, cqQPv;

                            cpQPu = MbP->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_u;

                            cpQPu = cpQPu < 0 ? cpQPu : QP_SCALE_CR[cpQPu];
                            cpQPu = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cpQPu + 8 * (input->sample_bit_depth - 8));
                            cqQPu = MbQ->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_u;

                            cqQPu = cqQPu < 0 ? cqQPu : QP_SCALE_CR[cqQPu];
                            cqQPu = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cqQPu + 8 * (input->sample_bit_depth - 8));
                            cQPu = (cpQPu + cqQPu + 1) >> 1;

                            cpQPv = MbP->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_v;

                            cpQPv = cpQPv < 0 ? cpQPv : QP_SCALE_CR[cpQPv];
                            cpQPv = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cpQPv + 8 * (input->sample_bit_depth - 8));
                            cqQPv = MbQ->qp - 8 * (input->sample_bit_depth - 8) + hd->chroma_quant_param_delta_v;

                            cqQPv = cqQPv < 0 ? cqQPv : QP_SCALE_CR[cqQPv];
                            cqQPv = Clip3(0, 63 + 8 * (input->sample_bit_depth - 8), cqQPv + 8 * (input->sample_bit_depth - 8));
                            cQPv = (cpQPv + cqQPv + 1) >> 1;
                            EdgeLoopX(SrcU, cQPu, dir, img->width_cr, 1, MbP, MbQ, mb_y << 1, mb_x << 1) ;
                            EdgeLoopX(SrcV, cQPv, dir, img->width_cr, 1, MbP, MbQ, mb_y << 1, mb_x << 1) ;
#else
                            EdgeLoopX(SrcU, QP_SCALE_CR[QP], dir, img->width_cr, 1, MbP, MbQ, mb_y << 1, mb_x << 1) ;
                            EdgeLoopX(SrcV, QP_SCALE_CR[QP], dir, img->width_cr, 1, MbP, MbQ, mb_y << 1, mb_x << 1) ;
#endif

                        }
                    }
                }

            }

        }
        //}
    }//end loop dir
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
byte BLK_NUM[2][2][2]  = {{{0, 8}, {2, 10}}, {{0, 2}, {8, 10}}} ;
void GetStrength(byte Strength[2], codingUnit *MbP, codingUnit *MbQ, int dir, int edge, int block_y, int block_x)
{
    int    blkQ, idx ;
    int    blk_x2, blk_y2, blk_x, blk_y ;
    int    ***tmp_fwMV = img->fw_mv;
    int    ***tmp_mv   =  img->tmp_mv;
    int    ***tmp_bwMV = img->bw_mv;
    int    **fw_refFrArr = img->fw_refFrArr;
    int    **bw_refFrArr = img->bw_refFrArr;

    for (idx = 0 ; idx < 2 ; idx++) {
        blkQ = BLK_NUM[dir][edge][idx] ;
        blk_y = block_y + (blkQ >> 2);
        blk_x = block_x + (blkQ & 3);
        blk_x >>= 1;
        blk_y >>= 1;
        blk_y2 = blk_y -  dir ;
        blk_x2 = blk_x - !dir ;
        if (IS_INTRA(MbP) || IS_INTRA(MbQ)) {
            Strength[idx] = 2;
        } else if ((img->type == F_IMG) || (img->type == P_IMG)) {
            Strength[idx] = (abs(tmp_mv[blk_y][blk_x][0] -   tmp_mv[blk_y2][blk_x2][0]) >= 4) ||
                            (abs(tmp_mv[blk_y][blk_x][1] -   tmp_mv[blk_y2][blk_x2][1]) >= 4) ||
                            (hc->refFrArr[blk_y][blk_x] != hc->refFrArr[blk_y2][blk_x2]);
        } else {
            Strength[idx] = (abs(tmp_fwMV[blk_y][blk_x][0] - tmp_fwMV[blk_y2][blk_x2][0]) >= 4) ||
                            (abs(tmp_fwMV[blk_y][blk_x][1] - tmp_fwMV[blk_y2][blk_x2][1]) >= 4) ||
                            (abs(tmp_bwMV[blk_y][blk_x][0] - tmp_bwMV[blk_y2][blk_x2][0]) >= 4) ||
                            (abs(tmp_bwMV[blk_y][blk_x][1] - tmp_bwMV[blk_y2][blk_x2][1]) >= 4) ||
                            (fw_refFrArr [blk_y][blk_x] !=   fw_refFrArr[blk_y2][blk_x2]) ||
                            (bw_refFrArr [blk_y][blk_x] !=   bw_refFrArr[blk_y2][blk_x2]);
        }
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
void EdgeLoop(byte *SrcPtr, byte Strength[2], int QP, int dir, int width, int Chro)
{
    int      pel, PtrInc, Strng ;
    int      inc, inc2, inc3 ;
    int      C0,  Delta, dif, AbsDelta ;
    int      L2, L1, L0, R0, R1, R2, RL0 ;
    int      Alpha, Beta ;
    int      ap, aq, small_gap;

    PtrInc  = dir ?  1 : width ;
    inc     = dir ?  width : 1 ;
    inc2    = inc << 1 ;
    inc3    = inc + inc2 ;
    Alpha = ALPHA_TABLE[Clip3(0, 63, QP + input->alpha_c_offset) ];    // jlzheng  7.8

    Beta = BETA_TABLE[Clip3(0, 63, QP + input->beta_offset) ];    // jlzheng  7.8

    for (pel = 0 ; pel < 16 ; pel++) {
        if ((Strng = Strength[pel >> 3])) {
            L2  = SrcPtr[-inc3] ;
            L1  = SrcPtr[-inc2] ;
            L0  = SrcPtr[-inc ] ;
            R0  = SrcPtr[    0] ;
            R1  = SrcPtr[ inc ] ;
            R2  = SrcPtr[ inc2] ;
            AbsDelta = abs(Delta = R0 - L0) ;

            if (AbsDelta < Alpha) {
                if ((abs(R0 - R1) < Beta) && (abs(L0 - L1) < Beta)) {
                    aq = (abs(R0 - R2) < Beta) ;
                    ap = (abs(L0 - L2) < Beta) ;

                    if (Strng == 2) {
                        RL0 = L0 + R0 ;
                        small_gap = (AbsDelta < ((Alpha >> 2) + 2));
                        aq &= small_gap;
                        ap &= small_gap;

                        SrcPtr[   0 ] = aq ? (R1 + RL0 +  R0 + 2) >> 2 : ((R1 << 1) + R0 + L0 + 2) >> 2 ;
                        SrcPtr[-inc ] = ap ? (L0 +  L1 + RL0 + 2) >> 2 : ((L1 << 1) + L0 + R0 + 2) >> 2 ;

                        SrcPtr[ inc ] = (aq && !Chro) ? (R0 + R1 + L0 + R1 + 2) >> 2 : SrcPtr[ inc ];
                        SrcPtr[-inc2] = (ap && !Chro) ? (L1 + L1 + L0 + R0 + 2) >> 2 : SrcPtr[-inc2];
                    } else { //Strng == 1
                        C0  = CLIP_TAB[Clip3(0, 63, QP + input->alpha_c_offset) ] ;    // jlzheng  7.12
                        dif             = IClip(-C0, C0, ((R0 - L0) * 3 + (L1 - R1) + 4) >> 3) ;
                        SrcPtr[  -inc ] = IClip(0, 255, L0 + dif) ;
                        SrcPtr[     0 ] = IClip(0, 255, R0 - dif) ;

                        if (!Chro) {
                            L0 = SrcPtr[-inc] ;
                            R0 = SrcPtr[   0] ;

                            if (ap) {
                                dif           = IClip(-C0, C0, ((L0 - L1) * 3 + (L2 - R0) + 4) >> 3) ;
                                SrcPtr[-inc2] = IClip(0, 255, L1 + dif) ;
                            }

                            if (aq) {
                                dif           = IClip(-C0, C0, ((R1 - R0) * 3 + (L0 - R2) + 4) >> 3) ;
                                SrcPtr[ inc ] = IClip(0, 255, R1 - dif) ;
                            }
                        }
                    }
                }
            }

            SrcPtr += PtrInc ;
            pel    += Chro ;
        } else {
            SrcPtr += PtrInc << (3 - Chro) ;
            pel    += 7 ;
        }  ;
    }
}

/*
*************************************************************************
* Function: EdgeLoopX
* Input:  byte* SrcPtr,int QP, int dir,int width,int Chro
* Output: void
* Return: void
*************************************************************************
*/
void EdgeLoopX(byte *SrcPtr, int QP, int dir, int width, int Chro, codingUnit *MbP, codingUnit *MbQ, int block_y,
               int block_x)
{

    int     pel , PtrInc;
    int     inc, inc2, inc3;
    int     AbsDelta ;
    int     L2, L1, L0, R0, R1, R2 ;
    int     fs; //fs stands for filtering strength.  The larger fs is, the stronger filter is applied.
    int     Alpha, Beta ;
    int     FlatnessL, FlatnessR;
    int  shift1 = input->sample_bit_depth - 8;
    int skipFilteringFlag = 0;
    // FlatnessL and FlatnessR describe how flat the curve is of one codingUnit.



    PtrInc  = dir ?  1 : width ;
    inc     = dir ?  width : 1 ;
    inc2    = inc << 1 ;
    inc3    = inc + inc2 ;
    if (input->sample_bit_depth > 8) { // coded as 10 bit, QP is added by 16 in config file
        Alpha =  ALPHA_TABLE[Clip3(0, 63, QP - (8 * (input->sample_bit_depth - 8)) + input->alpha_c_offset) ];    // jlzheng 7.8
        Beta =  BETA_TABLE[Clip3(0, 63, QP - (8 * (input->sample_bit_depth - 8)) + input->beta_offset) ];    // jlzheng 7.8
    } else { // coded as 8bit
        Alpha =  ALPHA_TABLE[Clip3(0, 63, QP + input->alpha_c_offset) ];    // jlzheng 7.8
        Beta =  BETA_TABLE[Clip3(0, 63, QP + input->beta_offset) ];    // jlzheng 7.8
    }

    Alpha = Alpha << shift1;
    Beta = Beta << shift1;

    for (pel = 0; pel < MIN_CU_SIZE ; pel++) {
        if (pel % 4 == 0) {
            skipFilteringFlag = SkipFiltering(MbP, MbQ, dir, 0, block_y + !dir * (pel >> 2), block_x + dir * (pel >> 2));
        }
        L2  = SrcPtr[-inc3] ;
        L1  = SrcPtr[-inc2] ;
        L0  = SrcPtr[-inc ] ;
        R0  = SrcPtr[    0] ;
        R1  = SrcPtr[ inc ] ;
        R2  = SrcPtr[ inc2] ;

        AbsDelta = abs(R0 - L0) ;
        if (!skipFilteringFlag && (AbsDelta < Alpha) && (AbsDelta > 1)) {
            if (abs(L1 - L0) < Beta) {
                FlatnessL = 2;
            } else {
                FlatnessL = 0;
            }

            if (abs(L2 - L0) < Beta) {
                FlatnessL += 1;
            }

            if (abs(R0 - R1) < Beta) {
                FlatnessR = 2;
            } else {
                FlatnessR = 0;
            }

            if (abs(R0 - R2) < Beta) {
                FlatnessR += 1;
            }


            switch (FlatnessL + FlatnessR) {
            case 6:

                if ((R1 == R0) && ((L0 == L1))) {
                    fs = 4;
                } else {
                    fs = 3;
                }

                break;
            case 5:

                if ((R1 == R0) && ((L0 == L1))) {
                    fs = 3;
                } else {
                    fs = 2;
                }

                break;
            case 4:

                if (FlatnessL == 2) {
                    fs = 2;
                } else {
                    fs = 1;
                }

                break;
            case 3:

                if (abs(L1 - R1) < Beta) {
                    fs = 1;
                } else {
                    fs = 0;
                }

                break;
            default:
                fs = 0;
            }

            if (Chro && fs > 0) {
                fs--;
            }

            switch (fs) {
            case 4:

                SrcPtr[-inc]  = (L0 + ((L0 + L2) << 3) + L2 + (R0 << 3) + (R2 << 2) + (R2 << 1) + 16) >> 5;             //L0
                SrcPtr[-inc2] = ((L0 << 3) - L0  + (L2 << 2) + (L2 << 1) + R0 + (R0 << 1) + 8) >> 4;           //L1
                SrcPtr[-inc3] = ((L0 << 2) + L2 + (L2 << 1) + R0 + 4) >> 3;       //L2

                SrcPtr[0]     = (R0 + ((R0 + R2) << 3) + R2 + (L0 << 3) + (L2 << 2) + (L2 << 1) + 16) >> 5;             //R0
                SrcPtr[inc]   = ((R0 << 3) - R0  + (R2 << 2) + (R2 << 1) + L0 + (L0 << 1) + 8) >> 4;           //R1
                SrcPtr[inc2]  = ((R0 << 2) + R2 + (R2 << 1) + L0 + 4) >> 3;       //R2
                break;
            case 3:



                SrcPtr[-inc] = (L2 + (L1 << 2)  + (L0 << 2) + (L0 << 1) + (R0 << 2) + R1 + 8) >> 4;           //L0
                SrcPtr[0] = (L1 + (L0 << 2) + (R0 << 2) + (R0 << 1) + (R1 << 2) + R2 + 8) >> 4;           //R0


                SrcPtr[-inc2] = (L2 * 3 + L1 * 8 + L0 * 4 + R0 + 8) >> 4;
                SrcPtr[inc] = (R2 * 3 + R1 * 8 + R0 * 4 + L0 + 8) >> 4;

                break;
            case 2:

                SrcPtr[-inc] = ((L1 << 1) + L1 + (L0 << 3) + (L0 << 1) + (R0 << 1) + R0 + 8) >> 4;
                SrcPtr[0] = ((L0 << 1) + L0 + (R0 << 3) + (R0 << 1) + (R1 << 1) + R1  + 8) >> 4;

                break;
            case 1:

                SrcPtr[-inc] = (L0 * 3 + R0 + 2) >> 2;
                SrcPtr[0] = (R0 * 3 + L0 + 2) >> 2;

                break;
            default:
                break;
            }

            SrcPtr += PtrInc ;    // Next row or column
            pel    += Chro ;
        } else {
            SrcPtr += PtrInc ;
            pel    += Chro ;
        }
    }
}
/*
*************************************************************************
* Function:  SkipFiltering
* Input:  codingUnit* MbP,codingUnit* MbQ,int dir,int edge,int block_y,int block_x
* Output: integer flag
* Return: 1 if skip filtering is needed.
*************************************************************************
*/

int SkipFiltering(codingUnit *MbP, codingUnit *MbQ, int dir, int edge, int block_y, int block_x)
{
    int    blk_x2, blk_y2, blk_x, blk_y ;
    int    ***tmp_mv   =  img->tmp_mv;
    int    iFlagSkipFiltering;

    blk_y = block_y;
    blk_x = block_x;
    blk_y2 = blk_y -  dir ;
    blk_x2 = blk_x - !dir ;

    switch (img->type) {
    case INTER_IMG:
    case F_IMG:
        if ((MbP->cbp == 0) && (MbQ->cbp == 0) &&
            (abs(tmp_mv[blk_y][blk_x][0] -   tmp_mv[blk_y2][blk_x2][0]) < 4) &&
            (abs(tmp_mv[blk_y][blk_x][1] -   tmp_mv[blk_y2][blk_x2][1]) < 4) &&
            (hc->refFrArr[blk_y][blk_x] == hc->refFrArr[blk_y2][blk_x2]) && hc->refFrArr[blk_y][blk_x] != -1)


        {
            iFlagSkipFiltering = 1;
        } else {
            iFlagSkipFiltering = 0;
        }

        break;

    default:
        iFlagSkipFiltering = 0;
    }

    return iFlagSkipFiltering;

}

void readParaSAO_one_SMB(int smb_index, int mb_y, int mb_x, int smb_mb_height, int smb_mb_width, int *slice_sao_on,
                         SAOBlkParam *saoBlkParam, SAOBlkParam *rec_saoBlkParam)
{
    int pix_x = mb_x << MIN_CU_SIZE_IN_BIT;
    int pix_y = mb_y << MIN_CU_SIZE_IN_BIT;
    int smb_pix_width = smb_mb_width << MIN_CU_SIZE_IN_BIT;
    int smb_pix_height = smb_mb_height << MIN_CU_SIZE_IN_BIT;
    if (!slice_sao_on[0] && !slice_sao_on[1] && !slice_sao_on[2]) {
        off_sao(rec_saoBlkParam);
        off_sao(saoBlkParam);
    } else {
        read_sao_smb(smb_index, pix_y, pix_x, smb_pix_width, smb_pix_height, slice_sao_on, saoBlkParam, rec_saoBlkParam);
    }

}
void read_sao_smb(int smb_index, int pix_y, int pix_x, int smb_pix_width, int smb_pix_height, int *slice_sao_on,
                  SAOBlkParam *sao_cur_param, SAOBlkParam *rec_sao_cur_param)
{
    int mb_x = pix_x >> MIN_CU_SIZE_IN_BIT;
    int mb_y = pix_y >> MIN_CU_SIZE_IN_BIT;
    int mb = mb_y * img->PicWidthInMbs + mb_x;
    SAOBlkParam merge_candidate[NUM_SAO_MERGE_TYPES][NUM_SAO_COMPONENTS];
    int merge_avail[NUM_SAO_MERGE_TYPES];
    int MergeLeftAvail, MergeUpAvail;
    int mergemode, saomode, saotype;
    int offset[32];
    int compIdx, i;
    int offsetTh = 7;
    int stBnd[2];
    int db_temp;
    getMergeNeighbor(smb_index, pix_y,  pix_x, smb_pix_width, smb_pix_height, input->g_uiMaxSizeInBit,
                     img->rec_saoBlkParams, merge_avail, merge_candidate);
    MergeLeftAvail = merge_avail[0];
    MergeUpAvail = merge_avail[1];
    mergemode = 0;
    if (MergeLeftAvail + MergeUpAvail > 0) {
        mergemode = read_sao_mergeflag(MergeLeftAvail, MergeUpAvail, mb);
    }
    if (mergemode) {
        /*sao_cur_param[SAO_Y].modeIdc = sao_cur_param[SAO_Cb].modeIdc = sao_cur_param[SAO_Cr].modeIdc = SAO_MODE_MERGE;
        sao_cur_param[SAO_Y].typeIdc = sao_cur_param[SAO_Cb].typeIdc = sao_cur_param[SAO_Cr].typeIdc = mergemode==1 ? SAO_MERGE_LEFT : SAO_MERGE_ABOVE;*/
        if (mergemode == 2) {
            copySAOParam_for_blk(rec_sao_cur_param, merge_candidate[SAO_MERGE_LEFT]);
        } else {
            assert(mergemode == 1);
            copySAOParam_for_blk(rec_sao_cur_param, merge_candidate[SAO_MERGE_ABOVE]);
        }
        copySAOParam_for_blk(sao_cur_param, rec_sao_cur_param);
        sao_cur_param->modeIdc = SAO_MODE_MERGE;
        sao_cur_param->typeIdc = mergemode;
    } else {
        for (compIdx = 0; compIdx < NUM_SAO_COMPONENTS; compIdx++) {
            if (!slice_sao_on[compIdx]) {
                sao_cur_param[compIdx].modeIdc = SAO_MODE_OFF;
            } else {
                if (1) {
                    saomode = read_sao_mode(mb);
                    switch (saomode) {
                    case 0:
                        sao_cur_param[compIdx].modeIdc = SAO_MODE_OFF;
                        break;
                    case 1:
                        sao_cur_param[compIdx].modeIdc = SAO_MODE_NEW;
                        sao_cur_param[compIdx].typeIdc = SAO_TYPE_BO;
                        break;
                    case 3:
                        sao_cur_param[compIdx].modeIdc = SAO_MODE_NEW;
                        sao_cur_param[compIdx].typeIdc = SAO_TYPE_EO_0;
                        break;
                    default:
                        assert(1);
                        break;
                    }
                } else {
                    sao_cur_param[compIdx].modeIdc = sao_cur_param[SAO_Cb].modeIdc;
                    if (sao_cur_param[compIdx].modeIdc != SAO_MODE_OFF) {
                        sao_cur_param[compIdx].typeIdc = (sao_cur_param[SAO_Cb].typeIdc == SAO_TYPE_BO) ? SAO_TYPE_BO : SAO_TYPE_EO_0;
                    }
                }
                if (sao_cur_param[compIdx].modeIdc == SAO_MODE_NEW) {
                    read_sao_offset(&(sao_cur_param[compIdx]), mb, offsetTh, offset);
                    if (1) {
                        saotype = read_sao_type(&(sao_cur_param[compIdx]), mb);
                    } else {
                        assert(compIdx == SAO_Cr && sao_cur_param[SAO_Cr].typeIdc == SAO_TYPE_EO_0);
                        saotype = sao_cur_param[SAO_Cb].typeIdc;
                    }

                    if (sao_cur_param[compIdx].typeIdc == SAO_TYPE_BO) {
                        memset(sao_cur_param[compIdx].offset, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);
                        db_temp = saotype >> NUM_SAO_BO_CLASSES_LOG2;
                        stBnd[0] = saotype - (db_temp << NUM_SAO_BO_CLASSES_LOG2);
                        stBnd[1] = (stBnd[0] + db_temp) % 32;
                        for (i = 0; i < 2; i++) {
                            sao_cur_param[compIdx].offset[stBnd[i]] = offset[i * 2];
                            sao_cur_param[compIdx].offset[(stBnd[i] + 1) % 32] = offset[i * 2 + 1];
                        }
                    } else {
                        assert(sao_cur_param[compIdx].typeIdc == SAO_TYPE_EO_0);
                        sao_cur_param[compIdx].typeIdc = saotype;
                        sao_cur_param[compIdx].offset[SAO_CLASS_EO_FULL_VALLEY] = offset[0];
                        sao_cur_param[compIdx].offset[SAO_CLASS_EO_HALF_VALLEY] = offset[1];
                        sao_cur_param[compIdx].offset[SAO_CLASS_EO_PLAIN      ] = 0;
                        sao_cur_param[compIdx].offset[SAO_CLASS_EO_HALF_PEAK  ] = offset[2];
                        sao_cur_param[compIdx].offset[SAO_CLASS_EO_FULL_PEAK  ] = offset[3];
                    }
                }
            }
        }
        copySAOParam_for_blk(rec_sao_cur_param, sao_cur_param);

    }

}


