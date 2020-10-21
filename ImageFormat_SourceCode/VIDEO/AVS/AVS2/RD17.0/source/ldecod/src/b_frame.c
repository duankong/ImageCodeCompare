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
* File name: b_frame.c
* Function: B picture decoding
*
*************************************************************************************
*/


#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/block_info.h"

/*
*************************************************************************
* Function:Copy decoded P frame to temporary image array
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void copy_Pframe()
{
    int i, j;

    /*
    * the mmin, mmax macros are taken out
    * because it makes no sense due to limited range of data type
    */

    for (i = 0; i < img->height; i++) {
        for (j = 0; j < img->width; j++) {
            hc->imgYPrev[i][j] = hc->imgY[i][j];
        }
    }

    for (i = 0; i < img->height_cr; i++) {
        for (j = 0; j < img->width_cr; j++) {
            hc->imgUVPrev[0][i][j] = hc->imgUV[0][i][j];
        }
    }

    for (i = 0; i < img->height_cr; i++) {
        for (j = 0; j < img->width_cr; j++) {
            hc->imgUVPrev[1][i][j] = hc->imgUV[1][i][j];
        }
    }
}

/*
*************************************************************************
* Function:init codingUnit B frames
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void init_codingUnit_Bframe(unsigned int uiPositionInPic)
{
    int i, j, k;
    int r, c, row, col, width, height;

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

    // reset vectors and pred. modes

    for (i = 0; i < 2 * num_of_orgMB_in_col; i++) {
        for (j = 0; j < 2 * num_of_orgMB_in_row; j++) {
            img->fw_mv[block8_y + j][block8_x + i][0] = img->fw_mv[block8_y + j][block8_x + i][1] = 0;
            img->bw_mv[block8_y + j][block8_x + i][0] = img->bw_mv[block8_y + j][block8_x + i][1] = 0;
        }
    }

    for (i = 0; i < 2 * num_of_orgMB_in_col; i++) {
        for (j = 0; j < 2 * num_of_orgMB_in_row; j++) {
            img->ipredmode[block8_y + j + 1][block8_x + i + 1] = -1; //cjw the default value should be -1
        }
    }

    // Set the reference frame information for motion vector prediction
    if (IS_INTRA(currMB) || IS_DIRECT(currMB)) {
        for (j = 0; j < 2 * num_of_orgMB_in_row; j++) {
            for (i = 0; i < 2 * num_of_orgMB_in_col; i++) {
                img->fw_refFrArr[block8_y + j][block8_x + i] = -1;
                img->bw_refFrArr[block8_y + j][block8_x + i] = -1;
            }
        }
    } else {
        for (j = 0; j < 2; j++) {
            for (i = 0; i < 2; i++) {
                k = 2 * j + i;
                get_b8_offset(currMB->cuType, currMB->ui_MbBitSize, i , j , &col , &row, &width, &height);
                for (r = 0; r < height; r++) {
                    for (c = 0; c < width; c++) {

                        img->fw_refFrArr[block8_y + row + r][block8_x + col + c] = ((currMB->b8pdir[k] == FORWARD ||
                                currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID) && currMB->b8mode[k] != 0 ? 0 : -1);
                        img->bw_refFrArr[block8_y + row + r][block8_x + col + c] = ((currMB->b8pdir[k] == BACKWARD ||
                                currMB->b8pdir[k] == SYM || currMB->b8pdir[k] == BID) && currMB->b8mode[k] != 0 ? 0 : -1);

                    }
                }
            }
        }
    }
}


