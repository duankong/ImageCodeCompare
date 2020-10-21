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

#include "block_info.h"
#if HALF_PIXEL_COMPENSATION
#include "assert.h"
#endif



/////////////////////////////////////////////////////////////////////////////
/// variables definition
/////////////////////////////////////////////////////////////////////////////


int g_blk_size[20][2] = {
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2 },
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2 },
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2 },
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 2 },
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE },
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE },
    {MIN_BLOCK_SIZE,  MIN_BLOCK_SIZE * 2 },
    { MIN_BLOCK_SIZE, MIN_BLOCK_SIZE * 2 },
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE / 2 },
    {MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE * 3 / 2 },
    { MIN_BLOCK_SIZE * 2,  MIN_BLOCK_SIZE * 3 / 2 },
    { MIN_BLOCK_SIZE * 2, MIN_BLOCK_SIZE / 2 },
    { MIN_BLOCK_SIZE / 2, MIN_BLOCK_SIZE * 2 },
    { MIN_BLOCK_SIZE * 3 / 2, MIN_BLOCK_SIZE * 2 },
    { MIN_BLOCK_SIZE * 3 / 2, MIN_BLOCK_SIZE * 2 },
    { MIN_BLOCK_SIZE / 2, MIN_BLOCK_SIZE * 2 },
    { MIN_BLOCK_SIZE, MIN_BLOCK_SIZE },
    { MIN_BLOCK_SIZE,  MIN_BLOCK_SIZE },
    { MIN_BLOCK_SIZE, MIN_BLOCK_SIZE },
    { MIN_BLOCK_SIZE, MIN_BLOCK_SIZE }
};


int g_Left_Down_Avail_Matrix64[16][16] = {
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
}
;
int g_Left_Down_Avail_Matrix32[8][8] = {
    {1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
}
;
int g_Left_Down_Avail_Matrix16[4][4] = {
    {1, 0, 1, 0},
    {1, 0, 0, 0},
    {1, 0, 1, 0},
    {0, 0, 0, 0}
}
;
int g_Left_Down_Avail_Matrix8[2][2] = {
    {1, 0},
    {0, 0},
}
;

int g_Up_Right_Avail_Matrix64[16][16] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0}
}
;
int g_Up_Right_Avail_Matrix32[8][8] =
    //qyu 0823 0: 8 1:16 2: 32  pu size
{
    {1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 0, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 1, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0},
    {1, 1, 1, 0, 1, 1, 1, 0},
    {1, 0, 1, 0, 1, 0, 1, 0}
}
;
int g_Up_Right_Avail_Matrix16[4][4] = {
    {1, 1, 1, 1},
    {1, 0, 1, 0},
    {1, 1, 1, 0},
    {1, 0, 1, 0}
}
;
int g_Up_Right_Avail_Matrix8[2][2] = {
    {1, 1},
    {1, 0},
}
;

/////////////////////////////////////////////////////////////////////////////
/// local function declaration
/////////////////////////////////////////////////////////////////////////////






/////////////////////////////////////////////////////////////////////////////
/// function definition
/////////////////////////////////////////////////////////////////////////////
void   get_b8_offset(int blocktype, int uiBitSize, int i, int j, int *start_x, int *start_y, int *width, int *height)
{
    if (uiBitSize == MIN_CU_SIZE_IN_BIT || (blocktype < 4 || blocktype > 7)) {
        *start_x = i << (uiBitSize - MIN_CU_SIZE_IN_BIT);
        *start_y = j << (uiBitSize - MIN_CU_SIZE_IN_BIT);
        *width = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);
        *height = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    } else {
        int k = j * 2 + i;
        int h0 = (blocktype == 4 ||
                  blocktype == 5) ? (g_blk_size[blocktype * 2][0] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_CU_SIZE_IN_BIT   :
                 (g_blk_size[blocktype * 2][0] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT;
        int v0 = (blocktype == 4 ||
                  blocktype == 5) ? (g_blk_size[blocktype * 2][1] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT  :
                 (g_blk_size[blocktype * 2][1] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_CU_SIZE_IN_BIT;
        int h1 = (blocktype == 4 ||
                  blocktype == 5) ? (g_blk_size[blocktype * 2 + 1][0] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_CU_SIZE_IN_BIT :
                 (g_blk_size[blocktype * 2 + 1][0] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >> MIN_BLOCK_SIZE_IN_BIT;
        int v1 = (blocktype == 4 ||
                  blocktype == 5) ? (g_blk_size[blocktype * 2 + 1][1] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >>
                 MIN_BLOCK_SIZE_IN_BIT : (g_blk_size[blocktype * 2 + 1][1] << (uiBitSize - MIN_CU_SIZE_IN_BIT)) >>
                 MIN_CU_SIZE_IN_BIT;
        switch (k) {
        case 0:
            *start_x = 0;
            *start_y = 0;
            *width = h0;
            *height = v0;
            break;
        case 1:
            *start_x = h0;
            *start_y = 0;
            *width = h1;
            *height = v0;
            break;
        case 2:
            *start_x = 0;
            *start_y = v0;
            *width = h0;
            *height = v1;
            break;
        case 3:
            *start_x = h0;
            *start_y = v0;
            *width = h1;
            *height = v1;
            break;
        }
    }
}
void   get_pix_offset(int blocktype, int uiBitSize, int i, int j, int *start_x, int *start_y, int *width, int *height)
{
    int h0 = g_blk_size[blocktype * 2][0] ;
    int v0 = g_blk_size[blocktype * 2][1] ;
    int h1 = g_blk_size[blocktype * 2 + 1][0] ;
    int v1 = g_blk_size[blocktype * 2 + 1][1] ;

    int block8x8 = 2 * j + i;


    h0 = ((h0 == MIN_CU_SIZE) ? (h0 / 2) : h0) << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    v0 = ((v0 == MIN_CU_SIZE) ? (v0 / 2) : v0) << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    h1 = ((h1 == MIN_CU_SIZE) ? (h1 / 2) : h1) << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    v1 = ((v1 == MIN_CU_SIZE) ? (v1 / 2) : v1) << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    switch (block8x8) {
    case 0:
        *start_x = 0;
        *start_y = 0;
        *width = h0;
        *height = v0;
        break;
    case 1:
        *start_x = h0;
        *start_y = 0;
        *width = h1;
        *height = v0;
        break;
    case 2:
        *start_x = 0;
        *start_y = v0;
        *width = h0;
        *height = v1;
        break;
    case 3:
        *start_x = h0;
        *start_y = v0;
        *width = h1;
        *height = v1;
        break;
    }
}


/*!
************************************************************************
* \brief
*    returns the x and y sample coordinates for a given MbAddress
************************************************************************
*/
void get_mb_pos(int mb_addr, int *x, int *y, unsigned int uiBitSize)
{
    int SizeScale = 1 << (uiBitSize - MIN_CU_SIZE_IN_BIT);
    *x = (mb_addr % (img->PicWidthInMbs)) / SizeScale;
    *y = (mb_addr / (img->PicWidthInMbs)) / SizeScale;

    (*x) *= MIN_CU_SIZE * SizeScale;
    (*y) *= MIN_CU_SIZE * SizeScale;
}


/*!
************************************************************************
* \brief
*    Check for available neighbouring blocks
*    and set pointers in current codingUnit
************************************************************************
*/
void CheckAvailabilityOfNeighbors(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic)
{
    //added by lzhang 10.23
    int i, j;
    const int mb_width = img->width / MIN_CU_SIZE;

    int   pix_x   = (uiPositionInPic % mb_width) * MIN_CU_SIZE;
    int   pix_y   = (uiPositionInPic / mb_width) * MIN_CU_SIZE;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            currMB->mb_available[i][j] = NULL;
        }
    }

    currMB->mb_available[1][1] = currMB;

    // Check MB to the left
    if (pix_x >= MIN_CU_SIZE) {
        int remove_prediction = currMB->slice_nr != img->mb_data[uiPositionInPic - 1].slice_nr;

        if (!remove_prediction) {
            currMB->mb_available[1][0] = & (img->mb_data[uiPositionInPic - 1]);
        }
    }

    // Check MB above
    if (pix_y >= MIN_CU_SIZE) {
        int remove_prediction = currMB->slice_nr != img->mb_data[uiPositionInPic - mb_width].slice_nr;

        // upper blocks
        if (!remove_prediction) {
            currMB->mb_available[0][1] = & (img->mb_data[uiPositionInPic - mb_width]);
        }
    }

    // Check MB left above
    if (pix_y >= MIN_CU_SIZE && pix_x >= MIN_CU_SIZE) {
        int remove_prediction = currMB->slice_nr != img->mb_data[uiPositionInPic - mb_width - 1].slice_nr;
        if (!remove_prediction) {
            currMB->mb_available[0][0] = & (img->mb_data[uiPositionInPic - mb_width - 1]);
        }
    }

    // Check MB right above
    if (pix_y >= MIN_CU_SIZE && pix_x < (img->width - (1 << uiBitSize))) {
        if (currMB->slice_nr == img->mb_data[- mb_width + min(uiPositionInPic + (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT)),
                                             uiPositionInPic + (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT))) ].slice_nr) {
            currMB->mb_available[0][2] = & (img->mb_data[-mb_width + min(uiPositionInPic + (1 <<
                                            (uiBitSize - MIN_CU_SIZE_IN_BIT)), uiPositionInPic + (1 << (uiBitSize - MIN_CU_SIZE_IN_BIT))) ]);
        }
    }

    currMB->mb_available_left = currMB->mb_available[1][0];
    currMB->mb_available_up   = currMB->mb_available[0][1];

}

/*!
************************************************************************
* \brief
*    get neighbouring positions. MB AFF is automatically used from img structure
* \param curr_mb_nr
*   current codingUnit number (decoding order)
* \param xN
*    input x position
* \param yN
*    input y position
* \param luma
*    1 if luma coding, 0 for chroma
* \param pix
*    returns position informations
* \author
*    added by lzhang
************************************************************************
*/
void getNeighbour(int xN, int yN, int luma, PixelPos *pix, int uiPosition, int uiBitSize, codingUnit *currMB)
{
    int maxWH;
    PixelPos *neighborPix = (PixelPos *) malloc(sizeof(PixelPos));
    codingUnit *neighborMB;
    int neighborMBsize;

    if (luma) {
        maxWH = (1 << uiBitSize);
    } else {
        maxWH = (1 << uiBitSize) / 2;
    }

    if ((xN < 0) && (yN < 0)) {
        pix->mb_addr   = uiPosition - 1 - img->PicWidthInMbs;
        pix->available = pix->mb_addr >= 0 && (img->mb_data[pix->mb_addr].slice_nr == img->mb_data[uiPosition].slice_nr);

        if (uiPosition % img->PicWidthInMbs == 0) {
            pix->available = 0;
        } else if (uiPosition < img->PicWidthInMbs) {
            pix->available = 0;
        }
    } else if ((xN < 0) && ((yN >= 0) && (yN < maxWH))) {
        pix->mb_addr  = uiPosition - 1 + img->PicWidthInMbs * (yN >> MIN_CU_SIZE_IN_BIT);
        pix->available = pix->mb_addr >= 0 && (img->mb_data[pix->mb_addr].slice_nr == img->mb_data[uiPosition].slice_nr);

        if (uiPosition % img->PicWidthInMbs == 0) {
            pix->available = 0;
        }
    } else  if (((xN >= 0) && (xN < maxWH)) && (yN < 0)) {
        pix->mb_addr  = uiPosition - img->PicWidthInMbs + (xN >> MIN_CU_SIZE_IN_BIT);
        pix->available = pix->mb_addr >= 0 && (img->mb_data[pix->mb_addr].slice_nr == img->mb_data[uiPosition].slice_nr);

        if (uiPosition < img->PicWidthInMbs) {
            pix->available = 0;
        }
    } else  if (((xN >= 0) && (xN < maxWH)) && ((yN >= 0) && (yN < maxWH))) {
        pix->mb_addr  = uiPosition;
        pix->available = 1;
    } else  if ((xN >= maxWH) && (yN < 0)) {
        pix->mb_addr  = uiPosition - img->PicWidthInMbs + (xN >> MIN_CU_SIZE_IN_BIT);
        pix->available = pix->mb_addr >= 0 && (img->mb_data[pix->mb_addr].slice_nr == img->mb_data[uiPosition].slice_nr);

        if (uiPosition < img->PicWidthInMbs) {
            pix->available = 0;
        } else if ((uiPosition + (xN >> MIN_CU_SIZE_IN_BIT)) / img->PicWidthInMbs != uiPosition / img->PicWidthInMbs) {
            pix->available = 0;
        }
    } else {
        pix->available = 0;
    }

    //if (pix->available || img->DeblockCall)

    if (pix->available) {
        if (pix->mb_addr == uiPosition) {
            neighborMB = currMB;
        } else {
            neighborMB = &img->mb_data[pix->mb_addr];
        }
        neighborMBsize = neighborMB->ui_MbBitSize ;
        if (luma) {
            maxWH = (1 << neighborMBsize);
        } else {
            maxWH = (1 << neighborMBsize) / 2;
        }

        pix->x = xN;
        pix->y = yN ;
        get_mb_pos(uiPosition, &(pix->pos_x), &(pix->pos_y), uiBitSize); //x2, y2


//      get_mb_pos ( pix->mb_addr, & ( pix->pos_x ), & ( pix->pos_y ), uiBitSize ); //x2, y2
        get_mb_pos(pix->mb_addr, & (neighborPix->pos_x), & (neighborPix->pos_y), neighborMB->ui_MbBitSize);        //x1, y1

        if (luma) {
            int tmpPosX = pix->pos_x;
            int tmpPosY = pix->pos_y;
            pix->pos_x += pix->x; //pix->x   x3
            pix->pos_y += pix->y; //pix->y   y3

            pix->x += (tmpPosX - neighborPix->pos_x);   //x3 - (x1-x2)
            pix->y += (tmpPosY - neighborPix->pos_y);   //y3 - (y1-y2)
        } else {
            pix->pos_x = (pix->pos_x / 2) + pix->x;
            pix->pos_y = (pix->pos_y / 2) + pix->y;
        }
//#endif
    }

    free(neighborPix);
}

/*!
************************************************************************
* \brief
*    get neighbouring  get neighbouring 8x8 luma block
* \param curr_mb_nr
*   current codingUnit number (decoding order)
* \param block_x
*    input x block position
* \param block_y
*    input y block position
* \param rel_x
*    relative x position of neighbor
* \param rel_y
*    relative y position of neighbor
* \param pix
*    returns position informations
* \author
*    added by lzhang
************************************************************************
*/

void getLuma8x8Neighbour(int b8_x, int b8_y, int rel_x, int rel_y, PixelPos *pix, int uiPosition, int uiBitSize,
                         codingUnit *currMB, int bw_flag)
{

    int width = (1 << uiBitSize);
    int height = (1 << uiBitSize);
    int sizeOfNeighborBlock;
    codingUnit *neighborMB;
    int b8_dest, pdir_dest, pdir, b8;
    int x, y;
    b8_x = (b8_x == 0) ? 0 : 1 ;
    b8_y = (b8_y == 0) ? 0 : 1 ;
    b8 = b8_y * 2 + b8_x;
    x = width * b8_x / 2 + rel_x; //qyu night
    y = height * b8_y / 2 + rel_y;

    getNeighbour(x, y, 1, pix, uiPosition, uiBitSize, currMB);

    if (pix->available) {
        //added by yuanyuan 09.4.20
        if (pix->mb_addr == uiPosition) {
            neighborMB = currMB;
        } else {
            neighborMB = &img->mb_data[pix->mb_addr];
        }

        sizeOfNeighborBlock = neighborMB->ui_MbBitSize;
        pix->x = pix->x / ((1 << sizeOfNeighborBlock) / 2);
        pix->y = pix->y / ((1 << sizeOfNeighborBlock) / 2);

        b8_dest = pix->x + pix->y * 2;
        pdir = currMB->b8pdir[b8];//img->mb_data[uiPosition].b8pdir[b8];
        pdir_dest = neighborMB->b8pdir[b8_dest];

        if ((pdir == FORWARD && pdir_dest == BACKWARD) || (pdir == BACKWARD && pdir_dest == FORWARD) || (pdir == SYM &&
                pdir_dest == BACKWARD) || (pdir == BACKWARD && pdir_dest == SYM) || (pdir == BID && !bw_flag &&
                        pdir_dest == BACKWARD) || (pdir == BID && bw_flag && pdir_dest == FORWARD) || (pdir == BID && bw_flag &&
                                pdir_dest == SYM) || (pdir == DUAL && bw_flag && pdir_dest == FORWARD)) {
            pix->available = 0;
        }

        pix->pos_x /= MIN_BLOCK_SIZE;
        pix->pos_y /= MIN_BLOCK_SIZE;
    }
}

#if HALF_PIXEL_COMPENSATION
int getDeltas(
    int *delt,          //delt for original MV
    int *delt2,         //delt for scaled MV
    int OriPOC, int OriRefPOC, int ScaledPOC, int ScaledRefPOC
)
{
    int factor = 2;

    *delt = 0;
    *delt2 = 0;
    if (img->is_field_sequence == 0) {
        return -1;
    }
    //assert(img->picture_structure == 1 );

    OriPOC  = (OriPOC  + 512) % 512;
    OriRefPOC  = (OriRefPOC  + 512) % 512;
    ScaledPOC  = (ScaledPOC  + 512) % 512;
    ScaledRefPOC  = (ScaledRefPOC  + 512) % 512;
    //assert((OriPOC % factor) + (OriRefPOC % factor) + (ScaledPOC % factor) + (ScaledRefPOC % factor) == 0);

    OriPOC /= factor;
    OriRefPOC /= factor;
    ScaledPOC /= factor;
    ScaledRefPOC /= factor;

    if (img->is_top_field) {   //Scaled is top field
        *delt2 = (ScaledRefPOC % 2) != (ScaledPOC % 2) ? 2 : 0;

        if ((ScaledPOC % 2) == (OriPOC % 2)) {   //ori is top
            *delt = (OriRefPOC % 2) != (OriPOC % 2) ? 2 : 0;
        } else {
            *delt = (OriRefPOC % 2) != (OriPOC % 2) ? -2 : 0;
        }
    } else { //Scaled is bottom field
        *delt2 = (ScaledRefPOC % 2) != (ScaledPOC % 2) ? -2 : 0;
        if ((ScaledPOC % 2) == (OriPOC % 2)) {   //ori is bottom
            *delt = (OriRefPOC % 2) != (OriPOC % 2) ? -2 : 0;
        } else {
            *delt = (OriRefPOC % 2) != (OriPOC % 2) ? 2 : 0;
        }
    }
    return 0;
}
#endif
