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


#include "commonVariables.h"
#include "defines.h"
#include "contributors.h"


/////////////////////////////////////////////////////////////////////////////
/// external function declaration
/////////////////////////////////////////////////////////////////////////////

void  get_b8_offset(int blocktype, int uiBitSize, int i, int j, int *start_x, int *start_y, int *width, int *height);
void  get_pix_offset(int blocktype, int uiBitSize, int i, int j, int *start_x, int *start_y, int *width, int *height);

void get_mb_pos(int mb_addr, int *x, int *y, unsigned int uiBitSize);

//AEC
void CheckAvailabilityOfNeighbors(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic);

void getLuma8x8Neighbour(int b8_x, int b8_y, int rel_x, int rel_y, PixelPos *pix, int uiPosition, int uiBitSize,
                         codingUnit *currMB, int bw_flag);
void getNeighbour(int xN, int yN, int luma, PixelPos *pix, int uiPosition, int uiBitSize, codingUnit *currMB);

#if HALF_PIXEL_COMPENSATION
int getDeltas(
    int *delt,          //delt for original MV
    int *delt2,         //delt for scaled MV
    int OriPOC, int OriRefPOC, int ScaledPOC, int ScaledRefPOC
);
#endif
