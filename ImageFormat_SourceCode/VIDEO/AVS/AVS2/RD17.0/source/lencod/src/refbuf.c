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

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <assert.h>

#include "refbuf.h"

static pel_t line[64];

/*
*************************************************************************
* Function:Reference buffer write routines
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void PutPel_14(pel_t **Pic, int y, int x, pel_t val)
{
    Pic [IMG_PAD_SIZE * 4 + y][IMG_PAD_SIZE * 4 + x] = val;
}

/*
*************************************************************************
* Function:Reference buffer read, Full pel
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

pel_t *FastLineX(int dummy, pel_t *Pic, int y, int x)
{
    return Pic + y * img->width + x;
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
pel_t *UMVLineX(int size, pel_t *Pic, int y, int x)
{
    int i, maxx;
    pel_t *Picy;

    Picy = Pic + max(0, min(img->height - 1, y)) * img->width;

    if (x < 0) {                          // Left edge
        maxx = min(0, x + size);

        for (i = x; i < maxx; i++) {
            line[i - x] = Picy [0];           // Replicate left edge pixel
        }

        maxx = x + size;

        for (i = 0; i < maxx; i++) {        // Copy non-edge pixels
            line[i - x] = Picy [i];
        }
    } else if (x > img->width - size) {   // Right edge
        maxx = img->width;

        for (i = x; i < maxx; i++) {
            line[i - x] = Picy [i];           // Copy non-edge pixels
        }

        maxx = x + size;

        for (i = max(img->width, x); i < maxx; i++) {
            line[i - x] = Picy [img->width - 1]; // Replicate right edge pixel
        }
    } else {                              // No edge
        return Picy + x;
    }

    return line;
}

/*
*************************************************************************
* Function:Reference buffer, 1/4 pel
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

pel_t UMVPelY_14(pel_t **Pic, int y, int x)
{
    int width4  = ((img->width + 2 * IMG_PAD_SIZE - 1) << 2);
    int height4 = ((img->height + 2 * IMG_PAD_SIZE - 1) << 2);

    x = x + IMG_PAD_SIZE * 4;
    y = y + IMG_PAD_SIZE * 4;

    if (x < 0) {
        if (y < 0) {
            return Pic [y & 3][x & 3];
        }

        if (y > height4) {
            return Pic [height4 + (y & 3) ][x & 3];
        }

        return Pic [y][x & 3];
    }

    if (x > width4) {
        if (y < 0) {
            return Pic [y & 3][width4 + (x & 3) ];
        }

        if (y > height4) {
            return Pic [height4 + (y & 3) ][width4 + (x & 3) ];
        }

        return Pic [y][width4 + (x & 3) ];
    }

    if (y < 0) {   // note: corner pixels were already processed
        return Pic [y & 3][x];
    }

    if (y > height4) {
        return Pic [height4 + (y & 3) ][x];
    }

    return Pic [y][x];
}

pel_t FastPelY_14(pel_t **Pic, int y, int x)
{
    return Pic [IMG_PAD_SIZE * 4 + y][IMG_PAD_SIZE * 4 + x];
}


