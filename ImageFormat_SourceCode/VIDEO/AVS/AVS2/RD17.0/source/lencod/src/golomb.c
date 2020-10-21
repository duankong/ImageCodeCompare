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
* File name: golomb.c
* Function: Description
*
*************************************************************************************
*/


#include <assert.h>
#include "codingUnit.h"
#include "vlc.h"

/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void encode_golomb_word(unsigned int symbol, unsigned int grad0, unsigned int max_levels, unsigned int *res_bits,
                        unsigned int *res_len)
{
    unsigned int level, res, numbits;

    res = 1UL << grad0;
    level = 1UL;
    numbits = 1UL + grad0;

    //find golomb level
    while (symbol >= res && level < max_levels) {
        symbol -= res;
        res = res << 1;
        level++;
        numbits += 2UL;
    }

    if (level >= max_levels) {
        if (symbol >= res) {
            symbol = res - 1UL;  //crop if too large.
        }
    }

    //set data bits
    *res_bits = res | symbol;
    *res_len = numbits;
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

void encode_multilayer_golomb_word(unsigned int symbol, const unsigned int *grad, const unsigned int *max_levels,
                                   unsigned int *res_bits, unsigned int *res_len)
{
    unsigned accbits, acclen, bits, len, tmp;
    accbits = acclen = 0UL;

    while (1) {
        encode_golomb_word(symbol, *grad, *max_levels, &bits, &len);
        accbits = (accbits << len) | bits;
        acclen += len;
        assert(acclen <= 32UL);    //we'l be getting problems if this gets longer than 32 bits.
        tmp = *max_levels - 1UL;

        if (!((len == (tmp << 1) + (*grad)) &&
              (bits == (1UL << (tmp + *grad)) - 1UL))) {         //is not last possible codeword? (Escape symbol?)
            break;
        }

        tmp = *max_levels;
        symbol -= (((1UL << tmp) - 1UL) << (*grad)) - 1UL;
        grad++;
        max_levels++;
    }

    *res_bits = accbits;
    *res_len = acclen;
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

int writeSyntaxElement_GOLOMB(SyntaxElement *se, Bitstream *bitstream)
{
    unsigned int bits, len, i;
    unsigned int grad[4], max_lev[4];

    if (!(se->golomb_maxlevels & ~0xFF)) {      //only bits 0-7 used? This means normal Golomb word.
        encode_golomb_word(se->value1, se->golomb_grad, se->golomb_maxlevels, &bits, &len);
    } else {
        for (i = 0UL; i < 4UL; i++) {
            grad[i] = (se->golomb_grad >> (i << 3)) & 0xFFUL;
            max_lev[i] = (se->golomb_maxlevels >> (i << 3)) & 0xFFUL;
        }

        encode_multilayer_golomb_word(se->value1, grad, max_lev, &bits, &len);
    }

    se->len = len;
    se->bitpattern = bits;

    writeUVLC2buffer(se, bitstream);

#if TRACE
    snprintf(se->tracestring, TRACESTRING_SIZE, "Coefficients");

    if (se->type <= 1) {
        trace2out(se);
    }

#endif

    return (se->len);
}
