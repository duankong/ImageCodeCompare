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

#include "../../lcommon/inc/contributors.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "codingUnit.h"
#include "vlc.h"

const int NCBP[64][2] = {                                  // jlzheng 7.20
    { 4, 0}, {16, 19}, {17, 16}, {19, 15}, {14, 18}, {9, 11}, {22, 31}, { 8, 13},
    {11, 17}, {21, 30}, {10, 12}, { 7, 9}, {12, 10}, { 6, 7}, { 5, 8}, { 1, 1},
    {35, 4}, {47, 42}, {48, 38}, {38, 27}, {46, 39}, {36, 33}, {50, 59}, {26, 26},
    {45, 40}, {52, 58}, {41, 35}, {28, 25}, {37, 29}, {23, 24}, {31, 28}, { 2, 3},
    {43, 5}, {51, 51}, {56, 52}, {39, 37}, {55, 50}, {33, 43}, {62, 63}, {27, 44},
    {54, 53}, {60, 62}, {40, 48}, {32, 47}, {42, 34}, {24, 45}, {29, 49}, { 3, 6},
    {49, 14}, {53, 55}, {57, 56}, {25, 36}, {58, 54}, {30, 41}, {59, 60}, { 15, 21},
    {61, 57}, {63, 61}, {44, 46}, {18, 22}, {34, 32}, {13, 20}, {20, 23}, { 0, 2}
};

/*
*************************************************************************
* Function:ue_v, writes an ue(v) syntax element, returns the length in bits
* Input:
tracestring
the string for the trace file
value
the value to be coded
bitstream
the Bitstream the value should be coded into
* Output:
* Return:  Number of bits used by the coded syntax element
* Attention: This function writes always the bit buffer for the progressive scan flag, and
should not be used (or should be modified appropriately) for the interlace crap
When used in the context of the Parameter Sets, this is obviously not a
problem.
*************************************************************************
*/

int ue_v(char *tracestring, int value, Bitstream *bitstream)
{
    SyntaxElement symbol, *sym = &symbol;
    sym->type = SE_HEADER;
    sym->mapping = ue_linfo;               // Mapping rule: unsigned integer
    sym->value1 = value;

#if TRACE
    strncpy(sym->tracestring, tracestring, TRACESTRING_SIZE);
#endif

    return writeSyntaxElement_UVLC(sym, bitstream);
}

/*
*************************************************************************
* Function:ue_v, writes an ue(v) syntax element, returns the length in bits
* Input:
tracestring
the string for the trace file
value
the value to be coded
bitstream
the Bitstream the value should be coded into
* Output:
* Return: Number of bits used by the coded syntax element
* Attention:
This function writes always the bit buffer for the progressive scan flag, and
should not be used (or should be modified appropriately) for the interlace crap
When used in the context of the Parameter Sets, this is obviously not a
problem.
*************************************************************************
*/

int se_v(char *tracestring, int value, Bitstream *bitstream)
{
    SyntaxElement symbol, *sym = &symbol;
    sym->type = SE_HEADER;
    sym->mapping = se_linfo;               // Mapping rule: signed integer
    sym->value1 = value;

#if TRACE
    strncpy(sym->tracestring, tracestring, TRACESTRING_SIZE);
#endif

    return writeSyntaxElement_UVLC(sym, bitstream);
}

/*
*************************************************************************
* Function: u_1, writes a flag (u(1) syntax element, returns the length in bits,
always 1
* Input:
tracestring
the string for the trace file
value
the value to be coded
bitstream
the Bitstream the value should be coded into
* Output:
* Return: Number of bits used by the coded syntax element (always 1)
* Attention:This function writes always the bit buffer for the progressive scan flag, and
should not be used (or should be modified appropriately) for the interlace crap
When used in the context of the Parameter Sets, this is obviously not a
problem.
*************************************************************************
*/

int u_1(char *tracestring, int value, Bitstream *bitstream)
{
    SyntaxElement symbol, *sym = &symbol;

    sym->bitpattern = value;
    sym->len = 1;
    sym->type = SE_HEADER;
    sym->value1 = value;

#if TRACE
    strncpy(sym->tracestring, tracestring, TRACESTRING_SIZE);
#endif

    return writeSyntaxElement_fixed(sym, bitstream);
}

/*
*************************************************************************
* Function:u_v, writes a a n bit fixed length syntax element, returns the length in bits,
* Input:
tracestring
the string for the trace file
value
the value to be coded
bitstream
the Bitstream the value should be coded into
* Output:
* Return: Number of bits used by the coded syntax element
* Attention:This function writes always the bit buffer for the progressive scan flag, and
should not be used (or should be modified appropriately) for the interlace crap
When used in the context of the Parameter Sets, this is obviously not a
problem.
*************************************************************************
*/
int u_v(int n, char *tracestring, int value, Bitstream *bitstream)
{
    SyntaxElement symbol, *sym = &symbol;

    sym->bitpattern = value;
    sym->len = n;
    sym->type = SE_HEADER;
    sym->value1 = value;

#if TRACE
    strncpy(sym->tracestring, tracestring, TRACESTRING_SIZE);
#endif

    return writeSyntaxElement_fixed(sym, bitstream);
}


/*
*************************************************************************
* Function:mapping for ue(v) syntax elements
* Input:
ue
value to be mapped
info
returns mapped value
len
returns mapped value length
* Output:
* Return:
* Attention:
*************************************************************************
*/

void ue_linfo(int ue, int dummy, int *len, int *info)
{
    int i, nn;

    nn = (ue + 1) / 2;

    for (i = 0; i < 16 && nn != 0; i++) {
        nn /= 2;
    }

    *len = 2 * i + 1;
    *info = ue + 1 - (int) pow(2, i);

}

/*
*************************************************************************
* Function:mapping for ue(v) syntax elements
* Input:
ue
value to be mapped
info
returns mapped value
len
returns mapped value length
* Output:
* Return:
* Attention:
*************************************************************************
*/

void se_linfo(int se, int dummy, int *len, int *info)
{

    int i, n, sign, nn;

    sign = 0;

    if (se <= 0) {
        sign = 1;
    }

    n = abs(se) << 1;

    //n+1 is the number in the code table.  Based on this we find length and info

    nn = n / 2;

    for (i = 0; i < 16 && nn != 0; i++) {
        nn /= 2;
    }

    *len = i * 2 + 1;
    *info = n - (int) pow(2, i) + sign;

}

/*
*************************************************************************
* Function:
* Input:Number in the code table
* Output:lenght and info
* Return:
* Attention:
*************************************************************************
*/

void cbp_linfo_intra(int cbp, int dummy, int *len, int *info)
{
    extern const int NCBP[64][2];
    ue_linfo(NCBP[cbp][0], dummy, len, info);
}


/*
*************************************************************************
* Function:
* Input:Number in the code table
* Output:lenght and info
* Return:
* Attention:
*************************************************************************
*/

void cbp_linfo_inter(int cbp, int dummy, int *len, int *info)
{
    extern const int NCBP[64][2];
    ue_linfo(NCBP[cbp][1], dummy, len, info);
}

/*
*************************************************************************
* Function:Makes code word and passes it back
A code word has the following format: 0 0 0 ... 1 Xn ...X2 X1 X0.
* Input: Info   : Xn..X2 X1 X0                                             \n
Length : Total number of bits in the codeword
* Output:
* Return:
* Attention:this function is called with sym->inf > (1<<(sym->len/2)).
The upper bits of inf are junk
*************************************************************************
*/

int symbol2uvlc(SyntaxElement *sym)
{
    int suffix_len = sym->len / 2;
    sym->bitpattern = (1 << suffix_len) | (sym->inf & ((1 << suffix_len) - 1));
    return 0;
}

/*
*************************************************************************
* Function:generates UVLC code and passes the codeword to the buffer
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int writeSyntaxElement_UVLC(SyntaxElement *se, Bitstream *bitstream)
{
    if (se->golomb_maxlevels && (se->type == SE_LUM_DC_INTRA || se->type == SE_LUM_AC_INTRA ||
                                 se->type == SE_LUM_DC_INTER || se->type == SE_LUM_AC_INTER)) {
        return writeSyntaxElement_GOLOMB(se, bitstream);
    }

    if (se->type == SE_REFFRAME) {
        se->bitpattern = se->value1;
        se->len = 1;
    } else {
        se->mapping(se->value1, se->value2, & (se->len), & (se->inf));
        symbol2uvlc(se);
    }

    writeUVLC2buffer(se, bitstream);

#if TRACE

    if (se->type <= 1) {
        trace2out(se);
    }

#endif

    return (se->len);
}

/*
*************************************************************************
* Function:passes the fixed codeword to the buffer
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int writeSyntaxElement_fixed(SyntaxElement *se, Bitstream *bitstream)
{
    writeUVLC2buffer(se, bitstream);

#if TRACE

    if (se->type <= 1) {
        trace2out(se);
    }

#endif

    return (se->len);
}

/*
*************************************************************************
* Function:writes UVLC code to the appropriate buffer
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void  writeUVLC2buffer(SyntaxElement *se, Bitstream *currStream)
{
    int i;
    unsigned int mask = 1 << (se->len - 1);

    // Add the new bits to the bitstream.
    // Write out a byte if it is full
    for (i = 0; i < se->len; i++) {
        currStream->byte_buf <<= 1;

        if (se->bitpattern & mask) {
            currStream->byte_buf |= 1;
        }

        currStream->bits_to_go--;
        mask >>= 1;

        if (currStream->bits_to_go == 0) {
            currStream->bits_to_go = 8;
            currStream->streamBuffer[currStream->byte_pos++] = currStream->byte_buf;
            currStream->byte_buf = 0;
        }
    }

}

/*
*************************************************************************
* Function:Makes code word and passes it back
* Input:
Info   : Xn..X2 X1 X0                                             \n
Length : Total number of bits in the codeword
* Output:
* Return:
* Attention:
*************************************************************************
*/

int symbol2vlc(SyntaxElement *sym)
{
    int info_len = sym->len;

    // Convert info into a bitpattern int
    sym->bitpattern = 0;

    // vlc coding
    while (--info_len >= 0) {
        sym->bitpattern <<= 1;
        sym->bitpattern |= (0x01 & (sym->inf >> info_len));
    }

    return 0;
}

/*
*************************************************************************
* Function:write VLC for Run Before Next Coefficient, VLC0
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

/*
*************************************************************************
* Function:Write out a trace string on the trace file
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

#if TRACE
void trace2out(SyntaxElement *sym)
{
    static int bitcounter = 0;
    int i, chars;

    if (hc->p_trace != NULL) {
        putc('@', hc->p_trace);
        chars = fprintf(hc->p_trace, "%i", bitcounter);

        while (chars++ < 6) {
            putc(' ', hc->p_trace);
        }

        chars += fprintf(hc->p_trace, "%s", sym->tracestring);

        while (chars++ < 55) {
            putc(' ', hc->p_trace);
        }

        // Align bitpattern
        if (sym->len < 15) {
            for (i = 0 ; i < 15 - sym->len ; i++) {
                fputc(' ', hc->p_trace);
            }
        }

        // Print bitpattern
        bitcounter += sym->len;

        for (i = 1 ; i <= sym->len ; i++) {
            if ((sym->bitpattern >> (sym->len - i)) & 0x1) {
                fputc('1', hc->p_trace);
            } else {
                fputc('0', hc->p_trace);
            }
        }

        fprintf(hc->p_trace, " (%3d) \n", sym->value1);
    }

    fflush(hc->p_trace);
}
#endif
