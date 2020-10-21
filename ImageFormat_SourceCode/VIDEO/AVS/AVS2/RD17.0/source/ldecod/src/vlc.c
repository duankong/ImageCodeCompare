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
* File name: vlc.c
* Function: VLC support functions
*
*************************************************************************************
*/

#include "../../lcommon/inc/contributors.h"

#include <math.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>
#include <assert.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "vlc.h"
#include "header.h"


// A little trick to avoid those horrible #if TRACE all over the source code
#if TRACE
#define SYMTRACESTRING(s) strncpy(sym->tracestring,s,TRACESTRING_SIZE)
#else
#define SYMTRACESTRING(s) // do nothing
#endif

extern void tracebits(const char *trace_str,  int len,  int info, int value1);

/*
*************************************************************************
* Function:ue_v, reads an ue(v) syntax element, the length in bits is stored in
the global UsedBits variable
* Input:
tracestring
the string for the trace file
bitstream
the stream to be read from
* Output:
* Return: the value of the coded syntax element
* Attention:
*************************************************************************
*/

int ue_v(char *tracestring)
{
    SyntaxElement symbol, *sym = &symbol;

    assert(currStream->streamBuffer != NULL);
    sym->type = SE_HEADER;
    sym->mapping = linfo_ue;   // Mapping rule
    SYMTRACESTRING(tracestring);
    readSyntaxElement_VLC(sym);

    return sym->value1;
}

/*
*************************************************************************
* Function:ue_v, reads an se(v) syntax element, the length in bits is stored in
the global UsedBits variable
* Input:
tracestring
the string for the trace file
bitstream
the stream to be read from
* Output:
* Return: the value of the coded syntax element
* Attention:
*************************************************************************
*/


int se_v(char *tracestring)
{
    SyntaxElement symbol, *sym = &symbol;

    assert(currStream->streamBuffer != NULL);
    sym->type = SE_HEADER;
    sym->mapping = linfo_se;   // Mapping rule: signed integer
    SYMTRACESTRING(tracestring);
    readSyntaxElement_VLC(sym);

    return sym->value1;
}

/*
*************************************************************************
* Function:ue_v, reads an u(v) syntax element, the length in bits is stored in
the global UsedBits variable
* Input:
tracestring
the string for the trace file
bitstream
the stream to be read from
* Output:
* Return: the value of the coded syntax element
* Attention:
*************************************************************************
*/

int u_v(int LenInBits, char *tracestring)
{
    SyntaxElement symbol, *sym = &symbol;

    assert(currStream->streamBuffer != NULL);
    sym->type = SE_HEADER;
    sym->mapping = linfo_ue;   // Mapping rule
    sym->len = LenInBits;
    SYMTRACESTRING(tracestring);
    readSyntaxElement_FLC(sym);

    return sym->inf;
}

/*
*************************************************************************
* Function:mapping rule for ue(v) syntax elements
* Input:lenght and info
* Output:number in the code table
* Return:
* Attention:
*************************************************************************
*/

void linfo_ue(int len, int info, int *value1, int *dummy)
{
    *value1 = (int) pow(2, (len / 2)) + info - 1;        // *value1 = (int)(2<<(len>>1))+info-1;
}

/*
*************************************************************************
* Function:mapping rule for se(v) syntax elements
* Input:lenght and info
* Output:signed mvd
* Return:
* Attention:
*************************************************************************
*/


void linfo_se(int len,  int info, int *value1, int *dummy)
{
    int n;
    n = (int) pow(2, (len / 2)) + info - 1;
    *value1 = (n + 1) / 2;

    if ((n & 0x01) == 0) {                      // lsb is signed bit
        *value1 = -*value1;
    }

}

/*
*************************************************************************
* Function:lenght and info
* Input:
* Output:cbp (intra)
* Return:
* Attention:
*************************************************************************
*/


void linfo_cbp_intra(int len, int info, int *cbp, int *dummy)
{
    // extern const byte NCBP[48][2];
    extern const byte NCBP[64][2];    //jlzheng 7.20
    int cbp_idx;
    linfo_ue(len, info, &cbp_idx, dummy);
    *cbp = NCBP[cbp_idx][0];
}

/*
*************************************************************************
* Function:
* Input:lenght and info
* Output:cbp (inter)
* Return:
* Attention:
*************************************************************************
*/

void linfo_cbp_inter(int len, int info, int *cbp, int *dummy)
{
    //extern const byte NCBP[48][2];
    extern const byte NCBP[64][2];  //cjw 20060321
    int cbp_idx;
    linfo_ue(len, info, &cbp_idx, dummy);
    *cbp = NCBP[cbp_idx][1];
}

/*
*************************************************************************
* Function:read next UVLC codeword from UVLC-partition and
map it to the corresponding syntax element
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


int readSyntaxElement_VLC(SyntaxElement *sym)
{
    int frame_bitoffset = currStream->frame_bitoffset;
    unsigned char *buf = currStream->streamBuffer;
    int BitstreamLengthInBytes = currStream->bitstream_length;

    sym->len =  GetVLCSymbol(buf, frame_bitoffset, & (sym->inf), BitstreamLengthInBytes);

    if (sym->len == -1) {
        return -1;
    }

    currStream->frame_bitoffset += sym->len;
    sym->mapping(sym->len, sym->inf, & (sym->value1), & (sym->value2));

#if TRACE
    tracebits(sym->tracestring, sym->len, sym->inf, sym->value1);
#endif

    return 1;
}


int GetVLCSymbol(unsigned char buffer[], int totbitoffset, int *info, int bytecount)
{

    register int inf;
    long byteoffset;      // byte from start of buffer
    int bitoffset;      // bit from start of byte
    int ctr_bit = 0;    // control bit for current bit posision
    int bitcounter = 1;
    int len;
    int info_bit;

    byteoffset = totbitoffset / 8;
    bitoffset = 7 - (totbitoffset % 8);
    ctr_bit = (buffer[byteoffset] & (0x01 << bitoffset));     // set up control bit

    len = 1;

    while (ctr_bit == 0) {
        // find leading 1 bit
        len++;
        bitoffset -= 1;
        bitcounter++;

        if (bitoffset < 0) {
            // finish with current byte ?
            bitoffset = bitoffset + 8;
            byteoffset++;
        }

        ctr_bit = buffer[byteoffset] & (0x01 << (bitoffset));
    }

    // make infoword
    inf = 0;                        // shortest possible code is 1, then info is always 0

    for (info_bit = 0; (info_bit < (len - 1)); info_bit++) {
        bitcounter++;
        bitoffset -= 1;

        if (bitoffset < 0) {
            // finished with current byte ?
            bitoffset = bitoffset + 8;
            byteoffset++;
        }

        if (byteoffset > bytecount) {
            return -1;
        }

        inf = (inf << 1);

        if (buffer[byteoffset] & (0x01 << (bitoffset))) {
            inf |= 1;
        }
    }

    *info = inf;

    return bitcounter;           // return absolute offset in bit from start of frame
}


/*
*************************************************************************
* Function:read FLC codeword from UVLC-partition
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int readSyntaxElement_FLC(SyntaxElement *sym)
{
    int frame_bitoffset = currStream->frame_bitoffset;
    unsigned char *buf = currStream->streamBuffer;
    int BitstreamLengthInBytes = currStream->bitstream_length;

    if ((GetBits(buf, frame_bitoffset, & (sym->inf), BitstreamLengthInBytes, sym->len)) < 0) {
        return -1;
    }

    currStream->frame_bitoffset += sym->len; // move bitstream pointer
    sym->value1 = sym->inf;

#if TRACE
    tracebits2(sym->tracestring, sym->len, sym->inf);
#endif

    return 1;
}

/*
*************************************************************************
* Function:Reads bits from the bitstream buffer
* Input:
byte buffer[]
containing VLC-coded data bits
int totbitoffset
bit offset from start of partition
int bytecount
total bytes in bitstream
int numbits
number of bits to read
* Output:
* Return:
* Attention:
*************************************************************************
*/


int GetBits(unsigned char buffer[], int totbitoffset, int *info, int bytecount, int numbits)
{
    register int inf;
    long byteoffset;      // byte from start of buffer
    int bitoffset;      // bit from start of byte

    int bitcounter = numbits;

    byteoffset = totbitoffset / 8;
    bitoffset = 7 - (totbitoffset % 8);

    inf = 0;

    while (numbits) {
        inf <<= 1;
        inf |= (buffer[byteoffset] & (0x01 << bitoffset)) >> bitoffset;
        numbits--;
        bitoffset--;

        if (bitoffset < 0) {
            byteoffset++;
            bitoffset += 8;

            if (byteoffset > bytecount) {
                return -1;
            }
        }
    }

    *info = inf;

    return bitcounter;           // return absolute offset in bit from start of frame
}

////////////////////////////////////////////////////////////////////////////
// Yulj 2004.07.15
// for decision of slice end.
////////////////////////////////////////////////////////////////////////////
int get_uv(int LenInBits, char *tracestring)
{
    SyntaxElement symbol, *sym = &symbol;

    assert(currStream->streamBuffer != NULL);
    sym->mapping = linfo_ue;   // Mapping rule
    sym->len = LenInBits;
    SYMTRACESTRING(tracestring);
    GetSyntaxElement_FLC(sym);

    return sym->inf;
}

/////////////////////////////////////////////////////////////
// Yulj 2004.07.15
// for decision of slice end.
/////////////////////////////////////////////////////////////
int GetSyntaxElement_FLC(SyntaxElement *sym)
{
    int frame_bitoffset = currStream->frame_bitoffset;
    unsigned char *buf = currStream->streamBuffer;
    int BitstreamLengthInBytes = currStream->bitstream_length;

    if ((GetBits(buf, frame_bitoffset, & (sym->inf), BitstreamLengthInBytes, sym->len)) < 0) {
        return -1;
    }

    sym->value1 = sym->inf;

#if TRACE
    tracebits2(sym->tracestring, sym->len, sym->inf);
#endif

    return 1;
}
