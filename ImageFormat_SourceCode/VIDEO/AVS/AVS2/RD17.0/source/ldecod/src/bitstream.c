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
* File name: bitstream.c
* Function: decode bitstream
*
*************************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "annexb.h"
#include "../../lcommon/inc/memalloc.h"
#include "biaridecod.h"

#define SVA_STREAM_BUF_SIZE 1024

extern StatBits *StatBitsPtr; //rm52k_r2
FILE *bitsfile = NULL;    //!< the bit stream file

typedef struct {
    FILE *f;
    unsigned char buf[SVA_STREAM_BUF_SIZE];
    unsigned int  uClearBits;
    unsigned int  uPre3Bytes;
    int iBytePosition;
    int iBufBytesNum;
    int iClearBitsNum;
    int iStuffBitsNum;
    int iBitsCount;
} InputStream;

InputStream IRABS;
InputStream *pIRABS = &IRABS;

void OpenIRABS(InputStream *p, char *fname)
{
    p->f = fopen(fname, "rb");

    if (p->f == NULL) {
        printf("\nCan't open file %s", fname);
        exit(-1);
    }

    p->uClearBits     = 0xffffffff;
    p->iBytePosition    = 0;
    p->iBufBytesNum     = 0;
    p->iClearBitsNum    = 0;
    p->iStuffBitsNum    = 0;
    p->iBitsCount     = 0;
    p->uPre3Bytes     = 0;
}

void CloseIRABS(InputStream *p)
{
    fclose(p->f);
}

/*
*************************************************************************
* Function: Check start code's type
* Input:
* Output:
* Return:
* Attention: If the start code is video_sequence_start_code,user_data_start_code
or extension_start_code, the demulation mechanism is forsymded.
* Author: X ZHENG, 20080515
*************************************************************************
*/
void CheckType(int startcode)
{
    startcode = startcode & 0x000000ff;

    switch (startcode) {
    case 0xb0:
    case 0xb2:
    case 0xb5:
        hd->demulate_enable = 0;
        break;
    default:
        hd->demulate_enable = 1;
        break;
    }
}
// move iBytePosition to the next byte of start code prefix
//return
//    0 : OK
//   -1 : arrive at stream end and start code is not found
//   -2 : p->iBytePosition error
//-------------------------------------------------------------------------
int NextStartCode(InputStream *p)
{
    int i, m;
    unsigned char a, b; // a b 0 1 2 3 4 ... M-3 M-2 M-1
    m = 0; // cjw 20060323 for linux envi

    while (1) {
        if (p->iBytePosition >= p->iBufBytesNum - 2) {   //if all bytes in buffer has been searched
            m = p->iBufBytesNum - p->iBytePosition;

            if (m < 0) {
                return -2;  // p->iBytePosition error
            }

            if (m == 1) {
                b = p->buf[p->iBytePosition + 1];
            }

            if (m == 2) {
                b = p->buf[p->iBytePosition + 1];
                a = p->buf[p->iBytePosition];
            }

            p->iBufBytesNum = (int)fread(p->buf, 1, SVA_STREAM_BUF_SIZE, p->f);
            p->iBytePosition = 0;
        }

        if (p->iBufBytesNum + m < 3) {
            return -1;  //arrive at stream end and start code is not found
        }

        if (m == 1 && b == 0 && p->buf[0] == 0 && p->buf[1] == 1) {
            p->iBytePosition  = 2;
            p->iClearBitsNum  = 0;
            p->iStuffBitsNum  = 0;
            p->iBitsCount   += 24;
            p->uPre3Bytes   = 1;
            return 0;
        }

        if (m == 2 && b == 0 && a == 0 && p->buf[0] == 1) {
            p->iBytePosition  = 1;
            p->iClearBitsNum  = 0;
            p->iStuffBitsNum  = 0;
            p->iBitsCount   += 24;
            p->uPre3Bytes   = 1;
            return 0;
        }

        if (m == 2 && b == 0 && p->buf[0] == 0 && p->buf[1] == 1) {
            p->iBytePosition  = 2;
            p->iClearBitsNum  = 0;
            p->iStuffBitsNum  = 0;
            p->iBitsCount   += 24;
            p->uPre3Bytes   = 1;
            return 0;
        }

        for (i = p->iBytePosition; i < p->iBufBytesNum - 2; i++) {
            if (p->buf[i] == 0 && p->buf[i + 1] == 0 && p->buf[i + 2] == 1) {
                p->iBytePosition  = i + 3;
                p->iClearBitsNum  = 0;
                p->iStuffBitsNum  = 0;
                p->iBitsCount   += 24;
                p->uPre3Bytes   = 1;
                return 0;
            }

            p->iBitsCount += 8;
        }

        p->iBytePosition = i;
    }
}

/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:  0 : OK
-1 : arrive at stream end
-2 : meet another start code
* Attention:
*************************************************************************
*/
int ClearNextByte(InputStream *p)
{
    int i, k, j;
    unsigned char temp[3];
    i = p->iBytePosition;
    k = p->iBufBytesNum - i;

    if (k < 3) {
        for (j = 0; j < k; j++) {
            temp[j] = p->buf[i + j];
        }

        p->iBufBytesNum = (int)fread(p->buf + k, 1, SVA_STREAM_BUF_SIZE - k, p->f);

        if (p->iBufBytesNum == 0) {
            if (k > 0) {
                p->uPre3Bytes = ((p->uPre3Bytes << 8) | p->buf[i]) & 0x00ffffff;

                if (p->uPre3Bytes < 4 && hd->demulate_enable) {
                    p->uClearBits = (p->uClearBits << 6) | (p->buf[i] >> 2);
                    p->iClearBitsNum += 6;
                    StatBitsPtr->emulate_bits += 2; //rm52k_r2
                } else {
                    p->uClearBits = (p->uClearBits << 8) | p->buf[i];
                    p->iClearBitsNum += 8;
                }

                p->iBytePosition++;
                return 0;
            } else {
                return -1;//arrive at stream end
            }
        } else {
            for (j = 0; j < k; j++) {
                p->buf[j] = temp[j];
            }

            p->iBufBytesNum += k;
            i = p->iBytePosition = 0;
        }
    }

    if (p->buf[i] == 0 && p->buf[i + 1] == 0 && p->buf[i + 2] == 1) {
        return -2;  // meet another start code
    }

    p->uPre3Bytes = ((p->uPre3Bytes << 8) | p->buf[i]) & 0x00ffffff;

    if (p->uPre3Bytes < 4 && hd->demulate_enable) {
        p->uClearBits = (p->uClearBits << 6) | (p->buf[i] >> 2);
        p->iClearBitsNum += 6;
        StatBitsPtr->emulate_bits += 2; //rm52k_r2
    } else {
        p->uClearBits = (p->uClearBits << 8) | p->buf[i];
        p->iClearBitsNum += 8;
    }

    p->iBytePosition++;
    return 0;
}


/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:  0 : OK
-1 : arrive at stream end
-2 : meet another start code
* Attention:
*************************************************************************
*/
int read_n_bit(InputStream *p, int n, int *v)
{
    int r;
    unsigned int t;

    while (n > p->iClearBitsNum) {
        r = ClearNextByte(p);

        if (r) {
            if (r == -1) {
                if (p->iBufBytesNum - p->iBytePosition > 0) {
                    break;
                }
            }

            return r;
        }
    }

    t = p->uClearBits;
    r = 32 - p->iClearBitsNum;
    *v = (t << r) >> (32 - n);
    p->iClearBitsNum -= n;
    return 0;
}
//==================================================================================

void OpenBitstreamFile(char *fn)
{
    OpenIRABS(pIRABS, fn);
}

void CloseBitstreamFile()
{
    CloseIRABS(pIRABS);
}


/*
*************************************************************************
* Function:For supporting multiple sequences in a stream
* Input:
* Output:
* Return:
* Attention: Modified by X ZHENG, 20080721
* Author: Carmen, 2008/01/22
*************************************************************************
*/
int IsEndOfBitstream()
{
    int ret, seof, m;
    ret = feof(pIRABS->f);
    m = pIRABS->iBufBytesNum - pIRABS->iBytePosition;
    seof = ((ret) && (!pIRABS->iBufBytesNum)) || ((ret) && (m == 1));
    return ((!seof) ? (0) : (1));
}

/*
*************************************************************************
* Function:check slice start code
* Input:
* Output:
* Return:
* Attention: jlzheng  6.30
*************************************************************************
*/
int checkstartcode()   //check slice start code    jlzheng  6.30
{
    int temp_i, temp_val;

    //multiple slice
    DecodingEnvironmentPtr dep_dp;


    dep_dp = &img->currentSlice->partArr[0].de_AEC;
    currStream->frame_bitoffset = arideco_bits_read(dep_dp);    //multiple slice, ??

    //multiple slice

    if (currStream->bitstream_length * 8 - currStream->frame_bitoffset == 0) {
        return 1;
    }

    if (img->current_mb_nr == 0) {
        //--- added by Yulj 2004.07.15
        if (currStream->bitstream_length * 8 - currStream->frame_bitoffset <= 8
            && currStream->bitstream_length * 8 - currStream->frame_bitoffset > 0) {
            temp_i = currStream->bitstream_length * 8 - currStream->frame_bitoffset;
            assert(temp_i > 0);
            temp_val = get_uv(temp_i, "filling data") ;
        }
    }

    if (img->current_mb_nr == 0) {
        if (hd->StartCodePosition > 4 && hd->StartCodePosition < 0x000fffff) {
            return 1;
        } else {
            currStream->frame_bitoffset = hd->currentbitoffset;
            return 0;
        }
    }

    if (img->current_mb_nr != 0) {
        //--- added by Yulj 2004.07.15
        if (currStream->bitstream_length * 8 - currStream->frame_bitoffset <= 8
            && currStream->bitstream_length * 8 - currStream->frame_bitoffset > 0) {
            temp_i = currStream->bitstream_length * 8 - currStream->frame_bitoffset;
            assert(temp_i > 0);
            temp_val = get_uv(temp_i, "stuffing pattern") ;    //modified at rm52k

            if (temp_val == (1 << (temp_i - 1))) {
                return 1;  // last MB in current slice
            }
        }

        return 0;       // not last MB in current slice
        //---end
    }

    return 1;
}



//------------------------------------------------------------
// buf          : buffer
// startcodepos :
// length       :
int GetOneUnit(char *buf, int *startcodepos, int *length)
{
    int i, j, k;
    i = NextStartCode(pIRABS);

    if (i != 0) {
        if (i == -1) {
            printf("\narrive at stream end and start code is not found!");
        }

        if (i == -2) {
            printf("\np->iBytePosition error!");
        }

        exit(i);
    }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 1;
    *startcodepos = 3;
    i = read_n_bit(pIRABS, 8, &j);
    buf[3] = (char) j;
    CheckType(buf[3]);    //X ZHENG, 20080515, for demulation

    if (buf[3] == SEQUENCE_END_CODE) {
        *length = 4;
        return -1;
    }

    k = 4;

    while (1) {
        i = read_n_bit(pIRABS, 8, &j);

        if (i < 0) {
            break;
        }

        buf[k++] = (char) j;
    }

    if (pIRABS->iClearBitsNum > 0) {
        int shift;
        shift = 8 - pIRABS->iClearBitsNum;
        i = read_n_bit(pIRABS, pIRABS->iClearBitsNum, &j);

        if (j != 0) {   //qhg 20060327 for de-emulation.
            buf[k++] = (char)(j << shift);
        }

        StatBitsPtr->last_unit_bits += shift;  //rm52k_r2
    }

    *length = k;
    return k;
}






