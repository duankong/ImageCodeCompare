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

#include <memory.h>
#include "transform.h"

#define absm(A) ((A)<(0) ? (-(A)):(A))


#define LOT_MAX_WLT_TAP             2           ///< number of wavelet transform tap, (5-3)
static int iEBuff[ 64 + LOT_MAX_WLT_TAP * 2 ];

/////////////////////////////////////////////////////////////////////////////
/// variable definition
/////////////////////////////////////////////////////////////////////////////

static int iEBuff[ 64 + LOT_MAX_WLT_TAP * 2 ];

#if FREQUENCY_WEIGHTING_QUANTIZATION
// Sequence level default Matrix

int g_quantDefault4x4_slight[16] = {
    16,     16,     16,     17,
    16,     16,     17,     18,
    16,     17,     19,     20,
    18,     19,     21,     24
};

int g_quantDefault8x8_slight[64] = {
    16,     16,     16,     16,     17,     17,     18,     19,
    16,     16,     16,     17,     18,     19,     21,     23,
    16,     16,     17,     18,     19,     20,     22,     25,
    16,     17,     18,     20,     21,     23,     25,     28,
    17,     18,     20,     21,     23,     26,     28,     32,
    19,     20,     21,     23,     26,     29,     33,     38,
    24,     25,     26,     29,     31,     35,     41,     47,
    26,     27,     29,     32,     38,     43,     48,     54
};

int g_quantDefault4x4_middle[16] = {
    16,     16,     16,     18,
    16,     17,     19,     22,
    17,     19,     23,     27,
    21,     24,     29,     36
};

int g_quantDefault8x8_middle[64] = {
    16,     16,     16,     17,     18,     19,     22,     24,
    16,     16,     17,     18,     21,     24,     28,     35,
    16,     17,     18,     22,     24,     27,     32,     39,
    17,     19,     22,     26,     29,     33,     38,     46,
    19,     22,     26,     29,     35,     41,     48,     57,
    24,     27,     30,     35,     41,     50,     60,     72,
    36,     39,     43,     49,     56,     66,     81,     96,
    43,     45,     49,     57,     72,     86,     101,    115
};

int g_quantDefault4x4_strong[16] = {
    16,     16,     17,     22,
    17,     18,     24,     30,
    19,     24,     32,     42,
    28,     36,     48,     64
};

int g_quantDefault8x8_strong[64] = {
    16,     16,     16,     18,     20,     24,     30,     36,
    16,     17,     18,     20,     27,     35,     46,     62,
    16,     18,     22,     31,     36,     43,     55,     72,
    18,     24,     31,     41,     48,     57,     70,     89,
    24,     31,     41,     48,     62,     76,     93,     116,
    36,     42,     50,     62,     76,     97,     122,    152,
    64,     72,     80,     96,     112,    136,    174,    210,
    81,     85,     96,     116,    152,    186,    220,    255
};

#endif

const int trans_core_4[4][4] = {
    {  32,    32,     32,     32 },
    {  42,    17,    -17,    -42 },
    {  32,   -32,    -32,     32 },
    {  17,   -42,     42,    -17 }
};
const int trans_core_8[8][8] = {
    {  32,    32,     32,     32,     32,     32,     32,     32    },
    {  44,    38,     25,      9,     -9,    -25,    -38,    -44    },
    {  42,    17,    -17,    -42,    -42,    -17,     17,     42    },
    {  38,    -9,    -44,    -25,     25,     44,      9,    -38    },
    {  32,   -32,    -32,     32,     32,    -32,    -32,     32    },
    {  25,   -44,      9,     38,    -38,     -9,     44,    -25    },
    {  17,   -42,     42,    -17,    -17,     42,    -42,     17    },
    {   9,   -25,     38,    -44,     44,    -38,     25,     -9    }
};
const int trans_core_16[16][16] = {
    {  32,    32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32    },
    {  45,    43,     40,     35,     29,     21,     13,      4,     -4,    -13,    -21,    -29,    -35,    -40,    -43,    -45    },
    {  44,    38,     25,      9,     -9,    -25,    -38,    -44,    -44,    -38,    -25,     -9,      9,     25,     38,     44    },
    {  43,    29,      4,    -21,    -40,    -45,    -35,    -13,     13,     35,     45,     40,     21,     -4,    -29,    -43    },
    {  42,    17,    -17,    -42,    -42,    -17,     17,     42,     42,     17,    -17,    -42,    -42,    -17,     17,     42    },
    {  40,     4,    -35,    -43,    -13,     29,     45,     21,    -21,    -45,    -29,     13,     43,     35,     -4,    -40    },
    {  38,    -9,    -44,    -25,     25,     44,      9,    -38,    -38,      9,     44,     25,    -25,    -44,     -9,     38    },
    {  35,   -21,    -43,      4,     45,     13,    -40,    -29,     29,     40,    -13,    -45,     -4,     43,     21,    -35    },
    {  32,   -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32    },
    {  29,   -40,    -13,     45,     -4,    -43,     21,     35,    -35,    -21,     43,      4,    -45,     13,     40,    -29    },
    {  25,   -44,      9,     38,    -38,     -9,     44,    -25,    -25,     44,     -9,    -38,     38,      9,    -44,     25    },
    {  21,   -45,     29,     13,    -43,     35,      4,    -40,     40,     -4,    -35,     43,    -13,    -29,     45,    -21    },
    {  17,   -42,     42,    -17,    -17,     42,    -42,     17,     17,    -42,     42,    -17,    -17,     42,    -42,     17    },
    {  13,   -35,     45,    -40,     21,      4,    -29,     43,    -43,     29,     -4,    -21,     40,    -45,     35,    -13    },
    {   9,   -25,     38,    -44,     44,    -38,     25,     -9,     -9,     25,    -38,     44,    -44,     38,    -25,      9    },
    {   4,   -13,     21,    -29,     35,    -40,     43,    -45,     45,    -43,     40,    -35,     29,    -21,     13,     -4    }
};
const int trans_core_32[32][32] = {
    {  32,    32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32,     32    },
    {  45,    45,     44,     43,     41,     39,     36,     34,     30,     27,     23,     19,     15,     11,      7,      2,     -2,     -7,    -11,    -15,    -19,    -23,    -27,    -30,    -34,    -36,    -39,    -41,    -43,    -44,    -45,    -45    },
    {  45,    43,     40,     35,     29,     21,     13,      4,     -4,    -13,    -21,    -29,    -35,    -40,    -43,    -45,    -45,    -43,    -40,    -35,    -29,    -21,    -13,     -4,      4,     13,     21,     29,     35,     40,     43,     45    },
    {  45,    41,     34,     23,     11,     -2,    -15,    -27,    -36,    -43,    -45,    -44,    -39,    -30,    -19,     -7,      7,     19,     30,     39,     44,     45,     43,     36,     27,     15,      2,    -11,    -23,    -34,    -41,    -45    },
    {  44,    38,     25,      9,     -9,    -25,    -38,    -44,    -44,    -38,    -25,     -9,      9,     25,     38,     44,     44,     38,     25,      9,     -9,    -25,    -38,    -44,    -44,    -38,    -25,     -9,      9,     25,     38,     44    },
    {  44,    34,     15,     -7,    -27,    -41,    -45,    -39,    -23,     -2,     19,     36,     45,     43,     30,     11,    -11,    -30,    -43,    -45,    -36,    -19,      2,     23,     39,     45,     41,     27,      7,    -15,    -34,    -44    },
    {  43,    29,      4,    -21,    -40,    -45,    -35,    -13,     13,     35,     45,     40,     21,     -4,    -29,    -43,    -43,    -29,     -4,     21,     40,     45,     35,     13,    -13,    -35,    -45,    -40,    -21,      4,     29,     43    },
    {  43,    23,     -7,    -34,    -45,    -36,    -11,     19,     41,     44,     27,     -2,    -30,    -45,    -39,    -15,     15,     39,     45,     30,      2,    -27,    -44,    -41,    -19,     11,     36,     45,     34,      7,    -23,    -43    },
    {  42,    17,    -17,    -42,    -42,    -17,     17,     42,     42,     17,    -17,    -42,    -42,    -17,     17,     42,     42,     17,    -17,    -42,    -42,    -17,     17,     42,     42,     17,    -17,    -42,    -42,    -17,     17,     42    },
    {  41,    11,    -27,    -45,    -30,      7,     39,     43,     15,    -23,    -45,    -34,      2,     36,     44,     19,    -19,    -44,    -36,     -2,     34,     45,     23,    -15,    -43,    -39,     -7,     30,     45,     27,    -11,    -41    },
    {  40,     4,    -35,    -43,    -13,     29,     45,     21,    -21,    -45,    -29,     13,     43,     35,     -4,    -40,    -40,     -4,     35,     43,     13,    -29,    -45,    -21,     21,     45,     29,    -13,    -43,    -35,      4,     40    },
    {  39,    -2,    -41,    -36,      7,     43,     34,    -11,    -44,    -30,     15,     45,     27,    -19,    -45,    -23,     23,     45,     19,    -27,    -45,    -15,     30,     44,     11,    -34,    -43,     -7,     36,     41,      2,    -39    },
    {  38,    -9,    -44,    -25,     25,     44,      9,    -38,    -38,      9,     44,     25,    -25,    -44,     -9,     38,     38,     -9,    -44,    -25,     25,     44,      9,    -38,    -38,      9,     44,     25,    -25,    -44,     -9,     38    },
    {  36,   -15,    -45,    -11,     39,     34,    -19,    -45,     -7,     41,     30,    -23,    -44,     -2,     43,     27,    -27,    -43,      2,     44,     23,    -30,    -41,      7,     45,     19,    -34,    -39,     11,     45,     15,    -36    },
    {  35,   -21,    -43,      4,     45,     13,    -40,    -29,     29,     40,    -13,    -45,     -4,     43,     21,    -35,    -35,     21,     43,     -4,    -45,    -13,     40,     29,    -29,    -40,     13,     45,      4,    -43,    -21,     35    },
    {  34,   -27,    -39,     19,     43,    -11,    -45,      2,     45,      7,    -44,    -15,     41,     23,    -36,    -30,     30,     36,    -23,    -41,     15,     44,     -7,    -45,     -2,     45,     11,    -43,    -19,     39,     27,    -34    },
    {  32,   -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32,     32,    -32,    -32,     32    },
    {  30,   -36,    -23,     41,     15,    -44,     -7,     45,     -2,    -45,     11,     43,    -19,    -39,     27,     34,    -34,    -27,     39,     19,    -43,    -11,     45,      2,    -45,      7,     44,    -15,    -41,     23,     36,    -30    },
    {  29,   -40,    -13,     45,     -4,    -43,     21,     35,    -35,    -21,     43,      4,    -45,     13,     40,    -29,    -29,     40,     13,    -45,      4,     43,    -21,    -35,     35,     21,    -43,     -4,     45,    -13,    -40,     29    },
    {  27,   -43,     -2,     44,    -23,    -30,     41,      7,    -45,     19,     34,    -39,    -11,     45,    -15,    -36,     36,     15,    -45,     11,     39,    -34,    -19,     45,     -7,    -41,     30,     23,    -44,      2,     43,    -27    },
    {  25,   -44,      9,     38,    -38,     -9,     44,    -25,    -25,     44,     -9,    -38,     38,      9,    -44,     25,     25,    -44,      9,     38,    -38,     -9,     44,    -25,    -25,     44,     -9,    -38,     38,      9,    -44,     25    },
    {  23,   -45,     19,     27,    -45,     15,     30,    -44,     11,     34,    -43,      7,     36,    -41,      2,     39,    -39,     -2,     41,    -36,     -7,     43,    -34,    -11,     44,    -30,    -15,     45,    -27,    -19,     45,    -23    },
    {  21,   -45,     29,     13,    -43,     35,      4,    -40,     40,     -4,    -35,     43,    -13,    -29,     45,    -21,    -21,     45,    -29,    -13,     43,    -35,     -4,     40,    -40,      4,     35,    -43,     13,     29,    -45,     21    },
    {  19,   -44,     36,     -2,    -34,     45,    -23,    -15,     43,    -39,      7,     30,    -45,     27,     11,    -41,     41,    -11,    -27,     45,    -30,     -7,     39,    -43,     15,     23,    -45,     34,      2,    -36,     44,    -19    },
    {  17,   -42,     42,    -17,    -17,     42,    -42,     17,     17,    -42,     42,    -17,    -17,     42,    -42,     17,     17,    -42,     42,    -17,    -17,     42,    -42,     17,     17,    -42,     42,    -17,    -17,     42,    -42,     17    },
    {  15,   -39,     45,    -30,      2,     27,    -44,     41,    -19,    -11,     36,    -45,     34,     -7,    -23,     43,    -43,     23,      7,    -34,     45,    -36,     11,     19,    -41,     44,    -27,     -2,     30,    -45,     39,    -15    },
    {  13,   -35,     45,    -40,     21,      4,    -29,     43,    -43,     29,     -4,    -21,     40,    -45,     35,    -13,    -13,     35,    -45,     40,    -21,     -4,     29,    -43,     43,    -29,      4,     21,    -40,     45,    -35,     13    },
    {  11,   -30,     43,    -45,     36,    -19,     -2,     23,    -39,     45,    -41,     27,     -7,    -15,     34,    -44,     44,    -34,     15,      7,    -27,     41,    -45,     39,    -23,      2,     19,    -36,     45,    -43,     30,    -11    },
    {   9,   -25,     38,    -44,     44,    -38,     25,     -9,     -9,     25,    -38,     44,    -44,     38,    -25,      9,      9,    -25,     38,    -44,     44,    -38,     25,     -9,     -9,     25,    -38,     44,    -44,     38,    -25,      9    },
    {   7,   -19,     30,    -39,     44,    -45,     43,    -36,     27,    -15,      2,     11,    -23,     34,    -41,     45,    -45,     41,    -34,     23,    -11,     -2,     15,    -27,     36,    -43,     45,    -44,     39,    -30,     19,     -7    },
    {   4,   -13,     21,    -29,     35,    -40,     43,    -45,     45,    -43,     40,    -35,     29,    -21,     13,     -4,     -4,     13,    -21,     29,    -35,     40,    -43,     45,    -45,     43,    -40,     35,    -29,     21,    -13,      4    },
    {   2,    -7,     11,    -15,     19,    -23,     27,    -30,     34,    -36,     39,    -41,     43,    -44,     45,    -45,     45,    -45,     44,    -43,     41,    -39,     36,    -34,     30,    -27,     23,    -19,     15,    -11,      7,     -2    }
};


unsigned short Q_TAB[80] = {
    32768, 29775, 27554, 25268, 23170, 21247, 19369, 17770,
    16302, 15024, 13777, 12634, 11626, 10624, 9742, 8958,
    8192, 7512, 6889, 6305, 5793, 5303, 4878, 4467,
    4091, 3756, 3444, 3161, 2894, 2654, 2435, 2235,
    2048, 1878, 1722, 1579, 1449, 1329, 1218, 1117,
    1024, 939, 861, 790, 724, 664, 609, 558,
    512, 470, 430, 395, 362, 332, 304, 279,
    256, 235, 215, 197, 181, 166, 152, 140,
    128, 116, 108, 99, 91, 83, 76, 69,
    64, 59, 54, 49, 45, 41, 38, 35

};

unsigned short IQ_TAB[80] = {

    32768, 36061, 38968, 42495, 46341, 50535, 55437, 60424,
    32932, 35734, 38968, 42495, 46177, 50535, 55109, 59933,
    65535, 35734, 38968, 42577, 46341, 50617, 55027, 60097,
    32809, 35734, 38968, 42454, 46382, 50576, 55109, 60056,
    65535, 35734, 38968, 42495, 46320, 50515, 55109, 60076,
    65535, 35744, 38968, 42495, 46341, 50535, 55099, 60087,
    65535, 35734, 38973, 42500, 46341, 50535, 55109, 60097,
    32771, 35734, 38965, 42497, 46341, 50535, 55109, 60099,
    32768, 36061, 38968, 42495, 46341, 50535, 55437, 60424,
    32932, 35734, 38968, 42495, 46177, 50535, 55109, 59933

};

short IQ_SHIFT[80] = {
    15, 15, 15, 15, 15, 15, 15, 15,
    14, 14, 14, 14, 14, 14, 14, 14,
    14, 13, 13, 13, 13, 13, 13, 13,
    12, 12, 12, 12, 12, 12, 12, 12,
    12, 11, 11, 11, 11, 11, 11, 11,
    11, 10, 10, 10, 10, 10, 10, 10,
    10, 9, 9, 9, 9, 9, 9, 9,
    8, 8, 8, 8, 8, 8, 8, 8,
    7, 7, 7, 7, 7, 7, 7, 7,
    6, 6, 6, 6, 6, 6, 6, 6

};

const short g_as_C_TRANS[4][4] = {

    {    34,    58,    72,     81,},
    {    77,    69,    -7,    -75,},
    {    79,   -33,   -75,     58,},
    {    55,   -84,    73,    -28,}

};

const short g_as_SEC_TRANS[SEC_TR_SIZE][SEC_TR_SIZE] = {
    {   123,   -35,    -8,    -3,},
    {   -32,  -120,    30,    10,},
    {    14,    25,   123,   -22,},
    {     8,    13,    19,   126,},
};

Video_Com_data  hhc;
Video_Com_data *hc = &hhc;

/////////////////////////////////////////////////////////////////////////////
/// local function declaration
/////////////////////////////////////////////////////////////////////////////

static void array_shift(int **src, int shift, int iSizeY, int iSizeX);




/////////////////////////////////////////////////////////////////////////////
/// function definition
/////////////////////////////////////////////////////////////////////////////
void partialButterfly4(int **src, int **dst, int iNumRows)
{
    int j;
    int E[4], O[4];
    for (j = 0; j < iNumRows; j++) {
        E[0] = trans_core_4[0][0] * src[j][0] + trans_core_4[2][0] * src[j][2];
        E[1] = trans_core_4[2][0] * src[j][0] - trans_core_4[0][0] * src[j][2];
        E[2] = trans_core_4[1][0] * src[j][0] - trans_core_4[3][0] * src[j][2];
        E[3] = trans_core_4[3][0] * src[j][0] + trans_core_4[1][0] * src[j][2];
        O[0] = trans_core_4[1][0] * src[j][1] + trans_core_4[3][0] * src[j][3];
        O[1] = trans_core_4[3][0] * src[j][1] - trans_core_4[1][0] * src[j][3];
        O[2] = trans_core_4[0][0] * src[j][1] - trans_core_4[2][0] * src[j][3];
        O[3] = trans_core_4[2][0] * src[j][1] + trans_core_4[0][0] * src[j][3];

        dst[0][j] = E[0] + O[3] ;
        dst[2][j] = E[1] - O[2] ;
        dst[1][j] = E[2] + O[1] ;
        dst[3][j] = E[3] - O[0] ;
    }
}
void partialButterflyInverse4(int **src, int **dst, int iNumRows)
{
    int j;
    int E[2], O[2];

    for (j = 0; j < iNumRows; j++) {
        E[0] = trans_core_4[0][0] * src[0][j] + trans_core_4[2][0] * src[2][j];
        E[1] = trans_core_4[2][0] * src[0][j] - trans_core_4[0][0] * src[2][j];
        O[0] = trans_core_4[1][0] * src[1][j] + trans_core_4[3][0] * src[3][j];
        O[1] = trans_core_4[3][0] * src[1][j] - trans_core_4[1][0] * src[3][j];

        dst[j][0] = E[0] + O[0];
        dst[j][2] = E[1] - O[1];
        dst[j][1] = E[1] + O[1];
        dst[j][3] = E[0] - O[0];
    }
}

void partialButterfly8(int **src, int **dst, int iNumRows)
{
    int j, k;
    int E[4], O[4];
    int EE[2], EO[2];


    for (j = 0; j < iNumRows; j++) {
        /* E and O*/
        for (k = 0; k < 4; k++) {
            E[k] = src[j][k] + src[j][7 - k];
            O[k] = src[j][k] - src[j][7 - k];
        }
        /* EE and EO */
        EE[0] = E[0] + E[3];
        EO[0] = E[0] - E[3];
        EE[1] = E[1] + E[2];
        EO[1] = E[1] - E[2];

        dst[0][j] = trans_core_8[0][0] * EE[0] + trans_core_8[0][1] * EE[1] ;
        dst[4][j] = trans_core_8[4][0] * EE[0] + trans_core_8[4][1] * EE[1] ;
        dst[2][j] = trans_core_8[2][0] * EO[0] + trans_core_8[2][1] * EO[1] ;
        dst[6][j] = trans_core_8[6][0] * EO[0] + trans_core_8[6][1] * EO[1] ;

        dst[1][j] = trans_core_8[1][0] * O[0] + trans_core_8[1][1] * O[1] + trans_core_8[1][2] * O[2] + trans_core_8[1][3] *
                    O[3] ;
        dst[3][j] = trans_core_8[3][0] * O[0] + trans_core_8[3][1] * O[1] + trans_core_8[3][2] * O[2] + trans_core_8[3][3] *
                    O[3] ;
        dst[5][j] = trans_core_8[5][0] * O[0] + trans_core_8[5][1] * O[1] + trans_core_8[5][2] * O[2] + trans_core_8[5][3] *
                    O[3] ;
        dst[7][j] = trans_core_8[7][0] * O[0] + trans_core_8[7][1] * O[1] + trans_core_8[7][2] * O[2] + trans_core_8[7][3] *
                    O[3] ;

    }
}
void partialButterflyInverse8(int **src, int **dst, int iNumRows)
{
    int j, k;
    int E[4], O[4];
    int EE[2], EO[2];

    for (j = 0; j < iNumRows; j++) {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        for (k = 0; k < 4; k++) {
            O[k] = trans_core_8[ 1][k] * src[1][j] + trans_core_8[ 3][k] * src[3][j] + trans_core_8[ 5][k] * src[5][j] +
                   trans_core_8[ 7][k] * src[7][j];
        }

        EO[0] = trans_core_8[2][0] * src[ 2 ][j] + trans_core_8[6][0] * src[ 6 ][j];
        EO[1] = trans_core_8[2][1] * src[ 2 ][j] + trans_core_8[6][1] * src[ 6 ][j];
        EE[0] = trans_core_8[0][0] * src[ 0 ][j] + trans_core_8[4][0] * src[ 4 ][j];
        EE[1] = trans_core_8[0][1] * src[ 0 ][j] + trans_core_8[4][1] * src[ 4 ][j];

        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        E[0] = EE[0] + EO[0];
        E[3] = EE[0] - EO[0];
        E[1] = EE[1] + EO[1];
        E[2] = EE[1] - EO[1];
        for (k = 0; k < 4; k++) {
            dst[j][ k   ] = E[k] + O[k];
            dst[j][ k + 4 ] = E[3 - k] - O[3 - k];
        }
    }
}

void partialButterfly16(int **src, int **dst, int iNumRows)
{
    int j, k;
    int E[8], O[8];
    int EE[4], EO[4];
    int EEE[2], EEO[2];

    for (j = 0; j < iNumRows; j++) {
        /* E and O*/
        for (k = 0; k < 8; k++) {
            E[k] = src[j][k] + src[j][15 - k];
            O[k] = src[j][k] - src[j][15 - k];
        }
        /* EE and EO */
        for (k = 0; k < 4; k++) {
            EE[k] = E[k] + E[7 - k];
            EO[k] = E[k] - E[7 - k];
        }
        /* EEE and EEO */
        EEE[0] = EE[0] + EE[3];
        EEO[0] = EE[0] - EE[3];
        EEE[1] = EE[1] + EE[2];
        EEO[1] = EE[1] - EE[2];

        dst[ 0 ][j] = trans_core_16[ 0][0] * EEE[0] + trans_core_16[ 0][1] * EEE[1] ;
        dst[ 8 ][j] = trans_core_16[ 8][0] * EEE[0] + trans_core_16[ 8][1] * EEE[1] ;
        dst[ 4 ][j] = trans_core_16[ 4][0] * EEO[0] + trans_core_16[ 4][1] * EEO[1] ;
        dst[ 12][j] = trans_core_16[12][0] * EEO[0] + trans_core_16[12][1] * EEO[1] ;

        for (k = 2; k < 16; k += 4) {
            dst[ k ][j] = trans_core_16[k][0] * EO[0] + trans_core_16[k][1] * EO[1] + trans_core_16[k][2] * EO[2] +
                          trans_core_16[k][3] * EO[3] ;
        }

        for (k = 1; k < 16; k += 2) {
            dst[ k ][j] = trans_core_16[k][0] * O[0] + trans_core_16[k][1] * O[1] + trans_core_16[k][2] * O[2] + trans_core_16[k][3]
                          * O[3] +
                          trans_core_16[k][4] * O[4] + trans_core_16[k][5] * O[5] + trans_core_16[k][6] * O[6] + trans_core_16[k][7] * O[7] ;
        }

    }
}
void partialButterflyInverse16(int **src, int **dst, int iNumRows)
{
    int j, k;
    int E[8], O[8];
    int EE[4], EO[4];
    int EEE[2], EEO[2];

    for (j = 0; j < iNumRows; j++) {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        for (k = 0; k < 8; k++) {
            O[k] = trans_core_16[ 1][k] * src[ 1 ][j] + trans_core_16[ 3][k] * src[ 3 ][j] + trans_core_16[ 5][k] * src[ 5 ][j] +
                   trans_core_16[ 7][k] * src[ 7 ][j] +
                   trans_core_16[ 9][k] * src[ 9 ][j] + trans_core_16[11][k] * src[ 11 ][j] + trans_core_16[13][k] * src[ 13 ][j] +
                   trans_core_16[15][k] * src[15 ][j];
        }
        for (k = 0; k < 4; k++) {
            EO[k] = trans_core_16[ 2][k] * src[ 2 ][j] + trans_core_16[ 6][k] * src[ 6 ][j] + trans_core_16[10][k] * src[10 ][j] +
                    trans_core_16[14][k] * src[14 ][j];
        }
        EEO[0] = trans_core_16[4][0] * src[ 4 ][j] + trans_core_16[12][0] * src[ 12 ][j];
        EEE[0] = trans_core_16[0][0] * src[ 0 ][j] + trans_core_16[ 8][0] * src[ 8  ][j];
        EEO[1] = trans_core_16[4][1] * src[ 4 ][j] + trans_core_16[12][1] * src[ 12 ][j];
        EEE[1] = trans_core_16[0][1] * src[ 0 ][j] + trans_core_16[ 8][1] * src[ 8  ][j];

        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        for (k = 0; k < 2; k++) {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 2] = EEE[1 - k] - EEO[1 - k];
        }
        for (k = 0; k < 4; k++) {
            E[k] = EE[k] + EO[k];
            E[k + 4] = EE[3 - k] - EO[3 - k];
        }
        for (k = 0; k < 8; k++) {
            dst[j][k]   = E[k] + O[k] ;
            dst[j][k + 8] = E[7 - k] - O[7 - k] ;
        }
    }
}

void partialButterfly32(int **src, int **dst, int iNumRows)
{
    int j, k;
    int E[16], O[16];
    int EE[8], EO[8];
    int EEE[4], EEO[4];
    int EEEE[2], EEEO[2];

    for (j = 0; j < iNumRows; j++) {
        /* E and O*/
        for (k = 0; k < 16; k++) {
            E[k] = src[j][k] + src[j][31 - k];
            O[k] = src[j][k] - src[j][31 - k];
        }
        /* EE and EO */
        for (k = 0; k < 8; k++) {
            EE[k] = E[k] + E[15 - k];
            EO[k] = E[k] - E[15 - k];
        }
        /* EEE and EEO */
        for (k = 0; k < 4; k++) {
            EEE[k] = EE[k] + EE[7 - k];
            EEO[k] = EE[k] - EE[7 - k];
        }
        /* EEEE and EEEO */
        EEEE[0] = EEE[0] + EEE[3];
        EEEO[0] = EEE[0] - EEE[3];
        EEEE[1] = EEE[1] + EEE[2];
        EEEO[1] = EEE[1] - EEE[2];

        dst[ 0  ][j] = trans_core_32[ 0][0] * EEEE[0] + trans_core_32[ 0][1] * EEEE[1] ;
        dst[ 16 ][j] = trans_core_32[16][0] * EEEE[0] + trans_core_32[16][1] * EEEE[1] ;
        dst[ 8  ][j] = trans_core_32[ 8][0] * EEEO[0] + trans_core_32[ 8][1] * EEEO[1] ;
        dst[ 24 ][j] = trans_core_32[24][0] * EEEO[0] + trans_core_32[24][1] * EEEO[1] ;
        for (k = 4; k < 32; k += 8) {
            dst[ k ][j] = trans_core_32[k][0] * EEO[0] + trans_core_32[k][1] * EEO[1] + trans_core_32[k][2] * EEO[2] +
                          trans_core_32[k][3] * EEO[3] ;
        }
        for (k = 2; k < 32; k += 4) {
            dst[ k ][j] = trans_core_32[k][0] * EO[0] + trans_core_32[k][1] * EO[1] + trans_core_32[k][2] * EO[2] +
                          trans_core_32[k][3] * EO[3] +
                          trans_core_32[k][4] * EO[4] + trans_core_32[k][5] * EO[5] + trans_core_32[k][6] * EO[6] + trans_core_32[k][7] * EO[7] ;
        }
        for (k = 1; k < 32; k += 2) {
            dst[ k ][j] = trans_core_32[k][ 0] * O[ 0] + trans_core_32[k][ 1] * O[ 1] + trans_core_32[k][ 2] * O[ 2] +
                          trans_core_32[k][ 3] * O[ 3] +
                          trans_core_32[k][ 4] * O[ 4] + trans_core_32[k][ 5] * O[ 5] + trans_core_32[k][ 6] * O[ 6] + trans_core_32[k][ 7] *
                          O[ 7] +
                          trans_core_32[k][ 8] * O[ 8] + trans_core_32[k][ 9] * O[ 9] + trans_core_32[k][10] * O[10] + trans_core_32[k][11] *
                          O[11] +
                          trans_core_32[k][12] * O[12] + trans_core_32[k][13] * O[13] + trans_core_32[k][14] * O[14] + trans_core_32[k][15] *
                          O[15] ;
        }
    }
}
void partialButterflyInverse32(int **src, int **dst, int iNumRows)
{
    int j, k;
    int E[16], O[16];
    int EE[8], EO[8];
    int EEE[4], EEO[4];
    int EEEE[2], EEEO[2];

    for (j = 0; j < iNumRows; j++) {
        /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
        for (k = 0; k < 16; k++) {
            O[k] = trans_core_32[ 1][k] * src[ 1 ][j] + trans_core_32[ 3][k] * src[ 3  ][j] + trans_core_32[ 5][k] * src[ 5  ][j] +
                   trans_core_32[ 7][k] * src[ 7 ][j] +
                   trans_core_32[ 9][k] * src[ 9  ][j] + trans_core_32[11][k] * src[ 11 ][j] + trans_core_32[13][k] * src[ 13 ][j] +
                   trans_core_32[15][k] * src[ 15 ][j] +
                   trans_core_32[17][k] * src[ 17 ][j] + trans_core_32[19][k] * src[ 19 ][j] + trans_core_32[21][k] * src[ 21 ][j] +
                   trans_core_32[23][k] * src[ 23 ][j] +
                   trans_core_32[25][k] * src[ 25 ][j] + trans_core_32[27][k] * src[ 27 ][j] + trans_core_32[29][k] * src[ 29 ][j] +
                   trans_core_32[31][k] * src[ 31 ][j];
        }
        for (k = 0; k < 8; k++) {
            EO[k] = trans_core_32[ 2][k] * src[ 2 ][j] + trans_core_32[ 6][k] * src[ 6 ][j] + trans_core_32[10][k] * src[ 10 ][j] +
                    trans_core_32[14][k] * src[ 14 ][j] +
                    trans_core_32[18][k] * src[ 18 ][j] + trans_core_32[22][k] * src[ 22 ][j] + trans_core_32[26][k] * src[ 26 ][j] +
                    trans_core_32[30][k] * src[ 30 ][j];
        }
        for (k = 0; k < 4; k++) {
            EEO[k] = trans_core_32[4][k] * src[ 4 ][j] + trans_core_32[12][k] * src[ 12 ][j] + trans_core_32[20][k] * src[ 20 ][j] +
                     trans_core_32[28][k] * src[ 28 ][j];
        }
        EEEO[0] = trans_core_32[8][0] * src[ 8 ][j] + trans_core_32[24][0] * src[ 24 ][j];
        EEEO[1] = trans_core_32[8][1] * src[ 8 ][j] + trans_core_32[24][1] * src[ 24 ][j];
        EEEE[0] = trans_core_32[0][0] * src[ 0 ][j] + trans_core_32[16][0] * src[ 16 ][j];
        EEEE[1] = trans_core_32[0][1] * src[ 0 ][j] + trans_core_32[16][1] * src[ 16 ][j];

        /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
        EEE[0] = EEEE[0] + EEEO[0];
        EEE[3] = EEEE[0] - EEEO[0];
        EEE[1] = EEEE[1] + EEEO[1];
        EEE[2] = EEEE[1] - EEEO[1];
        for (k = 0; k < 4; k++) {
            EE[k] = EEE[k] + EEO[k];
            EE[k + 4] = EEE[3 - k] - EEO[3 - k];
        }
        for (k = 0; k < 8; k++) {
            E[k] = EE[k] + EO[k];
            E[k + 8] = EE[7 - k] - EO[7 - k];
        }
        for (k = 0; k < 16; k++) {
            dst[j][k]    = E[k] + O[k] ;
            dst[j][k + 16] = E[15 - k] - O[15 - k] ;
        }
    }
}


void  wavelet64(int **curr_blk)
{
    int *pExt = &iEBuff[ LOT_MAX_WLT_TAP ];
    int  i, n, x, y;
    int  iN, iN2;
    int iLevel = 1, iSize = 64;

    int ySize, y2, x2;
    //int *pBuff = malloc( iSize * iSize * sizeof( int ) );
    int pBuff[64 * 64];
    for (i = 0; i < iSize; i++) {
        memcpy(&pBuff[i * iSize], &curr_blk[i][0], iSize * sizeof(int));
    }
    iN = iSize;
    iN2 = iN >> 1;
    for (i = 0; i < iLevel; i++) {
        // step #1: horizontal transform
        for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
            // copy
            memcpy(pExt, &pBuff[ ySize ], sizeof(int)*iN);

            // reflection
            pExt[ -1     ] = pExt[ +1   ];
            pExt[ -2     ] = pExt[ +2   ];
            pExt[ iN   ] = pExt[ iN - 2 ];
            pExt[ iN + 1 ] = pExt[ iN - 3 ];

            // filtering (H)
            for (n = -1; n < iN; n += 2) {
                pExt[ n ] -= (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // filtering (L)
            for (n = 0; n < iN; n += 2) {
                pExt[ n ] += (pExt[ n - 1 ] + pExt[ n + 1 ] + 2) >> 2;
            }

            // copy
            for (x = 0; x < iN2; x++) {
                x2 = x << 1;
                pBuff[ x + ySize ] = pExt[ x2 ];
            }
        }

        // step #2: vertical transform
        for (x = 0; x < iN; x++) {
            // copy
            for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
                pExt[ y ] = pBuff[ x + ySize ];
            }

            // reflection
            pExt[ -1     ] = pExt[ +1   ];
            pExt[ -2     ] = pExt[ +2   ];
            pExt[ iN   ] = pExt[ iN - 2 ];
            pExt[ iN + 1 ] = pExt[ iN - 3 ];

            // filtering (H)
            for (n = -1; n < iN; n += 2) {
                pExt[ n ] -= (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // filtering (L)

            for (n = 0; n < iN; n += 2) {
                pExt[ n ]  = (pExt[ n ] << 1) + ((pExt[ n - 1 ] + pExt[ n + 1 ] + 1) >> 1);
            }

            // copy
            for (y = 0, ySize = 0; y < iN2; y++, ySize += iSize) {
                y2 = y << 1;
                pBuff[ x + ySize ] = pExt[ y2 ];
            }
        }

        // shift size
        iN >>= 1;
        iN2 = iN >> 1;

    }
    for (i = 0; i < 32; i++) {
        memcpy(&curr_blk[i][0], &pBuff[i * iSize], 32 * sizeof(int));
    }
    //free( pBuff );
}


void  wavelet_NSQT(int **curr_blk, int is_Hor)
{

    int *pExt = &iEBuff[ LOT_MAX_WLT_TAP ];
    int  i, n, x, y;
    int  iN, iN2;
    int iLevel = 1, iSize;

    int ySize, y2, x2;
    //int *pBuff = malloc( 64 * 64 * sizeof( int ) );
    int pBuff[64 * 64];

    if (is_Hor) {
        iSize = 64;
        iN = 16;
    } else {
        iSize = 16;
        iN = 64;
    }

    for (i = 0; i < iN; i++) {
        memcpy(&pBuff[i * iSize], &curr_blk[i][0], iSize * sizeof(int));
    }


    for (i = 0; i < iLevel; i++) {
        // step #1: horizontal transform
        iN2 = iSize >> 1;
        for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
            // copy
            memcpy(pExt, &pBuff[ ySize ], sizeof(int)*iSize);

            // reflection
            pExt[ -1     ] = pExt[ +1   ];
            pExt[ -2     ] = pExt[ +2   ];
            pExt[ iSize   ] = pExt[ iSize - 2 ];
            pExt[ iSize + 1 ] = pExt[ iSize - 3 ];

            // filtering (H)
            for (n = -1; n < iSize; n += 2) {
                pExt[ n ] -= (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // filtering (L)
            for (n = 0; n < iSize; n += 2) {
                pExt[ n ] += (pExt[ n - 1 ] + pExt[ n + 1 ] + 2) >> 2;
            }

            // copy
            for (x = 0; x < iN2; x++) {
                x2 = x << 1;
                pBuff[ x + ySize ] = pExt[ x2 ];
            }
        }
        // step #2: vertical transform
        iN2 = iN >> 1;
        for (x = 0; x < iSize; x++) {
            // copy
            for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
                pExt[ y ] = pBuff[ x + ySize ];
            }

            // reflection
            pExt[ -1     ] = pExt[ +1   ];
            pExt[ -2     ] = pExt[ +2   ];
            pExt[ iN   ] = pExt[ iN - 2 ];
            pExt[ iN + 1 ] = pExt[ iN - 3 ];

            // filtering (H)
            for (n = -1; n < iN; n += 2) {
                pExt[ n ] -= (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // filtering (L)

            for (n = 0; n < iN; n += 2) {
                pExt[ n ]  = (pExt[ n ] << 1) + ((pExt[ n - 1 ] + pExt[ n + 1 ] + 1) >> 1);
            }

            // copy
            for (y = 0, ySize = 0; y < iN2; y++, ySize += iSize) {
                y2 = y << 1;
                pBuff[ x + ySize ] = pExt[ y2 ];
            }
        }
    }

    for (i = 0; i < (iN >> 1); i++) {
        memcpy(&curr_blk[i][0], &pBuff[i * iSize], (iSize >> 1) * sizeof(int));
    }

    //free( pBuff );
}

void  inv_wavelet_B64(int **curr_blk)
{
    int *pExt = &iEBuff[ LOT_MAX_WLT_TAP ];
    int  i, n, x, y;
    int  iN, iN2;
    int  ySize, y2, x2;
    int iSize = 64, iLevel = 1;
    //int *pBuff = malloc( iSize * iSize * sizeof( int ) );
    int pBuff[MAX_CU_SIZE * MAX_CU_SIZE];

    for (i = 0; i < 32; i++) {
        memcpy(&pBuff[i * iSize], &curr_blk[i][0], 32 * sizeof(int));
    }

    iN = iSize >> (iLevel - 1);
    iN2 = iN >> 1;
    for (i = iLevel - 1; i >= 0; i--) {
        // step #1: vertical transform
        for (x = 0; x < iN; x++) {
            // copy
            for (y = 0, ySize = 0; y < iN2; y++, ySize += iSize) {
                y2 = y << 1;
                pExt[ y2 ] = pBuff[ x + ySize ];
            }

            // reflection
            pExt[ iN ] = pExt[ iN - 2 ];

            // filtering (even pixel)
            for (n = 0; n <= iN;   n += 2) {
                pExt[ n ] >>= 1;
            }

            // filtering (odd pixel)
            for (n = 1; n <= iN - 1; n += 2) {
                pExt[ n ] = (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // copy
            for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
                pBuff[ x + ySize ] = pExt[ y ];
            }
        }

        // step #2: horizontal transform
        for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
            // copy
            for (x = 0; x < iN2; x++) {
                x2 = x << 1;
                pExt[ x2 ] = pBuff[ ySize + x ];
            }

            // reflection
            pExt[ iN ] = pExt[ iN - 2 ];

            // filtering (odd pixel)
            for (n = 1; n <= iN - 1; n += 2) {
                pExt[ n ] = (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // copy
            memcpy(&pBuff[ ySize ], pExt, sizeof(int)*iN);
        }

        // shift size
        iN <<= 1;
        iN2 = iN >> 1;
    }
    for (i = 0; i < iSize; i++) {
        memcpy(&curr_blk[i][0], &pBuff[i * iSize], iSize * sizeof(int));
    }
    //free( pBuff );
}

void  inv_wavelet_NSQT(int **curr_blk, int is_Hor)
{
    int *pExt = &iEBuff[ LOT_MAX_WLT_TAP ];
    int  i, n, x, y;
    int  iN, iN2;
    int  ySize, y2, x2;
    int iSize = 64, iLevel = 1;
    //int *pBuff = malloc( 64 * 64 * sizeof( int ) );
    int pBuff[MAX_CU_SIZE * MAX_CU_SIZE];

    if (is_Hor) {
        iSize = 64;
        iN = 16;
    } else {
        iSize = 16;
        iN = 64;
    }
    for (i = 0; i < iN; i++) {
        memcpy(&pBuff[i * iSize], &curr_blk[i][0], (iSize >> 1) * sizeof(int));
    }

    for (i = iLevel - 1; i >= 0; i--) {
        // step #1: vertical transform
        iN2 = iN >> 1;
        for (x = 0; x < (iSize >> 1); x++) {
            // copy
            for (y = 0, ySize = 0; y < iN2; y++, ySize += iSize) {
                y2 = y << 1;
                pExt[ y2 ] = pBuff[ x + ySize ];
            }

            // reflection
            pExt[ iN ] = pExt[ iN - 2 ];

            // filtering (even pixel)
            for (n = 0; n <= iN;   n += 2) {
                pExt[ n ] >>= 1;
            }

            // filtering (odd pixel)
            for (n = 1; n <= iN - 1; n += 2) {
                pExt[ n ] = (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // copy
            for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
                pBuff[ x + ySize ] = pExt[ y ];
            }
        }

        // step #2: horizontal transform
        iN2 = iSize >> 1;
        for (y = 0, ySize = 0; y < iN; y++, ySize += iSize) {
            // copy
            for (x = 0; x < iN2; x++) {
                x2 = x << 1;
                pExt[ x2 ] = pBuff[ ySize + x ];
            }

            // reflection
            pExt[ iSize ] = pExt[ iSize - 2 ];

            // filtering (odd pixel)
            for (n = 1; n <= iSize - 1; n += 2) {
                pExt[ n ] = (pExt[ n - 1 ] + pExt[ n + 1 ]) >> 1;
            }

            // copy
            memcpy(&pBuff[ ySize ], pExt, sizeof(int)*iSize);
        }
    }
    for (i = 0; i < iN; i++) {
        memcpy(&curr_blk[i][0], &pBuff[i * iSize], iSize * sizeof(int));
    }
    //free( pBuff );
}

void array_shift(int **src, int shift, int iSizeY, int iSizeX)
{
    int x, y;
    for (x = 0; x < iSizeY; x++) {
        for (y = 0; y < iSizeX; y++) {
            if (shift >= 1) {
                src[x][y] = (src[x][y] + (1 << (shift - 1))) >> shift;
            }
        }
    }
}
void array_shift_clip(int **src, int shift, int iSizeY, int iSizeX, int bit_depth)
{
    int x, y;
    int min_val, max_val;

    //min_val = - (1 << (bit_depth - 1));
    max_val = (1 << (bit_depth - 1)) - 1;
    min_val = - max_val - 1;

    for (x = 0; x < iSizeY; x++) {
        for (y = 0; y < iSizeX; y++) {
            if (shift >= 1) {
                src[x][y] = (src[x][y] + (1 << (shift - 1))) >> shift;
            }
            src[x][y] = Clip3(min_val, max_val, src[x][y]);
        }
    }
}

// Functions for Secondary Transforms
void xTr2nd_8_1d_Hor(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor;
    int tmpSrc[SEC_TR_SIZE][SEC_TR_SIZE];

    if (shift >= 1) {
        rnd_factor = 1 << (shift - 1);
    } else {
        rnd_factor = 0;
    }

    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            sum = rnd_factor;
            for (k = 0; k < SEC_TR_SIZE; k++) {
                sum += g_as_SEC_TRANS[i][k] * tmpSrc[j][k];
            }
            if (shift >= 1) {
                psSrc[j][i] = Clip3(-32768, 32767, sum >> shift);
            } else {
                psSrc[j][i] = Clip3(-32768, 32767, sum);
            }
        }
    }
}

void xTr2nd_8_1d_Vert(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor;
    int tmpSrc[SEC_TR_SIZE][SEC_TR_SIZE];

    if (shift >= 1) {
        rnd_factor = 1 << (shift - 1);
    } else {
        rnd_factor = 0;
    }

    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            sum = rnd_factor;
            for (k = 0; k < SEC_TR_SIZE; k++) {
                sum += g_as_SEC_TRANS[i][k] * tmpSrc[k][j];
            }
            if (shift >= 1) {
                psSrc[i][j] = Clip3(-32768, 32767, sum >> shift);
            } else {
                psSrc[i][j] = Clip3(-32768, 32767, sum);
            }
        }
    }
}

void xTr2nd_8_1d_Inv_Vert(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor;
    int tmpSrc[SEC_TR_SIZE][SEC_TR_SIZE];

    if (shift >= 1) {
        rnd_factor = 1 << (shift - 1);
    } else {
        rnd_factor = 0;
    }

    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            sum = rnd_factor;
            for (k = 0; k < SEC_TR_SIZE; k++) {
                sum += g_as_SEC_TRANS[k][i] * tmpSrc[k][j];
            }
            if (shift >= 1) {
                psSrc[i][j] = Clip3(-32768, 32767, sum >> shift);
            } else {
                psSrc[i][j] = Clip3(-32768, 32767, sum);
            }
        }
    }
}

void xTr2nd_8_1d_Inv_Hor(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor;
    int tmpSrc[SEC_TR_SIZE][SEC_TR_SIZE];

    if (shift >= 1) {
        rnd_factor = 1 << (shift - 1);
    } else {
        rnd_factor = 0;
    }
    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < SEC_TR_SIZE; i++) {
        for (j = 0; j < SEC_TR_SIZE; j++) {
            sum = rnd_factor;
            for (k = 0; k < SEC_TR_SIZE; k++) {
                sum += g_as_SEC_TRANS[k][i] * tmpSrc[j][k];
            }
            if (shift >= 1) {
                psSrc[j][i] = Clip3(-32768, 32767, sum >> shift);
            } else {
                psSrc[j][i] = Clip3(-32768, 32767, sum);
            }
        }
    }
}

void xCTr_4_1d_Hor(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            sum = rnd_factor;
            for (k = 0; k < 4; k++) {
                sum += g_as_C_TRANS[i][k] * tmpSrc[j][k];
            }
            psSrc[j][i] = Clip3(-32768, 32767, sum >> shift);
        }
    }
}

void xCTr_4_1d_Vert(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            sum = rnd_factor;
            for (k = 0; k < 4; k++) {
                sum += g_as_C_TRANS[i][k] * tmpSrc[k][j];
            }
            psSrc[i][j] = Clip3(-32768, 32767, sum >> shift);
        }
    }
}

void xCTr_4_1d_Inv_Vert(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            sum = rnd_factor;
            for (k = 0; k < 4; k++) {
                sum += g_as_C_TRANS[k][i] * tmpSrc[k][j];
            }
            psSrc[i][j] = Clip3(-32768, 32767, sum >> shift);
        }
    }
}

void xCTr_4_1d_Inv_Hor(int **psSrc, int shift)
{
    int i, j , k, sum;
    int rnd_factor = shift == 0 ? 0 : 1 << (shift - 1);
    int tmpSrc[4][4];
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tmpSrc[i][j] = psSrc[i][j];
        }
    }
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            sum = rnd_factor;
            for (k = 0; k < 4; k++) {
                sum += g_as_C_TRANS[k][i] * tmpSrc[j][k];
            }
            //psSrc[j][i] = Clip3(-32768,32767,sum >> shift);
            psSrc[j][i] = sum >> shift;
        }
    }
}


void transform_B8(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma, int secT_enabled,
                  int sample_bit_depth)
{
    int shift1, shift2, log2Size;
    int tranSize;
    int vt = 1, ht = 1;
    int uiMode = currMB->l_ipred_mode;
    int block_available_up = currMB->block_available_up;
    int block_available_left = currMB->block_available_left;

    log2Size = (trans_BitSize == B64X64_IN_BIT) ? trans_BitSize - 1 : trans_BitSize;
    tranSize = 1 << log2Size;
    shift1 = (sample_bit_depth + 1) + FACTO_BIT + log2Size - LIMIT_BIT;
    shift2 = FACTO_BIT + log2Size;
    if (trans_BitSize == B64X64_IN_BIT) {
        shift1 += 1; // because after wavelet, the residue is 10-bit
    }

    if (trans_BitSize == B8X8_IN_BIT) {
        partialButterfly8(curr_blk, tmp, 8);
        array_shift(tmp, shift1, 8, 8);
        partialButterfly8(tmp, curr_blk, 8);
        array_shift(curr_blk, shift2, 8, 8);
    } else if (trans_BitSize == B16X16_IN_BIT) {
        partialButterfly16(curr_blk, tmp, 16);
        array_shift(tmp, shift1, 16, 16);
        partialButterfly16(tmp, curr_blk, 16);
        array_shift(curr_blk, shift2, 16, 16);
    } else if (trans_BitSize == B32X32_IN_BIT) {
        partialButterfly32(curr_blk, tmp, 32);
        array_shift(tmp, shift1, 32, 32);
        partialButterfly32(tmp, curr_blk, 32);
        array_shift(curr_blk, shift2, 32, 32);
    } else if (trans_BitSize == B64X64_IN_BIT) {
        wavelet64(curr_blk);
        partialButterfly32(curr_blk, tmp, 32);
        array_shift(tmp, shift1, 32, 32);
        partialButterfly32(tmp, curr_blk, 32);
        array_shift(curr_blk, shift2, 32, 32);
    }

    else if (trans_BitSize == B4X4_IN_BIT) {
        if (IS_INTRA(currMB) && (!isChroma) &&  secT_enabled) {
#if BUGFIXED_COMBINED_ST_BD
            xCTr_4_1d_Hor(curr_blk, shift1 + 1);
            xCTr_4_1d_Vert(curr_blk, shift2 + 1);
#else
            xCTr_4_1d_Hor(curr_blk, 1);
            xCTr_4_1d_Vert(curr_blk, 8);
#endif
        } else {
            partialButterfly4(curr_blk, tmp, 4);
            array_shift(tmp, shift1, 4, 4);
            partialButterfly4(tmp, curr_blk, 4);
            array_shift(curr_blk, shift2, 4, 4);
        }
    }

    if (IS_INTRA(currMB) &&  secT_enabled && (!isChroma) && trans_BitSize >= SEC_TR_MIN_BITSIZE) {
        vt = (uiMode >= 0 && uiMode <= 23);
        ht = (uiMode >= 13 && uiMode <= 32) || (uiMode >= 0 && uiMode <= 2);
        vt = vt && block_available_up;
        ht = ht && block_available_left;
        if (vt) {
            xTr2nd_8_1d_Vert(curr_blk, 7);
        }
        if (ht) {
            xTr2nd_8_1d_Hor(curr_blk, 7);
        }
    }
}

void transform_NSQT(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma, int secT_enabled,
                    int sample_bit_depth) // block to be transformed.
{
    int iSizeX, iSizeY;
    int iVer = 0, iHor = 0;
    int shift1, shift2, log2Size;
    int tranSize;
    int iBlockType = currMB->cuType;
    int vt = 1, ht = 1;
    int uiMode = currMB->l_ipred_mode;
    int block_available_up = currMB->block_available_up;
    int block_available_left = currMB->block_available_left;

    log2Size = trans_BitSize;
    tranSize = 1 << log2Size;
    shift1 = (sample_bit_depth + 1) + FACTO_BIT + log2Size - LIMIT_BIT;
    shift2 = FACTO_BIT + log2Size;
    if (trans_BitSize == B32X32_IN_BIT) {
        shift1 += 1; // because after wavelet, the residue is 10-bit
    }

    if (iBlockType == P2NXN || iBlockType == PHOR_UP || iBlockType == PHOR_DOWN) {
        iVer = 0;
        iHor = 1;
    } else if (iBlockType == PNX2N || iBlockType == PVER_LEFT || iBlockType == PVER_RIGHT) {
        iVer = 1;
        iHor = 0;
    }
    if (iBlockType == InNxNMB) {
        iVer = 0;
        iHor = 1;
    } else if (iBlockType == INxnNMB) {
        iVer = 1;
        iHor = 0;
    }
    if (iHor == 1) {
        iSizeX = (1 << (trans_BitSize + 1));
        iSizeY = (1 << (trans_BitSize - 1));
    } else if (iVer == 1) {
        iSizeX = (1 << (trans_BitSize - 1));
        iSizeY = (1 << (trans_BitSize + 1));
    }
    if (trans_BitSize == B8X8_IN_BIT) {
        if (iHor == 1) {
            partialButterfly16(curr_blk, tmp, 4);
            array_shift(tmp, shift1 + 1, 16, 4);
            partialButterfly4(tmp, curr_blk, 16);
            array_shift(curr_blk, shift2 - 1, 4, 16);
        } else if (iVer == 1) {
            partialButterfly4(curr_blk, tmp, 16);
            array_shift(tmp, shift1 - 1, 4, 16);
            partialButterfly16(tmp, curr_blk, 4);
            array_shift(curr_blk, shift2 + 1, 16, 4);
        }
    } else if (trans_BitSize == B16X16_IN_BIT) {
        if (iHor == 1) {
            partialButterfly32(curr_blk, tmp, 8);
            array_shift(tmp, shift1 + 1, 32, 8);
            partialButterfly8(tmp, curr_blk, 32);
            array_shift(curr_blk, shift2 - 1, 8, 32);
        } else if (iVer == 1) {
            partialButterfly8(curr_blk, tmp, 32);
            array_shift(tmp, shift1 - 1, 8, 32);
            partialButterfly32(tmp, curr_blk, 8);
            array_shift(curr_blk, shift2 + 1, 32, 8);
        }
    } else if (trans_BitSize == B32X32_IN_BIT) {
        if (iHor == 1) {
            wavelet_NSQT(curr_blk, 1);
            partialButterfly32(curr_blk, tmp, 8);
            array_shift(tmp, shift1, 32, 8);
            partialButterfly8(tmp, curr_blk, 32);
            array_shift(curr_blk, shift2 - 2, 8, 32);
        } else if (iVer == 1) {
            wavelet_NSQT(curr_blk, 0);
            partialButterfly8(curr_blk, tmp, 32);
            array_shift(tmp, shift1 - 2, 8, 32);
            partialButterfly32(tmp, curr_blk, 8);
            array_shift(curr_blk, shift2, 32, 8);
        }
    }

    if (IS_INTRA(currMB) && secT_enabled && (!isChroma)) {
        vt = (uiMode >= 0 && uiMode <= 23);
        ht = (uiMode >= 13 && uiMode <= 32) || (uiMode >= 0 && uiMode <= 2);
        vt = vt && block_available_up;
        ht = ht && block_available_left;
        if (vt) {
            xTr2nd_8_1d_Vert(curr_blk, 7);
        }
        if (ht) {
            xTr2nd_8_1d_Hor(curr_blk, 7);
        }
    }
}



void inv_transform_B8(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma, int secT_enabled,
                      int sample_bit_depth)
{
    int shift5, shift6, clipValue1, clipValue2, tranSize;
    int vt = 1, ht = 1;
    int uiMode = currMB->l_ipred_mode;
    int block_available_up = currMB->block_available_up;
    int block_available_left = currMB->block_available_left;
    shift5 = 5;
    shift6 = 19 - sample_bit_depth;
    shift6 ++;
    clipValue1 = LIMIT_BIT;
    clipValue2 = sample_bit_depth + 1;
    tranSize = (trans_BitSize == B64X64_IN_BIT) ? (1 << (trans_BitSize - 1)) : (1 << trans_BitSize);

    if (IS_INTRA(currMB)  &&  secT_enabled && (!isChroma) && trans_BitSize >= SEC_TR_MIN_BITSIZE) {
        vt = (uiMode >= 0 && uiMode <= 23);
        ht = (uiMode >= 13 && uiMode <= 32) || (uiMode >= 0 && uiMode <= 2);
        vt = vt && block_available_up;
        ht = ht && block_available_left;
        if (ht) {
            xTr2nd_8_1d_Inv_Hor(curr_blk, 7);
        }
        if (vt) {
            xTr2nd_8_1d_Inv_Vert(curr_blk, 7);
        }
    }

    if (trans_BitSize == B64X64_IN_BIT) {
        shift6 = shift6 - 1;
        clipValue2 = clipValue2 + 1;
    }

    if (trans_BitSize == B8X8_IN_BIT) {
        partialButterflyInverse8(curr_blk, tmp, 8);
        array_shift_clip(tmp, shift5, 8, 8, clipValue1);
        partialButterflyInverse8(tmp, curr_blk, 8);
        array_shift_clip(curr_blk, shift6, 8, 8 , clipValue2);
    } else if (trans_BitSize == B16X16_IN_BIT) {
        partialButterflyInverse16(curr_blk, tmp, 16);
        array_shift_clip(tmp, shift5, 16, 16, clipValue1);
        partialButterflyInverse16(tmp, curr_blk, 16);
        array_shift_clip(curr_blk, shift6, 16, 16, clipValue2);
    } else if (trans_BitSize == B32X32_IN_BIT) {
        partialButterflyInverse32(curr_blk, tmp, 32);
        array_shift_clip(tmp, shift5, 32, 32, clipValue1);
        partialButterflyInverse32(tmp, curr_blk, 32);
        array_shift_clip(curr_blk, shift6, 32, 32, clipValue2);
    }

    else if (trans_BitSize == B64X64_IN_BIT) {
        partialButterflyInverse32(curr_blk, tmp, 32);
        array_shift_clip(tmp, shift5, 32, 32, clipValue1);
        partialButterflyInverse32(tmp, curr_blk, 32);
        array_shift_clip(curr_blk, shift6, 32, 32, clipValue2);
        inv_wavelet_B64(curr_blk);
    }

    else if (trans_BitSize == B4X4_IN_BIT) {
        if (IS_INTRA(currMB) && (!isChroma) &&  secT_enabled) {
            xCTr_4_1d_Inv_Vert(curr_blk, shift5);
            xCTr_4_1d_Inv_Hor(curr_blk, shift6 + 2);
            array_shift_clip(curr_blk, 0, 4, 4, clipValue2);
        } else {
            partialButterflyInverse4(curr_blk, tmp, 4);
            array_shift_clip(tmp, shift5, 4, 4, clipValue1);
            partialButterflyInverse4(tmp, curr_blk, 4);
            array_shift_clip(curr_blk, shift6, 4, 4, clipValue2);
        }
    }
}


void inv_transform_NSQT(int **curr_blk, unsigned int trans_BitSize, codingUnit *currMB, int isChroma,
                        int secT_enabled, int sample_bit_depth)  // block to be transformed.
{
    int iSizeX, iSizeY;
    int iVer = 0, iHor = 0;

    int shift5, shift6, clipValue1, clipValue2;
    int iBlockType = currMB->cuType;
    int vt = 1, ht = 1;
    int uiMode = currMB->l_ipred_mode;
    int block_available_up = currMB->block_available_up;
    int block_available_left = currMB->block_available_left;

    shift5 = 5;
    shift6 = 19 - sample_bit_depth;
    shift6 ++;

    clipValue1 = LIMIT_BIT;
    clipValue2 = sample_bit_depth + 1;
    if (trans_BitSize == B32X32_IN_BIT) {
        shift6 = shift6 - 1;
        clipValue2 = clipValue2 + 1;
    }
    if (iBlockType == P2NXN || iBlockType == PHOR_UP || iBlockType == PHOR_DOWN || iBlockType == InNxNMB) {
        iVer = 0;
        iHor = 1;
    } else if (iBlockType == PNX2N || iBlockType == PVER_LEFT || iBlockType == PVER_RIGHT || iBlockType == INxnNMB) {
        iVer = 1;
        iHor = 0;
    }
    if (iHor == 1) {
        iSizeX = (trans_BitSize == B32X32_IN_BIT) ? (1 << trans_BitSize) : (1 << (trans_BitSize + 1));
        iSizeY = (trans_BitSize == B32X32_IN_BIT) ? (1 << (trans_BitSize - 2)) : (1 << (trans_BitSize - 1));
    } else if (iVer == 1) {
        iSizeX = (trans_BitSize == B32X32_IN_BIT) ? (1 << (trans_BitSize - 2)) : (1 << (trans_BitSize - 1));
        iSizeY = (trans_BitSize == B32X32_IN_BIT) ? (1 << trans_BitSize) : (1 << (trans_BitSize + 1));
    }

    if (IS_INTRA(currMB)  &&  secT_enabled && (!isChroma)) {
        vt = (uiMode >= 0 && uiMode <= 23);
        ht = (uiMode >= 13 && uiMode <= 32) || (uiMode >= 0 && uiMode <= 2);
        vt = vt && block_available_up;
        ht = ht && block_available_left;
        if (ht) {
            xTr2nd_8_1d_Inv_Hor(curr_blk, 7);
        }
        if (vt) {
            xTr2nd_8_1d_Inv_Vert(curr_blk, 7);
        }
    }

    if (trans_BitSize == B8X8_IN_BIT) {
        if (iHor == 1) {
            partialButterflyInverse4(curr_blk, tmp, 16);
            array_shift_clip(tmp, shift5, 16, 4, clipValue1);
            partialButterflyInverse16(tmp, curr_blk, 4);
            array_shift_clip(curr_blk, shift6, 4, 16, clipValue2);
        } else if (iVer == 1) {
            partialButterflyInverse16(curr_blk, tmp, 4);
            array_shift_clip(tmp, shift5, 4, 16, clipValue1);
            partialButterflyInverse4(tmp, curr_blk, 16);
            array_shift_clip(curr_blk, shift6, 16, 4, clipValue2);
        }
    } else if (trans_BitSize == B16X16_IN_BIT) {
        if (iHor == 1) {
            partialButterflyInverse8(curr_blk, tmp, 32);
            array_shift_clip(tmp, shift5, 32, 8, clipValue1);
            partialButterflyInverse32(tmp, curr_blk, 8);
            array_shift_clip(curr_blk, shift6, 8, 32, clipValue2);
        } else if (iVer == 1) {
            partialButterflyInverse32(curr_blk, tmp, 8);
            array_shift_clip(tmp, shift5, 8, 32, clipValue1);
            partialButterflyInverse8(tmp, curr_blk, 32);
            array_shift_clip(curr_blk, shift6, 32, 8, clipValue2);
        }
    } else if (trans_BitSize == B32X32_IN_BIT) {
        if (iHor == 1) {
            partialButterflyInverse8(curr_blk, tmp, 32);
            array_shift_clip(tmp, shift5, 32, 8, clipValue1);
            partialButterflyInverse32(tmp, curr_blk, 8);
            array_shift_clip(curr_blk, shift6, 8, 32, clipValue2);
        } else if (iVer == 1) {
            partialButterflyInverse32(curr_blk, tmp, 8);
            array_shift_clip(tmp, shift5, 8, 32, clipValue1);
            partialButterflyInverse8(tmp, curr_blk, 32);
            array_shift_clip(curr_blk, shift6, 32, 8, clipValue2);
        }
        if (iHor == 1) {
            inv_wavelet_NSQT(curr_blk, 1);
        } else if (iVer == 1) {
            inv_wavelet_NSQT(curr_blk, 0);
        }
    }
}
