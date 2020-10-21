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
* File name: wquant.h
* Function:  Frequency Weighting Quantization,include
*               a). Frequency weighting model, quantization
*               b). Picture level user-defined frequency weighting
*               c). Macroblock level adaptive frequency weighting mode Decision
*            According to proposals: m1878,m2148,m2331
* Author:    Jianhua Zheng, Hisilicon
*************************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../lcommon/inc/commonStructures.h"
#include "../../lcommon/inc/memalloc.h"
#include "global.h"
#include "wquant.h"


int ** *LevelScale4x4;
int ** *LevelScale8x8; //LevelScale2Nx2N[intra/inter][j][i]
int ** *LevelScale16x16;
int ** *LevelScale32x32;



int WeightQuantEnable;
short UseDefaultScalingMatrixFlag[2];


#if !WQ_MATRIX_FCD
short wq_param_default[2][6] = {
    {135, 143, 143, 160, 160, 213},
    {128, 98, 106, 116, 116, 128}
};
#else
short wq_param_default[2][6] = {
    {67, 71, 71, 80, 80, 106},
    {64, 49, 53, 58, 58, 64}
};
#endif

//////////////////////////////////////////////////////////////////////////
///Scale Matrix only in encoder side
///for pre scale for transform coefficients

// Adaptive frequency weighting quantization
//#if FREQUENCY_WEIGHTING_QUANTIZATION

int ScaleM[4][4] = {
    {32768, 32768, 32768, 32768},
    {32768, 32768, 32768, 32768},
    {32768, 32768, 32768, 32768},
    {32768, 32768, 32768, 32768}
};


#if !WQ_MATRIX_FCD
int g_WqMDefault4x4[16] = {
    16,   16,     16,     17,
    16,   16,     17,     18,
    16,   17,     19,     20,
    18,   19,     21,     24
};


int g_WqMDefault8x8[64] = {
    16,   16,     16,     16,     17,     17,     18,     19,
    16,   16,     16,     17,     18,     19,     21,     23,
    16,   16,     17,     18,     19,     20,     22,     25,
    16,   17,     18,     20,     21,     23,     25,     28,
    17,   18,     20,     21,     23,     26,     28,     32,
    19,   20,     21,     23,     26,     29,     33,     38,
    24,   25,     26,     29,     31,     35,     41,     47,
    26,   27,     29,     32,     38,     43,     48,     54
};
#else
int g_WqMDefault4x4[16] = {
    64,   64,     64,     68,
    64,   64,     68,     72,
    64,   68,     76,     80,
    72,   76,     84,     96
};


int g_WqMDefault8x8[64] = {
    64,   64,     64,     64,     68,     68,     72,     76,
    64,   64,     64,     68,     72,     76,     84,     92,
    64,   64,     68,     72,     76,     80,     88,     100,
    64,   68,     72,     80,     84,     92,     100,    112,
    68,   72,     80,     84,     92,     104,    112,    128,
    76,   80,     84,     92,     104,    116,    132,    152,
    96,   100,    104,    116,    124,    140,    164,    188,
    104,  108,    116,    128,    152,    172,    192,    216
};
#endif


short cur_wq_matrix[4][64]; //cur_wq_matrix[matrix_id][coef]
short wq_matrix[2][2][64];  //wq_matrix[matrix_id][detail/undetail][coef]
short seq_wq_matrix[2][64]; //seq_wq_matrix[matrix_id][coef]
short pic_user_wq_matrix[2][64]; //pic_user_wq_matrix[matrix_id][coef]


short wq_param[2][6];


int cur_frame_wq_param;  // 7.2.3.1 weighting_quant_param
int cur_frame_wq_model;  // 7.2.3.1 weighting_quant_model
int cur_picture_scene_model;


/*
*************************************************************************
* Function: Allocate and initialise Q matrix values.
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void Init_QMatrix(void)
{
    get_mem3Dint(&LevelScale4x4, 2, 4, 4);
    get_mem3Dint(&LevelScale8x8, 2, 8, 8);
    get_mem3Dint(&LevelScale16x16, 2, 16, 16);
    get_mem3Dint(&LevelScale32x32, 2, 32, 32);
}


/*
*************************************************************************
* Function: Free Q matrix arrays.
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void free_QMatrix()
{
    free_mem3Dint(LevelScale4x4, 2);
    free_mem3Dint(LevelScale8x8, 2);
    free_mem3Dint(LevelScale16x16, 2);
    free_mem3Dint(LevelScale32x32, 2);
}


/*
*************************************************************************
* Function: Initializes the frequency weighting parameters for a new frame
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void InitSeqQuantParam()
{
    int uiWQMSizeId, uiWMQId;
    int i;
    int Seq_WQM[64];
    int uiBlockSize;

    for (uiWQMSizeId = 0; uiWQMSizeId < 4; uiWQMSizeId++)
        for (i = 0; i < 64; i++)
#if !WQ_MATRIX_FCD
            cur_wq_matrix[uiWQMSizeId][i] = 1 << 4;
#else
            cur_wq_matrix[uiWQMSizeId][i] = 1 << WQ_FLATBASE_INBIT;
#endif

    for (uiWQMSizeId = 0; uiWQMSizeId < 2; uiWQMSizeId++)

    {
        uiBlockSize = min(1 << (uiWQMSizeId + 2), 8);
        uiWMQId = (uiWQMSizeId < 2) ? uiWQMSizeId : 1;
        if (input->SeqWQM == 0) {
            //  GetDefaultWQM(uiWMQId, &Seq_WQM);
            GetDefaultWQM(uiWMQId, Seq_WQM);//yuquanhe@hisilico.com
        } else if (input->SeqWQM == 1) {
            // GetUserDefWQM(input->SeqWQFile, uiWMQId, &Seq_WQM);
            GetUserDefWQM(input->SeqWQFile, uiWMQId, Seq_WQM);//yuquanhe@hisilico.com
        }
        for (i = 0; i < (uiBlockSize * uiBlockSize); i++) {
            seq_wq_matrix[uiWQMSizeId][i] = Seq_WQM[i];
        }
    }
}


void GetUserDefWQM(char *pchFile, int sizeId, int *src)
{
    FILE *fp;
    char line[1024];
    char *ret;
    int x, y, coef, uiWqMSize;;

    if ((fp = fopen(pchFile, "r")) == (FILE *)NULL) {
        snprintf(hc->errortext, ET_SIZE, "Can't open file %s.\n %s.\n", pchFile);
        error(hc->errortext, 303);
    }
    uiWqMSize = 1 << (sizeId + 2);
    fseek(fp, 0, 0);
    do {
        ret = fgets(line, 1024, fp);
        if ((ret == NULL) || (strstr(line, WQMType[sizeId]) == NULL && feof(fp))) {
            snprintf(hc->errortext, ET_SIZE, "Error: can't read Matrix %s.\n", WQMType[sizeId]);
            error(hc->errortext, 304);
        }
    } while (strstr(line, WQMType[sizeId]) == NULL);

    for (y = 0; y < uiWqMSize; y++)
        for (x = 0; x < uiWqMSize; x++) {
            fscanf(fp, "%d,", &coef);
            if ((coef == 0) || coef > 255) {
                snprintf(hc->errortext, ET_SIZE, "QM coefficients %d is not in the range of [1, 255].\n", coef);
                error(hc->errortext, 305);
            } else {
                src[y * uiWqMSize + x] = coef;
            }
        }
    fclose(fp);
}

void GetDefaultWQM(int sizeId, int *src)
{
    int i, uiWqMSize;;
    uiWqMSize = 1 << (sizeId + 2);
    if (sizeId == 0) {
        for (i = 0; i < uiWqMSize * uiWqMSize; i++) {
            src[i] = g_WqMDefault4x4[i];
        }
    } else if (sizeId == 1) {
        for (i = 0; i < uiWqMSize * uiWqMSize; i++) {
            src[i] = g_WqMDefault8x8[i];
        }
    }
}


/*
*************************************************************************
* Function: Initializes the frequency weighting parameters for a new frame
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void InitFrameQuantParam()
{
    int i, j, k;
    int wq_model;
    int uiWQMSizeId, uiBlockSize, uiWMQId;
    int Pic_WQM[64];

    if ((input->WQEnable) && (input->PicWQEnable)) {
        WeightQuantEnable = 1;
    } else {
        WeightQuantEnable = 0;
    }

    if (WeightQuantEnable) {
        UseDefaultScalingMatrixFlag[0] = UseDefaultScalingMatrixFlag[1] = 0; // use weighed scaling matrix
    } else {
        UseDefaultScalingMatrixFlag[0] = UseDefaultScalingMatrixFlag[1] = 1; // use default scaling matrix
    }


    if (!WeightQuantEnable) {
        for (i = 0; i < 2; i++)
            for (j = 0; j < 6; j++) {
                wq_param[i][j] = 1 << WQ_FLATBASE_INBIT;
            }

        for (uiWQMSizeId = 0; uiWQMSizeId < 2; uiWQMSizeId++) {
            uiBlockSize = 1 << (uiWQMSizeId + 2);
            for (k = 0; k < 2; k++)
                for (j = 0; j < uiBlockSize; j++)
                    for (i = 0; i < uiBlockSize; i++) {
                        wq_matrix[uiWQMSizeId][k][j * uiBlockSize + i] = 1 << WQ_FLATBASE_INBIT;
                    }
        }
    } else {
        if (input->PicWQDataIndex == 1) {
            // Patch the Weighting Parameters
            // Use default weighted parameters, input->WQParam==0
            for (i = 0; i < 2; i++)
                for (j = 0; j < 6; j++) {
                    wq_param[i][j] = 1 << WQ_FLATBASE_INBIT;
                }

            // if input->WQParam!=0, update wq_param
            if (input->WQParam == 0) {
                cur_frame_wq_param = FRAME_WQ_DEFAULT;    // default Param - Detailed
                for (i = 0; i < 6; i++) {
                    wq_param[DETAILED][i] = wq_param_default[DETAILED][i];
                }
            } else if (input->WQParam == 1) {
                cur_frame_wq_param = USER_DEF_UNDETAILED; // 7.2.3.1 UnDetailed Param User Defined
                // Load user defined weighted parameters
                GetUserDefParam(input->WeightParamUnDetailed, 0);
            } else if (input->WQParam == 2) {
                cur_frame_wq_param = USER_DEF_DETAILED;   // 7.2.3.1 Detailed Param User Defined
                // Load user defined weighted parameters
                GetUserDefParam(input->WeightParamDetailed, 1);
            }

            // Reconstruct the Weighting matrix
            wq_model = input->WQModel;

            for (k = 0; k < 2; k++)
                for (j = 0; j < 8; j++)
                    for (i = 0; i < 8; i++) {
                        wq_matrix[1][k][j * 8 + i] = (wq_param[k][WeightQuantModel[wq_model][j * 8 + i]]);
                    }
            // afw_4x4  Jianhua Zheng 200906

            for (k = 0; k < 2; k++)
                for (j = 0; j < 4; j++)
                    for (i = 0; i < 4; i++) {
                        wq_matrix[0][k][j * 4 + i] = (wq_param[k][WeightQuantModel4x4[wq_model][j * 4 + i]]);
                    }
        }//input->PicWQDataIndex==1
        else if (input->PicWQDataIndex == 2) {
            for (uiWQMSizeId = 0; uiWQMSizeId < 2; uiWQMSizeId++) {
                uiBlockSize = min(1 << (uiWQMSizeId + 2), 8);
                uiWMQId = (uiWQMSizeId < 2) ? uiWQMSizeId : 1;
                // GetUserDefWQM(input->PicWQFile, uiWMQId, &Pic_WQM);
                GetUserDefWQM(input->PicWQFile, uiWMQId, Pic_WQM);//yuquanhe@hisilicon.com
                for (i = 0; i < (uiBlockSize * uiBlockSize); i++) {
                    pic_user_wq_matrix[uiWQMSizeId][i] = Pic_WQM[i];
                }
            }
        }//input->PicWQDataIndex==2
    }

    for (uiWQMSizeId = 0; uiWQMSizeId < 4; uiWQMSizeId++)
        for (i = 0; i < 64; i++) {
            cur_wq_matrix[uiWQMSizeId][i] = 1 << WQ_FLATBASE_INBIT;
        }
}

/*
*************************************************************************
* Function: Calculate the level scale matrix from the current frequency
*           weighting matrix
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void CalculateQuantParam(int uiMatrixId)
//  uiMatrixId   //0: 4x4  1:8x8  2: 16x16  3:32x32
{
    int i, j;
    int ***LevelScaleNxN;
    int uiBlockSize = 1 << (uiMatrixId + 2);

    if (uiMatrixId == 0) {
        LevelScaleNxN = LevelScale4x4;
    } else if (uiMatrixId == 1) {
        LevelScaleNxN = LevelScale8x8;
    }
#if AWQ_LARGE_BLOCK_ENABLE
    else if (uiMatrixId == 2) {
        LevelScaleNxN = LevelScale16x16;
    } else if (uiMatrixId == 3) {
        LevelScaleNxN = LevelScale32x32;
    }
#endif

    if (WeightQuantEnable) {
        for (j = 0; j < uiBlockSize; j++)
            for (i = 0; i < uiBlockSize; i++) {
                if (UseDefaultScalingMatrixFlag[1]) {
                    LevelScaleNxN[1][j][i]    = ScaleM[j & 3][i & 3];
                } else {
                    if ((uiMatrixId == 0) || (uiMatrixId == 1)) {
                        LevelScaleNxN[1][j][i]    = (int)((float)(ScaleM[j & 3][i & 3] << WQ_FLATBASE_INBIT) / cur_wq_matrix[uiMatrixId][j *
                                                          uiBlockSize + i]);
                    }
#if AWQ_LARGE_BLOCK_ENABLE
#if AWQ_LARGE_BLOCK_EXT_MAPPING
                    else if (uiMatrixId == 2) {
                        LevelScaleNxN[1][j][i]    = (int)((float)(ScaleM[j & 3][i & 3] << WQ_FLATBASE_INBIT) / cur_wq_matrix[uiMatrixId][(j >>
                                                          1) * (uiBlockSize >> 1) + (i >> 1)]);
                    } else if (uiMatrixId == 3) {
                        LevelScaleNxN[1][j][i]    = (int)((float)(ScaleM[j & 3][i & 3] << WQ_FLATBASE_INBIT) / cur_wq_matrix[uiMatrixId][(j >>
                                                          2) * (uiBlockSize >> 2) + (i >> 2)]);
                    }
#endif
#else
                    else if ((uiMatrixId == 2) || (uiMatrixId == 3)) {
                        LevelScaleNxN[1][j][i]    = ScaleM[j & 3][i & 3];
                    }
#endif
                }
                if (UseDefaultScalingMatrixFlag[0]) {
                    LevelScaleNxN[0][j][i]    = ScaleM[j & 3][i & 3];
                } else {
                    if ((uiMatrixId == 0) || (uiMatrixId == 1)) {
                        LevelScaleNxN[0][j][i]    = (int)((float)(ScaleM[j & 3][i & 3] << WQ_FLATBASE_INBIT) / cur_wq_matrix[uiMatrixId][j *
                                                          uiBlockSize + i]);
                    }
#if AWQ_LARGE_BLOCK_ENABLE
#if AWQ_LARGE_BLOCK_EXT_MAPPING
                    else if (uiMatrixId == 2) {
                        LevelScaleNxN[0][j][i]    = (int)((float)(ScaleM[j & 3][i & 3] << WQ_FLATBASE_INBIT) / cur_wq_matrix[uiMatrixId][(j >>
                                                          1) * (uiBlockSize >> 1) + (i >> 1)]);
                    } else if (uiMatrixId == 3) {
                        LevelScaleNxN[0][j][i]    = (int)((float)(ScaleM[j & 3][i & 3] << WQ_FLATBASE_INBIT) / cur_wq_matrix[uiMatrixId][(j >>
                                                          2) * (uiBlockSize >> 2) + (i >> 2)]);
                    }
#endif
#else
                    else if ((uiMatrixId == 2) || (uiMatrixId == 3)) {
                        LevelScaleNxN[0][j][i]    = ScaleM[j & 3][i & 3];
                    }
#endif
                }
            }

    } else {
        for (j = 0; j < uiBlockSize; j++)
            for (i = 0; i < uiBlockSize; i++) {
                LevelScaleNxN[1][j][i]         = ScaleM[j & 3][i & 3];
                LevelScaleNxN[0][j][i]         = ScaleM[j & 3][i & 3];
            }
    }
}




/*
*********************************************************************************
* Function: Read user-defined frequency weighting parameters from configure file
* Input:    str_param, input parameters string
*           mode,      =0  load string to the UnDetailed parameters
*                      =1  load string to the Detailed parameters
* Output:
* Return:
* Attention:
*********************************************************************************
*/

void GetUserDefParam(char *str_param, int mode)
{
    short param;

    char *p;
    short escape = 0;
    short num = 0;
    char str[WQMODEL_PARAM_SIZE];

    if (strlen(str_param) > WQMODEL_PARAM_SIZE) {
        snprintf(hc->errortext, ET_SIZE, "Cannot read the weight parameters in configuration file %s.\n", str_param);
        error(hc->errortext, 301);
    }
    strcpy(str, str_param);

    p = str;

    while (1) {
        if (*p == '[') {
            p++;
            param = 0;
            continue;
        } else if ((*p >= '0') && (*p <= '9')) {
            param = param * 10 + (short)(*p - '0');
        } else if ((*p == ',') || (*p == ' ')) {
            wq_param[mode][num] = param;
            num++;
            param = 0;
        }

        if (*p != ']') {
            p++;
        } else {
            wq_param[mode][num] = param;
            num++;
            break;
        }
    }
    if (num != PARAM_NUM) {
        snprintf(hc->errortext, ET_SIZE, "Not all of the weight parameters is loaded in configuration file %s.\n", str_param);
        error(hc->errortext, 302);
    }

}

/*
*************************************************************************
* Function: Update the frequency weighting matrix for current frame
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void FrameUpdateWQMatrix()
{
    int i;
    int uiWQMSizeId, uiWMQId;
    int uiBlockSize;

    if (WeightQuantEnable) {
#if AWQ_LARGE_BLOCK_ENABLE
        for (uiWQMSizeId = 0; uiWQMSizeId < 4; uiWQMSizeId++)
#else
        for (uiWQMSizeId = 0; uiWQMSizeId < 2; uiWQMSizeId++)
#endif
        {
            uiBlockSize = min(1 << (uiWQMSizeId + 2), 8);
            uiWMQId = (uiWQMSizeId < 2) ? uiWQMSizeId : 1;
            if (input->PicWQDataIndex == 0) {
                for (i = 0; i < (uiBlockSize * uiBlockSize); i++) {
                    cur_wq_matrix[uiWQMSizeId][i] = seq_wq_matrix[uiWMQId][i];
                }
            } else if (input->PicWQDataIndex == 1) {
                if (input->WQParam == 0) {
                    for (i = 0; i < (uiBlockSize * uiBlockSize); i++) {
                        cur_wq_matrix[uiWQMSizeId][i] = wq_matrix[uiWMQId][DETAILED][i];
                    }

                } else if (input->WQParam == 1) {
                    for (i = 0; i < (uiBlockSize * uiBlockSize); i++) {
                        cur_wq_matrix[uiWQMSizeId][i] = wq_matrix[uiWMQId][0][i];
                    }
                } else if (input->WQParam == 2) {
                    for (i = 0; i < (uiBlockSize * uiBlockSize); i++) {
                        cur_wq_matrix[uiWQMSizeId][i] = wq_matrix[uiWMQId][1][i];
                    }
                }
            } else if (input->PicWQDataIndex == 2) {

                for (i = 0; i < (uiBlockSize * uiBlockSize); i++) {
                    cur_wq_matrix[uiWQMSizeId][i] = pic_user_wq_matrix[uiWMQId][i];
                }
            }
        }
    }

#if AWQ_LARGE_BLOCK_ENABLE
    for (uiWQMSizeId = 0; uiWQMSizeId < 4; uiWQMSizeId++)
#else
    for (uiWQMSizeId = 0; uiWQMSizeId < 2; uiWQMSizeId++)
#endif
    {
        CalculateQuantParam(uiWQMSizeId);
    }
}

