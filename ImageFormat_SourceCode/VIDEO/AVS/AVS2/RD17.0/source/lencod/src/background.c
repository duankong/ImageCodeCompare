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
#include "../../lcommon/inc/commonStructures.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/defines.h"
#include "../../lcommon/inc/memalloc.h"
#include "global.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "memory.h"


/*
***************************************************************************
* Function:Background modeling using our proposed weight based running average algorithm
* Input:
* Output:
* Return:
* Attention:
* Author:  Zhang Xianguo, Peking University, 2010.08
***************************************************************************
*/

byte *imageData_current;
byte *imageData_next;
float *bgmodel_bgbuffer;
float *bgmodel_bgbuffer_temp;
unsigned short *imageData_index;
unsigned char *weight;
int frameWidth;
int frameHeight;
int AverageNumber = 0;
int threshold_before;
#if !RD1501_FIX_BG
int g_bg_number;
#endif
int mark_train_over;
byte *u_bgmodel_bgbuffer;

//int global_mv_modeling=0;
void bg_allocModel(byte *frame, int w, int h)
{
    int imgW = w;
    int imgH = h;
    int size_frame_int = imgW * imgH * 3 / 2 ;
    int size_y = imgW * imgH;
    int j;
    frameWidth = w;
    frameHeight = h;
    printf("alloc model");
    //////////////////FILE * LIST///////////////////
    imageData_current = (byte *)malloc(size_frame_int * sizeof(byte));
    imageData_next = (byte *)malloc(size_frame_int * sizeof(byte));
    if (input->bg_model_method <= 1) {
        bgmodel_bgbuffer  = (float *)malloc(size_frame_int * sizeof(float));
        bgmodel_bgbuffer_temp = (float *)malloc(size_frame_int * sizeof(float));
        imageData_index = (unsigned short *)malloc(size_frame_int * sizeof(unsigned short));
        weight = (unsigned char *)malloc(size_frame_int * sizeof(unsigned char));
        memcpy(imageData_current, frame, sizeof(byte) * size_frame_int);
        memcpy(imageData_next, frame, sizeof(byte) * size_frame_int);
        threshold_before = 14;
#if RD1501_FIX_BG
		he->g_bg_number = 0;
#else
		g_bg_number = 0;
#endif
        for (j = 0; j < size_frame_int; j++) {
            bgmodel_bgbuffer[j] = 0;
            bgmodel_bgbuffer_temp[j] = imageData_current[j];
            imageData_index[j] = 0;
            weight[j] = 1;
            if (input->bg_model_method == 0) {
                bgmodel_bgbuffer[j] = imageData_current[j];
                weight[j] = 1;
            }
        }
    }
}

void bg_releaseModel()
{
    printf("release model");
    if (input->bg_model_method <= 1) {
        free(bgmodel_bgbuffer);
        free(bgmodel_bgbuffer_temp);
        free(imageData_index);
        free(imageData_next);
        free(weight);
        free(imageData_current);
    } else {
        free(u_bgmodel_bgbuffer);
    }
}
void bg_insert(byte *frame)
{
    int imgW = frameWidth;
    int imgH = frameHeight;
    int bg_number;
    int size_frame_int = imgW * imgH * 3 / 2 ;
    int size_y = imgW * imgH;
    int start, j, c, temp;
    float threshold;
    int diff_number = 0;
    byte *imageData = imageData_current;
    int global_x = 0;
    int global_y = 0;
    threshold = 0;
    bg_number = 0;
    //if(global_mv_modeling)
    //{
    // int sum_global_mv=0;
    // int i;
    // int best_x=0;
    // int best_y=0;
    // Macroblock    *currMB;
    // for(i=0;i<32;i++)
    // {
    //  mvx[i]=0;
    //  mvy[i]=0;
    // }
    // mvx[16]=1;
    // mvy[16]=1;
    // for(j=0;j<size_y;j++){
    //  int b8_y;
    //  int b8_x;
    //  int mb_x;
    //  int mb_y;
    //  int index=j;
    //  int width=frameWidth;
    //  mb_x=index%width/16*16;
    //  mb_y=index/width/16*16;
    //  b8_y=(index/width-mb_y)/8;
    //  b8_x=(index%width-mb_x)/8;
    //  currMB    = &img->mb_data[mb_x/16+mb_y/16*frameWidth/16];
    //  if(currMB->mb_type!=I8MB || input->usefme==0){
    //    int value_x=all_mincost[mb_x/4+b8_x][mb_y/4+b8_y][0][4][1]/4;
    //    int value_y=all_mincost[mb_x/4+b8_x][mb_y/4+b8_y][0][4][2]/4;
    //    if(abs(value_x)+abs(value_y)<16)
    //    {
    //        mvx[value_x+16]++;
    //        mvy[value_y+16]++;
    //    }
    //  }
    // }
    // for(i=0;i<32;i++)
    // {
    //  if(mvx[i]>mvx[best_x])
    //    best_x=i;
    //  if(mvy[i]>mvy[best_y])
    //    best_y=i;
    // }
    // global_x=best_x-16;
    // global_y=best_y-16;

    // printf("global_mv,global_x=%d,global_y=%d\n",global_x,global_y);
    //}
    if (input->bg_model_method <= 1) {
        //////////////////FILE * LIST///////////////////
        memcpy(imageData, imageData_next, sizeof(byte) * size_frame_int);
        memcpy(imageData_next, frame, sizeof(byte) * size_frame_int);

        if (input->bg_model_method == 1) {
            for (j = 0; j < size_frame_int; j++) {
                int p_x;
                int p_y;
                int pos;
                if (j < size_y) {
                    int index = j;
                    int width = frameWidth;
                    p_x = index % width + global_x;
                    if (p_x >= width) {
                        p_x = width - 1;
                    }
                    if (p_x < 0) {
                        p_x = 0;
                    }
                    p_y = index / width + global_y;
                    if (p_y >= frameHeight) {
                        p_y = frameHeight - 1;
                    }
                    if (p_y < 0) {
                        p_y = 0;
                    }
                    pos = p_y * width + p_x;
                } else if ((j - size_y) < size_y / 4) {
                    int index = j - size_y;
                    int width = frameWidth / 2;
                    p_x = index % width + global_x / 2;
                    if (p_x >= width) {
                        p_x = width - 1;
                    }
                    if (p_x < 0) {
                        p_x = 0;
                    }

                    p_y = index / width + global_y / 2;
                    if (p_y >= frameHeight / 2) {
                        p_y = frameHeight / 2 - 1;
                    }
                    if (p_y < 0) {
                        p_y = 0;
                    }

                    pos = size_y + p_y * width + p_x;
                } else {
                    int index = j - size_y - size_y / 4;
                    int width = frameWidth / 2;
                    p_x = index % width + global_x / 2;
                    if (p_x >= width) {
                        p_x = width - 1;
                    }
                    if (p_x < 0) {
                        p_x = 0;
                    }

                    p_y = index / width + global_y / 2;
                    if (p_y >= frameHeight / 2) {
                        p_y = frameHeight / 2 - 1;
                    }
                    if (p_y < 0) {
                        p_y = 0;
                    }

                    pos = size_y * 5 / 4 + p_y * width + p_x;
                }
                if ((temp = abs(imageData_next[j] - imageData[pos])) < 30 && temp != 0) {
                    threshold = (float)((diff_number * threshold + temp * temp) / ((diff_number + 1) * 1.0));
                    diff_number++;
                }

                if ((abs((int)(imageData_next[j] - (imageData_index[j] ? bgmodel_bgbuffer[pos] : bgmodel_bgbuffer_temp[pos])))) < 14 &&
                    j < size_y) {
                    bg_number++;
                }
                c = threshold_before;
                if (abs(imageData[pos] - imageData_next[j]) > c) {
                    if (weight[pos] >= input->bg_model_number / 20) {
                        start = weight[pos] * (weight[pos]);
                        bgmodel_bgbuffer[pos] = (bgmodel_bgbuffer[pos] * (imageData_index[pos]) + start * bgmodel_bgbuffer_temp[pos]) / (float)(
                                                    imageData_index[pos] + start);
                        imageData_index[pos] += start;
                    }
                    bgmodel_bgbuffer_temp[pos] = imageData_next[j];
                    weight[pos] = 1;
                } else {
                    weight[pos]++;
                    bgmodel_bgbuffer_temp[pos] = (bgmodel_bgbuffer_temp[pos] * (weight[pos] - 1) + imageData_next[j]) / (float)weight[pos];
                }

            }
        } else {
            for (j = 0; j < size_frame_int; j++) {
                if ((temp = abs(imageData_next[j] - imageData[j])) < 30 && temp != 0) {
                    threshold = (float)((diff_number * threshold + temp * temp) / ((diff_number + 1) * 1.0));
                    diff_number++;
                }
                if ((abs((int)(imageData_next[j] - bgmodel_bgbuffer[j]))) < 14 && j < size_y) {
                    bg_number++;
                }
                bgmodel_bgbuffer[j] = (bgmodel_bgbuffer[j] * weight[j] + imageData_next[j]) / (weight[j] + 1);
                if (weight[j] > 255) {
                    weight[j] = 128;
                }
                weight[j]++;
            }
        }
//    if(bg_number > size_y/2)
#if RD1501_FIX_BG
he->g_bg_number++;
#else
g_bg_number++;
#endif
        threshold_before = min(15, (int)sqrt((double)threshold)) * 2;

        //fwrite(bgmodel_bgbuffer,sizeof(unsigned char),size_frame_int,Fp1);
        //printf("threshold_before=%d",threshold_before);
    }
}
void bg_build(byte *imageData)
{
    int imgW = frameWidth;
    int imgH = frameHeight;
    int size_frame_int = imgW * imgH * 3 / 2 ;
    int start, j;
    float temp;
    if (input->bg_model_method == 1) {
        for (j = 0; j < size_frame_int; j++) {
            temp = bgmodel_bgbuffer[j];
            start = weight[j] * (weight[j]);
            if (start == 0 && imageData_index[j] == 0) {
                bgmodel_bgbuffer[j] = imageData_next[j];
            } else {
                bgmodel_bgbuffer[j] = (bgmodel_bgbuffer[j] * (imageData_index[j]) +
                                       start * bgmodel_bgbuffer_temp[j]) / (float)(imageData_index[j] + start);
            }
            if (bgmodel_bgbuffer[j] + 0.5 < ((1 << input->sample_bit_depth) - 1)) {
                imageData[j] = (byte)(bgmodel_bgbuffer[j] + 0.5);
            } else {
                imageData[j] = (1 << input->sample_bit_depth) - 1;
            }
            bgmodel_bgbuffer[j] = temp;
        }
    } else if (input->bg_model_method == 0) {
        for (j = 0; j < size_frame_int; j++) {
            if (bgmodel_bgbuffer[j] + 0.5 < ((1 << input->sample_bit_depth) - 1)) {
                imageData[j] = (byte)(bgmodel_bgbuffer[j] + 0.5);
            } else {
                imageData[j] = (1 << input->sample_bit_depth) - 1;
            }
        }
    } else if (input->bg_model_method == 2) {
        for (j = 0; j < size_frame_int; j++) {
#if RD1501_FIX_BG
			imageData[j] = ((u_bgmodel_bgbuffer[j] * he->g_bg_number) + imageData[j] + (he->g_bg_number + 1) / 2) / (he->g_bg_number + 1);
#else
			imageData[j] = ((u_bgmodel_bgbuffer[j] * g_bg_number) + imageData[j] + (g_bg_number + 1) / 2) / (g_bg_number + 1);
#endif
        }
    } else {
        for (j = 0; j < size_frame_int; j++) {
            //assert(u_bgmodel_bgbuffer[j]!=0);
            if (u_bgmodel_bgbuffer[j] == 0) {
                u_bgmodel_bgbuffer[j] = imageData[j];
            }
            imageData[j] = u_bgmodel_bgbuffer[j];
        }

    }
}

