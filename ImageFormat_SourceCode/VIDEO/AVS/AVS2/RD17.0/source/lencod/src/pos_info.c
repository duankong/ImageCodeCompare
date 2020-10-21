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

#include <stdlib.h>
#include <assert.h>

#include "global.h"
#include "pos_info.h"
#include "vlc.h"
#include "bitstream.h"

#define TRACELENGTH 200
#define MAXINFOLENGTH 100

typedef enum {
    disappear = 0,
    temporal
} SkipMode;

typedef struct {
    int rectID;
    int x;            //0: frame_no
    int y;            //0: total_num
    int width;
    int height;
} Rectangle;

typedef struct {
    int skip;
    int para1;        //0: old_num
    int para2;        //0: total_write_num
    int para3;        //0: pre_old_num
    int para4;
    Rectangle *ptr;
    unsigned int flag[4];
    int len;
} Coding;

Rectangle **posInfoBuffer;
Coding bitsBuff[MAXINFOLENGTH];
int read_frame_num;
int buffSize;
int oriBitsUsed;
int frameNumOfP;

static FILE *fin;       // the position data file

int hierarchical_order[3][8] = {{0, 0, 0, 2, 0, 4}, {0, 0, 0, 2, 0, 4, 4}, {0, 0, 0, 2, 0, 4, 4, 6}};

void writeSkipMode(int code, Coding *buff)
{
    ++buff->skip;
    buff->flag[buff->len / 32] <<= 1;
    buff->flag[buff->len / 32] += code;
    ++buff->len;
}

int CeilLog2Abs(int val)
{
    int tmp;
    int ret = 0;

    if (val < 0) {
        tmp = -val;
    } else {
        tmp = val;
    }

    while (tmp != 0) {
        tmp >>= 1;
        ret++;
    }
    return ret;
}

free_buffer(int end)
{
    int i;

    for (i = 0; i < end; i++) {
        if (posInfoBuffer[i] != NULL) {
            free(posInfoBuffer[i]);
        }
        posInfoBuffer[i] = NULL;
    }
}

/*!
 ********************************************************************************************
 * \brief
 *    Opens the position data file
 *
 * \param Filename
 *    The filename of the file to be opened
 *
 * \return
 *    none.  Function terminates the program in case of an error
 *
 ********************************************************************************************
*/
void OpenPosFile(char *inFilename)
{
    int i;

    if (input->ROI_Coding == 0) {
        return;
    }
    if ((fin = fopen(inFilename, "r")) == NULL) {
        printf("Fatal: cannot open position information file '%s', exit (-1)\n", inFilename);
        exit(-1);
    }

    buffSize = input->successive_Bframe + 2;
    read_frame_num = -1;
    posInfoBuffer = (Rectangle **) calloc(buffSize, sizeof(Rectangle *));
    for (i = 0; i < buffSize; i++) {
        posInfoBuffer[i] = NULL;
    }
    oriBitsUsed = 4 + CeilLog2Abs((max(input->img_height, input->img_width) >> 4) - 1);
    //oriBitsUsed = 4 + CeilLog2Abs((max(/*416*//*1280*/1600, 0) >> 4) - 1);
}

/*!
 ********************************************************************************************
 * \brief
 *    Closes the position data file
 *
 * \return
 *    none.  Funtion trerminates the program in case of an error
 ********************************************************************************************
*/
void ClosePosFile()
{
    if (input->ROI_Coding == 0) {
        return;
    }
    if (fclose(fin)) {
        printf("Fatal: cannot close position information file, exit (-1)\n");
        exit(-1);
    }
    free_buffer(buffSize);
    free(posInfoBuffer);
}

void ReadPosInfo(int frame_num)
{
    int i, j;
    int num, frame;

    if (frame_num == 0) {
        free_buffer(buffSize);
        fscanf(fin, "%d %d", &frame, &num);
        posInfoBuffer[buffSize - 1] = (Rectangle *) calloc(num + 1, sizeof(Rectangle));
        posInfoBuffer[buffSize - 1][0].x = frame;
        posInfoBuffer[buffSize - 1][0].y = num;
        ++read_frame_num;
        for (j = 1; j <= num; j++) {
            fscanf(fin, "%d %d %d %d %d", &posInfoBuffer[buffSize - 1][j].rectID, &posInfoBuffer[buffSize - 1][j].x, \
                   &posInfoBuffer[buffSize - 1][j].y, &posInfoBuffer[buffSize - 1][j].width, &posInfoBuffer[buffSize - 1][j].height);
        }
    } else {
        free_buffer(buffSize - 1);
        posInfoBuffer[0] = posInfoBuffer[buffSize - 1];
        posInfoBuffer[buffSize - 1] = NULL;

        read_frame_num += buffSize - 1;
        for (i = 1; i < buffSize; i++) {
            fscanf(fin, "%d %d", &frame, &num);
            if (feof(fin)) {
                break;
            }
            posInfoBuffer[i] = (Rectangle *) calloc(num + 1, sizeof(Rectangle));
            posInfoBuffer[i][0].x = frame;
            posInfoBuffer[i][0].y = num;
            for (j = 1; j <= num; j++) {
                fscanf(fin, "%d %d %d %d %d", &posInfoBuffer[i][j].rectID, &posInfoBuffer[i][j].x, \
                       &posInfoBuffer[i][j].y, &posInfoBuffer[i][j].width, &posInfoBuffer[i][j].height);
            }
        }
    }
}

int encode_for_I(int frame_num, Bitstream *bitstream)
{
    int len = 0, ptr, i;
    char str[TRACELENGTH];

    ptr = frame_num % (buffSize - 1);
    if (ptr == 0) {
        ptr = buffSize - 1;
    }
    if (posInfoBuffer[ptr] == NULL) {
        printf("The number of encoded frame is not accorded with the frame number of ROI data!\n");
        len += u_v(8, "PH: total_coded_num", 0, bitstream);
        return -1;
    }

    len += u_v(8, "PH: total_coded_num", posInfoBuffer[ptr][0].y, bitstream);

    posInfoBuffer[ptr][0].rectID = 0;
    for (i = 1; i <= posInfoBuffer[ptr][0].y; i++) {
        snprintf(str, TRACELENGTH, "PH: rect%03d_index", i);
        len += ue_v(str, posInfoBuffer[ptr][i].rectID - posInfoBuffer[ptr][i - 1].rectID - 1, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        len += u_v(oriBitsUsed, str, posInfoBuffer[ptr][i].x, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        len += u_v(oriBitsUsed, str, posInfoBuffer[ptr][i].y, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        len += u_v(oriBitsUsed, str, posInfoBuffer[ptr][i].width, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        len += u_v(oriBitsUsed, str, posInfoBuffer[ptr][i].height, bitstream);
    }

    return len;
}

int encode_for_P(int frame_num, Bitstream *bitstream)
{
    int len = 0;
    int i, j = 0, l = 1;
    char str[TRACELENGTH];
    Boolean newFlag = FALSE;
    int ptr;

    frameNumOfP = frame_num;
    ptr = frame_num % (buffSize - 1);
    if (ptr == 0) {
        ptr = buffSize - 1;
    }
    if (posInfoBuffer[ptr] == NULL) {
        printf("The number of encoded frame is not accorded with the frame number of ROI data!\n");
        len += u_v(8, "PH: total_coded_num", 0, bitstream);
        return -1;
    }

    bitsBuff[0].para1 = 0;
    bitsBuff[0].para2 = 0;
    bitsBuff[l].skip = 0;
    bitsBuff[l].len = 0;
    bitsBuff[l].flag[0] = 0;
    bitsBuff[l].flag[1] = 0;
    bitsBuff[l].flag[2] = 0;
    bitsBuff[l].flag[3] = 0;

    for (i = 1; i <= posInfoBuffer[ptr][0].y; i++) {
        // find the ID-matched Rectangle
        while (newFlag == FALSE) {
            ++j;
            if (j > posInfoBuffer[0][0].y) {
                newFlag = TRUE;
                // initialize the new buffer
                bitsBuff[++l].skip = 0;
                bitsBuff[l].len = 0;
                bitsBuff[l].flag[0] = 0;
                bitsBuff[l].flag[1] = 0;
                bitsBuff[l].flag[2] = 0;
                bitsBuff[l].flag[3] = 0;
                break;
            }
            if (posInfoBuffer[ptr][i].rectID <= posInfoBuffer[0][j].rectID) {
                break;
            }
            // disppeared rect
            writeSkipMode(disappear, &bitsBuff[l]);
        }

        if (newFlag) {
            bitsBuff[l].para1 = posInfoBuffer[ptr][i].x;
            bitsBuff[l].para2 = posInfoBuffer[ptr][i].y;
            bitsBuff[l].para3 = posInfoBuffer[ptr][i].width;
            bitsBuff[l].para4 = posInfoBuffer[ptr][i].height;
            ++bitsBuff[0].para2;
            // initialize the new buffer
            bitsBuff[++l].skip = 0;
            bitsBuff[l].len = 0;
            bitsBuff[l].flag[0] = 0;
            bitsBuff[l].flag[1] = 0;
            bitsBuff[l].flag[2] = 0;
            bitsBuff[l].flag[3] = 0;
        } else {
            bitsBuff[l].para1 = posInfoBuffer[ptr][i].x - posInfoBuffer[0][j].x;
            bitsBuff[l].para2 = posInfoBuffer[ptr][i].y - posInfoBuffer[0][j].y;
            bitsBuff[l].para3 = posInfoBuffer[ptr][i].width - posInfoBuffer[0][j].width;
            bitsBuff[l].para4 = posInfoBuffer[ptr][i].height - posInfoBuffer[0][j].height;
            // temporal skip mode
            if (bitsBuff[l].para1 == 0 && bitsBuff[l].para2 == 0 && bitsBuff[l].para3 == 0 && bitsBuff[l].para4 == 0) {
                writeSkipMode(temporal, &bitsBuff[l]);
            }
            // write difference
            else {
                ++bitsBuff[0].para1;
                ++bitsBuff[0].para2;
                bitsBuff[l].ptr = &posInfoBuffer[ptr][i];
                // initialize the new buffer
                bitsBuff[++l].skip = 0;
                bitsBuff[l].len = 0;
                bitsBuff[l].flag[0] = 0;
                bitsBuff[l].flag[1] = 0;
                bitsBuff[l].flag[2] = 0;
                bitsBuff[l].flag[3] = 0;
            }
        }
    }
    bitsBuff[++l].skip = 0;

    len += u_v(8, "PH: total_coded_num", bitsBuff[0].para2, bitstream);
    if (bitsBuff[0].para2 > 0) {
        len += u_v(8, "PH: old_rect_num", bitsBuff[0].para1, bitstream);
    }

    for (i = 1; i <= bitsBuff[0].para1; i++) {
        len += ue_v("PH: pos_skip_run", bitsBuff[i].skip, bitstream);
        if (bitsBuff[i].len > 0) {
            if (bitsBuff[i].len <= 32) {
                len += u_v(bitsBuff[i].len, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
            } else {
                len += u_v(32, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
                len += u_v(bitsBuff[i].len - 32, "PH: skip_rect_mode", bitsBuff[i].flag[1], bitstream);
            }
        }
        snprintf(str, TRACELENGTH, "PH: rect%03d_dx = %3d (org_x %3d, pred_x %3d)", i, bitsBuff[i].para1, bitsBuff[i].ptr->x,
                 bitsBuff[i].ptr->x - bitsBuff[i].para1);
        len += se_v(str, bitsBuff[i].para1, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dy = %3d (org_y %3d, pred_y %3d)", i, bitsBuff[i].para2, bitsBuff[i].ptr->y,
                 bitsBuff[i].ptr->y - bitsBuff[i].para2);
        len += se_v(str, bitsBuff[i].para2, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dw = %3d (org_w %3d, pred_w %3d)", i, bitsBuff[i].para3,
                 bitsBuff[i].ptr->width, bitsBuff[i].ptr->width - bitsBuff[i].para3);
        len += se_v(str, bitsBuff[i].para3, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dh = %3d (org_h %3d, pred_h %3d)", i, bitsBuff[i].para4,
                 bitsBuff[i].ptr->height, bitsBuff[i].ptr->height - bitsBuff[i].para4);
        len += se_v(str, bitsBuff[i].para4, bitstream);
    }

    len += ue_v("PH: pos_skip_run", bitsBuff[i].skip, bitstream);
    if (bitsBuff[i].len > 0) {
        if (bitsBuff[i].len <= 32) {
            len += u_v(bitsBuff[i].len, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
        } else {
            len += u_v(32, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
            len += u_v(bitsBuff[i].len - 32, "PH: skip_rect_mode", bitsBuff[i].flag[1], bitstream);
        }
    }
    ++i;

    for (; i <= bitsBuff[0].para2 + 1; i++) {
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para1, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para2, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para3, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para4, bitstream);
    }

    return len;
}

int encode_for_B(int frame_num, Bitstream *bitstream)
{
    int len = 0;
    int i, j = 0, l = 1;
    char str[TRACELENGTH];
    Boolean newFlag = FALSE;
    int ptr, pre;

    ptr = frame_num % (buffSize - 1);
    pre = ptr - 1;
    if (posInfoBuffer[ptr] == NULL) {
        printf("The number of encoded frame is not accorded with the frame number of ROI data!\n");
        len += u_v(8, "PH: total_coded_num", 0, bitstream);
        return -1;
    }

    bitsBuff[0].para1 = 0;
    bitsBuff[0].para2 = 0;
    bitsBuff[l].skip = 0;
    bitsBuff[l].len = 0;
    bitsBuff[l].flag[0] = 0;
    bitsBuff[l].flag[1] = 0;
    bitsBuff[l].flag[2] = 0;
    bitsBuff[l].flag[3] = 0;

    for (i = 1; i <= posInfoBuffer[ptr][0].y; i++) {
        // find the ID-matched Rectangle
        while (newFlag == FALSE) {
            ++j;
            if (j > posInfoBuffer[pre][0].y) {
                newFlag = TRUE;
                // initialize the new buffer
                bitsBuff[++l].skip = 0;
                bitsBuff[l].len = 0;
                bitsBuff[l].flag[0] = 0;
                bitsBuff[l].flag[1] = 0;
                bitsBuff[l].flag[2] = 0;
                bitsBuff[l].flag[3] = 0;
                break;
            }
            if (posInfoBuffer[ptr][i].rectID <= posInfoBuffer[pre][j].rectID) {
                break;
            }
            // disppeared rect
            writeSkipMode(disappear, &bitsBuff[l]);
        }

        if (newFlag) {
            bitsBuff[l].para1 = posInfoBuffer[ptr][i].x;
            bitsBuff[l].para2 = posInfoBuffer[ptr][i].y;
            bitsBuff[l].para3 = posInfoBuffer[ptr][i].width;
            bitsBuff[l].para4 = posInfoBuffer[ptr][i].height;
            ++bitsBuff[0].para2;
            // initialize the new buffer
            bitsBuff[++l].skip = 0;
            bitsBuff[l].len = 0;
            bitsBuff[l].flag[0] = 0;
            bitsBuff[l].flag[1] = 0;
            bitsBuff[l].flag[2] = 0;
            bitsBuff[l].flag[3] = 0;
        } else {
            bitsBuff[l].para1 = posInfoBuffer[ptr][i].x - posInfoBuffer[pre][j].x;
            bitsBuff[l].para2 = posInfoBuffer[ptr][i].y - posInfoBuffer[pre][j].y;
            bitsBuff[l].para3 = posInfoBuffer[ptr][i].width - posInfoBuffer[pre][j].width;
            bitsBuff[l].para4 = posInfoBuffer[ptr][i].height - posInfoBuffer[pre][j].height;
            // temporal skip mode
            if (bitsBuff[l].para1 == 0 && bitsBuff[l].para2 == 0 && bitsBuff[l].para3 == 0 && bitsBuff[l].para4 == 0) {
                writeSkipMode(temporal, &bitsBuff[l]);
            }
            // write difference
            else {
                ++bitsBuff[0].para1;
                ++bitsBuff[0].para2;
                bitsBuff[l].ptr = &posInfoBuffer[ptr][i];
                // initialize the new buffer
                bitsBuff[++l].skip = 0;
                bitsBuff[l].len = 0;
                bitsBuff[l].flag[0] = 0;
                bitsBuff[l].flag[1] = 0;
                bitsBuff[l].flag[2] = 0;
                bitsBuff[l].flag[3] = 0;
            }
        }
    }
    bitsBuff[++l].skip = 0;

    len += u_v(8, "PH: total_coded_num", bitsBuff[0].para2, bitstream);
    if (bitsBuff[0].para2 > 0) {
        len += u_v(8, "PH: old_rect_num", bitsBuff[0].para1, bitstream);
    }

    for (i = 1; i <= bitsBuff[0].para1; i++) {
        len += ue_v("PH: pos_skip_run", bitsBuff[i].skip, bitstream);
        if (bitsBuff[i].len > 0) {
            if (bitsBuff[i].len <= 32) {
                len += u_v(bitsBuff[i].len, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
            } else {
                len += u_v(32, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
                len += u_v(bitsBuff[i].len - 32, "PH: skip_rect_mode", bitsBuff[i].flag[1], bitstream);
            }
        }
        snprintf(str, TRACELENGTH, "PH: rect%03d_dx = %3d (org_x %3d, pred_x %3d)", i, bitsBuff[i].para1, bitsBuff[i].ptr->x,
                 bitsBuff[i].ptr->x - bitsBuff[i].para1);
        len += se_v(str, bitsBuff[i].para1, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dy = %3d (org_y %3d, pred_y %3d)", i, bitsBuff[i].para2, bitsBuff[i].ptr->y,
                 bitsBuff[i].ptr->y - bitsBuff[i].para2);
        len += se_v(str, bitsBuff[i].para2, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dw = %3d (org_w %3d, pred_w %3d)", i, bitsBuff[i].para3,
                 bitsBuff[i].ptr->width, bitsBuff[i].ptr->width - bitsBuff[i].para3);
        len += se_v(str, bitsBuff[i].para3, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dh = %3d (org_h %3d, pred_h %3d)", i, bitsBuff[i].para4,
                 bitsBuff[i].ptr->height, bitsBuff[i].ptr->height - bitsBuff[i].para4);
        len += se_v(str, bitsBuff[i].para4, bitstream);
    }

    len += ue_v("PH: pos_skip_run", bitsBuff[i].skip, bitstream);
    if (bitsBuff[i].len > 0) {
        if (bitsBuff[i].len <= 32) {
            len += u_v(bitsBuff[i].len, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
        } else {
            len += u_v(32, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
            len += u_v(bitsBuff[i].len - 32, "PH: skip_rect_mode", bitsBuff[i].flag[1], bitstream);
        }
    }
    ++i;

    for (; i <= bitsBuff[0].para2 + 1; i++) {
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para1, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para2, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para3, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para4, bitstream);
    }

    return len;
}

int encode_for_HB(int frame_num, Bitstream *bitstream)
{
    int len = 0;
    int i, j = 0, l = 1;
    char str[TRACELENGTH];
    Boolean newFlag = FALSE;
    int ptr, pre;

    if (frameNumOfP % (buffSize - 1) == 0) {
        ptr = frame_num % (buffSize - 1);
        pre = hierarchical_order[2][ptr];
    } else if (frameNumOfP % (buffSize - 1) == 6) {
        ptr = frame_num % (buffSize - 1);
        pre = hierarchical_order[0][ptr];
    } else {
        ptr = frame_num % (buffSize - 1);
        pre = hierarchical_order[1][ptr];
    }

    bitsBuff[0].para1 = 0;
    bitsBuff[0].para2 = 0;
    bitsBuff[l].skip = 0;
    bitsBuff[l].len = 0;
    bitsBuff[l].flag[0] = 0;
    bitsBuff[l].flag[1] = 0;
    bitsBuff[l].flag[2] = 0;
    bitsBuff[l].flag[3] = 0;

    for (i = 1; i <= posInfoBuffer[ptr][0].y; i++) {
        // find the ID-matched Rectangle
        while (newFlag == FALSE) {
            ++j;
            if (j > posInfoBuffer[pre][0].y) {
                newFlag = TRUE;
                // initialize the new buffer
                bitsBuff[++l].skip = 0;
                bitsBuff[l].len = 0;
                bitsBuff[l].flag[0] = 0;
                bitsBuff[l].flag[1] = 0;
                bitsBuff[l].flag[2] = 0;
                bitsBuff[l].flag[3] = 0;
                break;
            }
            if (posInfoBuffer[ptr][i].rectID <= posInfoBuffer[pre][j].rectID) {
                break;
            }
            // disppeared rect
            writeSkipMode(disappear, &bitsBuff[l]);
        }

        if (newFlag) {
            bitsBuff[l].para1 = posInfoBuffer[ptr][i].x;
            bitsBuff[l].para2 = posInfoBuffer[ptr][i].y;
            bitsBuff[l].para3 = posInfoBuffer[ptr][i].width;
            bitsBuff[l].para4 = posInfoBuffer[ptr][i].height;
            ++bitsBuff[0].para2;
            // initialize the new buffer
            bitsBuff[++l].skip = 0;
            bitsBuff[l].len = 0;
            bitsBuff[l].flag[0] = 0;
            bitsBuff[l].flag[1] = 0;
            bitsBuff[l].flag[2] = 0;
            bitsBuff[l].flag[3] = 0;
        } else {
            bitsBuff[l].para1 = posInfoBuffer[ptr][i].x - posInfoBuffer[pre][j].x;
            bitsBuff[l].para2 = posInfoBuffer[ptr][i].y - posInfoBuffer[pre][j].y;
            bitsBuff[l].para3 = posInfoBuffer[ptr][i].width - posInfoBuffer[pre][j].width;
            bitsBuff[l].para4 = posInfoBuffer[ptr][i].height - posInfoBuffer[pre][j].height;
            // temporal skip mode
            if (bitsBuff[l].para1 == 0 && bitsBuff[l].para2 == 0 && bitsBuff[l].para3 == 0 && bitsBuff[l].para4 == 0) {
                writeSkipMode(temporal, &bitsBuff[l]);
            }
            // write difference
            else {
                ++bitsBuff[0].para1;
                ++bitsBuff[0].para2;
                bitsBuff[l].ptr = &posInfoBuffer[ptr][i];
                // initialize the new buffer
                bitsBuff[++l].skip = 0;
                bitsBuff[l].len = 0;
                bitsBuff[l].flag[0] = 0;
                bitsBuff[l].flag[1] = 0;
                bitsBuff[l].flag[2] = 0;
                bitsBuff[l].flag[3] = 0;
            }
        }
    }
    bitsBuff[++l].skip = 0;

    len += u_v(8, "PH: total_coded_num", bitsBuff[0].para2, bitstream);
    if (bitsBuff[0].para2 > 0) {
        len += u_v(8, "PH: old_rect_num", bitsBuff[0].para1, bitstream);
    }

    for (i = 1; i <= bitsBuff[0].para1; i++) {
        len += ue_v("PH: pos_skip_run", bitsBuff[i].skip, bitstream);
        if (bitsBuff[i].len > 0) {
            if (bitsBuff[i].len <= 32) {
                len += u_v(bitsBuff[i].len, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
            } else {
                len += u_v(32, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
                len += u_v(bitsBuff[i].len - 32, "PH: skip_rect_mode", bitsBuff[i].flag[1], bitstream);
            }
        }
        snprintf(str, TRACELENGTH, "PH: rect%03d_dx = %3d (org_x %3d, pred_x %3d)", i, bitsBuff[i].para1, bitsBuff[i].ptr->x,
                 bitsBuff[i].ptr->x - bitsBuff[i].para1);
        len += se_v(str, bitsBuff[i].para1, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dy = %3d (org_y %3d, pred_y %3d)", i, bitsBuff[i].para2, bitsBuff[i].ptr->y,
                 bitsBuff[i].ptr->y - bitsBuff[i].para2);
        len += se_v(str, bitsBuff[i].para2, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dw = %3d (org_w %3d, pred_w %3d)", i, bitsBuff[i].para3,
                 bitsBuff[i].ptr->width, bitsBuff[i].ptr->width - bitsBuff[i].para3);
        len += se_v(str, bitsBuff[i].para3, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_dh = %3d (org_h %3d, pred_h %3d)", i, bitsBuff[i].para4,
                 bitsBuff[i].ptr->height, bitsBuff[i].ptr->height - bitsBuff[i].para4);
        len += se_v(str, bitsBuff[i].para4, bitstream);
    }

    len += ue_v("PH: pos_skip_run", bitsBuff[i].skip, bitstream);
    if (bitsBuff[i].len > 0) {
        if (bitsBuff[i].len <= 32) {
            len += u_v(bitsBuff[i].len, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
        } else {
            len += u_v(32, "PH: skip_rect_mode", bitsBuff[i].flag[0], bitstream);
            len += u_v(bitsBuff[i].len - 32, "PH: skip_rect_mode", bitsBuff[i].flag[1], bitstream);
        }
    }
    ++i;

    for (; i <= bitsBuff[0].para2 + 1; i++) {
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para1, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para2, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para3, bitstream);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        len += u_v(oriBitsUsed, str, bitsBuff[i].para4, bitstream);
    }

    return len;
}

int roi_parameters_extension(int frame_num,  Bitstream *bitstream)
{
    int  bitscount = 0;

    if (input->ROI_Coding == 0) {
        return bitscount;
    }

#if !PicExtensionData
    if (bitstream->bits_to_go != 8) {   //   hzjia 2004-08-20
        bitstream->byte_buf <<= bitstream->bits_to_go;
        bitstream->byte_buf |= (1 << (bitstream->bits_to_go - 1));

        bitstream->streamBuffer[bitstream->byte_pos++] = bitstream->byte_buf;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    } else { // cjw 20060321
        bitstream->streamBuffer[bitstream->byte_pos++] = 0x80;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    }
#endif

    bitscount += u_v(32, "Extension start code", 0x1B5, bitstream);
#if RD1501_FIX_BG
	bitscount += u_v(4, "ROI parameter extension id", 12, bitstream);
#else
    bitscount += u_v(4, "ROI parameter extension id", 15, bitstream);
#endif
    if (read_frame_num < frame_num) {
        ReadPosInfo(frame_num);
    }

    switch (img->type) {
    case INTRA_IMG:
        bitscount += encode_for_I(frame_num, bitstream);
        break;
    case INTER_IMG:
    case F_IMG:
        bitscount += encode_for_P(frame_num, bitstream);
        break;
    case B_IMG:
        if (input->Hierarchical_B == 1 && (frameNumOfP % (buffSize - 1) > 5 || frameNumOfP % (buffSize - 1) == 0)) {
            bitscount += encode_for_HB(frame_num, bitstream);
        } else {
            bitscount += encode_for_B(frame_num, bitstream);
        }
        break;
    default:
        break;
    }

#if PicExtensionData
    if (bitstream->bits_to_go != 8) {   //   hzjia 2004-08-20
        bitstream->byte_buf <<= bitstream->bits_to_go;
        bitstream->byte_buf |= (1 << (bitstream->bits_to_go - 1));

        bitstream->streamBuffer[bitstream->byte_pos++] = bitstream->byte_buf;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    } else { // cjw 20060321
        bitstream->streamBuffer[bitstream->byte_pos++] = 0x80;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    }
#endif

    return bitscount;
}
