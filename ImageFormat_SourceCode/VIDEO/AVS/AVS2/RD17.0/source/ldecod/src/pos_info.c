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

#define TRACELENGTH 200
#define MAX_INFO_LENGTH 100

Boolean flag = FALSE;

typedef enum {
    disappear = 0,
    temporal
} SkipMode;

typedef struct Rectangle {
    int rectID;
    int x;
    int y;            //0: total_write_num
    int width;        //0: old_rect_num
    int height;   //0: total
} Rectangle;

typedef struct Node {
    int frame_num;
    int maxID;
    struct Rectangle *posInfoBuffer;
    struct Node *pre;
    struct Node *next;
} node;

int oriBitsUsed;
int total_frame;
struct Node *head;
struct Node *rear;
static FILE *fout;    // the output position data file

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

void free_buffer(int condition)
{
    struct Node *tmp, *dummy;
    if (condition == 0) {
        dummy = rear;
    } else {
        dummy = rear->pre;
    }
    while (head->next != dummy) {
        tmp = head->next;
        head->next = tmp->next;
        tmp->next->pre = head;
        free(tmp->posInfoBuffer);
        free(tmp);
    }
}

void write_buff2file(int condition)
{
    int j;
    struct Node *p;
    struct Rectangle *r;

    if (head->next->next == NULL) {
        return;
    }

    if (condition == 0) {
        p = head->next;
    } else {
        p = head->next->next;
    }
    r = p->posInfoBuffer;

    while (p != rear) {
        fprintf(fout, "%3d %3d", p->frame_num, r[0].height);
        //fprintf(fout, "%d\t%d\t", p->frame_num, r[0].height);
        for (j = 1; j <= r[0].height; j++) {
            fprintf(fout, "  %3d %4d %4d %4d %4d", r[j].rectID, r[j].x, r[j].y, r[j].width, r[j].height);
            //fprintf(fout, "%d\t%d\t%d\t%d\t%d\t", r[j].rectID, r[j].x, r[j].y, r[j].width, r[j].height);
        }
        fprintf(fout, "\n");
        p = p->next;
        r = p->posInfoBuffer;
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
void OpenPosFile(char *outFilename)
{
    if (flag) {
        return;
    } else {
        flag = TRUE;
    }

    if (input->ROI_Coding == 0) {
        return;
    }
    if ((fout = fopen(outFilename, "w")) == NULL) {
        printf("Fatal: cannot open position information file '%s', exit (-1)\n", outFilename);
        exit(-1);
    }
    head = (node *) malloc(sizeof(node));
    rear = (node *) malloc(sizeof(node));
    head->posInfoBuffer = NULL;
    head->pre = NULL;
    head->next = rear;
    rear->posInfoBuffer = NULL;
    rear->pre = head;
    rear->next = NULL;
    total_frame = -1;
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
    write_buff2file(1);
    if (fclose(fout)) {
        printf("Fatal: cannot close position information file, exit (-1)\n");
        exit(-1);
    }
    free_buffer(0);
    free(head);
    free(rear);
}

void decI_Slice()
{
    int i;
    char str[TRACELENGTH];
    struct Node *tmp;
    struct Rectangle *p;

    write_buff2file(1);
    free_buffer(img->tr);

    // Add a node
    if (img->tr == 0) {
        tmp = (node *) malloc(sizeof(node));
        tmp->frame_num = img->tr;
        tmp->maxID = 0;
        tmp->next = head->next;
        tmp->pre = head;
        head->next->pre = tmp;
        head->next = tmp;

        p = (Rectangle *) calloc(MAX_INFO_LENGTH, sizeof(Rectangle));
        tmp->posInfoBuffer = p;
    } else {
        tmp = (node *) malloc(sizeof(node));
        tmp->frame_num = img->tr;
        tmp->next = rear;
        tmp->pre = rear->pre;
        rear->pre->next = tmp;
        rear->pre = tmp;
        tmp->maxID = 0;

        p = (Rectangle *) calloc(MAX_INFO_LENGTH, sizeof(Rectangle));
        tmp->posInfoBuffer = p;
    }

    //oriBitsUsed = CeilLog2Abs(max(/*416*//*1280*/1600, 0)-1);
    oriBitsUsed = CeilLog2Abs(max(img->width, img->height) - 1);
    p[0].y = u_v(8, "PH: total_coded_num");
    p[0].rectID = 0;
    for (i = 1; i <= p[0].y; i++) {
        snprintf(str, TRACELENGTH, "PH: rect%03d_index", i);
        p[i].rectID = ue_v(str) + p[i - 1].rectID + 1;
        tmp->maxID = p[i].rectID;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        p[i].x = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        p[i].y = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        p[i].width = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        p[i].height = u_v(oriBitsUsed, str);
    }
    p[0].height = p[0].y;
    if (img->tr == 0) {
        write_buff2file(0);
    }
}

void decP_Slice()
{
    int i, j = 0;
    int skip, mode;
    int total = 0;
    char str[TRACELENGTH];
    struct Node *tmp;
    struct Rectangle *p, *ref;

    write_buff2file(1);
    free_buffer(1);

    // Add a node
    tmp = (node *) malloc(sizeof(node));
    tmp->frame_num = img->tr;
    tmp->next = rear;
    tmp->pre = rear->pre;
    rear->pre->next = tmp;
    rear->pre = tmp;
    tmp->maxID = tmp->pre->maxID;

    p = (Rectangle *) calloc(MAX_INFO_LENGTH, sizeof(Rectangle));
    tmp->posInfoBuffer = p;
    ref = head->next->posInfoBuffer;

    p[0].y = u_v(8, "PH: total_coded_num");
    if (p[0].y != 0) {
        p[0].width = u_v(8, "PH: old_rect_num");
    }
    for (i = 1; i <= p[0].width; i++) {
        skip = ue_v("PH: pos_skip_run");
        while (skip > 0) {
            --skip;
            mode = u_v(1, "PH: skip_rect_mode");
            switch (mode) {
            case disappear:
                ++j;
                break;
            case temporal:
                p[++total].rectID = ref[++j].rectID;
                p[total].x = ref[j].x;
                p[total].y = ref[j].y;
                p[total].width = ref[j].width;
                p[total].height = ref[j].height;
                break;
            default:
                printf("reserved skip_mode code %d\n", mode);
                break;
            }
        }
        p[++total].rectID = ref[++j].rectID;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        p[total].x = se_v(str) + ref[j].x;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        p[total].y = se_v(str) + ref[j].y;
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        p[total].width = se_v(str) + ref[j].width;
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        p[total].height = se_v(str) + ref[j].height;
    }

    skip = ue_v("PH: pos_skip_run");
    while (skip > 0) {
        --skip;
        mode = u_v(1, "PH: skip_rect_mode");
        switch (mode) {
        case disappear:
            ++j;
            break;
        case temporal:
            p[++total].rectID = ref[++j].rectID;
            p[total].x = ref[j].x;
            p[total].y = ref[j].y;
            p[total].width = ref[j].width;
            p[total].height = ref[j].height;
            break;
        default:
            printf("reserved skip_mode code %d\n", mode);
            break;
        }
    }

    for (; i <= p[0].y; i++) {
        p[++total].rectID = ++tmp->maxID;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        p[total].x = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        p[total].y = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        p[total].width = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        p[total].height = u_v(oriBitsUsed, str);
    }
    p[0].height = total;
}

void decB_Slice()
{
    int i, j = 0;
    int skip, mode;
    int total = 0;
    char str[TRACELENGTH];
    struct Node *tmp, *pn = head;
    struct Rectangle *p, *ref;

    // Add a node
    tmp = (node *) malloc(sizeof(node));
    tmp->frame_num = img->tr;
    while (pn->next->frame_num < tmp->frame_num) {
        pn = pn->next;
    }
    tmp->next = pn->next;
    tmp->pre = pn;
    pn->next->pre = tmp;
    pn->next = tmp;
    tmp->maxID = tmp->pre->maxID;

    p = (Rectangle *) calloc(MAX_INFO_LENGTH, sizeof(Rectangle));
    tmp->posInfoBuffer = p;
    ref = tmp->pre->posInfoBuffer;

    p[0].y = u_v(8, "PH: total_coded_num");
    if (p[0].y != 0) {
        p[0].width = u_v(8, "PH: old_rect_num");
    }
    for (i = 1; i <= p[0].width; i++) {
        skip = ue_v("PH: pos_skip_run");
        while (skip > 0) {
            --skip;
            mode = u_v(1, "PH: skip_rect_mode");
            switch (mode) {
            case disappear:
                ++j;
                break;
            case temporal:
                p[++total].rectID = ref[++j].rectID;
                p[total].x = ref[j].x;
                p[total].y = ref[j].y;
                p[total].width = ref[j].width;
                p[total].height = ref[j].height;
                break;
            default:
                printf("reserved skip_mode code %d\n", mode);
                break;
            }
        }
        p[++total].rectID = ref[++j].rectID;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        p[total].x = se_v(str) + ref[j].x;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        p[total].y = se_v(str) + ref[j].y;
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        p[total].width = se_v(str) + ref[j].width;
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        p[total].height = se_v(str) + ref[j].height;
    }

    skip = ue_v("PH: pos_skip_run");
    while (skip > 0) {
        --skip;
        mode = u_v(1, "PH: skip_rect_mode");
        switch (mode) {
        case disappear:
            ++j;
            break;
        case temporal:
            p[++total].rectID = ref[++j].rectID;
            p[total].x = ref[j].x;
            p[total].y = ref[j].y;
            p[total].width = ref[j].width;
            p[total].height = ref[j].height;
            break;
        default:
            printf("reserved skip_mode code %d\n", mode);
            break;
        }
    }

    for (; i <= p[0].y; i++) {
        p[++total].rectID = ++tmp->maxID;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        p[total].x = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        p[total].y = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        p[total].width = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        p[total].height = u_v(oriBitsUsed, str);
    }
    p[0].height = total;
}

void decDirectI_Slice()
{
    int i;
    char str[TRACELENGTH];
    struct Node *tmp, *pn = head;
    struct Rectangle *p;

    ++total_frame;

    // Add a node
    if (img->tr == 0) {
        tmp = (node *) malloc(sizeof(node));
        tmp->frame_num = img->tr;
        tmp->maxID = 0;
        tmp->next = head->next;
        head->next = tmp;
        tmp->next->pre = tmp;
        tmp->pre = head;

        p = (Rectangle *) calloc(MAX_INFO_LENGTH, sizeof(Rectangle));
        tmp->posInfoBuffer = p;
    } else {
        tmp = (node *) malloc(sizeof(node));
        tmp->frame_num = img->tr;

        tmp = (node *) malloc(sizeof(node));
        tmp->frame_num = img->tr;
        pn = head;
        while (pn->next != rear && pn->next->frame_num < tmp->frame_num) {
            pn = pn->next;
        }
        tmp->next = pn->next;
        pn->next = tmp;
        tmp->next->pre = tmp;
        tmp->pre = pn;

        p = (Rectangle *) calloc(MAX_INFO_LENGTH, sizeof(Rectangle));
        tmp->posInfoBuffer = p;
    }

    //oriBitsUsed = CeilLog2Abs(max(/*416*//*1280*/2560, 0));
    oriBitsUsed = CeilLog2Abs(max(img->width, img->height));
    p[0].y = u_v(8, "PH: total_coded_num");
    p[0].rectID = 0;
    for (i = 1; i <= p[0].y; i++) {
        snprintf(str, TRACELENGTH, "PH: rect%03d_index", i);
        p[i].rectID = ue_v(str) + p[i - 1].rectID + 1;
        tmp->maxID = p[i].rectID;
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisx", i);
        p[i].x = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_axisy", i);
        p[i].y = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_width", i);
        p[i].width = u_v(oriBitsUsed, str);
        snprintf(str, TRACELENGTH, "PH: rect%03d_height", i);
        p[i].height = u_v(oriBitsUsed, str);
    }
    p[0].height = p[0].y;
    if (img->tr == 0) {
        write_buff2file(0);
    } else if (total_frame % 8 == 0) {
        write_buff2file(1);
        free_buffer(img->tr);
    }
}

void roi_parameters_extension()
{
    switch (img->type) {
    case I_IMG:
        decI_Slice();
        break;
    case P_IMG:
    case F_IMG:
        decP_Slice();
        break;
    case B_IMG:
        decB_Slice();
        break;
    }
}
