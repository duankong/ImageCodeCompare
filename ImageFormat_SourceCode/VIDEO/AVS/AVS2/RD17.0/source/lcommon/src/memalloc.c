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
* File name: memalloc.c
* Function: Memory allocation and free helper funtions
*
*************************************************************************************
*/

//#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../lcommon/inc/memalloc.h"
#include "../../lcommon/inc/commonStructures.h"

// #include "global.h"

/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void error(char *text, int code)
{
    fprintf(stderr, "%s\n", text);
    exit(code);
}

/*
*************************************************************************
* Function:Allocate 1D memory array -> int array1D[num]
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/
int get_mem1Dint(int **array1D, int num)
{
    if ((*array1D      = (int *) calloc(num,        sizeof(int))) == NULL) {
        no_mem_exit("get_mem1Dint: array1D");
    }
    return num * sizeof(int);
}
/*
*************************************************************************
* Function:free 1D memory array which was alocated with get_mem1Dint()
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void free_mem1Dint(int *array1D)
{
    if (array1D) {
        free(array1D);
    } else {
        error("free_mem1Dint: trying to free unused memory", 100);
    }
}
/*
*************************************************************************
* Function:Allocate 1D memory array -> byte array2D[num]
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/
int get_mem1D(byte **array1D, int num)
{
    if ((*array1D      = (byte *) calloc(num,        sizeof(byte))) == NULL) {
        no_mem_exit("get_mem1Dint: array1D");
    }
    return num * sizeof(byte);
}
/*
*************************************************************************
* Function:free 1D memory array which was alocated with get_mem1D()
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void free_mem1D(byte *array1D)
{
    if (array1D) {
        free(array1D);
    } else {
        error("free_mem1D: trying to free unused memory", 100);
    }
}
/*
*************************************************************************
* Function:Allocate 2D memory array -> unsigned char array2D[rows][columns]
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/

int get_mem2D(byte ***array2D, int rows, int columns)
{
    int i;

    if ((*array2D      = (byte **) calloc(rows,        sizeof(byte *))) == NULL) {
        no_mem_exit("get_mem2D: array2D");
    }
    if (((*array2D) [0] = (byte *) calloc(columns * rows, sizeof(byte))) == NULL) {
        no_mem_exit("get_mem2D: array2D");
    }

    for (i = 1; i < rows; i++) {
        (*array2D) [i] = (*array2D) [i - 1] + columns ;
    }
    return rows * columns * sizeof(byte);
}

/*
*************************************************************************
* Function:Allocate 2D memory array -> int array2D[rows][columns]
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/

int get_mem2Dint(int ***array2D, int rows, int columns)
{
    int i;

    if ((*array2D      = (int **) calloc(rows,        sizeof(int *))) == NULL) {
        no_mem_exit("get_mem2Dint: array2D");
    }
    if (((*array2D) [0] = (int *) calloc(rows * columns, sizeof(int))) == NULL) {
        no_mem_exit("get_mem2Dint: array2D");
    }

    for (i = 1 ; i < rows ; i++) {
        (*array2D) [i] = (*array2D) [i - 1] + columns  ;
    }
    return rows * columns * sizeof(int);
}
/*
*************************************************************************
* Function:Allocate 3D memory array -> unsigned char array3D[frames][rows][columns]
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/

int get_mem3D(byte ** **array3D, int frames, int rows, int columns)
{
    int  j;

    if (((*array3D) = (byte ** *) calloc(frames, sizeof(byte **))) == NULL) {
        no_mem_exit("get_mem3D: array3D");
    }

    for (j = 0; j < frames; j++) {
        get_mem2D((*array3D) + j, rows, columns) ;
    }

    return frames * rows * columns * sizeof(byte);
}

/*
*************************************************************************
* Function:Allocate 3D memory array -> int array3D[frames][rows][columns]
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/

int get_mem3Dint(int ** **array3D, int frames, int rows, int columns)
{
    int  j;

    if (((*array3D) = (int ** *) calloc(frames, sizeof(int **))) == NULL) {
        no_mem_exit("get_mem3Dint: array3D");
    }

    for (j = 0; j < frames; j++) {
        get_mem2Dint((*array3D) + j, rows, columns) ;
    }

    return frames * rows * columns * sizeof(int);
}

/*
*************************************************************************
* Function:Allocate 4D memory array -> int array3D[frames][rows][columns][component]
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/

int get_mem4Dint(int ** ***array4D, int idx, int frames, int rows, int columns)
{
    int  j;

    if (((*array4D) = (int ** **) calloc(idx, sizeof(int **))) == NULL) {
        no_mem_exit("get_mem4Dint: array4D");
    }

    for (j = 0; j < idx; j++) {
        get_mem3Dint((*array4D) + j, frames, rows, columns) ;
    }

    return idx * frames * rows * columns * sizeof(int);
}
/*
*************************************************************************
* Function:
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/
int get_mem3DSAOstatdate(SAOStatData ** **array3D, int num_SMB, int num_comp, int num_class)
{
    int i, j;
    if (((*array3D) = (SAOStatData ** *) calloc(num_SMB, sizeof(SAOStatData **))) == NULL) {
        no_mem_exit("get_mem3DSAOstatdate: array3D");
    }
    for (i = 0; i < num_SMB; i++) {
        if ((*(*array3D + i) = (SAOStatData **) calloc(num_comp, sizeof(SAOStatData *))) == NULL) {
            no_mem_exit("get_mem2DSAOstatdate: array2D");
        }
        for (j = 0; j < num_comp; j++) {
            if ((*(*(*array3D + i) + j) = (SAOStatData *) calloc(num_class, sizeof(SAOStatData))) == NULL) {
                no_mem_exit("get_mem1DSAOstatdate: arrayD");
            }
        }
    }
    return num_SMB * num_comp * num_class * sizeof(SAOStatData);
}
/*
*************************************************************************
* Function:
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/
int get_mem2DSAOParameter(SAOBlkParam *** array2D, int num_SMB, int num_comp)
{
    int i;
    if (((*array2D) = (SAOBlkParam **) calloc(num_SMB, sizeof(SAOBlkParam *))) == NULL) {
        no_mem_exit("get_mem2DSAOBlkParam: array2D");
    }
    for (i = 0; i < num_SMB; i++) {
        if ((*(*array2D + i) = (SAOBlkParam *) calloc(num_comp, sizeof(SAOBlkParam))) == NULL) {
            no_mem_exit("get_mem1DSAOBlkParam: array1D");
        }
    }
    return num_SMB * num_comp * sizeof(SAOBlkParam);
}
/*
*************************************************************************
* Function:free 2D memory array which was alocated with get_mem2D()
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_mem2D(byte **array2D)
{
    if (array2D) {
        if (array2D[0]) {
            free(array2D[0]);
        } else {
            error("free_mem2D: trying to free unused memory", 100);
        }
        free(array2D);
    } else {
        error("free_mem2D: trying to free unused memory", 100);
    }

}

/*
*************************************************************************
* Function:free 2D memory array
which was alocated with get_mem2Dint()
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_mem2Dint(int **array2D)
{
    if (array2D) {
        if (array2D[0]) {
            free(array2D[0]);
            array2D[0] = NULL;
        } else {
            error("free_mem2D: trying to free unused memory", 100);
        }
        free(array2D);
    } else {
        error("free_mem2D: trying to free unused memory", 100);
    }

}
/*
*************************************************************************
* Function:free 3D memory array
which was alocated with get_mem3D()
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_mem3D(byte ***array3D, int frames)
{
    int i;

    if (array3D) {
        for (i = 0; i < frames; i++) {
            free_mem2D(array3D[i]);
        }

        free(array3D);
    } else {
        error("free_mem3D: trying to free unused memory", 100);
    }
}

/*
*************************************************************************
* Function:free 3D memory array
which was alocated with get_mem3Dint()
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_mem3Dint(int ***array3D, int frames)
{
    int i;

    if (array3D) {
        for (i = 0; i < frames; i++) {
            free_mem2Dint(array3D[i]);
        }

        free(array3D);
    } else {
        error("free_mem3D: trying to free unused memory", 100);
    }
}

/*
*************************************************************************
* Function:free 4D memory array
which was alocated with get_mem4Dint()
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_mem4Dint(int ** **array4D, int idx, int frames)
{
    int  j;

    if (array4D) {
        for (j = 0; j < idx; j++) {
            free_mem3Dint(array4D[j], frames) ;
        }

        free(array4D);
    } else {
        error("free_mem4D: trying to free unused memory", 100);
    }

}
/*
*************************************************************************
* Function:
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/
void free_mem3DSAOstatdate(SAOStatData *** array3D, int num_SMB, int num_comp)
{
    int i, j;
    if (array3D) {
        for (i = 0; i < num_SMB; i++) {
            if (array3D[i]) {
                for (j = 0; j < num_comp; j++) {
                    if (array3D[i][j]) {
                        free(array3D[i][j]);
                    }
                }
                free(array3D[i]);
            }
        }
        free(array3D);
    }
}
/*
*************************************************************************
* Function:
* Input:
* Output:memory size in bytes
* Return:
* Attention:
*************************************************************************
*/
void free_mem2DSAOParameter(SAOBlkParam **array2D, int num_SMB)
{
    int i;
    if (array2D) {
        for (i = 0; i < num_SMB; i++) {
            if (array2D[i]) {
                free(array2D[i]);
            }
        }
        /* if (array2D[0])
         {
          free(array2D[0]);
         }*/

    }
    free(array2D);
}
/*
*************************************************************************
* Function:Exit program if memory allocation failed (using error())
* Input: where
string indicating which memory allocation failed
* Output:
* Return:
* Attention:
*************************************************************************
*/

void no_mem_exit(char *where)
{
    snprintf(hc->errortext, ET_SIZE, "Could not allocate memory: %s", where);
    error(hc->errortext, 100);
}


void free_frame_t(avs2_frame_t *currfref)
{
    int i;

    if (currfref) {
        for (i = 0; i < 3; i++) {
            if (currfref->referenceFrame[i]) {
                free_mem2D(currfref->referenceFrame[i]);
                currfref->referenceFrame[i] = NULL;
            }
        }
        if (currfref->refbuf) {
            free_mem2Dint(currfref->refbuf);
            currfref->refbuf = NULL;
        }
        if (currfref->mvbuf) {
            free_mem3Dint(currfref->mvbuf, img->height / MIN_BLOCK_SIZE);
            currfref->mvbuf = NULL;
        }
        if (currfref->oneForthRefY) {
            free_mem2D(currfref->oneForthRefY);
            currfref->oneForthRefY = NULL;
        }
        free(currfref);
        currfref = NULL;
    }
}


void init_frame_t(avs2_frame_t *currfref)
{
    int i = 0;
    memset(currfref, 0, sizeof(avs2_frame_t));

    currfref->imgcoi_ref          = -257;
    currfref->is_output           = -1;
    currfref->refered_by_others   = -1;
    currfref->imgtr_fwRefDistance = -256;
    for (i = 0; i < 3; i++) {
        if (i == 0) {
            get_mem2D(&currfref->referenceFrame[i], img->height, img->width);
        } else {
            get_mem2D(&currfref->referenceFrame[i], img->height_cr, img->width_cr);
        }
    }
    currfref->ref = currfref->referenceFrame;
    get_mem3Dint(&currfref->mvbuf, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE, 2);
    get_mem2Dint(&currfref->refbuf, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);
    get_mem2D(&(currfref->oneForthRefY), (img->height + 2 * IMG_PAD_SIZE) * 4, (img->width + 2 * IMG_PAD_SIZE) * 4);

    for (i = 0; i < NUM_SAO_COMPONENTS; i++) {
        currfref->saorate[i] = 0;
    }
    memset(currfref->ref_poc, 0, sizeof(currfref->ref_poc));
}

