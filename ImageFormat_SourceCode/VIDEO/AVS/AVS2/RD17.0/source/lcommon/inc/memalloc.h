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
* File name: memalloc.h
* Function: Memory allocation and free helper funtions
*
*************************************************************************************
*/


#ifndef _MEMALLOC_H_
#define _MEMALLOC_H_

#include "commonVariables.h"

void error(char *text, int code);

int get_mem1Dint(int **array1D, int num);
void free_mem1Dint(int *array1D);
int get_mem1D(byte **array1D, int num);
void free_mem1D(byte *array1D);
int  get_mem2D(byte ***array2D, int rows, int columns);
int  get_mem2Dint(int ***array2D, int rows, int columns);
int  get_mem3D(byte ** **array2D, int frames, int rows, int columns);
int  get_mem3Dint(int ** **array3D, int frames, int rows, int columns);
int  get_mem4Dint(int ** ***array4D, int idx, int frames, int rows, int columns);

void free_mem2D(byte **array2D);
void free_mem2Dint(int **array2D);
void free_mem3D(byte ***array2D, int frames);
void free_mem3Dint(int ***array3D, int frames);
void free_mem4Dint(int ** **array4D, int idx, int frames);

void no_mem_exit(char *where);

int get_mem3DSAOstatdate(SAOStatData ** **array3D, int num_SMB, int num_comp, int num_class);
int get_mem2DSAOParameter(SAOBlkParam *** array2D, int num_SMB, int num_comp);
void free_mem3DSAOstatdate(SAOStatData *** array3D, int num_SMB, int num_comp);
void free_mem2DSAOParameter(SAOBlkParam **array2D, int num_SMB);

void init_frame_t(avs2_frame_t *currfref);
void free_frame_t(avs2_frame_t *currfref);
#endif
