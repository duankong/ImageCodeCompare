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
* File name: vlc.h
* Function: header for (CA)VLC coding functions
*
*************************************************************************************
*/


#ifndef _VLC_H_
#define _VLC_H_

int   se_v(char *tracestring);
int   ue_v(char *tracestring);

int   u_v(int LenInBits, char *tracestring);


// UVLC mapping
void  linfo_ue(int len, int info, int *value1, int *dummy);
void  linfo_se(int len, int info, int *value1, int *dummy);

void  linfo_cbp_intra(int len, int info, int *cbp, int *dummy);
void  linfo_cbp_inter(int len, int info, int *cbp, int *dummy);

int   readSyntaxElement_VLC(SyntaxElement *sym);
int   GetVLCSymbol(unsigned char buffer[], int totbitoffset, int *info, int bytecount);

int   readSyntaxElement_FLC(SyntaxElement *sym);
int   GetBits(unsigned char buffer[], int totbitoffset, int *info, int bytecount, int numbits);

int   get_uv(int LenInBits, char *tracestring);    // Yulj 2004.07.15  for decision of slice end.
int   GetSyntaxElement_FLC(SyntaxElement *sym);    // Yulj 2004.07.15  for decision of slice end.

#endif

