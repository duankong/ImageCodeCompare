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

#ifndef _BIT_STREAM_H

#define _BIT_STREAM_H

#include <stdio.h>

#define SVA_START_CODE_EMULATION

#ifdef SVA_START_CODE_EMULATION

#define SVA_STREAM_BUF_SIZE 1024 //must large than 3
typedef struct {
    FILE *f;
    unsigned char buf[SVA_STREAM_BUF_SIZE];
    unsigned int  uPreBytes;
    int iBytePosition;
    int iBitOffset;
    int iNumOfStuffBits;
    int iBitsCount;
} OutputStream;
int write_start_code(OutputStream *p, unsigned char code);
extern OutputStream *pORABS;
#endif

void CloseBitStreamFile();
void OpenBitStreamFile(char *Filename);
int  WriteSequenceHeader();
int  WriteSequenceDisplayExtension();
#if AVS2_HDR_HLS
int  WriteMasteringDisplayContentMetadataExtension();
#endif
int  WriteUserData(char *userdata);
int  WriteSequenceEnd();
int  WriteVideoEditCode();
void WriteBitstreamtoFile();
void WriteSlicetoFile();
int  WriteCopyrightExtension();
int  WriteCameraParametersExtension();
int  WriteScalableExtension();

#if PicExtensionData
int picture_display_extension(Bitstream *bitstream);
int picture_copyright_extension(Bitstream *bitstream);
int picture_cameraparameters_extension();
#endif
#endif
