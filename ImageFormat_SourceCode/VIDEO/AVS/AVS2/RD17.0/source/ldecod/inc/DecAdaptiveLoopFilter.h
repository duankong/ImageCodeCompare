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

#ifndef DEC_ADAPTIVE_LOOP_FILTER
#define DEC_ADAPTIVE_LOOP_FILTER
#include "../../lcommon/inc/ComAdaptiveLoopFilter.h"
#include "../../lcommon/inc/commonStructures.h"
#include "../../ldecod/inc/global.h"

typedef struct {
    int **m_filterCoeffSym;
    int m_varIndTab[NO_VAR_BINS];
    byte **m_varImg;
    Boolean **m_AlfLCUEnabled;
    ALFParam **m_alfPictureParam;
} DecALFVar;
extern DecALFVar *Dec_ALF;

void CreateAlfGlobalBuffer();
void ReleaseAlfGlobalBuffer();

void ALFProcess_dec(ALFParam **alfParam, ImageParameters *img, byte *imgY_alf_Dec, byte **imgUV_alf_Dec,
                    int sample_bit_depth);
void filterOneCTB(byte *pRest, byte *pDec, int stride, int compIdx, ALFParam *alfParam, int ctuYPos, int ctuHeight,
                  int ctuXPos, int ctuWidth,
                  Boolean isAboveAvail, Boolean isBelowAvail, Boolean isLeftAvail, Boolean isRightAvail, Boolean isAboveLeftAvail,
                  Boolean isAboveRightAvail, int sample_bit_depth);
void deriveLoopFilterBoundaryAvailibility(ImageParameters *img, int numLCUPicWidth, int numLCUPicHeight, int ctu,
        Boolean *isLeftAvail, Boolean *isRightAvail, Boolean *isAboveAvail, Boolean *isBelowAvail);

void deriveBoundaryAvail(int numLCUPicWidth, int numLCUPicHeight, int ctu, Boolean *isLeftAvail, Boolean *isRightAvail,
                         Boolean *isAboveAvail, Boolean *isBelowAvail, Boolean *isAboveLeftAvail, Boolean *isAboveRightAvail);

void AllocateAlfPar(ALFParam **alf_par, int cID);
void freeAlfPar(ALFParam *alf_par, int cID);
void allocateAlfAPS(ALF_APS *pAPS);
void freeAlfAPS(ALF_APS *pAPS);
void Read_ALF_param(char *buf, int startcodepos, int length);
#endif
