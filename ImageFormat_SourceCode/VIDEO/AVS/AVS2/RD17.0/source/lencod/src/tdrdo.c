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

#include<stdlib.h>
#include<stdio.h>
#include<math.h>
#include"tdrdo.h"
#include <string.h>
const double tdrdoAlpha = 0.94F;

DL *CreatDistortionList(unsigned int totalframenumber, unsigned int w, unsigned int h, unsigned int blocksize,
                        unsigned int cusize)
{
    unsigned int i;
    DL *NewDL;
    int tBlockNumInHeight, tBlockNumInWidth, tBlockNumber;
    NewDL = (DL *)calloc(1, sizeof(DL));
    NewDL->TotalFrameNumber = totalframenumber;
    NewDL->FrameWidth = w;
    NewDL->FrameHeight = h;
    blocksize = blocksize < 4 ? 4 : (blocksize > 64 ? 64 : blocksize);
    NewDL->BlockSize = blocksize;
    NewDL->FrameDistortionArray = (FD *)calloc(totalframenumber, sizeof(FD));
    tBlockNumInHeight = (int)ceil(1.0 * h / blocksize);
    tBlockNumInWidth = (int)ceil(1.0 * w / blocksize);
    tBlockNumber = tBlockNumInHeight * tBlockNumInWidth;
    for (i = 0; i < totalframenumber; i++) {
        NewDL->FrameDistortionArray[i].FrameNumber = i;
        NewDL->FrameDistortionArray[i].BlockSize = blocksize;
        NewDL->FrameDistortionArray[i].CUSize = cusize;
        NewDL->FrameDistortionArray[i].TotalNumOfBlocks = tBlockNumber;
        NewDL->FrameDistortionArray[i].TotalBlockNumInHeight = tBlockNumInHeight;
        NewDL->FrameDistortionArray[i].TotalBlockNumInWidth = tBlockNumInWidth;
        NewDL->FrameDistortionArray[i].BlockDistortionArray = NULL;
        NewDL->FrameDistortionArray[i].subFrameDistortionArray = NULL;
    }
    return NewDL;
}
void DestroyDistortionList(DL *SeqD)
{
    unsigned int i;
    if (SeqD == NULL) {
        return;
    }
    for (i = 0; i < SeqD->TotalFrameNumber; i++)
        if (SeqD->FrameDistortionArray[i].BlockDistortionArray != NULL) {
            free(SeqD->FrameDistortionArray[i].BlockDistortionArray);
        }
    free(SeqD->FrameDistortionArray);
    free(SeqD);
}

void getBlockFromFrame(Block *B, Frame *F)
{
    unsigned int h;
    byte *lume = B->lume;
    byte *cr    = B->cr;
    byte *cb    = B->cb;
    byte *Y = F->Y;
    byte *U = F->U;
    byte *V = F->V;

    Y = Y + B->OriginY * F->nStrideY + B->OriginX;
    for (h = 0; h < B->BlockHeight; h++) {
        memcpy(lume, Y, B->BlockWidth * sizeof(byte));
        lume = lume + B->BlockWidth;
        Y = Y + F->nStrideY;
    }

    U = U + B->OriginY / 2 * F->nStrideC + B->OriginX / 2;
    V = V + B->OriginY / 2 * F->nStrideC + B->OriginX / 2;

    for (h = 0; h < B->BlockHeight / 2; h++) {
        memcpy(cr, U, B->BlockWidth / 2 * sizeof(byte));
        cr = cr + B->BlockWidth / 2;
        U = U + F->nStrideC;
        memcpy(cb, V, B->BlockWidth / 2 * sizeof(byte));
        cb = cb + B->BlockWidth / 2;
        V = V + F->nStrideC;
    }
}

double CalculateBlockMSE(Block A, Block B, double *pMSE, double *pMAD)
{
    int i, e;
    int blockpixel = A.BlockHeight * A.BlockWidth;
    double dSAD, dSSE;
    dSAD = dSSE = 0;
    for (i = 0; i < blockpixel; i++) {
        e = A.lume[i] - B.lume[i];
        dSAD += abs(e);
        dSSE += e * e;
    }
    *pMSE = dSSE / blockpixel;
    *pMAD = dSAD / blockpixel;
    return *pMSE;
}

void MotionDistortion(FD *currentFD, Frame *FA, Frame *FB, unsigned int searchrange)
{
    int x, y;
    unsigned int blocksize, TotalBlockNumInHeight, TotalBlockNumInWidth, nBH, nBW;
    BD *currentBD;
    int top, bottom, left, right;
    double currentMSE, MSE, candidateMSE, MAD;
    Block BA, BB;
    Block *pBA, *pBB;
    static int dlx[9] = {0, -2, -1, 0, 1, 2, 1, 0, -1};
    static int dly[9] = {0, 0, -1, -2, -1, 0, 1, 2, 1};
    static int dsx[5] = {0, -1, 0, 1, 0};
    static int dsy[5] = {0, 0, -1, 0, 1};
    int *searchpatternx, *searchpatterny;
    int patternsize;
    int cx, cy;
    int l;
    int flag9p, flag5p;
    int nextcx, nextcy;
    pBA = &BA;
    pBB = &BB;

    blocksize = currentFD->BlockSize;
    TotalBlockNumInHeight = currentFD->TotalBlockNumInHeight;
    TotalBlockNumInWidth = currentFD->TotalBlockNumInWidth;

    for (nBH = 0; nBH < TotalBlockNumInHeight; nBH++)
        for (nBW = 0; nBW < TotalBlockNumInWidth;  nBW++) {
            memset(pBA, '\0', sizeof(BA));
            memset(pBB, '\0', sizeof(BB));

            pBA->OriginX = blocksize * nBW;
            pBA->OriginY = blocksize * nBH;
            pBA->BlockHeight = blocksize * (nBH + 1) < FA->FrameHeight ? blocksize : FA->FrameHeight - blocksize * nBH;
            pBA->BlockWidth  = blocksize * (nBW + 1) < FA->FrameWidth  ? blocksize : FA->FrameWidth  - blocksize * nBW;

            currentBD = &currentFD->BlockDistortionArray[nBH * TotalBlockNumInWidth + nBW];
            currentBD->GlobalBlockNumber = nBH * TotalBlockNumInWidth + nBW;
            currentBD->BlockNumInHeight = nBH;
            currentBD->BlockNumInWidth = nBW;
            currentBD->BlockWidth = pBA->BlockWidth;
            currentBD->BlockHeight = pBA->BlockHeight;
            currentBD->OriginX = pBA->OriginX;
            currentBD->OriginY = pBA->OriginY;
            currentBD->SearchRange = searchrange;

            top     = 0 + pBA->OriginY - searchrange;
            bottom  = 0 + pBA->OriginY + searchrange;
            left    = 0 + pBA->OriginX - searchrange;
            right   = 0 + pBA->OriginX + searchrange;
            top     = max(0, min(top, (int)(FB->FrameHeight - pBA->BlockHeight)));
            bottom  = max(0, min(bottom, (int)(FB->FrameHeight - pBA->BlockHeight)));
            left    = max(0, min(left, (int)(FB->FrameWidth - pBA->BlockWidth)));
            right   = max(0, min(right, (int)(FB->FrameWidth - pBA->BlockWidth)));

            pBB->BlockHeight = pBA->BlockHeight;
            pBB->BlockWidth  = pBA->BlockWidth;
            getBlockFromFrame(pBA, FA);

            flag5p = 0;
            flag9p = 1;
            cy = pBA->OriginY;
            cx = pBA->OriginX;
            while (flag9p || flag5p) {
                candidateMSE = 1048576; // 1048576 = 1024 * 1024;
                if (flag9p) {
                    searchpatternx = dlx;
                    searchpatterny = dly;
                    patternsize = 9;
                } else if (flag5p) {
                    searchpatternx = dsx;
                    searchpatterny = dsy;
                    patternsize = 5;
                }

                for (l = 0; l < patternsize; l++) {
                    y = cy + searchpatterny[l];
                    x = cx + searchpatternx[l];
                    if (x >= left && x <= right && y >= top && y <= bottom) {
                        pBB->OriginX = x;
                        pBB->OriginY = y;
                        getBlockFromFrame(pBB, FB);
                        currentMSE = CalculateBlockMSE(BA, BB, &MSE, &MAD);
                        if (currentMSE < candidateMSE) {
                            candidateMSE = currentMSE;
                            currentBD->MSE = MSE;
                            currentBD->MAD = MAD;
                            nextcx = x;
                            nextcy = y;
                            currentBD->MVx = pBA->OriginX - pBB->OriginX;
                            currentBD->MVy = pBA->OriginY - pBB->OriginY;
                            currentBD->MVL = sqrt(1.0 * currentBD->MVx * currentBD->MVx + 1.0 * currentBD->MVy * currentBD->MVy);
                        }
                    }
                }
                if (cy == nextcy && cx == nextcx) {
                    flag9p = 0;
                    flag5p = 1 - flag5p;
                } else {
                    cy = nextcy;
                    cx = nextcx;
                }
            }
        }
}

void StoreLCUInf(FD *curRealFD, int LeaderBlockNumber, int cuinwidth, int iqp, double dlambda, int curtype)
{
    unsigned int h, w, top, left, bottom, right;
    BD *workBD;
    int workBlockNum;

    int LeaderNumber = ((LeaderBlockNumber % cuinwidth) / 8 + LeaderBlockNumber / cuinwidth / 8 *
                        curRealFD->TotalBlockNumInWidth) * (curRealFD->CUSize / curRealFD->BlockSize);

    top = LeaderNumber / curRealFD->TotalBlockNumInWidth;
    left = LeaderNumber % curRealFD->TotalBlockNumInWidth;
    bottom = top + curRealFD->CUSize / curRealFD->BlockSize;
    bottom = bottom <= curRealFD->TotalBlockNumInHeight ? bottom : curRealFD->TotalBlockNumInHeight;
    right = left + curRealFD->CUSize / curRealFD->BlockSize;
    right = right <= curRealFD->TotalBlockNumInWidth ? right : curRealFD->TotalBlockNumInWidth;

    workBlockNum = LeaderNumber;
    for (h = top; h < bottom; h++) {
        for (w = left; w < right; w++) {
            workBD = &curRealFD->BlockDistortionArray[workBlockNum + w - left];
            workBD->BlockQP = iqp;
            workBD->BlockLambda = dlambda;
            workBD->BlockType = curtype;
        }
        workBlockNum = workBlockNum + curRealFD->TotalBlockNumInWidth;
    }
}

double F(double invalue)
{
    double f;
    if (invalue < 0.5F) {
        f = 0.015F;
    } else if (invalue < 2.0F) {
        f = (54.852103 * invalue * invalue + 10.295705 * invalue - 3.667158) / 1000;
    } else if (invalue < 8.0F) {
        f = (-19.235059 * invalue * invalue + 311.129530 * invalue - 317.360050) / 1000 - 0.2280 + 0.2363;
    } else {
        f = 0.949F;
    }
    return max(0.015F, min(f, 0.949F));
}
FD *SearchFrameDistortionArray(DL *omcplist, int FrameNumberInFile, int StepLength, int IntraPeriod)
{
    int keyframenum, subframenum, i, subframenumIndex;
    FD *NewFD;
    keyframenum = FrameNumberInFile / StepLength;
    subframenumIndex = FrameNumberInFile % StepLength;
    if (subframenumIndex == 0) {
        NewFD = &omcplist->FrameDistortionArray[keyframenum - 1];
    }
    if (subframenumIndex != 0 && IntraPeriod == 0) {
        NewFD = &omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[subframenumIndex - 1];
    }
    if (subframenumIndex != 0 && IntraPeriod != 0) {
        for (i = 0; i < StepLength - 1; i++) {
            subframenum = omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[i].FrameNumber;
            if (subframenum == FrameNumberInFile) {
                NewFD = &omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[i];
                break;
            }
        }
    }
    return NewFD;
}

void SearchMseMean(DL *omcplist, int FrameNumberInFile, int StepLength, int IntraPeriod, FD *p1bhFD, FD *p2bhFD)
{
    int keyframenum, subframenum, i, subframenumIndex, TotalBlocksInAframe;
    keyframenum = FrameNumberInFile / StepLength;
    subframenumIndex = FrameNumberInFile % StepLength;
    TotalBlocksInAframe = omcplist->FrameDistortionArray[0].TotalNumOfBlocks;
    if (subframenumIndex == 0 && IntraPeriod == 0) {

        p2bhFD->BlockDistortionArray =
            omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[0].BlockDistortionArray;
        if (FrameNumberInFile < (int)(omcplist->TotalFrameNumber - 1)*StepLength) {
            p1bhFD->BlockDistortionArray = omcplist->FrameDistortionArray[keyframenum].BlockDistortionArray;
        } else {
            p1bhFD->BlockDistortionArray = p2bhFD->BlockDistortionArray;
        }
    }
    if (subframenumIndex == 0 && IntraPeriod != 0) {
        p1bhFD->BlockDistortionArray = omcplist->FrameDistortionArray[keyframenum -
                                       1].subFrameDistortionArray[0].BlockDistortionArray;
        if (FrameNumberInFile < (int)(omcplist->TotalFrameNumber - 1)*StepLength) {
            p2bhFD->BlockDistortionArray = omcplist->FrameDistortionArray[keyframenum].BlockDistortionArray;
        } else {
            p2bhFD->BlockDistortionArray = p1bhFD->BlockDistortionArray;
        }

    }

    if (subframenumIndex != 0 && IntraPeriod == 0) {
        p2bhFD->BlockDistortionArray = p1bhFD->BlockDistortionArray =
                                           omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[subframenumIndex].BlockDistortionArray;
    }
    if (subframenumIndex != 0 && IntraPeriod != 0) {
        for (i = 0; i < StepLength - 1; i++) {
            subframenum = omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[i].FrameNumber;
            if (subframenum == FrameNumberInFile) {
                p2bhFD->BlockDistortionArray = omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[(i + 1) *
                                               2].BlockDistortionArray;
                p1bhFD->BlockDistortionArray = omcplist->FrameDistortionArray[keyframenum].subFrameDistortionArray[(i + 1) * 2 -
                                               1].BlockDistortionArray;
                break;
            }
        }
    }
}

void CaculateKappaTableLDP(DL *omcplist, DL *realDlist, int framenum, int FrameQP)
{
    int t, b;
    double fxvalue;
    BD *p1stBD, *pcurBD;
    int TotalBlocksInAframe = realDlist->FrameDistortionArray[0].TotalNumOfBlocks;
    int BlocksInWidth = realDlist->FrameDistortionArray[0].TotalBlockNumInWidth;
    int BlockSize = realDlist->FrameDistortionArray[0].BlockSize;

    double  *D, *DMCP;
    double  *BetaTable, *MultiplyBetas;
    double DsxKappa, Ds;
    double smoothfactor;
    int BetaLength;
    int PreFrameQP;

    smoothfactor = framenum < 8 ? 1.0 / framenum : 1.0 / 8;
    smoothfactor = 1.0;

    if (framenum == 0) {
        if (KappaTable != NULL) {
            free(KappaTable);
        }
        KappaTable = NULL;
        return;
    }

    D = (double *)calloc(TotalBlocksInAframe, sizeof(double));
    DMCP = (double *)calloc(TotalBlocksInAframe, sizeof(double));
    BetaTable = (double *)calloc(TotalBlocksInAframe, sizeof(double));
    MultiplyBetas = (double *)calloc(TotalBlocksInAframe, sizeof(double));

    BetaLength = realDlist->TotalFrameNumber - 1  - framenum - 1;
    BetaLength = min(2, BetaLength);

    if (KappaTable == NULL) {
        KappaTable = (double *)calloc(TotalBlocksInAframe, sizeof(double));
    }

    p1stBD = realDlist->FrameDistortionArray[framenum - 1].BlockDistortionArray;
    for (b = 0; b < TotalBlocksInAframe; b++) {
        D[b] = p1stBD[b].MSE;
        BetaTable[b] = 1.0F;
    }

    memset(KappaTable, 0, TotalBlocksInAframe * sizeof(double));
    for (b = 0; b < TotalBlocksInAframe; b++) {
        MultiplyBetas[b] = 1.0;
    }

    pcurBD = omcplist->FrameDistortionArray[framenum - 1].BlockDistortionArray;
    for (t = 0; t <= BetaLength; t++) {
        PreFrameQP = FrameQP - QpOffset[framenum % GroupSize] + QpOffset[(framenum + t) % GroupSize];
        for (b = 0; b < TotalBlocksInAframe; b++) {
            DMCP[b] = tdrdoAlpha * (D[b] + pcurBD[b].MSE);
        }
        for (b = 0; b < TotalBlocksInAframe; b++) {
            fxvalue = sqrt(2.0) * pow(2.0, (PreFrameQP) / 8.0) / sqrt(DMCP[b]);
            D[b] = DMCP[b] * F(fxvalue);
            BetaTable[b] = tdrdoAlpha * F(fxvalue);
            if (t > 0) {
                MultiplyBetas[b] *= BetaTable[b];
                KappaTable[b] += MultiplyBetas[b];
            }
        }
    }

    DsxKappa = Ds = 0.0F;

    for (b = 0; b < TotalBlocksInAframe; b++) {
        t = framenum - 1;
        Ds += realDlist->FrameDistortionArray[t].BlockDistortionArray[b].MSE;
        DsxKappa += realDlist->FrameDistortionArray[t].BlockDistortionArray[b].MSE * (1.0F + KappaTable[b]);
    }

    GlobeLambdaRatio = DsxKappa / Ds;

    free(D);
    free(DMCP);
    free(BetaTable);
    free(MultiplyBetas);
}

void  AdjustLcuQPLambdaLDP(FD *curOMCPFD, int LeaderBlockNumber, int cuinwidth, int *pQP, double *plambda)
{
    unsigned int h, w, top, left, bottom, right;
    int LeaderNumber;
    int workBlockNum;
    double ArithmeticMean, HarmonicMean, GeometricMean;
    double SumOfMSE;
    double Kappa, LambdaRatio, dDeltaQP;
    int counter, iDeltaQP;

    if (curOMCPFD == NULL) {
        return;
    }

    if (KappaTable == NULL) {
        dDeltaQP = 0.0F;
        iDeltaQP = dDeltaQP > 0 ? (int)(dDeltaQP + 0.5) : -(int)(-dDeltaQP + 0.5);
        iDeltaQP = max(-2, min(iDeltaQP, 2));
        return;
    }

    LeaderNumber = ((LeaderBlockNumber % cuinwidth) / 8 + LeaderBlockNumber / cuinwidth / 8 *
                    curOMCPFD->TotalBlockNumInWidth) * (curOMCPFD->CUSize / curOMCPFD->BlockSize);
    top = LeaderNumber / curOMCPFD->TotalBlockNumInWidth;
    left = LeaderNumber % curOMCPFD->TotalBlockNumInWidth;
    bottom = top + curOMCPFD->CUSize / curOMCPFD->BlockSize;
    bottom = bottom <= curOMCPFD->TotalBlockNumInHeight ? bottom : curOMCPFD->TotalBlockNumInHeight;
    right = left + curOMCPFD->CUSize / curOMCPFD->BlockSize;
    right = right <= curOMCPFD->TotalBlockNumInWidth ? right : curOMCPFD->TotalBlockNumInWidth;

    ArithmeticMean = 0.0;
    HarmonicMean = 0.0;
    GeometricMean = 1.0;
    SumOfMSE = 0.0;
    counter = 0;
    workBlockNum = LeaderNumber;
    for (h = top; h < bottom; h++) {
        for (w = left; w < right; w++) {
            SumOfMSE += curOMCPFD->BlockDistortionArray[workBlockNum + w - left].MSE;
            Kappa = KappaTable[workBlockNum + w - left];
            ArithmeticMean += Kappa;
            HarmonicMean += 1.0 / Kappa;
            GeometricMean *= Kappa;
            counter++;
        }
        workBlockNum = workBlockNum + curOMCPFD->TotalBlockNumInWidth;
    }
    if (counter == 0) {
        return;
    }
    Kappa = ArithmeticMean / counter;
    SumOfMSE = SumOfMSE / counter;

    LambdaRatio = GlobeLambdaRatio / (1.0F + Kappa);
    LambdaRatio = max(pow(2.0, -3.0 / 4.0), min(LambdaRatio, pow(2.0, 3.0 / 4.0)));
    dDeltaQP = 4.0 / log(2.0F) * log(LambdaRatio);
    iDeltaQP = dDeltaQP > 0.0 ? (int)(dDeltaQP + 0.5) : -(int)(-dDeltaQP - 0.5);
    iDeltaQP = max(-3, min(iDeltaQP, 3));
    *plambda *= LambdaRatio;
//  *pQP += iDeltaQP;
//  if(LeaderNumber==0)
//      fprintf(stdout,"\n");
//  fprintf(stdout,"%5.2f\t",dDeltaQP);
//  if(right==curOMCPFD->TotalBlockNumInWidth)
//      fprintf(stdout,"\n");
}
