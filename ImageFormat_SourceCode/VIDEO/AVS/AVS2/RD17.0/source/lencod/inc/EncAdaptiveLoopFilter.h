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

#ifndef ENCADAPTIVELOOPFILTER_H
#define ENCADAPTIVELOOPFILTER_H

#include "commonVariables.h"
#include "defines.h"
#include "contributors.h"
#include "../../lcommon/inc/ComAdaptiveLoopFilter.h"

typedef struct {
    double ** *ECorr; //!< auto-correlation matrix
    double  **yCorr; //!< cross-correlation
    double   *pixAcc;
    int componentID;
} AlfCorrData;

typedef struct {
    AlfCorrData **m_alfCorr[NUM_ALF_COMPONENT];
    AlfCorrData **m_alfNonSkippedCorr[NUM_ALF_COMPONENT];
    AlfCorrData  *m_alfCorrMerged[NUM_ALF_COMPONENT];
    AlfCorrData ** **m_alfPrevCorr;

    double  *m_y_merged[NO_VAR_BINS];
    double **m_E_merged[NO_VAR_BINS];
    double   m_y_temp[ALF_MAX_NUM_COEF];
    double   m_pixAcc_merged[NO_VAR_BINS];

    double **m_E_temp;
    ALFParam ** *m_alfPictureParam;

    int *m_coeffNoFilter[NO_VAR_BINS];

    int **m_filterCoeffSym;
    int m_varIndTab[NO_VAR_BINS];

    int *m_numSlicesDataInOneLCU;
    Boolean m_alfLowLatencyEncoding ;
    byte **m_varImg;
    int m_alfReDesignIteration ;
    Boolean **m_AlfLCUEnabled;
    unsigned int m_uiBitIncrement ;
    CSptr m_cs_alf_cu_ctr;
    CSptr m_cs_alf_initial;
} EncALFVar;
extern EncALFVar *Enc_ALF;

void copyToImage(byte *pDest, byte *pSrc, int stride_in, int img_height, int img_width, int formatShift);
void reset_alfCorr(AlfCorrData *alfCorr);
unsigned int uvlcBitrateEstimate(int val);
void AllocateAlfCorrData(AlfCorrData **dst, int cIdx);
void freeAlfCorrData(AlfCorrData *dst);
void setCurAlfParam(ALFParam **alfPictureParam, double lambda);
void ALFProcess(ALFParam **alfPictureParam, ImageParameters *img, byte *imgY_org, byte **imgUV_org, byte *imgY_alf,
                byte **imgUV_alf, double lambda_mode);
void getStatistics(ImageParameters *img, byte *imgY_org, byte **imgUV_org, byte *yuvExtRecY, byte **yuvExtRecUV,
                   int stride, double lambda);
void deriveLoopFilterBoundaryAvailibility(ImageParameters *img, int numLCUPicWidth, int numLCUPicHeight, int ctu,
        Boolean *isLeftAvail, Boolean *isRightAvail, Boolean *isAboveAvail, Boolean *isBelowAvail, double lambda);

void deriveBoundaryAvail(int numLCUPicWidth, int numLCUPicHeight, int ctu, Boolean *isLeftAvail, Boolean *isRightAvail,
                         Boolean *isAboveAvail, Boolean *isBelowAvail, Boolean *isAboveLeftAvail, Boolean *isAboveRightAvail);

void createAlfGlobalBuffers(ImageParameters *img);
void destroyAlfGlobalBuffers(ImageParameters *img, unsigned int uiMaxSizeInbit);
void getStatisticsOneLCU(Boolean skipCUBoundaries, int compIdx, int lcuAddr
                         , int ctuYPos, int ctuXPos, int ctuHeight, int ctuWidth
                         , Boolean isAboveAvail, Boolean isBelowAvail, Boolean isLeftAvail, Boolean isRightAvail, Boolean isAboveLeftAvail,
                         Boolean isAboveRightAvail
                         , AlfCorrData *alfCorr, byte *pPicOrg, byte *pPicSrc
                         , int stride, int formatShift, int numLCUPicWidth, int NumCUsInFrame);
void calcCorrOneCompRegionLuma(byte *imgOrg, byte *imgPad, int stride, int yPos, int xPos, int height, int width
                               , double ***eCorr, double **yCorr, double *pixAcc, int isLeftAvail, int isRightAvail, int isAboveAvail,
                               int isBelowAvail, int isAboveLeftAvail, int isAboveRightAvail);
void calcCorrOneCompRegionChma(byte *imgOrg, byte *imgPad, int stride, int yPos, int xPos, int height, int width,
                               double **eCorr,
                               double *yCorr, int isLeftAvail, int isRightAvail, int isAboveAvail, int isBelowAvail, int isAboveLeftAvail,
                               int isAboveRightAvail);

unsigned int estimateALFBitrateInPicHeader(ALFParam **alfPicParam);
double executePicLCUOnOffDecision(ALFParam **alfPictureParam
                                  , double lambda
                                  , Boolean isRDOEstimate
                                  , AlfCorrData *** alfCorr
                                  , byte *imgY_org, byte **imgUV_org, byte *imgY_Dec, byte **imgUV_Dec, byte *imgY_Res, byte **imgUV_Res
                                  , int stride
                                 );
unsigned int ALFParamBitrateEstimate(ALFParam *alfParam);
long estimateFilterDistortion(int compIdx, AlfCorrData *alfCorr, int **coeffSet, int filterSetSize, int *mergeTable,
                              Boolean doPixAccMerge);
void mergeFrom(AlfCorrData *dst, AlfCorrData *src, int *mergeTable, Boolean doPixAccMerge);
long xCalcSSD(byte *pOrg, byte *pCmp, int iWidth, int iHeight, int iStride);
double findFilterCoeff(double ***EGlobalSeq, double **yGlobalSeq, double *pixAccGlobalSeq, int **filterCoeffSeq,
                       int **filterCoeffQuantSeq, int intervalBest[NO_VAR_BINS][2], int varIndTab[NO_VAR_BINS], int sqrFiltLength,
                       int filters_per_fr, int *weights, double errorTabForce0Coeff[NO_VAR_BINS][2]);
double xfindBestCoeffCodMethod(int **filterCoeffSymQuant, int filter_shape, int sqrFiltLength, int filters_per_fr,
                               double errorForce0CoeffTab[NO_VAR_BINS][2], double lambda);
int getTemporalLayerNo(int curPoc, int picDistance);
void filterOneCTB(byte *pRest, byte *pDec, int stride, int compIdx, ALFParam *alfParam, int ctuYPos, int ctuHeight,
                  int ctuXPos, int ctuWidth
                  , Boolean isAboveAvail, Boolean isBelowAvail, Boolean isLeftAvail, Boolean isRightAvail, Boolean isAboveLeftAvail,
                  Boolean isAboveRightAvail);
void copyOneAlfBlk(byte *picDst, byte *picSrc, int stride, int ypos, int xpos, int height, int width, int isAboveAvail,
                   int isBelowAvail, int isChroma);
void ADD_AlfCorrData(AlfCorrData *A, AlfCorrData *B, AlfCorrData *C);
void accumulateLCUCorrelations(AlfCorrData **alfCorrAcc, AlfCorrData *** alfCorSrcLCU, Boolean useAllLCUs);
void decideAlfPictureParam(ALFParam **alfPictureParam, AlfCorrData **alfCorr, double m_dLambdaLuma);
void deriveFilterInfo(int compIdx, AlfCorrData *alfCorr, ALFParam *alfFiltParam, int maxNumFilters, double lambda);
void xcodeFiltCoeff(int **filterCoeff, int *varIndTab, int numFilters, ALFParam *alfParam);
void xQuantFilterCoef(double *h, int *qh);
void xFilterCoefQuickSort(double *coef_data, int *coef_num, int upper, int lower);
void xfindBestFilterVarPred(double **ySym, double ***ESym, double *pixAcc, int **filterCoeffSym,
                            int *filters_per_fr_best, int varIndTab[], double lambda_val, int numMaxFilters);
double mergeFiltersGreedy(double **yGlobalSeq, double ***EGlobalSeq, double *pixAccGlobalSeq,
                          int intervalBest[NO_VAR_BINS][2], int sqrFiltLength, int noIntervals);
void storeAlfTemporalLayerInfo(ALFParam **alfPictureParam, int temporalLayer, double lambda);
double calculateErrorAbs(double **A, double *b, double y, int size);
void predictALFCoeff(int **coeff, int numCoef, int numFilters);


double QuantizeIntegerFilterPP(double *filterCoeff, int *filterCoeffQuant, double **E, double *y, int sqrFiltLength,
                               int *weights);
void roundFiltCoeff(int *FilterCoeffQuan, double *FilterCoeff, int sqrFiltLength, int factor);
double calculateErrorCoeffProvided(double **A, double *b, double *c, int size);
void add_A(double **Amerged, double ***A, int start, int stop, int size);
void add_b(double *bmerged, double **b, int start, int stop, int size);
void Copy_frame_for_ALF();

int gnsSolveByChol(double **LHS, double *rhs, double *x, int noEq);
long xFastFiltDistEstimation(double **ppdE, double *pdy, int *piCoeff, int iFiltLength);
long calcAlfLCUDist(Boolean isSkipLCUBoundary, int compIdx, int lcuAddr, int ctuYPos, int ctuXPos, int ctuHeight,
                    int ctuWidth, Boolean isAboveAvail, Boolean isBelowAvail, Boolean isLeftAvail, Boolean isRightAvail
                    , byte *picSrc, byte *picCmp, int stride, int formatShift);

void AllocateAlfPar(ALFParam **alf_par, int cID);
void freeAlfPar(ALFParam *alf_par, int cID);
void allocateAlfAPS(ALF_APS *pAPS);
void freeAlfAPS(ALF_APS *pAPS);
#endif
