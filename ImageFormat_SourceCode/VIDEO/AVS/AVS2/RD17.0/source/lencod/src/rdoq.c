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
#include <memory.h>
#include "../../lcommon/inc/defines.h"
#include "../../lcommon/inc/transform.h"
#include "rdoq.h"

#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif

enum {
    LAST_POS,
    LAST_RUN,
    RUN_LEVEL_PAIR
};

typedef struct PairCostTag {
    double levelCost;
    double runCost;
    double uncodedCost;
} PairCost;

typedef struct CostStatTag {
    int pairNum;
    int posBlockX[16 + 1];
    int posBlockY[16 + 1];
    int scanPos[16 + 1];
    PairCost pairCost[16 + 1];
    double sigCGFlagCost;
    double sigCGFlagCost0;
    double lastRunCost;
    double sigCGFlag;
} CostStat;

typedef struct CostTag {
    double lastPos;
    double lastRun;
    double levelRunPair;
} Cost;

typedef struct NodeTag {
    struct NodeTag *prev;
    struct NodeTag *next;
    levelDataStruct *levelData;
    int attrib; // 0: last pos; 1: last run; 2: (Run, Level) pair
    int level;
    int run;
    double cost;
    int pos;    // scan position in CG
} Node;

typedef struct LinkedListTag {
    Node *head;
    Node *tail;
    int size;
} LinkedList;

typedef struct CGDataStructTag {
    int pairNum;                          // total number of (Run, Level) pairs
    int level[32 * 32];                   // levels in the transform block
    int run[32 * 32];                     // runs in the transform block
    double d64UncodedDist[8 * 8];         // distorion cost when coding all levels as 0
    double d64PreDist[8 * 8];             // distorion cost when coding all levels at pre-level
    int lastCGScanPos;                    // scan position for last CG
    int lastScanPosInLastCG;              // scan position of last coefficient in the last CG
    levelDataStruct levelData[32 * 32 + 1]; //levels and distortions
    int cgPreLevel[8 * 8][16];            // HDQ non-zero levels in current CG
    int cgLevel[8 * 8][16][3];            // All 3 possible non-zero levels (level, level-1, 0) in current CG
    double cgErrLevel[8 * 8][16][3];      // distortion of the associated level
    int cgNoLevels[8 * 8][16];            // Number of possible SDQ levels
    int cgCoeffBlockPosX[8 * 8][16];      // x coordinate of the coeff in transform block
    int cgCoeffBlockPosY[8 * 8][16];      // y coordinate of the coeff in transform block
    int cgScanPos[8 * 8][16];             // position of the coefficient in zig-zag scan order in current CG
    int cgRun[8 * 8][16];                 // runs in current CG
    int cgFlag[8 * 8];                    // significant CG flag
    int cgPairNum[8 * 8];                 // number of level-run pairs in current transform block
    int cgLastRun[8 * 8];                 // last run in current transform block
    double d64RateLastCG;                 // rate for last CG
    double d64RateLastPosLastCG;          // rate for last position in last CG
    double d64RateCGFlag[8 * 8];          // rate for CG flag
    double d64RateLastRun;                // rate for last Run
    double d64RateCGLevel[8 * 8][16][3];  // rate for CG Level
    double d64RateCGRun[8 * 8][16][3];    // rate for CG Run
    double d64RateCGSign[8 * 8][16][3];   // rate for sign

} CGDataStruct;

typedef struct coeffGroupRDCostTag {
    double d64CodedLevelandDist;  // distortion and level cost only
    double d64UncodedDist;        // all zero coded block distortion
    double d64RunCost;            // cost for coding Run
} coeffGroupRDCost;

void ll_init(LinkedList *list, int *index)
{
    list->head = NULL;
    list->tail = NULL;
    (*index) = 0;
}
Node *create_node(levelDataStruct *levelData, int attrib, int pos, Node *nodeBuf, int *index)
{
    Node *n = nodeBuf + *index;
    n->attrib = attrib;
    n->levelData = levelData;
    n->run = 0;
    n->pos = pos;
    n->prev = NULL;
    n->next = NULL;
    (*index)++;
    return n;
}

void append_node(LinkedList *list, Node *n)
{
    if (list->head == NULL) { // empty list
        list->head = n;
    } else {
        list->tail->next = n;
        n->prev = list->tail;
    }
    list->tail = n;
}

void remove_node(LinkedList *list, Node *n)
{
    n->prev->next = n->next;
    if (n->next != NULL) {
        n->next->prev = n->prev;
    }
}


extern int g_intraModeClassified[NUM_INTRA_PMODE];


void rdoq_cg(int qp, int mode, int **curr_blk, unsigned int uiBitSize, double lambda, int isChroma,
             unsigned int uiPositionInPic, int intraPredMode, codingUnit *currMB)
{
    int blockSize = (uiBitSize == B64X64_IN_BIT) ? uiBitSize -- : uiBitSize;
    int residue_bit;
    int shift_16_bit;
// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    int intra;
    int **levelscale;
    int wqm_coef, levelscale_coef;
    int WQMSizeId, WQMSize;
    int iStride;
    int wqm_shift;
#endif

    levelDataStruct levelData[16];
    CostStat costStat[64];
    int sigCGFlag[64];
    int lowerInt, qp_const;
    int shift, QPI;
    int coeff_num = (1 << uiBitSize) * (1 << uiBitSize);
    int coeff_ctr;
    int CG_num = coeff_num >> 4;
    int **AVS_SCAN;
    LinkedList llLevelRun;
    Node nodeBuf[16 +
                 1]; // buffer of linked list nodes, so that dynamic mem alloc is not necessary. this is for speed purpose
    int index = 0;
    int lastPos = -1;
    int xx, yy;
    int isLastCG = 0;
    int rank = 0, rank_pre = 0;
    int pair_num = 0;

    // initialize contexts
    BiContextType(*Primary) [NUM_MAP_CTX];
    BiContextTypePtr pCTX;
    BiContextTypePtr pCTXSigCG;
    BiContextTypePtr pCTXLastCG;
    BiContextTypePtr pCTXLastPosInCG;
    int ctx = 0, offset = 0, ctx2 = 0, sigCGctx = 0, firstCGctx = 0;
    int lastScanPos = -1;
    int iCGLastScanPos = -1;
    double d64BaseCost = 0;
    int iCG;
    //int CGLastX, CGLastY;
    double prevLastCGCost, prevLastPosCost, prevLevelCost, prevRunCost, prevUncodedCost;
    int CGx = 0, CGy = 0;
    double bestCost;
    double currCost;
    double blockUnCodedCost = 0;
    int ctxmode = INTRA_PRED_DC_DIAG;

    if (!isChroma) {
        ctxmode = g_intraModeClassified[intraPredMode];
        if (ctxmode == INTRA_PRED_HOR) {
            ctxmode = INTRA_PRED_VER;
        }
    }
    memset(sigCGFlag, 0, sizeof(int) * 64);
    memset(costStat, 0, sizeof(CostStat) * 64);
    if (!isChroma) {
        Primary = img->currentSlice->syn_ctx->map_contexts;
        pCTXSigCG = img->currentSlice->syn_ctx->sigCG_contexts;
        pCTXLastCG = img->currentSlice->syn_ctx->lastCG_contexts;
        pCTXLastPosInCG = img->currentSlice->syn_ctx->lastPos_contexts;
    } else {
        Primary = img->currentSlice->syn_ctx->last_contexts;
        pCTXSigCG = img->currentSlice->syn_ctx->sigCG_contexts + NUM_SIGCG_CTX_LUMA;
        pCTXLastCG = img->currentSlice->syn_ctx->lastCG_contexts + NUM_LAST_CG_CTX_LUMA;
        pCTXLastPosInCG = img->currentSlice->syn_ctx->lastPos_contexts + NUM_LAST_POS_CTX_LUMA;
    }
    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
            (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                          (currMB->cuType == PHOR_DOWN)))) {
        if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_SCAN4x16;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_SCAN8x32;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_SCAN8x32;
            coeff_num = coeff_num >> 2;
            CG_num = CG_num >> 2;
        }

// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (uiBitSize == B8X8_IN_BIT) {
            WQMSizeId = 2;
        } else if ((uiBitSize == B16X16_IN_BIT) || (uiBitSize == B32X32_IN_BIT)) {
            WQMSizeId = 3;
        }
#endif
    } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
               (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                       (currMB->cuType == PVER_RIGHT)))) {
#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_SCAN16x4;
            WQMSizeId = 2;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_SCAN32x8;
            WQMSizeId = 3;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_SCAN32x8;
            coeff_num = coeff_num >> 2;
            CG_num = CG_num >> 2;
            WQMSizeId = 3;
        }
        if (uiBitSize == B8X8_IN_BIT) {
            WQMSizeId = 2;
        } else if ((uiBitSize == B16X16_IN_BIT) || (uiBitSize == B32X32_IN_BIT)) {
            WQMSizeId = 3;
        }
#else
        if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_SCAN16x4;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_SCAN32x8;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_SCAN32x8;
            coeff_num = coeff_num >> 2;
            CG_num = CG_num >> 2;
        }
#endif
    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma)  && (currMB->trans_size == 1) &&
                                  currMB->cuType == InNxNMB)) {   //add yuqh20130824
        if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_SCAN4x16;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_SCAN8x32;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_SCAN8x32;
            coeff_num = coeff_num >> 2;
            CG_num = CG_num >> 2;
        }
#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (uiBitSize == B8X8_IN_BIT) {
            WQMSizeId = 2;
        } else if ((uiBitSize == B16X16_IN_BIT) || (uiBitSize == B32X32_IN_BIT)) {
            WQMSizeId = 3;
        }
#endif

    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma)   && (currMB->trans_size == 1) &&
                                  currMB->cuType == INxnNMB)) {  //add yuqh20130824
        if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_SCAN16x4;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_SCAN32x8;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_SCAN32x8;
            coeff_num = coeff_num >> 2;
            CG_num = CG_num >> 2;
        }
#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (uiBitSize == B8X8_IN_BIT) {
            WQMSizeId = 2;
        } else if ((uiBitSize == B16X16_IN_BIT) || (uiBitSize == B32X32_IN_BIT)) {
            WQMSizeId = 3;
        }
#endif
    } else {
#if FREQUENCY_WEIGHTING_QUANTIZATION
        //zig-zag scan;
        if (uiBitSize == B4X4_IN_BIT) {
            AVS_SCAN = AVS_SCAN4x4;
            WQMSizeId = 0;
        } else if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN8x8;
            WQMSizeId = 1;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN16x16;
            WQMSizeId = 2;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN32x32;
            WQMSizeId = 3;
        }
#else
        //zig-zag scan;
        if (uiBitSize == B4X4_IN_BIT) {
            AVS_SCAN = AVS_SCAN4x4;
        } else if (uiBitSize == B8X8_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN8x8;
        } else if (uiBitSize == B16X16_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN16x16;
        } else if (uiBitSize == B32X32_IN_BIT) {
            AVS_SCAN = AVS_CG_SCAN32x32;
        }
#endif
    }

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    intra = (mode > 3) ? 1 : 0;
    if (WeightQuantEnable) {
        WQMSize = 1 << (WQMSizeId + 2);
        if (WQMSizeId == 0) {
            levelscale = LevelScale4x4[intra];
        } else if (WQMSizeId == 1) {
            levelscale = LevelScale8x8[intra];
        } else if (WQMSizeId == 2) {
            levelscale = LevelScale16x16[intra];
        } else if (WQMSizeId == 3) {
            levelscale = LevelScale32x32[intra];
        }
    }
#endif

    if (mode > 3) {
        qp_const = (1 << 15) * 10 / 31;
    } else {
        qp_const = (1 << 15) * 10 / 62;
    }

    shift = IQ_SHIFT[qp];
    QPI   = IQ_TAB[qp];

    for (coeff_ctr = coeff_num - 1; coeff_ctr >= 0; coeff_ctr--) {
        int coeff_ctr_in_cg = coeff_ctr & 0xf;
        int iCG = coeff_ctr >> 4;
        double rec_double;
        int level, QPI;
        double temp = 16384.0 / Q_TAB[qp] ;
        int ii;
        double err;
        Node *node;
        int shift5;

        // quant_init
        xx = AVS_SCAN[coeff_ctr][0];
        yy = AVS_SCAN[coeff_ctr][1];
        if (!isChroma && g_intraModeClassified[intraPredMode] == INTRA_PRED_HOR && currMB->cuType != InNxNMB &&
            currMB->cuType != INxnNMB) {
            SWAP(xx, yy);
        }

// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION

        if (WeightQuantEnable) {
#if !WQ_MATRIX_FCD
            wqm_shift = (input->PicWQDataIndex == 1) ? 3 : 0;
#else
            wqm_shift = 2;
#endif

            if ((WQMSizeId == 0) || (WQMSizeId == 1)) {
                iStride = WQMSize;
                wqm_coef = cur_wq_matrix[WQMSizeId][(yy & (iStride - 1)) * iStride + (xx & (iStride - 1))];
                //levelscale_coef=levelscale[(yy&(WQMSize-1))*WQMSize+(xx&(WQMSize-1))];
                levelscale_coef = levelscale[yy & (WQMSize - 1)][xx & (WQMSize - 1)];
            } else if (WQMSizeId == 2) {
                iStride = WQMSize >> 1;
#if AWQ_LARGE_BLOCK_EXT_MAPPING
                wqm_coef = cur_wq_matrix[WQMSizeId][((yy >> 1) & (iStride - 1)) * iStride + ((xx >> 1) & (iStride - 1))];
#endif
                //levelscale_coef=levelscale[(yy&(WQMSize-1))*WQMSize+(xx&(WQMSize-1))];
                levelscale_coef = levelscale[yy & (WQMSize - 1)][xx & (WQMSize - 1)];

            } else if (WQMSizeId == 3) {
                iStride = WQMSize >> 2;
#if AWQ_LARGE_BLOCK_EXT_MAPPING
                wqm_coef = cur_wq_matrix[WQMSizeId][((yy >> 2) & (iStride - 1)) * iStride + ((xx >> 2) & (iStride - 1))];
#endif
                //levelscale_coef=levelscale[(yy&(WQMSize-1))*WQMSize+(xx&(WQMSize-1))];
                levelscale_coef = levelscale[yy & (WQMSize - 1)][xx & (WQMSize - 1)];
            }
        }
#endif

        QPI   = IQ_TAB[qp];
        temp = 16384.0 / Q_TAB[qp];

        residue_bit = (blockSize == B64X64_IN_BIT) ? (input->sample_bit_depth + 2) : (input->sample_bit_depth + 1);
        shift_16_bit = 16 - residue_bit - uiBitSize;
        if (shift_16_bit >= 1) {
#if FREQUENCY_WEIGHTING_QUANTIZATION
            if (WeightQuantEnable) {
#if AWQ_WEIGHTING
                levelData[coeff_ctr_in_cg].levelDouble = (((int)absm(curr_blk[yy][xx]) * levelscale_coef) + (1 <<
                        (shift_16_bit - 1))) >> shift_16_bit;
#else
                levelData[coeff_ctr_in_cg].levelDouble = (absm(curr_blk[yy][xx]) + (1 << (shift_16_bit - 1))) >> shift_16_bit;
#endif
            } else {
                levelData[coeff_ctr_in_cg].levelDouble = (absm(curr_blk[yy][xx]) + (1 << (shift_16_bit - 1))) >> shift_16_bit;
            }
#else
            levelData[coeff_ctr_in_cg].levelDouble = (absm(curr_blk[yy][xx]) + (1 << (shift_16_bit - 1))) >> shift_16_bit;
#endif
        }


        levelData[coeff_ctr_in_cg].levelDouble2 =  absm(curr_blk[yy][xx]);
        level = (int)(levelData[coeff_ctr_in_cg].levelDouble2 * Q_TAB[qp] >> (15 + shift_16_bit));
#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (WeightQuantEnable) {
            level = Clip3((-((1 << 18) / wqm_coef)), (((1 << 18) / wqm_coef) - 1), level); //M2239
        }
#endif
        shift5 = IQ_SHIFT[qp] - shift_16_bit;

#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (!WeightQuantEnable) {
            rec_double = (level * QPI + (1 << (shift5 - 1))) >> (shift5);
        } else {
            rec_double = (double)(((((((long long int)(level * wqm_coef)) >> wqm_shift) * QPI) >> 4) + (long long int)(1 <<
                                   (shift5 - 1))) >> (shift5));
        }
#if QuantMatrixClipFix
        level = Clip3(0 - (1 << 15), (1 << 15) - 1, level);
#endif
#if QuantClip
        rec_double = Clip3(-32768, 32767, rec_double);
#endif
#else
        rec_double = (level * QPI + (1 << (shift5 - 1))) >> (shift5);
#endif

        lowerInt = ((levelData[coeff_ctr_in_cg].levelDouble2 - rec_double) / (1 << (shift_16_bit >= 0 ? shift_16_bit : 0)) <=
                    temp) ? 1 : 0;


        levelData[coeff_ctr_in_cg].level[0] = 0;
        levelData[coeff_ctr_in_cg].noLevels = 1;
        levelData[coeff_ctr_in_cg].scanPos = coeff_ctr;
        if (level == 0 && lowerInt == 1) {
            levelData[coeff_ctr_in_cg].noLevels = 1;
        } else if (level == 0 && lowerInt == 0) {
            levelData[coeff_ctr_in_cg].level[1] = 1;
            levelData[coeff_ctr_in_cg].noLevels = 2;
        } else if (level > 0 && lowerInt == 1) {
            levelData[coeff_ctr_in_cg].level[1] = level;
            levelData[coeff_ctr_in_cg].noLevels = 2;
        } else {
            levelData[coeff_ctr_in_cg].level[1] = level;
            levelData[coeff_ctr_in_cg].level[2] = level + 1;
            levelData[coeff_ctr_in_cg].level[2] = Clip3(-32768, 32767, levelData[coeff_ctr_in_cg].level[2]);
            levelData[coeff_ctr_in_cg].noLevels = 3;
        }

        for (ii = 0; ii < levelData[coeff_ctr_in_cg].noLevels; ii++) {
// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
            if (WeightQuantEnable) {
#if AWQ_WEIGHTING
                rec_double = (double)(((((((long long int)levelData[coeff_ctr_in_cg].level[ii] * wqm_coef) >> wqm_shift) * QPI) >> 4)
                                       + (long long int)(1 << (shift5 - 1))) >> (shift5));
#else
                rec_double = (double)((levelData[coeff_ctr_in_cg].level[ii] * QPI + (1 << (shift5 - 1))) >> (shift5));
#endif
            } else {
                rec_double = (double)((levelData[coeff_ctr_in_cg].level[ii] * QPI + (1 << (shift5 - 1))) >> (shift5));
            }
#if QuantClip
            rec_double = Clip3(-32768, 32767, rec_double);
#endif
            err = (rec_double - levelData[coeff_ctr_in_cg].levelDouble2);
            levelData[coeff_ctr_in_cg].errLevel[ii] = 256 * err * err / (1 << ((shift_16_bit >= 0 ? shift_16_bit : 0) * 2));

#else //FREQUENCY_WEIGHTING_QUANTIZATION

            rec_double = (double)((levelData[coeff_ctr_in_cg].level[ii] * QPI + (1 << (shift5 - 1))) >> (shift5));
            err = (rec_double - levelData[coeff_ctr_in_cg].levelDouble2);
            levelData[coeff_ctr_in_cg].errLevel[ii] = 256 * err * err / (1 << ((shift_16_bit >= 0 ? shift_16_bit : 0) * 2));
#endif //FREQUENCY_WEIGHTING_QUANTIZATION

        }
        if (levelData[coeff_ctr_in_cg].noLevels == 1) {
            // set curr_blk to hdq result
            curr_blk[yy][xx] = 0;
        } else {
            // set curr_blk to hdq result
            curr_blk[yy][xx] = levelData[coeff_ctr_in_cg].level[1];
        }

        levelData[coeff_ctr_in_cg].xx = xx;
        levelData[coeff_ctr_in_cg].yy = yy;

        // build (Level, Run) pair linked list
        if (lastPos == -1) { // last is not found yet
            if (levelData[coeff_ctr_in_cg].noLevels > 1) {
                ll_init(&llLevelRun, &index);
                // found last position in last CG
                lastPos = coeff_ctr_in_cg;
                // first node in the list is last position
                node = create_node(NULL, LAST_POS, coeff_ctr_in_cg, nodeBuf, &index);
                append_node(&llLevelRun, node);

                // the second node is the (run, pair) pair
                node = create_node(&levelData[coeff_ctr_in_cg], RUN_LEVEL_PAIR, coeff_ctr_in_cg, nodeBuf, &index);
                append_node(&llLevelRun, node);

                CG_num = iCG + 1;
                sigCGFlag[iCG] = 1; // this is the last CG
            }
        } else { // last is found
            // first node is last run
            if (coeff_ctr_in_cg == 0xf) { // a new CG begins
                ll_init(&llLevelRun, &index);
                // the position of the last run is always initialized to 15
                node = create_node(NULL, LAST_RUN, coeff_ctr_in_cg, nodeBuf, &index);
                append_node(&llLevelRun, node);
            }

            // starting from the 2nd node, it is (level, run) node
            if (levelData[coeff_ctr_in_cg].noLevels > 1) {
                node = create_node(&levelData[coeff_ctr_in_cg], RUN_LEVEL_PAIR, coeff_ctr_in_cg, nodeBuf, &index);
                append_node(&llLevelRun, node);
                sigCGFlag[iCG] = 1;
                // get the real position of last run
                if (node->prev->attrib == LAST_RUN) {
                    node->prev->pos = coeff_ctr_in_cg;
                }
            } else {
                node->run++;
            }
        }

        if (lastPos != -1) {
            if (coeff_ctr_in_cg == 0) { // a CG just ended
                const int T_Chr[5] = { 0, 1, 2, 4, 3000};
                int isFirstCG = (iCG == 0);
                int isSigCG = 0;
                double lagrUncoded = 0;
                double lagrAcc = 0;
                int pairsInCG = 0;
                node = llLevelRun.head;
                rank_pre = rank;

                // rdoq for this CG
                while (node != NULL) {
                    if (node->attrib == LAST_POS) {
                        isLastCG = 1;
                    } else if (node->attrib == LAST_RUN) { // this is not the last CG
                        isLastCG = 0;

                        if (uiBitSize - 3 == 0) {
                            CGx = iCG & 1;
                            CGy = iCG / 2;
                        } else if (uiBitSize - 3 == 1) {
                            CGx = AVS_SCAN4x4[iCG][0];
                            CGy = AVS_SCAN4x4[iCG][1];
                        } else if (uiBitSize - 3 == 2) {
                            CGx = AVS_SCAN8x8[iCG][0];
                            CGy = AVS_SCAN8x8[iCG][1];
                        } else { //4x4
                            CGx = 0;
                            CGy = 0;
                        }

                        if (node->run != 16) {
                            xx = AVS_SCAN4x4[15 - (node->run)][0];
                            yy = AVS_SCAN4x4[15 - (node->run)][1];
                            costStat[iCG].lastRunCost = lambda * estLastPosRate(xx, yy, pCTXLastPosInCG, 0, CGx, CGy, iCG, ctxmode, uiBitSize,
                                                        isChroma);
                        } else {
                            costStat[iCG].lastRunCost = 0;
                        }
                        lagrAcc += costStat[iCG].lastRunCost;
                    } else { // a (level, run) pair
                        // try level, level-1 first, then compare to level=0 case
                        int levelNo;
                        int n = 0;
                        int k;
                        int absSum5 = 0;
                        int indiv;
                        int absLevel;
                        int rateRunMerged = 0;
                        int bestState;
                        double lagr;
                        double minlagr = 1.7e308;
                        double lagrDelta;
                        double lagrDelta0 = 0.0;

                        isSigCG = 1;
                        for (levelNo = 1; levelNo < node->levelData->noLevels; levelNo++) {
                            double lagr = 0;
                            int rateLevel = 0;
                            int rateRunCurr = 0;
                            int rateRunPrev = 0;
                            double curCost = 0;

                            // rate: Level
                            absLevel = abs(node->levelData->level[levelNo]);

                            indiv = min(2, (pairsInCG + 1) / 2);
                            pCTX = Primary[ min(rank, indiv + 2) ];

                            rateLevel = estLevelRate(absLevel, pCTX, rank, pairsInCG, iCG, 15 - node->pos, isChroma, uiBitSize);

                            // rate: Sign
                            rateLevel += estSignRate(node->levelData->level[levelNo]);

                            // rate: Run
                            // Run[i]
                            absSum5 = absLevel;
                            for (k = node->pos + 1; k <= min(node->pos + 6, 15); k ++) {
                                xx = levelData[k].xx;
                                yy = levelData[k].yy;
                                absSum5 += abs(curr_blk[yy][xx]);
                            }

                            pCTX = Primary[ min(absSum5 / 2, 2) ];
                            rateRunCurr = estRunRate(node->run, pCTX, 15 - node->pos, iCG, node->pos, isChroma, ctxmode, uiBitSize);

                            // Run[i+1]
                            // node->prev always exists
                            if (node->prev->attrib == LAST_POS) {
                                // don't do anything here
                            } else if (node->prev->attrib == LAST_RUN) {
                                if (uiBitSize - 3 == 0) {
                                    CGx = iCG & 1;
                                    CGy = iCG / 2;
                                } else if (uiBitSize - 3 == 1) {
                                    CGx = AVS_SCAN4x4[iCG][0];
                                    CGy = AVS_SCAN4x4[iCG][1];
                                } else if (uiBitSize - 3 == 2) {
                                    CGx = AVS_SCAN8x8[iCG][0];
                                    CGy = AVS_SCAN8x8[iCG][1];
                                } else { //4x4
                                    CGx = 0;
                                    CGy = 0;
                                }

                                if (node->prev->run != 16) {
                                    xx = AVS_SCAN4x4[15 - (node->prev->run)][0];
                                    yy = AVS_SCAN4x4[15 - (node->prev->run)][1];
                                    rateRunPrev = estLastPosRate(xx, yy, pCTXLastPosInCG, 0, CGx, CGy, iCG, ctxmode, uiBitSize, isChroma);
                                } else {
                                    rateRunPrev = 0;
                                }
                                costStat[iCG].lastRunCost = lambda * rateRunPrev;
                            } else { // RUN_LEVEL_PAIR
                                absSum5 = 0;
                                for (k = node->prev->pos; k <= min(node->prev->pos + 6, 15); k ++) {
                                    xx = levelData[k].xx;
                                    yy = levelData[k].yy;
                                    absSum5 += abs(curr_blk[yy][xx]);
                                }

                                pCTX = Primary[ min(absSum5 / 2, 2) ];
                                rateRunPrev = estRunRate(node->prev->run, pCTX, 15 - node->prev->pos, iCG, node->prev->pos, isChroma, ctxmode,
                                                         uiBitSize);

                            }

                            // cost for the current (Level, Run) pair
                            costStat[iCG].pairCost[pairsInCG].levelCost = node->levelData->errLevel[levelNo] + lambda * rateLevel;
                            costStat[iCG].pairCost[pairsInCG].runCost   = lambda * rateRunCurr;
                            costStat[iCG].scanPos[pairsInCG] = node->levelData->scanPos;

                            // calculate cost: distLevel[i] + rateLevel[i] + rateRun[i] + rateRun[i+1]
                            lagr = node->levelData->errLevel[levelNo] +
                                   lambda * (rateLevel + rateRunCurr + rateRunPrev);

                            if (lagr < minlagr) {
                                minlagr = lagr;
                                bestState = levelNo;
                            }

                            lagrDelta = lambda * rateRunPrev;
                        }
                        costStat[iCG].pairCost[pairsInCG].uncodedCost = node->levelData->errLevel[0];

                        // compare cost of level or level-1 with uncoded case (level=0)
                        // Run[i]

                        if (node->prev->attrib == LAST_POS || (node->prev->attrib == LAST_RUN && node->next == NULL)) {
                            // don't do anything here
                        } else {
                            if (node->prev->attrib == RUN_LEVEL_PAIR) {
                                absSum5 = 0;
                                for (k = node->prev->pos; k <= min(node->prev->pos + 6, 15); k ++) {
                                    xx = levelData[k].xx;
                                    yy = levelData[k].yy;
                                    absSum5 += abs(curr_blk[yy][xx]);
                                }

                                pCTX = Primary[ min(absSum5 / 2, 2) ];
                                rateRunMerged = estRunRate(node->prev->run + 1 + node->run, pCTX, 15 - node->prev->pos, iCG, node->prev->pos,
                                                           isChroma, ctxmode, uiBitSize);

                                lagrDelta0 = costStat[iCG].pairCost[pairsInCG - 1].runCost;
                            } else { // LAST_RUN
                                if (node->next != NULL) { // only try 0 when there's more than 1 pair in the CG
                                    if (uiBitSize - 3 == 0) {
                                        CGx = iCG & 1;
                                        CGy = iCG / 2;
                                    } else if (uiBitSize - 3 == 1) {
                                        CGx = AVS_SCAN4x4[iCG][0];
                                        CGy = AVS_SCAN4x4[iCG][1];
                                    } else if (uiBitSize - 3 == 2) {
                                        CGx = AVS_SCAN8x8[iCG][0];
                                        CGy = AVS_SCAN8x8[iCG][1];
                                    } else { //4x4
                                        CGx = 0;
                                        CGy = 0;
                                    }
                                    if (node->prev->run != 16) {
                                        xx = AVS_SCAN4x4[15 - (node->prev->run + 1 + node->run)][0];
                                        yy = AVS_SCAN4x4[15 - (node->prev->run + 1 + node->run)][1];
                                        rateRunMerged = estLastPosRate(xx, yy, pCTXLastPosInCG, 0, CGx, CGy, iCG, ctxmode, uiBitSize, isChroma);
                                    } else {
                                        rateRunMerged = 0;
                                    }
                                    lagrDelta0 = costStat[iCG].lastRunCost;
                                }
                            }

                            // calculate cost: distLevel[i][0] + rate(Run[i] + Run[i+1] + 1)
                            lagr = node->levelData->errLevel[0] + lambda * rateRunMerged;

                            if (lagr < minlagr) {
                                minlagr = lagr;
                                lagrDelta = lagrDelta0;
                                bestState = 0;
                            }
                        }

                        // set SDQ results
                        {
                            xx = levelData[node->pos].xx;
                            yy = levelData[node->pos].yy;
                            curr_blk[yy][xx] = node->levelData->level[bestState];
                            node->level = curr_blk[yy][xx];
                            absLevel = abs(node->levelData->level[bestState]);
                            lagrAcc += minlagr - lagrDelta;
                            lagrUncoded += node->levelData->errLevel[0];
                        }
                        if (!isChroma && g_intraModeClassified[intraPredMode] == INTRA_PRED_HOR && currMB->cuType != InNxNMB &&
                            currMB->cuType != INxnNMB) {
                            SWAP(xx, yy);
                        }
                        costStat[iCG].posBlockX[pairsInCG] = xx;
                        costStat[iCG].posBlockY[pairsInCG] = yy;
                        costStat[iCG].scanPos[pairsInCG] = levelData[node->pos].scanPos;
                        if (bestState == 0) {
                            // adjust the run of the previous node and remove the current node
                            node->prev->run += node->run + 1;
                            if (node->prev->attrib == LAST_RUN) {
                                costStat[iCG].lastRunCost = lambda * rateRunMerged;
                            } else {
                                costStat[iCG].pairCost[pairsInCG - 1].runCost = lambda * rateRunMerged;
                            }

                            remove_node(&llLevelRun, node);
                        } else {
                            pairsInCG++;
                        }

                        //! Update Rank
                        if (absLevel > T_Chr[rank]) {
                            if (absLevel <= 2) {
                                rank = absLevel;
                            } else if (absLevel <= 4) {
                                rank = 3;
                            } else {
                                rank = 4;
                            }
                        }
                    }

                    node = node->next;
                }

                if (!isLastCG) {
                    int CGx, CGy;
                    int iCG = (coeff_ctr >> 4);
                    int prevCGFlagSum = (iCG + 1 >= CG_num ? 0 : sigCGFlag[ iCG + 1 ]) * 2 + (iCG + 2 >= CG_num ? 0 : sigCGFlag[ iCG + 2 ]);
                    int bitSize = uiBitSize - 3;
                    if (bitSize == 0) {
                        CGx = iCG & 1;
                        CGy = iCG / 2;
                    } else if (bitSize == 1) {
                        if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                                (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                              (currMB->cuType == PHOR_DOWN)))) {
                            CGx = AVS_SCAN2x8[iCG][0];
                            CGy = AVS_SCAN2x8[iCG][1];
                        } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                                   (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                           (currMB->cuType == PVER_RIGHT)))) {
                            CGx = AVS_SCAN8x2[iCG][0];
                            CGy = AVS_SCAN8x2[iCG][1];
                        } else if (input->useSDIP    && (IS_INTRA(currMB) && !(isChroma)  && (currMB->trans_size == 1) &&
                                                         currMB->cuType == InNxNMB)) {   //add yuqh20130824
                            CGx = AVS_SCAN2x8[iCG][0];
                            CGy = AVS_SCAN2x8[iCG][1];
                        } else if (input->useSDIP    && (IS_INTRA(currMB) && !(isChroma)  && (currMB->trans_size == 1) &&
                                                         currMB->cuType == INxnNMB)) {   //add yuqh20130824
                            CGx = AVS_SCAN8x2[iCG][0];
                            CGy = AVS_SCAN8x2[iCG][1];
                        } else {
                            CGx = AVS_SCAN4x4[iCG][0];
                            CGy = AVS_SCAN4x4[iCG][1];
                        }
                    } else if (bitSize == 2) {
                        if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                                (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                              (currMB->cuType == PHOR_DOWN)))) {
                            CGx = AVS_SCAN2x8[iCG][0];
                            CGy = AVS_SCAN2x8[iCG][1];
                        } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                                   (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                           (currMB->cuType == PVER_RIGHT)))) {
                            CGx = AVS_SCAN8x2[iCG][0];
                            CGy = AVS_SCAN8x2[iCG][1];
                        } else if (input->useSDIP    && (IS_INTRA(currMB) && !(isChroma)  && (currMB->trans_size == 1) &&
                                                         currMB->cuType == InNxNMB)) {   //add yuqh20130824
                            CGx = AVS_SCAN2x8[iCG][0];
                            CGy = AVS_SCAN2x8[iCG][1];
                        } else if (input->useSDIP    && (IS_INTRA(currMB) && !(isChroma)  && (currMB->trans_size == 1) &&
                                                         currMB->cuType == INxnNMB)) {   //add yuqh20130824
                            CGx = AVS_SCAN8x2[iCG][0];
                            CGy = AVS_SCAN8x2[iCG][1];
                        } else {
                            CGx = AVS_SCAN8x8[iCG][0];
                            CGy = AVS_SCAN8x8[iCG][1];
                        }
                    } else { //4x4
                        CGx = 0;
                        CGy = 0;
                    }
                    sigCGctx = isChroma ? 0 : ((iCG == 0) ?  0 : 1);

                    if (isSigCG) {
                        costStat[iCG].sigCGFlagCost = lambda * estSigCGFlagRate(1, pCTXSigCG, sigCGctx);
                        lagrAcc +=  costStat[iCG].sigCGFlagCost;
                        costStat[iCG].sigCGFlagCost0 = lambda * estSigCGFlagRate(0, pCTXSigCG, sigCGctx);
                        lagrUncoded +=  costStat[iCG].sigCGFlagCost0;
                    } else {
                        costStat[iCG].sigCGFlagCost = lambda * estSigCGFlagRate(0, pCTXSigCG, sigCGctx);
                    }

                }

                // try to turn CG to all-zero here. don't do this to last CG
                if (!isLastCG && isSigCG) {
                    if (lagrUncoded < lagrAcc) {
                        int i;
                        //set the coefficients in CG to zero
                        for (i = coeff_ctr; i < coeff_ctr + 16; i++) {
                            xx = AVS_SCAN[i][0];
                            yy = AVS_SCAN[i][1];
                            if (!isChroma && g_intraModeClassified[intraPredMode] == INTRA_PRED_HOR && currMB->cuType != InNxNMB &&
                                currMB->cuType != INxnNMB) {
                                SWAP(xx, yy);
                            }
                            curr_blk[yy][xx] = 0;
                        }
                        sigCGFlag[iCG] = 0;

                        costStat[iCG].sigCGFlagCost = isFirstCG ? 0 : costStat[iCG].sigCGFlagCost0;
                        costStat[iCG].lastRunCost = 0;
                        pairsInCG = 0;
                        rank = rank_pre;
                    }
                }
                costStat[iCG].pairNum = pairsInCG;
            }
        }
    } // end   for (coeff_ctr = coeff_num - 1; coeff_ctr >= 0; coeff_ctr--)

    // estimate last
    if (lastPos != -1) {
        iCG = CG_num - 1;
        lastScanPos = lastPos + (iCG << 4); // get scan position of the last
        prevLastCGCost = lambda * estCGLastRate(CG_num - 1, pCTXLastCG, uiBitSize - 3, &CGx, &CGy, isChroma, ctxmode, currMB,
                                                g_intraModeClassified[intraPredMode]);
        xx = costStat[CG_num - 1].posBlockX[0] & 0x3;
        yy = costStat[CG_num - 1].posBlockY[0] & 0x3;
        prevLastPosCost = lambda * estLastPosRate(xx, yy, pCTXLastPosInCG, 1, CGx, CGy, iCG, ctxmode, uiBitSize, isChroma);
        prevLevelCost = 0;
        prevRunCost   = 0;
        prevUncodedCost = 0;

        // init bestCost
        bestCost = prevLastCGCost + prevLastPosCost + prevLevelCost + prevRunCost;
        currCost = bestCost;

        for (iCG = CG_num - 1; iCG >= 0; iCG--) {
            int pairNo;
            double currLastCGCost;
            double currLastPosCost;
            double currUncodedCost;
            int prev_last_CG, prev_last_pair;

            // cross CG border
            // Last CG: this following function needs to update for Intra Mode dependent context models
            currLastCGCost = lambda * estCGLastRate(iCG, pCTXLastCG, uiBitSize - 3, &CGx, &CGy, isChroma, ctxmode, currMB,
                                                    g_intraModeClassified[intraPredMode]);

            // when pairNo == 0, the cost is either last position or last run
            if (iCG == CG_num - 1) { // last position for last CG
                // Last Position in Last CG
                xx = costStat[iCG].posBlockX[0] & 0x3;
                yy = costStat[iCG].posBlockY[0] & 0x3;

                currUncodedCost = costStat[iCG].pairCost[0].uncodedCost;
                prev_last_CG = iCG;
                prev_last_pair = 0;
            } else { // last run otherwise
                bestCost += costStat[iCG].lastRunCost;
                if (iCG > 0) {
                    bestCost += costStat[iCG].sigCGFlagCost;
                }
            }

            for (pairNo = 0; pairNo < costStat[iCG].pairNum; pairNo++) { // when pairNo == 0, it is the last pair in CG
                // Last Position in Last CG
                xx = costStat[iCG].posBlockX[pairNo] & 0x3;
                yy = costStat[iCG].posBlockY[pairNo] & 0x3;

                currLastPosCost = lambda * estLastPosRate(xx, yy, pCTXLastPosInCG, 1, CGx, CGy, iCG, ctxmode, uiBitSize, isChroma);
                currCost = currCost - prevLastCGCost  + currLastCGCost
                           - prevLastPosCost + currLastPosCost
                           - prevLevelCost   + costStat[iCG].pairCost[pairNo].levelCost
                           - prevRunCost     + costStat[iCG].pairCost[pairNo].runCost
                           + prevUncodedCost;

                bestCost += costStat[iCG].pairCost[pairNo].levelCost + costStat[iCG].pairCost[pairNo].runCost;
                blockUnCodedCost += costStat[iCG].pairCost[pairNo].uncodedCost;

                prevUncodedCost = costStat[iCG].pairCost[pairNo].uncodedCost;
                prevLevelCost   = costStat[iCG].pairCost[pairNo].levelCost;
                prevRunCost     = costStat[iCG].pairCost[pairNo].runCost;
                prevLastPosCost = currLastPosCost;
                prevLastCGCost  = currLastCGCost;

                if (currCost <= bestCost) {
                    int scanPos;

                    bestCost = currCost;
                    // set all coefficients after the last to zeros
                    for (scanPos = costStat[iCG].scanPos[pairNo] + 1; scanPos <= lastScanPos; scanPos++) {
                        xx = AVS_SCAN[scanPos][0];
                        yy = AVS_SCAN[scanPos][1];
                        if (!isChroma && g_intraModeClassified[intraPredMode] == INTRA_PRED_HOR && currMB->cuType != InNxNMB &&
                            currMB->cuType != INxnNMB) {
                            SWAP(xx, yy);
                        }
                        curr_blk[yy][xx] = 0;
                    }
                }
            }
        }

        //blockUnCodedCost is the total uncoded distortion
        //bestCost is the summation of Best LastPos and Best lastCG and CGSign and lastrun and (run,level)s
        if (blockUnCodedCost < bestCost) {
            int scanPos;

            for (scanPos = 0; scanPos <= lastScanPos; scanPos++) {
                xx = AVS_SCAN[scanPos][0];
                yy = AVS_SCAN[scanPos][1];
                if (!isChroma && g_intraModeClassified[intraPredMode] == INTRA_PRED_HOR && currMB->cuType != InNxNMB &&
                    currMB->cuType != INxnNMB) {
                    SWAP(xx, yy);
                }
                curr_blk[yy][xx] = 0;
            }
        }
    }
    // estimate last for each non-last CG
    {
        int lastScanPosInCG;
        int pairNo;
        double currLastPosCost;
        double prevLastPosCost;
        double prevLevelCost;
        double prevRunCost;
        double prevUncodedCost;

        for (iCG = CG_num - 2; iCG >= 0; iCG--) {

            if (sigCGFlag[iCG] == 0) {
                continue;
            }
            lastScanPosInCG = costStat[iCG].scanPos[0];

            // Last Position in current CG
            xx = costStat[iCG].posBlockX[0] & 0x3;
            yy = costStat[iCG].posBlockY[0] & 0x3;
            prevLastPosCost = lambda * estLastPosRate(xx, yy, pCTXLastPosInCG, 0, CGx, CGy, iCG, ctxmode, uiBitSize, isChroma);
            prevLevelCost = 0;
            prevRunCost   = 0;
            prevUncodedCost = 0;
            currCost = prevLastPosCost;
            bestCost = currCost;

            for (pairNo = 0; pairNo < costStat[iCG].pairNum; pairNo++) {
                // Last Position in current CG
                xx = costStat[iCG].posBlockX[pairNo] & 0x3;
                yy = costStat[iCG].posBlockY[pairNo] & 0x3;
                currLastPosCost = lambda * estLastPosRate(xx, yy, pCTXLastPosInCG, 0, CGx, CGy, iCG, ctxmode, uiBitSize, isChroma);

                currCost = currCost - prevLastPosCost + currLastPosCost
                           - prevLevelCost   + costStat[iCG].pairCost[pairNo].levelCost
                           - prevRunCost     + costStat[iCG].pairCost[pairNo].runCost
                           + prevUncodedCost;

                if (pairNo == costStat[iCG].pairNum - 1) {
                    currCost += costStat[iCG].sigCGFlagCost0 - costStat[iCG].sigCGFlagCost;
                }

                bestCost += costStat[iCG].pairCost[pairNo].levelCost + costStat[iCG].pairCost[pairNo].runCost;

                prevUncodedCost = costStat[iCG].pairCost[pairNo].uncodedCost;
                prevLevelCost   = costStat[iCG].pairCost[pairNo].levelCost;
                prevRunCost     = costStat[iCG].pairCost[pairNo].runCost;
                prevLastPosCost = currLastPosCost;

                if (currCost <= bestCost) {
                    int scanPos;

                    bestCost = currCost;
                    // set all coefficients after the last to zeros
                    for (scanPos = costStat[iCG].scanPos[pairNo] + 1; scanPos <= lastScanPosInCG; scanPos++) {
                        xx = AVS_SCAN[scanPos][0];
                        yy = AVS_SCAN[scanPos][1];
                        if (!isChroma && g_intraModeClassified[intraPredMode] == INTRA_PRED_HOR && currMB->cuType != InNxNMB &&
                            currMB->cuType != INxnNMB) {
                            SWAP(xx, yy);
                        }

                        curr_blk[yy][xx] = 0;
                    }
                }
            }
        }
    }
}

void rdoq_block(int qp, int mode, int **curr_blk, unsigned int uiBitSize, int isChroma, unsigned int uiPositionInPic,
                int intraPredMode, codingUnit *currMB)
{
    int temp[32][32], i, j;
    double lambda_rdoq = he->global_lambda;
    int iSizeX, iSizeY;

    int blockSize = uiBitSize;
    if (uiBitSize == B64X64_IN_BIT) {
        uiBitSize --;
    }

    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
            (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                          (currMB->cuType == PHOR_DOWN)))) {
        iSizeX = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize)) : (1 << (uiBitSize + 1));
        iSizeY = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize - 2)) : (1 << (uiBitSize - 1));
    } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
               (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                       (currMB->cuType == PVER_RIGHT)))) {
        iSizeX = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize - 2)) : (1 << (uiBitSize - 1));
        iSizeY = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize)) : (1 << (uiBitSize + 1));
    } else if (input->useSDIP  && !(isChroma) && currMB->cuType == InNxNMB && (currMB->trans_size == 1)) {
        iSizeX = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize)) : (1 << (uiBitSize + 1));
        iSizeY = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize - 2)) : (1 << (uiBitSize - 1));
    } else if (input->useSDIP && !(isChroma) && currMB->cuType == INxnNMB && (currMB->trans_size == 1)) {
        iSizeX = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize - 2)) : (1 << (uiBitSize - 1));
        iSizeY = (uiBitSize == B32X32_IN_BIT) ? (1 << (uiBitSize)) : (1 << (uiBitSize + 1));
    } else {
        iSizeX = 1 << (uiBitSize);
        iSizeY = 1 << (uiBitSize);
    }
    for (i = 0; i < iSizeY; i++) {
        for (j = 0; j < iSizeX; j++) {
            temp[i][j] = curr_blk[i][j];
        }
    }

    rdoq_cg(qp, mode, curr_blk, blockSize, lambda_rdoq, isChroma, uiPositionInPic, intraPredMode, currMB);

    for (i = 0; i < iSizeY; i++) {
        for (j = 0; j < iSizeX; j++) {
            curr_blk[i][j] = (temp[i][j] > 0) ? curr_blk[i][j] : -curr_blk[i][j];
        }
    }
}
int estimate_bits(int **Quant_Coeff, unsigned int uiBitSize, int uiPositionInPic)
{

    codingUnit  *currMB  = &(img->mb_data [img->current_mb_nr]);
    codingUnit  tempMB;

    copyMBInfo(currMB, &tempMB);

    if (he->g_block8x8 > 3) {
        return estChromaCoeff8x8(Quant_Coeff, &tempMB, uiBitSize, he->g_block8x8, uiPositionInPic);
    } else {
        return estLumaCoeff8x8(Quant_Coeff, &tempMB, uiBitSize, he->g_intra_mode, uiPositionInPic);
    }
}

