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
#include <math.h>
#include <memory.h>
#include <assert.h>
#include "AEC.h"
#include "image.h"
#include "../../lcommon/inc/block_info.h"
#include "../../lcommon/inc/ComAdaptiveLoopFilter.h"
#include "../../lencod/inc/vlc.h"
#if MB_DQP
int last_dquant = 0;
int temp_dquant = 0;
#endif

/***********************************************************************
* L O C A L L Y   D E F I N E D   F U N C T I O N   P R O T O T Y P E S
***********************************************************************
*/
#if MB_DQP
void AEC_new_slice()
{
    last_dquant = 0;
}

void lastdqp2tempdqp()
{
    temp_dquant = last_dquant;
}

void tempdqp2lastdqp()
{
    last_dquant = temp_dquant;
}
#endif

void unary_bin_max_encode(EncodingEnvironmentPtr eep_frame,
                          unsigned int symbol,
                          BiContextTypePtr ctx,
                          int ctx_offset,
                          unsigned int max_symbol);



/*!
************************************************************************
* \brief
*    Allocation of contexts models for the motion info
*    used for arithmetic encoding
************************************************************************
*/
SyntaxInfoContexts *create_contexts_SyntaxInfo(void)
{
    SyntaxInfoContexts *codec_ctx;

    codec_ctx = (SyntaxInfoContexts *) calloc(1, sizeof(SyntaxInfoContexts));
    if (codec_ctx == NULL) {
        no_mem_exit("create_contexts_SyntaxInfo: codec_ctx");
    }
    return codec_ctx;
}



/*!
************************************************************************
* \brief
*    Frees the memory of the contexts models
*    used for arithmetic encoding of the motion info.
************************************************************************
*/
void delete_contexts_SyntaxInfo(SyntaxInfoContexts *codec_ctx)
{
    if (codec_ctx == NULL) {
        return;
    }
    free(codec_ctx);
    return;
}


/*!
**************************************************************************
* \brief
*    generates arithmetic code and passes the code to the buffer
**************************************************************************
*/
int writeSyntaxElement_AEC(codingUnit *currMB, SyntaxElement *se, DataPartition *this_dataPart, int uiPosition)
{
    int curr_len;
    EncodingEnvironmentPtr eep_dp = & (this_dataPart->ee_AEC);

    curr_len = arienco_bits_written(eep_dp);
    // perform the actual coding by calling the appropriate method
    se->writing(currMB, se, eep_dp, uiPosition);
    if (se->type != SE_HEADER) {
        this_dataPart->bitstream->write_flag = 1;
    }
    return (se->len = (arienco_bits_written(eep_dp) - curr_len));
}

/*!
***************************************************************************
* \brief
*    This function is used to arithmetically encode the codingUnit
*    type info of a given MB.
***************************************************************************
*/


void writeCuTypeInfo(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{

    int  binidx = 0;

    int act_ctx = 0;
    int act_sym;

    SyntaxInfoContexts *ctx         = (img->currentSlice)->syn_ctx;


    //const int MapPCUType[14] = {1, 3, 4, 6, 4, 4, 6, 6, 5, 2, 2,2,2,2};
    const int MapPCUType[14] = {1, 2, 3, 4, 3, 3, 4, 4, 5, 6, 6, 6, 6, 6};
    //const int MapBCUType[14] = {5, 1, 2, 3, 2, 2, 3, 3, 4, 6, 6,6,6,6}; //  0, 1, 2, 3, 4, 5, 6, 7, PNXN, I16MB, I8MB,xxx? InNxN,INxnN
    const int MapBCUType[14] = {1, 2, 3, 4, 3, 3, 4, 4, 5, 6, 6, 6, 6, 6}; //  0, 1, 2, 3, 4, 5, 6, 7, PNXN, I16MB, I8MB,xxx? InNxN,INxnN

    const int MapPCUTypeMin[14] = {1, 2, 3, 4, 3, 3, 4, 4, -1, 5, 5, 5, 5, 5};
    const int MapBCUTypeMin[14] = {1, 2, 3, 4, 3, 3, 4, 4, -1, 5, 5, 5, 5, 5};

    BiContextTypePtr pCTX = ctx->cuType_contexts;
    BiContextTypePtr pAMPCTX = ctx->amp_contexts;
    int max_bit = 0;
    int real_bit = 0 ;

    if (img->type != INTRA_IMG) {
        ///////////////////////  act_sym  calculation //////////////////////////////
        if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
            max_bit = 5;
        } else {
            max_bit = 6;
        }


        if (currMB->ui_MbBitSize == B8X8_IN_BIT) {
            act_sym = img->type == B_IMG ? MapBCUTypeMin[currMB->cuType] : MapPCUTypeMin[currMB->cuType];
        } else {
            act_sym = img->type == B_IMG ? MapBCUType[currMB->cuType] : MapPCUType[currMB->cuType];
        }


        if (currMB->cuType == PSKIPDIRECT  && currMB->cbp == 0) {
            act_sym = 0;
        }


        real_bit = act_sym;
        ///////////////////////  arithm coding  //////////////////////////////
        //      biari_encode_symbol ( eep_dp, act_sym == 0 , pCTX );
        //      if (act_sym!=0)
        //      {
        //          act_sym--;
        //          biari_encode_symbol(eep_dp, act_sym==0, pCTX=1);
        //      }

        act_ctx = 0;


        while (act_sym >= 1) {
            if ((binidx == 5) && (currMB->ui_MbBitSize != MIN_CU_SIZE_IN_BIT)) {
                biari_encode_symbol_final(eep_dp, 0);
            } else {
                biari_encode_symbol(eep_dp, 0, pCTX + act_ctx);
            }



            binidx++;

            act_sym--;
            act_ctx++;

            if (act_ctx >= 5) {
                act_ctx = 5;
            }
        }

        if (real_bit < max_bit) {
            if ((binidx == 5) && (currMB->ui_MbBitSize != MIN_CU_SIZE_IN_BIT)) {
                biari_encode_symbol_final(eep_dp, 1);
            } else {
                biari_encode_symbol(eep_dp, 1, pCTX + act_ctx);
            }
        }

        //for AMP
        if (currMB->ui_MbBitSize >= B16X16_IN_BIT)
            if (input->InterSearchAMP) {
                if (currMB->cuType == P2NXN || currMB->cuType == PNX2N) {
                    biari_encode_symbol(eep_dp, 1, pAMPCTX + 0);     //SMP - AMP signal bit
                } else if (currMB->cuType >= PHOR_UP && currMB->cuType <= PVER_RIGHT) {
                    biari_encode_symbol(eep_dp, 0, pAMPCTX + 0);     //SMP - AMP signal bit
                    biari_encode_symbol(eep_dp, !(currMB->cuType % 2), pAMPCTX + 1);    //AMP shape
                }
            }
    }




}



void writeCuTypeInfo_SFRAME(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{

    int  binidx = 0;

    int act_ctx = 0;
    int act_sym;

    SyntaxInfoContexts *ctx         = (img->currentSlice)->syn_ctx;

    BiContextTypePtr pCTX = ctx->cuType_contexts;
    int max_bit = 0;
    int real_bit = 0 ;

    max_bit = 2;

    act_sym = se->value1;

    real_bit = act_sym;

    act_ctx = 0;


    while (act_sym >= 1) {

        biari_encode_symbol(eep_dp, 0, pCTX + act_ctx);
        binidx++;

        act_sym--;
        act_ctx++;

    }

    if (real_bit < max_bit) {
        biari_encode_symbol(eep_dp, 1, pCTX + act_ctx);
    }



}


void writeCuTypeInfo_SDIP(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    int  binidx = 0;
    int act_ctx = 0;

    SyntaxInfoContexts *ctx         = (img->currentSlice)->syn_ctx;

    BiContextTypePtr pCTX = ctx->cuType_contexts;
    BiContextTypePtr pAMPCTX = ctx->amp_contexts;
    int max_bit = 0;
    int real_bit = 0 ;
    pCTX = ctx->cuType_contexts;
    act_ctx = 0;
    if ((currMB->ui_MbBitSize == 5 || currMB->ui_MbBitSize == 4) && currMB->trans_size == 1) {
        if (currMB->ui_MbBitSize == MIN_CU_SIZE_IN_BIT) {
            if (currMB->cuType == I8MB) {
                biari_encode_symbol(eep_dp, 0, pCTX + 15);
            } else {
                biari_encode_symbol(eep_dp, 1, pCTX + 15);
                if (currMB->cuType == InNxNMB) {
                    biari_encode_symbol(eep_dp, 1, pCTX + 16);
                } else if (currMB->cuType == INxnNMB) {
                    biari_encode_symbol(eep_dp, 0, pCTX + 16);
                }
            }
        } else {
            if (currMB->cuType == InNxNMB) {
                biari_encode_symbol(eep_dp, 1, pCTX + 15);
            } else if (currMB->cuType == INxnNMB) {
                biari_encode_symbol(eep_dp, 0, pCTX + 15);
            } else {
                printf("error currMB->cuType encoder ");
            }
        }
    }
}

/*!
***************************************************************************
* \brief
*    This function is used to arithmetically encode the 8x8 block
*    type info
***************************************************************************
*/
void writeB8TypeInfo(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    int act_ctx;
    int act_sym;

    SyntaxInfoContexts *ctx    = (img->currentSlice)->syn_ctx;
    BiContextTypePtr pCTX = ctx->b8_type_contexts;

    act_sym = se->value1;
    act_ctx = 0;

    if (act_sym & 0x02) {
        biari_encode_symbol(eep_dp, 1, pCTX + 0);
        act_ctx = 2;
    } else {
        biari_encode_symbol(eep_dp, 0, pCTX + 0);
        act_ctx = 1;
    }

    biari_encode_symbol(eep_dp, (unsigned char)(act_sym & 0x01), pCTX + act_ctx);

    if (se->value1 >= 3) {
        biari_encode_symbol(eep_dp, ((unsigned char)(act_sym & 0x4)) >> 2, pCTX + 3);
    }

}

void writeB8TypeInfo_dhp(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    int act_ctx;
    int act_sym;

    SyntaxInfoContexts *ctx    = (img->currentSlice)->syn_ctx;
    BiContextTypePtr pCTX = ctx->b8_type_dhp_contexts;

    act_sym = se->value1 == 0 ? 0 : 1;
    act_ctx = 0;

    biari_encode_symbol(eep_dp, act_sym, pCTX + act_ctx);

}

/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode a pair of
*    intra prediction modes of a given MB.
* \author
*    added by lzhang for AEC
****************************************************************************
*/
void writeIntraPredMode(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    BiContextTypePtr pCTX;
    int ctx;
    int symbol = se->value1;
    int value = symbol;
    pCTX = img->currentSlice->syn_ctx->l_intra_mode_contexts;
    ctx = 0;

    if (symbol >= 0) {
        biari_encode_symbol(eep_dp, 0, pCTX + ctx);
        biari_encode_symbol(eep_dp, (symbol & 0x10) >> 4, pCTX + ctx + 1);
        biari_encode_symbol(eep_dp, (symbol & 0x08) >> 3, pCTX + ctx + 2);
        biari_encode_symbol(eep_dp, (symbol & 0x04) >> 2, pCTX + ctx + 3);
        biari_encode_symbol(eep_dp, (symbol & 0x02) >> 1, pCTX + ctx + 4);
        biari_encode_symbol(eep_dp, (symbol & 0x01),         pCTX + ctx + 5);
    } else {
        biari_encode_symbol(eep_dp, 1, pCTX + ctx);
        biari_encode_symbol(eep_dp, symbol + 2, pCTX + ctx + 6);

    }
}

/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode the reference
*    parameter of a given MB.
****************************************************************************
*/

void writeRefFrame(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;

    int   act_ctx;
    int   act_sym;
    int **refframe_array = ((img->type == B_IMG) ? img->fw_refFrArr : hc->refFrArr);     //enc_picture->ref_idx[se->value2];
    int bslice = (img->type == B_IMG);




    int bw_flag = se->value2 & 0x01;

    act_ctx     = 0;
    se->context = act_ctx; // store context
    act_sym     = se->value1;
    //! Edit Start "AEC - SideInfo (NO MBAFF)" '10-01-2005 by Ning Zhang (McMaster Univ.)

    if (act_sym == 0) {
        biari_encode_symbol(eep_dp, 1, ctx->ref_no_contexts + act_ctx);    //modified by yy 09.6.10
    } else {
        biari_encode_symbol(eep_dp, 0, ctx->ref_no_contexts + act_ctx);

        if (bslice == 0) {
            BiContextTypePtr pCTX;
            pCTX = ctx->ref_no_contexts;
            act_sym--;
            act_ctx = 4;

            while (act_sym >= 1) {
                biari_encode_symbol(eep_dp, 0, pCTX + act_ctx);
                act_sym -= 1;
                act_ctx ++;

                if (act_ctx >= 5) {
                    act_ctx = 5;
                }
            }

            //if( se->value1<3 )
            if (se->value1 < img->num_of_references - 1) {
                biari_encode_symbol(eep_dp, 1, pCTX + act_ctx);
            }
        }
    }

    //! Edit End "AEC - SideInfo (NO MBAFF)" '10-01-2005 by Ning Zhang (McMaster Univ.)
}

/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode the motion
*    vector data of a B-frame MB.
****************************************************************************
*/
void writeMVD_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    int i = img->subblock_x;
    int j = img->subblock_y;

    int act_ctx;
    unsigned int act_sym;
    int mv_pred_res;
    int mv_sign;
    int list_idx = se->value2 & 0x01;
    int k = (se->value2 >> 1);
    int exp_golomb_order = 0;

    int uiBitSize = currMB->ui_MbBitSize;

    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;


    act_ctx = 0;

    mv_pred_res = se->value1;
    se->context = act_ctx;
    act_sym = absm(mv_pred_res);

    if (act_sym < 3) {   // 0, 1, 2
        if (act_sym == 0) {
            biari_encode_symbol(eep_dp, 0, &ctx->mvd_contexts[k][act_ctx]);
        } else if (act_sym == 1) {
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][act_ctx]);
            biari_encode_symbol(eep_dp, 0, &ctx->mvd_contexts[k][3]);
        } else if (act_sym == 2) {
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][act_ctx]);
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][3]);
            biari_encode_symbol(eep_dp, 0, &ctx->mvd_contexts[k][4]);
        }

    } else {
        if (act_sym % 2 == 1) {   //odds >3
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][act_ctx]);
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][3]);
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][4]);
            biari_encode_symbol_eq_prob(eep_dp, 0);
            act_sym = (act_sym - 3) / 2;

        } else { //even >3
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][act_ctx]);
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][3]);
            biari_encode_symbol(eep_dp, 1, &ctx->mvd_contexts[k][4]);
            biari_encode_symbol_eq_prob(eep_dp, 1);
            act_sym = (act_sym - 4) / 2;
        }

        // exp_golomb part
        while (1) {
            if (act_sym >= (unsigned int)(1 << exp_golomb_order)) {
                biari_encode_symbol_eq_prob(eep_dp, 0);
                act_sym = act_sym - (1 << exp_golomb_order);
                exp_golomb_order++;
            } else {
                biari_encode_symbol_eq_prob(eep_dp, 1);

                while (exp_golomb_order--) {   //next binary part
                    biari_encode_symbol_eq_prob(eep_dp, (unsigned char)((act_sym >> exp_golomb_order) & 1));
                }

                break;
            }
        }
    }

    if (mv_pred_res != 0) {
        mv_sign = (mv_pred_res >= 0) ? 0 : 1;
        biari_encode_symbol_eq_prob(eep_dp, (unsigned char) mv_sign);
    }
}
/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode the index of PMV candidate
****************************************************************************
*/
void writePMVindex_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    BiContextTypePtr pCTX = ctx->pmv_idx_contexts[0];
    int counter = 0;
    int act = 0;
    if (se->value1 == 0) {
        biari_encode_symbol(eep_dp, 1, pCTX);

    } else {
        biari_encode_symbol(eep_dp, 0, pCTX);
        counter++;
        act++;
        while (counter < se->value1) {

            biari_encode_symbol(eep_dp, 0, pCTX + act);
            counter++;
            act++;
        }
        assert(se->value1 == counter);
        if (se->value1 == se->value2 - 1) {
            return;
        } else {
            biari_encode_symbol(eep_dp, 1, pCTX + act);
            counter++;
            act++;
        }


    }
}

int writeBRPFlag(int brpFlag, codingUnit *currMB, int uiBitSize)
{
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    int            *bitCount  = currMB->bitcounter;
    Slice          *currSlice = img->currentSlice;
    DataPartition  *dataPart  ;
    int rate = 0;
    dataPart = & (currSlice->partArr[0]);

    currSE->value1 = brpFlag;
    currSE->value2 = uiBitSize;
    currSE->writing = writeBRPFlag_AEC;
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, 0);    //img->current_mb_nr); //check: img->current_mb_nr

#if TRACE

    if (he->AEC_writting) {
        fprintf(hc->p_trace, "splitflag = %3d\n", brpFlag);
    }

#endif
    rate = currSE->len;
    bitCount[BITS_INTER_MB] += currSE->len;
    return rate;
}

void writeBRPFlag_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    int   act_sym = se->value1;

    if (act_sym == 0) {
        biari_encode_symbol(eep_dp, 1, ctx->brp_contexts + se->value2);
    } else {
        biari_encode_symbol(eep_dp, 0, ctx->brp_contexts + se->value2);
    }
}


void writeDmhMode(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    BiContextTypePtr pCTX = img->currentSlice->syn_ctx->mvd_contexts[2];

    int dmh_mode = se->value1;
    int d = dmh_mode / DMH_MODE_NUM; // distance flag

    int iEncMapTab[9] = {0, 5, 6, 1, 2, 7, 8, 3, 4};
    int iMapVal = iEncMapTab[dmh_mode];
    int iSymbol = (iMapVal == 0) ? 0 : 1;

    pCTX += (se->value2 - 3) * 4;
    biari_encode_symbol(eep_dp, iSymbol, pCTX);

    if (iSymbol) {
        if (iMapVal < 3) {
            iSymbol = (iMapVal == 1) ? 0 : 1;
            biari_encode_symbol(eep_dp, 0, pCTX + 1);
            biari_encode_symbol_eq_prob(eep_dp, iSymbol);
        } else if (iMapVal < 5) {
            iSymbol = (iMapVal == 3) ? 0 : 1;
            biari_encode_symbol(eep_dp, 1, pCTX + 1);
            biari_encode_symbol(eep_dp, 0, pCTX + 2);
            biari_encode_symbol_eq_prob(eep_dp, iSymbol);
        } else {
            biari_encode_symbol(eep_dp, 1, pCTX + 1);
            biari_encode_symbol(eep_dp, 1, pCTX + 2);
            iSymbol = (iMapVal < 7) ? 0 : 1;
            biari_encode_symbol_eq_prob(eep_dp, iSymbol);
            iSymbol = (iMapVal % 2) ? 0 : 1;
            biari_encode_symbol_eq_prob(eep_dp, iSymbol);
        }
    }

    return;
}



void writeTrSize(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{

    BiContextTypePtr pCTX;
    int   act_sym = se->value1;
    int   act_ctx = se->value2;
    pCTX  = img->currentSlice->syn_ctx->tu_contexts;
    if (currMB ->ui_MbBitSize == B8X8_IN_BIT || ((currMB ->ui_MbBitSize == B32X32_IN_BIT ||
            currMB ->ui_MbBitSize == B16X16_IN_BIT) && input->useSDIP)) {
        if (currMB->ui_MbBitSize == B32X32_IN_BIT || currMB->ui_MbBitSize == B16X16_IN_BIT) {
            act_ctx++;
        }
        biari_encode_symbol(eep_dp, act_sym, pCTX + act_ctx);
    }
}



void writePdir(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{

    int pdir0   = currMB->b8pdir[0];
    int pdir1   = currMB->b8pdir[3];
    int act_ctx = 0;
    int act_sym;
    int symbol;
    int new_pdir[4] = {2, 1, 3, 0};
    SyntaxInfoContexts *ctx         = (img->currentSlice)->syn_ctx;

    BiContextTypePtr pCTX;

    if ((currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT) && currMB->ui_MbBitSize == B8X8_IN_BIT) {
        pCTX = ctx->pdirMin_contexts;
    } else {
        pCTX = ctx->pdir_contexts;
    }


    if (currMB->cuType == P2NX2N) {
        act_sym = pdir0;
        while (act_sym >= 1) {
            biari_encode_symbol(eep_dp, 0, pCTX + act_ctx);
            act_sym--;
            act_ctx++;
        }
        if (pdir0 != 3) {
            biari_encode_symbol(eep_dp, 1, pCTX + act_ctx);
        }

    } else if ((currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT) && currMB->ui_MbBitSize == B8X8_IN_BIT) {
        pdir0 = new_pdir[pdir0];
        pdir1 = new_pdir[pdir1];

        act_sym = pdir0;

        if (act_sym == 1) { // BW
            biari_encode_symbol(eep_dp, 0, pCTX + act_ctx);
        } else {           // FW
            biari_encode_symbol(eep_dp, 1, pCTX + act_ctx);
        }

        act_ctx = 1;

        symbol = (pdir0 == pdir1);

        biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
    } else if (currMB->cuType >= P2NXN || currMB->cuType <= PVER_RIGHT) { //1010
        pdir0 = new_pdir[pdir0];
        pdir1 = new_pdir[pdir1];
        act_sym = pdir0;
        while (act_sym >= 1) {
            biari_encode_symbol(eep_dp, 0, pCTX + act_ctx + 4);
            act_sym--;
            act_ctx++;
        }
        if (pdir0 != 3) {
            biari_encode_symbol(eep_dp, 1, pCTX + act_ctx + 4);
        }

        act_ctx = 8;
        symbol = (pdir0 == pdir1);
        biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);

        if (!symbol) {
            switch (pdir0) {
            case 0:
                act_ctx = 9;
                symbol = (pdir1 == 1);
                biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                if (!symbol) {
                    act_ctx = 10;
                    symbol = (pdir1 == 2);
                    biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                }
                break;


            case 1:
                act_ctx = 11;
                symbol = (pdir1 == 0);
                biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                if (!symbol) {
                    act_ctx = 12;
                    symbol = (pdir1 == 2);
                    biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                }
                break;

            case 2:
                act_ctx = 13;
                symbol = (pdir1 == 0);
                biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                if (!symbol) {
                    act_ctx = 14;
                    symbol = (pdir1 == 1);
                    biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                }
                break;

            case 3:
                act_ctx = 15;
                symbol = (pdir1 == 0);
                biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                if (!symbol) {
                    act_ctx = 16;
                    symbol = (pdir1 == 1);
                    biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx);
                }
                break;
            }
        }

    }


}




void writePdir_dhp(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    int pdir0   = currMB->b8pdir[0];
    int pdir1   = currMB->b8pdir[3];
    int act_ctx = 0;
    int symbol;
    SyntaxInfoContexts *ctx         = (img->currentSlice)->syn_ctx;
    BiContextTypePtr pCTX           = ctx->pdir_dhp_contexts;

    pdir0 = (pdir0 == 0) ? 0 : 1;
    pdir1 = (pdir1 == 0) ? 0 : 1;


    if (currMB->cuType == P2NX2N) {
        biari_encode_symbol(eep_dp, pdir0, pCTX + act_ctx);
    } else if (currMB->cuType >= P2NXN || currMB->cuType <= PVER_RIGHT) {   //1010
        biari_encode_symbol(eep_dp, pdir0, pCTX + act_ctx + 1);

        symbol = (pdir0 == pdir1);
        biari_encode_symbol(eep_dp, symbol, pCTX + act_ctx + 2);
    }
}


void writeWPM(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    int i, binIdx = 0;
    int ref = se->value1;
    BiContextTypePtr pCTX = img->currentSlice->syn_ctx->wpm_contexts;

    if (ref == img->num_of_references - 1) {
        for (i = 0; i < ref; i++) {
            biari_encode_symbol(eep_dp, 0, pCTX + binIdx);
            binIdx = min(binIdx + 1, 2);
        }
    } else {
        for (i = 0; i < ref; i++) {
            biari_encode_symbol(eep_dp, 0, pCTX + binIdx);
            binIdx = min(binIdx + 1, 2);
        }
        biari_encode_symbol(eep_dp, 1, pCTX + binIdx);
    }
}

void write_b_dir_skip(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts *ctx         = (img->currentSlice)->syn_ctx;
    BiContextTypePtr pCTX           = ctx->b_dir_skip_contexts;
    int symbol = currMB->md_directskip_mode;
    int offset = 0;

    for (offset = 0; offset < symbol; offset++) {
        biari_encode_symbol(eep_dp, 0, pCTX + offset);
    }
    if (symbol < DIRECTION) {
        biari_encode_symbol(eep_dp, 1, pCTX + offset);
    }
}

void write_p_skip_mode(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts *ctx         = (img->currentSlice)->syn_ctx;
    BiContextTypePtr pCTX           = ctx->p_skip_mode_contexts;
    int symbol = currMB->md_directskip_mode;
    int offset = 0;

    for (offset = 0; offset < symbol; offset++) {
        biari_encode_symbol(eep_dp, 0, pCTX + offset);
    }
    if (symbol < MH_PSKIP_NUM) {
        biari_encode_symbol(eep_dp, 1, pCTX + offset);
    }
}

void writePartBiPredictionFlag(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    biari_encode_symbol_eq_prob(eep_dp, (unsigned char) se->value1);
}


/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode the chroma
*    intra prediction mode of an 8x8 block
****************************************************************************
*/
void writeCIPredMode(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts *ctx     = img->currentSlice->syn_ctx;
    //codingUnit          *currMB  = &img->mb_data[img->current_mb_nr];
    int                 act_ctx, a, b;
    int                 act_sym  = se->value1;
    int lmode = 0;
    Boolean is_redundant = FALSE;
    int block8_y = (uiPosition / img->PicWidthInMbs) << 1;
    int block8_x = (uiPosition % img->PicWidthInMbs) << 1;
    int LumaMode = img->ipredmode[1 + block8_y][1 + block8_x];

    if (LumaMode == VERT_PRED || LumaMode == HOR_PRED || LumaMode == DC_PRED || LumaMode == BI_PRED) {
        lmode = LumaMode == VERT_PRED ? VERT_PRED_C : (LumaMode == HOR_PRED ? HOR_PRED_C :
                (LumaMode == DC_PRED ?  DC_PRED_C : BI_PRED_C));
        is_redundant = TRUE;
    }
    if (currMB->mb_available_up == NULL) {
        b = 0;
    } else {
        b = (((currMB->mb_available_up)->c_ipred_mode != 0) ? 1 : 0);
    }

    if (currMB->mb_available_left == NULL) {
        a = 0;
    } else {
        a = (((currMB->mb_available_left)->c_ipred_mode != 0) ? 1 : 0);
    }
    act_ctx = a;

    if (act_sym == 0) {
        biari_encode_symbol(eep_dp, 1, ctx->c_intra_mode_contexts + act_ctx);
    } else {
        biari_encode_symbol(eep_dp, 0, ctx->c_intra_mode_contexts + act_ctx);
        if (is_redundant) {
            if (act_sym > lmode) {
                act_sym  =  act_sym - 2 ;
            } else {
                act_sym  =  act_sym - 1 ;
            }
        } else {
            act_sym = act_sym - 1;
        }



        unary_bin_max_encode(eep_dp, (unsigned int) act_sym , ctx->c_intra_mode_contexts + 3, 0, 3);
    }
}

/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode the coded
*    block pattern of an 8x8 block
****************************************************************************
*/
void writeCbpBit(int b8, int bit, int cbp, codingUnit *currMB, int inter, EncodingEnvironmentPtr eep_dp,
                 int uiPosition)
{
    int a, b;
    int mb_x = b8 % 2 ;
    int mb_y = b8 / 2 ;
    PixelPos block_a, block_b;
    codingUnit *upMB, *leftMB;
    BiContextTypePtr pCTX;
    int uiBitSize = currMB->ui_MbBitSize;
    int width = (1 << uiBitSize);
    int height = (1 << uiBitSize);
    int x, y, sizeOfNeighborBlock;
    codingUnit *neighborMB;
    int block_a_nsqt_hor = 0, block_a_nsqt_ver = 0;
    int block_b_nsqt_hor = 0, block_b_nsqt_ver = 0;
    int currMB_hor = ((currMB->cuType == P2NXN && uiBitSize > 3 || currMB->cuType == PHOR_UP ||
                       currMB->cuType == PHOR_DOWN) && currMB->trans_size == 1 && input->useNSQT == 1 || currMB->cuType == InNxNMB);
    int currMB_ver = ((currMB->cuType == PNX2N && uiBitSize > 3 || currMB->cuType == PVER_LEFT ||
                       currMB->cuType == PVER_RIGHT) && currMB->trans_size == 1 && input->useNSQT == 1 || currMB->cuType == INxnNMB);
    if (b8 == 4) {
        mb_x = mb_y = 0;
    }

    mb_x = (mb_x == 0) ? 0 : 1 ;
    mb_y = (mb_y == 0) ? 0 : 1 ;

    if (currMB_hor) {
        x = -1;
        y = height * (mb_x + mb_y * 2) / 4 + 0;
    } else if (currMB_ver) {
        x = width * (mb_x + mb_y * 2) / 4 - 1;
        y = 0;
    } else {
        x = width * mb_x / 2  - 1; //qyu night
        y = height * mb_y / 2 + 0;
    }
    getNeighbour(x, y, 1,  &block_a, uiPosition, uiBitSize, currMB);

    if (currMB_hor) {
        x = 0;
        y = height * (mb_x + mb_y * 2) / 4 - 1;
    } else if (currMB_ver) {
        x = width * (mb_x + mb_y * 2) / 4 + 0;
        y = -1;
    } else {
        x = width * mb_x / 2  + 0; //qyu night
        y = height * mb_y / 2  - 1;
    }

    getNeighbour(x, y, 1,  &block_b, uiPosition, uiBitSize, currMB);
    if (block_a.available) {

        if (block_a.mb_addr == uiPosition) {
            neighborMB = currMB;
        } else {
            neighborMB = &img->mb_data[block_a.mb_addr];
        }

        sizeOfNeighborBlock = neighborMB->ui_MbBitSize;
        block_a_nsqt_hor = ((neighborMB->cuType == P2NXN && neighborMB->ui_MbBitSize > 3 || neighborMB->cuType == PHOR_UP ||
                             neighborMB->cuType == PHOR_DOWN) && neighborMB->trans_size == 1 && input->useNSQT == 1 ||
                            neighborMB->cuType == InNxNMB);
        block_a_nsqt_ver = ((neighborMB->cuType == PNX2N && neighborMB->ui_MbBitSize > 3 || neighborMB->cuType == PVER_LEFT ||
                             neighborMB->cuType == PVER_RIGHT) && neighborMB->trans_size == 1 && input->useNSQT == 1 ||
                            neighborMB->cuType == INxnNMB);
        if (block_a_nsqt_hor) {
            block_a.x = 0;
            block_a.y = block_a.y / ((1 << sizeOfNeighborBlock) / 4);

        } else if (block_a_nsqt_ver) {
            block_a.x = block_a.x / ((1 << sizeOfNeighborBlock) / 4);
            block_a.y = 0;
        } else {
            block_a.x = block_a.x / ((1 << sizeOfNeighborBlock) / 2);
            block_a.y = block_a.y / ((1 << sizeOfNeighborBlock) / 2);
        }
    }

    if (block_b.available) {

        if (block_b.mb_addr == uiPosition) {
            neighborMB = currMB;
        } else {
            neighborMB = &img->mb_data[block_b.mb_addr];
        }

        sizeOfNeighborBlock = neighborMB->ui_MbBitSize;
        block_b_nsqt_hor = ((neighborMB->cuType == P2NXN && neighborMB->ui_MbBitSize > 3 || neighborMB->cuType == PHOR_UP ||
                             neighborMB->cuType == PHOR_DOWN) && neighborMB->trans_size == 1 && input->useNSQT == 1 ||
                            neighborMB->cuType == InNxNMB);
        block_b_nsqt_ver = ((neighborMB->cuType == PNX2N && neighborMB->ui_MbBitSize > 3 || neighborMB->cuType == PVER_LEFT ||
                             neighborMB->cuType == PVER_RIGHT) && neighborMB->trans_size == 1 && input->useNSQT == 1 ||
                            neighborMB->cuType == INxnNMB);
        if (block_b_nsqt_hor) {
            block_b.x = 0;
            block_b.y = block_b.y / ((1 << sizeOfNeighborBlock) / 4);

        } else if (block_b_nsqt_ver) {
            block_b.x = block_b.x / ((1 << sizeOfNeighborBlock) / 4);
            block_b.y = 0;
        } else {
            block_b.x = block_b.x / ((1 << sizeOfNeighborBlock) / 2);
            block_b.y = block_b.y / ((1 << sizeOfNeighborBlock) / 2);
        }
    }

    if (block_b.mb_addr == uiPosition) {
        upMB = currMB;
    } else {
        upMB = &img->mb_data[block_b.mb_addr];
    }

    if (block_a.mb_addr == uiPosition) {
        leftMB = currMB;
    } else {
        leftMB = &img->mb_data[block_a.mb_addr];
    }

    if (b8 == 4) {
        pCTX = img->currentSlice->syn_ctx->cbp_contexts[2];
    } else {
        if (block_a.available) {
            if (block_a_nsqt_hor) {
                a = (leftMB->cbp & (1 << (block_a.y))) != 0 ;
            } else if (block_a_nsqt_ver) {
                a = (leftMB->cbp & (1 << (block_a.x))) != 0 ;
            } else {
                a = (leftMB->cbp & (1 << (block_a.x + block_a.y * 2))) != 0 ;
            }
        } else {
            a = 0;
        }
        if (block_b.available) {
            if (block_b_nsqt_hor) {
                b = (upMB->cbp & (1 << (block_b.y))) != 0 ;
            } else if (block_b_nsqt_ver) {
                b = (upMB->cbp & (1 << (block_b.x))) != 0 ;
            } else {
                b = (upMB->cbp & (1 << (block_b.x + block_b.y * 2))) != 0 ;
            }
        } else {
            b = 0;
        }
        pCTX = img->currentSlice->syn_ctx->cbp_contexts[0] + a + 2 * b;
    }

    //===== WRITE BIT =====
    biari_encode_symbol(eep_dp, (unsigned char) bit, pCTX);      //+inter*4
}

/*!
****************************************************************************
* \brief
*    This function is used to arithmetically encode the coded
*    block pattern of a codingUnit
***************************************************************************
*/
void writeCBP(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts *ctx = img->currentSlice->syn_ctx;
    int cbp = se->value1; // symbol to encode
    int cbp_bit;
    int b8;
#if MB_DQP
    if (!cbp) {
        last_dquant = 0;
    }
#endif

    if (!IS_INTRA(currMB)) {
        if (!(IS_DIRECT(currMB) || IS_P_SKIP(currMB))) {
            writeCbpBit(4, cbp == 0 , cbp, currMB, 4 , eep_dp, uiPosition);
        }
        if (cbp != 0) {
            unsigned char chroma0 = (cbp > 15) ? 1 : 0;;
            //////////////////////////////   tr_size   /////////////////////////////////
            biari_encode_symbol(eep_dp, (unsigned char) currMB->trans_size, ctx->tu_contexts);

            //////////////////////////////   chroma==0   /////////////////////////////////
            biari_encode_symbol(eep_dp, (unsigned char) chroma0, ctx->cbp_contexts[1]);// 0
            if (currMB->trans_size == 0) {
                if (chroma0 == 0) {
                    return;
                } else {
                    //////////////////////////////   chroma   /////////////////////////////////
                    if (cbp > 15) {
                        cbp_bit = (cbp > 47) ? 1 : 0;           //changed by lzhang
                        biari_encode_symbol(eep_dp, (unsigned char) cbp_bit, ctx->cbp_contexts[1] + 2);     // 2

                        if (cbp <= 47) {
                            cbp_bit = ((cbp >> 4) == 2) ? 1 : 0;
                            biari_encode_symbol(eep_dp, (unsigned char) cbp_bit, ctx->cbp_contexts[1] + 2);     // 2
                        }
                    }
                    ////////////////////////////////   luma  ////////////////////////////////
                    writeCbpBit(0, (cbp & 1) != 0 , cbp, currMB, 1, eep_dp, uiPosition);
                }
            } else {
                //////////////////////////////   chroma   /////////////////////////////////
                if (cbp > 15) {
                    cbp_bit = (cbp > 47) ? 1 : 0;           //changed by lzhang
                    biari_encode_symbol(eep_dp, (unsigned char) cbp_bit, ctx->cbp_contexts[1] + 2);     // 2

                    if (cbp <= 47) {
                        cbp_bit = ((cbp >> 4) == 2) ? 1 : 0;
                        biari_encode_symbol(eep_dp, (unsigned char) cbp_bit, ctx->cbp_contexts[1] + 2);     // 2
                    }
                }
                ////////////////////////////////   luma  ////////////////////////////////
                writeCbpBit(0, (cbp & 1) != 0, cbp, currMB, 1, eep_dp, uiPosition);
                writeCbpBit(1, (cbp & 2) != 0, cbp, currMB, 1, eep_dp, uiPosition);
                writeCbpBit(2, (cbp & 4) != 0, cbp, currMB, 1, eep_dp, uiPosition);
                writeCbpBit(3, (cbp & 8) != 0, cbp, currMB, 1, eep_dp, uiPosition);
            }
        }

    } else {
        //////////////////////////  intra luma  /////////////////////////////////
        if (currMB->trans_size == 0 || currMB->cuType == I16MB) {
            b8 = 0;
            writeCbpBit(b8, (cbp & (0xF)) != 0, cbp, currMB, 0, eep_dp, uiPosition);
        } else {
            for (b8 = 0; b8 < 4; b8++) {
                writeCbpBit(b8, (cbp & (1 << b8)) != 0, cbp, currMB, 0, eep_dp, uiPosition);
            }
        }

        //////////////////////////  intra chroma  /////////////////////////////////
        cbp_bit = (cbp > 15) ? 1 : 0;
        biari_encode_symbol(eep_dp, (unsigned char) cbp_bit, ctx->cbp_contexts[1] + 1);     // 1

        if (cbp > 15) {
            cbp_bit = (cbp > 47) ? 1 : 0;           //changed by lzhang
            biari_encode_symbol(eep_dp, (unsigned char) cbp_bit, ctx->cbp_contexts[1] + 3);     // 3

            if (cbp <= 47) {
                cbp_bit = ((cbp >> 4) == 2) ? 1 : 0;
                biari_encode_symbol(eep_dp, (unsigned char) cbp_bit, ctx->cbp_contexts[1] + 3);     // 3
            }
        }
    }
}




/////////////////////////// write DQP
#if MB_DQP
void writeDqp(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts *ctx = img->currentSlice->syn_ctx;
    int act_ctx;
    int act_sym;
    int dquant;

    act_ctx = ((last_dquant != 0) ? 1 : 0);
    //BiContextTypePtr qpCTX = ctx->delta_qp_contexts;
    dquant = currMB->delta_qp;

    if (dquant > 0) {
        act_sym = 2 * dquant - 1;
    } else {
        act_sym = -2 * dquant;
    }

    if (act_sym == 0) {
        biari_encode_symbol(eep_dp, 1, ctx->delta_qp_contexts + act_ctx);
    } else {
        biari_encode_symbol(eep_dp, 0, ctx->delta_qp_contexts + act_ctx);
        act_ctx = 2;
        if (act_sym == 1) {
            biari_encode_symbol(eep_dp, 1, ctx->delta_qp_contexts + act_ctx);
        } else {
            biari_encode_symbol(eep_dp, 0, ctx->delta_qp_contexts + act_ctx);
            act_ctx++;
            while (act_sym > 2) {
                biari_encode_symbol(eep_dp, 0, ctx->delta_qp_contexts + act_ctx);
                act_sym--;
            }
            biari_encode_symbol(eep_dp, 1, ctx->delta_qp_contexts + act_ctx);
        }
    }

    last_dquant = dquant;
}
#endif
/*!
****************************************************************************
* \brief
*    Write coefficients
****************************************************************************
*/
int DCT_Pairs = 0;
int DCT_Level[MAX_CU_SIZE * MAX_CU_SIZE + 1] = {0};
int DCT_Run[MAX_CU_SIZE * MAX_CU_SIZE + 1] = {0};
int DCT_CGFlag[CG_SIZE * CG_SIZE] = {0};
int DCT_PairsInCG[CG_SIZE * CG_SIZE] = {0};
int DCT_CGLastRun[CG_SIZE * CG_SIZE];


const int T_Chr[5] = { 0, 1, 2, 4, 3000};


int estRunLevelRef(codingUnit *currMB,  int context)
{
    int pairs = 0, rank = 0, pos = 0;
    int  absLevel = 0, symbol = 0;
    int rate = 0;
    int numOfCoeff = (1 << (currMB->ui_MbBitSize - 1)) * (1 << (currMB->ui_MbBitSize - 1));
    int bitSize = currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT;
    int k, Level, Run;


    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && ((currMB->trans_size == 1) && (currMB->ui_MbBitSize == 6) &&
            (IS_INTER(currMB)) && (currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT))) {
        numOfCoeff = (1 << (currMB->ui_MbBitSize - 2)) * (1 << (currMB->ui_MbBitSize - 2));
        bitSize -= 1;
    }

    //! Accumulate Run-Level Information

    Level = 1;
    for (k = 0; k <= numOfCoeff && Level != 0; k++) {
        Level = DCT_Level[k] = he->alllevel[k]; // level
        Run = DCT_Run[k] = he->allrun[k]; // run
        DCT_Pairs++;
    }


    if (DCT_Pairs > 0) {
        BiContextType(*Primary)[NUM_MAP_CTX];
        BiContextTypePtr pCTX;
        BiContextTypePtr pCTX2;
        //codingUnit   *currMB      = &img->mb_data[img->current_mb_nr];
        int ctx = 0, offset = 0, ctx2 = 0;

        if (context == LUMA_8x8) {
            Primary = img->currentSlice->syn_ctx->map_contexts;
        } else {
            Primary = img->currentSlice->syn_ctx->last_contexts;

        }
        pairs = DCT_Pairs;
        rank = 0;
        pos  = 0;
        for (; pairs >= 0; pairs--) {
            if (pairs == 0) {
                Level = 0;
                Run = 0;
            } else {
                Level = DCT_Level[pairs - 1];
                Run = DCT_Run[pairs - 1];
            }
            absLevel = abs(Level);
            pCTX = Primary[rank];
            //! EOB
            if (rank > 0) {
                pCTX2 = Primary[5 + (pos >> (5 + bitSize * 2))];
                ctx2 = (pos >> 1) & 0x0f;
                ctx = 0;
                if (absLevel == 0) {
                    rate += biari_encode_symbolW_est(1, pCTX + ctx, pCTX2 + ctx2);
                    break;
                } else {
                    rate += biari_encode_symbolW_est(0, pCTX + ctx, pCTX2 + ctx2);
                }
            }
            //! Level
            symbol = absLevel - 1;
            ctx = 1;
            while (symbol >= 1) {
                symbol -= 1;
                rate += biari_encode_symbol_est(0, pCTX + ctx);

                ctx ++;
                if (ctx >= 2) {
                    ctx = 2;
                }
            }
            rate += biari_encode_symbol_est(1, pCTX + ctx);
            //! Sign
            if (Level < 0) {
                rate += biari_encode_symbol_eq_prob_est(1);
            } else {
                rate += biari_encode_symbol_eq_prob_est(0);
            }
            //! Run
            if (absLevel == 1) {
                offset = 4;
            } else {
                offset = 6;
            }
            symbol = Run;
            ctx = 0;
            while (symbol >= 1) {
                symbol -= 1;
                rate += biari_encode_symbol_est(0, pCTX + ctx + offset);
                ctx ++;
                if (ctx >= 1) {
                    ctx = 1;
                }
            }
            rate += biari_encode_symbol_est(1, pCTX + ctx + offset);
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
            //! Update position
            pos += (Run + 1);
            if (pos >= numOfCoeff) {
                pos = numOfCoeff - 1;
            }
        }
    }
    //--- Reset Counters ---

    DCT_Pairs = 0;


    return rate;
}

// 0: INTRA_PRED_VER
// 1: INTRA_PRED_HOR
// 2: INTRA_PRED_DC_DIAG

int g_intraModeClassified[NUM_INTRA_PMODE] = {2, 2, 2, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 0
                                             };

void writeRunLevelRef(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp , int uiPosition)
{
    int pairs = 0, rank = 0, pos = 0;
    int Run = 0, Level = 0, absLevel = 0, symbol = 0;
    int iCG, numOfCG, DCT_CGNum = 0;
    int i, numOfCoeffInCG;
    int numFirstCG = 0;
    int offsetFirstCG = 0;
    int pairsInCG = 0;

    int absSum5, n, k;
    int xx, yy;
    int CGLastX = 0, CGLastY = 0;
    int CGx = 0, CGy = 0;
    int px, py, moddiv, indiv, ctxpos;
    int isChroma = 0;
    int ctxmode = INTRA_PRED_DC_DIAG;

    int numOfCoeff = (1 << (currMB->ui_MbBitSize - 1)) * (1 << (currMB->ui_MbBitSize - 1));
    int bitSize = currMB->ui_MbBitSize - MIN_CU_SIZE_IN_BIT;

    int exp_golomb_order = 0;
    int bins = 0;

    int Level_sign[17] = { -1};

    if ((currMB->trans_size == 0 || currMB->cuType == I16MB) && currMB->ui_MbBitSize != MAX_CU_SIZE_IN_BIT  &&
        se->context == LUMA_8x8) {
        numOfCoeff = (1 << currMB->ui_MbBitSize) * (1 << currMB->ui_MbBitSize);
        bitSize += 1;
    }
    if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && ((currMB->trans_size == 1) &&
            (currMB->ui_MbBitSize == 6) && (IS_INTER(currMB)) && (currMB->cuType >= P2NXN && currMB->cuType <= PVER_RIGHT) &&
            (se->context == LUMA_8x8))) {
        numOfCoeff = (1 << (currMB->ui_MbBitSize - 2)) * (1 << (currMB->ui_MbBitSize - 2));
        bitSize -= 1;
    }


    if (se->context == LUMA_8x8 && se->type == SE_LUM_AC_INTRA) {
        assert(currMB->l_ipred_mode < NUM_INTRA_PMODE);
        ctxmode = g_intraModeClassified[currMB->l_ipred_mode];
        if (ctxmode == INTRA_PRED_HOR) {
            ctxmode = INTRA_PRED_VER;
        }
    }
    //! Accumulate Run-Level Information
    Level = 1;
    for (k = 0; k <= numOfCoeff && Level != 0; k++) {
        Level = DCT_Level[k] = he->alllevel[k]; // level
        Run = DCT_Run[k] = he->allrun[k]; // run
        DCT_Pairs++;
    }
    DCT_Pairs --;


    numOfCG = (numOfCoeff >> 4);
    numOfCoeffInCG = 16;

    if (DCT_Pairs > 0) {
        BiContextType(*Primary) [NUM_MAP_CTX];
        BiContextTypePtr pCTX;
        BiContextTypePtr pCTXSigCG;
        BiContextTypePtr pCTXLastCG;
        BiContextTypePtr pCTXLastPosInCG;
        int ctx = 0, offset = 0, ctx2 = 0, sigCGctx = 0, firstCGctx = 0;

        if (se->context == LUMA_8x8) {
            Primary = img->currentSlice->syn_ctx->map_contexts;
            pCTXSigCG = img->currentSlice->syn_ctx->sigCG_contexts;
            pCTXLastCG = img->currentSlice->syn_ctx->lastCG_contexts;
            pCTXLastPosInCG = img->currentSlice->syn_ctx->lastPos_contexts;
        } else {
            isChroma = 1;
            Primary = img->currentSlice->syn_ctx->last_contexts;
            pCTXSigCG = img->currentSlice->syn_ctx->sigCG_contexts + NUM_SIGCG_CTX_LUMA;
            pCTXLastCG = img->currentSlice->syn_ctx->lastCG_contexts + NUM_LAST_CG_CTX_LUMA;
            pCTXLastPosInCG = img->currentSlice->syn_ctx->lastPos_contexts + NUM_LAST_POS_CTX_LUMA;
        }

        pairs = DCT_Pairs;
        rank = 0;

        for (iCG = numOfCG - 1; iCG >= 0; iCG --) {
            //! Last CG Position
            if (rank == 0 && DCT_CGFlag[ iCG ] && bitSize) {
                int numCGminus1 = numOfCG - 1;
                int count = 0;
                int offset;
                int numCGminus1X;
                int numCGminus1Y;

                if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                        (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                      (currMB->cuType == PHOR_DOWN)))) {
                    numCGminus1X = (1 << (bitSize + 1)) - 1;
                    numCGminus1Y = (1 << (bitSize - 1)) - 1;
                } else if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                           (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                   (currMB->cuType == PVER_RIGHT)))) {
                    numCGminus1X = (1 << (bitSize - 1)) - 1;
                    numCGminus1Y = (1 << (bitSize + 1)) - 1;
                } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                              ((currMB->cuType == InNxNMB)))) {

                    numCGminus1X = (1 << (bitSize + 1)) - 1;
                    numCGminus1Y = (1 << (bitSize - 1)) - 1;
                } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                              ((currMB->cuType == INxnNMB)))) {
                    numCGminus1X = (1 << (bitSize - 1)) - 1;
                    numCGminus1Y = (1 << (bitSize + 1)) - 1;
                } else {
                    numCGminus1X = numCGminus1Y = (1 << bitSize) - 1;
                }
                if (bitSize == 0) {
                    // no need to encode LastCG0Flag.
                } else if (bitSize == 1) { //8x8

                    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                            (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                          (currMB->cuType == PHOR_DOWN)))) {
                        CGLastX = iCG;
                        CGLastY = 0;
                    } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                               (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                       (currMB->cuType == PVER_RIGHT)))) {
                        CGLastX = 0;
                        CGLastY = iCG ;
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == InNxNMB)))) {
                        CGLastX = iCG;
                        CGLastY = 0;
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == INxnNMB)))) {
                        CGLastX = 0;
                        CGLastY = iCG ;
                    } else {
                        CGLastX = iCG & 1;
                        CGLastY = iCG / 2;
                    }

                    count = 0;
                    while (count < iCG) {
                        biari_encode_symbol(eep_dp, 0, pCTXLastCG + count);
                        count++;
                    }
                    if (iCG < 3) {
                        biari_encode_symbol(eep_dp, 1, pCTXLastCG + iCG);
                    }
                } else {
                    if (bitSize == 2) { //16x16
                        if ((input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && IS_INTER(currMB) && !(isChroma) &&
                             (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                           (currMB->cuType == PHOR_DOWN)))) {
                            CGLastX = AVS_SCAN2x8[iCG][0];
                            CGLastY = AVS_SCAN2x8[iCG][1];
                        } else if ((input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && IS_INTER(currMB) && !(isChroma) &&
                                    (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                            (currMB->cuType == PVER_RIGHT)))) {
                            CGLastX = AVS_SCAN8x2[iCG][0];
                            CGLastY = AVS_SCAN8x2[iCG][1];
                        } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                      ((currMB->cuType == InNxNMB)))) {
                            CGLastX = AVS_SCAN2x8[iCG][0];
                            CGLastY = AVS_SCAN2x8[iCG][1];
                        } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                      ((currMB->cuType == INxnNMB)))) {
                            CGLastX = AVS_SCAN8x2[iCG][0];
                            CGLastY = AVS_SCAN8x2[iCG][1];
                        } else {
                            CGLastX = AVS_SCAN4x4[iCG][0];
                            CGLastY = AVS_SCAN4x4[iCG][1];
                        }
                    } else if (bitSize == 3) { //32x32
                        {
                            CGLastX = AVS_SCAN8x8[iCG][0];
                            CGLastY = AVS_SCAN8x8[iCG][1];
                        }
                    }
                    if (se->context == LUMA_8x8 && ctxmode == INTRA_PRED_DC_DIAG) {
                        SWAP(CGLastX, CGLastY);
                        SWAP(numCGminus1X, numCGminus1Y);
                    }

                    offset = isChroma ? 3 : 9;
                    if (CGLastX == 0 && CGLastY == 0) {

                        biari_encode_symbol(eep_dp, 0, pCTXLastCG + offset);
                    } else {
                        biari_encode_symbol(eep_dp, 1, pCTXLastCG + offset);
                        count = 0;

                        offset = isChroma ? 4 : 10;
                        while (count < CGLastX) {
                            biari_encode_symbol(eep_dp, 0, pCTXLastCG + offset);
                            count++;
                        }
                        if (CGLastX < numCGminus1X) {
                            biari_encode_symbol(eep_dp, 1, pCTXLastCG + offset);
                        }

                        offset = isChroma ? 5 : 11;
                        if (CGLastX == 0) {
                            count = 0;
                            while (count < CGLastY - 1) {
                                biari_encode_symbol(eep_dp, 0, pCTXLastCG + offset);
                                count++;
                            }
                            if (CGLastY < numCGminus1Y) {
                                biari_encode_symbol(eep_dp, 1, pCTXLastCG + offset);
                            }
                        } else {
                            count = 0;
                            while (count < CGLastY) {
                                biari_encode_symbol(eep_dp, 0, pCTXLastCG + offset);
                                count++;
                            }
                            if (CGLastY < numCGminus1Y) {
                                biari_encode_symbol(eep_dp, 1, pCTXLastCG + offset);
                            }
                        }
                    }
                    if (se->context == LUMA_8x8 && ctxmode == INTRA_PRED_DC_DIAG) {
                        SWAP(CGLastX, CGLastY);
                    }
                }
                CGx = CGLastX;
                CGy = CGLastY;
            }

            //! Sig CG Flag
            if (rank > 0) {

                if (bitSize == 1) {
                    if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                            (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                          (currMB->cuType == PHOR_DOWN)))) {
                        CGx = iCG;
                        CGy = 0;
                    } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                               (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                       (currMB->cuType == PVER_RIGHT)))) {
                        CGx = 0;
                        CGy = iCG ;
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == InNxNMB)))) {
                        CGx = iCG;
                        CGy = 0;
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == INxnNMB)))) {
                        CGx = 0;
                        CGy = iCG ;
                    } else {
                        CGx = iCG & 1;
                        CGy = iCG / 2;
                    }
                } else if (bitSize == 2) {
                    if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && (IS_INTER(currMB) && !(isChroma) &&
                            (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                          (currMB->cuType == PHOR_DOWN)))) {
                        CGx = AVS_SCAN2x8[iCG][0];
                        CGy = AVS_SCAN2x8[iCG][1];
                    } else if (input->useNSQT && currMB->ui_MbBitSize > B8X8_IN_BIT  && IS_INTER(currMB) && !(isChroma) &&
                               (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                       (currMB->cuType == PVER_RIGHT))) {
                        CGx = AVS_SCAN8x2[iCG][0];
                        CGy = AVS_SCAN8x2[iCG][1];
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == InNxNMB)))) {
                        CGx = AVS_SCAN2x8[iCG][0];
                        CGy = AVS_SCAN2x8[iCG][1];
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == INxnNMB)))) {
                        CGx = AVS_SCAN8x2[iCG][0];
                        CGy = AVS_SCAN8x2[iCG][1];
                    } else {
                        CGx = AVS_SCAN4x4[iCG][0];
                        CGy = AVS_SCAN4x4[iCG][1];
                    }
                } else if (bitSize == 3) {
                    if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && IS_INTER(currMB) && !(isChroma) &&
                        (currMB->trans_size == 1) && ((currMB->cuType == P2NXN) || (currMB->cuType == PHOR_UP) ||
                                                      (currMB->cuType == PHOR_DOWN))) {
                        CGx = AVS_SCAN2x8[iCG][0];
                        CGy = AVS_SCAN2x8[iCG][1];
                    } else if (input->useNSQT  && currMB->ui_MbBitSize > B8X8_IN_BIT && IS_INTER(currMB) && !(isChroma) &&
                               (currMB->trans_size == 1) && ((currMB->cuType == PNX2N) || (currMB->cuType == PVER_LEFT) ||
                                       (currMB->cuType == PVER_RIGHT))) {
                        CGx = AVS_SCAN8x2[iCG][0];
                        CGy = AVS_SCAN8x2[iCG][1];
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == InNxNMB)))) {
                        CGx = AVS_SCAN2x8[iCG][0];
                        CGy = AVS_SCAN2x8[iCG][1];
                    } else if (input->useSDIP && (IS_INTRA(currMB) && !(isChroma) && (currMB->trans_size == 1) &&
                                                  ((currMB->cuType == INxnNMB)))) {
                        CGx = AVS_SCAN8x2[iCG][0];
                        CGy = AVS_SCAN8x2[iCG][1];
                    } else {
                        CGx = AVS_SCAN8x8[iCG][0];
                        CGy = AVS_SCAN8x8[iCG][1];
                    }
                }

                sigCGctx = isChroma ? 0 : ((iCG == 0) ?  0 : 1);

                {
                    biari_encode_symbol(eep_dp, DCT_CGFlag[ iCG ], pCTXSigCG + sigCGctx);
                }
            }

            //! (Run, Level)
            if (DCT_CGFlag[ iCG ]) {
                pos  = 0;
                pairsInCG = 0;
                for (i = DCT_PairsInCG[ iCG ]; i >= 0; i--, pairs--, pairsInCG++) {
                    if (i == 0) {
                        Level = 0;
                        Run = 0;
                    } else {
                        Level = DCT_Level[pairs - 1];
                        Run = DCT_Run[pairs - 1];
                    }

                    absLevel = abs(Level);
                    pCTX = Primary[rank];

                    //! LAST IN CG
                    if (i == DCT_PairsInCG[ iCG ]) {
                        {

                            xx = AVS_SCAN4x4[15 - DCT_CGLastRun[iCG]][0];
                            yy = AVS_SCAN4x4[15 - DCT_CGLastRun[iCG]][1];
                            if (rank != 0) {
                                //if(ctxmode!=1)
                                {
                                    xx = 3 - xx;
                                }
                                if (ctxmode != 0) {
                                    yy = 3 - yy;
                                }
                            }

                            if ((CGx == 0 && CGy > 0 && ctxmode == 2) /*|| (ctxmode == 1)*/) {
                                yy = yy ^ xx;
                                xx = yy ^ xx;
                                yy = xx ^ yy;
                            }

                            if (se->context == LUMA_8x8) {
                                offset = (bitSize == 0) ? (ctxmode / 2) * 4 : (((CGx > 0 &&
                                         CGy > 0) ? 0 : ((ctxmode / 2) * 4 + ((iCG == 0) ? 4 : 12))) + 8);
                            } else {
                                offset = (bitSize == 0) ? 0 : 4;
                            }

                            offset += (rank == 0 ? 0 : (isChroma ? NUM_LAST_POS_CTX_CHROMA / 2 : NUM_LAST_POS_CTX_LUMA / 2));

                            symbol = xx ;

                            ctx = 0;
                            while (symbol >= 1) {
                                symbol -= 1;
                                biari_encode_symbol(eep_dp, 0, pCTXLastPosInCG + offset + ctx);

                                ctx ++;
                                if (ctx >= 2) {
                                    ctx = 2;
                                }
                                if (ctx >= 1) {
                                    ctx = 1;
                                }
                            }
                            if (xx != 3) {
                                biari_encode_symbol(eep_dp, 1, pCTXLastPosInCG + offset + ctx);
                            }

                            symbol = yy ;

                            ctx = 0;
                            while (symbol >= 1) {
                                symbol -= 1;
                                biari_encode_symbol(eep_dp, 0, pCTXLastPosInCG + offset + ctx + 2);
                                ctx ++;
                                if (ctx >= 2) {
                                    ctx = 2;
                                }
                                if (ctx >= 1) {
                                    ctx = 1;
                                }
                            }
                            if (yy != 3) {
                                biari_encode_symbol(eep_dp, 1, pCTXLastPosInCG + offset + ctx + 2);
                            }
                            pos += DCT_CGLastRun[iCG];

                        }
                    }

                    if (pos == 16) {
                        break;
                    }

                    //! Level
                    symbol = absLevel - 1;

                    indiv = min(2, (pairsInCG + 1) / 2);
                    pCTX = Primary[ min(rank, indiv + 2) ];
                    offset = ((iCG == 0 && pos > 12) ? 0 : 3) + indiv + 8;
                    if (!isChroma) {
                        offset += 3;
                    }
                    if (symbol > 31) {
                        exp_golomb_order = 0;
                        biari_encode_symbol_final(eep_dp, 1);
                        symbol = symbol - 32;
                        while (1) {
                            if ((unsigned int)symbol >= (unsigned int)(1 << exp_golomb_order)) {
                                biari_encode_symbol_eq_prob(eep_dp, 0);
                                symbol = symbol - (1 << exp_golomb_order);
                                exp_golomb_order++;
                            } else {
                                biari_encode_symbol_eq_prob(eep_dp, 1);

                                while (exp_golomb_order--) {   //next binary part
                                    biari_encode_symbol_eq_prob(eep_dp, (unsigned char)((symbol >> exp_golomb_order) & 1));
                                }

                                break;
                            }
                        }
                    } else {
                        biari_encode_symbol_final(eep_dp, 0);
                        bins = 0;
                        while (symbol >= 1) {
                            symbol -= 1;
                            biari_encode_symbol(eep_dp, 0, pCTX + offset);
                            bins++;
                        }
                        if (bins < 31) {
                            biari_encode_symbol(eep_dp, 1, pCTX + offset);
                        }


                    }

                    //! Sign
                    if (Level > 0) {
                        Level_sign[i] = 0;
                    } else {
                        Level_sign[i] = 1;
                    }
                    //! Run
                    absSum5 = 0;
                    n = 0;
                    for (k = pairs + 1; k <= pairs + pairsInCG; k ++) {
                        n += DCT_Run[k - 1];
                        if (n >= 6) {
                            break;
                        }
                        absSum5 += abs(DCT_Level[k - 1]);
                        n ++;
                    }

                    pCTX = Primary[ min((absSum5 + absLevel) / 2, 2) ];

                    ctxpos = 0;
                    if (15 - pos > 0) {
                        px = AVS_SCAN4x4[15 - pos - 1 - ctxpos][0];
                        py = AVS_SCAN4x4[15 - pos - 1 - ctxpos][1];
                        //#if BBRY_CU8/////how to modify
                        moddiv = (ctxmode == INTRA_PRED_VER) ? (py >> 1) : (/*(ctxmode == INTRA_PRED_HOR)?(px >> 1):*/(pos + ctxpos <= 9));
                        offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv)) : (se->context == LUMA_8x8 ? 2 : 3 + moddiv)) +
                                 (bitSize == 0 ? 0 : 3);
                        if (se->context == LUMA_8x8) {
                            moddiv = (ctxmode == INTRA_PRED_VER) ? ((py + 1) / 2) : (/*(ctxmode == INTRA_PRED_HOR)?(((px+1)/2)+3):*/((
                                         pos + ctxpos) > 11 ? 6 : ((pos + ctxpos) > 4 ? 7 : 8)));
                            offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv % 3)) : (4 + moddiv % 3)) + (bitSize == 0 ? 0 : 4);
                        }
                    }

                    symbol = Run;
                    while (symbol >= 1) {
                        symbol -= 1;
                        biari_encode_symbol(eep_dp, 0, pCTX + offset);

                        ctxpos ++;
                        if ((15 - pos - 1 - ctxpos) >= 0) {
                            px = AVS_SCAN4x4[15 - pos - 1 - ctxpos][0];
                            py = AVS_SCAN4x4[15 - pos - 1 - ctxpos][1];
                            //#if BBRY_CU8/////how to modify
                            moddiv = (ctxmode == INTRA_PRED_VER) ? (py >> 1) : (/*(ctxmode == INTRA_PRED_HOR)?(px >> 1):*/(pos + ctxpos <= 9));
                            offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv)) : (se->context == LUMA_8x8 ? 2 : 3 + moddiv)) +
                                     (bitSize == 0 ? 0 : 3);
                            if (se->context == LUMA_8x8) {
                                moddiv = (ctxmode == INTRA_PRED_VER) ? ((py + 1) / 2) : (/*(ctxmode == INTRA_PRED_HOR)?(((px+1)/2)+3):*/((
                                             pos + ctxpos) > 11 ? 6 : ((pos + ctxpos) > 4 ? 7 : 8)));
                                offset = ((iCG == 0) ? (pos + ctxpos == 14 ? 0 : (1 + moddiv % 3)) : (4 + moddiv % 3)) + (bitSize == 0 ? 0 : 4);
                            }
                        }
                    }

                    if (Run < numOfCoeffInCG - 1 - pos) {
                        biari_encode_symbol(eep_dp, 1, pCTX + offset);
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

                    if (Run == numOfCoeffInCG - 1 - pos) {
                        pairs --;
                        pairsInCG ++;
                        break;
                    }

                    //! Update position
                    pos += (Run + 1);

                }
                // Sign of Level
                for (i = DCT_PairsInCG[ iCG ]; i > 0; i--) {
                    biari_encode_symbol_eq_prob(eep_dp, Level_sign[i]);
                }
            }
        }
    }

    //--- Reset Counters ---

    DCT_Pairs = 0;

}





/*!
************************************************************************
* \brief
*    Unary binarization and encoding of a symbol by using
*    one or two distinct models for the first two and all
*    remaining bins; no terminating "0" for max_symbol
*    (finite symbol alphabet)
************************************************************************
*/
void unary_bin_max_encode(EncodingEnvironmentPtr eep_dp,
                          unsigned int symbol,
                          BiContextTypePtr ctx,
                          int ctx_offset,
                          unsigned int max_symbol)
{
    unsigned int l;
    BiContextTypePtr ictx;
    if (symbol == 0) {
        biari_encode_symbol(eep_dp, 1, ctx);
        return;
    } else {
        biari_encode_symbol(eep_dp, 0, ctx);
        l = symbol;
        ictx = ctx + ctx_offset;

        while ((--l) > 0) {
            biari_encode_symbol(eep_dp, 0, ictx);
        }

        if (symbol < max_symbol) {
            biari_encode_symbol(eep_dp, 1, ictx);
        }
    }
    return;
}

int writeSplitFlag(int splitFlag, codingUnit *currMB, int uiBitSize)
{
    SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
    int            *bitCount  = currMB->bitcounter;
    Slice          *currSlice = img->currentSlice;
    DataPartition  *dataPart  ;
    int rate = 0;
    dataPart = & (currSlice->partArr[0]);

    currSE->value1 = splitFlag;
    currSE->value2 = uiBitSize;
    currSE->writing = writeSplitFlag_AEC;
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, 0);    //img->current_mb_nr); //check: img->current_mb_nr

#if TRACE

    if (he->AEC_writting) {
        fprintf(hc->p_trace, "splitflag = %3d\n", splitFlag);
    }

#endif
    rate = currSE->len;
    bitCount[BITS_INTER_MB] += currSE->len;
    return rate;
}

void writeSplitFlag_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPosition)
{
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    int   act_sym = se->value1;

    biari_encode_symbol(eep_dp, act_sym, ctx->split_contexts + se->value2);
}
int write_sao_mergeflag(int mergeleft_avail, int mergeup_avail, SAOBlkParam *saoBlkParam, int uiPositionInPic)
{
    int MergeLeft = 0;
    int MergeUp = 0;
    Slice          *currSlice;
    codingUnit *currMB ;
    SyntaxElement   *currSE ;
    DataPartition  *dataPart;
    int *bitCount;
    int rate;
    currSlice = img->currentSlice;
    currMB = &img->mb_data[uiPositionInPic];
    currSE  = &img->MB_SyntaxElements[currMB->currSEnr];
    bitCount  = currMB->bitcounter;
    dataPart = & (currSlice->partArr[0]);


    if (mergeleft_avail) {
        MergeLeft = ((saoBlkParam->modeIdc == SAO_MODE_MERGE) && (saoBlkParam->typeIdc == SAO_MERGE_LEFT));
        currSE->value1 = MergeLeft ? 1 : 0;

    }
    if (mergeup_avail && !MergeLeft) {
        MergeUp = ((saoBlkParam->modeIdc == SAO_MODE_MERGE) && (saoBlkParam->typeIdc == SAO_MERGE_ABOVE));
        currSE->value1 = MergeUp ? (1 + mergeleft_avail) : 0;
    }


    currSE->writing = write_sao_mergeflag_AEC;
    currSE->value2 = mergeleft_avail + mergeup_avail;
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);

    rate = currSE->len;
    bitCount[BITS_SAO_MB] += currSE->len;

    return rate;
}
void write_sao_mergeflag_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic)
{
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    int  act_sym = se->value1;
    int act_ctx = se->value2;
    if (act_ctx == 1) {
        assert(act_sym <= 1);
        biari_encode_symbol(eep_dp, act_sym, &ctx->saomergeflag_context[0]);
    } else if (act_ctx == 2) {
        assert(act_sym <= 2);
        biari_encode_symbol(eep_dp,  act_sym & 0x01, &ctx->saomergeflag_context[1]);
        if (act_sym != 1)
        { biari_encode_symbol(eep_dp, (act_sym >> 1) & 0x01, &ctx->saomergeflag_context[2]); }
    }
}
int write_sao_mode(SAOBlkParam *saoBlkParam, int uiPositionInPic)
{
    Slice      *currSlice;
    codingUnit *currMB ;
    SyntaxElement   *currSE ;
    DataPartition  *dataPart;
    int *bitCount;
    int rate;
    currSlice = img->currentSlice;
    currMB = &img->mb_data[uiPositionInPic];
    currSE  = &img->MB_SyntaxElements[currMB->currSEnr];
    bitCount  = currMB->bitcounter;
    dataPart = & (currSlice->partArr[0]);


    if (saoBlkParam->modeIdc == SAO_MODE_OFF) {
        currSE->value1 = 0;
    } else if (saoBlkParam->typeIdc == SAO_TYPE_BO) {
        currSE->value1 = 1;
    } else {
        currSE->value1 = 3;
    }

    currSE->writing = write_sao_mode_AEC;
    dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
    rate = currSE->len;
    bitCount[BITS_SAO_MB] += currSE->len;
    return rate;

}
void write_sao_mode_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic)
{
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    int  act_sym = se->value1;
    if (act_sym == 0) {
        biari_encode_symbol(eep_dp,  1, ctx->saomode_context);
    } else {
        biari_encode_symbol(eep_dp,  0, ctx->saomode_context);
        biari_encode_symbol_eq_prob(eep_dp, !((act_sym >> 1) & 0x01));
    }
}
int write_sao_offset(SAOBlkParam *saoBlkParam, int uiPositionInPic, int offsetTh)
{
    Slice      *currSlice;
    codingUnit *currMB ;
    SyntaxElement   *currSE ;
    DataPartition  *dataPart;
    int *bitCount;
    int rate = 0;
    int i;
    int bandIdxBO[4];
    currSlice = img->currentSlice;
    currMB = &img->mb_data[uiPositionInPic];
    currSE  = &img->MB_SyntaxElements[currMB->currSEnr];
    bitCount  = currMB->bitcounter;
    dataPart = & (currSlice->partArr[0]);

    assert(saoBlkParam->modeIdc == SAO_MODE_NEW);
    if (saoBlkParam->typeIdc == SAO_TYPE_BO) {
        bandIdxBO[0] = saoBlkParam->startBand;
        bandIdxBO[1] = bandIdxBO[0] + 1;
        bandIdxBO[2] = saoBlkParam->startBand2;
        bandIdxBO[3] = bandIdxBO[2] + 1;
        for (i = 0; i < 4; i++) {
            currSE->value1 = saoBlkParam->offset[ bandIdxBO[i] ];
            currSE->value2  = SAO_CLASS_BO;

            currSE->writing = write_sao_offset_AEC;
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
            rate += currSE->len;
            bitCount[BITS_SAO_MB] += currSE->len;
        }
    } else {
        assert(saoBlkParam->typeIdc >= SAO_TYPE_EO_0 && saoBlkParam->typeIdc <= SAO_TYPE_EO_45);
        for (i = SAO_CLASS_EO_FULL_VALLEY; i < NUM_SAO_EO_CLASSES; i++) {
            if (i == SAO_CLASS_EO_PLAIN) {
                continue;
            }
            currSE->value1 = saoBlkParam->offset[i];
            currSE->value2  = i;

            currSE->writing = write_sao_offset_AEC;
            dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
            rate += currSE->len;
            bitCount[BITS_SAO_MB] += currSE->len;
        }
    }

    return rate;
}
void write_sao_offset_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic)
{

    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    int  act_sym;
    int signsymbol = se->value1 >= 0 ? 0 : 1;
    int temp, maxvalue;
    int offset_type = se->value2;

    assert(offset_type != SAO_CLASS_EO_PLAIN);
    if (offset_type == SAO_CLASS_EO_FULL_VALLEY) {
        act_sym = EO_OFFSET_MAP[se->value1 + 1];
    } else if (offset_type == SAO_CLASS_EO_FULL_PEAK) {
        act_sym = EO_OFFSET_MAP[-se->value1 + 1];
    } else {
        act_sym = abs(se->value1);
    }
    maxvalue = saoclip[offset_type][2];
    temp = act_sym;
    if (temp == 0) {
        if (offset_type == SAO_CLASS_BO) {
            biari_encode_symbol(eep_dp, 1, &ctx->saooffset_context[0]);
        } else {
            biari_encode_symbol_eq_prob(eep_dp,  1);
        }
    } else {
        while (temp != 0) {
            if (offset_type == SAO_CLASS_BO && temp == act_sym) {
                biari_encode_symbol(eep_dp, 0, &ctx->saooffset_context[0]);
            } else {
                biari_encode_symbol_eq_prob(eep_dp, 0);
            }

            temp--;
        }
        if (act_sym < maxvalue) {
            biari_encode_symbol_eq_prob(eep_dp, 1);
        }
    }
    if (offset_type == SAO_CLASS_BO && act_sym) {
        biari_encode_symbol_eq_prob(eep_dp, signsymbol);
    }

}
int write_sao_type(SAOBlkParam *saoBlkParam, int uiPositionInPic)
{
    Slice      *currSlice;
    codingUnit *currMB ;
    SyntaxElement   *currSE ;
    DataPartition  *dataPart;
    int *bitCount;
    int rate = 0;
    currSlice = img->currentSlice;
    currMB = &img->mb_data[uiPositionInPic];
    currSE  = &img->MB_SyntaxElements[currMB->currSEnr];
    bitCount  = currMB->bitcounter;
    dataPart = & (currSlice->partArr[0]);


    assert(saoBlkParam->modeIdc == SAO_MODE_NEW);
    if (saoBlkParam->typeIdc == SAO_TYPE_BO) {
        currSE->value1 = saoBlkParam->startBand;
        currSE->value2 = 1;//write start band for BO
        currSE->writing = write_sao_type_AEC;
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        rate += currSE->len;
        bitCount[BITS_SAO_MB] += currSE->len;

        assert(saoBlkParam->deltaband >= 2);
        currSE->value1 = saoBlkParam->deltaband - 2;
        currSE->value2 = 2;//write delta start band for BO
        currSE->writing = write_sao_type_AEC;
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        rate += currSE->len;
        bitCount[BITS_SAO_MB] += currSE->len;
#if TRACE
        if (he->AEC_writting) {
            fprintf(hc->p_trace, "coded band = %d, second band = %d, delta band = %d\n", saoBlkParam->startBand,
                    saoBlkParam->startBand2,  saoBlkParam->deltaband);
        }
#endif
    } else {
        assert(saoBlkParam->typeIdc >= SAO_TYPE_EO_0 && saoBlkParam->typeIdc <= SAO_TYPE_EO_45);
        currSE->value1 = saoBlkParam->typeIdc;
        currSE->value2 = 0;
        currSE->writing = write_sao_type_AEC;
        dataPart->writeSyntaxElement(currMB, currSE, dataPart, uiPositionInPic);
        rate += currSE->len;
        bitCount[BITS_SAO_MB] += currSE->len;

    }

    return rate;

}
void write_sao_type_AEC(codingUnit *currMB, SyntaxElement *se, EncodingEnvironmentPtr eep_dp, int uiPositionInPic)
{
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    int  act_sym = se->value1;
    int temp;
    int i, length;
    int exp_golomb_order;
    temp = act_sym;
    exp_golomb_order = 1;

    switch (se->value2) {
    case 0:
        length = NUM_SAO_EO_TYPES_LOG2;
        break;
    case 1:
        length = NUM_SAO_BO_CLASSES_LOG2;
        break;
    case 2:
        length = NUM_SAO_BO_CLASSES_LOG2 - 1;
        break;
    default:
        length = 0;
        break;
    }
    if (se->value2 == 2) {

        while (1) {
            if ((unsigned int)temp >= (unsigned int)(1 << exp_golomb_order)) {
                biari_encode_symbol_eq_prob(eep_dp, 0);
                temp = temp - (1 << exp_golomb_order);
                exp_golomb_order++;
            } else {

                if (exp_golomb_order == 4) {
                    exp_golomb_order = 0;
                } else {
                    biari_encode_symbol_eq_prob(eep_dp, 1);
                }

                while (exp_golomb_order--) {   //next binary part
                    biari_encode_symbol_eq_prob(eep_dp, (unsigned char)((temp >> exp_golomb_order) & 1));
                }

                break;
            }
        }
    } else {
        for (i = 0; i < length; i++) {
            biari_encode_symbol_eq_prob(eep_dp, temp & 0x0001);
            temp = temp >> 1;
        }
    }
}

int writeAlfLCUCtrl(int iflag, DataPartition *this_dataPart, int compIdx, int ctx_idx)
{
    EncodingEnvironmentPtr eep_dp = &(this_dataPart->ee_AEC);
    SyntaxInfoContexts  *ctx    = img->currentSlice->syn_ctx;
    int curr_len, post_len;
    unsigned char symbol;

    curr_len = arienco_bits_written(eep_dp);
    symbol  = (unsigned char)iflag ;
    biari_encode_symbol(eep_dp, symbol , &(ctx->m_cALFLCU_Enable_SCModel[0][0]));
    post_len = (arienco_bits_written(eep_dp) - curr_len);
    return (post_len);
}
void writeAlfCoeff(ALFParam *Alfp)
{
    int pos, i;
    int groupIdx[NO_VAR_BINS];

    int f = 0;
    const int numCoeff = (int)ALF_MAX_NUM_COEF;
    unsigned int noFilters;

    Bitstream *bitstream = currBitStream;

    switch (Alfp->componentID) {
    case ALF_Cb:
    case ALF_Cr: {
        for (pos = 0; pos < numCoeff; pos++) {
            se_v("Chroma ALF coefficients", Alfp->coeffmulti[0][pos], bitstream);
        }
    }
    break;
    case ALF_Y: {
        noFilters = Alfp->filters_per_group - 1;
        ue_v("ALF filter number", noFilters, bitstream);
        groupIdx[0] = 0;
        f++;
        if (Alfp->filters_per_group > 1) {
            for (i = 1; i < NO_VAR_BINS; i++) {
                if (Alfp->filterPattern[i] == 1) {
                    groupIdx[f] = i;
                    f++;
                }
            }
        }

        for (f = 0; f < Alfp->filters_per_group; f++) {
            if (f > 0 && Alfp->filters_per_group != 16) {
                ue_v("Region distance", (unsigned int)(groupIdx[f] - groupIdx[f - 1]), bitstream);
            }

            for (pos = 0; pos < numCoeff; pos++) {
                se_v("Luma ALF coefficients", Alfp->coeffmulti[f][pos], bitstream);
            }
        }
    }
    break;
    default: {
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
    }
}

