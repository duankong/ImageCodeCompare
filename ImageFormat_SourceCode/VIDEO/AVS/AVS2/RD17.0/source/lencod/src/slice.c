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

#include "../../lcommon/inc/contributors.h"

#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "header.h"
#include "bitstream.h"
#include "vlc.h"
#include "image.h"
#include "rdopt_coding_state.h"
#include "AEC.h"
#include "../../lcommon/inc/loop-filter.h"
#include "tdrdo.h"
#include "pos_info.h"

#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif


#include "../../lcommon/inc/commonStructures.h"
#include "../../lcommon/inc/ComAdaptiveLoopFilter.h"
#include "../../lencod/inc/EncAdaptiveLoopFilter.h"
#include "../../lencod/inc/image.h"
void init_slice(int start_mb_addr, Picture *currPic);
void init_slice_Bitstream(int start_mb_addr, Picture *currPic);
static Slice *malloc_slice();
extern void slice_fastskipSAO(int *slice_sao_on, int *num_off_sao);

#if TDRDO
#include"tdrdo.h"
#endif

#if RATECONTROL
#include "ratecontrol.h"
#endif


/*
*************************************************************************
* Function:This function terminates a slice
* Input:
* Output:
* Return: 0 if OK,                                                         \n
1 in case of error
* Attention:                                                  by jlzheng
*************************************************************************
*/

int start_slice()
{
    Bitstream *currStream;

    currStream = currBitStream;

    if (img->current_mb_nr != 0) {
        Demulate(currStream, he->current_slice_bytepos);     //qhg 20060327  for de-emulation
    } else {
        he->current_slice_bytepos = currStream->byte_pos;
        return 0;
    }

    if (currStream->bits_to_go != 8) {   //   hzjia 2004-08-20

        currStream->byte_buf <<= currStream->bits_to_go;
        currStream->byte_buf |= (1 << (currStream->bits_to_go - 1));

        currStream->streamBuffer[currStream->byte_pos++] = currStream->byte_buf;
        currStream->bits_to_go = 8;
        currStream->byte_buf = 0;
    } else { // cjw 20060321
        currStream->streamBuffer[currStream->byte_pos++] = 0x80;
        currStream->bits_to_go = 8;
        currStream->byte_buf = 0;
    }

    //qhg 20060327 for de-emulation
    he->current_slice_bytepos = currStream->byte_pos;

    return 0;
}

void demulateFunction()
{
    Bitstream *currStream = currBitStream;

    Demulate(currStream, he->current_slice_bytepos);                   //qhg 20060327

    if (currStream->bits_to_go != 8) {   //   hzjia 2004-08-20

        currStream->byte_buf <<= currStream->bits_to_go;
        currStream->byte_buf |= (1 << (currStream->bits_to_go - 1));

        currStream->streamBuffer[currStream->byte_pos++] = currStream->byte_buf;
        currStream->bits_to_go = 8;
        currStream->byte_buf = 0;
    } else { // cjw 20060321
        currStream->streamBuffer[currStream->byte_pos++] = 0x80;
        currStream->bits_to_go = 8;
        currStream->byte_buf = 0;
    }

    he->current_slice_bytepos = currStream->byte_pos;
}

/*
*************************************************************************
* Function:This function terminates a picture
* Input:
* Output:
* Return: 0 if OK,                                                         \n
1 in case of error
* Attention:
*************************************************************************
*/

int terminate_picture()
{
    Bitstream *currStream = currBitStream;

    Demulate(currStream, he->current_slice_bytepos);                   //qhg 20060327

    currStream->byte_buf <<= currStream->bits_to_go;
    currStream->byte_buf |= (1 << (currStream->bits_to_go - 1));     // Yulj 2004.07.16

    currStream->streamBuffer[currStream->byte_pos++] = currStream->byte_buf;
    currStream->bits_to_go = 8;
    currStream->byte_buf = 0;

    return 0;
}

/*
*************************************************************************
* Function:Encodes one slice
* Input:
* Output:
* Return: the number of coded MBs in the SLice
* Attention:
*************************************************************************
*/
void picture_data(Picture *pic)
{
    Boolean end_of_picture = FALSE;
    int CurrentMbNumber = 0;
    int MBRowSize = img->width / MIN_CU_SIZE;         // jlzheng 7.1
    int slice_nr = 0;
    int slice_qp = img->qp;
    int len;
    int old_bit;
    double qp;
    ALFParam *alfPictureParam[NUM_ALF_COMPONENT];
    Boolean currAlfEnable;
    DataPartition  *dataPart;
    int compIdx, ctx_idx;
    int numLCUInPicWidth, numLCUInPicHeight, NumCUInFrame, lcuHeight, lcuWidth;

    //!EDIT START <added by lzhang AEC
// int Bytes_After_Header;                 //added by lzhang
    EncodingEnvironmentPtr eep;
    Slice *currSlice ;
    Bitstream *currStream;
    int   i;
    //!EDIT end <added by lzhang AEC

    int num_of_orgMB_in_col ;
    int num_of_orgMB_in_row ;
    int size = 1 << input->g_uiMaxSizeInBit;
    int pix_x;
    int pix_y;
    int num_of_orgMB;
#if MB_DQP
#endif

    int smb_index = 0;
    int num_off_sao[NUM_SAO_COMPONENTS];
    int preSlicePos = 0;
    int lcuIndex = 0;

    CreateEdgeFilter();

    lcuHeight = 1 << input->g_uiMaxSizeInBit;
    lcuWidth  = lcuHeight;
    numLCUInPicWidth  = img->width / lcuWidth ;
    numLCUInPicHeight = img->height / lcuHeight ;
    numLCUInPicWidth  += (img->width % lcuWidth) ? 1 : 0;
    numLCUInPicHeight += (img->height % lcuHeight) ? 1 : 0;
    NumCUInFrame = numLCUInPicHeight * numLCUInPicWidth;

#if IMGY_ALF_REC // test
    if (hc->imgY_alf_Rec == NULL) {
        printf("\nimgY_alf_Rec is NULL: after BitStreamCopy\n");
    }
#endif

    while (end_of_picture == FALSE) {   // loop over codingUnits
        lcuIndex++;
        set_MB_parameters(CurrentMbNumber);

        pix_x = (img->current_mb_nr % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
        pix_y = (img->current_mb_nr / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

        if (pix_x + size >= img->width) {
            num_of_orgMB_in_row = (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT;
        } else {
            num_of_orgMB_in_row = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);
        }

        if (pix_y + size >= img->height) {
            num_of_orgMB_in_col = (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT;
        } else {
            num_of_orgMB_in_col = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);
        }

        num_of_orgMB = num_of_orgMB_in_col * num_of_orgMB_in_row;

#if RATECONTROL
        if (input->EncControl && input->useDQP && img->current_mb_nr == 0) {
            Init_LCURateControl(pRC, num_of_orgMB);
        }
#endif

        if (img->current_mb_nr == 0 || (img->current_mb_nr > 0 &&
                                        img->mb_data[img->current_mb_nr].slice_nr != img->mb_data[img->current_mb_nr - 1].slice_nr)) {

            old_bit = currBitStream->byte_pos * 8 + 8 - currBitStream->bits_to_go ;
            start_slice();

#if MB_DQP
            AEC_new_slice();
#endif

            slice_fastskipSAO(img->slice_sao_on, num_off_sao);
            SliceHeader(slice_nr, slice_qp);
            {
                int mb_width = img->width / MIN_BLOCK_SIZE;    //4
                int mb_height = img->height / MIN_BLOCK_SIZE;  //4
                int cuInPicWidth = img->width / MIN_CU_SIZE;   //8
                int cuInPicHeight = img->height / MIN_CU_SIZE; //8
                int mb_x = (img->current_mb_nr % cuInPicWidth) << 1;
                int mb_y = (img->current_mb_nr / cuInPicWidth) << 1;
                int curr_minBlock_nr = mb_y * mb_width + mb_x; //min predition block order in a picture
                int block_i, block_j;

                for (block_j = 0; block_j < (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT));
                     block_j++) { //from previous slice pos to the end of the line for preSlicePos or the pos of current LCU
                    int currPos = Clip3(0, mb_height, ((preSlicePos / cuInPicWidth) << 1) + block_j) * mb_width + ((
                                      preSlicePos % cuInPicWidth) << 1);
                    int endPos = min((currPos / mb_width + 1) * mb_width,
                                     curr_minBlock_nr + block_j * mb_width); // the end of the line for preSlicePos or the pos of current LCU
                    for (block_i = currPos; block_i < endPos; block_i++) {
                        img->rec_ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = img->ipredmode[1 + block_i / mb_width][1 +
                                (block_i % mb_width)];
                        img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
                    }
                }
                for (; block_i < curr_minBlock_nr - mb_x;
                     block_i++) {  //from the next line of preSlicePos to the start of the line of current LCU
                    img->rec_ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = img->ipredmode[1 + block_i / mb_width][1 +
                            (block_i % mb_width)];
                    img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
                }
                for (block_j = 0; block_j < (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)); block_j++) {  //the left
                    //int currPos = max(Clip3(0, mb_height, ((preSlicePos/cuInPicWidth)<<1)+block_j)*mb_width+((preSlicePos%cuInPicWidth)<<1), Clip3(0, mb_height, mb_y + block_j)*mb_width);
                    int currPos = max(block_i, Clip3(0, mb_height, mb_y + block_j) * mb_width);
                    int endPos = Clip3(0, mb_height, (mb_y + block_j)) * mb_width + mb_x;
                    for (block_i = currPos; block_i < endPos; block_i++) {
                        img->rec_ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = img->ipredmode[1 + block_i / mb_width][1 +
                                (block_i % mb_width)];
                        img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
                    }
                }
                preSlicePos = img->current_mb_nr;
            }

            init_slice(img->current_mb_nr , pic);
            // Bytes_After_Header = img->currentSlice->partArr[0].bitstream->byte_pos;
            currSlice = img->currentSlice;

            for (i = 0; i < 1; i++) {
                (currSlice->partArr[i]).bitstream = currBitStream;
                currStream = (currSlice->partArr[i]).bitstream;
                currStream->write_flag = 0;
                eep = & ((currSlice->partArr[i]).ee_AEC);

                if (currBitStream->bits_to_go < 8) {
                    // trailing bits to process
                    currBitStream->byte_buf = (currBitStream->byte_buf << currBitStream->bits_to_go) | (0xff >>
                                              (8 - currBitStream->bits_to_go));
                    stat->bit_use_stuffingBits[img->type] += currBitStream->bits_to_go;
                    currBitStream->streamBuffer[currStream->byte_pos++] = currBitStream->byte_buf;
                    currBitStream->bits_to_go = 8;
                }

                arienco_start_encoding(eep, currStream->streamBuffer, & (currStream->byte_pos), img->type);
            }

            init_contexts();


            slice_nr++;

        }

        he->AEC_writting = 0;
        start_codingUnit();

#if REFINED_QP
        if (!input->use_refineQP)
#endif
        {
            int prevP_no, RPS_idx, p_interval, no_IPframes;
            //double qp     = ( double ) img->qp - SHIFT_QP;

#if FREQUENCY_WEIGHTING_QUANTIZATION
            int     lambdaQP;     //M2331 2008-04
            double  lambdaF;      //M2331 2008-04

            if (WeightQuantEnable) {
                he->mb_wq_mode = WQ_MODE_D;
                if (img->type == F_IMG) {
                    lambdaQP = LambdaQPTab[P_IMG][he->mb_wq_mode];
                    lambdaF = LambdaFTab[P_IMG][he->mb_wq_mode];
                } else {
                    lambdaQP = LambdaQPTab[img->type][he->mb_wq_mode];
                    lambdaF = LambdaFTab[img->type][he->mb_wq_mode];
                }
            }
#endif


#if (FREQUENCY_WEIGHTING_QUANTIZATION && AWQ_WEIGHTING)
            if (WeightQuantEnable) {
                qp     = (double) img->qp - SHIFT_QP + lambdaQP;
            } else {
                qp     = (double) img->qp - SHIFT_QP;
            }
#else
            qp     = (double) img->qp - SHIFT_QP;
#endif

            p_interval = 0;
            for (i = 0; i <= he->subGopID; i++) {
                p_interval += (input->jumpd_sub[i] + 1);
            }
            prevP_no      = (img->number - 1) * input->jumpd_all + (p_interval - input->jumpd - 1);
            RPS_idx       = he->subGopNum == 1 ? ((hc->coding_order - 1) % he->gop_size) : (hc->coding_order - 1 - prevP_no);
            RPS_idx       = he->subGopNum == 1 ? ((hc->coding_order - 1) % he->gop_size) : (hc->coding_order - 1 - prevP_no);
            no_IPframes = (input->no_frames + (input->successive_Bframe_all + he->subGopNum - 1) - 1) /
                          (input->successive_Bframe_all + he->subGopNum) + 1;
            if (input->intra_period == 1) {
                he->lambda_mode = 0.85 * pow(2, qp / 4.0) *  LAM_2Level_TU;
            } else {
                //lambda_mode   = 0.68 * pow ( 2, qp / 4.0 ) * ( img->type == B_IMG ? 1.2 : 0.8 );
#if (FREQUENCY_WEIGHTING_QUANTIZATION && AWQ_WEIGHTING)
                if (WeightQuantEnable) {
                    he->lambda_mode = lambdaF * pow(2, qp / 4.0) * (img->type == B_IMG ? 1.2 : 0.8);
                } else {
                    he->lambda_mode = 0.68 * pow(2, qp / 4.0) * (img->type == B_IMG ? 1.2 : 0.8);
                }
#else
                he->lambda_mode   = 0.68 * pow(2, qp / 4.0) * (img->type == B_IMG ? 1.2 : 0.8);
#endif

                if (input->successive_Bframe > 0) {
                    if ((img->type != I_IMG && RPS_idx != 0) || (img->number - he->background_number == no_IPframes - 1)) {
                        he->lambda_mode *= max(2.00, min(4.00, qp / 8.0));
                    } else if (img->type == INTER_IMG || img->type == F_IMG) {
                        he->lambda_mode *= 1.25;
                    }
                } else {
                    if ((RPS_idx + 1) % he->gop_size != 0
                        && img->typeb != BACKGROUND_IMG
                       ) {
                        he->lambda_mode *= max(2.00, min(4.00, qp / 8.0)) * 0.8;
                    }
                }
            }
        }

#if TDRDO
        CurMBQP = img->qp;
        if (input->TDEnable != 0) {
            if (GlobeFrameNumber < input->no_frames && img->type != 0) {
                if (input->successive_Bframe_all && input->no_frames > 1 &&
                    GlobeFrameNumber <= ((int)((input->no_frames - 1) / StepLength))*StepLength) {
                    pOMCPFD = SearchFrameDistortionArray(OMCPDList, GlobeFrameNumber, StepLength, img->type);
                } else if (!input->successive_Bframe_all && input->no_frames > StepLength && GlobeFrameNumber % StepLength == 0) {
                    pOMCPFD = &OMCPDList->FrameDistortionArray[(GlobeFrameNumber - 1) / StepLength];
                } else {
                    pOMCPFD = NULL;
                }
            }
            // Only for LD. does not support RA or AI.
            if (img->type != I_IMG && input->intra_period == 0 && input->successive_Bframe_all == 0) {
                AdjustLcuQPLambdaLDP(pOMCPFD, img->current_mb_nr, input->img_width / MIN_CU_SIZE, &CurMBQP, &he->lambda_mode);
                CurMBQP = max(MIN_QP, min(CurMBQP, MAX_QP + (input->sample_bit_depth - 8) * 8));
            }
        }
#endif

#if RATECONTROL
        if (input->EncControl != 0) {
            if (img->current_mb_nr == 0) {
                pRC->SumMBQP = pRC->NumMB = pRC->LCUbaseQP = 0;
            }
            pRC->RcMBQP = pRC->LCUbaseQP = img->qp;
            if (input->EncControl && input->useDQP) {
                if (input->intra_period == 1) {
                    if (img->current_mb_nr == 0) {
                        pRC->RcMBQP = img->qp;
                    } else {
                        pRC->prclcu->LCUdeltaQP = CalculateLCUDeltaQP(pRC);
                        pRC->RcMBQP = img->qp + pRC->prclcu->LCUdeltaQP;
                        pRC->RcMBQP = max(img->qp - 3, min(pRC->RcMBQP, img->qp + 3));
                    }
                }
                if (input->intra_period == 0) {
                    if (img->number == 0) {
                        pRC->RcMBQP = img->qp;
                    } else {
                        pRC->prclcu->LCUdeltaQP = CalculateLCUDeltaQP(pRC);
                        pRC->RcMBQP = img->qp + pRC->prclcu->LCUdeltaQP;
                        pRC->RcMBQP = max(img->qp - 5, min(pRC->RcMBQP, img->qp + 5));
                    }
                }
                if (input->intra_period > 1) {
                    if (img->number == 0) {
                        pRC->RcMBQP = img->qp;
                    } else {
                        pRC->prclcu->LCUdeltaQP = CalculateLCUDeltaQP(pRC);
                        pRC->RcMBQP = img->qp + pRC->prclcu->LCUdeltaQP;
                        pRC->RcMBQP = max(img->qp - 5, min(pRC->RcMBQP, img->qp + 5));
                    }
                }
                he->lambda_mode *= pow(2, (pRC->RcMBQP - img->qp) / 4.0);
                pRC->RcMBQP = max(MIN_QP, min(pRC->RcMBQP, MAX_QP + (input->sample_bit_depth - 8) * 8));
            }
            pRC->SumMBQP += pRC->RcMBQP;
            pRC->NumMB++;
        }
#endif

#if MB_DQP
        lastdqp2tempdqp();
        he->cof_flag = 0;
#endif

        encode_one_SMB(input->g_uiMaxSizeInBit, img->current_mb_nr, 0);

#if TDRDO
        if (input->TDEnable != 0)
            if ((GlobeFrameNumber % StepLength == 0 && !input->successive_Bframe_all) || input->successive_Bframe_all) {
                StoreLCUInf(pRealFD, img->current_mb_nr, input->img_width / MIN_CU_SIZE, CurMBQP, he->lambda_mode,
                            img->type);    // stores for key frame
            }
#endif

        if (input->sao_enable) {
            xSetEdgeFilter_One_SMB(input->g_uiMaxSizeInBit, img->current_mb_nr);
            sao_cross_slice = input->crossSliceLoopFilter;
            Copy_SMB_for_SAO(smb_index, pix_y >> MIN_CU_SIZE_IN_BIT, pix_x >> MIN_CU_SIZE_IN_BIT,  num_of_orgMB_in_col,
                             num_of_orgMB_in_row);

            Deblock_One_SMB(smb_index, pix_y >> MIN_CU_SIZE_IN_BIT, pix_x >> MIN_CU_SIZE_IN_BIT,  num_of_orgMB_in_col,
                            num_of_orgMB_in_row);
            getStatisticsSAO_one_SMB(smb_index, pix_y >> MIN_CU_SIZE_IN_BIT, pix_x >> MIN_CU_SIZE_IN_BIT,  num_of_orgMB_in_col,
                                     num_of_orgMB_in_row, img->saostatData[smb_index]);

            getParaSAO_one_SMB(smb_index, pix_y >> MIN_CU_SIZE_IN_BIT, pix_x >> MIN_CU_SIZE_IN_BIT, num_of_orgMB_in_col,
                               num_of_orgMB_in_row, img->slice_sao_on, img->saostatData[smb_index], img->saoBlkParams[smb_index],
                               img->rec_saoBlkParams, num_off_sao, he->lambda_mode);

        }
        he->AEC_writting = 1;

#if MB_DQP
        he->cof_flag = 1;
        tempdqp2lastdqp();
#endif

        if (img->current_mb_nr != img->currentSlice->start_mb_nr) {    //qyu 0906
            write_terminating_bit(0);
        }

        if (input->sao_enable) {
            writeParaSAO_one_SMB(smb_index, pix_y >> MIN_CU_SIZE_IN_BIT, pix_x >> MIN_CU_SIZE_IN_BIT,  num_of_orgMB_in_col,
                                 num_of_orgMB_in_row, img->slice_sao_on, img->saoBlkParams[smb_index]);
            smb_index++;
        }
        write_one_SMB(input->g_uiMaxSizeInBit, img->current_mb_nr, 1);
        //multiple slice
        //if ( input->slice_row_nr > 0 && (( ( img->coded_mb_nr + num_of_orgMB - img->slice_offset ) % ( input->slice_row_nr * ( 1 << ( input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT ) ) ) == 0 ) || ( img->coded_mb_nr + num_of_orgMB == img->PicSizeInMbs ) ) )
        if ((lcuIndex % input->slice_row_nr == 0) || (img->coded_mb_nr + num_of_orgMB == img->PicSizeInMbs)) {
            terminate_slice();
            for (i = 0; i < NUM_SAO_COMPONENTS; i++) {
                img->cur_saorate[i] = (double)num_off_sao[i] / (double)max(smb_index, 1);
            }
        }

        //multiple slice
        terminate_codingUnit(&end_of_picture, lcuIndex);
        proceed2nextcodingUnit();

        if (end_of_picture) {
            smb_index = 0;
        }

        if ((CurrentMbNumber + num_of_orgMB_in_row) % MBRowSize == 0) {     //end of the row
            CurrentMbNumber += (num_of_orgMB_in_row + MBRowSize * ((1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT)) -
                                1));   //eg. 64 +: (4-1)*num_mb_inWidth+4
        } else {
            CurrentMbNumber += num_of_orgMB_in_row;
        }
#if RATECONTROL
        if (input->EncControl != 0) {
            int LCUbits;
            if (img->current_mb_nr == 0) {
                pRC->prclcu->PreBits = 0;
            }
            pRC->prclcu->CurBits = currBitStream->byte_pos * 8;
            LCUbits = pRC->prclcu->CurBits - pRC->prclcu->PreBits;
            pRC->prclcu->PreBits = pRC->prclcu->CurBits;
            UpdataLCURateControl(pRC, pRC->RcMBQP, he->lambda_mode, LCUbits, numLCUInPicWidth * numLCUInPicHeight);
        }
#endif
    }

    {
        int mb_width = img->width / MIN_BLOCK_SIZE;
        int mb_height = img->height / MIN_BLOCK_SIZE;
        int cuInPicWidth = img->width / MIN_CU_SIZE;   //8
        int cuInPicHeight = img->height / MIN_CU_SIZE; //8
        int block_i, block_j;

        for (block_j = 0; block_j < (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)); block_j++) {
            int currPos = Clip3(0, mb_height, ((preSlicePos / cuInPicWidth) << 1) + block_j) * mb_width + ((
                              preSlicePos % cuInPicWidth) << 1);
            int endPos = (currPos / mb_width + 1) * mb_width;
            for (block_i = currPos; block_i < endPos; block_i++) {
                img->rec_ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = img->ipredmode[1 + block_i / mb_width][1 +
                        (block_i % mb_width)];
                img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
            }
        }
        for (; block_i < mb_width * mb_height; block_i++) {
            img->rec_ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = img->ipredmode[1 + block_i / mb_width][1 +
                    (block_i % mb_width)];
            img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
        }

    }

    AEC_new_slice();

    if (!input->loop_filter_disable) {
        DeblockFrame(hc->imgY, hc->imgUV);
    }
    if (input->sao_enable) {
        Copy_frame_for_SAO();
        SAOFrame(input->g_uiMaxSizeInBit, img->rec_saoBlkParams, img->slice_sao_on,
                 input->sample_bit_depth);
    }

    if (input->alf_enable) {
        {
            int mb_width = img->width / MIN_BLOCK_SIZE;
            int mb_height = img->height / MIN_BLOCK_SIZE;
            int block_i;
            for (block_i = 0; block_i < mb_width * mb_height; block_i++) {
                img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = img->rec_ipredmode[1 + block_i / mb_width][1 +
                        (block_i % mb_width)];
            }

            preSlicePos = 0;
            lcuIndex = 0;
        }
        Copy_frame_for_ALF();
        for (i = 0; i < NUM_ALF_COMPONENT; i++) {
            alfPictureParam[i] = NULL;
            AllocateAlfPar(&(alfPictureParam[i]), i);
        }
        arienco_start_encoding(&(img->dp_ALF->ee_AEC), currBitStream->streamBuffer, &(currBitStream->byte_pos),
                               img->type);
        ALFProcess(alfPictureParam, img, hc->imgY_alf_Org, hc->imgUV_alf_Org, hc->imgY_alf_Rec, hc->imgUV_alf_Rec,
                   he->lambda_mode * LAMBDA_SCALE_LUMA);

        currAlfEnable = !(alfPictureParam[ALF_Y]->alf_flag == 0 && alfPictureParam[ALF_Cb]->alf_flag == 0 &&
                          alfPictureParam[ALF_Cr]->alf_flag == 0);
        for (i = 0; i < NUM_ALF_COMPONENT; i++) {
            img->pic_alf_on[i] = alfPictureParam[i]->alf_flag;
        }

        // write syntax element
        smb_index = 0;
        slice_nr = 0;
        if (input->alf_enable) {
            BitStreamCopy(currBitStream, currBitStream_ALF);
        }
        he->current_slice_bytepos = he->current_slice_bytepos_alf;  //re-store the start pos for de-emulation when alf is on
        picture_header_ALF(alfPictureParam);
        demulateFunction();
#if PicExtensionData
        if (input->PicExtentData && input->alf_enable) {
            picture_copyright_extension(currBitStream);
            picture_cameraparameters_extension(currBitStream);
            picture_display_extension(currBitStream);
        }
#endif
#if ROI_M3264
        if (input->ROI_Coding && input->alf_enable) {
            roi_parameters_extension(img->tr, currBitStream);
        }
#endif
        img->current_mb_nr  = 0;
        pic->no_slices = 0;
        end_of_picture = FALSE;
        CurrentMbNumber = 0;

        while (end_of_picture == FALSE) {   // loop over codingUnits
            lcuIndex++;
            set_MB_parameters(CurrentMbNumber);
            pix_x = (img->current_mb_nr % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
            pix_y = (img->current_mb_nr / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;

            if (pix_x + size >= img->width) {
                num_of_orgMB_in_row = (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT;
            } else {
                num_of_orgMB_in_row = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);
            }

            if (pix_y + size >= img->height) {
                num_of_orgMB_in_col = (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT;
            } else {
                num_of_orgMB_in_col = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);
            }

            num_of_orgMB = num_of_orgMB_in_col * num_of_orgMB_in_row;

            //if ( ( input->slice_set_enable || ( input->slice_row_nr ) && ( img->current_mb_nr == 0 || ( img->mb_data[img->current_mb_nr].slice_nr != img->mb_data[img->current_mb_nr - 1].slice_nr ) ) ) )
            if (img->current_mb_nr == 0 || (img->current_mb_nr > 0 &&
                                            img->mb_data[img->current_mb_nr].slice_nr != img->mb_data[img->current_mb_nr - 1].slice_nr)) {

                old_bit = currBitStream->byte_pos * 8 + 8 - currBitStream->bits_to_go ;
                start_slice();
                len = SliceHeader(slice_nr, slice_qp);
                {
                    int mb_width = img->width / MIN_BLOCK_SIZE;
                    int mb_height = img->height / MIN_BLOCK_SIZE;
                    int cuInPicWidth = img->width / MIN_CU_SIZE;   //8
                    int cuInPicHeight = img->height / MIN_CU_SIZE; //8
                    int mb_x = (img->current_mb_nr % cuInPicWidth) << 1;
                    int mb_y = (img->current_mb_nr / cuInPicWidth) << 1;
                    int curr_minBlock_nr = mb_y * mb_width + mb_x;
                    int block_i, block_j;

                    for (block_j = 0; block_j < (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)); block_j++) {
                        int currPos = Clip3(0, mb_height, ((preSlicePos / cuInPicWidth) << 1) + block_j) * mb_width + ((
                                          preSlicePos % cuInPicWidth) << 1);
                        int endPos = min((currPos / mb_width + 1) * mb_width, curr_minBlock_nr + block_j * mb_width);
                        for (block_i = currPos; block_i < endPos; block_i++) {
                            img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
                        }
                    }
                    for (; block_i < curr_minBlock_nr - mb_x; block_i++) {
                        img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
                    }
                    for (block_j = 0; block_j < (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)); block_j++) {
                        //int currPos = max((preSlicePos<<2)+block_j*mb_width, Clip3(0, mb_height, mb_y + block_j)*mb_width);
                        //int currPos = max(Clip3(0, mb_height, ((preSlicePos/cuInPicWidth)<<1)+block_j)*mb_width+((preSlicePos%cuInPicWidth)<<1), Clip3(0, mb_height, mb_y + block_j)*mb_width);
                        int currPos = max(block_i, Clip3(0, mb_height, mb_y + block_j) * mb_width);
                        int endPos = Clip3(0, mb_height, (mb_y + block_j)) * mb_width + mb_x;
                        for (block_i = currPos; block_i < endPos; block_i++) {
                            img->ipredmode[1 + block_i / mb_width][1 + (block_i % mb_width)] = -1;
                        }
                    }
                    preSlicePos = img->current_mb_nr;
                }
                pic->no_slices++;
                img->currentSlice = pic->slices[pic->no_slices - 1];
                img->currentSlice->start_mb_nr = img->current_mb_nr;
                currSlice = img->currentSlice;
                for (i = 0; i < 1; i++) {
                    (currSlice->partArr[i]).bitstream = currBitStream;
                    currStream = (currSlice->partArr[i]).bitstream;
                    currStream->write_flag = 0;
                    eep = & ((currSlice->partArr[i]).ee_AEC);

                    if (currBitStream->bits_to_go < 8) {
                        // trailing bits to process
                        currBitStream->byte_buf = (currBitStream->byte_buf << currBitStream->bits_to_go) | (0xff >>
                                                  (8 - currBitStream->bits_to_go));
                        stat->bit_use_stuffingBits[img->type] += currBitStream->bits_to_go;
                        currBitStream->streamBuffer[currStream->byte_pos++] = currBitStream->byte_buf;
                        currBitStream->bits_to_go = 8;
                    }

                    arienco_start_encoding(eep, currStream->streamBuffer, & (currStream->byte_pos), img->type);
                }

                init_contexts();
                stat->bit_slice += len;
                slice_nr++;
                he->slice_header[img->type == F_IMG ? P_IMG : img->type] += (currBitStream->byte_pos * 8 + 8 - currBitStream->bits_to_go
                        - old_bit);

            }
            he->AEC_writting = 1;

            if (img->current_mb_nr != img->currentSlice->start_mb_nr) {    //qyu 0906
                write_terminating_bit(0);
            }

            if (input->sao_enable) {
                writeParaSAO_one_SMB(smb_index, pix_y >> MIN_CU_SIZE_IN_BIT, pix_x >> MIN_CU_SIZE_IN_BIT,  num_of_orgMB_in_col,
                                     num_of_orgMB_in_row, img->slice_sao_on, img->saoBlkParams[smb_index]);

            }
            if (input->alf_enable) {
                dataPart = &(currSlice->partArr[0]);
                for (compIdx = 0; compIdx < NUM_ALF_COMPONENT; compIdx++) {
                    if (img->pic_alf_on[compIdx]) {
                        ctx_idx = 0;
                        writeAlfLCUCtrl((int)Enc_ALF->m_AlfLCUEnabled[smb_index][compIdx], dataPart, compIdx, ctx_idx);
                    }
                }
            }
            if (input->sao_enable || input->alf_enable) {
                smb_index++;
            }
            write_one_SMB(input->g_uiMaxSizeInBit, img->current_mb_nr, 1);

            //multiple slice
            //if ( input->slice_row_nr > 0 && (( ( img->coded_mb_nr + num_of_orgMB - img->slice_offset ) % ( input->slice_row_nr * ( 1 << ( input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT ) ) ) == 0 ) || ( img->coded_mb_nr + num_of_orgMB == img->PicSizeInMbs )) )
            if ((lcuIndex % input->slice_row_nr == 0) || (img->coded_mb_nr + num_of_orgMB == img->PicSizeInMbs)) {
                terminate_slice();
            }
            terminate_codingUnit(&end_of_picture, lcuIndex);
            proceed2nextcodingUnit();
            if (end_of_picture) {
                smb_index = 0;
            }
            if ((CurrentMbNumber + num_of_orgMB_in_row) % MBRowSize == 0) {     //end of the row
                CurrentMbNumber += (num_of_orgMB_in_row + MBRowSize * ((1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT)) -
                                    1));   //eg. 64 +: (4-1)*num_mb_inWidth+4
            } else {
                CurrentMbNumber += num_of_orgMB_in_row;
            }
        }

        for (i = 0; i < NUM_ALF_COMPONENT; i++) {
            freeAlfPar(alfPictureParam[i], i);
        }
    }

    old_bit = currBitStream->byte_pos * 8 + 8 - currBitStream->bits_to_go ;
    terminate_picture();
    stat->bit_use_stuffingBits[img->type] += (currBitStream->byte_pos * 8 + 8 - currBitStream->bits_to_go - old_bit);

    if ((img->type == P_IMG || img->type == F_IMG) && input->bg_enable && he->background_reference_enable) {
        int j, i;
        for (j = 0; j < img->height / MIN_BLOCK_SIZE; j++)
            for (i = 0; i < img->width / MIN_BLOCK_SIZE; i++) {
                int refframe  = hc->refFrArr[j][i];
                if (refframe == img->num_of_references - 1) {
                    hc->refFrArr[j][i] = -1;
                }
            }
    }
    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        int j, i;
        for (j = 0; j < img->height / MIN_BLOCK_SIZE; j++) {
            for (i = 0; i < img->width / MIN_BLOCK_SIZE; i++) {
                hc->refFrArr[j][i] = -1;
            }
        }
    }


}


void BitStreamCopy(Bitstream *BitStream_dst, Bitstream *BitStream_src)
{
    const int buffer_size = (img->width * img->height * 4);
    BitStream_dst->bits_to_go = BitStream_src->bits_to_go;
    BitStream_dst->byte_buf = BitStream_src->byte_buf;
    BitStream_dst->byte_pos = BitStream_src->byte_pos;
    BitStream_dst->write_flag = BitStream_src->write_flag;

    memcpy(BitStream_dst->streamBuffer, BitStream_src->streamBuffer, buffer_size * sizeof(unsigned char));
}


void AllocateBitstream_ALF()
{
    const int buffer_size = (img->width * img->height * 4);

    {
        if ((currBitStream_ALF = (Bitstream *) calloc(1, sizeof(Bitstream))) == NULL) {
            no_mem_exit("malloc_slice: Bitstream");
        }
        if ((currBitStream_ALF->streamBuffer = (unsigned char *) calloc(buffer_size,
                                               sizeof(unsigned char))) == NULL) {
            no_mem_exit("malloc_slice: StreamBuffer");
        }

        currBitStream_ALF->bits_to_go = 8;
    }
}

void FreeBitstream_ALF()
{
    if (currBitStream_ALF->streamBuffer) {
        free(currBitStream_ALF->streamBuffer);
    }
    if (currBitStream_ALF) {
        free(currBitStream_ALF);
    }
}
/*
*************************************************************************
* Function: allocates the memory for the coded picture data
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void AllocateBitstream()
{
    const int buffer_size = (img->width * img->height * 4);

    {
        if ((currBitStream = (Bitstream *) calloc(1, sizeof(Bitstream))) == NULL) {
            no_mem_exit("malloc_slice: Bitstream");
        }
        if ((currBitStream->streamBuffer = (unsigned char *) calloc(buffer_size, sizeof(unsigned char))) == NULL) {
            no_mem_exit("malloc_slice: StreamBuffer");
        }

        currBitStream->bits_to_go = 8;
    }
}
/*
*************************************************************************
* Function:free the allocated memory for the coded picture data
* Input:
* Output:
* Return:
*************************************************************************
*/
void FreeBitstream()
{
    if (currBitStream->streamBuffer) {
        free(currBitStream->streamBuffer);
    }

    if (currBitStream) {
        free(currBitStream);
    }
}

//////////////////////////////////////////////////////////////////////////
////////////********************* qhg add 20060327*****************///////
//////////////////////////////////////////////////////////////////////////
void Demulate(Bitstream *currStream, int current_slice_bytepos)
{
    int i, j;
    unsigned int rawbitsequence = -1;
    int bitvalue, nonzero, bitnum;

    if (!(currBitStream->streamBuffer[current_slice_bytepos] == 0 &&
          currBitStream->streamBuffer[current_slice_bytepos + 1] == 0 &&
          currBitStream->streamBuffer[current_slice_bytepos + 2] == 1)) {
        printf("Fatal bitstream error");
    }

    AllocatetmpBitstream();

    bitnum = 8;
    he->tmpStream->bits_to_go = 0;
    currStream->streamBuffer[currStream->byte_pos] = currStream->byte_buf << currStream->bits_to_go;

    for (he->tmpStream->byte_pos = i = current_slice_bytepos + 3; i <= currStream->byte_pos; i++) {
        for (j = 8; j > (i == currStream->byte_pos ? currStream->bits_to_go : 0); j--) {
            bitvalue = currBitStream->streamBuffer[i] & (0x01 << (j - 1));

            if (bitnum == 2) {
                nonzero = rawbitsequence & 0x003fffff;

                if (!nonzero) {
                    he->tmpStream->streamBuffer[he->tmpStream->byte_pos] = 0x02;
                    he->tmpStream->byte_pos++;
                    he->tmpStream->bits_to_go += 2;
                    rawbitsequence = 0x00000002;
                    bitnum = 8;
                }
            }

            rawbitsequence <<= 1;

            if (bitvalue) {
                he->tmpStream->streamBuffer[he->tmpStream->byte_pos] |= 1 << (bitnum - 1);
                rawbitsequence |= 1;
            } else {
                he->tmpStream->streamBuffer[he->tmpStream->byte_pos] &= (~(1 << (bitnum - 1)));
            }

            bitnum--;
            he->tmpStream->bits_to_go++;

            if (bitnum == 0) {
                bitnum = 8;
                he->tmpStream->byte_pos++;
            }
        }

    }

    for (i = current_slice_bytepos + 3; i <= he->tmpStream->byte_pos; i++) {
        currStream->streamBuffer[i] = he->tmpStream->streamBuffer[i];
    }

    currStream->byte_pos = he->tmpStream->byte_pos;
    currStream->bits_to_go = 8 - he->tmpStream->bits_to_go % 8;
    currStream->byte_buf = he->tmpStream->streamBuffer[he->tmpStream->byte_pos] >> currStream->bits_to_go;
    FreetmpBitstream();
}

void AllocatetmpBitstream()
{
    const int buffer_size = (img->width * img->height * 4);   //add by wuzhongmou 0610

    if ((he->tmpStream = (Bitstream *)calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("malloc_slice: Bitstream");
    }
    if ((he->tmpStream->streamBuffer = (unsigned char *)calloc(buffer_size, sizeof(unsigned char))) == NULL) {
        no_mem_exit("malloc_slice: StreamBuffer");
    }

    he->tmpStream->bits_to_go = 8;
}

void FreetmpBitstream()
{
    if (he->tmpStream->streamBuffer) {
        free(he->tmpStream->streamBuffer);
    }

    if (he->tmpStream) {
        free(he->tmpStream);
    }
}
/*!
************************************************************************
* \brief
*    Initializes the parameters for a new slice and
*     allocates the memory for the coded slice in the Picture structure
*  \par Side effects:
*      Adds slice/partition header symbols to the symbol buffer
*      increments Picture->no_slices, allocates memory for the
*      slice, sets img->currSlice
*  \author
*      added by lzhang for AEC
************************************************************************
*/
void init_slice(int start_mb_addr, Picture *currPic)
{
    int i;
    Slice *curr_slice;
    DataPartition *dataPart;

    img->current_mb_nr = start_mb_addr;

    // Allocate new Slice in the current Picture, and set img->currentSlice
    assert(currPic != NULL);
    if (img->current_mb_nr == 0) {
        currPic->no_slices = 0;
    }
    currPic->no_slices += 1;
    if (currPic->no_slices > MAXSLICEPERPICTURE) {
        error("Too many slices per picture, increase MAXLSLICESPERPICTURE in global.h.", -1);
    }

    currPic->slices[currPic->no_slices - 1] = malloc_slice();
    curr_slice = currPic->slices[currPic->no_slices - 1];
    img->currentSlice = curr_slice;

    curr_slice->qp = img->qp;

#if MB_DQP
    he->lastQP = curr_slice->qp;
#endif

    curr_slice->start_mb_nr = start_mb_addr;

    for (i = 0; i < 1; i++) {
        dataPart = & (curr_slice->partArr[i]);
        dataPart->writeSyntaxElement = writeSyntaxElement_AEC;
    }
}
void init_slice_Bitstream(int start_mb_addr, Picture *currPic)
{
    int i;
    Slice *curr_slice;
    DataPartition *dataPart;
    Bitstream *currStream;

    img->current_mb_nr = start_mb_addr;

    // Allocate new Slice in the current Picture, and set img->currentSlice
    assert(currPic != NULL);
    currPic->no_slices = 1;

    if (currPic->no_slices > MAXSLICEPERPICTURE) {
        error("Too many slices per picture, increase MAXLSLICESPERPICTURE in global.h.", -1);
    }
    curr_slice = img->currentSlice;

    for (i = 0; i < 1; i++) {
        dataPart = & (curr_slice->partArr[i]);
        dataPart->writeSyntaxElement = writeSyntaxElement_AEC;

        currStream = dataPart->bitstream;
        currStream->bits_to_go = 8;
        currStream->byte_pos = 0;
        currStream->byte_buf = 0;
    }
}

/*!
************************************************************************
* \brief
*    Allocates a slice structure along with its dependentdata structures
* \return
*    Pointer to a Slice
*  \author
*      added by lzhang for AEC
************************************************************************
*/
static Slice *malloc_slice()
{
    Slice *slice;

    if ((slice = (Slice *) calloc(1, sizeof(Slice))) == NULL) {
        no_mem_exit("malloc_slice: slice structure");
    }

    // create all context models
    slice->syn_ctx = create_contexts_SyntaxInfo();

    if ((slice->partArr = (DataPartition *) calloc(1, sizeof(DataPartition))) == NULL) {
        no_mem_exit("malloc_slice: partArr");
    }

    return slice;
}
/*!
************************************************************************
* \brief
*    This function terminates a slice (but doesn't write it out),
*    the old terminate_slice (0)
* \return
*    0 if OK,                                                         \n
*    1 in case of error
*  \author
*      added by lzhang for AEC
*
************************************************************************
*/
int terminate_slice()
{
    Bitstream *currStream;
    Slice *currSlice = img->currentSlice;
    EncodingEnvironmentPtr eep;
    int i;

    write_terminating_bit(1);

    for (i = 0; i < 1; i++) {
        currStream = (currSlice->partArr[i]).bitstream;
        eep = & ((currSlice->partArr[i]).ee_AEC);
        arienco_done_encoding(eep);
        currStream->bits_to_go = eep->Ebits_to_go;
        currStream->byte_buf   = eep->Ebuffer;
    }

    return 0;
}

