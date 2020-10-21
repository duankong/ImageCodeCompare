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

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <assert.h>
#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "configfile.h"
#include "../../lcommon/inc/memalloc.h"
#include "image.h"
#include "header.h"
//#include "C:\Program Files (x86)\Visual Leak Detector\include\vld.h"
#include "vlc.h"
#include "AEC.h"
#include "bitstream.h"


#include "fast_me.h"

#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif


#include "../../lencod/inc/EncAdaptiveLoopFilter.h"

#if ROI_M3264
#include "pos_info.h"
#endif

#if TDRDO
#include "tdrdo.h"
#endif

#if RATECONTROL
#include "ratecontrol.h"
#endif


#include "../../lcommon/inc/md5.h"
unsigned int   MD5val[4];
char           MD5str[33];


#define _S_IREAD      0000400         /* read permission, owner */
#define _S_IWRITE     0000200         /* write permission, owner */

InputParameters inputs, *input = &inputs;
ImageParameters images, *img   = &images;
SNRParameters   snrs,   *snr   = &snrs;
StatParameters  stats, *stat = &stats;


Bitstream *currStream;
int    AEC_encoding = 0;

Video_Enc_data  hh;
Video_Enc_data *he = &hh;
/*
*************************************************************************
* Function:Main function for encoder.
* Input:argc
number of command line arguments
argv
command line arguments
* Output:
* Return: exit code
* Attention:
*************************************************************************
*/


void Init_Motion_Search_Module();
void Clear_Motion_Search_Module();
void init_frame_buffers(InputParameters *inp, ImageParameters *img);
void InitZigzagScan(int **scan, unsigned int uiBitSize);
void InitZigzagScan_NSQT(int **scan, int iWidth, int iHeight);
void  InitCGZigzagScan(int **scan, int iWidth, int iHeight);
extern void clear_sao_entropy();
extern void init_sao_entropy();
#if INTERLACE_CODING
void mergeFld2Frm();
void readOneFrameFromDisk(long long int framesize_in_bytes, int FrameNoInFile, FILE *p_img, unsigned char   *buf);
#endif

int main(int argc, char **argv)
{
    int len = 0;
    int no_IPframes;
    int  tmp_buf_cycle;    // jlzheng 7.21

#if RATECONTROL
    RateControl RC;
    RCLCU       RClcu;
#endif

    int j;
    int delta_dd = 100;
    int tmp_delta_dd;
    int tmp_pdd = 0;
    int subgopOffset0 = 0;
    int subgopOffset1 = 0;
    int cfg_dd_poc[32];
    int flag_dd = 0;
    int temp_tr;
    int i;
    int k, c;
    he->subGopNum = 0;
    img->numIPFrames = 0;
    he->p_dec = he->p_stat = hc->p_log = hc->p_trace = NULL;

#if AVS2_S2_FASTMODEDECISION
    he->sum_diff_32x32 = 0;
    he->sum_diff_64x64 = 0;
#endif

    hc->coding_order = -1;
    he->flag_gop = 0;
    {
        he->last_output = -1;
        he->curr_IDRtr = 0;
        he->curr_IDRcoi = 0;
        he->next_IDRtr = 0;
        he->next_IDRcoi = 0;
        he->use_RPSflag = 1;

    }

    hc->seq_header = 0;
    he->slice_header[0] = he->slice_header[1] = he->slice_header[2] = 0;

    Configure(argc, argv);

#if ROI_M3264
    OpenPosFile(input->infile_data);
#endif

    hc->total_frames = 0;

    init_img();
    {
        for (i = 0; i < 4; i++) {
            input->successive_Bframe_sub[i] = (input->successive_Bframe_all >> (i * 8)) & (0xFF);
            input->jumpd_sub[i] = (input->jumpd_all >> (i * 8)) & (0xFF);
            if (input->successive_Bframe_sub[i]) {
                he->subGopNum++;
            } else {
                break;
            }
        }
        he->subGopNum = he->subGopNum == 0 ? 1 : he->subGopNum;

        input->successive_Bframe_all = input->successive_Bframe_sub[0] + input->successive_Bframe_sub[1] +
                                       input->successive_Bframe_sub[2] + input->successive_Bframe_sub[3];
        input->jumpd_all = input->jumpd_sub[0] + input->jumpd_sub[1] + input->jumpd_sub[2] + input->jumpd_sub[3] +
                           he->subGopNum;
        no_IPframes = (input->no_frames + (input->successive_Bframe_all + he->subGopNum - 1) - 1) /
                      (input->successive_Bframe_all + he->subGopNum) + 1;   //qyuI帧和P帧的数量  no_frames指的是真实的编码帧数
    }
    frame_pic = malloc_picture();

    init_rdopt();
    init_frame_buffers(input, img);

    init_global_buffers();
    Init_Motion_Search_Module();


    memset(MD5val, 0, 16);
    memset(MD5str, 0, 33);

    he->last_background_frame_number = 0;
    he->background_number = 0;
    he->background_output_flag = 1;
    he->duplicated_gb_flag = 0;
    he->gb_is_ready = 0;
    if (input->bg_enable) {
        if ((he->p_org_background = fopen(input->bg_file_name, "wb")) == NULL) {
            snprintf(hc->errortext, ET_SIZE, "Error open file %s", input->bg_file_name);
            error(hc->errortext, 500);
        }
    }

    if (input->bg_enable) {   //set input->always_no_bgmodel zl
        if (input->bg_input_number == 0) {
            input->always_no_bgmodel = 1;
            input->bg_input_number = 1;
        } else {
            input->always_no_bgmodel = 0;
        }

        if (input->bg_period && input->always_no_bgmodel == 0 &&
            input->bg_model_number > (input->bg_period * (input->successive_Bframe + 1))) {
            printf("\n bg_model_number invalid or unsupported.\n");
            exit(-1);
        }
    }

    if (input->bg_enable) {
        if ((input->no_frames * (1 + input->successive_Bframe) - input->successive_Bframe) - input->bg_model_number >
            0) { //set total_bg_number zl
            he->total_bg_number = 1 + ((input->no_frames * (1 + input->successive_Bframe) - input->successive_Bframe) -
                                       input->bg_model_number) / input->bg_period / (1 + input->successive_Bframe);
        } else {
            he->total_bg_number = 0;
        }
    }

    img->typeb_before = 0;
    he->background_output_flag_before = 1;


    init_sao_entropy();
#if !TH_ME
    if (input->usefme) {
        DefineThreshold();
    }
#endif
    get_mem2Dint(&AVS_SCAN8x8, 64, 2);
    get_mem2Dint(&AVS_SCAN16x16, 256, 2);
    get_mem2Dint(&AVS_SCAN32x32, 1024, 2);
    InitZigzagScan(AVS_SCAN8x8, 3);
    InitZigzagScan(AVS_SCAN16x16, 4);
    InitZigzagScan(AVS_SCAN32x32, 5);
    get_mem2Dint(&AVS_SCAN4x4, 16, 2);
    InitZigzagScan(AVS_SCAN4x4, 2);
    get_mem2Dint(&AVS_CG_SCAN8x8, 64, 2);
    get_mem2Dint(&AVS_CG_SCAN16x16, 256, 2);
    get_mem2Dint(&AVS_CG_SCAN32x32, 1024, 2);
    get_mem2Dint(&AVS_SCAN4x16, 64, 2);
    get_mem2Dint(&AVS_SCAN16x4, 64, 2);
    get_mem2Dint(&AVS_SCAN2x8, 16, 2);
    get_mem2Dint(&AVS_SCAN8x2, 16, 2);
    get_mem2Dint(&AVS_SCAN32x8, 256, 2);
    get_mem2Dint(&AVS_SCAN8x32, 256, 2);
    InitCGZigzagScan(AVS_SCAN4x16, 16, 4);
    InitCGZigzagScan(AVS_SCAN16x4, 4, 16);
    InitCGZigzagScan(AVS_SCAN32x8, 8, 32);
    InitCGZigzagScan(AVS_SCAN8x32, 32, 8);

    InitCGZigzagScan(AVS_CG_SCAN8x8, 8, 8);
    InitCGZigzagScan(AVS_CG_SCAN16x16, 16, 16);
    InitCGZigzagScan(AVS_CG_SCAN32x32, 32, 32);

    InitZigzagScan_NSQT(AVS_SCAN2x8, 8, 2);
    InitZigzagScan_NSQT(AVS_SCAN8x2, 2, 8);
    memset(g_log2size, -1, MAX_CU_SIZE + 1);
    c = 2;
    for (k = 4; k <= MAX_CU_SIZE; k *= 2) {
        g_log2size[k] = c;
        c++;
    }

    information_init();


    // B pictures
    hc->Bframe_ctr = 0;
    hc->tot_time = 0;               // time for total encoding session
    if (input->low_delay == 0) {
        for (i = 0; i < he->gop_size_all; i++) {
            cfg_dd_poc[i] = he->cfg_ref_all[i].poc + subgopOffset0;
            if (i == input->jumpd_sub[0] && input->jumpd_sub[0] != 0) {
                subgopOffset0 = input->jumpd_sub[0] + 1;
            }

            if (i == input->jumpd_sub[1] + input->jumpd_sub[0] + 1 && input->jumpd_sub[0] != 0 && input->jumpd_sub[1] != 0) {
                subgopOffset0 = input->jumpd_sub[1] + 1;
            }

            if (i == input->jumpd_sub[2] + input->jumpd_sub[1] + input->jumpd_sub[0] + 2 && input->jumpd_sub[0] != 0 &&
                input->jumpd_sub[1] != 0 && input->jumpd_sub[2] != 0) {
                subgopOffset0 = input->jumpd_sub[2] + 1;
            };
        }
        if ((input->no_frames) <= input->jumpd_sub[0] + 2) { //<sub[0]
            for (j = 0; j < input->no_frames - 1; j++) {
                if (j == 0) {
                    cfg_dd_poc[j] = input->no_frames - 1;
                } else {
                    int offset = j;
                    int flag_dd = 0;
                    temp_tr = cfg_dd_poc[offset];
                    while (temp_tr >= input->no_frames - 1 || cfg_dd_poc[offset] == -1) {
                        int offset_tmp = 0;
                        flag_dd++;
                        offset_tmp = j + flag_dd;
                        temp_tr = cfg_dd_poc[offset_tmp];
                        cfg_dd_poc[offset_tmp] = -1;
                        if (temp_tr == cfg_dd_poc[0]) {
                            cfg_dd_poc[j] = -1;
                        } else {
                            cfg_dd_poc[j] = temp_tr;
                        }


                    }
                }
            }
            for (j = 0; j < input->no_frames - 1; j++) {
                tmp_delta_dd = cfg_dd_poc[j] - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }
            }
            if (delta_dd < 0) {
                he->picture_reorder_delay = abs(delta_dd);
            } else {
                he->picture_reorder_delay = 0;
            }
        }
        if (input->jumpd_sub[1] + input->jumpd_sub[0] + 3 >= (input->no_frames) &&
            (input->no_frames) > input->jumpd_sub[0] + 2 && input->jumpd_sub[1] != 0) {  //less than sub[1]
            for (j = input->jumpd_sub[0] + 1; j < input->no_frames - 1; j++) {
                if (j == input->jumpd_sub[0] + 1) {
                    cfg_dd_poc[j] = input->no_frames - 1;
                } else {
                    int offset = j;
                    int flag_dd = 0;
                    temp_tr = cfg_dd_poc[offset];
                    while (temp_tr >= input->no_frames - 1 || cfg_dd_poc[offset] == -1) {
                        int offset_tmp = 0;
                        flag_dd++;
                        offset_tmp = j + flag_dd;
                        temp_tr = cfg_dd_poc[offset_tmp];
                        cfg_dd_poc[offset_tmp] = -1;
                        if (temp_tr == cfg_dd_poc[0]) {
                            cfg_dd_poc[j] = -1;
                        } else {
                            cfg_dd_poc[j] = temp_tr;
                        }

                    }
                }
            }
            for (j = input->jumpd_sub[0] + 1; j < input->no_frames - 1; j++) {
                tmp_delta_dd = cfg_dd_poc[j] - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }
            }
            if (delta_dd < 0) {
                he->picture_reorder_delay = abs(delta_dd);
            } else {
                he->picture_reorder_delay = 0;
            }

            for (j = 0; j < input->jumpd_sub[0] + 1; j++) {
                tmp_delta_dd = cfg_dd_poc[j] - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }
            }
            if (delta_dd < 0) {
                tmp_pdd = abs(delta_dd);
            } else {
                tmp_pdd = 0;
            }
            he->picture_reorder_delay = max(tmp_pdd, he->picture_reorder_delay);
        }

        if (input->jumpd_sub[2] + input->jumpd_sub[1] + input->jumpd_sub[0] + 4 >= (input->no_frames) &&
            (input->no_frames) > input->jumpd_sub[1] + input->jumpd_sub[0] + 3
            && input->jumpd_sub[2] != 0 && input->jumpd_sub[1] != 0) { //less than sub[2]
            for (j = input->jumpd_sub[0] + input->jumpd_sub[1] + 2; j < input->no_frames - 1; j++) {
                if (j == input->jumpd_sub[0] + input->jumpd_sub[1] + 2) {
                    cfg_dd_poc[j] = input->no_frames - 1;
                } else {
                    int offset = j;
                    int flag_dd = 0;
                    temp_tr = cfg_dd_poc[offset];
                    while (temp_tr >= input->no_frames - 1 || cfg_dd_poc[offset] == -1) {
                        int offset_tmp = 0;
                        flag_dd++;
                        offset_tmp = j + flag_dd;
                        temp_tr = cfg_dd_poc[offset_tmp];
                        cfg_dd_poc[offset_tmp] = -1;
                        if (temp_tr == cfg_dd_poc[input->jumpd_sub[0] + input->jumpd_sub[1] + 2]) {
                            cfg_dd_poc[j] = -1;
                        } else {
                            cfg_dd_poc[j] = temp_tr;
                        }


                    }
                }
            }
            for (j = input->jumpd_sub[0] + input->jumpd_sub[1] + 2; j < input->no_frames - 1; j++) {
                tmp_delta_dd = cfg_dd_poc[j] - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }
            }
            if (delta_dd < 0) {
                he->picture_reorder_delay = abs(delta_dd);
            } else {
                he->picture_reorder_delay = 0;
            }

            for (j = 0; j < input->jumpd_sub[0] + input->jumpd_sub[1] + 2; j++) {
                tmp_delta_dd = cfg_dd_poc[j] - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }
            }
            if (delta_dd < 0) {
                tmp_pdd = abs(delta_dd);
            } else {
                tmp_pdd = 0;
            }
            he->picture_reorder_delay = max(tmp_pdd, he->picture_reorder_delay);
        }

        if (input->jumpd_sub[3] + input->jumpd_sub[2] + input->jumpd_sub[1] + input->jumpd_sub[0] + 5 >= (input->no_frames) &&
            (input->no_frames) > input->jumpd_sub[2] + input->jumpd_sub[1] + input->jumpd_sub[0] + 4
            && input->jumpd_sub[3] != 0  && input->jumpd_sub[2] != 0 && input->jumpd_sub[1] != 0) {
            for (j = input->jumpd_sub[0] + input->jumpd_sub[1] + input->jumpd_sub[2] + 3; j < input->no_frames - 1; j++) {
                if (j == input->jumpd_sub[0] + input->jumpd_sub[1] + input->jumpd_sub[2] + 3) {
                    cfg_dd_poc[j] = input->no_frames - 1;
                } else {
                    int offset = j;
                    int flag_dd = 0;
                    temp_tr = cfg_dd_poc[offset];
                    while (temp_tr >= input->no_frames - 1 || cfg_dd_poc[offset] == -1) {
                        int offset_tmp = 0;
                        flag_dd++;
                        offset_tmp = j + flag_dd;
                        temp_tr = cfg_dd_poc[offset_tmp];
                        cfg_dd_poc[offset_tmp] = -1;
                        if (temp_tr == cfg_dd_poc[input->jumpd_sub[0] + input->jumpd_sub[1] + input->jumpd_sub[2] + 3]) {
                            cfg_dd_poc[j] = -1;
                        } else {
                            cfg_dd_poc[j] = temp_tr;
                        }


                    }
                }
            }
            for (j = input->jumpd_sub[0] + input->jumpd_sub[1] + input->jumpd_sub[2] + 3; j < input->no_frames - 1; j++) {
                tmp_delta_dd = cfg_dd_poc[j] - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }
            }
            if (delta_dd < 0) {
                he->picture_reorder_delay = abs(delta_dd);
            } else {
                he->picture_reorder_delay = 0;
            }

            for (j = 0; j < input->jumpd_sub[0] + input->jumpd_sub[1] + input->jumpd_sub[2] + 3; j++) {
                tmp_delta_dd = cfg_dd_poc[j] - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }
            }
            if (delta_dd < 0) {
                tmp_pdd = abs(delta_dd);
            } else {
                tmp_pdd = 0;
            }
            he->picture_reorder_delay = max(tmp_pdd, he->picture_reorder_delay);
        }


        if (input->no_frames > he->gop_size_all) {
            for (j = 0; j < he->gop_size_all; j++) {
                tmp_delta_dd = he->cfg_ref_all[j].poc + subgopOffset1 - (j + 1);
                if (tmp_delta_dd < delta_dd) {
                    delta_dd = tmp_delta_dd;
                }

                if (j == input->jumpd_sub[0] && input->jumpd_sub[0] != 0) {
                    subgopOffset1 = input->jumpd_sub[0] + 1;
                }

                if (j == input->jumpd_sub[1] + input->jumpd_sub[0] + 1 && input->jumpd_sub[0] != 0 && input->jumpd_sub[1] != 0) {
                    subgopOffset1 = input->jumpd_sub[1] + 1;
                }

                if (j == input->jumpd_sub[2] + input->jumpd_sub[1] + input->jumpd_sub[0] + 2 && input->jumpd_sub[0] != 0 &&
                    input->jumpd_sub[1] != 0 && input->jumpd_sub[2] != 0) {
                    subgopOffset1 = input->jumpd_sub[2] + 1;
                }
            }
            if (delta_dd < 0) {
                he->picture_reorder_delay = abs(delta_dd);
            } else {
                he->picture_reorder_delay = 0;
            }
        }
    }

#if TDRDO
    OMCPDList = RealDList = NULL;
    porgF = ppreF = precF = prefF = NULL;
    KappaTable = NULL;
    KappaTable1 = NULL;
    StepLength = input->successive_Bframe_all == 0 ? 1 : he->gop_size_all;
    input->TDEnable = (input->intra_period == 0 && input->successive_Bframe_all == 0) ? input->TDEnable : 0; // Just for LD
    if (input->TDEnable != 0) {

        int no_frames_add = 0;
        if (input->bg_enable) {
            no_frames_add = ((input->no_frames - input->bg_model_number + input->bg_period - 1) / input->bg_period);
        }
        if (!input->successive_Bframe_all) {
            OMCPDList = CreatDistortionList(input->no_frames / StepLength + no_frames_add, input->img_width, input->img_height,
                                            WORKBLOCKSIZE, 1 << input->g_uiMaxSizeInBit);
            RealDList = CreatDistortionList(input->no_frames / StepLength + no_frames_add, input->img_width, input->img_height,
                                            WORKBLOCKSIZE, 1 << input->g_uiMaxSizeInBit);
        } else {
            OMCPDList = CreatDistortionList((input->no_frames - 1) / StepLength + no_frames_add, input->img_width,
                                            input->img_height, WORKBLOCKSIZE, 1 << input->g_uiMaxSizeInBit);
            RealDList = CreatDistortionList(input->no_frames + no_frames_add, input->img_width, input->img_height, WORKBLOCKSIZE,
                                            1 << input->g_uiMaxSizeInBit);
        }


        porgF = (Frame *)calloc(1, sizeof(Frame));
        ppreF = (Frame *)calloc(1, sizeof(Frame));
        precF = (Frame *)calloc(1, sizeof(Frame));
        prefF = (Frame *)calloc(1, sizeof(Frame));
        porgF->FrameWidth  = input->img_width;
        porgF->FrameHeight = input->img_height;
        porgF->nStrideY = input->img_width;
        porgF->nStrideC = input->img_width / 2;
        *ppreF = *precF = *prefF = *porgF;
    }
#endif

#if AQPO
#if AQPOM3694 || AQPOM3762
    for (i = 0; i < AQPStepLength; i++) {
        preGopQPF[i] = input->qpI;
    }
    GopQpbase = input->qpI;
#endif
    AQPStepLength = he->gop_size_all;
    {
        int i;
        MaxQPoffset = 0;
        memset(AQPoffset, 0, sizeof(AQPoffset));
        for (i = 0; i < AQPStepLength; i++) {
            AQPoffset[i] = he->cfg_ref_all[i].qp_offset;
            MaxQPoffset = AQPoffset[i] > MaxQPoffset ? AQPoffset[i] : MaxQPoffset;
        }
    }
#if AQPOM3694 || AQPOM3762
    if (input->AQPEnable) {
        if (input->intra_period == 1) { // AI
            input->AQPEnable = 0;
            printf("Adaptive QP offset does not support AI. AQPEnable is reset to 0. \n");
        } else if (!input->successive_Bframe) { // LD
            if (!input->TDEnable) {
                input->AQPEnable = 0;
                printf("Adaptive QP offset does not work when TDRDOEnable is 0 for LD. AQPEnable is reset to 0. \n");
            }
        } else { // RA
            input->AQPEnable = 1;
        }
    }
#else
    if (input->AQPEnable == 1 && (input->intra_period == 1 || input->successive_Bframe_all != 0)) {
        input->AQPEnable = 0;
        printf("Adaptive QP offset does not support AI or RA. AQPEnable is reset to 0. \n");
    }
#endif
#endif

#if RATECONTROL
    if (input->EncControl != 0) {
        pRCLCU = & RClcu;
        memset(pRCLCU, 0, sizeof(RCLCU));
        if (input->EncControl == 1 && input->intra_period > 1 && input->useDQP == 1) {
            input->useDQP = 0;
            printf("LCU Rate Control does not support RA. DeltaQP is reset to 0. \n");
        }

        pRC = & RC;
        memset(pRC, 0, sizeof(RateControl));
        {
            unsigned int i;
            int IniQP = (input->EncControl == 0 || input->ECInitialQP == 0) ? input->qpI : input->ECInitialQP;
            char filename[256], *ch;

            for (i = (unsigned int)strlen(input->infile); (i >= 0 && input->infile[i] != '\\'); ch = &input->infile[i--]);

            strncpy(filename, ch, sizeof(filename));
            ch = filename;
            while (*ch != '\0' && *ch != '.') {
                ch++;
            }
            *ch = '\0';
            ch = filename;
            ch = NULL; // do not want record Frame/LCU buffer status of RC.
            Init_RateControl(pRC, input->EncControl, input->useDQP, input->no_frames, input->intra_period, input->TargetBitRate,
                             img->framerate, IniQP, input->img_width, input->img_height, ch);
        }
    }
#endif

    // Write sequence header
    stat->bit_use_header[3] = start_sequence();

    img->EncodeEnd_flag = 0;
    tmp_buf_cycle = img->buf_cycle;
    if (input->alf_enable) {
        createAlfGlobalBuffers(img);
    }

    for (img->number = 0; img->number - he->background_number < no_IPframes; img->number++)

    {
        for (i = 0; i < he->subGopNum; i++) {
            int j, tmp = 0;
            input->successive_Bframe = input->successive_Bframe_sub[i];
            input->jumpd = input->jumpd_sub[i];
            he->gop_size = he->subGopNum == 1 ? he->gop_size_all : input->successive_Bframe + 1;
            he->subGopID = i;
            for (j = 0; j < he->subGopID; j++) {
                tmp += (input->jumpd_sub[j] + 1);
            }
            for (j = 0; j < he->gop_size; j++) {
                he->cfg_ref[j] = he->cfg_ref_all[tmp + j];
            }
            tmp = ((img->numIPFrames) / he->subGopNum) * (input->successive_Bframe_all + he->subGopNum);
            for (j = 0; j < ((img->numIPFrames) % he->subGopNum); j++) {
                tmp += input->successive_Bframe_sub[j];
            }

            img->buf_cycle = tmp_buf_cycle;

            img->frame_num = img->number % (1 << (LOG2_MAX_FRAME_NUM_MINUS4 + 4));
            SetImgType();

            if (img->number != 0 && input->seqheader_period != 0 &&
                (img->numIPFrames % (input->seqheader_period * input->intra_period) == 0 || (img->typeb == BACKGROUND_IMG &&
                        he->background_output_flag)) && tmp + 1 < input->no_frames)

            {
                if (img->type == P_IMG && img->typeb == BP_IMG && img->number != 0 &&
                    (input->bg_enable & img->number) != (he->last_background_frame_number + 1)) {
                    img->type = INTRA_IMG;
                    img->typeb = BACKGROUND_IMG;
                    he->duplicated_gb_flag = 1;
                    he->background_output_flag = 0;
                    stat->bit_use_header[img->type] += len;


                    encode_one_frame();  //encode duplicated GB frame


                    img->type = P_IMG;
                    img->typeb = BP_IMG;
                    he->duplicated_gb_flag = 0;
                    he->background_output_flag = 1;
                }
                stat->bit_use_header[3] += terminate_sequence();

                if (img->number != 0 && input->vec_period != 0 &&
                    img->number % (input->vec_period * input->seqheader_period * input->intra_period) == 0) {
                    len =  WriteVideoEditCode();
                    stat->bit_use_header[3] += len;
                }

                stat->bit_use_header[3] += start_sequence();
                hc->seq_header ++;
                he->next_IDRtr = (img->number - he->background_number) * input->jumpd_all;

                he->next_IDRtr -= hc->total_frames * 256;
                he->next_IDRcoi = hc->coding_order + 1;
                img->num_ref_pic_active_fwd_minus1 = 0;   //yling

            }

            if (hc->seq_header > 1 && (img->type == INTRA_IMG || img->type == F_IMG || img->type == P_IMG)) {
                hc->seq_header = 0;
                img->num_of_references = 1;
            }
            if (hc->seq_header == 1 && (img->type == INTRA_IMG || img->typeb == BP_IMG))

            {
                hc->seq_header++;
            }
            if (hc->seq_header == 1) {
                hc->seq_header = 0;
                img->num_of_references = 1;
            }

            stat->bit_use_header[img->type] += len;

            // for 1 reference is used field coding (I bottom only 1 reference is used)

            encode_one_frame(); // encode one I- or P-frame

            if (img->typeb == BACKGROUND_IMG && (input->bg_input_number == 0) && img->number != 0 && input->bg_enable) {
                he->background_number++;
            }

            if (img->typeb == BACKGROUND_IMG  && input->bg_enable) {
                if (input->bg_input_number && input->always_no_bgmodel == 0) {
                    input->bg_input_number--;
                }
            }


            img->typeb_before = img->typeb;
            he->background_output_flag_before = he->background_output_flag;


            if ((input->successive_Bframe != 0) && img->number > 0 && he->background_output_flag != 0) {
                int no_b_frames;
                if (input->bg_enable)
                    no_b_frames = min(input->successive_Bframe,
                                      input->no_frames + input->successive_Bframe - (img->number - he->background_number) * (input->successive_Bframe + 1) -
                                      1);  //consider the last Pb...bP
                else {
                    no_b_frames = min(input->successive_Bframe,
                                      input->no_frames + input->successive_Bframe - img->number * (input->successive_Bframe + 1) -
                                      1);     //consider the last Pb...bP
                }

                img->typeb = 0;


                picture_coding_type = 1;

                //cjw for weighted prediction
                img->buf_cycle = tmp_buf_cycle;

                img->frame_num++;                 //increment frame_num once for B-frames
                img->frame_num %= (1 << (LOG2_MAX_FRAME_NUM_MINUS4 + 4));

                for (img->b_frame_to_code = 1; img->b_frame_to_code <= no_b_frames; img->b_frame_to_code++) {
                    img->type = B_IMG;            // set image type to B-frame
                    if (hc->coding_order + hc->total_frames * 256 + 1 >= (input->no_frames + he->background_number * input->bg_enable)) {
                        break;
                    }

                    encode_one_frame();  // encode one B-

                }
            }
            if (img->number == 0 || hc->coding_order + 1 + hc->total_frames * 256 >= input->no_frames) {
                break;
            }
        }
    }

#if TDRDO
    if (input->TDEnable != 0) {
        DestroyDistortionList(OMCPDList);
        DestroyDistortionList(RealDList);
        if (KappaTable) {
            free(KappaTable);
        }
        if (KappaTable1) {
            free(KappaTable1);
        }
        if (porgF) {
            free(porgF);
        }
        if (ppreF) {
            free(ppreF);
        }
        if (precF) {
            free(precF);
        }
        if (prefF) {
            free(prefF);
        }
    }
#endif

    img->EncodeEnd_flag = 1;
#if REF_OUTPUT
    if (input->output_enc_pic) {

        for (j = 0; j < REF_MAXBUFFER; j++) {

            int tmp_min, pos = -1;
            tmp_min = 1 << 20;
            for (i = 0; i < REF_MAXBUFFER; i++) {
                if (fref[i]->imgtr_fwRefDistance < tmp_min && fref[i]->imgtr_fwRefDistance > he->last_output) {
                    tmp_min = fref[i]->imgtr_fwRefDistance;
                    pos = i;
                }
            }
            if (pos != -1) {
                he->last_output = tmp_min;
                write_frame(he->p_dec, pos);
            }
        }
    }
#endif

    stat->bit_use_stuffingBits[3] += terminate_sequence(); // YiminZHOU revised begin
    switch (img->type) {
    case INTRA_IMG:
        stat->bit_ctr_0 += 32;
        break;
    case B_IMG:
        stat->bit_ctr_B += 32;
        break;
    default:
        stat->bit_ctr_P += 32;
        break;
    }
    printf("Termination       32\n");                      // YiminZHOU revised end

#if MB_DQP
    if (input->useDQP) {
        printf("The total number of zero DQPs is: %d \n", he->zeroDqpCount);
        printf("The total number of DQPs is: %d \n", he->DqpCount);
        printf("The ratio of zero DQPs over all DQPs is: %f \n", (float)he->zeroDqpCount / (float)he->DqpCount);
    }
#endif


    fclose(he->p_in);

    if (he->p_dec && input->output_enc_pic) {
        fclose(he->p_dec);
    }

    if (he->p_dec_background && input->output_enc_pic) {
        fclose(he->p_dec_background);
    }

#if INTERLACE_CODING
    snr->i_snr_ya = snr->i_snr_ua = snr->i_snr_va = 0.0;
    if (input->InterlaceCodingOption == 3 && input->output_enc_pic) {
        mergeFld2Frm();
    }
#endif

#if ROI_M3264
    ClosePosFile();
#endif

    if (hc->p_trace) {
        fclose(hc->p_trace);
    }
    if (input->bg_enable) {
        if (he->p_org_background) {
            fclose(he->p_org_background);
        }

        if (he->train_start) {
            bg_releaseModel();
        }
    }

    free_mem2Dint(AVS_SCAN4x16);
    free_mem2Dint(AVS_SCAN16x4);
    free_mem2Dint(AVS_SCAN2x8);
    free_mem2Dint(AVS_SCAN8x2);
    free_mem2Dint(AVS_SCAN32x8);
    free_mem2Dint(AVS_SCAN8x32);
    free_mem2Dint(AVS_SCAN8x8);
    free_mem2Dint(AVS_SCAN16x16);
    free_mem2Dint(AVS_SCAN32x32);
    free_mem2Dint(AVS_SCAN4x4);
    free_mem2Dint(AVS_CG_SCAN8x8);
    free_mem2Dint(AVS_CG_SCAN16x16);
    free_mem2Dint(AVS_CG_SCAN32x32);
    Clear_Motion_Search_Module();

    // free structure for rd-opt. mode decision
    clear_rdopt();
    clear_sao_entropy();

    // report everything
    report();


    if (input->MD5Enable & 0x01) {
//    clock_t start;
        long long filelength;
        unsigned int MD5val[4];
//    start = clock();
        filelength = FileMD5(input->outfile, MD5val);
        printf("\n================================================================\n");
        printf("Output Stream Size   : %lld bit\n", filelength << 3);
        printf("Output Stream MD5    : %08X%08X%08X%08X\n", MD5val[0], MD5val[1], MD5val[2], MD5val[3]);
//    printf("MD5 Calculation Time : %.3f sec\n", 1.0*(clock()-start)/CLOCKS_PER_SEC);
        printf("----------------------------------------------------------------\n");
//    start = clock();

        filelength = FileMD5(input->ReconFile, MD5val);

        printf("Reconstruct YUV Size : %lld byte\n", filelength);
        printf("Reconstruct YUV MD5  : %08X%08X%08X%08X\n", MD5val[0], MD5val[1], MD5val[2], MD5val[3]);
//    printf("MD5 Calculation Time : %.3f sec\n", 1.0*(clock()-start)/CLOCKS_PER_SEC);
        printf("================================================================\n");
    }


    free_picture(frame_pic);

    free_global_buffers();


    // free image mem
    free_img();
    if (input->alf_enable) {
        destroyAlfGlobalBuffers(img, g_MaxSizeInbit);
    }
    return 0;
}
void InitZigzagScan(int **scan, unsigned int uiBitSize)
{
    int i, j, pos;
    scan[0][0] = 0;
    scan[0][1] = 0;
    pos = 0;

    for (i = 1; i < ((1 << (uiBitSize + 1)) - 1); i++) {
        if (i < (1 << uiBitSize)) {
            if (i % 2 == 1) {
                pos++;
                scan[pos][0] = scan[pos - 1][0] + 1;
                scan[pos][1] = scan[pos - 1][1];

                for (j = 1; j < min(i + 1, ((1 << (uiBitSize + 1)) - i - 1)); j++) {
                    pos++;
                    scan[pos][0] = scan[pos - 1][0] - 1;
                    scan[pos][1] = scan[pos - 1][1] + 1;
                }
            } else {
                pos++;
                scan[pos][0] = scan[pos - 1][0];
                scan[pos][1] = scan[pos - 1][1] + 1;

                for (j = 1; j < min(i + 1, ((1 << (uiBitSize + 1)) - i - 1)); j++) {
                    pos++;
                    scan[pos][0] = scan[pos - 1][0] + 1;
                    scan[pos][1] = scan[pos - 1][1] - 1;
                }
            }
        } else {
            if (i % 2 == 1) {
                pos++;
                scan[pos][0] = scan[pos - 1][0];
                scan[pos][1] = scan[pos - 1][1] + 1;

                for (j = 1; j < min(i + 1, ((1 << (uiBitSize + 1)) - i - 1)); j++) {
                    pos++;
                    scan[pos][0] = scan[pos - 1][0] - 1;
                    scan[pos][1] = scan[pos - 1][1] + 1;
                }
            } else {
                pos++;
                scan[pos][0] = scan[pos - 1][0] + 1;
                scan[pos][1] = scan[pos - 1][1];

                for (j = 1; j < min(i + 1, ((1 << (uiBitSize + 1)) - i - 1)); j++) {
                    pos++;
                    scan[pos][0] = scan[pos - 1][0] + 1;
                    scan[pos][1] = scan[pos - 1][1] - 1;
                }
            }
        }
    }
}

void InitZigzagScan_NSQT(int **scan, int iWidth, int iHeight)
{
    int x, y, c = 0;

    // starting point
    scan[c][0] = 0;
    scan[c][1] = 0;
    c++;

    // loop
    x = 1;
    y = 0;
    while (1) {
        // decrease loop
        while (x >= 0) {
            if (x >= 0 && x < iWidth && y >= 0 && y < iHeight) {
                scan[c][0] = x;
                scan[c][1] = y;
                c++;
            }
            x--;
            y++;
        }
        x = 0;

        // increase loop
        while (y >= 0) {
            if (x >= 0 && x < iWidth && y >= 0 && y < iHeight) {
                scan[c][0] = x;
                scan[c][1] = y;
                c++;
            }
            x++;
            y--;
        }
        y = 0;

        // termination condition
        if (c >= iWidth * iHeight) {
            break;
        }
    }
}
void  InitCGZigzagScan(int **scan, int iWidth, int iHeight)
{

    int *** AVS_SCAN_4x4;
    int *** AVS_SCAN_CG;
    int i, j, k;

    get_mem3Dint(&AVS_SCAN_4x4, 2, 16, 2);
    get_mem3Dint(&AVS_SCAN_CG, 2, iWidth * iHeight / 16, 2);
    InitZigzagScan(AVS_SCAN_4x4[1],  2);
    InitZigzagScan_NSQT(AVS_SCAN_CG[1], iWidth / 4, iHeight / 4);

    scan[0][0] = 0;
    scan[0][1] = 0;

    for (i = 1; i < iWidth * iHeight ; i++) {
        j = i / 16;
        k = i % 16;

        scan[i][0] = AVS_SCAN_CG[1][j][0] * 4 + AVS_SCAN_4x4[1][k][0];
        scan[i][1] = AVS_SCAN_CG[1][j][1] * 4 + AVS_SCAN_4x4[1][k][1];
    }
    free_mem3Dint(AVS_SCAN_4x4, 2);
    free_mem3Dint(AVS_SCAN_CG, 2);
}
void init_frame_buffers(InputParameters *inp, ImageParameters *img)
{
    int i;

    for (i = 0; i < REF_MAXBUFFER; i++) {
        fref[i] = (avs2_frame_t *)malloc(sizeof(avs2_frame_t));  //fref[i] memory allocation
        init_frame_t(fref[i]);
    }
    if (input->bg_enable) {
        get_mem2D(&he->background_frame_quarter, (img->height + 2 * IMG_PAD_SIZE) * 4, (img->width + 2 * IMG_PAD_SIZE) * 4);
    }

}


/*
*************************************************************************
* Function:Initializes the Image structure with appropriate parameters.
* Input:Input Parameters struct inp_par *inp
* Output:Image Parameters struct img_par *img
* Return:
* Attention:
*************************************************************************
*/

void init_img()
{
    int i, j;
    double FrameRate[8] = {24000.0 / 1001.0, 24.0, 25.0, 30000.0 / 1001.0, 30.0, 50.0, 60000.0 / 1001.0, 60.0};

    img->buf_cycle = input->no_multpred;

    img->width    = (input->img_width + img->auto_crop_right);   //add by wuzhongmou 0610
    img->height   = (input->img_height + img->auto_crop_bottom);   //add by wuzhongmou 0610
    img->width_cr = img->width / 2;
    img->height_cr = img->height / (input->chroma_format == 1 ? 2 : 1);
    //!EDIT START <added by lzhang AEC
    img->PicWidthInMbs  = img->width / MIN_CU_SIZE;
    //!EDIT end <added by lzhang AEC
    img->framerate = (int) FrameRate[input->frame_rate_code - 1];


    get_mem_mv(& (img->mv));      //img->mv[2:b8_x][2:b8_y][buf_cycle:max_ref][9:blocktype][2:x,y]
    get_mem_mv(& (img->predBFwMv));      //MVDFW
    get_mem_mv(& (img->predBBwMv));      //MVDBW

    get_mem_mv(& (img->predBidFwMv));      //MVDFW
    get_mem_mv(& (img->predBidBwMv));      //MVDBW
    get_mem_mv(& (img->forwardpred_mv));
    get_mem_mv(& (img->forwardpred_allFwMv));
    get_mem_mv(& (img->allFwMv));
    get_mem_mv(& (img->allBwMv));
    get_mem_mv(& (img->predSymMv));
    get_mem_mv(& (img->allSymMv));

    get_mem_mv(& (img->allBidFwMv));
    get_mem_mv(& (img->allBidBwMv));

    get_mem_mv(& (img->predDualFstMv));
    get_mem_mv(& (img->predDualSndMv));

    get_mem_mv(& (img->allDualFstMv));
    get_mem_mv(& (img->allDualSndMv));

    get_mem_mv(& (img->allIntegerBFwMv));
    get_mem_mv(& (img->allIntegerBBwMv));
    get_mem_mv(& (img->allIntegerPFwMv));

    get_mem2Dint(& (img->Coeff_all), 2 * (1 << input->g_uiMaxSizeInBit),
                 (1 << input->g_uiMaxSizeInBit));    //qyu 0821
    get_mem2Dint(& (img->Coeff_all_to_write), 2 * (1 << input->g_uiMaxSizeInBit),
                 (1 << input->g_uiMaxSizeInBit));    //qyu 0823
    get_mem2Dint(& (img->currRef), (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));      //qyu 0823
    get_mem2Dint(& (img->currPSndRef), (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));
    get_mem2Dint(& (img->currFwRef), (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));      //qyu 0823
    get_mem2Dint(& (img->currBwRef), (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));      //qyu 0823
    get_mem2D(& (img->recon_currY), (1 << input->g_uiMaxSizeInBit), (1 << input->g_uiMaxSizeInBit));
    get_mem2D(& (img->recon_currU), (1 << input->g_uiMaxSizeInBit), (1 << input->g_uiMaxSizeInBit));
    get_mem2D(& (img->recon_currV), (1 << input->g_uiMaxSizeInBit), (1 << input->g_uiMaxSizeInBit));
    get_mem2Dint(& (img->ipredmode_curr), (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));      //qyu 0823  4:2 5:4 6:8
    get_mem3Dint(&img->currMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)), 3);
    get_mem3Dint(&img->currPSndMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)), 3);
    get_mem3Dint(&img->currFwMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)), 3);
    get_mem3Dint(&img->currBwMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)),
                 (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)), 3);
    get_mem3Dint(& (img->predBlockY), NUM_INTRA_PMODE, (1 << input->g_uiMaxSizeInBit),
                 (1 << input->g_uiMaxSizeInBit));
    get_mem2Dint(& (img->predBlock), (1 << input->g_uiMaxSizeInBit), (1 << input->g_uiMaxSizeInBit));
    get_mem2Dint(&img->resiY, (1 << (input->g_uiMaxSizeInBit + 1)), (1 << (input->g_uiMaxSizeInBit + 1)));
    get_mem4Dint(&img->predBlockUV, 2, NUM_INTRA_PMODE_CHROMA, (1 << (input->g_uiMaxSizeInBit - 1)),
                 (1 << input->g_uiMaxSizeInBit));

    //if ( ( img->MB_SyntaxElements = ( SyntaxElement* ) calloc ( ( ( img->height * img->width / 256 ) + 1200 ) * 64, sizeof ( SyntaxElement ) ) ) == NULL ) //qyu 0830
    if ((img->MB_SyntaxElements = (SyntaxElement *) calloc(((img->height * img->width / 64) + 1200) * 64,
                                  sizeof(SyntaxElement))) == NULL) {     //qyu 0830
        no_mem_exit("init_img: MB_SyntaxElements");
    }
    if ((img->quad = (int *) calloc(((1 << (input->sample_bit_depth + 1)) - 1), sizeof(int))) == NULL) {
        no_mem_exit("init_img: img->quad");
    }

    img->quad += ((1 << input->sample_bit_depth) - 1);
    for (i = 0; i < (1 << input->sample_bit_depth); ++i) {
        img->quad[i] = img->quad[-i] = i * i;
    }
    if (((img->mb_data) = (codingUnit *) calloc((img->width / MIN_CU_SIZE) * (img->height / MIN_CU_SIZE),
                          sizeof(codingUnit))) == NULL) {
        no_mem_exit("init_img: img->mb_data");
    }

    for (i = 0; i < (img->width / MIN_CU_SIZE) * (img->height / MIN_CU_SIZE); i++) {
        img->mb_data[i].slice_nr = -1;//qyu 0907
    }

    // allocate memory for intra pred mode buffer for each block: img->ipredmode
    get_mem2Dint(& (img->ipredmode), img->height / MIN_BLOCK_SIZE + 100, img->width / MIN_BLOCK_SIZE + 100);
    get_mem2Dint(& (img->rec_ipredmode), img->height / MIN_BLOCK_SIZE + 100, img->width / MIN_BLOCK_SIZE + 100);

    // Prediction mode is set to -1 outside the frame, indicating that no prediction can be made from this part
    for (j = 0; j < img->height / (MIN_BLOCK_SIZE) + 100; j++) {
        for (i = 0; i < img->width / (MIN_BLOCK_SIZE) + 100; i++) {
            img->ipredmode[j][i] = -1;
        }
    }

    for (j = 0; j < (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)); j++) {
        for (i = 0; i < (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)); i++) {
            img->ipredmode_curr[j][i] = -1;
        }
    }

    get_mem2Dint(&tmp, MAX_CU_SIZE, MAX_CU_SIZE);
    get_mem2Dint(&hc->tmp_block_88_inv, MAX_CU_SIZE, MAX_CU_SIZE);

    get_mem3DSAOstatdate(&(img->saostatData),
                         ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                     img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS,
                         NUM_SAO_NEW_TYPES);

    get_mem2DSAOParameter(&(img->saoBlkParams),
                          ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                      img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS);

    get_mem2DSAOParameter(&img->rec_saoBlkParams,
                          ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                      img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS);

    if ((img->cur_saorate = (double *) calloc(NUM_SAO_COMPONENTS, sizeof(double))) == NULL) {
        no_mem_exit("init_img: img->cur_saorate");
    }

    get_mem3Dint(&(img->Coeff_all_to_write_ALF),
                 ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                             img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)),
                 2 * (1 << input->g_uiMaxSizeInBit), (1 << input->g_uiMaxSizeInBit));

    if ((img->dp_ALF = (AlfDataPart *) calloc(1, sizeof(AlfDataPart))) == NULL) {
        no_mem_exit("init_img: dp_ALF");
    }
    img->dp_ALF->syn_ctx = create_contexts_SyntaxInfo();
}

/*
*************************************************************************
* Function:Free the Image structures
* Input:Image Parameters struct img_par *img
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_img()
{
    free_mem_mv(img->mv);
    free_mem_mv(img->predBFwMv);
    free_mem_mv(img->predBBwMv);

    free_mem_mv(img->predBidFwMv);
    free_mem_mv(img->predBidBwMv);
    free_mem_mv(img->forwardpred_mv);
    free_mem_mv(img->forwardpred_allFwMv);
    free_mem_mv(img->allFwMv);
    free_mem_mv(img->allBwMv);
    free_mem_mv(img->predSymMv);
    free_mem_mv(img->allSymMv);

    free_mem_mv(img->allBidFwMv);
    free_mem_mv(img->allBidBwMv);

    free_mem_mv(img->predDualFstMv);
    free_mem_mv(img->predDualSndMv);

    free_mem_mv(img->allDualFstMv);
    free_mem_mv(img->allDualSndMv);

    free_mem_mv(img->allIntegerBFwMv);
    free_mem_mv(img->allIntegerBBwMv);
    free_mem_mv(img->allIntegerPFwMv);
    free_mem2Dint(img->Coeff_all);
    free_mem2Dint(img->Coeff_all_to_write);    //qyu 0823
    free_mem2Dint(img->currRef);    //qyu 0823
    free_mem2Dint(img->currPSndRef);    //qyu 0823
    free_mem2Dint(img->currFwRef);    //qyu 0823
    free_mem2Dint(img->currBwRef);    //qyu 0823
    free_mem3Dint(img->currMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));        //qyu 0906
    free_mem3Dint(img->currPSndMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));        //qyu 0906
    free_mem3Dint(img->currFwMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));
    free_mem3Dint(img->currBwMv, (1 << (input->g_uiMaxSizeInBit - MIN_BLOCK_SIZE_IN_BIT)));
    free_mem2D(img->recon_currY);
    free_mem2D(img->recon_currU);
    free_mem2D(img->recon_currV);
    free_mem2Dint(img->ipredmode_curr);    //qyu 0823  4:2 5:4 6:8
    free_mem3Dint(img->predBlockY, NUM_INTRA_PMODE);
    free_mem2Dint(img->predBlock);
    free_mem2Dint(img->resiY);
    free_mem4Dint(img->predBlockUV, 2, NUM_INTRA_PMODE_CHROMA);
    free(img->quad - ((1 << input->sample_bit_depth) - 1));
    free(sliceregion);

    free(img->MB_SyntaxElements);
    free_mem2Dint(tmp);
    free_mem2Dint(hc->tmp_block_88_inv);
    free_mem3DSAOstatdate(img->saostatData,
                          ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                      img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS);

    free_mem2DSAOParameter(img->saoBlkParams,
                           ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                       img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)));

    free_mem2DSAOParameter(img->rec_saoBlkParams,
                           ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                       img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)));

    free(img->cur_saorate);


    free_mem3Dint(img->Coeff_all_to_write_ALF,
                  ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                              img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)));

    delete_contexts_SyntaxInfo(img->dp_ALF->syn_ctx);
    free(img->dp_ALF);
}
/*
*************************************************************************
* Function:Allocates the picture structure along with its dependent
data structures
* Input:
* Output:
* Return: Pointer to a Picture
* Attention:
*************************************************************************
*/

Picture *malloc_picture()
{
    Picture *pic;

    if ((pic = calloc(1, sizeof(Picture))) == NULL) {
        no_mem_exit("malloc_picture: Picture structure");
    }

    return pic;
}

/*
*************************************************************************
* Function:Frees a picture
* Input:pic: POinter to a Picture to be freed
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_picture(Picture *pic)
{
    if (pic != NULL) {
        free(pic);
    }

}
/*
*************************************************************************
* Function:Reports the gathered information to appropriate outputs
* Input:  struct inp_par *inp,                                            \n
struct img_par *img,                                            \n
struct stat_par *stat,                                          \n
struct stat_par *stat
* Output:
* Return:
* Attention:
*************************************************************************
*/

void report()
{
    long long int bit_use[2][2] ;
    long long int bit_use_Bframe = 0;
    long long int total_bits;
    int i, j;
    char name[20];
    double frame_rate;
    double bit_rate;
    double mean_motion_info_bit_use[2];

    int no_IPframes = (input->no_frames + input->successive_Bframe - 1) / (input->successive_Bframe + 1) + 1;
#ifndef WIN32
    time_t now;
    struct tm *l_time;
    char string[1000];
#else
    char timebuf[128];
#endif

    bit_use[0][0] = 1;
    bit_use[1][0] = max(1, no_IPframes - 1);

    //  Accumulate bit usage for inter and intra frames
    bit_use[0][1] = bit_use[1][1] = 0;

    for (i = 0; i < 11; i++) {
        bit_use[1][1] += stat->bit_use_mode_inter[0][i];
    }

    for (j = 0; j < 2; j++) {
        bit_use[j][1] += stat->bit_use_header[j];
        bit_use[j][1] += stat->bit_use_cuType[j];
        bit_use[j][1] += stat->tmp_bit_use_cbp[j];
        bit_use[j][1] += stat->bit_use_coeffY[j];
        bit_use[j][1] += stat->bit_use_coeffC[j];
        bit_use[j][1] += stat->bit_use_delta_quant[j];
        bit_use[j][1] += stat->bit_use_stuffingBits[j];
    }

    // B pictures
    if (hc->Bframe_ctr != 0) {
        bit_use_Bframe = 0;

        for (i = 0; i < 11; i++) {
            bit_use_Bframe += stat->bit_use_mode_inter[1][i];
        }

        bit_use_Bframe += stat->bit_use_header[2];
        bit_use_Bframe += stat->bit_use_cuType[2];
        bit_use_Bframe += stat->tmp_bit_use_cbp[2];
        bit_use_Bframe += stat->bit_use_coeffY[2];
        bit_use_Bframe += stat->bit_use_coeffC[2];
        bit_use_Bframe += stat->bit_use_delta_quant[2];
        bit_use_Bframe += stat->bit_use_stuffingBits[2];

        stat->bitrate_P = (stat->bit_ctr_0 + stat->bit_ctr_P) * (double)(img->framerate /
                          (input->jumpd + 1)) / no_IPframes;
        stat->bitrate_B = (stat->bit_ctr_B) * (double)(img->framerate / (input->jumpd + 1)) *
                          input->successive_Bframe / hc->Bframe_ctr;
    }

    fprintf(stdout, "-----------------------------------------------------------------------------\n");
    fprintf(stdout,   " Freq. for encoded bitstream       : %1.0f\n",
            (double)(img->framerate * (input->successive_Bframe + 1)) / (double)(input->jumpd + 1));

    if (input->hadamard) {
        fprintf(stdout, " Hadamard transform                : Used\n");
    } else {
        fprintf(stdout, " Hadamard transform                : Not used\n");
    }


    fprintf(stdout, " Image (Encoding) format           : %dx%d\n", img->width, img->height);
    fprintf(stdout, " Image (Recon) format              : %dx%d\n", (img->width - img->auto_crop_right),
            (img->height - img->auto_crop_bottom));
#if INTERLACE_CODING
    if (input->InterlaceCodingOption == 3) {
        fprintf(stdout, " Image format for original image   : %dx%d\n", input->org_img_width, input->org_img_height);
    }
#endif

    fprintf(stdout,    " Fast Motion Estimation            : %s\n", input->usefme ? "On" : "Off");       //rm52k_r1
    fprintf(stdout,    " Search range                      : %d\n", input->search_range);


    fprintf(stdout,   " Num of ref. frames used in P pred : %d\n", input->no_multpred);

    if (input->successive_Bframe != 0) {
        fprintf(stdout,   " Num of ref. frames used in B pred : %d\n", /*input->no_multpred*/2);   //1105
    }

    fprintf(stdout,   " Total encoding time for the seq.  : %.3f sec \n", hc->tot_time * 0.001);

    // B pictures
    //fprintf ( stdout, " Sequence type                     :" );

    if (input->successive_Bframe == 1) {
        fprintf(stdout, " IBPBP (QP: I %d, P %d, B %d) \n", input->qpI, input->qpP, input->qpB);
    } else if (input->successive_Bframe == 2) {
        fprintf(stdout, " IBBPBBP (QP: I %d, P %d, B %d) \n", input->qpI, input->qpP, input->qpB);
    } else if (input->successive_Bframe == 0 && input->intra_period != 1) {
        fprintf(stdout, " IPPP (QP: I %d, P %d) \n", input->qpI, input->qpP);
    } else if (input->successive_Bframe == 0 && input->intra_period == 1) {
        fprintf(stdout, " IPPP (QP: I %d) \n", input->qpI);
    }

    // report on entropy coding  method
    fprintf(stdout, " Entropy coding method             : AEC\n");

    if (input->rdopt) {
        fprintf(stdout, " RD-optimized mode decision        : used\n");
    } else {
        fprintf(stdout, " RD-optimized mode decision        : not used\n");
    }


    fprintf(stdout, "------------------ Average data all frames  ---------------------------------\n");
    fprintf(stdout, " SNR Y(dB)                         : %5.2f\n", snr->snr_ya);
    fprintf(stdout, " SNR U(dB)                         : %5.2f\n", snr->snr_ua);
    fprintf(stdout, " SNR V(dB)                         : %5.2f\n", snr->snr_va);

#if INTERLACE_CODING
    if (input->InterlaceCodingOption == 3 && input->output_enc_pic) {
        fprintf(stdout, "------------ Average data all frames for interlace  -------------------------\n");
        fprintf(stdout, " Merged SNR Y(dB)                  : %5.2f\n", snr->i_snr_ya);
        fprintf(stdout, " Merged SNR U(dB)                  : %5.2f\n", snr->i_snr_ua);
        fprintf(stdout, " Merged SNR V(dB)                  : %5.2f\n", snr->i_snr_va);
        fprintf(stdout, "-----------------------------------------------------------------------------\n");
    }
#endif

    if (hc->Bframe_ctr != 0) {
        fprintf(stdout, " Total bits                        : %lld (I %5lld, P %5lld, B %lld) \n",
                total_bits = stat->bit_ctr_P + stat->bit_ctr_0 + stat->bit_ctr_B, stat->bit_ctr_0, stat->bit_ctr_P, stat->bit_ctr_B);
        frame_rate = (double)(img->framerate * (input->successive_Bframe + 1)) / (double)(input->jumpd + 1);
        stat->bitrate = ((double) total_bits * frame_rate) / ((double)(input->no_frames));
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3)
        { bit_rate =  2 * stat->bitrate / 1000; }
        else
#endif
            bit_rate = stat->bitrate / 1000;
    } else {
        fprintf(stdout, " Total bits                        : %lld (I %5lld, P %5lld) \n",
                total_bits = stat->bit_ctr_P + stat->bit_ctr_0 , stat->bit_ctr_0, stat->bit_ctr_P);
        frame_rate = (double) img->framerate / ((double)(input->jumpd + 1));
        stat->bitrate = ((double)total_bits * frame_rate) / ((double)input->no_frames);
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3)
        { bit_rate = 2 * stat->bitrate / 1000; }
        else
#endif
            bit_rate = stat->bitrate / 1000;

    }
    fprintf(stdout, " Bit rate (kbit/s)  @ %5.2f Hz     : %5.2f\n", frame_rate, bit_rate);

    if (input->bg_enable) {
        fprintf(stdout, " Duplicated GB bits                : %lld \n", stat->bit_ctr_dup_gb);
        {
            double bitrate_without_dup_gb = (((double)total_bits - stat->bit_ctr_dup_gb) * frame_rate) / ((double)(
                                                input->no_frames));
            fprintf(stdout, " Bit rate (no-dup-GB) @ %5.2f Hz   : %5.2f\n", frame_rate, bitrate_without_dup_gb / 1000);
        }
    }
    {
        FILE *fp_psnr = fopen("psnr.txt", "a");
        fprintf(fp_psnr, "%.2f\t%.2f\t%.2f\t%.2f\n", stat->bitrate / 1000, snr->snr_ya, snr->snr_ua, snr->snr_va);
        fclose(fp_psnr);
    }

    fprintf(stdout, " Bits to avoid Startcode Emulation : %d \n", stat->bit_ctr_emulationprevention);

    // Adaptive frequency weighting quantization
#if ( FREQUENCY_WEIGHTING_QUANTIZATION && COUNT_BIT_OVERHEAD)
    fprintf(stdout, "\n Total bits to store QMs : %8d\n", he->g_count_overhead_bit);
#endif


    fprintf(stdout, "-----------------------------------------------------------------------------\n");

    fprintf(stdout, "Exit RD %s encoder ver %s ", RD, VERSION);
    fprintf(stdout, "\n");

#ifdef REPORT
    {
        char *ch;
        FILE *logtable = fopen("EncRD.log", "a+");
        for (i = strlen(input->infile); i >= 0 && input->infile[i] != '\\'; ch = &input->infile[i--]);
        fprintf(logtable, "%dx%d\t", img->width, img->height);
        fprintf(logtable, "%s\t", ch);
        fprintf(logtable, "%d\t", input->qpI);
        fprintf(logtable, "%.2f\t", stat->bitrate / 1000);
        fprintf(logtable, "%.2f\t", snr->snr_ya);
        fprintf(logtable, "%.2f\t", snr->snr_ua);
        fprintf(logtable, "%.2f\t", snr->snr_va);
        fprintf(logtable, "%.3f\t", hc->tot_time * 0.001);
//    fprintf ( logtable, "%.3f", (pRC->HighestBufferSizeBpp-pRC->LowestBufferSizeBpp)/pRC->TargetBitPerPixel );
        fprintf(logtable, "\n");
        fclose(logtable);
    }
#endif

    // status file
    if ((he->p_stat = fopen("stat.dat", "at")) == 0) {
        snprintf(hc->errortext, ET_SIZE, "Error open file %s", "stat.dat");
        error(hc->errortext, 500);
    }

    fprintf(he->p_stat, "\n ------------------ Average data all frames  ------------------------------\n");
    fprintf(he->p_stat, " SNR Y(dB)                         : %5.2f\n", snr->snr_ya);
    fprintf(he->p_stat, " SNR U(dB)                         : %5.2f\n", snr->snr_ua);
    fprintf(he->p_stat, " SNR V(dB)                         : %5.2f\n", snr->snr_va);
    fprintf(he->p_stat, " Total bits                        : %lld (I %5lld, P BS %5lld, B %lld) \n",
            total_bits = stat->bit_ctr_P + stat->bit_ctr_0 + stat->bit_ctr_B, stat->bit_ctr_0, stat->bit_ctr_P, stat->bit_ctr_B);
#if INTERLACE_CODING
    if (input->InterlaceCodingOption == 3)
    { bit_rate = 2 * stat->bitrate / 1000; }
    else
#endif
        bit_rate = stat->bitrate / 1000;

    fprintf(he->p_stat, " Bit rate (kbit/s)  @ %5.2f Hz     : %5.2f\n", frame_rate, bit_rate);

    // Adaptive frequency weighting quantization
#if ( FREQUENCY_WEIGHTING_QUANTIZATION && COUNT_BIT_OVERHEAD)
    fprintf(he->p_stat, "\n Total bits to store QMs : %8d\n", he->g_count_overhead_bit);
#endif

    fprintf(he->p_stat, " -------------------------------------------------------------- \n");
    fprintf(he->p_stat, "  This file contains statistics for the last encoded sequence   \n");
    fprintf(he->p_stat, " -------------------------------------------------------------- \n");
    fprintf(he->p_stat, " Sequence                     : %s\n", input->infile);
    fprintf(he->p_stat, " No.of coded pictures         : %4d\n", input->no_frames);
    fprintf(he->p_stat, " Freq. for encoded bitstream  : %4.0f\n", frame_rate);

    // B pictures
    if (input->successive_Bframe != 0) {
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3) {
            fprintf(he->p_stat, " IPFrame Bitrate(kb/s)      : %6.2f\n", 2 * stat->bitrate_P / 1000);    //rm52k
            fprintf(he->p_stat, " BFrame Bitrate(kb/s)  : %6.2f\n", 2 * stat->bitrate_B / 1000);     //rm52k
        } else {
            fprintf(he->p_stat, " IPFrame Bitrate(kb/s)      : %6.2f\n", stat->bitrate_P / 1000);    //rm52k
            fprintf(he->p_stat, " BFrame Bitrate(kb/s)  : %6.2f\n", stat->bitrate_B / 1000);     //rm52k
        }
#else
        fprintf(he->p_stat,   " IPFrame Bitrate(kb/s)      : %6.2f\n", stat->bitrate_P / 1000);    //rm52k
        fprintf(he->p_stat,   " BFrame Bitrate(kb/s)  : %6.2f\n", stat->bitrate_B / 1000);     //rm52k
#endif
    } else {
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3) {
            fprintf(he->p_stat, " Bitrate(kb/s)                : %6.2f\n", 2 * stat->bitrate / 1000);
        } else
#endif
            fprintf(he->p_stat, " Bitrate(kb/s)                : %6.2f\n", stat->bitrate / 1000);
    }

    if (input->hadamard) {
        fprintf(he->p_stat, " Hadamard transform           : Used\n");
    } else {
        fprintf(he->p_stat, " Hadamard transform           : Not used\n");
    }

    fprintf(he->p_stat, " Image format                 : %dx%d\n", img->width, img->height);    //add by wuzhongmou 0610

    fprintf(he->p_stat, " Search range                 : %d\n", input->search_range);


    fprintf(he->p_stat, " No of frame used in P pred   : %d\n", input->no_multpred);

    if (input->successive_Bframe != 0) {
        fprintf(he->p_stat, " No of frame used in B pred   : %d\n", input->no_multpred);
    }

    fprintf(he->p_stat, " Entropy coding method        : AEC\n");

    fprintf(he->p_stat, " Search range restrictions    : none\n");

    if (input->rdopt) {
        fprintf(he->p_stat, " RD-optimized mode decision   : used\n");
    } else {
        fprintf(he->p_stat, " RD-optimized mode decision   : not used\n");
    }

    fprintf(he->p_stat, " -------------------|---------------|---------------|\n");
    fprintf(he->p_stat, "     Item           |     Intra     |   All frames  |\n");
    fprintf(he->p_stat, " -------------------|---------------|---------------|\n");
    fprintf(he->p_stat, " SNR Y(dB)          |");
    fprintf(he->p_stat, " %5.2f         |", snr->snr_y1);
    fprintf(he->p_stat, " %5.2f         |\n", snr->snr_ya);
    fprintf(he->p_stat, " SNR U/V (dB)       |");
    fprintf(he->p_stat, " %5.2f/%5.2f   |", snr->snr_u1, snr->snr_v1);
    fprintf(he->p_stat, " %5.2f/%5.2f   |\n", snr->snr_ua, snr->snr_va);

    // QUANT.
    fprintf(he->p_stat, " Average quant      |");
    fprintf(he->p_stat, " %5d         |", absm(input->qpI));
    fprintf(he->p_stat, " %5.2f         |\n", (double)stat->quant1 / max(1.0, (double)stat->quant0));

    // MODE
    fprintf(he->p_stat, "\n -------------------|---------------|\n");
    fprintf(he->p_stat, "   Intra            |   Mode used   |\n");
    fprintf(he->p_stat, " -------------------|---------------|\n");
    fprintf(he->p_stat, " Mode 0  intra      | %5d         |\n", stat->mode_use_intra[I16MB]);
    fprintf(he->p_stat, "\n -------------------|---------------|-----------------|\n");
    fprintf(he->p_stat, "   Inter            |   Mode used   | MotionInfo bits |\n");
    fprintf(he->p_stat, " -------------------|---------------|-----------------|");
    fprintf(he->p_stat, "\n Mode  0  (skip)    | %5d         |    %8.2f     |", stat->mode_use_inter[0][0],
            (double) stat->bit_use_mode_inter[0][0   ] / (double) bit_use[1][0]);
    fprintf(he->p_stat, "\n Mode  1  (16x16)   | %5d         |    %8.2f     |", stat->mode_use_inter[0][1],
            (double) stat->bit_use_mode_inter[0][1   ] / (double) bit_use[1][0]);
    fprintf(he->p_stat, "\n Mode  2  (16x8)    | %5d         |    %8.2f     |", stat->mode_use_inter[0][2],
            (double) stat->bit_use_mode_inter[0][2   ] / (double) bit_use[1][0]);
    fprintf(he->p_stat, "\n Mode  3  (8x16)    | %5d         |    %8.2f     |", stat->mode_use_inter[0][3],
            (double) stat->bit_use_mode_inter[0][3   ] / (double) bit_use[1][0]);
    fprintf(he->p_stat, "\n Mode  4  (8x8)     | %5d         |    %8.2f     |", stat->mode_use_inter[0][PNXN],
            (double) stat->bit_use_mode_inter[0][PNXN] / (double) bit_use[1][0]);
    fprintf(he->p_stat, "\n Mode  5  intra     | %5d         |-----------------|", stat->mode_use_inter[0][I16MB]);
    mean_motion_info_bit_use[0] = (double)(stat->bit_use_mode_inter[0][0] + stat->bit_use_mode_inter[0][1] +
                                           stat->bit_use_mode_inter[0][2] + stat->bit_use_mode_inter[0][3] + stat->bit_use_mode_inter[0][PNXN]) /
                                  (double) bit_use[1][0];

    // B pictures
    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, "\n\n -------------------|---------------|-----------------|\n");
        fprintf(he->p_stat, "   B frame          |   Mode used   | MotionInfo bits |\n");
        fprintf(he->p_stat, " -------------------|---------------|-----------------|");
        fprintf(he->p_stat, "\n Mode  0  (skip)    | %5d         |    %8.2f     |", stat->mode_use_inter[1][0],
                (double) stat->bit_use_mode_inter[1][0   ] / (double) hc->Bframe_ctr);
        fprintf(he->p_stat, "\n Mode  1  (16x16)   | %5d         |    %8.2f     |", stat->mode_use_inter[1][1],
                (double) stat->bit_use_mode_inter[1][1   ] / (double) hc->Bframe_ctr);
        fprintf(he->p_stat, "\n Mode  2  (16x8)    | %5d         |    %8.2f     |", stat->mode_use_inter[1][2],
                (double) stat->bit_use_mode_inter[1][2   ] / (double) hc->Bframe_ctr);
        fprintf(he->p_stat, "\n Mode  3  (8x16)    | %5d         |    %8.2f     |", stat->mode_use_inter[1][3],
                (double) stat->bit_use_mode_inter[1][3   ] / (double) hc->Bframe_ctr);
        fprintf(he->p_stat, "\n Mode  4  (8x8)     | %5d         |    %8.2f     |", stat->mode_use_inter[1][PNXN],
                (double) stat->bit_use_mode_inter[1][PNXN] / (double) hc->Bframe_ctr);
        fprintf(he->p_stat, "\n Mode  5  intra     | %5d         |-----------------|", stat->mode_use_inter[1][I16MB]);
        mean_motion_info_bit_use[1] = (double)(stat->bit_use_mode_inter[1][0] + stat->bit_use_mode_inter[1][1] +
                                               stat->bit_use_mode_inter[1][2]
                                               + stat->bit_use_mode_inter[1][3] + stat->bit_use_mode_inter[1][PNXN]) / (double)hc->Bframe_ctr;
    }
    fprintf(he->p_stat, "\n\n --------------------|----------------|----------------|----------------|\n");
    fprintf(he->p_stat, "  Bit usage:         |      Intra     |      Inter     |    B frame     |\n");
    fprintf(he->p_stat, " --------------------|----------------|----------------|----------------|\n");
    fprintf(he->p_stat, " Seq heaer           |");
    fprintf(he->p_stat, " %10.2f              |\n", (double)hc->seq_header);

    fprintf(he->p_stat, " Seq End             |");
    fprintf(he->p_stat, " %10.2f              |\n", (double)stat->bit_use_stuffingBits[3]);

    fprintf(he->p_stat, " Slice Header        |");
    fprintf(he->p_stat, " %10.2f     |", (double)he->slice_header[0] / bit_use[0][0]);
    fprintf(he->p_stat, " %10.2f     |", (double)he->slice_header[1] / bit_use[1][0]);

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " %10.2f     |", (double)he->slice_header[2] / hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " %10.2f     |", 0.);
    }

    fprintf(he->p_stat, "\n");

    fprintf(he->p_stat, " Header              |");
    fprintf(he->p_stat, " %10.2f     |", (double)stat->bit_use_header[0] / bit_use[0][0]);
    fprintf(he->p_stat, " %10.2f     |", (double)stat->bit_use_header[1] / bit_use[1][0]);

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " %10.2f     |", (double)stat->bit_use_header[2] / hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " %10.2f     |", 0.);
    }

    fprintf(he->p_stat, "\n");
    fprintf(he->p_stat, " Mode                |");
    fprintf(he->p_stat, " %10.2f     |", (double)stat->bit_use_cuType[0] / bit_use[0][0]);
    fprintf(he->p_stat, " %10.2f     |", (double)stat->bit_use_cuType[1] / bit_use[1][0]);

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " %10.2f     |", (double)stat->bit_use_cuType[2] / hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " %10.2f     |", 0.);
    }

    fprintf(he->p_stat, "\n");
    fprintf(he->p_stat, " Motion Info         |");
    fprintf(he->p_stat, "        ./.     |");
    fprintf(he->p_stat, " %10.2f     |", mean_motion_info_bit_use[0]);

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " %10.2f     |", mean_motion_info_bit_use[1]);
    } else {
        fprintf(he->p_stat, " %10.2f     |", 0.);
    }

    fprintf(he->p_stat, "\n");
    fprintf(he->p_stat, " CBP Y/C             |");

    for (j = 0; j < 2; j++) {
        fprintf(he->p_stat, " %10.2f     |", (double)stat->tmp_bit_use_cbp[j] / bit_use[j][0]);
    }

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " %10.2f     |", (double)stat->tmp_bit_use_cbp[2] / hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " %10.2f     |", 0.);
    }

    fprintf(he->p_stat, "\n");

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " Coeffs. Y           | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_coeffY[0] / bit_use[0][0], (double) stat->bit_use_coeffY[1] / bit_use[1][0],
                (double)stat->bit_use_coeffY[2] / hc->Bframe_ctr);
    }

    else {
        fprintf(he->p_stat, " Coeffs. Y           | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_coeffY[0] / bit_use[0][0], (double) stat->bit_use_coeffY[1] / (double) bit_use[1][0], 0.);
    }

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " Coeffs. C           | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_coeffC[0] / bit_use[0][0], (double) stat->bit_use_coeffC[1] / bit_use[1][0],
                (double) stat->bit_use_coeffC[2] / hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " Coeffs. C           | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_coeffC[0] / bit_use[0][0], (double) stat->bit_use_coeffC[1] / bit_use[1][0], 0.);
    }

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " Delta quant         | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_delta_quant[0] / bit_use[0][0], (double) stat->bit_use_delta_quant[1] / bit_use[1][0],
                (double) stat->bit_use_delta_quant[2] / hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " Delta quant         | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_delta_quant[0] / bit_use[0][0], (double) stat->bit_use_delta_quant[1] / bit_use[1][0], 0.);
    }

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " Stuffing Bits       | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_stuffingBits[0] / bit_use[0][0], (double) stat->bit_use_stuffingBits[1] / bit_use[1][0],
                (double) stat->bit_use_stuffingBits[2] / hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " Stuffing Bits       | %10.2f     | %10.2f     | %10.2f     |\n",
                (double) stat->bit_use_stuffingBits[0] / bit_use[0][0], (double) stat->bit_use_stuffingBits[1] / bit_use[1][0],
                0.);
    }

    fprintf(he->p_stat, " --------------------|----------------|----------------|----------------|\n");
    fprintf(he->p_stat, " average bits/frame  |");

    for (i = 0; i < 2; i++) {
        fprintf(he->p_stat, " %10.2f     |", (double)bit_use[i][1] / (double)bit_use[i][0]);
    }

    if (input->successive_Bframe != 0 && hc->Bframe_ctr != 0) {
        fprintf(he->p_stat, " %10.2f     |", (double) bit_use_Bframe / (double) hc->Bframe_ctr);
    } else {
        fprintf(he->p_stat, " %10.2f     |", 0.);
    }

    fprintf(he->p_stat, "\n");
    fprintf(he->p_stat, " --------------------|----------------|----------------|----------------|\n");

    fclose(he->p_stat);

    // write to log file
    if ((hc->p_log = fopen("log.dat", "r")) == 0) {               // check if file exist
        if ((hc->p_log = fopen("log.dat", "a")) == NULL) {        // append new statistic at the end
            snprintf(hc->errortext, ET_SIZE, "Error open file %s  \n", "log.dat");
            error(hc->errortext, 500);
        } else {                                        // Create header for new log file
#ifdef M3624MDLOG
            fprintf(hc->p_log,
                    " ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
            fprintf(hc->p_log,
                    "| Encoder statistics. This file is generated during first encoding session, new sessions will be appended                                                                           |\n");
            fprintf(hc->p_log,
                    " ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
            fprintf(hc->p_log,
                    "|Date |Time | Sequence                                   |#Img|Q1|Qn|  Format |Hada|S R|Ref|Freq|IP|SNRY 1|SNRU 1|SNRV 1|SNRY N|SNRU N|SNRV N| #Bitr P | #Bitr B |Bit Rate| Ecode T |\n");
            fprintf(hc->p_log,
                    " ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
#endif
        }
    } else {
        fclose(hc->p_log);

        if ((hc->p_log = fopen("log.dat", "a")) == NULL) {        // File exist,just open for appending
            snprintf(hc->errortext, ET_SIZE, "Error open file %s  \n", "log.dat");
            error(hc->errortext, 500);
        }
    }

#ifdef WIN32
    _strdate(timebuf);
    fprintf(hc->p_log, "|%1.5s|", timebuf);

    _strtime(timebuf);
    fprintf(hc->p_log, "%1.5s|", timebuf);
#else
    now = time((time_t *) NULL);      // Get the system time and put it into 'now' as 'calender time'
    time(&now);
    l_time = localtime(&now);
    strftime(string, sizeof string, "%d-%b-%Y", l_time);
    fprintf(hc->p_log, "| %1.5s |", string);

    strftime(string, sizeof string, "%H:%M:%S", l_time);
    fprintf(hc->p_log, " %1.5s |", string);
#endif

    for (i = 0; i < 20; i++) {
        name[i] = input->infile[i + max(0, strlen(input->infile) - 20) ];
    }
#ifdef M3624MDLOG
    {
        char *ch;
        int i;
        for (i = (int)strlen(input->infile); i >= 0 && input->infile[i] != '\\'; ch = &input->infile[i--]);
        fprintf(hc->p_log, "%-44s|", ch);
    }
    fprintf(hc->p_log, "%4d|", input->no_frames);
    fprintf(hc->p_log, "%2d|", input->qpI);
    fprintf(hc->p_log, "%2d|", input->qpP);
    fprintf(hc->p_log, "%4dx%-4d|", img->width, img->height);

    if (input->hadamard == 1) {
        fprintf(hc->p_log, " ON |");
    } else {
        fprintf(hc->p_log, "OFF |");
    }

    fprintf(hc->p_log, "%3d|", input->search_range);
    fprintf(hc->p_log, "%2d |", input->no_multpred);
    fprintf(hc->p_log, " %2d |", img->framerate / (input->jumpd + 1));
    fprintf(hc->p_log, "%2d|", input->intra_period);
    fprintf(hc->p_log, "%6.3f|", snr->snr_y1);
    fprintf(hc->p_log, "%6.3f|", snr->snr_u1);
    fprintf(hc->p_log, "%6.3f|", snr->snr_v1);
    fprintf(hc->p_log, "%6.3f|", snr->snr_ya);
    fprintf(hc->p_log, "%6.3f|", snr->snr_ua);
    fprintf(hc->p_log, "%6.3f|", snr->snr_va);

    if (input->successive_Bframe != 0) {
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3) {
            fprintf(hc->p_log, "%9.0f|", stat->bitrate_P);
            fprintf(hc->p_log, "%9.0f|", stat->bitrate_B);
        } else {
            fprintf(hc->p_log, "%9.0f|", stat->bitrate_P);
            fprintf(hc->p_log, "%9.0f|", stat->bitrate_B);
        }
#else
        fprintf(hc->p_log, "%7.0f|", stat->bitrate_P);
        fprintf(hc->p_log, "%7.0f|\n", stat->bitrate_B);
#endif
    } else {
        fprintf(hc->p_log, "%9.0f|", stat->bitrate);
        fprintf(hc->p_log, "%9.0f|", 0.0);
    }
    fprintf(hc->p_log, "%8.2f|", stat->bitrate / 1000);
    fprintf(hc->p_log, "%9.2f|\n", hc->tot_time * 0.001);
#endif
    fclose(hc->p_log);
}


/*
*************************************************************************
* Function:Prints the header of the protocol.
* Input:struct inp_par *inp
* Output:
* Return:
* Attention:
*************************************************************************
*/

void information_init()
{

    printf("-----------------------------------------------------------------------------\n");
    printf(" Input YUV file                    : %s \n", input->infile);

    printf(" Output AVS bitstream              : %s \n", input->outfile);

    if (he->p_dec != NULL) {

        printf(" Output YUV file                   : %s \n", input->ReconFile);

    }

    printf(" Output log file                   : log.dat \n");
    printf(" Output statistics file            : stat.dat \n");
#if REFINED_QP
    if (input->use_refineQP) {
        fprintf(stdout, " REFINED_QP IS SWITCHED ON. THE INPUT QP FROM CFG FILE MIGHT BE CHANGED!\n");
    }
#endif
    printf("-----------------------------------------------------------------------------\n");


    if (input->MD5Enable == 2 || input->MD5Enable == 3) {
        printf(" Frame   Bit/pic   QP   SnrY    SnrU    SnrV    Time(ms)  FRM/FLD  IntraMBs\tYUV_MD5\n");
    } else {
        printf(" Frame   Bit/pic   QP   SnrY    SnrU    SnrV    Time(ms)  FRM/FLD  IntraMBs\n");
    }

}

/*
*************************************************************************
* Function:Dynamic memory allocation of frame size related global buffers
buffers are defined in global.h, allocated memory must be freed in
void free_global_buffers()
* Input:  Input Parameters struct inp_par *inp,                            \n
Image Parameters struct img_par *img
* Output:
* Return: Number of allocated bytes
* Attention:
*************************************************************************
*/


int init_global_buffers()
{
    int memory_size = 0;


    int i;

    if (input->bg_enable) {
        for (i = 0; i < 3; i++) {
            if (i == 0) {
                get_mem2D(&he->background_frame[i], img->height, img->width);
            } else {
                get_mem2D(&he->background_frame[i], img->height_cr, img->width_cr);
            }
        }
    }


#if TDRDO
    he->imgY_pre_buffer = (byte *)malloc((img->height * img->width * 3 / 2) * sizeof(byte));
#endif

    if (input->chroma_format == 1)

    {
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3) {
            he->imgY_org_buffer = (byte *)malloc((input->org_img_height * input->org_img_width * 3 / 2) * sizeof(byte));
            memory_size += (input->org_img_height * input->org_img_width * 3 / 2) * sizeof(byte);
            if (input->bg_enable) {
                he->imgY_GB_org_buffer = (byte *)malloc((input->org_img_height * input->org_img_width * 3 / 2) * sizeof(byte));
                memory_size += (input->org_img_height * input->org_img_width * 3 / 2) * sizeof(byte);
            }
        } else {
            he->imgY_org_buffer = (byte *)malloc((img->height * img->width * 3 / 2) * sizeof(byte));
            memory_size += (img->height * img->width * 3 / 2) * sizeof(byte);
            if (input->bg_enable) {
                he->imgY_GB_org_buffer = (byte *)malloc((img->height * img->width * 3 / 2) * sizeof(byte));
                memory_size += (img->height * img->width * 3 / 2) * sizeof(byte);
            }
        }
#else
        he->imgY_org_buffer = (byte *) malloc((img->height * img->width * 3 / 2) * sizeof(byte));
        memory_size += (img->height * img->width * 3 / 2) * sizeof(byte);
        if (input->bg_enable) {
            he->imgY_GB_org_buffer = (byte *) malloc((img->height * img->width * 3 / 2) * sizeof(byte));
            memory_size += (img->height * img->width * 3 / 2) * sizeof(byte);
        }

#endif
    }


    memory_size += get_mem2D(&hc->imgYPrev, img->height, img->width);
    memory_size += get_mem3D(&hc->imgUVPrev, 2, img->height_cr, img->width_cr);


    // allocate memory for reference frame buffers: imgY_org, imgUV_org
    memory_size += get_mem2D(&he->imgY_org, img->height, img->width);
    memory_size += get_mem3D(&he->imgUV_org, 2, img->height_cr, img->width_cr);
    // allocate memory for temp P and B-frame motion vector buffer: tmp_mv, temp_mv_block

    memory_size += get_mem3Dint(&img->tmp_mv, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE, 3);

    memory_size += get_mem3Dint(&img->p_snd_tmp_mv, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE, 3);


    // allocate memory for reference frames of each block: refFrArr
    memory_size += get_mem2Dint(&hc->refFrArr, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);

    memory_size += get_mem2Dint(&hc->p_snd_refFrArr, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);




    if (input->successive_Bframe != 0) {
        // allocate memory for temp B-frame motion vector buffer: fw_refFrArr, bw_refFrArr
        memory_size += get_mem2Dint(&img->fw_refFrArr, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);
        memory_size += get_mem2Dint(&img->bw_refFrArr, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);
    }

    // allocate memory for B frame coding: nextP_imgY, nextP_imgUV
    if (input->successive_Bframe != 0) {
        // allocate memory for temp B-frame motion vector buffer: img->fw_mv, img->bw_mv, dfMV, dbMV
        memory_size += get_mem3Dint(&img->fw_mv, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE, 3);
        memory_size += get_mem3Dint(&img->bw_mv, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE, 3);
    }

    // allocate memory for temp quarter pel luma frame buffer: img4Y_tmp
    memory_size += get_mem2Dint(&he->img4Y_tmp, (img->height + 2 * IMG_PAD_SIZE) * 4,
                                (img->width + 2 * IMG_PAD_SIZE) * 4);

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    if (input->usefme) {
        memory_size += get_mem_FME();
    }
    get_mem2D(&hc->imgY_sao, img->height, img->width);
    get_mem2D(&(hc->imgUV_sao[0]), img->height_cr, img->width_cr);
    get_mem2D(&(hc->imgUV_sao[1]), img->height_cr, img->width_cr);
    get_mem1D(&hc->imgY_alf_Org, img->height * img->width);
    get_mem1D(&hc->imgY_alf_Rec, (img->height * img->width));
    get_mem2D(&hc->imgUV_alf_Org, 2, (img->height_cr * img->width_cr));
    get_mem2D(&hc->imgUV_alf_Rec, 2, (img->height_cr * img->width_cr));

    for (i = 0; i < 3; i++) {
        if (i == 0) {
            get_mem2D(&hc->backgroundReferenceFrame[i], img->height, img->width);
        } else {
            get_mem2D(&hc->backgroundReferenceFrame[i], img->height_cr, img->width_cr);
        }
    }
    hc->background_ref = hc->backgroundReferenceFrame;


    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    Init_QMatrix();
#endif


    return (memory_size);
}

/*
*************************************************************************
* Function:Free allocated memory of frame size related global buffers
buffers are defined in global.h, allocated memory is allocated in
int get_mem4global_buffers()
* Input: Input Parameters struct inp_par *inp,                             \n
Image Parameters struct img_par *img
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_global_buffers()
{
    int  i, j;

#if TDRDO
    free(he->imgY_pre_buffer);
#endif
    if (input->bg_enable) {
        for (i = 0; i < 3; i++) {
            free_mem2D(he->background_frame[i]);
        }
        free_mem2D(he->background_frame_quarter);
    }


    free(he->imgY_org_buffer);

    if (input->bg_enable && he->imgY_GB_org_buffer != NULL) {
        free(he->imgY_GB_org_buffer);
    }
    free_mem2D(he->imgY_org);
    free_mem3D(he->imgUV_org, 2);

    free_mem3Dint(img->tmp_mv, img->height / MIN_BLOCK_SIZE);

    free_mem3Dint(img->p_snd_tmp_mv, img->height / MIN_BLOCK_SIZE);
    free_mem2Dint(hc->p_snd_refFrArr);

    free_mem2Dint(hc->refFrArr);
    free_mem2D(hc->imgYPrev);
    free_mem3D(hc->imgUVPrev, 2);
    // number of reference frames increased by one for next P-frame


    for (j = 0; j < 3; j++) {
        free_mem2D(hc->backgroundReferenceFrame[j]);
    }



    for (i = 0; i < REF_MAXBUFFER; i++) {
        free_frame_t(fref[i]);
    }


    // free multiple ref frame buffers
    // number of reference frames increased by one for next P-frame


    if (input->successive_Bframe != 0) {
        // free last P-frame buffers for B-frame coding
        free_mem3Dint(img->fw_mv, img->height / MIN_BLOCK_SIZE);
        free_mem3Dint(img->bw_mv, img->height / MIN_BLOCK_SIZE);
        free_mem2Dint(img->fw_refFrArr);
        free_mem2Dint(img->bw_refFrArr);
    } // end if B frame


    free_mem2Dint(he->img4Y_tmp);    // free temp quarter pel frame buffer
    // free mem, allocated in init_img()
    // free intra pred mode buffer for blocks
    free_mem2Dint(img->ipredmode);
    free_mem2Dint(img->rec_ipredmode);
    free(img->mb_data);

    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    if (input->usefme) {
        free_mem_FME();
    }

    free_mem2D(hc->imgY_sao);
    free_mem2D(hc->imgUV_sao[0]);
    free_mem2D(hc->imgUV_sao[1]);

    free_mem1D(hc->imgY_alf_Org);
    free_mem2D(hc->imgUV_alf_Org);
    free_mem1D(hc->imgY_alf_Rec);
    free_mem2D(hc->imgUV_alf_Rec);
    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    free_QMatrix();
#endif

}

/*
*************************************************************************
* Function:Allocate memory for mv
* Input:Image Parameters struct img_par *img                             \n
int****** mv
* Output:
* Return: memory size in bytes
* Attention:
*************************************************************************
*/

int get_mem_mv(int **** **mv)
{
    int i, j, k, l;

    if ((*mv = (int **** *) calloc(2, sizeof(int ** **))) == NULL)

    {
        no_mem_exit("get_mem_mv: mv");
    }

    for (i = 0; i < 2; i++) {
        if (((*mv) [i] = (int ** **) calloc(2, sizeof(int ** *))) == NULL) {
            no_mem_exit("get_mem_mv: mv");
        }

        for (j = 0; j < 2; j++) {
            if (((*mv) [i][j] = (int ** *) calloc(BUF_CYCLE, sizeof(int **))) == NULL) {
                no_mem_exit("get_mem_mv: mv");
            }

            for (k = 0; k < BUF_CYCLE; k++) {
                if (((*mv) [i][j][k] = (int **) calloc(9, sizeof(int *))) == NULL) {
                    no_mem_exit("get_mem_mv: mv");
                }

                for (l = 0; l < 9; l++) {
                    if (((*mv) [i][j][k][l] = (int *) calloc(3, sizeof(int))) == NULL) {               // mvx, mvy, dmh_mode
                        no_mem_exit("get_mem_mv: mv");
                    }
                }
            }
        }
    }

    return 2 * 2 * BUF_CYCLE * 9 * 2 * sizeof(int);
}


/*
*************************************************************************
* Function:Free memory from mv
* Input:int****** mv
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_mem_mv(int ** *** mv)
{
    int i, j, k, l;

    for (i = 0; i < 2; i++) {
        for (j = 0; j < 2; j++) {
            for (k = 0; k < BUF_CYCLE; k++) {
                for (l = 0; l < 9; l++) {
                    free(mv[i][j][k][l]);
                }

                free(mv[i][j][k]);
            }

            free(mv[i][j]);
        }

        free(mv[i]);
    }

    free(mv);
}
/*
*************************************************************************
* Function:SetImgType
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void SetImgType()
{
    int model_number, start;
    start = (input->bg_input_number == 0);
    model_number = (he->last_background_frame_number ? (input->bg_model_number / (1 + input->successive_Bframe)) :
                    (input->bg_model_number / (1 + input->successive_Bframe) + (input->bg_model_number %
                            (1 + input->successive_Bframe) != 0)));
    if (!(input->bg_enable)) {

        if (input->intra_period == 0) {
            if (img->number == 0) {
                img->type = INTRA_IMG;        // set image type for first image to I-frame
            } else {
                if (input->fframe_enabled) {
                    img->type = F_IMG;
                } else {
                    img->type = P_IMG;
                }
                picture_coding_type = 0;
            }
        } else if (input->intra_period == 1) {
            img->type = INTRA_IMG;
        } else {
            if ((img->numIPFrames % input->intra_period) == 0) {
                int i;
                int tmp = ((img->numIPFrames) / he->subGopNum) * (input->successive_Bframe_all + he->subGopNum);
                for (i = 0; i < ((img->numIPFrames) % he->subGopNum); i++) {
                    tmp += input->successive_Bframe_sub[i];
                }
                if (tmp + 1 >= input->no_frames && input->no_frames != 1) {
                    if (input->fframe_enabled) {
                        img->type = F_IMG;
                    } else {
                        img->type = P_IMG;
                    }
                } else {
                    img->type = INTRA_IMG;
                }
            } else {
                if (input->fframe_enabled) {
                    img->type = F_IMG;
                } else {
                    img->type = P_IMG;
                }
            }
        }

        img->typeb = 0;
    } else {
        if (img->number == 0) {           //the first frame,output G frame
            img->type = INTRA_IMG;
            img->typeb = BACKGROUND_IMG;
            he->intra_temp = 0;
        } else if (input->bg_model_number && img->number - model_number == 0 &&
                   input->always_no_bgmodel == 0) {  //the first trained GB frame
            img->type = INTRA_IMG;
            img->typeb = BACKGROUND_IMG;
            if (img->number - he->background_number - he->intra_temp == input->intra_period && input->always_no_bgmodel) {
                he->intra_temp = img->number - he->background_number;
            }
        } else if (input->bg_period) {
            if (img->number - he->last_background_frame_number == input->bg_period + (1 -
                    input->always_no_bgmodel)) { //the updated GB/B frame
                img->type = INTRA_IMG;
                img->typeb = BACKGROUND_IMG;
                if (img->number - he->background_number - he->intra_temp == input->intra_period && input->always_no_bgmodel) {
                    he->intra_temp = img->number - he->background_number;
                }
            } else if (input->intra_period == 0) {                                //under intra_period==0 condition
                if (input->fframe_enabled) {
                    img->type = F_IMG;
                    picture_coding_type = 3;
                    img->typeb = 0;
                    he->background_reference_enable = 1;
                } else {
                    img->type = INTER_IMG;
                    picture_coding_type = 0;
                    img->typeb = 0;
                    he->background_reference_enable = 1;
                }
            } else if (img->number - he->background_number - he->intra_temp == input->intra_period) {         //S frame
                he->intra_temp = img->number - he->background_number;
                img->type = INTER_IMG;
                img->typeb = BP_IMG;
                he->background_reference_enable = 0;
            }

            else {                                                                //P frame
                if (input->fframe_enabled) {
                    img->type = F_IMG;
                    picture_coding_type = 3;
                    img->typeb = 0;
                    he->background_reference_enable = 1;
                } else {
                    img->type = INTER_IMG;
                    img->typeb = 0;
                    he->background_reference_enable = 1;
                }
            }
        } else if (input->intra_period == 0) {
            if (input->fframe_enabled) {
                img->type = F_IMG;
                picture_coding_type = 3;
                img->typeb = 0;
                he->background_reference_enable = 1;
            } else {
                img->type = INTER_IMG;
                picture_coding_type = 0;
                img->typeb = 0;
                he->background_reference_enable = 1;
            }
        } else if (img->number - he->background_number - he->intra_temp == input->intra_period) {         //S frame
            he->intra_temp = img->number - he->background_number;
            img->type = INTER_IMG;
            img->typeb = BP_IMG;
            he->background_reference_enable = 0;
        }

        else {
            if (input->fframe_enabled) {
                img->type = F_IMG;
                picture_coding_type = 3;
                img->typeb = 0;
                he->background_reference_enable = 1;
            } else {
                img->type = INTER_IMG;
                img->typeb = 0;
                he->background_reference_enable = 1;
            }
        }
    }

    if (img->typeb == BACKGROUND_IMG) {
        if (input->bg_input_number) {
            he->background_output_flag = 1;
        } else {
            he->background_output_flag = 0;
        }
    } else {
        he->background_output_flag = 1;
    }



}

#if INTERLACE_CODING
void mergeFld2Frm()
{
    int input_width_cr  = (input->org_img_width / 2);
    int input_height_cr = (input->org_img_height / 2);
    unsigned int  bytes_y = input->org_img_width * input->org_img_height;
    const unsigned int  bytes_uv = input_width_cr * input_height_cr;
    const long long int framesize_in_bytes = bytes_y + 2 * bytes_uv;


    int off_y_fld = bytes_y / 2;
    int off_u_fld = input->org_img_width * input->org_img_height / 8;
    int off_uv_fld = bytes_uv / 2;
    int off_yuv_fld = off_y_fld + 2 * off_uv_fld;

    unsigned char   *imgY_org_buf, *imgY_org_buf_fld, *imgY_out_buf;

    int diff_y, diff_u, diff_v;

    FILE *p_org_img;
    FILE *p_rec_img;
    FILE *p_out_img;

    char dst_fld[1000] = "\0";

    double maxSignal = (double)((1 << input->input_sample_bit_depth) - 1) * (double)((1 << input->input_sample_bit_depth) -
                       1);
    int nSampleSize = (input->input_sample_bit_depth == 8 ? 1 : 2);
    int shift1 = input->sample_bit_depth - input->input_sample_bit_depth;

    int i, w, h;

#define _S_IREAD      0000400         /* read permission, owner */
#define _S_IWRITE     0000200         /* write permission, owner */

    strcat(dst_fld, input->ReconFile);
    strcat(dst_fld, ".fld.yuv");

    diff_y = diff_u = diff_v = 0;

    imgY_org_buf = (unsigned char *) malloc(input->org_img_width * input->org_img_height * 3 / 2);
    imgY_org_buf_fld = (unsigned char *) malloc(input->org_img_width * input->org_img_height * 3 / 2);
    imgY_out_buf = (unsigned char *) malloc(input->org_img_width * input->org_img_height * 3 / 2);

    if (strlen(input->infile) > 0 && (p_org_img = fopen(input->infile, "rb")) == NULL) {
        printf("Input file %s does not exist", input->infile);
    }

    if (strlen(input->ReconFile) > 0 && (p_rec_img = fopen(input->ReconFile, "rb")) == NULL) {
        printf("Input file %s does not exist", input->ReconFile);
    }

    if (input->output_merged_picture)
        if (strlen(dst_fld) > 0 && (p_out_img = fopen(dst_fld, "wb")) == NULL) {
            printf("Input file %s does not exist", dst_fld);
        }

    for (i = 0; i < input->org_no_frames; i++) {
        diff_y = diff_u = diff_v = 0;
        readOneFrameFromDisk(framesize_in_bytes * nSampleSize, i, p_org_img, imgY_org_buf);
        readOneFrameFromDisk(framesize_in_bytes * nSampleSize, i, p_rec_img, imgY_org_buf_fld);

        for (h = 0; h < input->org_img_height / 2; h++)
            for (w = 0; w < input->org_img_width; w++) {
                diff_y += img->quad[abs(imgY_org_buf[2 * h * input->org_img_width + w] - imgY_org_buf_fld[h * input->org_img_width +
                                        w])];
                diff_y += img->quad[abs(imgY_org_buf[(2 * h + 1) * input->org_img_width + w] - imgY_org_buf_fld[off_yuv_fld + h *
                                        input->org_img_width + w])];
                imgY_out_buf[2 * h * input->org_img_width + w] = imgY_org_buf_fld[h * input->org_img_width + w];
                imgY_out_buf[(2 * h + 1) * input->org_img_width + w] = imgY_org_buf_fld[off_yuv_fld + h * input->org_img_width + w];
            }

        for (h = 0; h < input->org_img_height / 4; h++)
            for (w = 0; w < input->org_img_width / 2; w++) {
                diff_u += img->quad[abs(imgY_org_buf[bytes_y + 2 * h * input->org_img_width / 2 + w] - imgY_org_buf_fld[off_y_fld + h *
                                        input->org_img_width / 2 + w])];
                diff_u += img->quad[abs(imgY_org_buf[bytes_y + (2 * h + 1) * input->org_img_width / 2 + w] -
                                        imgY_org_buf_fld[off_yuv_fld + off_y_fld + h * input->org_img_width / 2 + w])];
                imgY_out_buf[bytes_y + 2 * h * input->org_img_width / 2 + w] = imgY_org_buf_fld[off_y_fld + h * input->org_img_width / 2
                        + w];
                imgY_out_buf[bytes_y + (2 * h + 1) * input->org_img_width / 2 + w] = imgY_org_buf_fld[off_yuv_fld + off_y_fld + h *
                        input->org_img_width / 2 + w];

                diff_v += img->quad[abs(imgY_org_buf[bytes_y + 2 * off_u_fld + 2 * h * input->org_img_width / 2 + w] -
                                        imgY_org_buf_fld[off_y_fld + off_u_fld + h * input->org_img_width / 2 + w])];
                diff_v += img->quad[abs(imgY_org_buf[bytes_y + 2 * off_u_fld + (2 * h + 1) * input->org_img_width / 2 + w] -
                                        imgY_org_buf_fld[off_yuv_fld + off_u_fld + off_y_fld + h * input->org_img_width / 2 + w])];
                imgY_out_buf[bytes_y + 2 * off_u_fld + 2 * h * input->org_img_width / 2 + w] = imgY_org_buf_fld[off_y_fld + off_u_fld +
                        h * input->org_img_width / 2 + w];
                imgY_out_buf[bytes_y + 2 * off_u_fld + (2 * h + 1) * input->org_img_width / 2 + w] = imgY_org_buf_fld[off_yuv_fld +
                        off_u_fld + off_y_fld + h * input->org_img_width / 2 + w];
            }

        if (input->output_merged_picture) {
            fwrite(imgY_out_buf, sizeof(unsigned char), (size_t)(framesize_in_bytes * nSampleSize), p_out_img);
            fflush(p_out_img);
        }

        if (i == 0) {
            snr->i_snr_ya = 10 * log10(maxSignal * (float)(input->org_img_width * input->org_img_height) / (float)diff_y);
            snr->i_snr_ua = 10 * log10(maxSignal * (float)(input_width_cr * input_height_cr) / (float)diff_u);
            snr->i_snr_va = 10 * log10(maxSignal * (float)(input_width_cr * input_height_cr) / (float)diff_v);
        } else {
            snr->i_snr_ya = (snr->i_snr_ya * (double)i + 10 * log10(maxSignal * (float)(input->org_img_width *
                             input->org_img_height) / (float)diff_y)) / (double)(i + 1);
            snr->i_snr_ua = (snr->i_snr_ua * (double)i + 10 * log10(maxSignal * (float)(input_width_cr * input_height_cr) /
                             (float)diff_u)) / (double)(i + 1);
            snr->i_snr_va = (snr->i_snr_va * (double)i + 10 * log10(maxSignal * (float)(input_width_cr * input_height_cr) /
                             (float)diff_v)) / (double)(i + 1);
        }
    }

    free(imgY_org_buf);
    free(imgY_org_buf_fld);
    free(imgY_out_buf);
    fclose(p_org_img);
    fclose(p_rec_img);
    if (input->output_merged_picture) {
        fclose(p_out_img);
    }
}

void readOneFrameFromDisk(long long int framesize_in_bytes, int FrameNoInFile, FILE *p_img, unsigned char   *buf)
{
    if (fseek(p_img, framesize_in_bytes * FrameNoInFile, SEEK_SET) != 0) {
        printf("ReadOneFrame: cannot fseek to (Header size) in p_img");
    }

    if (fread(buf, 1, (size_t)framesize_in_bytes, p_img) != (int) framesize_in_bytes) {
        printf("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", framesize_in_bytes);
    }
}
#endif


