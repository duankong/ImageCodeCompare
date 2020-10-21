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
#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>

#include "header.h"
#include "../../lcommon/inc/defines.h"
#include "../../lcommon/inc/transform.h"
#include "vlc.h"




#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif

#if ROI_M3264
#include "pos_info.h"
#endif


const int  InterlaceSetting[] = { 4, 4, 0, 5, 0, 5, 0, 5, 3, 3, 3, 3, 0,
                                  0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 0, 0, 4, 0
                                };  //rm52k

/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int frametotc(int frame, int reserve_bit)
{
    int fps, pict, sec, minute, hour, tc;

    fps = (int)(img->framerate + 0.5);
    pict = frame % fps;
    frame = (frame - pict) / fps;
    sec = frame % 60;
    frame = (frame - sec) / 60;
    minute = frame % 60;
    frame = (frame - minute) / 60;
    hour = frame % 24;

    if (fps >= 64) {
        pict = (int)(pict * 64 / fps);
    }

    tc = (reserve_bit << 23) | (hour << 18) | (minute << 12) | (sec << 6) | pict;

    return tc;
}

int IsEqualRps(ref_man Rps1, ref_man Rps2)
{
    int i, sum = 0;
    sum += (Rps1.num_of_ref != Rps2.num_of_ref);
    for (i = 0; i < Rps1.num_of_ref; i++) {
        sum += (Rps1.ref_pic[i] != Rps2.ref_pic[i]);
    }
    sum += (Rps1.remove_pic != Rps2.remove_pic);
    for (i = 0; i < Rps1.num_to_remove; i++) {
        sum += (Rps1.remove_pic[i] != Rps2.remove_pic[i]);
    }
    return (sum == 0);
}

static int get_valid_qp(int i_qp)
{
    int i_max_qp = 63 + 8 * (input->sample_bit_depth - 8);
    return Clip3(0, i_max_qp, i_qp);
}

/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int IPictureHeader(int frame)
{
    Bitstream *bitstream = currBitStream;
    int i;
    int len = 0;
    int tc;
    int time_code_flag;
    int bbv_delay, bbv_delay_extension;

    // Adaptive frequency weighting quantization
#if (FREQUENCY_WEIGHTING_QUANTIZATION&&COUNT_BIT_OVERHEAD)
    int bit_overhead;
#endif

    marker_bit = 1;
    time_code_flag = 0;

#if !INTERLACE_CODING
    interlaceIdx = input->progressive_sequence * 16 + img->progressive_frame * 8 + img->picture_structure * 4
                   + input->top_field_first * 2 + input->repeat_first_field; //rm52k
    CheckInterlaceSetting(interlaceIdx);                                     //rm52k
#endif

    len += u_v(32, "I picture start code", 0x1B3, bitstream);

#if MB_DQP
    if (input->useDQP) {
        input->fixed_picture_qp = 0;/*lgp*/
    } else {
        input->fixed_picture_qp = 1;/*lgp*/
    }
#else
    input->fixed_picture_qp = 1;
#endif

    bbv_delay = 0xFFFF;
    bbv_delay_extension = 0xFF;

    //xyji 12.23
    len += u_v(32, "bbv delay", bbv_delay, bitstream);

    len += u_v(1, "time_code_flag", 0, bitstream);

    if (time_code_flag) {
        tc = frametotc(frame, img->tc_reserve_bit);
        len += u_v(24, "time_code", tc, bitstream);
    }
    if (input->bg_enable) {
        len += u_v(1, "background_picture_flag", img->typeb == BACKGROUND_IMG, bitstream);
        if (img->typeb == BACKGROUND_IMG) {
            len += u_v(1, "background_picture_output_flag", he->background_output_flag, bitstream);
            if (he->duplicated_gb_flag == 0) {
                he->last_background_frame_number = img->number;
            }
        }
    }


    {
        unsigned int ROI_coding_order = hc->coding_order % 256;
        int displaydelay;
        if (input->low_delay == 0) {
            displaydelay = img->tr - hc->coding_order + he->picture_reorder_delay;//add sixiaohua
        }
        len += u_v(8, "coding_order", ROI_coding_order, bitstream);

#if M3480_TEMPORAL_SCALABLE
        if (he->temporal_id_exist_flag == 1) {
            if (img->typeb == BACKGROUND_IMG || (img->type == INTER_IMG && img->typeb == BP_IMG)) {
                he->cur_layer = 0;
            }
            len += u_v(TEMPORAL_MAXLEVEL_BIT, "temporal_id", he->cur_layer, bitstream);
        }
#endif
        if (input->low_delay == 0 && !(input->bg_enable && he->background_output_flag == 0)) { //cdp
            len += ue_v("picture_output_delay", displaydelay, bitstream);
        }
    }
    {
        int RPS_idx = (hc->coding_order - 1) % he->gop_size_all;

        int IsEqual = IsEqualRps(he->curr_RPS, he->cfg_ref_all[RPS_idx]);

        he->use_RPSflag = 0;
        if (input->intra_period == 1) {
            he->use_RPSflag = 1;
        }
        len += u_v(1, "use RCS in SPS", (IsEqual || he->use_RPSflag), bitstream);
        if (IsEqual || he->use_RPSflag) {
            len += u_v(5, "predict for RCS", RPS_idx, bitstream);
        } else {
            int j;
            len += u_v(1, "refered by others ", he->curr_RPS.referd_by_others, bitstream);
            len += u_v(3, "num of reference picture", he->curr_RPS.num_of_ref, bitstream);
            for (j = 0; j < he->curr_RPS.num_of_ref; j++) {
                len += u_v(6, "delta COI of ref pic", he->curr_RPS.ref_pic[j], bitstream);
            }
            len += u_v(3, "num of removed picture", he->curr_RPS.num_to_remove, bitstream);
            for (j = 0; j < he->curr_RPS.num_to_remove; j++) {
                len += u_v(6, "delta COI of removed pic", he->curr_RPS.remove_pic[j], bitstream);
            }
            len += u_v(1, "marker bit", 1, bitstream);

        }
        he->use_RPSflag = 1;
    }

    if (input->low_delay) {
        len += ue_v("bbv check times", bbv_check_times, bitstream);
    }

    len += u_v(1, "progressive frame", img->progressive_frame, bitstream);

    if (!img->progressive_frame) {
        len += u_v(1, "picture_structure", img->picture_structure, bitstream);
    }

    len += u_v(1, "top field first", input->top_field_first, bitstream);
    len += u_v(1, "repeat first field", input->repeat_first_field, bitstream);
#if INTERLACE_CODING
    if (input->InterlaceCodingOption == 3) {
        len += u_v(1, "is top field", img->is_top_field, bitstream);
        len += u_v(1, "reserved bit for interlace coding", 1, bitstream);
    }
#endif
    len += u_v(1, "fixed picture qp", input->fixed_picture_qp, bitstream);

    if (input->bg_enable) {
        if (img->typeb == BACKGROUND_IMG && (he->background_output_flag == 0)
            && he->gb_is_ready == 1
           ) {
            img->qp = input->bg_qp;    //set GB frame's qp
        }
        img->qp = get_valid_qp(img->qp);
        len += u_v(7, "I picture QP", img->qp, bitstream);
    } else {
#if REFINED_QP
        if (input->use_refineQP) {
            img->qp = get_valid_qp(img->qp);
            len += u_v(7, "I picture QP", img->qp, bitstream);
        } else {
            input->qpI = get_valid_qp(input->qpI);
            len += u_v(7, "I picture QP", input->qpI, bitstream);
        }
#else
        len += u_v(6, "I picture QP", input->qpI, bitstream);
#endif
#if REFINED_QP
        if (!input->use_refineQP) {
#if RATECONTROL
            if (input->EncControl == 0)
#endif
                img->qp = input->qpI;
        }
#else
#if RATECONTROL
        if (input->EncControl == 0)
#endif
            img->qp = input->qpI;
#endif
    }


    len += u_v(1, "loop filter disable", input->loop_filter_disable, bitstream);

    if (!input->loop_filter_disable) {
        len += u_v(1, "loop filter parameter flag", input->loop_filter_parameter_flag, bitstream);

        if (input->loop_filter_parameter_flag) {
            len += se_v("alpha offset", input->alpha_c_offset, bitstream);
            len += se_v("beta offset", input->beta_offset, bitstream);
        } else { // set alpha_offset, beta_offset to 0 when loop filter parameters aren't transmitted. encoder issue
            input->alpha_c_offset = 0;
            input->beta_offset = 0;
        }
    }

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#if COUNT_BIT_OVERHEAD
    bit_overhead = len;
#endif




#if CHROMA_DELTA_QP
    len += u_v(1, "chroma_quant_param_disable", input->chroma_quant_param_disable, bitstream);
  
#if !HDR_CHROMA_DELTA_QP
	if (!input->chroma_quant_param_disable) {
		len += se_v("chroma_quant_param_delta_cb", input->chroma_quant_param_delta_u, bitstream);
		len += se_v("chroma_quant_param_delta_cr", input->chroma_quant_param_delta_v, bitstream);
	} else {
		input->chroma_quant_param_delta_u = 0;
		input->chroma_quant_param_delta_v = 0;
	}
#else
if(!input->chroma_quant_param_disable){
	if(!input->chroma_hdr_chroma_delta_disable)	{
		double chromaQpScale_d, chromaCbQpScale_d, chromaCrQpScale_d, chromaQpOffset_d, chromaQp = 0.0;
		int cbQP, crQP = 0;
		chromaQpScale_d = input->chromaQpScale/100.0;
		chromaQpOffset_d = input->chromaQpOffset/100.0;
		chromaCbQpScale_d = input->chromaCbQpScale/100.0;
		chromaCrQpScale_d = input->chromaCrQpScale/100.0;

		chromaQp = chromaQpScale_d * img->qp + chromaQpOffset_d;
		cbQP = chromaQp < 0 ? (int)(chromaCbQpScale_d * chromaQp - 0.5) : (int)(chromaCbQpScale_d * chromaQp + 0.5);
		crQP = chromaQp < 0 ? (int)(chromaCrQpScale_d * chromaQp - 0.5) : (int)(chromaCrQpScale_d * chromaQp + 0.5);
		input->chroma_quant_param_delta_u = Clip3( -16, 16, min(0, cbQP) + input->chroma_quant_param_delta_u );
		input->chroma_quant_param_delta_v = Clip3( -16, 16, min(0, crQP) + input->chroma_quant_param_delta_v );

		len+=se_v("chroma_quant_param_delta_cb", input->chroma_quant_param_delta_u,bitstream);
		len+=se_v("chroma_quant_param_delta_cr", input->chroma_quant_param_delta_v,bitstream);
	}
	else{
	
		len += se_v("chroma_quant_param_delta_cb", input->chroma_quant_param_delta_u, bitstream);
		len += se_v("chroma_quant_param_delta_cr", input->chroma_quant_param_delta_v, bitstream);
	}
}
else{
		input->chroma_quant_param_delta_u = 0;
		input->chroma_quant_param_delta_v = 0;
	
}
#endif

#endif

    if (input->WQEnable) {
        len += u_v(1, "pic_weight_quant_enable", input->PicWQEnable, bitstream);
        if (input->PicWQEnable) {
            len += u_v(2, "pic_weight_quant_data_index", input->PicWQDataIndex, bitstream);

            if (input->PicWQDataIndex == 1) {
                len += u_v(1, "reserved_bits", 0, bitstream); //M2331 2008-04
#if CHROMA_DELTA_QP
#endif
                len += u_v(2, "weighting_quant_param_index", input->WQParam, bitstream);
                len += u_v(2, "weighting_quant_model", input->WQModel, bitstream);

                //M2331 2008-04
                if ((input->WQParam == 1) || ((input->MBAdaptQuant) && (input->WQParam == 3))) {
                    for (i = 0; i < 6; i++) {
#if Wq_param_Clip
						wq_param[UNDETAILED][i]=Clip3(1,255,wq_param[UNDETAILED][i]);
						len += se_v("quant_param_delta_u", Clip3(-128,127,(wq_param[UNDETAILED][i] - wq_param_default[UNDETAILED][i])), bitstream);
#else
                        len += se_v("quant_param_delta_u", (wq_param[UNDETAILED][i] - wq_param_default[UNDETAILED][i]), bitstream);
#endif
                    }
                }
                if ((input->WQParam == 2) || ((input->MBAdaptQuant) && (input->WQParam == 3))) {
                    for (i = 0; i < 6; i++) {
#if Wq_param_Clip
						wq_param[DETAILED][i]=Clip3(1,255,wq_param[DETAILED][i]);
						len += se_v("quant_param_delta_d", Clip3(-128,127,(wq_param[DETAILED][i] - wq_param_default[DETAILED][i])), bitstream);
#else
                        len += se_v("quant_param_delta_d", (wq_param[DETAILED][i] - wq_param_default[DETAILED][i]), bitstream);
#endif
                    }
                }

                //M2331 2008-04
            }//input->PicWQDataIndex== 1
            else if (input->PicWQDataIndex == 2) {
                int x, y, sizeId, uiWqMSize;
                for (sizeId = 0; sizeId < 2; sizeId++) {
                    int i = 0;
                    uiWqMSize = min(1 << (sizeId + 2), 8);
                    for (y = 0; y < uiWqMSize; y++) {
                        for (x = 0; x < uiWqMSize; x++) {
#if Wq_param_Clip
							pic_user_wq_matrix[sizeId][i]=Clip3(1,255,pic_user_wq_matrix[sizeId][i]);
#endif
                            len += ue_v("weight_quant_coeff", pic_user_wq_matrix[sizeId][i++], bitstream);
                        }
                    }
                }
            } //input->PicWQDataIndex== 2
        }
    }
#if COUNT_BIT_OVERHEAD
    he->g_count_overhead_bit += len - bit_overhead;
    printf(" QM_overhead I : %8d\n", (len - bit_overhead));
#endif

#endif


    return len;
}


/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int PBPictureHeader()
{
    Bitstream *bitstream = currBitStream;
    int i;
    int len = 0;
    int bbv_delay, bbv_delay_extension;

    // Adaptive frequency weighting quantization
#if (FREQUENCY_WEIGHTING_QUANTIZATION&&COUNT_BIT_OVERHEAD)
    int bit_overhead;
#endif
#if !INTERLACE_CODING
    interlaceIdx = input->progressive_sequence * 16 + img->progressive_frame * 8 + img->picture_structure * 4
                   + input->top_field_first * 2 + input->repeat_first_field; //rm52k
    CheckInterlaceSetting(interlaceIdx);                                     //rm52k
#endif

    if (img->type == INTER_IMG) {
        picture_coding_type = 1;
    } else if (img->type == F_IMG) {
        picture_coding_type = 3;
    } else {
        picture_coding_type = 2;  //add by wuzhongmou
    }

    if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) {
        he->background_pred_flag = 1;
    } //S frame picture coding type
    else {
        he->background_pred_flag = 0;
    }


#if MB_DQP
    if (input->useDQP) {
        input->fixed_picture_qp = 0;/*lgp*/
    } else {
        input->fixed_picture_qp = 1;/*lgp*/
    }

#else
    input->fixed_picture_qp = 1;
#endif

    bbv_delay = 0xFFFF;
    bbv_delay_extension = 0xFF;

    len += u_v(24, "start code prefix", 1, bitstream);
    len += u_v(8, "PB picture start code", 0xB6, bitstream);
    //xyji 12.23
    len += u_v(32, "bbv delay", bbv_delay, bitstream);

    len += u_v(2, "picture coding type", picture_coding_type, bitstream);

    if (input->bg_enable && (picture_coding_type == 1 || picture_coding_type == 3)) {
        if (picture_coding_type == 1) {   //only P can be extended to S
            len += u_v(1, "background_pred_flag", he->background_pred_flag, bitstream);
        }
        if (he->background_pred_flag == 0) {

            len += u_v(1, "background_reference_enable", he->background_reference_enable, bitstream);
        }

    }

    {
        unsigned int ROI_coding_order = hc->coding_order % 256;
        int displaydelay;
        if (input->low_delay == 0) {
            displaydelay = img->tr - hc->coding_order + he->picture_reorder_delay;//add sixiaohua
        }
        len += u_v(8, "coding_order", ROI_coding_order, bitstream);


#if M3480_TEMPORAL_SCALABLE
        if (he->temporal_id_exist_flag == 1) {
            if (img->typeb == BACKGROUND_IMG || (img->type == INTER_IMG && img->typeb == BP_IMG)) {
                he->cur_layer = 0;
            }
            len += u_v(TEMPORAL_MAXLEVEL_BIT, "temporal_id", he->cur_layer, bitstream);
        }
#endif
        if (input->low_delay == 0) {
            len += ue_v("displaydelay", displaydelay, bitstream);
        }
    }
    {
        int RPS_idx = (hc->coding_order - 1) % he->gop_size_all;

        int IsEqual = 1;
        if (he->curr_RPS.num_of_ref != he->cfg_ref_all[RPS_idx].num_of_ref ||
            he->curr_RPS.num_to_remove != he->cfg_ref_all[RPS_idx].num_to_remove) {
            IsEqual = 0;
        } else {
            for (i = 0; i < he->curr_RPS.num_of_ref; i++) {
                if (he->curr_RPS.ref_pic[i] != he->cfg_ref_all[RPS_idx].ref_pic[i]) {
                    IsEqual = 0;
                }
            }
        }
        len += u_v(1, "use RPS in SPS", IsEqual, bitstream);
        if (IsEqual) {
            len += u_v(5, "predict for RPS", RPS_idx, bitstream);
        } else {
            int j;
            len += u_v(1, "refered by others ", he->curr_RPS.referd_by_others, bitstream);
            len += u_v(3, "num of reference picture", he->curr_RPS.num_of_ref, bitstream);
            for (j = 0; j < he->curr_RPS.num_of_ref; j++) {
                len += u_v(6, "delta COI of ref pic", he->curr_RPS.ref_pic[j], bitstream);
            }
            len += u_v(3, "num of removed picture", he->curr_RPS.num_to_remove, bitstream);
            for (j = 0; j < he->curr_RPS.num_to_remove; j++) {
                len += u_v(6, "delta COI of removed pic", he->curr_RPS.remove_pic[j], bitstream);
            }
            len += u_v(1, "marker bit", 1, bitstream);

        }
    }

    if (input->low_delay) {   //cjw 20070414
        len += ue_v("bbv check times", bbv_check_times, bitstream);
    }

    len += u_v(1, "progressive frame", img->progressive_frame, bitstream);

    if (!img->progressive_frame) {
        len += u_v(1, "picture_structure", img->picture_structure, bitstream);
    }

    len += u_v(1, "top field first", input->top_field_first, bitstream);
    len += u_v(1, "repeat first field", input->repeat_first_field, bitstream);
#if INTERLACE_CODING
    if (input->InterlaceCodingOption == 3) {
        len += u_v(1, "is top field", img->is_top_field, bitstream);
        len += u_v(1, "reserved bit for interlace coding", 1, bitstream);
    }
#endif
    len += u_v(1, "fixed qp", input->fixed_picture_qp, bitstream);

    //rate control
    img->qp = get_valid_qp(img->qp);
    if (img->type == INTER_IMG) {
        len += u_v(7, "P picture QP", img->qp, bitstream);
    } else if (img->type == F_IMG) {
        len += u_v(7, "F picture QP", img->qp, bitstream);
    } else if (img->type == B_IMG) {
        len += u_v(7, "B picture QP", img->qp, bitstream);
    }

    if (!(picture_coding_type == 2 && img->picture_structure == 1)) {
        len += u_v(1, "reserved_bit", 0, bitstream);
    }

    if (img->tr >= he->next_IDRtr) {
        len += u_v(1, "random_access_decodable_flag", 1, bitstream);
    } else {
        len += u_v(1, "random_access_decodable_flag", 0, bitstream);
    }

    //len += u_v ( 3, "reserved bits", 0, bitstream );  // Added by cjw, 20070327


//
//   len+=u_v(1,"skip mode flag",input->skip_mode_flag, bitstream);  //qyu 0822 delete skip_mode_flag

    len += u_v(1, "loop filter disable", input->loop_filter_disable, bitstream);

    if (!input->loop_filter_disable) {
        len += u_v(1, "loop filter parameter flag", input->loop_filter_parameter_flag, bitstream);

        if (input->loop_filter_parameter_flag) {
            len += se_v("alpha offset", input->alpha_c_offset, bitstream);
            len += se_v("beta offset", input->beta_offset, bitstream);
        } else { // set alpha_offset, beta_offset to 0 when loop filter parameters aren't transmitted. encoder issue
            input->alpha_c_offset = 0;
            input->beta_offset = 0;
        }
    }

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#if COUNT_BIT_OVERHEAD
    bit_overhead = len;
#endif

#if CHROMA_DELTA_QP
    len += u_v(1, "chroma_quant_param_disable", input->chroma_quant_param_disable, bitstream);
#if !HDR_CHROMA_DELTA_QP
	if (!input->chroma_quant_param_disable) {
        len += se_v("chroma_quant_param_delta_cb", input->chroma_quant_param_delta_u, bitstream);
        len += se_v("chroma_quant_param_delta_cr", input->chroma_quant_param_delta_v, bitstream);
    } else {
        input->chroma_quant_param_delta_u = 0;
        input->chroma_quant_param_delta_v = 0;
    }
#else
	if(!input->chroma_quant_param_disable){
		if(!input->chroma_hdr_chroma_delta_disable){	
		double chromaQpScale_d, chromaCbQpScale_d, chromaCrQpScale_d, chromaQpOffset_d, chromaQp = 0.0;
		int cbQP, crQP = 0;
		chromaQpScale_d = input->chromaQpScale/100.0;
		chromaQpOffset_d = input->chromaQpOffset/100.0;
		chromaCbQpScale_d = input->chromaCbQpScale/100.0;
		chromaCrQpScale_d = input->chromaCrQpScale/100.0;

		chromaQp = chromaQpScale_d * img->qp + chromaQpOffset_d;
		cbQP = chromaQp < 0 ? (int)(chromaCbQpScale_d * chromaQp - 0.5) : (int)(chromaCbQpScale_d * chromaQp + 0.5);
		crQP = chromaQp < 0 ? (int)(chromaCrQpScale_d * chromaQp - 0.5) : (int)(chromaCrQpScale_d * chromaQp + 0.5);
		input->chroma_quant_param_delta_u = Clip3( -16, 16, min(0, cbQP) + input->chroma_quant_param_delta_u );
		input->chroma_quant_param_delta_v = Clip3( -16, 16, min(0, crQP) + input->chroma_quant_param_delta_v );

		len+=se_v("chroma_quant_param_delta_cb", input->chroma_quant_param_delta_u,bitstream);
		len+=se_v("chroma_quant_param_delta_cr", input->chroma_quant_param_delta_v,bitstream);

		}
		else{
			len += se_v("chroma_quant_param_delta_cb", input->chroma_quant_param_delta_u, bitstream);
			len += se_v("chroma_quant_param_delta_cr", input->chroma_quant_param_delta_v, bitstream);
		} 

	}
	else{
	
		input->chroma_quant_param_delta_u = 0;
		input->chroma_quant_param_delta_v = 0;
	}
#endif
#endif

    if (input->WQEnable) {
        len += u_v(1, "pic_weight_quant_enable", input->PicWQEnable, bitstream);
        if (input->PicWQEnable) {
            len += u_v(2, "pic_weight_quant_data_index", input->PicWQDataIndex, bitstream);

            if (input->PicWQDataIndex == 1) {
                len += u_v(1, "reserved_bits", 0, bitstream); //M2331 2008-04
#if CHROMA_DELTA_QP
#endif
                len += u_v(2, "weighting_quant_param_index", input->WQParam, bitstream);
                len += u_v(2, "weighting_quant_model", input->WQModel, bitstream);

                //M2331 2008-04
                if ((input->WQParam == 1) || ((input->MBAdaptQuant) && (input->WQParam == 3))) {
                    for (i = 0; i < 6; i++) {
#if Wq_param_Clip
                       wq_param[UNDETAILED][i]=Clip3(1,255,wq_param[UNDETAILED][i]);
                       len += se_v("quant_param_delta_u", Clip3(-128,127,(wq_param[UNDETAILED][i] - wq_param_default[UNDETAILED][i])), bitstream);
#else
                        len += se_v("quant_param_delta_u", (wq_param[UNDETAILED][i] - wq_param_default[UNDETAILED][i]), bitstream);
#endif
                    }
                }
                if ((input->WQParam == 2) || ((input->MBAdaptQuant) && (input->WQParam == 3))) {
                    for (i = 0; i < 6; i++) {
#if Wq_param_Clip
						wq_param[DETAILED][i]=Clip3(1,255,wq_param[DETAILED][i]);
						len += se_v("quant_param_delta_d", Clip3(-128,127,(wq_param[DETAILED][i] - wq_param_default[DETAILED][i])), bitstream);
#else
                        len += se_v("quant_param_delta_d", (wq_param[DETAILED][i] - wq_param_default[DETAILED][i]), bitstream);
#endif
                    }
                }

                //M2331 2008-04
            }//input->PicWQDataIndex== 1
            else if (input->PicWQDataIndex == 2) {
                int x, y, sizeId, uiWqMSize;
                for (sizeId = 0; sizeId < 2; sizeId++) {
                    int i = 0;
                    uiWqMSize = min(1 << (sizeId + 2), 8);
                    for (y = 0; y < uiWqMSize; y++) {
                        for (x = 0; x < uiWqMSize; x++) {
#if Wq_param_Clip
							pic_user_wq_matrix[sizeId][i]=Clip3(1,255,pic_user_wq_matrix[sizeId][i]);
#endif
                            len += ue_v("weight_quant_coeff", pic_user_wq_matrix[sizeId][i++], bitstream);
                        }
                    }
                }
            } //input->PicWQDataIndex== 2
        }
    }
#if COUNT_BIT_OVERHEAD
    he->g_count_overhead_bit += len - bit_overhead;
    printf(" QM_overhead I : %8d\n", (len - bit_overhead));
#endif

#endif

    return len;
}


int SliceHeader(int slice_nr, int slice_qp)
{
    Bitstream *bitstream = currBitStream;
    int len = 0;
    int mb_row;                  //added by mz, 2008.04
    int slice_set_index = img->mb_data[img->current_mb_nr].slice_set_index; //added by mz, 2008.04
    int slice_header_flag = img->mb_data[img->current_mb_nr].slice_header_flag; //added by mz, 2008.04

    len += u_v(24, "start code prefix", 1, bitstream);
    len += u_v(8, "slice vertical position",
               (img->current_mb_nr / (img->width >> MIN_CU_SIZE_IN_BIT)) >> (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT),
               bitstream);

    if (img->height > (144 * (1 << input->g_uiMaxSizeInBit))) {   //add by wuzhongmou 200612
        len += u_v(3, "slice vertical position extension", slice_vertical_position_extension, bitstream);
    }

    if (img->height > (144 * (1 << input->g_uiMaxSizeInBit))) {   //added by mz, 2008.04
        mb_row = (slice_vertical_position_extension << 7) + img->current_mb_nr / (img->width >> MIN_CU_SIZE_IN_BIT);
    } else {
        mb_row = img->current_mb_nr / (img->width >> MIN_CU_SIZE_IN_BIT);
    }

    len += u_v(8, "slice horizontal position",
               ((img->current_mb_nr - mb_row * (img->width >> MIN_CU_SIZE_IN_BIT)) >> (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT)),
               bitstream);
    if (img->width > (255 * (1 << input->g_uiMaxSizeInBit))) {
        len += u_v(2, "slice horizontal position extension", slice_horizontal_position_extension, bitstream);
    }

    if (!input->fixed_picture_qp) {
#if MB_DQP
        len += u_v(1, "fixed_slice_qp", !input->useDQP, bitstream);
#else
        len += u_v(1, "fixed_slice_qp", 1, bitstream);
#endif

        len += u_v(7, "slice_qp", slice_qp, bitstream);
        img->qp = slice_qp;
    }

    if (input->sao_enable) {
        len += u_v(1, "sao_slice_flag_Y", img->slice_sao_on[0], bitstream);
        len += u_v(1, "sao_slice_flag_Cb", img->slice_sao_on[1], bitstream);
        len += u_v(1, "sao_slice_flag_Cr", img->slice_sao_on[2], bitstream);
    }

    return len;
}

/*
*************************************************************************
* Function: Check the settings of progressive_sequence, progressive_frame,
picture_structure, top_field_first and repeat_first_field
* Input:    Index of the input parameters
* Output:
* Return:
* Attention:
* Author: Xiaozhen ZHENG, 200811, rm52k
*************************************************************************
*/
int CheckInterlaceSetting(int idx)
{
    //0: OK
    //1: progressive_sequence
    //2: progressive_frame
    //3: picture_structure
    //4: top_field_first
    //5: repeat_first_field

    int status;
    status = InterlaceSetting[idx];

    switch (status) {
    case 0:
        return 1;
    case 1:
        printf("Invalid setting of progressive_sequence!\n");
        return 0;
    case 2:
        printf("Invalid setting of progressive_frame!\n");
        return 0;
    case 3:
        printf("Invalid setting of picture_structure!\n");
        return 0;
    case 4:
        printf("Invalid setting of top_field_first!\n");
        return 0;
    case 5:
        printf("Invalid setting of repeat_first_field!\n");
        return 0;
    default:
        printf("Invalid input!\n");
        return 0;
    }

    return 1;
}

