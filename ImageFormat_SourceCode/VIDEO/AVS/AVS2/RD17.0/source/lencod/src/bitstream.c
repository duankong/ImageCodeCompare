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
* Function:  Annex B Byte Stream format NAL Unit writing routines
*
*************************************************************************************
*/


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/transform.h"
#include "bitstream.h"
#define MAXHEADERSIZE 2000
#include "vlc.h"
#include "AEC.h"

#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif


CopyRight CopyRights, *cp = &CopyRights;
CameraParamters CameraParameter, *camera = &CameraParameter;



////////////////////////////////////////////////////////////////////////////////
#ifdef SVA_START_CODE_EMULATION

unsigned char bit[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
OutputStream  ORABS;
OutputStream *pORABS = &ORABS;

void OpenORABS(OutputStream *p, char *fname)
{
    p->f = fopen(fname, "wb");

    if (p->f == NULL) {
        printf("\nCan't open file %s", fname);
        exit(-1);
    }

    p->uPreBytes      = 0xffffffff;
    p->iBytePosition    = 0;
    p->iBitOffset     = 0;
    p->iNumOfStuffBits    = 0;
    p->iBitsCount     = 0;
}
void CloseORABS(OutputStream *p)
{
    if (p->iBitOffset) {
        fwrite(p->buf, 1, p->iBytePosition + 1, p->f);
    } else {
        fwrite(p->buf, 1, p->iBytePosition  , p->f);
    }

    fclose(p->f);
}
void FlushORABS(OutputStream *p)
{
    fflush(p->f);
}


int write_1_bit(OutputStream *p, int b)
{
    int i;

    if (p->iBytePosition == SVA_STREAM_BUF_SIZE) {
        i = (int)fwrite(p->buf, 1, SVA_STREAM_BUF_SIZE, p->f);

        if (i != SVA_STREAM_BUF_SIZE) {
            printf("Fatal: write file error, exit (-1)\n");
            exit(-1);
        }

        p->iBytePosition  = 0;
        p->iBitOffset   = 0;
    }

    p->uPreBytes <<= 1;

    if (b) {
        p->buf[p->iBytePosition] |= bit[p->iBitOffset];
        p->uPreBytes |= 1;
    } else {
        p->buf[p->iBytePosition] &= (~bit[p->iBitOffset]);
    }

    p->iBitOffset++;

    if (p->iBitOffset == 8) {
        p->iBitOffset = 0;
        p->iBytePosition++;
    }

    p->iBitsCount++;
    return 0;
}

int write_n_bit(OutputStream *p, int b, int n)
{

    if (n > 30) {
        return 1;
    }


    while (n > 0) {
        write_1_bit(p, b & (0x01 << (n - 1)));
        n--;
    }

    return 0;
}


/*
*************************************************************************
* Function: one bit "1" is added to the end of stream, then some bits "0" are added to bytealigned position.
* Input:
* Output:
* Return:
* Attention:
* Author: Yulj 2004.07.16
*************************************************************************
*/
int write_align_stuff(OutputStream *p)
{
    unsigned char c;
    int len;    //bit,rm52k_r2

    c = 0xff << (8 - p->iBitOffset);
    p->buf[p->iBytePosition] = (c & p->buf[p->iBytePosition]) | (0x80 >> (p->iBitOffset));
    p->iBitsCount += 8 - p->iBitOffset;
    len   = 8 - p->iBitOffset;       //bit,rm52k_r2
    p->uPreBytes  = (p->uPreBytes << (8 - p->iBitOffset)) & c;
    p->iNumOfStuffBits  += 8 - p->iBitOffset;
    p->iBitOffset = 0;
    p->iBytePosition++;
    return len;   //bit,rm52k_r2
}
//---end

int write_start_code(OutputStream *p, unsigned char code)
{
    int i;

    if (p->iBytePosition >= SVA_STREAM_BUF_SIZE - 4 && p->iBytePosition > 0) {
        i = (int)fwrite(p->buf, 1, p->iBytePosition, p->f);

        if (i != p->iBytePosition) {
            printf("\nWrite file error");
            exit(-1);
        }
        p->iBytePosition  = 0;
        p->iBitOffset   = 0;
    }

    p->buf[p->iBytePosition  ] = 0;
    p->buf[p->iBytePosition + 1] = 0;
    p->buf[p->iBytePosition + 2] = 1;
    p->buf[p->iBytePosition + 3] = code;
    p->iBytePosition += 4;
    p->iBitsCount += 32;
    p->uPreBytes  = (unsigned int) code + 256;

    return 0;
}

/*
*************************************************************************
* Function:Open the output file for the bytestream
* Input: The filename of the file to be opened
* Output:
* Return: none.Function terminates the program in case of an error
* Attention:
*************************************************************************
*/


void OpenBitStreamFile(char *Filename)
{
    OpenORABS(pORABS, Filename);
}
void CloseBitStreamFile()
{
    CloseORABS(pORABS);
}

/*
*************************************************************************
* Function:Write video edit code
* Input:
* Output:
* Return: 32bit for video edit code
* Attention:
*************************************************************************
*/

int WriteVideoEditCode()
{
    Bitstream *bitstream;
    unsigned char VideoEditCode[32];
    int  bitscount = 0;

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("Seuqence Header: bitstream");
    }

    bitstream->streamBuffer = VideoEditCode;
    bitstream->bits_to_go = 8;

    bitscount = u_v(32, "video_edit_code", 0x1B7, bitstream);

    write_start_code(pORABS, 0xb7);

    free(bitstream);

    return bitscount;
}



/*
*************************************************************************
* Function:Write sequence header information
* Input:
* Output:
* Return: sequence header length, including stuffing bits
* Attention:
*************************************************************************
*/

int WriteSequenceHeader()
{
    Bitstream *bitstream;
    unsigned char SequenceHeader[MAXHEADERSIZE];
    int  bitscount = 0;
    int  i, j, k;

#if (FREQUENCY_WEIGHTING_QUANTIZATION && COUNT_BIT_OVERHEAD)
    int bit_overhead;
    he->g_count_overhead_bit = 0;
#endif

    printf("Sequence Header \n");

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("Sequence Header: bitstream");
    }

    bitstream->streamBuffer = SequenceHeader;
    bitstream->bits_to_go = 8;

    input->display_horizontal_size = img->width;
    input->display_vertical_size   = img->height;
    input->sample_precision = 1;

    input->aspect_ratio_information = 1;
    input->bit_rate_lower = 0;
    input->bit_rate_upper = 0;

    //Set BitRate to half level's maximum bitrate. M3784
    switch (input->level_id) {
    case 0x10:
        input->bit_rate_lower = 1875;
        break;
    case 0x12:
        input->bit_rate_lower = 2500;
        break;
    case 0x14:
        input->bit_rate_lower = 3125;
        break;
    case 0x20:
        input->bit_rate_lower = 12500;
        break;
    case 0x22:
        input->bit_rate_lower = 25000;
        break;
    case 0x40:
        input->bit_rate_lower = 31250;
        break;
    case 0x42:
        input->bit_rate_lower = 125000;
        break;
    case 0x44:
        input->bit_rate_lower = 50000;
        break;
    case 0x46:
        input->bit_rate_lower = 200000;
        break;
    case 0x48:
        input->bit_rate_lower = 75000;
        break;
    case 0x4A:
        input->bit_rate_lower = 37856;
        input->bit_rate_upper = 1;
        break;
    case 0x50:
        input->bit_rate_lower = 50000;
        break;
    case 0x52:
        input->bit_rate_lower = 200000;
        break;
    case 0x54:
        input->bit_rate_lower = 75000;
        break;
    case 0x56:
        input->bit_rate_lower = 37856;
        input->bit_rate_upper = 1;
        break;
    case 0x58:
        input->bit_rate_lower = 150000;
        break;
    case 0x5A:
        input->bit_rate_lower = 75712;
        input->bit_rate_upper = 2;
        break;
    case 0x60:
        input->bit_rate_lower = 75000;
        break;
    case 0x62:
        input->bit_rate_lower = 37856;
        input->bit_rate_upper = 1;
        break;
    case 0x64:
        input->bit_rate_lower = 150000;
        break;
    case 0x66:
        input->bit_rate_lower = 75712;
        input->bit_rate_upper = 2;
        break;
    case 0x68:
        input->bit_rate_lower = 37856;
        input->bit_rate_upper = 1;
        break;
    case 0x6A:
        input->bit_rate_lower = 213568;
        input->bit_rate_upper = 3;
        break;
    default:
        input->bit_rate_lower = 1;
    }

    bitscount += u_v(32, "sequence start code", 0x1b0, bitstream);
    bitscount += u_v(8, "profile_id", input->profile_id, bitstream);
    bitscount += u_v(8, "level_id", input->level_id, bitstream);
    bitscount += u_v(1, "progressive_sequence", input->progressive_sequence, bitstream);
#if INTERLACE_CODING
    bitscount += u_v(1, "field_coded_sequence", img->is_field_sequence, bitstream);
#endif

    bitscount += u_v(14, "horizontal_size", (img->width - img->auto_crop_right), bitstream);
    bitscount += u_v(14, "vertical_size", (img->height - img->auto_crop_bottom), bitstream);

    bitscount += u_v(2, "chroma format", input->chroma_format, bitstream);
    if (input->profile_id == BASELINE10_PROFILE) { // 10bit profile
        bitscount += u_v(3, "sample precision", ((input->input_sample_bit_depth - 6) / 2), bitstream);
        bitscount += u_v(3, "encoding precision", ((input->sample_bit_depth - 6) / 2), bitstream);
    } else { // other profile
        bitscount += u_v(3, "sample precision", input->sample_precision, bitstream);
    }
    bitscount += u_v(4, "aspect ratio information", input->aspect_ratio_information, bitstream);
    bitscount += u_v(4, "frame rate code", input->frame_rate_code, bitstream);

    bitscount += u_v(18, "bit rate lower", input->bit_rate_lower, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(12, "bit rate upper", input->bit_rate_upper, bitstream);
    bitscount += u_v(1, "low delay", input->low_delay, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

#if M3480_TEMPORAL_SCALABLE
    bitscount += u_v(1, "temporal_id exist flag", he->temporal_id_exist_flag, bitstream);
#endif
    bitscount += u_v(18, "bbv buffer size", input->bbv_buffer_size, bitstream);
    bitscount += u_v(3, "Largest Coding Block Size", input->g_uiMaxSizeInBit, bitstream);

#if FREQUENCY_WEIGHTING_QUANTIZATION

#if COUNT_BIT_OVERHEAD
    bit_overhead = bitscount;
#endif

    bitscount += u_v(1, "weight_quant_enable", input->WQEnable, bitstream);
    if (input->WQEnable) {
        bitscount += u_v(1, "load_seq_weight_quant_data_flag", input->SeqWQM, bitstream);
        if (input->SeqWQM) {
            int x, y, sizeId, uiWqMSize;
            for (sizeId = 0; sizeId < 2; sizeId++) {
                uiWqMSize = min(1 << (sizeId + 2), 8);
                for (y = 0; y < uiWqMSize; y++) {
                    for (x = 0; x < uiWqMSize; x++) {
                        bitscount += ue_v("weight_quant_coeff", seq_wq_matrix[sizeId][y * uiWqMSize + x], bitstream);
                    }
                }
            }
        }
    }


#if COUNT_BIT_OVERHEAD
    he->g_count_overhead_bit += bitscount - bit_overhead;
#endif
#endif

    bitscount += u_v(1, "background_picture_disable", input->bg_enable ^ 0x01, bitstream);






    bitscount += u_v(1, "mhpskip_enable", input->b_mhpskip_enabled, bitstream);

    bitscount += u_v(1, "dhp enabled", input->dhp_enabled, bitstream);
    bitscount += u_v(1, "wsm enabled", input->wsm_enabled, bitstream);

    bitscount += u_v(1,  "Asymmetric Motion Partitions", input->InterSearchAMP, bitstream);

    bitscount += u_v(1, "useNSQT", input->useNSQT, bitstream);

    bitscount += u_v(1, "useNSIP", input->useSDIP, bitstream);

    bitscount += u_v(1, "secT enabled", input->b_secT_enabled, bitstream);

    bitscount += u_v(1,  "SAO Enable Flag", input->sao_enable, bitstream);
    bitscount += u_v(1,  "ALF Enable Flag", input->alf_enable, bitstream);

    bitscount += u_v(1, "pmvr enabled", input->b_pmvr_enabled, bitstream);

    bitscount += u_v(1, "marker bit", 1, bitstream);


    bitscount += u_v(6, "num_of_RPS", he->gop_size_all, bitstream);
    for (i = 0; i < he->gop_size_all; i++) {
        bitscount += u_v(1, "refered by others ", he->cfg_ref_all[i].referd_by_others, bitstream);
        bitscount += u_v(3, "num of reference picture", he->cfg_ref_all[i].num_of_ref, bitstream);

        for (j = 0; j < he->cfg_ref_all[i].num_of_ref; j++) {
            bitscount += u_v(6, "delta COI of ref pic", he->cfg_ref_all[i].ref_pic[j], bitstream);
        }
        bitscount += u_v(3, "num of removed picture", he->cfg_ref_all[i].num_to_remove, bitstream);
        for (j = 0; j < he->cfg_ref_all[i].num_to_remove; j++) {
            bitscount += u_v(6, "delta COI of removed pic", he->cfg_ref_all[i].remove_pic[j], bitstream);
        }
        bitscount += u_v(1, "marker bit", 1, bitstream);

    }

    if (input->low_delay == 0) {
        bitscount += u_v(5, "picture_reorder_delay", he->picture_reorder_delay, bitstream);
    }

    bitscount += u_v(1, "Cross Loop Filter Flag", input->crossSliceLoopFilter, bitstream);

    bitscount += u_v(2, "reserved bits", 0, bitstream);

    k = bitscount >> 3;
    j = bitscount % 8;

    write_start_code(pORABS, 0xb0);

    for (i = 4; i < k; i++) {
        write_n_bit(pORABS, SequenceHeader[i], 8);
    }

    if (j != 0) {
        write_n_bit(pORABS,/*SequenceHeader[k]*/bitstream->byte_buf, j);
    }

    bitscount += write_align_stuff(pORABS);    //bit,rm52k_r2

    free(bitstream);

    return bitscount;
}

int WriteSequenceEnd()
{
    write_start_code(pORABS, 0xb1);
    return 32;
}

/*
*************************************************************************
* Function:Write sequence display extension information
* Input:
* Output:
* Return: sequence display extension information lenght
* Attention:
*************************************************************************
*/


int WriteSequenceDisplayExtension()
{
    Bitstream *bitstream;
    unsigned char SequenceDisplayExtension[MAXHEADERSIZE];
    int  bitscount = 0;

    int  i, j, k;

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("Sequence Display Extension: bitstream");
    }

    input->video_format = 1;
    input->video_range = 1;
    //input->display_horizontal_size = 1920;
    //input->display_vertical_size = 1280;

    bitstream->streamBuffer = SequenceDisplayExtension;
    bitstream->bits_to_go = 8;

    bitscount += u_v(32, "sequence display extension start code", 0x1b5, bitstream);
    bitscount += u_v(4, "extension id", 2, bitstream);
    bitscount += u_v(3, "video format", input->video_format, bitstream);
    bitscount += u_v(1, "video range", input->video_range, bitstream);
    bitscount += u_v(1, "color description", input->color_description, bitstream);

    if (input->color_description) {
        bitscount += u_v(8, "color primaries", input->color_primaries, bitstream);
        bitscount += u_v(8, "transfer characteristics", input->transfer_characteristics, bitstream);
        bitscount += u_v(8, "matrix coefficients", input->matrix_coefficients, bitstream);
    }

    bitscount += u_v(14, "display horizontal size", input->display_horizontal_size, bitstream);
    //xyji 12.23
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(14, "display vertical size", input->display_vertical_size, bitstream);
    //xyji 12.23
    bitscount += u_v(1, "3D mode", input->TD_mode, bitstream);

    if (input->TD_mode) {

        bitscount += u_v(8, "3D packing mode", input->view_packing_mode, bitstream);
        bitscount += u_v(1, "view reverse", input->view_reverse, bitstream);

    }

    k = bitscount >> 3;
    j = bitscount % 8;

    write_start_code(pORABS, 0xb5);

    for (i = 4; i < k; i++) {
        write_n_bit(pORABS, SequenceDisplayExtension[i], 8);
    }

#if WRITENBIT_FIX
    write_n_bit(pORABS, bitstream->byte_buf, j);
#else
    write_n_bit(pORABS, SequenceDisplayExtension[k], j);
#endif
    bitscount += write_align_stuff(pORABS);

    free(bitstream);

    return bitscount;
}

/*
*************************************************************************
* Function:Write copyright extension information
* Input:
* Output:
* Return: copyright extension information lenght
* Attention:
*************************************************************************
*/
int WriteCopyrightExtension()
{
    Bitstream *bitstream;
    unsigned char CopyrightExtension[MAXHEADERSIZE];
    int  bitscount = 0;
    int  i, j, k;

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("Copyright Extension: bitstream");
    }

    bitstream->streamBuffer = CopyrightExtension;
    bitstream->bits_to_go = 8;

    bitscount += u_v(32, "copyright extension start code", 0x1b5, bitstream);
    bitscount += u_v(4, "extension id", 4, bitstream);
    bitscount += u_v(1, "copyright flag", cp->copyright_flag, bitstream);
    bitscount += u_v(8, "copyright id", cp->copyright_id, bitstream);
    bitscount += u_v(1, "original or copy", cp->original_or_copy, bitstream);

    bitscount += u_v(7, "reserved_bits", 0, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(20, "copyright number 1", cp->copyright_number, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(22, "copyright number 2", cp->copyright_number, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(22, "copyright number 3", cp->copyright_number, bitstream);

    k = bitscount >> 3;
    j = bitscount % 8;

    write_start_code(pORABS, 0xb5);

    for (i = 4; i < k; i++) {
        write_n_bit(pORABS, CopyrightExtension[i], 8);
    }

#if WRITENBIT_FIX
    write_n_bit(pORABS, bitstream->byte_buf, j);
#else
    write_n_bit(pORABS, CopyrightExtension[k], j);
#endif
    bitscount += write_align_stuff(pORABS);

    free(bitstream);

    return bitscount;
}


/*
*************************************************************************
* Function:Write camera parameter extension information
* Input:
* Output:
* Return: camera parameter  extension information lenght
* Attention:
*************************************************************************
*/
int WriteCameraParametersExtension()
{
    Bitstream *bitstream;
    unsigned char CameraParametersExtension[MAXHEADERSIZE];
    int  bitscount = 0;
    int  i, j, k;

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("Camera Parameters Extension: bitstream");
    }

    bitstream->streamBuffer = CameraParametersExtension;
    bitstream->bits_to_go = 8;

    bitscount += u_v(32, "camera parameters extension start code", 0x1b5, bitstream);
    bitscount += u_v(4, "extension id", 11, bitstream);
    bitscount += u_v(1, "reserved_bits", 0, bitstream);
    bitscount += u_v(7, "camera id", camera->camera_id, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "height_of_image_device", camera->height_of_image_device, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "focal_length", camera->focal_length, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "f_number", camera->f_number, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "vertical_angle_of_view", camera->vertical_angle_of_view, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_x_upper", camera->camera_direction_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "camera_position_x_lower", camera->camera_direction_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_y_upper", camera->camera_direction_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_y_lower", camera->camera_direction_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_z_upper", camera->camera_direction_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "camera_position_z_lower", camera->camera_direction_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "camera_direction_x", camera->camera_direction_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "camera_direction_y", camera->camera_direction_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "camera_direction_z", camera->camera_direction_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "image_plane_vertical_x", camera->image_plane_vertical_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "image_plane_vertical_y", camera->image_plane_vertical_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "image_plane_vertical_z", camera->image_plane_vertical_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(32, "reserved_bits", 0, bitstream);

    k = bitscount >> 3;
    j = bitscount % 8;

    write_start_code(pORABS, 0xb5);

    for (i = 4; i < k; i++) {
        write_n_bit(pORABS, CameraParametersExtension[i], 8);
    }

#if WRITENBIT_FIX
    write_n_bit(pORABS, bitstream->byte_buf, j);
#else
    write_n_bit(pORABS, CameraParametersExtension[k], j);
#endif
    bitscount += write_align_stuff(pORABS);

    free(bitstream);

    return bitscount;
}
#if M3480_TEMPORAL_SCALABLE
int WriteScalableExtension()
{
    Bitstream *bitstream;
    unsigned char TemporalScalableExtension[MAXHEADERSIZE];
    int  bitscount = 0;
    int  i, j, k;

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("Temporal Scalable Extension: bitstream");
    }

    bitstream->streamBuffer = TemporalScalableExtension;
    bitstream->bits_to_go = 8;

    bitscount += u_v(32, "scalable extension start code", 0x1b5, bitstream);
    bitscount += u_v(4, "extension id", 3, bitstream);
    bitscount += u_v(3, "max temporal level", TEMPORAL_MAXLEVEL - 1, bitstream);

    for (i = 0; i < TEMPORAL_MAXLEVEL - 1; i ++) {
        bitscount += u_v(4, "fps code per temporal level", 1, bitstream);
        bitscount += u_v(18, "bit rate lower", 0, bitstream);
        bitscount += u_v(1, "marker bit", 1, bitstream);
        bitscount += u_v(12, "bit rate upper", 1, bitstream);
    }

    k = bitscount >> 3;
    j = bitscount % 8;

    write_start_code(pORABS, 0xb5);

    for (i = 4; i < k; i++) {
        write_n_bit(pORABS, TemporalScalableExtension[i], 8);
    }

#if WRITENBIT_FIX
    write_n_bit(pORABS, bitstream->byte_buf, j);
#else
    write_n_bit(pORABS, TemporalScalableExtension[k], j);
#endif
    bitscount += write_align_stuff(pORABS);

    free(bitstream);

    return bitscount;
}
#endif

#if AVS2_HDR_HLS
int WriteMasteringDisplayContentMetadataExtension()
{
    Bitstream *bitstream;
    unsigned char MasteringDisplayContentMetadatExtension[MAXHEADERSIZE];
    int  bitscount = 0;
    int  i, j, k;

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("Mastering Display and Content Metadata Extension: bitstream");
    }

    bitstream->streamBuffer = MasteringDisplayContentMetadatExtension;
    bitstream->bits_to_go = 8;

    // The user can change the following parameters according to the particular use case
    input->display_primaries_x0 = 0;
    input->display_primaries_y0 = 0;
    input->display_primaries_x1 = 0;
    input->display_primaries_y1 = 0;
    input->display_primaries_x2 = 0;
    input->display_primaries_y2 = 0;
    input->white_point_x = 0;
    input->white_point_y = 0;
    input->max_display_mastering_luminance = 1000;
    input->min_display_mastering_luminance = 10;
    input->maximum_content_light_level = 4000;
    input->maximum_frame_average_light_level = 1000;

    bitscount += u_v(32, "sequence display extension start code", 0x1b5, bitstream);
    bitscount += u_v(4, "extension id", 10, bitstream);

    bitscount += u_v(16, "display_primaries_x[0]", input->display_primaries_x0, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "display_primaries_y[0]", input->display_primaries_y0, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "display_primaries_x[1]", input->display_primaries_x1, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "display_primaries_y[1]", input->display_primaries_y1, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "display_primaries_x[2]", input->display_primaries_x2, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "display_primaries_y[2]", input->display_primaries_y2, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "white_point_x", input->white_point_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "white_point_y", input->white_point_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "max_display_mastering_luminance", input->max_display_mastering_luminance, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "min_display_mastering_luminance", input->min_display_mastering_luminance, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "maximum_content_light_level", input->maximum_content_light_level, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "maximum_frame_average_light_level", input->maximum_frame_average_light_level, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "reserved bit", 0, bitstream);


    k = bitscount >> 3;
    j = bitscount % 8;

    write_start_code(pORABS, 0xb5);

    for (i = 4; i < k; i++) {
        write_n_bit(pORABS, MasteringDisplayContentMetadatExtension[i], 8);
    }

#if WRITENBIT_FIX
    write_n_bit(pORABS, bitstream->byte_buf, j);
#else
    write_n_bit(pORABS, MasteringDisplayContentMetadatExtension[k], j);
#endif
    bitscount += write_align_stuff(pORABS);

    free(bitstream);

    return bitscount;
}
#endif

/*
*************************************************************************
* Function:Write user data
* Input:
* Output:
* Return: user data length
* Attention:
*************************************************************************
*/

int WriteUserData(char *userdata)
{
    Bitstream *bitstream;
    unsigned char UserData[MAXHEADERSIZE];
    int  bitscount = 0;

    if ((bitstream = calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("User data: bitstream");
    }

    bitstream->streamBuffer = UserData;
    bitstream->bits_to_go = 8;

    bitscount += u_v(32, "user data start code", 0x1b2, bitstream);
    write_start_code(pORABS, 0xb2);

    while (*userdata) {
        write_n_bit(pORABS, *userdata, 8);
        bitscount += u_v(8, "user data", *userdata++, bitstream);
    }

    bitscount += write_align_stuff(pORABS);
    free(bitstream);

    return bitscount;
}

/*
*************************************************************************
* Function:Write bit steam to file
* Input:
* Output:
* Return: none
* Attention:
*************************************************************************
*/

void WriteBitstreamtoFile()
{
    int n, i;
    n = currBitStream->byte_pos;

    for (i = 0; i < n; i++) {
        if (currBitStream->streamBuffer[i] == 0 && currBitStream->streamBuffer[i + 1] == 0 &&
            currBitStream->streamBuffer[i + 2] == 1) {
            write_start_code(pORABS, currBitStream->streamBuffer[i + 3]);
            i = i + 4;
        }

        write_n_bit(pORABS, currBitStream->streamBuffer[i], 8);
    }

    if (img->type == INTRA_IMG && img->number == 0) {
        hc->seq_header = stat->bit_use_header[3];
    }

    stat->bit_ctr += 8 * n;
    stat->bit_ctr += stat->bit_use_header[3];  //rm52k
    stat->bit_use_header[3] = 0;               //rm52k
}
#endif


/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
int start_sequence()
{
    int len = 0;
    char id_string[255] = "AVS test stream";

    if (img->number == 0) {
        OpenBitStreamFile(input->outfile);
    }

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    if (input->WQEnable) {
        InitSeqQuantParam();
    }
#endif

    len = WriteSequenceHeader();

    if (img->number == 0) {
        len += WriteSequenceDisplayExtension();
        len += WriteCopyrightExtension();
        len += WriteCameraParametersExtension();
#if M3480_TEMPORAL_SCALABLE
        len += WriteScalableExtension();
#endif

#if AVS2_HDR_HLS
        if (input->hdr_metadata) {
            len += WriteMasteringDisplayContentMetadataExtension();
        }
#endif

        if (strlen(id_string) > 1) {
            len += WriteUserData(id_string);
        }
    }

    return len;
}
/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:Mainly flushing of everything Add termination symbol, etc.
*************************************************************************
*/
int terminate_sequence()
{
    int len;
    len = WriteSequenceEnd();

    if (img->EncodeEnd_flag == 1) {
        CloseBitStreamFile();
    }
    switch (img->type) {
    case INTRA_IMG:
        stat->bit_use_stuffingBits[0] += len;
        break;
    case B_IMG:
        stat->bit_use_stuffingBits[2] += len;
        break;
    default:
        stat->bit_use_stuffingBits[1] += len;
        break;
    }
    printf("Sequence End\n");
    return len;
}

#if PicExtensionData
int picture_copyright_extension(Bitstream *bitstream)
{
    int  bitscount = 0;

    bitscount += u_v(32, "Extension start code", 0x1B5, bitstream);
    bitscount += u_v(4, "picture copyright extension id", 4, bitstream);

    bitscount += u_v(1, "copyright flag", cp->copyright_flag, bitstream);
    bitscount += u_v(8, "copyright id", cp->copyright_id, bitstream);
    bitscount += u_v(1, "original or copy", cp->original_or_copy, bitstream);

    bitscount += u_v(7, "reserved_bits", 0, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(20, "copyright number 1", cp->copyright_number, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(22, "copyright number 2", cp->copyright_number, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(22, "copyright number 3", cp->copyright_number, bitstream);

    if (bitstream->bits_to_go != 8) {   //   hzjia 2004-08-20
        bitstream->byte_buf <<= bitstream->bits_to_go;
        bitstream->byte_buf |= (1 << (bitstream->bits_to_go - 1));

        bitstream->streamBuffer[bitstream->byte_pos++] = bitstream->byte_buf;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    } else { // cjw 20060321
        bitstream->streamBuffer[bitstream->byte_pos++] = 0x80;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    }

    return bitscount;
}

int picture_cameraparameters_extension(Bitstream *bitstream)
{
    int  bitscount = 0;

    bitscount += u_v(32, "Extension start code", 0x1B5, bitstream);
    bitscount += u_v(4, "picture copyright extension id", 11, bitstream);

    bitscount += u_v(1, "reserved_bits", 0, bitstream);
    bitscount += u_v(7, "camera id", camera->camera_id, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "height_of_image_device", camera->height_of_image_device, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "focal_length", camera->focal_length, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "f_number", camera->f_number, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "vertical_angle_of_view", camera->vertical_angle_of_view, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_x_upper", camera->camera_direction_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "camera_position_x_lower", camera->camera_direction_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_y_upper", camera->camera_direction_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_y_lower", camera->camera_direction_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(16, "camera_position_z_upper", camera->camera_direction_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);
    bitscount += u_v(16, "camera_position_z_lower", camera->camera_direction_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "camera_direction_x", camera->camera_direction_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "camera_direction_y", camera->camera_direction_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "camera_direction_z", camera->camera_direction_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "image_plane_vertical_x", camera->image_plane_vertical_x, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "image_plane_vertical_y", camera->image_plane_vertical_y, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(22, "image_plane_vertical_z", camera->image_plane_vertical_z, bitstream);
    bitscount += u_v(1, "marker bit", 1, bitstream);

    bitscount += u_v(32, "reserved_bits", 0, bitstream);

    if (bitstream->bits_to_go != 8) {   //   hzjia 2004-08-20
        bitstream->byte_buf <<= bitstream->bits_to_go;
        bitstream->byte_buf |= (1 << (bitstream->bits_to_go - 1));

        bitstream->streamBuffer[bitstream->byte_pos++] = bitstream->byte_buf;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    } else { // cjw 20060321
        bitstream->streamBuffer[bitstream->byte_pos++] = 0x80;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    }

    return bitscount;
}

int picture_display_extension(Bitstream *bitstream)
{
    int  bitscount = 0;
    int  i;

    int numberofFrameCentreOffsets = 0;

    if (input->progressive_sequence == 1) {
        if (input->repeat_first_field == 1) {
            if (input->top_field_first == 1) {
                numberofFrameCentreOffsets = 3;
            } else {
                numberofFrameCentreOffsets = 2;
            }
        } else {
            numberofFrameCentreOffsets = 1;
        }
    } else {
        if (img->picture_structure == 0) {
            numberofFrameCentreOffsets = 1;
        } else {
            if (input->repeat_first_field = 1) {
                numberofFrameCentreOffsets = 3;
            } else {
                numberofFrameCentreOffsets = 2;
            }
        }
    }

    bitscount += u_v(32, "Extension start code", 0x1B5, bitstream);
    bitscount += u_v(4, "picture copyright extension id", 7, bitstream);

    for (i = 0; i < numberofFrameCentreOffsets; i++) {
        bitscount += u_v(16, "picture_centre_horizontal_offset", 16 * (i + 1), bitstream);
        bitscount += u_v(1, "marker bit", 1, bitstream);

        bitscount += u_v(16, "picture_centre_vertical_offset", 16 * (i + 1), bitstream);
        bitscount += u_v(1, "marker bit", 1, bitstream);
    }

    if (bitstream->bits_to_go != 8) {   //   hzjia 2004-08-20
        bitstream->byte_buf <<= bitstream->bits_to_go;
        bitstream->byte_buf |= (1 << (bitstream->bits_to_go - 1));

        bitstream->streamBuffer[bitstream->byte_pos++] = bitstream->byte_buf;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    } else { // cjw 20060321
        bitstream->streamBuffer[bitstream->byte_pos++] = 0x80;
        bitstream->bits_to_go = 8;
        bitstream->byte_buf = 0;
    }

    return bitscount;
}
#endif
