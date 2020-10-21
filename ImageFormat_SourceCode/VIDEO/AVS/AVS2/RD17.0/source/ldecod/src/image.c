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
* File name: image.c
* Function: Decode a Slice
*
*************************************************************************************
*/


#include "../../lcommon/inc/contributors.h"

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <sys/timeb.h>
#include <string.h>
#include <assert.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "header.h"
#include "annexb.h"
#include "../../lcommon/inc/memalloc.h"
#include "AEC.h"
#include "biaridecod.h"
#include "../../lcommon/inc/loop-filter.h"
#include "../../lcommon/inc/inter-prediction.h"
#include "../../lcommon/inc/ComAdaptiveLoopFilter.h"
#include "../../lcommon/inc/md5.h"
#include "../../ldecod/inc/DecAdaptiveLoopFilter.h"
void Copy_frame_for_ALF();
#if ROI_M3264
#include "pos_info.h"
#endif

// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif

void DecideMvRange(); // rm52k

/* 08.16.2007--for user data after pic header */
unsigned char *temp_slice_buf;
int first_slice_length;
int first_slice_startpos;
/* 08.16.2007--for user data after pic header */
extern void readParaSAO_one_SMB(int smb_index, int mb_y, int mb_x, int smb_mb_height, int smb_mb_width,
                                int *slice_sao_on, SAOBlkParam *saoBlkParam, SAOBlkParam *rec_saoBlkParam);
extern StatBits *StatBitsPtr;


extern unsigned int   MD5val[4];
extern char           MD5str[33];

#if SEQ_CHANGE_CHECKER
byte *seq_checker_buf = NULL;
int seq_checker_length = 0;
#endif
/*
*************************************************************************
* Function:decodes one I- or P-frame
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int decode_one_frame(SNRParameters *snr)
{
    int current_header;
    int N8_SizeScale;

    time_t ltime1;                  // for time measurement
    time_t ltime2;

#ifdef WIN32
    struct _timeb tstruct1;
    struct _timeb tstruct2;
#else
    struct timeb tstruct1;
    struct timeb tstruct2;
#endif

    double framerate[8] = {24000.0 / 1001.0, 24.0, 25.0, 30000.0 / 1001.0, 30.0, 50.0, 60000.0 / 1001.0, 60.0};

#ifdef WIN32
    _ftime(&tstruct1);              // start time ms
#else
    ftime(&tstruct1);               // start time ms
#endif
    time(&ltime1);                  // start time s

    img->current_mb_nr =
        -4711; // initialized to an impossible value for debugging -- correct value is taken from slice header

    current_header = Header();

    DecideMvRange();  //rm52k

    if (current_header == EOS) {
        return EOS;
    }


    {
        N8_SizeScale = 1;

        if (hd->horizontal_size % (MIN_CU_SIZE * N8_SizeScale) != 0) {
            img->auto_crop_right = (MIN_CU_SIZE * N8_SizeScale) - (hd->horizontal_size % (MIN_CU_SIZE * N8_SizeScale));
        } else {
            img->auto_crop_right = 0;
        }
#if !INTERLACE_CODING
        if (hd->progressive_sequence)
#endif
        {
            if (hd->vertical_size % (MIN_CU_SIZE * N8_SizeScale) != 0) {
                img->auto_crop_bottom = (MIN_CU_SIZE * N8_SizeScale) - (hd->vertical_size % (MIN_CU_SIZE * N8_SizeScale));
            } else {
                img->auto_crop_bottom = 0;
            }
        }

        // Reinit parameters (NOTE: need to do before init_frame //
        img->width          = (hd->horizontal_size + img->auto_crop_right);
        img->height         = (hd->vertical_size + img->auto_crop_bottom);
        img->width_cr       = (img->width >> 1);

        if (input->chroma_format == 1) {
            img->height_cr      = (img->height >> 1);
        }

        img->PicWidthInMbs  = img->width / MIN_CU_SIZE;
        img->PicHeightInMbs = img->height / MIN_CU_SIZE;
        img->PicSizeInMbs   = img->PicWidthInMbs * img->PicHeightInMbs;
        img->max_mb_nr      = (img->width * img->height) / (MIN_CU_SIZE * MIN_CU_SIZE);
    }

    if (img->new_sequence_flag && img->sequence_end_flag) {
        hd->end_SeqTr = img->tr;
        img->sequence_end_flag = 0;
    }
    if (img->new_sequence_flag) {
        hd->next_IDRtr = img->tr;
        hd->next_IDRcoi = img->coding_order;
        img->new_sequence_flag = 0;
    }

    // allocate memory for frame buffers

    if (img->number == 0) {
        init_global_buffers();
    }

    img->current_mb_nr = 0;
    init_frame();

    img->types = img->type;   // jlzheng 7.15

    if (img->type != B_IMG) {
        hd->pre_img_type = img->type;
        hd->pre_img_types = img->types;
    }

    picture_data();

    if (img->typeb == BACKGROUND_IMG && hd->background_picture_enable) {
        int l;
        for (l = 0; l < img->height; l++) {
            memcpy(hd->background_frame[0][l], hc->imgY[l], img->width * sizeof(byte));
        }
        for (l = 0; l < img->height_cr; l++) {
            memcpy(hd->background_frame[1][l], hc->imgUV[0][l], img->width_cr * sizeof(byte));
            memcpy(hd->background_frame[2][l], hc->imgUV[1][l], img->width_cr * sizeof(byte));
        }
    }

    if (img->typeb == BACKGROUND_IMG && hd->background_picture_output_flag == 0) {
        hd->background_number++;
    }


    if (img->type == B_IMG) {
        fref[0]->imgtr_fwRefDistance = hd->trtmp;
    }

    img->height = (hd->vertical_size + img->auto_crop_bottom);
    img->height_cr =  img->height / (input->chroma_format == 1 ? 2 : 1);
    img->PicWidthInMbs  = img->width / MIN_CU_SIZE;
    img->PicHeightInMbs = img->height / MIN_CU_SIZE;
    img->PicSizeInMbs   = img->PicWidthInMbs * img->PicHeightInMbs;


#ifdef WIN32
    _ftime(&tstruct2);    // end time ms
#else
    ftime(&tstruct2);     // end time ms
#endif

    time(&ltime2);                                  // end time sec
    hd->tmp_time = (int)((ltime2 * 1000 + tstruct2.millitm) - (ltime1 * 1000 + tstruct1.millitm));
    hc->tot_time = hc->tot_time + hd->tmp_time;

    //rm52k_r2
    StatBitsPtr->curr_frame_bits = StatBitsPtr->curr_frame_bits * 8 + StatBitsPtr->emulate_bits -
                                   StatBitsPtr->last_unit_bits;
    StatBitsPtr->bitrate += StatBitsPtr->curr_frame_bits;
    StatBitsPtr->coded_pic_num++;

    if ((int)(StatBitsPtr->coded_pic_num - (StatBitsPtr->time_s + 1) *framerate[hd->frame_rate_code - 1] + 0.5) == 0) {
        StatBitsPtr->total_bitrate[StatBitsPtr->time_s++] = StatBitsPtr->bitrate;
        StatBitsPtr->bitrate = 0;
    }

    // record the reference list information
    get_reference_list_info(hc->str_list_reference);


    frame_postprocessing();

    StatBitsPtr->curr_frame_bits = 0;
    StatBitsPtr->emulate_bits    = 0;
    StatBitsPtr->last_unit_bits  = 0;

#if FIX_PROFILE_LEVEL_DPB_RPS_1
    //////////////////////////////////////////////////////////////////////////
    // delete the frame that will never be used
    {
        int i, j;
        for (i = 0; i < hd->curr_RPS.num_to_remove; i++) {
            for (j = 0; j < REF_MAXBUFFER; j++) {

                if (fref[j]->imgcoi_ref >= -256 && fref[j]->imgcoi_ref == img->coding_order - hd->curr_RPS.remove_pic[i])

                {
                    break;
                }
            }
            if (j < REF_MAXBUFFER) {
#if FIX_RPS_PICTURE_REMOVE
				/* Label new frames as "un-referenced" */
				fref[j]->refered_by_others = 0;

				/* remove frames which have been outputted */
				if (fref[j]->is_output == -1) {
					fref[j]->imgtr_fwRefDistance = -256;
					fref[j]->imgcoi_ref = -257;
					fref[j]->temporal_id = -1;
				}
#else
                fref[j]->imgcoi_ref = -257;
#if M3480_TEMPORAL_SCALABLE
                fref[j]->temporal_id = -1;
#endif
                if (fref[j]->is_output == -1)
                { fref[j]->imgtr_fwRefDistance = -256; }
#endif
            }
        }
    }
#endif


    //! TO 19.11.2001 Known Problem: for init_frame we have to know the picture type of the actual frame
    //! in case the first slice of the P-Frame following the I-Frame was lost we decode this P-Frame but//! do not write it because it was assumed to be an I-Frame in init_frame.So we force the decoder to
    //! guess the right picture type. This is a hack a should be removed by the time there is a clean
    //! solution where we do not have to know the picture type for the function init_frame.
    //! End TO 19.11.2001//Lou

    {
        if (img->type == I_IMG || img->type == P_IMG || img->type == F_IMG) {
            img->number++;
        } else {
            hc->Bframe_ctr++;  // B pictures
        }
    }

    return (SOP);
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
void report_frame(outdata data, int pos)
{
    FILE *file;
    char *Frmfld;
    char Frm[] = "FRM";
    char Fld[] = "FLD";
    STDOUT_DATA *p_stdoutdata = &data.stdoutdata[pos];
    const char *typ;

    file = fopen("stat.dat", "at");

    if (input->MD5Enable & 0x02) {
        sprintf(MD5str, "%08X%08X%08X%08X\0",
                p_stdoutdata->DecMD5Value[0],
                p_stdoutdata->DecMD5Value[1],
                p_stdoutdata->DecMD5Value[2],
                p_stdoutdata->DecMD5Value[3]);
    } else {
        memset(MD5val, 0, 16);
        memset(MD5str, 0, 33);
    }

    if (p_stdoutdata->picture_structure) {
        Frmfld = Frm;
    } else {
        Frmfld = Fld;
    }
#if INTERLACE_CODING
    if (img->is_field_sequence) { //rcs??
        Frmfld = Fld;
    }
#endif
    if ((p_stdoutdata->tr + hc->total_frames * 256) == hd->end_SeqTr) {   // I picture
        //if ( img->new_sequence_flag == 1 )
        {
            img->sequence_end_flag = 0;
            fprintf(stdout, "Sequence End\n\n");
        }
    }
    if ((p_stdoutdata->tr + hc->total_frames * 256) == hd->next_IDRtr) {
        if (hd->vec_flag) {
            hd->vec_flag = 0;
            fprintf(stdout, "Video Edit Code\n");
        }
    }

    if (p_stdoutdata->typeb == BACKGROUND_IMG) {
        typ = (hd->background_picture_output_flag != 0) ? "G" : "GB";
    } else {
#if	REMOVE_UNUSED 
        typ = (p_stdoutdata->type == INTRA_IMG) ? "I" : (p_stdoutdata->type == INTER_IMG) ?
			((p_stdoutdata->typeb == BP_IMG) ? "S" : "P") : (p_stdoutdata->type == F_IMG ? "F" : "B");
#else
        typ = (p_stdoutdata->type == INTRA_IMG) ? "I" : (p_stdoutdata->type == INTER_IMG) ?
              ((p_stdoutdata->type == BP_IMG) ? "S" : "P") : (p_stdoutdata->type == F_IMG ? "F" : "B");
#endif
    }

    fprintf(file, "%3d(%s)  %3d %5d %7.4f %7.4f %7.4f %5d\t\t%s %8d %6d\t%s",
            p_stdoutdata->framenum + hc->total_frames * 256, typ, p_stdoutdata->tr + hc->total_frames * 256,
            p_stdoutdata->qp, p_stdoutdata->snr_y, p_stdoutdata->snr_u, p_stdoutdata->snr_v,
            p_stdoutdata->tmp_time, Frmfld, p_stdoutdata->curr_frame_bits, p_stdoutdata->emulate_bits,
            MD5str);
    printf("%3d(%s)  %3d %5d %7.4f %7.4f %7.4f %5d\t\t%s %8d %6d\t%s",
           p_stdoutdata->framenum + hc->total_frames * 256, typ, p_stdoutdata->tr + hc->total_frames * 256,
           p_stdoutdata->qp, p_stdoutdata->snr_y, p_stdoutdata->snr_u, p_stdoutdata->snr_v,
           p_stdoutdata->tmp_time, Frmfld, p_stdoutdata->curr_frame_bits, p_stdoutdata->emulate_bits,
           MD5str);

    fprintf(file, " %s\n", p_stdoutdata->str_reference_list);
    printf(" %s\n", p_stdoutdata->str_reference_list);

    fclose(file);
    fflush(stdout);
    hd->FrameNum++;
}

/*
*************************************************************************
* Function:Find PSNR for all three components.Compare decoded frame with
the original sequence. Read input->jumpd frames to reflect frame skipping.
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void find_snr(SNRParameters *snr, FILE *p_ref)            //!< filestream to reference YUV file
{
    int i, j;
    long long int diff_y, diff_u, diff_v;
    int uv;
    int uvformat = input->chroma_format == 1 ? 4 : 2;

    int nOutputSampleSize = (input->output_bit_depth > 8) ? 2 : 1;
    unsigned char chTemp[2];
    int nBitDepthDiff = input->sample_bit_depth -
                        input->output_bit_depth; // assume coding bit depth no less than output bit depth

    long long int  status;

    int img_width     = (img->width - img->auto_crop_right);
    int img_height    = (img->height - img->auto_crop_bottom);
    int img_width_cr  = (img_width / 2);
    int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));
    const unsigned int  bytes_y = img_width * img_height;
    const unsigned int  bytes_uv = img_width_cr * img_height_cr;
    const long long int framesize_in_bytes = (long long int)(bytes_y + 2 * bytes_uv);

    unsigned char *buf;

    int   offset_units;

    if (!input->yuv_structure) {
        buf = malloc(nOutputSampleSize * bytes_y);
    } else {
        buf = malloc(nOutputSampleSize * (bytes_y + 2 * bytes_uv));
    }

    if (NULL == buf) {
        no_mem_exit("find_snr: buf");
    }

    snr->snr_y = 0.0;
    snr->snr_u = 0.0;
    snr->snr_v = 0.0;

    fseek(p_ref, 0, SEEK_SET);

    if (hd->RefPicExist) {
        if (!input->ref_pic_order) {   //ref order
            status = fseek(p_ref, (long long int)(nOutputSampleSize * framesize_in_bytes * (img->tr + hc->total_frames *
                                                  256)/*( img->type == B_IMG ? img->tr : FrameNum )*/), 0);
        } else {
            status = fseek(p_ref, (long long int)(nOutputSampleSize * framesize_in_bytes * hd->dec_ref_num), 0);
        }

        if (status != 0) {
            snprintf(hc->errortext, ET_SIZE, "Error in seeking img->tr: %d", img->tr);
            hd->RefPicExist = 0;
        }
    }


    if (!input->yuv_structure) {
        fread(buf, sizeof(unsigned char), nOutputSampleSize * bytes_y, p_ref);
    } else {
        fread(buf, sizeof(unsigned char), nOutputSampleSize * (bytes_y + 2 * bytes_uv), p_ref);
    }

    if (!input->yuv_structure) {   //YUV
        for (j = 0; j < img_height; j++) {
            if (input->output_bit_depth == 8) {
                for (i = 0; i < img_width; i++) {
                    hd->imgYRef[j][i] = buf[j * img_width + i];
                    hd->imgYRef[j][i] &= 0xff; // reset high 8 bits
                }
            } else if (input->output_bit_depth == 10) {
                for (i = 0; i < img_width; i++) {
                    hd->imgYRef[j][i] = ((byte *)buf)[j * img_width + i] & 0x3ff;
                }
            } else if (input->output_bit_depth == 12) {
                for (i = 0; i < img_width; i++) {
                    hd->imgYRef[j][i] = ((byte *)buf)[j * img_width + i] & 0xfff;
                }
            }

        }
    } else { //U0Y0 V1Y1
        for (j = 0; j < img_height; j++) {
            for (i = 0; i < img_width; i++) {
                if (input->output_bit_depth == 8) {
                    offset_units = j * img_width + i;
                    hd->imgYRef[j][i] = buf[2 * offset_units + 1];
                    hd->imgYRef[j][i] &= 0xff;
                    if (offset_units % 2 == 0) {   //U component
                        hd->imgUVRef[0][j][i / 2] = buf[2 * offset_units];
                    } else {              //V component
                        hd->imgUVRef[1][j][i / 2] = buf[2 * offset_units];
                    }
                    hd->imgYRef[j][i] = fgetc(p_ref);
                } else if (input->output_bit_depth == 10) {
                    offset_units = j * img_width + i;
                    hd->imgYRef[j][i] = ((byte *)buf)[2 * offset_units + 1];
                    hd->imgYRef[j][i] = hd->imgYRef[j][i] & 0x3ff;
                    if (offset_units % 2 == 0) {   //U component
                        hd->imgUVRef[0][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[0][j][i / 2] &= 0x3ff;
                    } else {              //V component
                        hd->imgUVRef[1][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[1][j][i / 2] &= 0x3ff;
                    }
                    chTemp[0] = fgetc(p_ref);
                    chTemp[1] = fgetc(p_ref);
                    hd->imgYRef[j][i] = ((byte *)(&(chTemp[0])))[0] & 0x3ff;
                } else if (input->output_bit_depth == 12) {
                    offset_units = j * img_width + i;
                    hd->imgYRef[j][i] = ((byte *)buf)[2 * offset_units + 1];
                    hd->imgYRef[j][i] = hd->imgYRef[j][i] & 0xfff;
                    if (offset_units % 2 == 0) {   //U component
                        hd->imgUVRef[0][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[0][j][i / 2] &= 0xfff;
                    } else {              //V component
                        hd->imgUVRef[1][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[1][j][i / 2] &= 0xfff;
                    }
                    chTemp[0] = fgetc(p_ref);
                    chTemp[1] = fgetc(p_ref);
                    hd->imgYRef[j][i] = ((byte *)(&(chTemp[0])))[0] & 0xfff;
                }

            }
        }
    }

    for (uv = 0; uv < 2; uv++) {
        if (!input->yuv_structure) {
            fread(buf, sizeof(unsigned char), nOutputSampleSize * img_height_cr * img_width_cr, p_ref);

            for (j = 0; j < img_height_cr; j++) {
                for (i = 0; i < img_width_cr; i++) {
                    if (input->output_bit_depth == 8) {
                        hd->imgUVRef[uv][j][i] = buf[j * img_width_cr + i];
                        hd->imgUVRef[uv][j][i] &= 0xff;
                    } else if (input->output_bit_depth == 10) {
                        hd->imgUVRef[uv][j][i] = ((byte *)buf)[j * img_width_cr + i] & 0x3ff;
                    } else if (input->output_bit_depth == 12) {
                        hd->imgUVRef[uv][j][i] = ((byte *)buf)[j * img_width_cr + i] & 0xfff;
                    }
                }
            }
        }
    }

    img->quad[0] = 0;
    diff_y = 0;

    for (j = 0; j < img_height; ++j) {
        for (i = 0; i < img_width; ++i) {
            if (nBitDepthDiff == 0) {
                diff_y += img->quad[abs(hc->imgY[j][i] - hd->imgYRef[j][i])];
            } else if (nBitDepthDiff > 0) {
                diff_y += img->quad[abs(Clip1((hc->imgY[j][i] + (1 << (nBitDepthDiff - 1))) >> nBitDepthDiff) - hd->imgYRef[j][i])];
            }

        }
    }

    // Chroma
    diff_u = 0;
    diff_v = 0;

    for (j = 0; j < img_height_cr; ++j) {
        for (i = 0; i < img_width_cr; ++i) {
            if (nBitDepthDiff == 0) {
                diff_u += img->quad[abs(hd->imgUVRef[0][j][i] - hc->imgUV[0][j][i])];
                diff_v += img->quad[abs(hd->imgUVRef[1][j][i] - hc->imgUV[1][j][i])];
            } else if (nBitDepthDiff > 0) {
                diff_u += img->quad[abs(hd->imgUVRef[0][j][i] - Clip1((hc->imgUV[0][j][i] + (1 << (nBitDepthDiff - 1))) >>
                                        nBitDepthDiff))];
                diff_v += img->quad[abs(hd->imgUVRef[1][j][i] - Clip1((hc->imgUV[1][j][i] + (1 << (nBitDepthDiff - 1))) >>
                                        nBitDepthDiff))];
            }
        }
    }

    // Collecting SNR statistics
    if (diff_y != 0) {
        snr->snr_y = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                        (float)(img_width/*img->width*/) * (img_height/*img->height*/) /
                                        (float) diff_y));    // luma snr for current frame
    }

    if (diff_u != 0) {
        snr->snr_u = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                        (float)(img_width/*img->width*/) * (img_height/*img->height*/) /
                                        (float)(/*4*/uvformat * diff_u)));     //  chroma snr for current frame,422
    }

    if (diff_v != 0) {
        snr->snr_v = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                        (float)(img_width/*img->width*/) * (img_height/*img->height*/) /
                                        (float)(/*4*/uvformat * diff_v)));     //  chroma snr for current frame,422
    }

    if (img->number == 0) {   // first
        snr->snr_y1 = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                         (float)(img_width/*img->width*/) * (img_height/*img->height*/) /
                                         (float)diff_y));   // keep luma snr for first frame
        snr->snr_u1 = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                         (float)(img_width/*img->width*/) * (img_height/*img->height*/) /
                                         (float)(/*4*/uvformat * diff_u)));    // keep chroma snr for first frame,422
        snr->snr_v1 = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                         (float)(img_width/*img->width*/) * (img_height/*img->height*/) /
                                         (float)(/*4*/uvformat * diff_v)));    // keep chroma snr for first frame,422
        snr->snr_ya = snr->snr_y1;
        snr->snr_ua = snr->snr_u1;
        snr->snr_va = snr->snr_v1;

        if (diff_y == 0) {
            snr->snr_ya =/*50*/0;
        }

        if (diff_u == 0) {
            snr->snr_ua =/*50*/0;
        }

        if (diff_v == 0) {
            snr->snr_va =/*50*/0;
        }
    } else {
        snr->snr_ya = (snr->snr_ya * (double)(img->number + hc->Bframe_ctr) + snr->snr_y) /
                      (double)(img->number + hc->Bframe_ctr + 1);  // average snr lume for all frames inc. first
        snr->snr_ua = (snr->snr_ua * (double)(img->number + hc->Bframe_ctr) + snr->snr_u) /
                      (double)(img->number + hc->Bframe_ctr + 1);  // average snr u croma for all frames inc. first
        snr->snr_va = (snr->snr_va * (double)(img->number + hc->Bframe_ctr) + snr->snr_v) /
                      (double)(img->number + hc->Bframe_ctr + 1);  // average snr v croma for all frames inc. first
    }

    free(buf);
}

/*
*************************************************************************
* Function:Find PSNR for BACKGROUND PICTURE.Compare decoded frame with
the original sequence. Read input->jumpd frames to reflect frame skipping.
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void find_snr_background(SNRParameters *snr, FILE *p_ref)            //!< filestream to reference YUV file
{
    int i, j;
    long long int diff_y, diff_u, diff_v;
    int uv;
    int uvformat = input->chroma_format == 1 ? 4 : 2;

    int nOutputSampleSize = (input->output_bit_depth > 8) ? 2 : 1;
    int nBitDepthDiff = input->sample_bit_depth -
                        input->output_bit_depth; // assume coding bit depth no less than output bit depth

    long long int status;

    int img_width = (img->width - img->auto_crop_right);
    int img_height = (img->height - img->auto_crop_bottom);
    int img_width_cr = (img_width / 2);
    int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));
    const unsigned int  bytes_y = img_width * img_height;
    const unsigned int  bytes_uv = img_width_cr * img_height_cr;
    const long long int framesize_in_bytes = (long long int)(bytes_y + 2 * bytes_uv);

    unsigned char *buf;

    int   offset_units;

    if (!input->yuv_structure) {
        buf = malloc(nOutputSampleSize * bytes_y);
    } else {
        buf = malloc(nOutputSampleSize * (bytes_y + 2 * bytes_uv));
    }

    if (NULL == buf) {
        no_mem_exit("find_snr_background: buf");
    }

    snr->snr_y = 0.0;
    snr->snr_u = 0.0;
    snr->snr_v = 0.0;

    rewind(p_ref);

    if (hd->BgRefPicExist) {
        if (!input->ref_pic_order) {   //ref order
            status = fseek(p_ref, (long long int)(nOutputSampleSize * framesize_in_bytes * (hd->background_number - 1)), 0);
        } else {
            status = fseek(p_ref, (long long int)(nOutputSampleSize * framesize_in_bytes * (hd->background_number - 1)), 0);
        }

        if (status != 0) {
            snprintf(hc->errortext, ET_SIZE, "Error in seeking img->tr: %d", img->tr);
            hd->BgRefPicExist = 0;
        }
    }

    if (!input->yuv_structure) {
        fread(buf, sizeof(unsigned char), nOutputSampleSize * bytes_y, p_ref);
    } else {
        fread(buf, sizeof(unsigned char), nOutputSampleSize * (bytes_y + 2 * bytes_uv), p_ref);
    }

    if (!input->yuv_structure) {   //YUV
        for (j = 0; j < img_height; j++) {
            if (input->output_bit_depth == 8) {
                for (i = 0; i < img_width; i++) {
                    hd->imgYRef[j][i] = buf[j * img_width + i];
                    hd->imgYRef[j][i] &= 0xff; // reset high 8 bits
                }
            } else if (input->output_bit_depth == 10) {
                for (i = 0; i < img_width; i++) {
                    hd->imgYRef[j][i] = ((byte *)buf)[j * img_width + i] & 0x3ff;
                }
            } else if (input->output_bit_depth == 12) {
                for (i = 0; i < img_width; i++) {
                    hd->imgYRef[j][i] = ((byte *)buf)[j * img_width + i] & 0xfff;
                }
            }

        }
    } else { //U0Y0 V1Y1
        for (j = 0; j < img_height; j++) {
            for (i = 0; i < img_width; i++) {
                if (input->output_bit_depth == 8) {
                    offset_units = j * img_width + i;
                    hd->imgYRef[j][i] = buf[2 * offset_units + 1];
                    hd->imgYRef[j][i] &= 0xff;
                    if (offset_units % 2 == 0) {   //U component
                        hd->imgUVRef[0][j][i / 2] = buf[2 * offset_units];
                    } else {              //V component
                        hd->imgUVRef[1][j][i / 2] = buf[2 * offset_units];
                    }
                } else if (input->output_bit_depth == 10) {
                    offset_units = j * img_width + i;
                    hd->imgYRef[j][i] = ((byte *)buf)[2 * offset_units + 1];
                    hd->imgYRef[j][i] = hd->imgYRef[j][i] & 0x3ff;
                    if (offset_units % 2 == 0) {   //U component
                        hd->imgUVRef[0][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[0][j][i / 2] &= 0x3ff;
                    } else {              //V component
                        hd->imgUVRef[1][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[1][j][i / 2] &= 0x3ff;
                    }
                } else if (input->output_bit_depth == 12) {
                    offset_units = j * img_width + i;
                    hd->imgYRef[j][i] = ((byte *)buf)[2 * offset_units + 1];
                    hd->imgYRef[j][i] = hd->imgYRef[j][i] & 0xfff;
                    if (offset_units % 2 == 0) {   //U component
                        hd->imgUVRef[0][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[0][j][i / 2] &= 0xfff;
                    } else {              //V component
                        hd->imgUVRef[1][j][i / 2] = ((byte *)buf)[2 * offset_units];
                        hd->imgUVRef[1][j][i / 2] &= 0xfff;
                    }
                }

            }
        }
    }


    for (uv = 0; uv < 2; uv++) {
        if (!input->yuv_structure) {
            fread(buf, sizeof(unsigned char), nOutputSampleSize * img_height_cr * img_width_cr, p_ref);

            for (j = 0; j < img_height_cr; j++) {
                for (i = 0; i < img_width_cr; i++) {
                    if (input->output_bit_depth == 8) {
                        hd->imgUVRef[uv][j][i] = buf[j * img_width_cr + i];
                        hd->imgUVRef[uv][j][i] &= 0xff;
                    } else if (input->output_bit_depth == 10) {
                        hd->imgUVRef[uv][j][i] = ((byte *)buf)[j * img_width_cr + i] & 0x3ff;
                    } else if (input->output_bit_depth == 12) {
                        hd->imgUVRef[uv][j][i] = ((byte *)buf)[j * img_width_cr + i] & 0xfff;
                    }
                }
            }
        }
    }

    img->quad[0] = 0;
    diff_y = 0;

    for (j = 0; j < img_height; ++j) {
        for (i = 0; i < img_width; ++i) {
            if (nBitDepthDiff == 0) {
                diff_y += img->quad[abs(hc->imgY[j][i] - hd->imgYRef[j][i])];
            } else if (nBitDepthDiff > 0) {
                diff_y += img->quad[abs(Clip1((hc->imgY[j][i] + (1 << (nBitDepthDiff - 1))) >> nBitDepthDiff) - hd->imgYRef[j][i])];
            }

        }
    }

    // Chroma
    diff_u = 0;
    diff_v = 0;

    for (j = 0; j < img_height_cr; ++j) {
        for (i = 0; i < img_width_cr; ++i) {
            if (nBitDepthDiff == 0) {
                diff_u += img->quad[abs(hd->imgUVRef[0][j][i] - hc->imgUV[0][j][i])];
                diff_v += img->quad[abs(hd->imgUVRef[1][j][i] - hc->imgUV[1][j][i])];
            } else if (nBitDepthDiff > 0) {
                diff_u += img->quad[abs(hd->imgUVRef[0][j][i] - Clip1((hc->imgUV[0][j][i] + (1 << (nBitDepthDiff - 1))) >>
                                        nBitDepthDiff))];
                diff_v += img->quad[abs(hd->imgUVRef[1][j][i] - Clip1((hc->imgUV[1][j][i] + (1 << (nBitDepthDiff - 1))) >>
                                        nBitDepthDiff))];
            }
        }
    }

    // Collecting SNR statistics
    if (diff_y != 0) {
        snr->snr_y = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                        (float)(img_width/*img->width*/) * (img_height/*img->height*/) / (float)
                                        diff_y));    // luma snr for current frame
    }

    if (diff_u != 0) {
        snr->snr_u = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                        (float)(img_width/*img->width*/) * (img_height/*img->height*/) / (float)(/*4*/uvformat *
                                                diff_u)));     //  chroma snr for current frame,422
    }

    if (diff_v != 0) {
        snr->snr_v = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                        (float)(img_width/*img->width*/) * (img_height/*img->height*/) / (float)(/*4*/uvformat *
                                                diff_v)));     //  chroma snr for current frame,422
    }

    if (img->number == /*0*/1) {   // first
        snr->snr_y1 = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                         (float)(img_width/*img->width*/) * (img_height/*img->height*/) / (float)
                                         diff_y));   // keep luma snr for first frame
        snr->snr_u1 = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                         (float)(img_width/*img->width*/) * (img_height/*img->height*/) / (float)(/*4*/uvformat *
                                                 diff_u)));    // keep chroma snr for first frame,422
        snr->snr_v1 = (float)(10 * log10(((1 << input->output_bit_depth) - 1) * ((1 << input->output_bit_depth) - 1) *
                                         (float)(img_width/*img->width*/) * (img_height/*img->height*/) / (float)(/*4*/uvformat *
                                                 diff_v)));    // keep chroma snr for first frame,422
        snr->snr_ya = snr->snr_y1;
        snr->snr_ua = snr->snr_u1;
        snr->snr_va = snr->snr_v1;

        if (diff_y == 0) {
            snr->snr_ya =/*50*/0;
        }

        if (diff_u == 0) {
            snr->snr_ua =/*50*/0;
        }

        if (diff_v == 0) {
            snr->snr_va =/*50*/0;
        }
    } else {
        snr->snr_ya = (float)(snr->snr_ya * (img->number + hc->Bframe_ctr - 1) + snr->snr_y) /
                      (img->number + hc->Bframe_ctr);   // average snr chroma for all frames 20080721
        snr->snr_ua = (float)(snr->snr_ua * (img->number + hc->Bframe_ctr - 1) + snr->snr_u) /
                      (img->number + hc->Bframe_ctr);   // average snr luma for all frames 20080721
        snr->snr_va = (float)(snr->snr_va * (img->number + hc->Bframe_ctr - 1) + snr->snr_v) /
                      (img->number + hc->Bframe_ctr);   // average snr luma for all frames 20080721
    }

    free(buf);
}

#if M3480_TEMPORAL_SCALABLE
void cleanRefMVBufRef(int pos)
{
    int k, x, y;
    //re-init mvbuf
    for (k = 0; k < 2; k++) {
        for (y = 0; y < img->height / MIN_BLOCK_SIZE; y++) {
            for (x = 0; x < img->width / MIN_BLOCK_SIZE; x++) {
                fref[pos]->mvbuf[y][x][k] = 0;
            }
        }
    }
    //re-init refbuf
    for (y = 0; y < img->height / MIN_BLOCK_SIZE; y++) {
        for (x = 0; x < img->width / MIN_BLOCK_SIZE ; x++) {
            fref[pos]->refbuf[y][x] = -1;
        }
    }
}
#endif

void prepare_RefInfo()
{
    int i, j;
    int flag = 0;
    avs2_frame_t *tmp_fref;

    //////////////////////////////////////////////////////////////////////////
    //update IDR frame
    if (img->tr > hd->next_IDRtr && hd->curr_IDRtr != hd->next_IDRtr) {
        hd->curr_IDRtr  = hd->next_IDRtr;
        hd->curr_IDRcoi = hd->next_IDRcoi;
    }
    //////////////////////////////////////////////////////////////////////////
    // re-order the ref buffer according to RPS
    img->num_of_references = hd->curr_RPS.num_of_ref;

    for (i = 0; i < hd->curr_RPS.num_of_ref; i++) {
        int accumulate = 0;
        /* copy tmp_fref from fref[i] */
        tmp_fref = fref[i];

#if REMOVE_UNUSED
        for (j = i; j < REF_MAXBUFFER; j++) {  ///////////////to be modified  IDR
			if (fref[j]->imgcoi_ref == img->coding_order - hd->curr_RPS.ref_pic[i]) {
				break;
			}
		}
#else

        for (j = i; j < REF_MAXBUFFER; j++) {  ///////////////to be modified  IDR
            int k , tmp_tr;
            for (k = 0; k < REF_MAXBUFFER; k++) {
                if (((int)img->coding_order - (int)hd->curr_RPS.ref_pic[i]) == fref[k]->imgcoi_ref && fref[k]->imgcoi_ref >= -256) {
                    break;
                }
            }
            if (k == REF_MAXBUFFER) {
                tmp_tr = -1;
            } else {
                tmp_tr = fref[k]->imgtr_fwRefDistance;
            }
            if (tmp_tr < hd->curr_IDRtr) {
                hd->curr_RPS.ref_pic[i] = img->coding_order - hd->curr_IDRcoi;

                for (k = 0; k < i; k++) {
                    if (hd->curr_RPS.ref_pic[k] == hd->curr_RPS.ref_pic[i]) {
                        accumulate++;
                        break;
                    }
                }
            }
            if (fref[j]->imgcoi_ref == img->coding_order - hd->curr_RPS.ref_pic[i]) {
                break;
            }
        }
        if (j == REF_MAXBUFFER || accumulate) {
            img->num_of_references--;
        }
#endif
        if (j != REF_MAXBUFFER) {
            /* copy fref[i] from fref[j] */
            fref[i] = fref[j];
            /* copy fref[j] from ferf[tmp] */
            fref[j] = tmp_fref;

        }
    }
    if (img->type == B_IMG && (fref[0]->imgtr_fwRefDistance <= img->tr || fref[1]->imgtr_fwRefDistance >= img->tr)) {

        printf("wrong reference configuration for B frame");
        exit(-1);
        //******************************************//
    }

#if !FIX_PROFILE_LEVEL_DPB_RPS_1
    //////////////////////////////////////////////////////////////////////////
    // delete the frame that will never be used
    for (i = 0; i < hd->curr_RPS.num_to_remove; i++) {
        for (j = 0; j < REF_MAXBUFFER; j++) {
            if (fref[j]->imgcoi_ref >= -256 && fref[j]->imgcoi_ref == img->coding_order - hd->curr_RPS.remove_pic[i]) {
                break;
            }
        }
        if (j < REF_MAXBUFFER && j >= img->num_of_references) {
            fref[j]->imgcoi_ref = -257;
#if M3480_TEMPORAL_SCALABLE
            fref[j]->temporal_id = -1;
#endif
            if (fref[j]->is_output == -1) {
                fref[j]->imgtr_fwRefDistance = -256;
            }
        }
    }
#endif

    //////////////////////////////////////////////////////////////////////////
    // add inter-view reference picture

    //////////////////////////////////////////////////////////////////////////
    //   add current frame to ref buffer
    for (i = 0; i < REF_MAXBUFFER; i++) {
        if ((fref[i]->imgcoi_ref < -256 || abs(fref[i]->imgtr_fwRefDistance - img->tr) >= 128) && fref[i]->is_output == -1) {
            break;
        }
    }
    if (i == REF_MAXBUFFER) {
        i--;
    }

    hc->f_rec        = fref[i];
    hc->currentFrame = hc->f_rec->ref;
    hc->f_rec->imgtr_fwRefDistance = img->tr;
    hc->f_rec->imgcoi_ref = img->coding_order;
#if M3480_TEMPORAL_SCALABLE
    hc->f_rec->temporal_id = hd->cur_layer;
#endif
    hc->f_rec->is_output = 1;
    hc->f_rec->refered_by_others = hd->curr_RPS.referd_by_others;

    if (img->type != B_IMG) {
        for (j = 0; j < img->num_of_references; j++) {
            hc->f_rec->ref_poc[j] = fref[j]->imgtr_fwRefDistance;
        }
    } else {
        hc->f_rec->ref_poc[0] = fref[1]->imgtr_fwRefDistance;
        hc->f_rec->ref_poc[1] = fref[0]->imgtr_fwRefDistance;
    }

#if M3480_TEMPORAL_SCALABLE

    for (j = img->num_of_references; j < 4; j++) {
        hc->f_rec->ref_poc[j] = 0;
    }

    if (img->type == INTRA_IMG) {
        int l;
        for (l = 0; l < 4; l ++) {
            hc->f_rec->ref_poc[l] = img->tr;
        }
    }
    cleanRefMVBufRef(i);
#endif

    for (j = 0; j < NUM_SAO_COMPONENTS; j++) {
        hc->f_rec->saorate[j] = img->cur_saorate[j];
    }

    //////////////////////////////////////////////////////////////////////////
    // updata ref pointer

    if (img->type != I_IMG) {
#if FIX_MAX_REF
        for (j = 0; j < MAXREF; j++) {   //ref_index = 0
#else
        for (j = 0; j < 4; j++) {   //ref_index = 0
#endif
            hd->integerRefY[j]     = fref[j]->ref[0];
            hd->integerRefUV[j][0] = fref[j]->ref[1];
            hd->integerRefUV[j][1] = fref[j]->ref[2];
        }

        // forward/backward reference buffer
        // f_ref[ref_index][yuv][height(height/2)][width] ref_index=0 for B frame, ref_index = 0,1 for B field
        hd->f_ref[0] = fref[1]->ref;
        // b_ref[ref_index][yuv][height(hei ght/2)][width] ref_index=0 for B frame, ref_index = 0,1 for B field
        hd->b_ref[0] = fref[0]->ref;

        for (j = 0; j < 1; j++) {   //ref_index = 0 luma = 0
            hd->integerRefY_fref[j] = hd->f_ref[j][0];
            hd->integerRefY_bref[j] = hd->b_ref[j][0];
        }
        //chroma for backward
        for (j = 0; j < 1; j++) {   //ref_index = 0
            for (i = 0; i < 2; i++) {   // chroma uv =0,1; 1,2 for referenceFrame
                hd->integerRefUV_fref[j][i] = hd->f_ref[j][i + 1];
                hd->integerRefUV_bref[j][i] = hd->b_ref[j][i + 1];
            }
        }
        img->imgtr_next_P = img->type == B_IMG ? fref[0]->imgtr_fwRefDistance : img->tr;
        if (img->type == B_IMG) {
            hd->trtmp = fref[0]->imgtr_fwRefDistance;
            fref[0]->imgtr_fwRefDistance = fref[1]->imgtr_fwRefDistance;
        }
    }

    {
        int k, x, y, ii;
#if B_BACKGROUND_Fix
        for (ii = 0; ii < REF_MAXBUFFER; ii++) {
#else
        for (ii = 0; ii < 8; ii++) {
#endif
            for (k = 0; k < 2; k++) {
                for (y = 0; y < img->height / MIN_BLOCK_SIZE; y++) {
                    for (x = 0; x < img->width / MIN_BLOCK_SIZE ; x++) {
                        if (img->typeb == BP_IMG) {
                            fref[ii]->mvbuf[y][x][k] = 0;
                            fref[ii]->refbuf[y][x] = 0;
                        }
                    }
                }
            }
        }
    }
}
#if Mv_Rang    //M3959 he-yuan.lin@mstarsemi.com
void check_mv_boundary(int pos_x, int pos_y, int dx, int dy, int block_width, int block_height)
{
	int is_out_of_boundary = 0;
	int extend_x_left = (dx != 0) ? 3 : 0;
	int extend_y_top = (dy != 0) ? 3 : 0;
	int extend_x_right = (dx != 0) ? 4 : 0;
	int extend_y_down = (dy != 0) ? 4 : 0;

	// left
	if ((pos_x - extend_x_left) < -64) {
		is_out_of_boundary = 1;
	}
	// right
	if (((pos_x + block_width -1) + extend_x_right) >= (hd->horizontal_size + 64)) {
		is_out_of_boundary = 1;
	}
	// up
	if ((pos_y - extend_y_top) < -64) {
		is_out_of_boundary = 1;
	}
	// bottom
	if (((pos_y + block_height -1) + extend_y_down) >= (hd->vertical_size + 64)) {
		is_out_of_boundary = 1;
	}

	if (is_out_of_boundary) {
		printf("Non-conformance stream: invalid reference block (x, y) = (%d, %d)\n", pos_x, pos_y);
	}
}
#endif
/*
*************************************************************************
* Function:Interpolation of 1/4 subpixel
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void get_block(int ref_frame, int x_pos, int y_pos, int step_h, int step_v, int block[MAX_CU_SIZE][MAX_CU_SIZE],
               byte **ref_pic
               , int ioff, int joff

              )
{
    int max_pel_value = (1 << input->sample_bit_depth) - 1;
    int shift1 = input->sample_bit_depth - 8;
    int shift2 = 14 - input->sample_bit_depth;
    int shift3 = 20 - input->sample_bit_depth;
    int dx, dy;
    int x, y;
    int i, j;
    int maxold_x, maxold_y;
    int result;
    int tmp_res[140][140];

    static const int COEF_8tap[3][8] = {
        { -1, 4, -10, 57, 19,  -7, 3, -1 },
        { -1, 4, -11, 40, 40, -11, 4, -1 },
        { -1, 3, -7, 19, 57,  -10, 4, -1 }
    };

    const int *COEF[3];
    byte **p_ref;


    COEF[0] = COEF_8tap[0];
    COEF[1] = COEF_8tap[1];
    COEF[2] = COEF_8tap[2];


    //    A  a  1  b  B
    //    c  d  e  f
    //    2  h  3  i
    //    j  k  l  m
    //    C           D

    dx = x_pos & 3;
    dy = y_pos & 3;
    x_pos = (x_pos - dx) / 4;
    y_pos = (y_pos - dy) / 4;
    maxold_x = img->width - 1;
    maxold_y = img->height - 1;
#if Mv_Rang
	 check_mv_boundary(x_pos, y_pos, dx, dy, step_h, step_v);
#endif
    if (hd->background_reference_enable && ref_frame == img->num_of_references - 1 &&
        (img->type == P_IMG || img->type == F_IMG) && img->typeb != BP_IMG) {
        p_ref = hd->background_frame[0];
    } else if (img->typeb == BP_IMG) {
        p_ref = hd->background_frame[0];
    }

    else {
        p_ref = ref_pic;
    }


    if (dx == 0 && dy == 0) {
        for (j = 0; j < step_v; j++) {
            for (i = 0; i < step_h; i++) {
                block[j][i] = p_ref[max(0, min(maxold_y, y_pos + j))][max(0, min(maxold_x, x_pos + i))];
            }
        }
    } else {
        if (dy == 0) {
            for (j = 0; j < step_v; j++) {
                for (i = 0; i < step_h; i++) {
                    int y = max(0, min(maxold_y, y_pos + j));
                    int x_base = x_pos + i;
                    for (result = 0, x = -3; x < 5; x++) {
                        result += p_ref[y][max(0, min(maxold_x, x_base + x))] * COEF[dx - 1][x + 3];
                    }
                    block[j][i] = max(0, min(max_pel_value, (result + (1 << (shift1 + shift2 - 1))) >> (shift1 + shift2)));
                }
            }
        } else if (dx == 0) {
            for (j = 0; j < step_v; j++) {
                for (i = 0; i < step_h; i++) {
                    int x = max(0, min(maxold_x, x_pos + i));
                    int y_base = y_pos + j;
                    for (result = 0, y = -3; y < 5; y++) {
                        result += p_ref[max(0, min(maxold_y, y_base + y))][x] * COEF[dy - 1][y + 3];
                    }
                    block[j][i] = max(0, min(max_pel_value, (result + (1 << (shift1 + shift2 - 1))) >> (shift1 + shift2)));
                }
            }
        } else  {
            for (j = -3; j < step_v + 4; j++) {
                for (i = 0; i < step_h; i++) {
                    int y = max(0, min(maxold_y, y_pos + j));
                    int x_base = x_pos + i;
                    for (result = 0, x = -3; x < 5; x++) {
                        result += p_ref[y][max(0, min(maxold_x, x_base + x))] * COEF[dx - 1][x + 3];
                    }
                    tmp_res[j + 3][i] = shift1 ? (result + (1 << (shift1 - 1))) >> shift1 : result;
                }
            }

            for (j = 0; j < step_v; j++) {
                for (i = 0; i < step_h; i++) {
                    for (result = 0, y = -3; y < 5; y++) {
                        result += tmp_res[j + 3 + y][i] * COEF[dy - 1][y + 3];
                    }
                    block[j][i] = max(0, min(max_pel_value, (result + (1 << (shift3 - 1))) >> shift3));
                }
            }
        }
    }
}

/*
*************************************************************************
* Function:Reads the content of two successive startcode from the bitstream
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int Header()
{
    unsigned char *Buf;
    int startcodepos, length;
    static unsigned long prev_pos = 0; // 08.16.2007

    if ((Buf = (char *) calloc(MAX_CODED_FRAME_SIZE , sizeof(char))) == NULL) {
        no_mem_exit("GetAnnexbNALU: Buf");
    }

    while (1) {
        hd->StartCodePosition = GetOneUnit(Buf, &startcodepos, &length);    //jlzheng  7.5

        switch (Buf[startcodepos]) {
        case SEQUENCE_HEADER_CODE:

            img->new_sequence_flag = 1;
#if SEQ_CHANGE_CHECKER
            if (seq_checker_buf == NULL) {
                seq_checker_buf = malloc(length);
                seq_checker_length = length;
                memcpy(seq_checker_buf, Buf, length);
            } else {
                if ((seq_checker_length != length) || (memcmp(seq_checker_buf, Buf, length) != 0)) {
    				free(seq_checker_buf); 
                    fprintf(stdout, "Non-conformance stream: sequence header cannot change !!\n");
                }
            }
#endif
            SequenceHeader(Buf, startcodepos, length);

            if (input->alf_enable && alfParAllcoated == 0) {
                CreateAlfGlobalBuffer();
                alfParAllcoated = 1;
            }

            img->seq_header_indicate = 1;
            break;
        case EXTENSION_START_CODE:
            extension_data(Buf, startcodepos, length);
            break;
        case USER_DATA_START_CODE:
            user_data(Buf, startcodepos, length);
            break;
        case VIDEO_EDIT_CODE:
            video_edit_code_data(Buf, startcodepos, length);
#if SEQ_CHANGE_CHECKER
            if (seq_checker_buf != NULL) {
                free(seq_checker_buf);
                seq_checker_buf = NULL;
                seq_checker_length = 0;
            }
#endif
            break;
        case I_PICTURE_START_CODE:
            I_Picture_Header(Buf, startcodepos, length);
            calc_picture_distance();

            Read_ALF_param(Buf, startcodepos, length);

            if (!img->seq_header_indicate) {
                img->B_discard_flag = 1;
                fprintf(stdout, "    I   %3d\t\tDIDSCARD!!\n", img->tr);
                break;
            }

            break;
        case PB_PICTURE_START_CODE:
            PB_Picture_Header(Buf, startcodepos, length);
            calc_picture_distance();

            Read_ALF_param(Buf, startcodepos, length);
            // xiaozhen zheng, 20071009
            if (!img->seq_header_indicate) {
                img->B_discard_flag = 1;

                if (img->type == P_IMG) {
                    fprintf(stdout, "    P   %3d\t\tDIDSCARD!!\n", img->tr);
                }
                if (img->type == F_IMG) {
                    fprintf(stdout, "    F   %3d\t\tDIDSCARD!!\n", img->tr);
                } else {
                    fprintf(stdout, "    B   %3d\t\tDIDSCARD!!\n", img->tr);
                }

                break;
            }

            if (img->seq_header_indicate == 1 && img->type != B_IMG) {
                img->B_discard_flag = 0;
            }
            if (img->type == B_IMG && img->B_discard_flag == 1 && !img->random_access_decodable_flag) {
                fprintf(stdout, "    B   %3d\t\tDIDSCARD!!\n", img->tr);
                break;
            }

            break;
        case SEQUENCE_END_CODE:
#if SEQ_CHANGE_CHECKER
            if (seq_checker_buf != NULL) {
                free(seq_checker_buf);
                seq_checker_buf = NULL;
                seq_checker_length = 0;
            }
#endif
            img->new_sequence_flag = 1;
            img->sequence_end_flag = 1;
            free(Buf);
            return EOS;
            break;
        default:

            if ((Buf[startcodepos] >= SLICE_START_CODE_MIN && Buf[startcodepos] <= SLICE_START_CODE_MAX)
                && ((!img->seq_header_indicate) || (img->type == B_IMG && img->B_discard_flag == 1 &&
                                                    !img->random_access_decodable_flag))) {
                break;
            } else if (Buf[startcodepos] >= SLICE_START_CODE_MIN) {   // modified by jimmy 2009.04.01
                if ((temp_slice_buf = (char *) calloc(MAX_CODED_FRAME_SIZE ,
                                                      sizeof(char))) == NULL) {       // modified 08.16.2007
                    no_mem_exit("GetAnnexbNALU: Buf");
                }

                first_slice_length = length;
                first_slice_startpos = startcodepos;
                memcpy(temp_slice_buf, Buf, length);
                free(Buf);
                return SOP;
            } else {
                printf("Can't find start code");
                free(Buf);
                return EOS;
            }
        }
    }

}

/*
*************************************************************************
* Function:Initializes the parameters for a new frame
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void init_frame()
{
    static int first_P = TRUE;
    int i, j;

    for (i = 0; i < img->width / (MIN_BLOCK_SIZE) + 1; i++) {     // set edge to -1, indicate nothing to predict from
        img->ipredmode[0][i + 1] = -1;
        img->ipredmode[img->height / (MIN_BLOCK_SIZE) + 1][i + 1] = -1;
    }

    for (j = 0; j < img->height / (MIN_BLOCK_SIZE) + 1; j++) {
        img->ipredmode[j + 1][0] = -1;
        img->ipredmode[j + 1][img->width / (MIN_BLOCK_SIZE) + 1] = -1;
    }

    img->ipredmode    [0][0] = -1;

    for (i = 0; i < img->max_mb_nr; i++) {
        img->mb_data[i].slice_nr = -1;
    }

#if RD1510_FIX_BG
    if (img->type == I_IMG && img->typeb == BACKGROUND_IMG) { // G/GB frame
        img->num_of_references = 0;
    } else if (img->type == P_IMG && img->typeb == BP_IMG) { // only one reference frame(G\GB) for S frame
        img->num_of_references = 1;
    }
#endif

    if (img->typeb == BACKGROUND_IMG && hd->background_picture_output_flag == 0) {
        hc->currentFrame = hc->background_ref;
    } else {
        prepare_RefInfo();
    }

    hc->imgY = hc->currentFrame[0];
    hc->imgUV = &hc->currentFrame[1];

#if RD160_FIX_BG //fred.chiu@mediatek.com
	if (hd->weight_quant_enable_flag && hd->pic_weight_quant_enable_flag) {
		WeightQuantEnable = 1;
	} else {
		WeightQuantEnable = 0;
	}
#endif
    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    if (hd->weight_quant_enable_flag && hd->pic_weight_quant_enable_flag) {
        InitFrameQuantParam();
        FrameUpdateWQMatrix();        //M2148 2007-09
    }
#endif

}

/*
*************************************************************************
* Function:decodes one picture
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void picture_data()
{
    unsigned char *Buf;
    int startcodepos, length;
    const int mb_nr = img->current_mb_nr;
    int mb_width = img->width / MIN_CU_SIZE;
    int first_slice = 1; //08.16.2007 tianf
    int N8_SizeScale = 1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT);
    int j;
    int num_of_orgMB_in_row ;//4:1  5:2  6:4
    int num_of_orgMB_in_col ;
    int size = 1 << input->g_uiMaxSizeInBit;
    int pix_x = (img->current_mb_nr % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    int pix_y = (img->current_mb_nr / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
    Slice *currSlice = img->currentSlice;      //lzhang for AEC
    int    ByteStartPosition;
    int    i;
    int    new_slice = 0;
    int    smb_index;
    Boolean currAlfEnable;
    DataPartition *dP;
    int compIdx, ctx_idx;
    int img_height, img_width, lcuHeight, lcuWidth, numLCUInPicWidth, numLCUInPicHeight, NumCUInFrame;
    Boolean   aec_mb_stuffing_bit;
    img->current_slice_nr = -1;                 // jlzheng 6.30
    hd->currentbitoffset = currStream->frame_bitoffset;   // jlzheng 6.30
    currStream->frame_bitoffset = 0;                  // jlzheng 6.30

    if ((Buf = (char *) calloc(MAX_CODED_FRAME_SIZE , sizeof(char))) == NULL) {
        no_mem_exit("GetAnnexbNALU: Buf");     //jlzheng  6.30
    }

    smb_index = 0;
    lcuHeight         = 1 << input->g_uiMaxSizeInBit;
    lcuWidth          = lcuHeight;
    img_height        = img->height;
    img_width         = img->width;
    numLCUInPicWidth  = img_width / lcuWidth ;
    numLCUInPicHeight = img_height / lcuHeight ;
    numLCUInPicWidth  += (img_width % lcuWidth) ? 1 : 0;
    numLCUInPicHeight += (img_height % lcuHeight) ? 1 : 0;
    NumCUInFrame = numLCUInPicHeight * numLCUInPicWidth;
    g_MaxSizeInbit = input->g_uiMaxSizeInBit;
    while (img->current_mb_nr < img->PicSizeInMbs) {   // loop over super codingUnits
        //decode slice header   jlzheng 6.30
        if (img->current_mb_nr < img->PicSizeInMbs) {   // check every LCU
            if (first_slice) {
                SliceHeader(temp_slice_buf, first_slice_startpos, first_slice_length);
                free(temp_slice_buf);
                img->current_slice_nr++;
                first_slice = 0;
                new_slice = 1;
            } else {
                if (checkstartcode()) {
                    GetOneUnit(Buf, &startcodepos, &length);
                    StatBitsPtr->curr_frame_bits += length;
                    SliceHeader(Buf, startcodepos, length);
                    img->current_slice_nr++;
                    new_slice = 1;
                } else {
                    new_slice = 0;
                }
            }

            if (new_slice) {

                init_contexts();
                AEC_new_slice();
                ByteStartPosition = (currStream->frame_bitoffset) / 8;


                for (i = 0; i < 1; i++) {
                    img->currentSlice->partArr[i].readSyntaxElement = readSyntaxElement_AEC;
                    img->currentSlice->partArr[i].bitstream = currStream ;
                }

                currStream = currSlice->partArr[0].bitstream;

                if ((currStream->frame_bitoffset) % 8 != 0) {
                    ByteStartPosition++;
                }

                arideco_start_decoding(&img->currentSlice->partArr[0].de_AEC, currStream->streamBuffer, (ByteStartPosition),
                                       & (currStream->read_len), img->type);
            }
        }  //decode slice header


        pix_x = (img->current_mb_nr % img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
        pix_y = (img->current_mb_nr / img->PicWidthInMbs) << MIN_CU_SIZE_IN_BIT;
        num_of_orgMB_in_col = N8_SizeScale;
        num_of_orgMB_in_row = N8_SizeScale;

        if (pix_x + size >= img->width) {
            num_of_orgMB_in_col = (img->width - pix_x) >> MIN_CU_SIZE_IN_BIT;
        }

        if (pix_y + size >= img->height) {
            num_of_orgMB_in_row = (img->height - pix_y) >> MIN_CU_SIZE_IN_BIT;
        }

        for (i = 0; i < num_of_orgMB_in_row; i++) {
            int pos = img->current_mb_nr + i * img->PicWidthInMbs;

            for (j = 0; j < num_of_orgMB_in_col; j++, pos++) {
                img->mb_data[pos].slice_nr = img->current_slice_nr;
            }
        }

#if TRACE
        fprintf(hc->p_trace, "\n*********** Pic: %i (I/P) MB: %i Slice: %i Type %d **********\n", img->tr, img->current_mb_nr,
                img->current_slice_nr, img->type);
#endif
        start_codingUnit(input->g_uiMaxSizeInBit);

        if (input->sao_enable) {
            sao_cross_slice = input->crossSliceLoopFilter;
            readParaSAO_one_SMB(smb_index, pix_y >> MIN_CU_SIZE_IN_BIT, pix_x >> MIN_CU_SIZE_IN_BIT,  num_of_orgMB_in_col,
                                num_of_orgMB_in_row, img->slice_sao_on, img->saoBlkParams[smb_index], img->rec_saoBlkParams[smb_index]);
        }

        if (input->alf_enable) {
            dP = &(currSlice->partArr[0]);
            for (compIdx = 0; compIdx < NUM_ALF_COMPONENT; compIdx++) {
                if (img->pic_alf_on[compIdx]) {
                    ctx_idx =  0;
                    Dec_ALF->m_AlfLCUEnabled[smb_index][compIdx] = readAlfLCUCtrl(img, &(dP->de_AEC), compIdx, ctx_idx);
                } else {
                    Dec_ALF->m_AlfLCUEnabled[smb_index][compIdx] = FALSE;
                }
            }
        }
        if (input->sao_enable || input->alf_enable) {
            smb_index++;
        }
        decode_SMB(input->g_uiMaxSizeInBit, img->current_mb_nr);
        if (img->current_mb_nr % img->PicWidthInMbs + N8_SizeScale >= img->PicWidthInMbs) {
            if (img->current_mb_nr / img->PicWidthInMbs + N8_SizeScale >= img->PicHeightInMbs) {
                img->current_mb_nr = img->max_mb_nr;
            } else {
                img->current_mb_nr = (img->current_mb_nr / img->PicWidthInMbs + N8_SizeScale) * img->PicWidthInMbs;
            }
        } else {
            img->current_mb_nr = img->current_mb_nr + N8_SizeScale;
        }

        aec_mb_stuffing_bit = AEC_startcode_follows(1);
    }

    free(Buf);


    if (!hd->loop_filter_disable) {
        CreateEdgeFilter();
        SetEdgeFilter();
        DeblockFrame(hc->imgY, hc->imgUV);
    }

    if (input->sao_enable) {
        Copy_frame_for_SAO();
        SAOFrame(input->g_uiMaxSizeInBit, img->rec_saoBlkParams, img->slice_sao_on,
                 input->sample_bit_depth);
    }

    if (input->alf_enable) {
        currAlfEnable = !(Dec_ALF->m_alfPictureParam[ALF_Y]->alf_flag == 0 &&
                          Dec_ALF->m_alfPictureParam[ALF_Cb]->alf_flag == 0 && Dec_ALF->m_alfPictureParam[ALF_Cr]->alf_flag == 0);

        if (currAlfEnable) {
            Copy_frame_for_ALF();
            ALFProcess_dec(Dec_ALF->m_alfPictureParam, img, hc->imgY_alf_Rec, hc->imgUV_alf_Rec, input->sample_bit_depth);

        }
    }
    if ((img->type == P_IMG || img->type == F_IMG) && hd->background_picture_enable && hd->background_reference_enable) {
        int j, i;
        for (j = 0; j < img->height / MIN_BLOCK_SIZE; j++)
            for (i = 0; i < img->width / MIN_BLOCK_SIZE; i++) {
                int refframe = hc->refFrArr[j][i];
                if (refframe == img->num_of_references - 1) {
                    hc->refFrArr[j][i] = -1;
                }
            }
    }
    if (img->type == P_IMG && img->typeb == BP_IMG && hd->background_picture_enable) {
        int j, i;
        for (j = 0; j < img->height / MIN_BLOCK_SIZE; j++) {
            for (i = 0; i < img->width / MIN_BLOCK_SIZE; i++) {
                hc->refFrArr[j][i] = -1;
            }
        }
    }

}
void Copy_frame_for_ALF()
{
    int i, j, k;

    for (j = 0; j < img->height; j++) {
        for (i = 0; i < img->width; i++) {
            hc->imgY_alf_Rec[j * (img->width) + i] = hc->imgY[j][i];
        }
    }

    for (k = 0; k < 2; k++) {
        for (j = 0; j < img->height_cr; j++) {
            for (i = 0; i < img->width_cr; i++) {
                hc->imgUV_alf_Rec[k][j * (img->width_cr) + i] = hc->imgUV[k][j][i];
            }
        }
    }
}

void min_tr(outdata data, int *pos)
{
    int i, tmp_min;
    tmp_min = data.stdoutdata[0].tr;
    *pos = 0;
    for (i = 1; i < data.buffer_num; i++) {
        if (data.stdoutdata[i].tr < tmp_min) {
            tmp_min = data.stdoutdata[i].tr;
            *pos = i;
        }
    }
}
void delete_trbuffer(outdata *data, int pos)
{
    int i;
    for (i = pos; i < data->buffer_num - 1; i++) {
        data->stdoutdata[i] = data->stdoutdata[i + 1];
    }
    data->buffer_num--;
}


void addCurrMvtoBuf()
{
    int k, x, y;

    for (k = 0; k < 2; k++) {
        for (y = 0; y < img->height / MIN_BLOCK_SIZE; y++) {
            for (x = 0; x < img->width / MIN_BLOCK_SIZE ; x++) {
                hc->f_rec->mvbuf[y][x][k] = ((img->type != B_IMG) ? img->tmp_mv : img->fw_mv)[y][x][k];
            }
        }
    }

    for (y = 0; y < img->height / MIN_BLOCK_SIZE; y++) {
        for (x = 0; x < img->width / MIN_BLOCK_SIZE ; x++) {
            hc->f_rec->refbuf[y][x] = ((img->type != B_IMG) ? hc->refFrArr : img->fw_refFrArr)[y][x];
        }
    }
}

// Sub-sampling of the stored reference index and motion vector

void compressMotion()
{
    int x, y;
    int width = img->width;
    int height = img->height;
    int xPos, yPos;

    for (y = 0; y < height / MIN_BLOCK_SIZE; y ++) {
        for (x = 0; x < width / MIN_BLOCK_SIZE; x ++) {
            yPos = y / MV_DECIMATION_FACTOR * MV_DECIMATION_FACTOR + 2;
            xPos = x / MV_DECIMATION_FACTOR * MV_DECIMATION_FACTOR + 2;

            if (yPos >= height / MIN_BLOCK_SIZE) {
                yPos = (y / MV_DECIMATION_FACTOR * MV_DECIMATION_FACTOR + height / MIN_BLOCK_SIZE) / 2;
            }
            if (xPos >= width / MIN_BLOCK_SIZE) {
                xPos = (x / MV_DECIMATION_FACTOR * MV_DECIMATION_FACTOR + width / MIN_BLOCK_SIZE) / 2;
            }
            hc->f_rec->mvbuf[y][x][0] = hc->f_rec->mvbuf[yPos][xPos][0];
            hc->f_rec->mvbuf[y][x][1] = hc->f_rec->mvbuf[yPos][xPos][1];
            hc->f_rec->refbuf[y][x]   = hc->f_rec->refbuf[yPos][xPos];
        }
    }
}
/*
*************************************************************************
* Function:Prepare field and frame buffer after frame decoding
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void frame_postprocessing()
{
    int pointer_tmp = outprint.buffer_num;
    int i;
    STDOUT_DATA *p_outdata;
#if RD160_FIX_BG
	int j,tmp_min,output_cur_dec_pic,pos = -1;
	int search_times = outprint.buffer_num;
#endif
    //pic dist by Grandview Semi. @ [06-07-20 15:25]
    img->PrevPicDistanceLsb = (img->coding_order % 256);
    if (hd->p_ref && !input->ref_pic_order) {
        if (img->typeb == BACKGROUND_IMG && hd->background_picture_output_flag == 0) {
            find_snr_background(snr, hd->p_ref_background);
        } else
            find_snr(snr, hd->p_ref);    // if ref sequence exist
    }

    pointer_tmp = outprint.buffer_num;
    p_outdata   = &outprint.stdoutdata[pointer_tmp];

    p_outdata->type = img->type;
    p_outdata->typeb = img->typeb;
    p_outdata->framenum = img->tr;
    p_outdata->tr = img->tr;
    p_outdata->qp = img->qp;
    p_outdata->snr_y = snr->snr_y;
    p_outdata->snr_u = snr->snr_u;
    p_outdata->snr_v = snr->snr_v;
    p_outdata->tmp_time = hd->tmp_time;
    p_outdata->picture_structure = img->picture_structure;
    p_outdata->curr_frame_bits = StatBitsPtr->curr_frame_bits;
    p_outdata->emulate_bits = StatBitsPtr->emulate_bits;
#if RD1501_FIX_BG
	p_outdata->background_picture_output_flag=hd->background_picture_output_flag;//Longfei.Wang@mediatek.com
#endif

#if	RD160_FIX_BG
	p_outdata->picture_reorder_delay = hd->picture_reorder_delay;
#endif
    outprint.buffer_num++;

    // record the reference list
    strcpy(p_outdata->str_reference_list, hc->str_list_reference);

    if (input->MD5Enable & 0x02) {
        int j, k;
        int img_width = (img->width - img->auto_crop_right);
        int img_height = (img->height - img->auto_crop_bottom);
        int img_width_cr = (img_width / 2);
        int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));
        int nSampleSize = input->output_bit_depth == 8 ? 1 : 2;
        int shift1 = input->sample_bit_depth - input->output_bit_depth;
        unsigned char *pbuf;
        unsigned char *md5buf;
        md5buf = (unsigned char *)malloc(img_height * img_width * nSampleSize + img_height_cr * img_width_cr * nSampleSize * 2);

        if (md5buf != NULL) {
            if (!shift1 && input->output_bit_depth == 8) { // 8bit input -> 8bit encode
                pbuf = md5buf;
                for (j = 0; j < img_height; j++)
                    for (k = 0; k < img_width; k++) {
                        pbuf[j * img_width + k] = (unsigned char)hc->imgY[j][k];
                    }

                pbuf = md5buf + img_height * img_width * sizeof(unsigned char);
                for (j = 0; j < img_height_cr; j++)
                    for (k = 0; k < img_width_cr; k++) {
                        pbuf[j * img_width_cr + k] = (unsigned char)hc->imgUV[0][j][k];
                    }

                pbuf = md5buf + img_height * img_width * sizeof(unsigned char) + img_height_cr * img_width_cr * sizeof(unsigned char);
                for (j = 0; j < img_height_cr; j++)
                    for (k = 0; k < img_width_cr; k++) {
                        pbuf[j * img_width_cr + k] = (unsigned char)hc->imgUV[1][j][k];
                    }
            } else if (!shift1 && input->output_bit_depth > 8) { // 10/12bit input -> 10/12bit encode
                pbuf = md5buf;
                for (j = 0; j < img_height; j++) {
                    memcpy(pbuf + j * img_width * nSampleSize, &(hc->imgY[j][0]), img_width * sizeof(byte));
                }

                pbuf = md5buf + img_height * img_width * sizeof(byte);
                for (j = 0; j < img_height_cr; j++) {
                    memcpy(pbuf + j * img_width_cr * nSampleSize, &(hc->imgUV[0][j][0]), img_width_cr * sizeof(byte));
                }

                pbuf = md5buf + img_height * img_width * sizeof(byte) + img_height_cr * img_width_cr * sizeof(byte);
                for (j = 0; j < img_height_cr; j++) {
                    memcpy(pbuf + j * img_width_cr * nSampleSize, &(hc->imgUV[1][j][0]), img_width_cr * sizeof(byte));
                }
            } else if (shift1 && input->output_bit_depth == 8) { // 8bit input -> 10/12bit encode
                pbuf = md5buf;
                for (j = 0; j < img_height; j++)
                    for (k = 0; k < img_width; k++) {
                        pbuf[j * img_width + k] = (unsigned char)Clip1((hc->imgY[j][k] + (1 << (shift1 - 1))) >> shift1);
                    }

                pbuf = md5buf + img_height * img_width * sizeof(unsigned char);
                for (j = 0; j < img_height_cr; j++)
                    for (k = 0; k < img_width_cr; k++) {
                        pbuf[j * img_width_cr + k] = (unsigned char)Clip1((hc->imgUV[0][j][k] + (1 << (shift1 - 1))) >> shift1);
                    }

                pbuf = md5buf + img_height * img_width * sizeof(unsigned char) + img_height_cr * img_width_cr * sizeof(unsigned char);
                for (j = 0; j < img_height_cr; j++)
                    for (k = 0; k < img_width_cr; k++) {
                        pbuf[j * img_width_cr + k] = (unsigned char)Clip1((hc->imgUV[1][j][k] + (1 << (shift1 - 1))) >> shift1);
                    }
            }
            BufferMD5(md5buf, img_height * img_width * nSampleSize + img_height_cr * img_width_cr * nSampleSize * 2, MD5val);
        } else {
            printf("malloc md5 buffer error!\n");
            memset(MD5val, 0, 16);
        }
        if (md5buf) {
            free(md5buf);
        }
    } else {
        memset(MD5val, 0, 16);
    }
    {
        int j;
        for (j = 0; j < 4; j++) {
            p_outdata->DecMD5Value[j] = MD5val[j];
        }
    }

    if (hd->curr_RPS.referd_by_others && img->type != I_IMG) {
        addCurrMvtoBuf();
        compressMotion();
    }
#if !REF_OUTPUT
    for (i = 0; i < outprint.buffer_num; i++) {
        min_tr(outprint, &pos);
        if (outprint.stdoutdata[pos].tr < img->tr ||  outprint.stdoutdata[pos].tr == (hd->last_output + 1)) {
            hd->last_output = outprint.stdoutdata[pos].tr;
            report_frame(outprint, pos);
            write_frame(hd->p_out, outprint.stdoutdata[pos].tr);
            delete_trbuffer(&outprint, pos);
            i--;
        } else {
            break;
        }
    }
#else
#if RD160_FIX_BG //Longfei.Wang@mediatek.com
	tmp_min = 1 << 20;
	i = 0, j = 0;
	output_cur_dec_pic = 0;
	pos = -1;
	for (j = 0; j < search_times; j++) {
		pos = -1;
		tmp_min = (1 << 20);
		//search for min poi picture to display
		for (i = 0; i < outprint.buffer_num; i++) {
			if ((outprint.stdoutdata[i].tr < tmp_min) &&
				((outprint.stdoutdata[i].tr + outprint.stdoutdata[i].picture_reorder_delay) <= (int)img->coding_order)) {
					pos = i;
					tmp_min = outprint.stdoutdata[i].tr;
			}
		}

		if ((0 == hd->displaydelay) && (0 == output_cur_dec_pic)) {
			if (img->tr < tmp_min) {
				//output current decode picture right now
				pos = outprint.buffer_num - 1;
				output_cur_dec_pic = 1;
			}
		}

		if (pos != -1) {
			hd->last_output = outprint.stdoutdata[pos].tr;
			report_frame(outprint, pos);
			if (outprint.stdoutdata[pos].typeb == BACKGROUND_IMG && outprint.stdoutdata[pos].background_picture_output_flag == 0) {
				write_GB_frame(hd->p_out_background);
			}
			else {
				write_frame(hd->p_out, outprint.stdoutdata[pos].tr);
			}
			delete_trbuffer(&outprint, pos);
		}
	}

#else
    if (img->coding_order + (unsigned int)hc->total_frames * 256 >= (unsigned int)hd->picture_reorder_delay) 

	{
        int tmp_min, pos = -1;
        tmp_min = 1 << 20;

        for (i = 0; i < outprint.buffer_num; i++) {
            if (outprint.stdoutdata[i].tr < tmp_min &&
                outprint.stdoutdata[i].tr >= hd->last_output) { //GB has the same "tr" with "last_output"

                pos = i;
                tmp_min = outprint.stdoutdata[i].tr;

            }
        }

        if (pos != -1) {
            hd->last_output = outprint.stdoutdata[pos].tr;
            report_frame(outprint, pos);
			#if RD1501_FIX_BG
			 if (outprint.stdoutdata[pos].typeb == BACKGROUND_IMG && outprint.stdoutdata[pos].background_picture_output_flag == 0) {
          #else
            if (outprint.stdoutdata[pos].typeb == BACKGROUND_IMG && hd->background_picture_output_flag == 0) {
           #endif
                write_GB_frame(hd->p_out_background);
            } else {
                write_frame(hd->p_out, outprint.stdoutdata[pos].tr);
            }
            delete_trbuffer(&outprint, pos);

        }

    }
#endif
#endif
}


/*
******************************************************************************
*  Function: Determine the MVD's value (1/4 pixel) is legal or not.
*  Input:
*  Output:
*  Return:   0: out of the legal mv range; 1: in the legal mv range
*  Attention:
*  Author: Xiaozhen ZHENG, rm52k
******************************************************************************
*/
void DecideMvRange()
{

    if (input->profile_id == BASELINE_PROFILE || input->profile_id == BASELINE10_PROFILE) {
        switch (input->level_id) {
        case 0x10:
            hc->Min_V_MV = -512;
            hc->Max_V_MV = 511;
            hc->Min_H_MV = -8192;
            hc->Max_H_MV = 8191;
            break;
        case 0x20:
            hc->Min_V_MV = -1024;
            hc->Max_V_MV = 1023;
            hc->Min_H_MV = -8192;
            hc->Max_H_MV = 8191;
            break;
        case 0x22:
            hc->Min_V_MV = -1024;
            hc->Max_V_MV = 1023;
            hc->Min_H_MV = -8192;
            hc->Max_H_MV = 8191;
            break;
        case 0x40:
            hc->Min_V_MV = -2048;
            hc->Max_V_MV = 2047;
            hc->Min_H_MV = -8192;
            hc->Max_H_MV = 8191;
            break;
        case 0x42:
            hc->Min_V_MV = -2048;
            hc->Max_V_MV = 2047;
            hc->Min_H_MV = -8192;
            hc->Max_H_MV = 8191;
            break;
        }
    }
}
