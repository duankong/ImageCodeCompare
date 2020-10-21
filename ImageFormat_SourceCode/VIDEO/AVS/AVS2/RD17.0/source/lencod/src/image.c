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

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/memalloc.h"
#include "../../lcommon/inc/inter-prediction.h"
#include "../../lcommon/inc/md5.h"
#include "image.h"
#include "refbuf.h"
#include "header.h"
#include "bitstream.h"
#include "vlc.h"
#include "configfile.h"
#include "AEC.h"

#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif

#if TDRDO
#include "tdrdo.h"
#endif

#if RATECONTROL
#include "ratecontrol.h"
#endif



extern unsigned int   MD5val[4];
extern char           MD5str[33];


#include "pos_info.h" //ZHAOHAIWU

#define TO_SAVE 4711
#define FROM_SAVE 4712
#define Clip(min,max,val) (((val)<(min))?(min):(((val)>(max))?(max):(val)))

static void code_a_picture(Picture *frame);

static void ReadOneFrame(int FrameNoInFile, int FrameSkip, int xs, int ys);


static void write_reconstructed_image();
static int  writeout_picture();
static int  writeout_slice();
static void find_snr();
static void init_frame();

int terminate_picture();

void copy_Pframe();
void write_prev_Pframe();

void put_buffer_frame();

static void CopyFrameToOldImgOrgVariables();
static void UnifiedOneForthPix(pel_t **imgY, pel_t **imgU, pel_t **imgV,
                               pel_t **out4Y);
//static void ReportIntra ( int tmp_time );
//static void ReportP ( int tmp_time );
//static void ReportB ( int tmp_time );
static void Report_frame(int tmp_time);

static int CalculateFrameNumber();  // Calculates the next frame number
static int FrameNumberInFile;       // The current frame number in the input file

extern void DecideMvRange();  // mv_range, 20071009

#define  IClip( Min, Max, Val) (((Val)<(Min))? (Min):(((Val)>(Max))? (Max):(Val)))



/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void reduce_yuv_dim(byte *imgYuv_buffer, byte **imgY, byte ***imgUV, int img_width, int img_height, int img_width_cr,
                    int img_height_cr)
{
    int j;
    for (j = 0; j < img_height; j++) {
        memcpy(imgYuv_buffer + j * img_width, &(imgY[j][0]), img_width * sizeof(byte));
    }

    imgYuv_buffer += img_height * img_width;
    for (j = 0; j < img_height_cr; j++) {
        memcpy(imgYuv_buffer + j * img_width_cr, &(imgUV[0][j][0]), img_width_cr * sizeof(byte));
    }

    imgYuv_buffer += img_width_cr * img_height_cr;
    for (j = 0; j < img_height_cr; j++) {
        memcpy(imgYuv_buffer + j * img_width_cr, &(imgUV[1][j][0]), img_width_cr * sizeof(byte));
    }
}

void write_yuv_shift(FILE *p_out, byte *img, unsigned char *buf, int length, int shift1)
{
    int j;

    for (j = 0; j < length; j++) {
        buf[j] = (unsigned char)Clip1((img[j] + (1 << (shift1 - 1))) >> shift1);
    }

    fwrite(buf, sizeof(unsigned char), length, p_out);
}

void write_yuv_byte(FILE *p_out, byte *img, unsigned char *buf, int length)
{
    fwrite(img, sizeof(byte), length, p_out);
}

void write_yuv_uchar(FILE *p_out, byte *img, unsigned char *buf, int length)
{
    int j;

    for (j = 0; j < length; j++) {
        buf[j] = (unsigned char)img[j];
    }

    fwrite(buf, sizeof(unsigned char), length, p_out);
}

void write_yuv_frame(FILE *p_out, byte *img, int img_length)
{
    int nSampleSize;
    int shift1;
    unsigned char *buf;

    if (input->input_sample_bit_depth == 8) {
        nSampleSize = 1;
    } else if (input->input_sample_bit_depth > 8) {
        nSampleSize = 2;
    }
    shift1 = input->sample_bit_depth - input->input_sample_bit_depth;
    buf = malloc(img_length * nSampleSize);

    if (!shift1 && input->input_sample_bit_depth == 8) { // 8bit input -> 8bit encode
        write_yuv_uchar(p_out, img, buf, img_length);
    } else if (!shift1 && input->input_sample_bit_depth > 8) { // 10/12bit input -> 10/12bit encode
        write_yuv_byte(p_out, img, buf, img_length);
    } else if (shift1 && input->input_sample_bit_depth == 8) { // 8bit input -> 10/12bit encode
        write_yuv_shift(p_out, img, buf, img_length, shift1);
    }
    free(buf);
    fflush(p_out);
}

static void picture_header()
{
    int len = 0;

    he->current_slice_bytepos = currBitStream->byte_pos;
    he->current_slice_bytepos_alf = he->current_slice_bytepos;

    if (img->type == INTRA_IMG) {
        len = len + IPictureHeader(img->number);
    } else {
        len = len + PBPictureHeader();
    }

    // Bug fix
    if (input->alf_enable) {
        BitStreamCopy(currBitStream_ALF, currBitStream);
    }
    demulateFunction();

    // Update statistics
    stat->bit_slice += len;
    stat->bit_use_header[img->type] += len;
}

void picture_header_ALF(ALFParam **alfPictureParam)
{
    int len = 0, compIdx;

    if (input->alf_enable) {
        len += u_v(1, "alf_pic_flag_Y", img->pic_alf_on[0], currBitStream);
        len += u_v(1, "alf_pic_flag_Cb", img->pic_alf_on[1], currBitStream);
        len += u_v(1, "alf_pic_flag_Cr", img->pic_alf_on[2], currBitStream);
    }


    if (img->pic_alf_on[0] || img->pic_alf_on[1] || img->pic_alf_on[2]) {
        for (compIdx = 0; compIdx < NUM_ALF_COMPONENT; compIdx++) {
            if (img->pic_alf_on[compIdx]) {
                writeAlfCoeff(alfPictureParam[compIdx]);
            }
        }
    }

    stat->bit_slice += len;
    stat->bit_use_header[img->type] += len;
}

void report_frame(int tmp_time)
{
    if (input->MD5Enable & 0x02) {
        int j, k;
        int img_width = (img->width - img->auto_crop_right);
        int img_height = (img->height - img->auto_crop_bottom);
        int img_width_cr = (img_width / 2);
        int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));
        int nSampleSize = input->input_sample_bit_depth == 8 ? 1 : 2;
        int shift1 = input->sample_bit_depth - input->input_sample_bit_depth;
        unsigned char *pbuf;
        unsigned char *md5buf;
        md5buf = (unsigned char *)malloc(img_height * img_width * nSampleSize + img_height_cr * img_width_cr * nSampleSize * 2);

        if (md5buf != NULL) {
            if (!shift1 && input->input_sample_bit_depth == 8) { // 8bit input -> 8bit encode
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
            } else if (!shift1 && input->input_sample_bit_depth > 8) { // 10/12bit input -> 10/12bit encode
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
            } else if (shift1 && input->input_sample_bit_depth == 8) { // 8bit input -> 10/12bit encode
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
        sprintf(MD5str, "%08X%08X%08X%08X\0", MD5val[0], MD5val[1], MD5val[2], MD5val[3]);
    } else {
        memset(MD5val, 0, 16);
        memset(MD5str, 0, 33);
    }
    if (img->number == 0) {
        Report_frame(tmp_time);

        stat->bitr0 = stat->bitr;
#if 1
        switch (img->type) {
        case INTRA_IMG:
            if (input->bg_enable && img->typeb == BACKGROUND_IMG && he->duplicated_gb_flag == 1) {
                stat->bit_ctr_dup_gb += stat->bit_ctr - stat->bit_ctr_n;
            }
            stat->bit_ctr_0 = stat->bit_ctr;
            break;
        case B_IMG:
            stat->bit_ctr_B = stat->bit_ctr;
            break;
        default:
            stat->bit_ctr_P = stat->bit_ctr;
            break;
        }
#else
        stat->bit_ctr_0 = stat->bit_ctr;
#endif
        //  stat->bit_ctr = 0;   //rm52k
    } else {

        switch (img->type) {
        case INTRA_IMG:
            if (input->bg_enable && img->typeb == BACKGROUND_IMG && he->duplicated_gb_flag == 1) {
                stat->bit_ctr_dup_gb += stat->bit_ctr - stat->bit_ctr_n;
            }
            stat->bit_ctr_0 += stat->bit_ctr - stat->bit_ctr_n;
            break;
        case B_IMG:
            stat->bit_ctr_B += stat->bit_ctr - stat->bit_ctr_n;
            break;
        default:
            stat->bit_ctr_P += stat->bit_ctr - stat->bit_ctr_n;
            break;
        }
        Report_frame(tmp_time);
    }
    fflush(stdout);
}

/*
*************************************************************************
* Function:Encodes one frame
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

int encode_one_frame()
{
    time_t ltime1;
    time_t ltime2;
#ifdef WIN32
    struct _timeb tstruct1;
    struct _timeb tstruct2;
#else
    struct timeb tstruct1;
    struct timeb tstruct2;
#endif
    int tmp_time;

#ifdef WIN32
    _ftime(&tstruct1);            // start time ms
#else
    ftime(&tstruct1);
#endif
    time(&ltime1);                // start time s

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    if ((input->WQEnable) && (input->PicWQEnable)) {
        InitFrameQuantParam();
        FrameUpdateWQMatrix();        //M2148 2007-09
    }
#endif

    if (img->typeb != BACKGROUND_IMG || input->bg_input_number != 0) { //I or P or output G
        init_frame();
        FrameNumberInFile = CalculateFrameNumber();
        ReadOneFrame(FrameNumberInFile, input->infile_frameskip, input->img_width,
                     input->img_height);  //modify by wuzhongmou 0610


        if (img->typeb == BACKGROUND_IMG && he->background_output_flag == 1) {
            he->gb_is_ready = 0;
            if (input->InterlaceCodingOption == 3) {
                memcpy(he->imgY_GB_org_buffer, he->imgY_org_buffer,
                       sizeof(byte)*input->org_img_height * input->org_img_width * 3 / 2);  //back up the org GB
            } else {
                memcpy(he->imgY_GB_org_buffer, he->imgY_org_buffer, sizeof(byte)*img->width * img->height * 3 / 2); //back up the org GB
            }
        }
    }

    if (img->typeb == BACKGROUND_IMG && input->bg_input_number == 0 && img->number != 0 &&
        input->always_no_bgmodel == 0) { //not output G
        if (he->g_bg_number >= input->bg_model_number / 2
            || he->duplicated_gb_flag == 1
           ) {
            init_frame();
            FrameNumberInFile = CalculateFrameNumber();
            if (he->duplicated_gb_flag == 1) {
                if (input->InterlaceCodingOption == 3) {
                    memcpy(he->imgY_org_buffer, he->imgY_GB_org_buffer,
                           sizeof(byte)*input->org_img_height * input->org_img_width * 3 / 2);  //encode the duplicated GB
                } else {
                    memcpy(he->imgY_org_buffer, he->imgY_GB_org_buffer,
                           sizeof(byte)*img->width * img->height * 3 / 2); //encode the duplicated GB
                }
            } else {
                bg_build(he->imgY_org_buffer);
#if RD1510_FIX_BG
                if (input->InterlaceCodingOption == 3) {
                    memcpy(he->imgY_GB_org_buffer, he->imgY_org_buffer, sizeof(byte)*input->org_img_height * input->org_img_width * 3 / 2);  //back up the org GB
                } else {
                    memcpy(he->imgY_GB_org_buffer, he->imgY_org_buffer, sizeof(byte)*img->width*img->height * 3 / 2);  //back up the org GB
                }
#else
                if (input->InterlaceCodingOption == 3) {
                    memcpy(he->imgY_org_buffer, he->imgY_GB_org_buffer,
                           sizeof(byte)*input->org_img_height * input->org_img_width * 3 / 2);  //encode the duplicated GB
                } else {
                    memcpy(he->imgY_org_buffer, he->imgY_GB_org_buffer,
                           sizeof(byte)*img->width * img->height * 3 / 2); //encode the duplicated GB
                }
#endif
                he->gb_is_ready = 1;
            }
#if AVS2_S2_FASTMODEDECISION
            if (input->bg_input_number == 1) {
                he->bg_flag = 1;
            }
#endif
            if (he->duplicated_gb_flag == 0) {
                he->last_input_frame = FrameNumberInFile;
                bg_releaseModel();
                he->train_start = 0;
                printf("\ntrain close g_bg_number=%d\n", he->g_bg_number);
            }

            if (he->p_org_background) {
                write_GB_org_frame(he->p_org_background);
            }

        } else {
            printf("\nnot surveillance g_bg_number=%d\n", he->g_bg_number);
            bg_releaseModel();
            input->always_no_bgmodel = 1;
            input->bg_input_number = 1;
            he->background_number = 0;
            he->background_output_flag = 1;
            init_frame();
            FrameNumberInFile = CalculateFrameNumber();
        }
    }


#if TDRDO
    if (input->TDEnable != 0) {
        GlobeFrameNumber = img->number; //FrameNumberInFile;
#if AQPO
        if (input->AQPEnable != 0 && !input->successive_Bframe) {
            FNIndex = GlobeFrameNumber % AQPStepLength;
            if (FNIndex == 0) {
                FNIndex = AQPStepLength - 1;
            } else {
                FNIndex = FNIndex - 1;
            }
            if (GlobeFrameNumber > 0) {
                preLogMAD[FNIndex] = LogMAD[FNIndex];
            }
        }
#endif
        if (input->successive_Bframe_all) {
            pRealFD = &RealDList->FrameDistortionArray[GlobeFrameNumber];
        } else {
            pRealFD = &RealDList->FrameDistortionArray[he->globenumber];
        }
        pRealFD->BlockDistortionArray = (BD *)calloc(pRealFD->TotalNumOfBlocks, sizeof(BD));
        if (GlobeFrameNumber % StepLength == 0) {
            if (FrameNumberInFile == 0) {
                porgF->base = he->imgY_org_buffer;
                porgF->Y = porgF->base;
                porgF->U = porgF->Y + porgF->FrameWidth * porgF->FrameHeight;
                porgF->V = porgF->U + porgF->FrameWidth * porgF->FrameHeight / 4 ;
                ppreF->base = he->imgY_pre_buffer;
                ppreF->Y = ppreF->base;
                ppreF->U = ppreF->Y + ppreF->FrameWidth * ppreF->FrameHeight;
                ppreF->V = ppreF->U + ppreF->FrameWidth * ppreF->FrameHeight / 4 ;
                memcpy(he->imgY_pre_buffer, he->imgY_org_buffer, input->img_width * input->img_height * 3 / 2 * sizeof(byte));
            } else  if (FrameNumberInFile < input->no_frames) {
                pOMCPFD = &OMCPDList->FrameDistortionArray[GlobeFrameNumber - 1]; //FrameNumberInFile-1
                pOMCPFD->BlockDistortionArray = (BD *)calloc(pOMCPFD->TotalNumOfBlocks, sizeof(BD));
                MotionDistortion(pOMCPFD, ppreF, porgF, SEARCHRANGE);
                memcpy(he->imgY_pre_buffer, he->imgY_org_buffer, input->img_width * input->img_height * 3 / 2 * sizeof(byte));

#if AQPO
                {
                    unsigned int b;
                    LogMAD[FNIndex] = 0.0;
                    for (b = 0; b < pOMCPFD->TotalNumOfBlocks; b++) {
                        LogMAD[FNIndex] += pOMCPFD->BlockDistortionArray[b].MAD;
                    }
                    LogMAD[FNIndex] = log(LogMAD[FNIndex]);
                }
#endif
            }
            pOMCPFD = NULL;
        }
    }
#endif

#if AQPO
    if (input->AQPEnable && !input->successive_Bframe) {
        if (FrameNumberInFile > AQPStepLength && img->type != 0) {
            FNIndex = GlobeFrameNumber % AQPStepLength;
            if (FNIndex == 0) {
                FNIndex = AQPStepLength - 1;
            } else {
                FNIndex = FNIndex - 1;
            }
#if AQPOM3762
            if (FNIndex % 2 != 0) {
                int i;
                double sumpreLogMAD;
                int deltaQP;
                int gopdeltaQp;
                double GopdeltaQp;
                double sumLogMAD = 0.0;
                int factor = FNIndex == (AQPStepLength - 1) ? 180 : 105;

                sumpreLogMAD = 0.0;
                for (i = 0; i < AQPStepLength; i++) {
                    sumpreLogMAD += preLogMAD[i];
                }

                if (input->AQPEnable & 0x01) {
                    if (FNIndex == 3) {
                        GopdeltaQp = ((LogMAD[FNIndex] - preLogMAD[FNIndex]) / (sumpreLogMAD + sumLogMAD) * 8 * 130);
                        gopdeltaQp = (GopdeltaQp < 0 ? (int)(GopdeltaQp - 0.5) : (int)(GopdeltaQp + 0.5));
                        gopdeltaQp = min(2, max(-2, gopdeltaQp));
                        GopQpbase = (int)(GopQpbase + gopdeltaQp);
                        if ((GlobeFrameNumber / 4) % AQPStepLength == 0) {
                            GopQpbase = min(input->qpI + 1, max(input->qpI - 2, GopQpbase));
                            preGopQPF[0] = GopQpbase;
                        } else if ((GlobeFrameNumber / 4) % AQPStepLength == 1) {
                            GopQpbase = max(input->qpI - 1, min(input->qpI + 2, max(preGopQPF[0], GopQpbase)));
                            preGopQPF[1] = GopQpbase;
                        } else if ((GlobeFrameNumber / 4) % AQPStepLength == 2) {
                            GopQpbase = max(input->qpI, min(preGopQPF[1] - 1, max(preGopQPF[0], GopQpbase)));
                            preGopQPF[2] = GopQpbase;
                        } else if ((GlobeFrameNumber / 4) % AQPStepLength == 3) {
                            GopQpbase = max(input->qpI - 1, min(input->qpI + 2, max(preGopQPF[2], GopQpbase)));
                            preGopQPF[3] = GopQpbase;
                        }
                    }
                }

                if (input->AQPEnable & 0x02) {
                    deltaQP = (int)((LogMAD[FNIndex] - preLogMAD[FNIndex]) / (sumpreLogMAD + LogMAD[FNIndex]) * 5 * factor);
                    AQPoffset[FNIndex] = he->cfg_ref[FNIndex].qp_offset + deltaQP;
                    if (AQPStepLength >= 4)
                        if (FNIndex == AQPStepLength - 1) {
                            AQPoffset[FNIndex] = min(AQPoffset[FNIndex - 2], max(1, AQPoffset[FNIndex]));
                        } else {
                            AQPoffset[FNIndex] = min(MaxQPoffset  , max(2, AQPoffset[FNIndex]));
                        }
                }

            }
            QpOffset[FNIndex] = AQPoffset[FNIndex];

            if (input->AQPEnable & 0x01) {
                img->qp = GopQpbase + AQPoffset[OffsetIndex];
            }

            else if (input->AQPEnable & 0x02) {
                img->qp = input->qpI + AQPoffset[OffsetIndex];
            }
#else
            if (FNIndex % 2 != 0) {
                int i;
                double sumpreLogMAD = 0.0;
                int deltaQP;
                int factor = FNIndex == (AQPStepLength - 1) ? 180 : 105;
                for (i = 0; i < AQPStepLength; i++) {
                    sumpreLogMAD += preLogMAD[i];
                }

                deltaQP = (int)((LogMAD[FNIndex] - preLogMAD[FNIndex]) / (sumpreLogMAD + LogMAD[FNIndex]) * 5 * factor);
                AQPoffset[FNIndex] = he->cfg_ref[FNIndex].qp_offset + deltaQP;

                if (AQPStepLength >= 4)
                    if (FNIndex == AQPStepLength - 1) {
                        AQPoffset[FNIndex] = min(AQPoffset[FNIndex - 2], max(1, AQPoffset[FNIndex]));
                    } else {
                        AQPoffset[FNIndex] = min(MaxQPoffset  , max(2, AQPoffset[FNIndex]));
                    }
            }
            QpOffset[FNIndex] = AQPoffset[FNIndex];
            img->qp = input->qpI + AQPoffset[OffsetIndex];
#endif
        }
    }
#endif

    CopyFrameToOldImgOrgVariables();//image padding

#if TDRDO
    if (input->TDEnable != 0 && GlobeFrameNumber % StepLength == 0 && GlobeFrameNumber < input->no_frames - 1  &&
        input->intra_period == 0) {
        CaculateKappaTableLDP(OMCPDList, RealDList, GlobeFrameNumber, img->qp);
    }
#endif

#if RATECONTROL
    if (input->EncControl == 1) {
        if (pRC->IntraPeriod == 0) {
            if (FrameNumberInFile % he->gop_size_all == 1) {
                pRC->DeltaQP = CalculateGopDeltaQP_RateControl(pRC, img->type, FrameNumberInFile, he->gop_size_all);
            }
        } else if (pRC->IntraPeriod == 1) {
            if ((FrameNumberInFile % he->gop_size_all == 0) && (FrameNumberInFile != 0)) {
                pRC->DeltaQP = CalculateGopDeltaQP_RateControl(pRC, img->type, FrameNumberInFile, he->gop_size_all);
            }
        } else {
            if ((img->type == 0) && ((pRC->TotalFrames - pRC->CodedFrameNumber) <= (2 * pRC->IntraPeriod * he->gop_size_all))) {
                Init_FuzzyController(0.50);//enhance adjusting strength of the last GOP group by lmk
            }
            if ((FrameNumberInFile % he->gop_size_all == 0) && (FrameNumberInFile != 0)) {
                pRC->DeltaQP = CalculateGopDeltaQP_RateControl(pRC, img->type, FrameNumberInFile, he->gop_size_all);
            } else if (pRC->TotalFrames - pRC->CodedFrameNumber == (pRC->TotalFrames - 1) % he->gop_size_all) {
                pRC->DeltaQP = CalculateGopDeltaQP_RateControl(pRC, img->type, FrameNumberInFile, he->gop_size_all);
            }
        }

        if ((pRC->CodedFrameNumber % he->gop_size_all == 1) || (pRC->IntraPeriod == 1)) {
            if (pRC->IntraPeriod > 1 && img->type == 0) {
                int remainGOPnum;
                if ((pRC->TotalFrames - pRC->CodedFrameNumber) > (pRC->IntraPeriod * he->gop_size_all + he->gop_size_all)) {
                    remainGOPnum = pRC->IntraPeriod;
                } else {
                    remainGOPnum = (int)ceil(1.0F * (pRC->TotalFrames - pRC->CodedFrameNumber) / he->gop_size_all);
                }

                // handle final GOP by ymzhou
                if ((pRC->TotalFrames - pRC->CodedFrameNumber) <= (pRC->IntraPeriod * he->gop_size_all + he->gop_size_all)) {
                    if ((1.0 * remainGOPnum / pRC->IntraPeriod) <= 1.0 / 3) {
                        pRC->DeltaQP += 8;    //as bitrate halve
                    } else if ((1.0 * remainGOPnum / pRC->IntraPeriod) <= 1.0 / 2) {
                        pRC->DeltaQP += 3;
                    } else if ((1.0 * remainGOPnum / pRC->IntraPeriod) <= 2.0 / 3) {
                        pRC->DeltaQP += 4;
                    } else if ((1.0 * remainGOPnum / pRC->IntraPeriod) <= 3.0 / 4) {
                        pRC->DeltaQP += 0;
                    } else if ((1.0 * remainGOPnum / pRC->IntraPeriod) <= 4.0 / 4) {
                        pRC->DeltaQP += 2;
                    } else {
                        pRC->DeltaQP += 0;
                    }
                }

                img->qp = pRC->GopAvgBaseQP / pRC->GopAvgBaseQPCount + pRC->DeltaQP;
                input->qpI = img->qp;
            } else {
                img->qp    += pRC->DeltaQP;
                input->qpI += pRC->DeltaQP;
            }
            img->qp    = max(MIN_QP, min(img->qp,    MAX_QP + (input->sample_bit_depth - 8) * 8 - 5));
            input->qpI = max(MIN_QP, min(input->qpI, MAX_QP + (input->sample_bit_depth - 8) * 8 - 5));
            input->qpP = input->qpI + 1;
            input->qpB = input->qpI + 4;
        }
        pRC->RcQP = input->qpI;
    }
#endif

#if !INTERLACE_CODING
    if (input->InterlaceCodingOption == FRAME_CODING)   // !! frame coding
#endif
    {
        put_buffer_frame();      //initialize frame buffer
        //frame picture
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3) {
            img->is_top_field = (img->tr + hc->total_frames * 256) % 2 == 0;
        }
#endif
#if INTERLACE_CODING_FIX
        if (input->InterlaceCodingOption == 0) { // frame picture coding
            if (input->progressive_frame) {
                img->progressive_frame = 1;
                img->picture_structure = 1;
            } else {
                img->progressive_frame = 0;
                img->picture_structure = 1;
            }
        } else if (input->InterlaceCodingOption == 3) { // field picture coding
            img->progressive_frame = 0;
            img->picture_structure = 0;
        }
#else
        if (input->progressive_frame) {   //add by wuzhongmou 0610
            img->progressive_frame = 1;
            img->picture_structure = 1;
        } else {
            img->progressive_frame = 0;
            img->picture_structure = 1;
        }                 //add by wuzhongmou 0610
#endif
        DecideMvRange();  // 20071224


        if (img->type == B_IMG) {
            hc->Bframe_ctr++;  // Bframe_ctr only used for statistics, should go to stat->
        }

        code_a_picture(frame_pic);

        if (img->type == B_IMG) {
            fref[0]->imgtr_fwRefDistance = he->trtmp;
        }
        if (he->curr_RPS.referd_by_others) {
            int i;
            for (i = 0; i < REF_MAXBUFFER; i++) {
                if (img->tr == fref[i]->imgtr_fwRefDistance) {
                    break;
                }
            }
            if (!(img->typeb == BACKGROUND_IMG && he->background_output_flag == 0))
                UnifiedOneForthPix(hc->imgY, hc->imgUV[0], hc->imgUV[1], fref[i]->oneForthRefY);
        }


        if (img->typeb == BACKGROUND_IMG && input->bg_enable) {
            int l;
            UnifiedOneForthPix(hc->imgY, hc->imgUV[0], hc->imgUV[1], he->background_frame_quarter);
            for (l = 0; l < img->height; l++) {
                memcpy(he->background_frame[0][l], hc->imgY[l], sizeof(byte) * img->width);
            }
            for (l = 0; l < img->height_cr; l++) {
                memcpy(he->background_frame[1][l], hc->imgUV[0][l], sizeof(byte) * img->width_cr);
                memcpy(he->background_frame[2][l], hc->imgUV[1][l], sizeof(byte) * img->width_cr);
            }
        }


    }

    writeout_picture(); //qyu delete PAFF

    if (input->bg_enable && input->always_no_bgmodel == 0
        && !(img->typeb == BACKGROUND_IMG && he->background_output_flag == 0)
       ) {
        if ((img->number == 0 ||
             FrameNumberInFile - he->last_input_frame + input->bg_model_number >= input->bg_period *
             (1 + input->successive_Bframe) &&
             he->background_number < he->total_bg_number && he->train_start == 0)) { //total_bg_number
            bg_allocModel(he->imgY_org_buffer, input->img_width, input->img_height);
            he->train_start = 1;
            printf("\ntrain start\n");
        } else {
            if (he->train_start == 1 && input->always_no_bgmodel == 0) {
                bg_insert(he->imgY_org_buffer);
            }
        }
    }


    free_slice();
    FreeBitstream_ALF();
    FreeBitstream();

    find_snr();

#if TDRDO
    if (input->TDEnable != 0) {
        int DelFDNumber;
        FD *pDelFD;
        precF->base = NULL;
        precF->Y = hc->imgY[0];
        precF->U = hc->imgUV[0][0];
        precF->V = hc->imgUV[1][0];
        if ((FrameNumberInFile % StepLength == 0 && !input->successive_Bframe_all) || input->successive_Bframe_all) {
            MotionDistortion(pRealFD, porgF, precF, 0);
        }
        pRealFD->FrameNumber = FrameNumberInFile;
        he->globenumber++;

        DelFDNumber = he->globenumber - StepLength - 1;
        if (DelFDNumber >= 0) {
            pDelFD = &RealDList->FrameDistortionArray[DelFDNumber];
            if (pDelFD->BlockDistortionArray != NULL) {
                free(pDelFD->BlockDistortionArray);
            }
            pDelFD->BlockDistortionArray = NULL;
        }
        if (FrameNumberInFile % StepLength == 0) {
            DelFDNumber = FrameNumberInFile / StepLength - 2;
            if (DelFDNumber >= 0) {
                pDelFD = &OMCPDList->FrameDistortionArray[DelFDNumber];
                if (pDelFD->BlockDistortionArray != NULL) {
                    free(pDelFD->BlockDistortionArray);
                }
                pDelFD->BlockDistortionArray = NULL;
                if (pDelFD->subFrameDistortionArray != NULL) {
                    free(pDelFD->subFrameDistortionArray);
                }
                pDelFD->subFrameDistortionArray = NULL;
            }
        }
    }
#endif

#if RATECONTROL
    if (input->EncControl != 0) {
        if (input->useDQP) {
            img->qp = (int)((0.5 + pRC->SumMBQP) / pRC->NumMB);
        }
        //if (input->EncControl!=0)
        Updata_RateControl(pRC, (int)(stat->bit_ctr - stat->bit_ctr_n), img->qp, img->type, FrameNumberInFile,
                           he->gop_size_all);
    }
#endif

    time(&ltime2);                // end time sec
#ifdef WIN32
    _ftime(&tstruct2);            // end time ms
#else
    ftime(&tstruct2);             // end time ms
#endif

    tmp_time = (int)((ltime2 * 1000 + tstruct2.millitm) - (ltime1 * 1000 + tstruct1.millitm));
    hc->tot_time = hc->tot_time + tmp_time;

    if (he->curr_RPS.referd_by_others && img->type != INTRA_IMG)

    {
        addCurrMvtoBuf();
        compressMotion();
    }


    if (input->output_enc_pic) {
        if (img->typeb == BACKGROUND_IMG && he->background_output_flag == 0 && he->p_dec_background) {
            write_GB_frame(he->p_dec_background);
        } else {
            write_reconstructed_image();
        }
    }

#if FIX_PROFILE_LEVEL_DPB_RPS_1
    //////////////////////////////////////////////////////////////////////////
    // delete the frame that will never be used
    {
        int i, j;
        for (i = 0; i < he->curr_RPS.num_to_remove; i++) {
            for (j = 0; j < REF_MAXBUFFER; j++) {

                if (fref[j]->imgcoi_ref >= -256 && fref[j]->imgcoi_ref == hc->coding_order - he->curr_RPS.remove_pic[i]) {
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

#if M3480_TEMPORAL_SCALABLE
                fref[j]->temporal_id = -1;
#endif
                fref[j]->imgcoi_ref = -257;
                if (fref[j]->is_output == -1)
                { fref[j]->imgtr_fwRefDistance = -256; }
#endif
            }
        }
    }
#endif

    report_frame(tmp_time);

    stat->bit_ctr_n = stat->bit_ctr;


    if (img->type != B_IMG
        && !(img->typeb == BACKGROUND_IMG && he->background_output_flag == 0)
       ) {
        img->numIPFrames++;
    }


    if (img->number == 0) {
        return 0;
    } else {
        return 1;
    }
}

/*
*************************************************************************
* Function:This function write out a picture
* Input:
* Output:
* Return: 0 if OK,                                                         \n
1 in case of error
* Attention:
*************************************************************************
*/

static int writeout_picture()
{

    assert(currBitStream->bits_to_go == 8);     //! should always be the case,
    //! the byte alignment is done in terminate_slice
    WriteBitstreamtoFile();

    return 0;
}

/*
*************************************************************************
* Function:Encodes a frame picture
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

static void code_a_picture(Picture *frame)
{
    AllocateBitstream();
    AllocateBitstream_ALF();

#if REFINED_QP
    if (input->use_refineQP) {
        int   lambdaQP;     //M2331 2008-04
        double  lambdaF;    //M2331 2008-04
        int qp;
        int i;
        int prevP_no, RPS_idx, p_interval, no_IPframes;
        //double qp     = ( double ) img->qp - SHIFT_QP;
#if FREQUENCY_WEIGHTING_QUANTIZATION
        if (WeightQuantEnable) {
            he->mb_wq_mode = WQ_MODE_D;
        }
#endif
        if (img->type == F_IMG) {
            lambdaQP = LambdaQPTab[P_IMG][he->mb_wq_mode];
            lambdaF = LambdaFTab[P_IMG][he->mb_wq_mode];
        } else {
            lambdaQP = LambdaQPTab[img->type][he->mb_wq_mode];
            lambdaF = LambdaFTab[img->type][he->mb_wq_mode];
        }

#if (FREQUENCY_WEIGHTING_QUANTIZATION && AWQ_WEIGHTING)
        if (WeightQuantEnable) {
            qp     = img->qp - SHIFT_QP + lambdaQP;
        } else {
            qp     = img->qp - SHIFT_QP;
        }
#else
        qp     = img->qp - SHIFT_QP;
#endif

        p_interval = 0;
        for (i = 0; i <= he->subGopID; i++) {
            p_interval += (input->jumpd_sub[i] + 1);
        }
        prevP_no      = (img->number - 1) * input->jumpd_all + (p_interval - input->jumpd - 1);
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

        if (input->intra_period != 1) {
            img->qp = (int)(5.661 * log(he->lambda_mode) + 13.131 + 0.5);
        }
    }
#endif

    img->qp = Clip3(0, MAX_QP, img->qp);
    picture_header();

#if PicExtensionData
    if (input->PicExtentData && !input->alf_enable) {
        picture_copyright_extension(currBitStream);
        picture_cameraparameters_extension(currBitStream);
        picture_display_extension(currBitStream);
    }
#endif
#if ROI_M3264
    if (input->ROI_Coding && !input->alf_enable) {
        roi_parameters_extension(img->tr, currBitStream);
    }
#endif

    picture_data(frame);

    frame->bits_per_picture = 8 * (currBitStream->byte_pos);
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

#if M3480_TEMPORAL_SCALABLE
    int RPS_idx;
    int layer;
    if (he->temporal_id_exist_flag == 1) {
        int prevP_no;
        int p_interval;

        p_interval = 0;
        for (i = 0; i <= he->subGopID; i++) {
            p_interval += (input->jumpd_sub[i] + 1);
        }
        prevP_no = (img->number - 1) * input->jumpd_all + (p_interval - input->jumpd - 1);

        RPS_idx = he->subGopNum == 1 ? ((hc->coding_order - 1) % he->gop_size) : (hc->coding_order - 1 - prevP_no);
    }
    if (he->temporal_id_exist_flag == 1 && img->type != INTRA_IMG) {
        if (RPS_idx >= 0 && RPS_idx < he->gop_size_all) {
            if (he->cfg_ref_all[RPS_idx].temporal_id < 0 || he->cfg_ref_all[RPS_idx].temporal_id >= TEMPORAL_MAXLEVEL) {
                he->cfg_ref_all[RPS_idx].temporal_id = TEMPORAL_MAXLEVEL - 1;    //the lowest level
            }
            layer = he->cfg_ref_all[RPS_idx].temporal_id;
        } else {
            layer = TEMPORAL_MAXLEVEL - 1;
        }
    } else {
        layer = 0;
    }
#endif
    //////////////////////////////////////////////////////////////////////////
    //update IDR frame
    if (img->tr > he->next_IDRtr && he->curr_IDRtr != he->next_IDRtr) {
        he->curr_IDRtr  = he->next_IDRtr;
        he->curr_IDRcoi = he->next_IDRcoi;
    }

    //////////////////////////////////////////////////////////////////////////
    // re-order the ref buffer according to RPS
    img->num_of_references = he->curr_RPS.num_of_ref;//

    for (i = 0; i < he->curr_RPS.num_of_ref; i++) {
        int accumulate = 0;
        /* copy tmp_fref from fref[i] */
        tmp_fref = fref[i];

        for (j = i; j < REF_MAXBUFFER; j++) {
            int k , tmp_tr;
            for (k = 0; k < REF_MAXBUFFER; k++) {
                if (((int)hc->coding_order - (int)he->curr_RPS.ref_pic[i]) == fref[k]->imgcoi_ref && fref[k]->imgcoi_ref >= -256) {
                    break;
                }
            }
            if (k == REF_MAXBUFFER) {
                tmp_tr = -1;
            } else {
                tmp_tr = fref[k]->imgtr_fwRefDistance;
            }
            if (tmp_tr < he->curr_IDRtr) {
                he->curr_RPS.ref_pic[i] = hc->coding_order - he->curr_IDRcoi;

                for (k = 0; k < i; k++) {
                    if (he->curr_RPS.ref_pic[k] == he->curr_RPS.ref_pic[i]) {
                        accumulate++;
                        break;
                    }
                }
            }
            if (fref[j]->imgcoi_ref == hc->coding_order - he->curr_RPS.ref_pic[i]) {
                break;
            }
        }
        if (j == REF_MAXBUFFER || accumulate) {
            img->num_of_references--;
        }
        if (j != REF_MAXBUFFER) {
            /* copy fref [i] from [j] */
            fref[i] = fref[j];
#if M3480_TEMPORAL_SCALABLE
            if (fref[i]->temporal_id > layer) {
                assert("Frame with lower temporal_i can not ref higher frames!");
            }
#endif
            /* copy fref [j] from [tmp] */
            fref[j] = tmp_fref;
        }
    }
    he->curr_RPS.num_of_ref = img->num_of_references;

    if (img->type == B_IMG && (fref[0]->imgtr_fwRefDistance <= img->tr || fref[1]->imgtr_fwRefDistance >= img->tr)) {
        //******************************************//
        int max_for = 1, k;
        int min_bck = 0;
        int tmp_max_for = 0;
        int tmp_min_bck = 1 << 30;
        for (i = 1; i < REF_MAXBUFFER; i++) {
            int tmp_ref = fref[i]->imgtr_fwRefDistance;
            if (tmp_ref < img->tr && tmp_ref > tmp_max_for && tmp_ref >= he->curr_IDRtr) {
                max_for = i;//find forward ref
                tmp_max_for = tmp_ref;
            }
            if (tmp_ref > img->tr && tmp_ref < tmp_min_bck) {
                min_bck = i;//find backward ref
                tmp_min_bck = tmp_ref;
            }
        }
        for (k = 0; k < 2; k++) {
            i = (k == 0 ? max_for : min_bck);
            j = (k == 0 ? 1 : 0);
            /* copy tmp_fref from fref[i] */
            tmp_fref = fref[i];
            /* copy fref[i] from fref[j] */
            fref[i] = fref[j];
            /* copy fref[j] from tmp_fref */
            fref[j] = tmp_fref;
        }
        //change the wrong RPS
        for (i = 0; i < he->curr_RPS.num_of_ref; i++) {
            he->curr_RPS.ref_pic[i] = hc->coding_order - fref[i]->imgcoi_ref;
        }
        printf("wrong reference configuration for frame:%d has been corrected\n", img->tr);
    }

    if ((img->type == P_IMG) || (img->type == F_IMG)) {
        for (i = img->num_of_references; i < input->no_multpred; i++) {
            int max_for = img->num_of_references;
            int tmp_max_for = he->curr_IDRtr;
            int flag = 0;
            for (j = img->num_of_references; j < REF_MAXBUFFER; j++) {
                int tmp_ref = fref[j]->imgtr_fwRefDistance;
#if M3480_TEMPORAL_SCALABLE
                if (tmp_ref < img->tr && tmp_ref > he->curr_IDRtr && (he->temporal_id_exist_flag != 1 ||
                        layer >= fref[j]->temporal_id) &&
                    abs(fref[j]->imgtr_fwRefDistance - img->tr) < 128)
#else
                if (tmp_ref < img->tr && tmp_ref > he->curr_IDRtr && abs(fref[j]->imgtr_fwRefDistance - img->tr) < 128)
#endif
                {
                    flag = 1;

                    if (tmp_max_for < tmp_ref) {
                        max_for = j;
                        tmp_max_for = tmp_ref;
                    }
                }
            }
            if (flag) {
                j = max_for;
                /* copy tmp_fref from fref[i] */
                tmp_fref = fref[i];
                /* copy fref[i] from fref[j] */
                fref[i] = fref[j];
                he->curr_RPS.ref_pic[i] = hc->coding_order - fref[j]->imgcoi_ref;
                /* copy fref[j] from tmp_fref */
                fref[j] = tmp_fref;

                flag = 0;
                img->num_of_references++;
            } else {
                break;
            }
        }
    }
    he->curr_RPS.num_of_ref = img->num_of_references;

    if (img->num_of_references > 1) {
        img->num_ref_pic_active_fwd_minus1 = 1;  //yling
    }

#if !FIX_PROFILE_LEVEL_DPB_RPS_1
    //////////////////////////////////////////////////////////////////////////
    // delete the frame that will never be used
    {
        int actualRemovePic[MAXREF] = { -1};
        int removeNum = 0;
        for (i = 0; i < he->curr_RPS.num_to_remove; i++) {
            for (j = 0; j < REF_MAXBUFFER; j++) {
                if (fref[j]->imgcoi_ref >= -256 && fref[j]->imgcoi_ref == hc->coding_order - he->curr_RPS.remove_pic[i]) {
                    break;
                }
            }
            if (j < REF_MAXBUFFER && j >= img->num_of_references) {
#if M3480_TEMPORAL_SCALABLE
                if (fref[j]->temporal_id < layer) {
                    assert("Can not remove lower-temporal frame.");
                }
                fref[j]->temporal_id = -1;
#endif
                fref[j]->imgcoi_ref = -257;
                if (fref[j]->is_output == -1) {
                    fref[j]->imgtr_fwRefDistance = -256;
                }
                actualRemovePic[removeNum] = he->curr_RPS.remove_pic[i];
                removeNum++;
            }
        }
        he->curr_RPS.num_to_remove = removeNum;
        for (i = 0; i < removeNum; i++) {
            he->curr_RPS.remove_pic[i] = actualRemovePic[i];
        }
    }
#else
    //change RPS
    {
        int actualRemovePic[MAXREF] = { -1 };
        int removeNum = 0;
        Boolean deletePOC = 1;
        for (i = 0; i < he->curr_RPS.num_to_remove; i++) {
            for (j = 0; j < REF_MAXBUFFER; j++) {
                if (fref[j]->imgcoi_ref >= -256 && fref[j]->imgcoi_ref == hc->coding_order - he->curr_RPS.remove_pic[i]) {
                    break;
                }
            }
            if (j < REF_MAXBUFFER) {
                actualRemovePic[removeNum] = he->curr_RPS.remove_pic[i];
                removeNum++;
            }
        }
        he->curr_RPS.num_to_remove = removeNum;
        for (i = 0; i < removeNum; i++) {
            he->curr_RPS.remove_pic[i] = actualRemovePic[i];
        }
    }
#endif


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

    hc->f_rec = fref[i];
    hc->currentFrame = hc->f_rec->ref;
    hc->f_rec->imgtr_fwRefDistance = img->tr;
    hc->f_rec->imgcoi_ref = hc->coding_order;

#if M3480_TEMPORAL_SCALABLE
    hc->f_rec->temporal_id = he->cur_layer = layer;
#endif
    hc->f_rec->is_output = 1;
    hc->f_rec->refered_by_others = he->curr_RPS.referd_by_others;

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
        for (l = 0; l < 4; l++) {
            hc->f_rec->ref_poc[l] = img->tr;
        }
    }
    cleanRefMVBufRef(i);
#endif

#if !RD1510_FIX_BG
    get_reference_list_info(hc->str_list_reference);
#endif

    for (j = 0; j < NUM_SAO_COMPONENTS; j++) {
        hc->f_rec->saorate[j] = img->cur_saorate[j];
    }

    //////////////////////////////////////////////////////////////////////////
    // update ref pointer
    if (img->type != INTRA_IMG) {
        img->imgtr_next_P = img->type == B_IMG ? fref[0]->imgtr_fwRefDistance : img->tr;
        if (img->type == B_IMG) {
            he->trtmp = fref[0]->imgtr_fwRefDistance;
            fref[0]->imgtr_fwRefDistance = fref[1]->imgtr_fwRefDistance;
        }
    }
    {
        int k, x, y, ii;
        for (ii = 0; ii < REF_MAXBUFFER; ii++) {
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

/*
*************************************************************************
* Function:Initializes the parameters for a new frame
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
static void init_frame()
{
    int i, j, k;
    int prevP_no, nextP_no;
    int b_interval, p_interval;

    //---Yulj 2004.07.15
    int widthMB = img->width  / MIN_CU_SIZE;
    int heightMB = img->height / MIN_CU_SIZE;
    int minCUsInLCU = (1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT)) * (1 << (input->g_uiMaxSizeInBit -
                      MIN_CU_SIZE_IN_BIT));
    int lcusInOneRow = (img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) != 0 ? 1 : 0);
    int lcuSize = 1 << input->g_uiMaxSizeInBit;
    img->mb_no_currSliceLastMB = (input->slice_row_nr != 0)
                                 ? min(((input->slice_row_nr / lcusInOneRow) * widthMB * (1 << (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT)))
                                       + ((input->slice_row_nr % lcusInOneRow) << (2 * (input->g_uiMaxSizeInBit - MIN_CU_SIZE_IN_BIT))),
                                       widthMB * heightMB)
                                 : widthMB * heightMB ;
    input->slice_row_nr = input->slice_row_nr != 0 ? input->slice_row_nr : img->mb_no_currSliceLastMB + 1;

    //---end.



    p_interval = 0;
    for (i = 0; i <= he->subGopID; i++) {
        p_interval += (input->jumpd_sub[i] + 1);
    }

    img->current_mb_nr = 0;
    img->current_slice_nr = 0;


    if (!(img->typeb == BACKGROUND_IMG && he->background_output_flag == 0)) {

        hc->coding_order++;
    }

    if (hc->coding_order == 256) {
        hc->coding_order = 0;

        for (i = 0; i < REF_MAXBUFFER; i++) {
            if (fref[i]->imgtr_fwRefDistance >= 0) {
                fref[i]->imgtr_fwRefDistance -= 256;
                fref[i]->imgcoi_ref -= 256;
                for (j = 0; j < 4; j++) {
                    fref[i]->ref_poc[j] -= 256;
                }
            }
        }
        hc->total_frames++;
        he->curr_IDRtr -= 256;
        he->curr_IDRcoi -= 256;
        he->next_IDRtr -= 256;
        he->next_IDRcoi -= 256;
        he->last_output -= 256;

    }
    stat->bit_slice = 0;
    img->coded_mb_nr = 0;
    img->slice_offset = 0;

    img->mb_y = img->mb_x = 0;
    img->pix_y = img->pix_c_y = 0; // img->block_y =
    img->pix_x = img->pix_c_x = 0;//img->block_x =


    if (img->type != B_IMG) {
        if (img->number == 0) {
            img->tr = 0;
        } else {
            prevP_no = (img->number - he->background_number - 1) * input->jumpd_all + (p_interval - input->jumpd - 1);
            if (img->typeb == BACKGROUND_IMG && he->background_output_flag == 0) {
                img->tr = (img->number - he->background_number - 1 - 1) * input->jumpd_all + p_interval;
            } else {
                img->tr = (img->number - he->background_number - 1) * input->jumpd_all + p_interval;
            }

            img->tr = min(img->tr, input->no_frames - 1);
        }
    } else {

        prevP_no = (img->number - he->background_number - 1) * input->jumpd_all + (p_interval - input->jumpd - 1);
        nextP_no = (img->number - he->background_number - 1) * input->jumpd_all + p_interval;

        nextP_no         = min(nextP_no, input->no_frames - 1);

        b_interval = (int)((double)(input->jumpd + 1) / (input->successive_Bframe + 1.0) + 0.49999);

        {
            int offset = hc->coding_order - 1 - prevP_no + he->flag_gop + hc->total_frames * 256;
            img->tr = prevP_no + he->cfg_ref[offset].poc;
        }
        while (img->tr >= input->no_frames + he->background_number - 1) {
            int offset_tmp;
            he->flag_gop++;
            offset_tmp = hc->coding_order - 1 - prevP_no + he->flag_gop + hc->total_frames * 256;
            img->tr = prevP_no + he->cfg_ref[offset_tmp].poc;
        }
        // initialize arrays

        for (k = 0; k < 2; k++) {
            for (j = 0; j < img->height / MIN_BLOCK_SIZE; j++) {
                for (i = 0; i < img->width / MIN_BLOCK_SIZE ; i++) {
                    img->fw_mv[j][i][k] = 0;
                    img->bw_mv[j][i][k] = 0;
                }
            }
        }

        for (j = 0; j < img->height / MIN_BLOCK_SIZE; j++) {
            for (i = 0; i < img->width / MIN_BLOCK_SIZE; i++) {
                img->fw_refFrArr[j][i] = img->bw_refFrArr[j][i] = -1;
            }
        }
    }


    img->tr -= hc->total_frames * 256;


    for (i = 0; i < img->PicWidthInMbs * img->height / MIN_CU_SIZE; i++) {
        img->mb_data[i].slice_nr = -1;
    }

    img->PicSizeInMbs = (img->width * img->height) / (MIN_CU_SIZE * MIN_CU_SIZE);



#if REMOVE_UNUSED
	 hc->picture_distance = img->tr;
#else
     hc->picture_distance = img->tr % 256;          //yling
#endif

#if AQPOM3694 // RA
    if (input->successive_Bframe_all && input->AQPEnable) {
        if (img->tr != 0) {
            if (img->tr % StepLength == 0) {
                int index = (img->tr / StepLength) % input->intra_period;
                int nextIndex;
                int offsetTable[4][8] = {{0, 0, 1}, {0, 0, 1, 2}, {0, 1, 1, 2, 2, 2}, {0, 1, 1, 1, 2, 2, 2, 2}};
                if (img->tr < (8 * input->intra_period)) {
                    index -= 1;
                }
                nextIndex = (index + 1) % input->intra_period;
                if (input->intra_period == 3) {
                    GopQpbase = input->qpI + offsetTable[0][index];
                }
                if (input->intra_period == 4) {
                    GopQpbase = input->qpI + offsetTable[1][index];
                }
                if (input->intra_period == 6) {
                    GopQpbase = input->qpI + offsetTable[2][index];
                }
                if (input->intra_period == 8) {
                    GopQpbase = input->qpI + offsetTable[3][index];
                }
            }
        } else {
            GopQpbase = input->qpI;
        }
    }
#endif

#if !FIX_PROFILE_LEVEL_DPB_RPS_2
    if (img->type == INTRA_IMG) {
        if (input->intra_period == 1) {
            he->curr_RPS = he->cfg_ref[0];
            img->qp = input->qpI + he->cfg_ref[0].qp_offset;
        } else {
            he->curr_RPS.num_of_ref = 0;
            he->curr_RPS.referd_by_others = 1;
            if (input->bg_enable && img->typeb == BACKGROUND_IMG &&
                he->background_output_flag == 0) { //bugfix: GB does not remove any pictures and does not call Prepare_RefInfo()
                he->curr_RPS.num_to_remove = 0;
            }
            img->qp = input->qpI;
        }
    }

    else
#endif
    {
        int RPS_idx = he->subGopNum == 1 ? ((hc->coding_order + hc->total_frames * 256 - 1) % he->gop_size) :
                      (hc->coding_order + hc->total_frames * 256 - 1 - prevP_no);
        he->curr_RPS = he->cfg_ref[RPS_idx];
        img->qp = input->qpI + he->cfg_ref[RPS_idx + he->flag_gop].qp_offset;
#if AQPOM3694 // RA
        if (input->successive_Bframe_all && input->AQPEnable) {
            img->qp = GopQpbase + he->cfg_ref[RPS_idx + he->flag_gop].qp_offset;
        }
#endif
#if AQPO
        OffsetIndex = RPS_idx;
#endif

#if FIX_PROFILE_LEVEL_DPB_RPS_2
        if (img->type == INTRA_IMG) {
            if (input->intra_period == 1) {
                he->curr_RPS = he->cfg_ref[0];
                img->qp = input->qpI + he->cfg_ref[0].qp_offset;
            } else {
                he->curr_RPS.num_of_ref = 0;
                he->curr_RPS.referd_by_others = 1;
                if (input->bg_enable && img->typeb == BACKGROUND_IMG &&
                    he->background_output_flag == 0) { //bugfix: GB does not remove any pictures and does not call Prepare_RefInfo()
                    he->curr_RPS.num_to_remove = 0;
                }
                img->qp = input->qpI;
            }
        }

#endif
    }


    img->qp = Clip3(0, MAX_QP, img->qp);


#if TH_ME
    if (img->type != INTRA_IMG && input->usefme) {
        DefineThreshold();
    }
#endif

    if (img->typeb == BACKGROUND_IMG && he->background_output_flag == 0) {
        hc->currentFrame = hc->background_ref;
    } else {
        prepare_RefInfo();
    }
#if RD1510_FIX_BG
    if (img->type == I_IMG && img->typeb == BACKGROUND_IMG) { // G/GB frame
        img->num_of_references = 0;
    } else if (img->type == P_IMG && img->typeb == BP_IMG && input->bg_enable) { //only one reference frame(G\GB) for S frame
        img->num_of_references = 1;
    }
    get_reference_list_info(hc->str_list_reference);
#endif
}

int min_tr(int ref[REF_MAXBUFFER], int last_out, int *pos)
{
    int i, tmp_min;
    tmp_min = 1 << 20;
    *pos = -1;
    for (i = 0; i < REF_MAXBUFFER; i++) {
        if (ref[i] < tmp_min && ref[i] > last_out) {
            tmp_min = ref[i];
            *pos = i;
        }
    }
    return tmp_min;
}

void addCurrMvtoBuf()
{
    int k, x, y;

    for (k = 0; k < 2; k++) {
        for (y = 0; y < img->height / MIN_BLOCK_SIZE; y++) {
            for (x = 0; x < img->width / MIN_BLOCK_SIZE; x++) {
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
            // middle pixel's motion information
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
* Function:Copy decoded P frame to temporary image array
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void copy_Pframe()
{
    int i, j;

    /*
    * the mmin, mmax macros are taken out
    * because it makes no sense due to limited range of data type
    */

    for (i = 0; i < img->height; i++) {
        for (j = 0; j < img->width; j++) {
            hc->imgYPrev[i][j] = hc->imgY[i][j];
        }
    }

    for (i = 0; i < img->height_cr; i++) {
        for (j = 0; j < img->width_cr; j++) {
            hc->imgUVPrev[0][i][j] = hc->imgUV[0][i][j];
        }
    }

    for (i = 0; i < img->height_cr; i++) {
        for (j = 0; j < img->width_cr; j++) {
            hc->imgUVPrev[1][i][j] = hc->imgUV[1][i][j];
        }
    }
}
/*
*************************************************************************
* Function:Writes ORG GB frame to file
This can be done more elegant!
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void write_GB_org_frame(FILE *p_dec)           //!< filestream to output file
{
    int img_width = (img->width - img->auto_crop_right);
    int img_height = (img->height - img->auto_crop_bottom);
    int img_width_cr = (img_width / 2);
    int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));

    if (p_dec != NULL) {
        // assume "input->sample_bit_depth" is greater or equal to "input->input_sample_bit_depth"
        int img_length = img_height * img_width + img_width_cr * img_height_cr * 2;
        write_yuv_frame(p_dec, he->imgY_org_buffer, img_length);
    }
}

/*
*************************************************************************
* Function:Writes reconstructed GB frame to file
This can be done more elegant!
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void write_GB_frame(FILE *p_dec)           //!< filestream to output file
{
    int img_width = (img->width - img->auto_crop_right);
    int img_height = (img->height - img->auto_crop_bottom);
    int img_width_cr = (img_width / 2);
    int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));

    byte **imgY_out;
    byte ***imgUV_out;

    if (p_dec != NULL) {
        // assume "input->sample_bit_depth" is greater or equal to "input->input_sample_bit_depth"
        int img_length = img_height * img_width + img_width_cr * img_height_cr * 2;
        byte *imgYuv_buffer = malloc(img_length * sizeof(byte));

        imgY_out = hc->imgY;
        imgUV_out = hc->imgUV;

        reduce_yuv_dim(imgYuv_buffer, imgY_out, imgUV_out, img_width, img_height, img_width_cr, img_height_cr);
        write_yuv_frame(p_dec, imgYuv_buffer, img_length);
        free(imgYuv_buffer);
    }
}
/*
*************************************************************************
* Function:Writes reconstructed image(s) to file
This can be done more elegant!
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void write_frame(FILE *p_dec , int pos)
{
    int img_width = (img->width - img->auto_crop_right);
    int img_height = (img->height - img->auto_crop_bottom);
    int img_width_cr = (img_width / 2);
    int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));
    byte **imgY_out ;
    byte ***imgUV_out ;
    if (p_dec != NULL) {
        // assume "input->sample_bit_depth" is greater or equal to "input->input_sample_bit_depth"
        int img_length = img_height * img_width + img_width_cr * img_height_cr * 2;
        byte *imgYuv_buffer = malloc(img_length * sizeof(byte));
        imgY_out = fref[pos]->ref[0];
        imgUV_out = &fref[pos]->ref[1];
        fref[pos]->is_output = -1;
        if (fref[pos]->refered_by_others == 0 || fref[pos]->imgcoi_ref == -257) {
            fref[pos]->imgtr_fwRefDistance = -256;
            fref[pos]->imgcoi_ref = -257;
#if M3480_TEMPORAL_SCALABLE
            fref[pos]->temporal_id = -1;
#endif
        }

        reduce_yuv_dim(imgYuv_buffer, imgY_out, imgUV_out, img_width, img_height, img_width_cr, img_height_cr);
        write_yuv_frame(p_dec, imgYuv_buffer, img_length);
        free(imgYuv_buffer);
    }
}

static void write_reconstructed_image()
{
    int i;


#if !REF_OUTPUT
    for (i = 0; i < REF_MAXBUFFER; i++) {
        int output_next = min_tr(img->imgtr_fwRefDistance, he->last_output, &pos);
        if (output_next < img->tr || output_next == (he->last_output + 1)) {
            he->last_output = output_next;
            write_frame(he->p_dec, pos);
            i--;
        } else {
            break;
        }
    }
#else
    if (hc->coding_order + hc->total_frames * 256 >= he->picture_reorder_delay) {
        int tmp_min, pos = -1;
        tmp_min = 1 << 20;
        for (i = 0; i < REF_MAXBUFFER; i++) {

            if (fref[i]->imgtr_fwRefDistance < tmp_min && fref[i]->imgtr_fwRefDistance > he->last_output)

            {
                tmp_min = fref[i]->imgtr_fwRefDistance;
                pos = i;
            }
        }
        if (pos != -1) {

            he->last_output = tmp_min;

            write_frame(he->p_dec, pos);
        }

    }
#endif
}
/*
*************************************************************************
* Function:Write previous decoded P frame to output file
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void write_prev_Pframe()
{
    int writeout_width = input->img_width;
    int writeout_height = input->img_height;
    int writeout_width_cr =  input->img_width / 2;
    int writeout_height_cr = input->img_height / 2;

    int writeout_length = writeout_height * writeout_width + writeout_width_cr * writeout_height_cr * 2;

    byte *imgYuv_buffer;
    imgYuv_buffer = malloc(writeout_length * sizeof(byte));
    reduce_yuv_dim(imgYuv_buffer, hc->imgYPrev, hc->imgUVPrev, writeout_width, writeout_height, writeout_width_cr,
                   writeout_height_cr);
    write_yuv_frame(he->p_dec, imgYuv_buffer, writeout_length);
    free(imgYuv_buffer);
}

/*
*************************************************************************
* Function:Upsample 4 times, store them in out4x.  Color is simply copied
* Input:srcy, srcu, srcv, out4y, out4u, out4v
* Output:
* Return:
* Attention:Side Effects_
Uses (writes) img4Y_tmp.  This should be moved to a static variable
in this module
*************************************************************************
*/
#define  IClip( Min, Max, Val) (((Val)<(Min))? (Min):(((Val)>(Max))? (Max):(Val)))

static void UnifiedOneForthPix(pel_t **imgY, pel_t **imgU, pel_t **imgV,
                               pel_t **out4Y)
{
    int max_pel_value = (1 << input->sample_bit_depth) - 1;
    int shift1 = input->sample_bit_depth - 8;
    int shift2 = 14 - input->sample_bit_depth;
    int shift3 = 20 - input->sample_bit_depth;
    int is;
    int temp;
    int dx, dy;
    int x, y;
    int i, j;// j4;
    int ii, jj;

    static const int COEF[3][8] = {
        { -1, 4, -10, 57, 19,  -7, 3, -1 },
        { -1, 4, -11, 40, 40, -11, 4, -1 },
        { -1, 3,  -7, 19, 57, -10, 4, -1 }
    };



    //    A  a  1  b  B
    //    c  d  e  f
    //    2  h  3  i
    //    j  k  l  m
    //    C           D

    //fullpel position: A
    for (j = -IMG_PAD_SIZE; j < img->height + IMG_PAD_SIZE; j++) {
        for (i = -IMG_PAD_SIZE; i < img->width + IMG_PAD_SIZE; i++) {
            he->img4Y_tmp[(j + IMG_PAD_SIZE) * 4][(i + IMG_PAD_SIZE) * 4] =
                imgY[IClip(0, img->height - 1, j) ][IClip(0, img->width - 1, i) ];
        }
    }

    //horizontal positions: a,1,b
    for (j = -IMG_PAD_SIZE; j < img->height + IMG_PAD_SIZE; j++) {
        for (i = -IMG_PAD_SIZE; i < img->width + IMG_PAD_SIZE; i++) {
            jj = IClip(0, img->height - 1, j);

            for (dx = 1; dx < 4; dx++) {
                for (is = 0, x = -3; x < 5; x++) {
                    is += imgY[jj][IClip(0, img->width - 1, i + x) ] * COEF[dx - 1][x + 3];
                }
                he->img4Y_tmp[(j + IMG_PAD_SIZE) * 4][(i + IMG_PAD_SIZE) * 4 + dx] = is;
            }
        }
    }

    //vertical positions: c,2,j
    for (j = -IMG_PAD_SIZE; j < img->height + IMG_PAD_SIZE; j++) {
        for (i = -IMG_PAD_SIZE; i < img->width + IMG_PAD_SIZE; i++) {
            ii = IClip(0, img->width - 1, i);

            for (dy = 1; dy < 4; dy++) {

                for (is = 0, y = -3; y < 5; y++) {
                    is += imgY[IClip(0, img->height - 1, j + y) ][ii] * COEF[dy - 1][y + 3];
                }

                is = IClip(0, max_pel_value, (is + 32) / 64);

                he->img4Y_tmp[(j + IMG_PAD_SIZE) * 4 + dy][(i + IMG_PAD_SIZE) * 4] = is;
            }
        }
    }


    //vertical positions: d,h,k; e,3,1; f,i,m
    for (i = 0; i < (img->width  + 2 * IMG_PAD_SIZE) * 4; i += 4) {
        for (j = 0; j < (img->height  + 2 * IMG_PAD_SIZE) * 4; j += 4) {
            for (dx = 1; dx < 4; dx++) {
                for (dy = 1; dy < 4; dy++) {

                    for (is = 0, y = -3; y < 5; y++) {
                        jj = IClip(0, (img->height + 2 * IMG_PAD_SIZE) * 4 - 4, j + 4 * y);
                        ii = IClip(0, (img->width + 2 * IMG_PAD_SIZE) * 4 - 4, i + dx);
                        //  is += img4Y_tmp[jj][ii] * COEF[dy - 1][y + 3];
                        if (shift1) {
                            is += ((he->img4Y_tmp[jj][ii] + (1 << (shift1 - 1))) >> shift1) * COEF[dy - 1][y + 3];
                        } else {
                            is += he->img4Y_tmp[jj][ii] * COEF[dy - 1][y + 3];
                        }
                    }

                    //   img4Y_tmp[j  + dy][i + dx] = IClip ( 0, max_pel_value, ( is + 2048 ) / 4096 );
                    he->img4Y_tmp[j + dy][i + dx] = IClip(0, max_pel_value, (is + (1 << (shift3 - 1))) >> shift3);
                }
            }
        }
    }

    //positions: a,1,b
    for (j = 0; j < (img->height  + 2 * IMG_PAD_SIZE) * 4; j += 4) {
        for (i = 0; i < (img->width  + 2 * IMG_PAD_SIZE) * 4; i ++) {
            if (i % 4 != 0) {
                temp = IClip(0, max_pel_value, (he->img4Y_tmp[j][i] + 32) >> 6);
                he->img4Y_tmp[j][i] = temp;
            }
        }
    }


    for (j = 0; j < (img->height  + 2 * IMG_PAD_SIZE) * 4; j ++) {
        for (i = 0; i < (img->width  + 2 * IMG_PAD_SIZE) * 4; i ++) {
            PutPel_14(out4Y, j - IMG_PAD_SIZE * 4, i - IMG_PAD_SIZE * 4, (pel_t)he->img4Y_tmp[j][i]);
        }
    }

}

/*
*************************************************************************
* Function:Find SNR for all three components
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

static void find_snr()
{
    int i, j;
    long long int diff_y, diff_u, diff_v;
    int impix;
    int uvformat = input->chroma_format == 1 ? 4 : 2;

    double maxSignal = (double)((1 << input->input_sample_bit_depth) - 1) * (double)((1 << input->input_sample_bit_depth) -
                       1);
    int shift1 = input->sample_bit_depth - input->input_sample_bit_depth;
    //  Calculate  PSNR for Y, U and V.
    //     Luma.
    impix = input->img_height * input->img_width;

    diff_y = 0;


    diff_u = 0;
    diff_v = 0;

    if (shift1 != 0) {
        for (i = 0; i < input->img_width; ++i) {
            for (j = 0; j < input->img_height; ++j) {
                diff_y += img->quad[(he->imgY_org[j][i] >> shift1) - Clip1((hc->imgY[j][i] + (1 << (shift1 - 1))) >> shift1)];
            }
        }

        {
            for (i = 0; i < input->img_width / 2; i++) {
                for (j = 0; j < input->img_height / 2; j++) {
                    diff_u += img->quad[(he->imgUV_org[0][j][i] >> shift1) - Clip1((hc->imgUV[0][j][i] + (1 << (shift1 - 1))) >> shift1)];
                    diff_v += img->quad[(he->imgUV_org[1][j][i] >> shift1) - Clip1((hc->imgUV[1][j][i] + (1 << (shift1 - 1))) >> shift1)];
                }
            }
        }
    } else {
        for (i = 0; i < input->img_width; ++i) {
            for (j = 0; j < input->img_height; ++j) {
                diff_y += img->quad[he->imgY_org[j][i] - hc->imgY[j][i]];
            }
        }

        {
            for (i = 0; i < input->img_width / 2; i++) {
                for (j = 0; j < input->img_height / 2; j++) {
                    diff_u += img->quad[he->imgUV_org[0][j][i] - hc->imgUV[0][j][i]];
                    diff_v += img->quad[he->imgUV_org[1][j][i] - hc->imgUV[1][j][i]];
                }
            }
        }
    }

    //  Collecting SNR statistics
    if (diff_y != 0) {
        snr->snr_y = (10.0 * log10(maxSignal * (double) impix / (double) diff_y));          // luma snr for current frame
        snr->snr_u = (10.0 * log10(maxSignal * (double) impix / (double)(/*4*/uvformat *
                                   diff_u)));    // u croma snr for current frame, 1/4 of luma samples
        snr->snr_v = (10.0 * log10(maxSignal * (double) impix / (double)(/*4*/uvformat *
                                   diff_v)));    // v croma snr for current frame, 1/4 of luma samples
    }

    if (img->number == 0) {
        snr->snr_y1 = (10.0 * log10(maxSignal * (double) impix / (double) diff_y));          // keep luma snr for first frame
        snr->snr_u1 = (10.0 * log10(maxSignal * (double) impix / (double)(/*4*/uvformat *
                                    diff_u)));    // keep croma u snr for first frame
        snr->snr_v1 = (10.0 * log10(maxSignal * (double) impix / (double)(/*4*/uvformat *
                                    diff_v)));    // keep croma v snr for first frame
        snr->snr_ya = snr->snr_y1;
        snr->snr_ua = snr->snr_u1;
        snr->snr_va = snr->snr_v1;
    }
    // B pictures
    else {
        if (input->bg_enable) {
            if (!(img->typeb == BACKGROUND_IMG && he->background_output_flag == 0)) {
                snr->snr_ya = (snr->snr_ya * (double)(img->number - he->background_number + hc->Bframe_ctr) + snr->snr_y) / (double)(
                                  img->number - he->background_number + hc->Bframe_ctr + 1);  // average snr lume for all frames inc. first
                snr->snr_ua = (snr->snr_ua * (double)(img->number - he->background_number + hc->Bframe_ctr) + snr->snr_u) / (double)(
                                  img->number - he->background_number + hc->Bframe_ctr + 1);  // average snr u croma for all frames inc. first
                snr->snr_va = (snr->snr_va * (double)(img->number - he->background_number + hc->Bframe_ctr) + snr->snr_v) / (double)(
                                  img->number - he->background_number + hc->Bframe_ctr + 1);  // average snr v croma for all frames inc. first
            }
        } else {
            snr->snr_ya = (snr->snr_ya * (double)(img->number + hc->Bframe_ctr) + snr->snr_y) / (double)(
                              img->number + hc->Bframe_ctr + 1);  // average snr lume for all frames inc. first
            snr->snr_ua = (snr->snr_ua * (double)(img->number + hc->Bframe_ctr) + snr->snr_u) / (double)(
                              img->number + hc->Bframe_ctr + 1);  // average snr u croma for all frames inc. first
            snr->snr_va = (snr->snr_va * (double)(img->number + hc->Bframe_ctr) + snr->snr_v) / (double)(
                              img->number + hc->Bframe_ctr + 1);  // average snr v croma for all frames inc. first
        }

    }
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
static void Report_frame(int tmp_time)
{

    FILE *file = fopen("stat.dat", "at");
    char *Frmfld;
    char Frm[] = "FRM";
    char Fld[] = "FLD";
    if (img->number == 0) {
        fprintf(file, "\n -------------------- DEBUG_INFO_START -------------------- \n");
    }
    if (img->picture_structure) {
        Frmfld = Frm;
    } else {
        Frmfld = Fld;
    }
#if INTERLACE_CODING
    if (input->InterlaceCodingOption == 3) {
        Frmfld = Fld;
    }
#endif

    if (img->typeb == BACKGROUND_IMG && input->bg_enable) {
        const char *typ = (input->bg_input_number == 0) ? "GB" : "G";
        fprintf(file, "%3d(%s) %8lld %4d %7.4f %7.4f %7.4f  %6d    %3s %5d %s",
                img->tr + hc->total_frames * 256, typ, stat->bit_ctr - stat->bit_ctr_n, img->qp, snr->snr_y,
                snr->snr_u, snr->snr_v, tmp_time, Frmfld, he->intras, MD5str);
        printf("%3d(%s) %8lld %4d %7.4f %7.4f %7.4f  %6d    %3s %5d %s",
               img->tr + hc->total_frames * 256, typ, stat->bit_ctr - stat->bit_ctr_n, img->qp, snr->snr_y,
               snr->snr_u, snr->snr_v, tmp_time, Frmfld, he->intras, MD5str);
    } else {
        char typ = (img->type == INTRA_IMG) ? 'I' : (img->type == INTER_IMG) ? ((img->typeb == BP_IMG) ? 'S' : 'P') :
                   (img->type == F_IMG ? 'F' : 'B');
        if (typ != 'I') {
            fprintf(file, "%3d(%c) %8lld %4d %7.4f %7.4f %7.4f  %6d    %3s %5d %s",
                    img->tr + hc->total_frames * 256, typ, stat->bit_ctr - stat->bit_ctr_n, img->qp, snr->snr_y,
                    snr->snr_u, snr->snr_v, tmp_time, Frmfld, he->intras, MD5str);
            printf("%3d(%c) %8lld %4d %7.4f %7.4f %7.4f  %6d    %3s %5d %s",
                   img->tr + hc->total_frames * 256, typ, stat->bit_ctr - stat->bit_ctr_n, img->qp, snr->snr_y,
                   snr->snr_u, snr->snr_v, tmp_time, Frmfld, he->intras, MD5str);
        } else {
            fprintf(file, "%3d(%c) %8lld %4d %7.4f %7.4f %7.4f  %6d    %3s       %s",
                    img->tr + hc->total_frames * 256, typ, stat->bit_ctr - stat->bit_ctr_n, img->qp, snr->snr_y,
                    snr->snr_u, snr->snr_v, tmp_time, Frmfld, MD5str);
            printf("%3d(%c) %8lld %4d %7.4f %7.4f %7.4f  %6d    %3s       %s",
                   img->tr + hc->total_frames * 256, typ, stat->bit_ctr - stat->bit_ctr_n, img->qp, snr->snr_y,
                   snr->snr_u, snr->snr_v, tmp_time, Frmfld, MD5str);
        }
    }

    printf(" %s\n", hc->str_list_reference);
    fprintf(file, " %s\n", hc->str_list_reference);

    fclose(file);
}

/*
*************************************************************************
* Function:Copies contents of a Sourceframe structure into the old-style
*    variables imgY_org and imgUV_org.  No other side effects
* Input:  sf the source frame the frame is to be taken from
* Output:
* Return:
* Attention:
*************************************************************************
*/

// image padding, 20071009
static void CopyFrameToOldImgOrgVariables()
{
    int x, y;
    byte *u_buffer, *v_buffer;
    int input_height_cr = input->chroma_format == 1 ? input->img_height / 2 : input->img_height;
    int img_height_cr   = input->chroma_format == 1 ? img->height / 2 : img->height;
#if INTERLACE_CODING
    if (input->InterlaceCodingOption == 3) {
        u_buffer = he->imgY_org_buffer + input->org_img_width * input->org_img_height;
    } else
#endif
        u_buffer = he->imgY_org_buffer + input->img_width * input->img_height;

    if (input->chroma_format == 1) {
#if INTERLACE_CODING
        if (input->InterlaceCodingOption == 3) {
            v_buffer = he->imgY_org_buffer + input->org_img_width * input->org_img_height * 5 / 4;
        } else
#endif
            v_buffer = he->imgY_org_buffer + input->img_width * input->img_height * 5 / 4;
    }

    // Y
    if (!(input->chroma_format == 2 && input->yuv_structure == 1)) {      //20080721,422
        for (y = 0; y < input->img_height; y++) {
            for (x = 0; x < input->img_width; x++) {
#if INTERLACE_CODING
                if (input->InterlaceCodingOption == 3) {
                    he->imgY_org [y][x] = he->imgY_org_buffer[(2 * y + (img->tr + hc->total_frames * 256) % 2) * input->img_width +
                                          x];    //img->tr%2==0: top field, img->tr%2=1: bot field
                } else {
                    he->imgY_org[y][x] = he->imgY_org_buffer[y * input->img_width + x];
                }
#else
                he->imgY_org [y][x] = he->imgY_org_buffer[y * input->img_width + x];
#endif
            }
        }
    }

    // UV
    if (!(input->chroma_format == 2 && input->yuv_structure == 1)) {      //20080721,422
        for (y = 0; y < input_height_cr; y++) {   //X ZHENG, 422
            for (x = 0; x < input->img_width / 2; x++) {
#if INTERLACE_CODING
                if (input->InterlaceCodingOption == 3) {
                    he->imgUV_org[0][y][x] = u_buffer[(2 * y + (img->tr + hc->total_frames * 256) % 2) * input->img_width / 2 + x];
                    he->imgUV_org[1][y][x] = v_buffer[(2 * y + (img->tr + hc->total_frames * 256) % 2) * input->img_width / 2 + x];
                } else {
                    he->imgUV_org[0][y][x] = u_buffer[y * input->img_width / 2 + x];
                    he->imgUV_org[1][y][x] = v_buffer[y * input->img_width / 2 + x];
                }
#else
                he->imgUV_org[0][y][x] = u_buffer[y * input->img_width / 2 + x];
                he->imgUV_org[1][y][x] = v_buffer[y * input->img_width / 2 + x];
#endif
            }
        }
    }

    // Y's width padding
    for (y = 0; y < input->img_height; y++) {
        for (x = input->img_width; x < img->width; x++) {
            he->imgY_org[y][x] = he->imgY_org[y][x - 1];
        }
    }

    //Y's padding bottom border
    for (y = input->img_height; y < img->height; y++) {
        for (x = 0; x < img->width; x++) {
            he->imgY_org[y][x] = he->imgY_org[y - 1][x];
        }
    }

    // UV's padding width
    for (y = 0; y < input_height_cr; y++) {
        for (x = input->img_width / 2; x < img->width / 2; x++) {
            he->imgUV_org[0][y][x] = he->imgUV_org[0][y][x - 1];
            he->imgUV_org[1][y][x] = he->imgUV_org[1][y][x - 1];
        }
    }

    // UV's padding bottom border
    for (y = input_height_cr; y < img_height_cr; y++) {
        for (x = 0; x < img->width / 2; x++) {
            he->imgUV_org[0][y][x] = he->imgUV_org[0][y - 1][x];
            he->imgUV_org[1][y][x] = he->imgUV_org[1][y - 1][x];
        }
    }
}
// image padding, 20071009

/*
*************************************************************************
* Function: Calculates the absolute frame number in the source file out
of various variables in img-> and input->
* Input:
* Output:
* Return: frame number in the file to be read
* Attention: \side effects
global variable frame_no updated -- dunno, for what this one is necessary
*************************************************************************
*/


static int CalculateFrameNumber()
{
    // printf the position of frame in input sequence
    printf("%5d", img->tr + hc->total_frames * 256);
    return (img->tr + hc->total_frames * 256);
}

/*
*************************************************************************
* Function:Reads one new frame from file
* Input: FrameNoInFile: Frame number in the source file
HeaderSize: Number of bytes in the source file to be skipped
xs: horizontal size of frame in pixels, must be divisible by 16
ys: vertical size of frame in pixels, must be divisible by 16 or
32 in case of MB-adaptive frame/field coding
sf: Sourceframe structure to which the frame is written
* Output:
* Return:
* Attention:
*************************************************************************
*/
static void ReadOneFrame(int FrameNoInFile, int FrameSkip, int xs, int ys)
{
    //const unsigned int  bytes_y = input->img_width *input->stuff_height;    //modify by wuzhongmou 0610
    int input_width_cr  = (input->img_width / 2);                             //20080721
    int input_height_cr = (input->img_height / (input->chroma_format == 1 ? 2 : 1));
    unsigned int  bytes_y = input->img_width * input->img_height;             //add by wuzhongmou 0610
#if INTERLACE_CODING
    unsigned int  bytes_uv = input_width_cr * input_height_cr;
    long long int framesize_in_bytes = bytes_y + 2 * bytes_uv;
#else
    const unsigned int  bytes_uv = input_width_cr * input_height_cr;
    const long long int framesize_in_bytes = bytes_y + 2 * bytes_uv;
#endif
    //int stuff_height_cr = (input->img_height-input->stuff_height)/2;
    int off_y  = input->img_width * input->img_height;
    int off_uv = input_width_cr * input_height_cr;
    unsigned char *tempPix;
    int nSampleSize = (input->input_sample_bit_depth == 8 ? 1 : 2);
    int offSet;
    unsigned int i;
    int shift1 = input->sample_bit_depth - input->input_sample_bit_depth;

#if INTERLACE_CODING
    int RealFrameNoInFile = FrameNumberInFile / (input->InterlaceCodingOption == 3 ? 2 :
                            1);  //In case of field picture coding, two field picture share one picture buffer that is stored as interleave format
    if (input->InterlaceCodingOption == 3) {
        input_width_cr  = (input->org_img_width / 2);                             //20080721
        input_height_cr = (input->org_img_height / (input->chroma_format == 1 ? 2 : 1));
        bytes_y = input->org_img_width * input->org_img_height;             //add by wuzhongmou 0610
        bytes_uv = input_width_cr * input_height_cr;
        framesize_in_bytes = bytes_y + 2 * bytes_uv;
        off_y  = input->org_img_width * input->org_img_height;
        off_uv = input_width_cr * input_height_cr;
    }
#endif



    assert(FrameNumberInFile == FrameNoInFile);


#if INTERLACE_CODING
    if (fseek(he->p_in, (long long int)(framesize_in_bytes * (RealFrameNoInFile + FrameSkip) * nSampleSize), SEEK_SET) != 0)
#else
    if (fseek(he->p_in, (long long int)(framesize_in_bytes * (FrameNoInFile + FrameSkip) * nSampleSize), SEEK_SET) != 0)
#endif

    {
        error("ReadOneFrame: cannot fseek to (Header size) in p_in", -1);
    }
    if (fread((void *)he->imgY_org_buffer, 1, (bytes_y * nSampleSize), he->p_in) != (unsigned)(bytes_y * nSampleSize)) {
        printf("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", (bytes_y * nSampleSize));
        exit(-1);
    }

    if (fread((void *)(he->imgY_org_buffer + off_y), 1, (bytes_uv * nSampleSize),
              he->p_in) != (unsigned)(bytes_uv * nSampleSize)) {
        printf("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", (bytes_uv * nSampleSize));
        exit(-1);
    }

    if (fread((void *)(he->imgY_org_buffer + off_y + off_uv), 1, (bytes_uv * nSampleSize),
              he->p_in) != (unsigned)(bytes_uv * nSampleSize)) {
        printf("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", (bytes_uv * nSampleSize));
        exit(-1);
    }

    if (input->input_sample_bit_depth == 8) {
        for (i = 0; i < bytes_y; i ++) {
            tempPix = (unsigned char *)(&he->imgY_org_buffer[bytes_y - i - 1]);
            tempPix[0] = *((unsigned char *)(&he->imgY_org_buffer[(bytes_y - i - 1) / 2]) + ((i + 1) % 2));
            tempPix[1] = 0; // reset high byte
        }
        offSet = off_y;
        for (i = 0; i < bytes_uv; i ++) {
            tempPix = (unsigned char *)(&he->imgY_org_buffer[offSet + (bytes_uv - i - 1)]);
            tempPix[0] = *((unsigned char *)(&he->imgY_org_buffer[offSet + (bytes_uv - i - 1) / 2]) + ((i + 1) % 2));
            tempPix[1] = 0; // reset high byte
        }
        offSet = (offSet + off_uv);
        for (i = 0; i < bytes_uv; i ++) {
            tempPix = (unsigned char *)(&he->imgY_org_buffer[offSet + (bytes_uv - i - 1)]);
            tempPix[0] = *((unsigned char *)(&he->imgY_org_buffer[offSet + (bytes_uv - i - 1) / 2]) + ((i + 1) % 2));
            tempPix[1] = 0; // reset high byte
        }

        for (i = 0; i <  bytes_y + bytes_uv * 2; i ++) {
            he->imgY_org_buffer[i] = he->imgY_org_buffer[i] << shift1; // shift 8 bit samples to 10 bit samples
        }
    }
    if (input->input_sample_bit_depth == 10) {
        for (i = 0; i < bytes_y + bytes_uv * 2; i ++) {
            he->imgY_org_buffer[i] = he->imgY_org_buffer[i] & 0x3ff; // only keep 10 bits
        }
    }
    if (input->input_sample_bit_depth == 12) {
        for (i = 0; i < bytes_y + bytes_uv * 2; i ++) {
            he->imgY_org_buffer[i] = he->imgY_org_buffer[i] & 0xfff; // only keep 12 bits
        }
    }
}

/*
*************************************************************************
* Function:point to frame coding variables
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void put_buffer_frame()
{
    int i, j;

    //integer pixel for chroma
#if FIX_MAX_REF
    for (j = 0; j < MAXREF; j++) {
#else
    for (j = 0; j < 4; j++) {
#endif
        for (i = 0; i < 2; i++) {
            he->integerRefUV[j][i]   = fref[j]->ref[i + 1];
        }
    }

    //integer pixel for luma
#if FIX_MAX_REF
    for (i = 0; i < MAXREF; i++) {
#else
    for (i = 0; i < 4; i++) {
#endif
        he->Refbuf11[i] = &fref[i]->ref[0][0][0];
    }

    //current reconstructed image

    hc->imgY = hc->currentFrame[0];
    hc->imgUV = &hc->currentFrame[1];

}
/*
*************************************************************************
* Function:update the decoder picture buffer
* Input:frame number in the bitstream and the video sequence
* Output:
* Return:
* Attention:
*************************************************************************
*/
//edit start added by lzhang for AEC
void write_terminating_bit(unsigned char bit)
{
    DataPartition          *dataPart;
    EncodingEnvironmentPtr  eep_dp;

    //--- write non-slice termination symbol if the codingUnit is not the first one in its slice ---
    dataPart = & (img->currentSlice->partArr[0]);
    dataPart->bitstream->write_flag = 1;
    eep_dp   = & (dataPart->ee_AEC);

    biari_encode_symbol_final(eep_dp, bit);
#if TRACE
    if (he->AEC_writting) {
        fprintf(hc->p_trace, "      aec_mb_stuffing_bit = %d\n", bit);
    }
#endif

}


void free_slice()
{
    Slice *slice = img->currentSlice;

    if (slice != NULL) {
        if (slice->partArr != NULL) {
            free(slice->partArr);
        }

        delete_contexts_SyntaxInfo(slice->syn_ctx);

        //free(img->currentSlice);
        free(slice);
    }
}
//edit end added by lzhang for AEC
