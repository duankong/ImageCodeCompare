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



/*!
***********************************************************************
*  \mainpage
*     This is the AVS decoder reference software. For detailed documentation
*     see the comments in each file.
*
*  \author
*     The main contributors are listed in contributors.h
*
*
*  \note
*     tags are used for document system "doxygen"
*     available at http://www.doxygen.org
*
*  \par Limitations:
*     Using different NAL's the assignment of partition-id to containing
*     syntax elements may got lost, if this information is not transmitted.
*     The same has to be stated for the partitionlength if partitions are
*     merged by the NAL.
*  \par
*     The presented solution in Q15-K-16 solves both of this problems as the
*     departitioner parses the bitstream before decoding. Due to syntax element
*     dependencies both, partition bounds and partitionlength information can
*     be parsed by the departitioner.
*
*  \par Handling partition information in external file:
*     As the TML is still a work in progress, it makes sense to handle this
*     information for simplification in an external file, here called partition
*     information file, which can be found by the extension .dp extending the
*     original encoded AVS bitstream. In this file partition-ids followed by its
*     partitionlength is written. Instead of parsing the bitstream we get the
*     partition information now out of this file.
*     This information is assumed to be never sent over transmission channels
*     (simulation scenarios) as it's information we allways get using a
*     "real" departitioner before decoding
*
*  \par Extension of Interim File Format:
*     Therefore a convention has to be made within the interim file format.
*     The underlying NAL has to take care of fulfilling these conventions.
*     All partitions have to be bytealigned to be readable by the decoder,
*     So if the NAL-encoder merges partitions, >>this is only possible to use the
*     VLC structure of the AVS bitstream<<, this bitaligned structure has to be
*     broken up by the NAL-decoder. In this case the NAL-decoder is responsable to
*     read the partitionlength information from the partition information file.
*     Partitionlosses are signaled with a partition of zero length containing no
*     syntax elements.
*
*/

/*
*************************************************************************************
* File name: ldecod.c
* Function: TML decoder project main()
*
*************************************************************************************
*/
#include "../../lcommon/inc/contributors.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/timeb.h>
#include <assert.h>
#include <stdio.h>

#include "../../lcommon/inc/defines.h"
#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/memalloc.h"

#include "annexb.h"
#include "header.h"
#include "AEC.h"
#include "../../lcommon/inc/ComAdaptiveLoopFilter.h"
#include "../../ldecod/inc/DecAdaptiveLoopFilter.h"
#if ROI_M3264
#include "pos_info.h"
#endif

// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#include "wquant.h"
#endif





#define LOGFILE     "log.dec"
#define DATADECFILE "dataDec.txt"
#define TRACEFILE   "trace_dec.txt"

#define _S_IREAD        0000400         /* read permission, owner */
#define _S_IWRITE       0000200         /* write permission, owner */

extern FILE *bits;

struct inp_par    *input;       //!< input parameters from input configuration file
SNRParameters    *snr;         //!< statistics
ImageParameters    *img;         //!< image parameters
StatBits *StatBitsPtr;

Bitstream *currStream;
FILE *reffile, *reffile2;


#include "../../lcommon/inc/md5.h"
unsigned int   MD5val[4];
char           MD5str[33];



/*
*************************************************************************
* Function:main function for decoder
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
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

#if OUTPUT_INTERLACE_MERGED_PIC
void readOneFrameFromDisk(long long int framesize_in_bytes, int FrameNoInFile, FILE *p_img, unsigned char   *buf)
{
    if (fseek(p_img, framesize_in_bytes * FrameNoInFile, SEEK_SET) != 0) {
        printf("ReadOneFrame: cannot fseek to (Header size) in p_img");
    }

    if (fread(buf, 1, (size_t)framesize_in_bytes, p_img) != (int) framesize_in_bytes) {
        printf("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", framesize_in_bytes);
    }
}

void mergeFld2Frm()
{
    int input_width_cr  = hd->horizontal_size / 2;
    int input_height_cr = hd->vertical_size;
    unsigned int  bytes_y = hd->horizontal_size * hd->vertical_size * 2;
    const unsigned int  bytes_uv = input_width_cr * input_height_cr;
    long long int framesize_in_bytes = bytes_y + 2 * bytes_uv;


    int off_y_fld = bytes_y / 2;
    int off_u_fld = hd->horizontal_size * hd->vertical_size / 4;
    int off_uv_fld = bytes_uv / 2;
    int off_yuv_fld = off_y_fld + 2 * off_uv_fld;

    unsigned char   *imgY_org_buf, *imgY_org_buf_fld, *imgY_out_buf;

    int diff_y, diff_u, diff_v;


    FILE *p_rec_img;
    FILE *p_out_img;

    char dst_fld[1000] = "\0";

    double maxSignal = (double)((1 << input->output_bit_depth) - 1) * (double)((1 << input->output_bit_depth) - 1);
    int nSampleSize = (input->output_bit_depth == 8 ? 1 : 2);
    int shift1 = input->sample_bit_depth - input->output_bit_depth;

    int i, w, h;

#define _S_IREAD      0000400         /* read permission, owner */
#define _S_IWRITE     0000200         /* write permission, owner */

    strcat(dst_fld, input->outfile);
    strcat(dst_fld, ".fld.yuv");

    diff_y = diff_u = diff_v = 0;

    imgY_org_buf = (unsigned char *) malloc(hd->horizontal_size * hd->vertical_size * 2 * 3 / 2);
    imgY_org_buf_fld = (unsigned char *) malloc(hd->horizontal_size * hd->vertical_size * 2 * 3 / 2);
    imgY_out_buf = (unsigned char *) malloc(hd->horizontal_size * hd->vertical_size * 2 * 3 / 2);

    if (input->output_dec_pic == 1 && (p_rec_img = fopen(input->outfile, "rb")) == NULL) {
        printf("Input file %s does not exist", input->outfile);
    }

    if (input->output_interlace_merged_picture)
        if (strlen(dst_fld) > 0 && (p_out_img = fopen(dst_fld, "wb")) == NULL) {
            printf("Input file %s does not exist", dst_fld);
        }

    for (i = 0; i < (img->number + hc->Bframe_ctr) / 2; i++) {
        diff_y = diff_u = diff_v = 0;
        //readOneFrameFromDisk(framesize_in_bytes * nSampleSize, i, p_org_img, imgY_org_buf);
        readOneFrameFromDisk(framesize_in_bytes * nSampleSize, i, p_rec_img, imgY_org_buf_fld);

        for (h = 0; h < hd->vertical_size; h++)
            for (w = 0; w < hd->horizontal_size; w++) {
                imgY_out_buf[2 * h * hd->horizontal_size + w] = imgY_org_buf_fld[h * hd->horizontal_size + w];
                imgY_out_buf[(2 * h + 1) * hd->horizontal_size + w] = imgY_org_buf_fld[off_yuv_fld + h * hd->horizontal_size + w];
            }

        for (h = 0; h < hd->vertical_size / 2; h++)
            for (w = 0; w < hd->horizontal_size / 2; w++) {
                imgY_out_buf[bytes_y + 2 * h * hd->horizontal_size / 2 + w] = imgY_org_buf_fld[off_y_fld + h * hd->horizontal_size / 2 +
                        w];
                imgY_out_buf[bytes_y + (2 * h + 1) * hd->horizontal_size / 2 + w] = imgY_org_buf_fld[off_yuv_fld + off_y_fld + h *
                        hd->horizontal_size / 2 + w];

                imgY_out_buf[bytes_y + 2 * off_u_fld + 2 * h * hd->horizontal_size / 2 + w] = imgY_org_buf_fld[off_y_fld + off_u_fld +
                        h * hd->horizontal_size / 2 + w];
                imgY_out_buf[bytes_y + 2 * off_u_fld + (2 * h + 1) * hd->horizontal_size / 2 + w] = imgY_org_buf_fld[off_yuv_fld +
                        off_u_fld + off_y_fld + h * hd->horizontal_size / 2 + w];
            }

        if (input->output_interlace_merged_picture) {
            fwrite(imgY_out_buf, sizeof(unsigned char), (size_t)framesize_in_bytes * nSampleSize, p_out_img);
            fflush(p_out_img);
        }
    }

    free(imgY_org_buf);
    free(imgY_org_buf_fld);
    free(imgY_out_buf);
    //fclose(p_org_img);
    fclose(p_rec_img);
    if (input->output_interlace_merged_picture) {
        fclose(p_out_img);
    }
}
#endif

int main(int argc, char **argv)
{
#if REF_OUTPUT
    int i, j;
#endif
    int k, c;
    // allocate memory for the structures
    if ((input = (struct inp_par *) calloc(1, sizeof(struct inp_par))) == NULL) {
        no_mem_exit("main: input");
    }

    if ((snr = (SNRParameters *) calloc(1, sizeof(SNRParameters))) == NULL) {
        no_mem_exit("main: snr");
    }


    if ((img = (ImageParameters *) calloc(1, sizeof(ImageParameters))) == NULL) {
        no_mem_exit("main: img");
    }


    if ((StatBitsPtr = (struct StatBits *) calloc(1, sizeof(struct StatBits))) == NULL) {
        no_mem_exit("main: StatBits");
    }


    img->seq_header_indicate = 0;
    img->B_discard_flag = 0;
    StatBitsPtr->curr_frame_bits = StatBitsPtr->prev_frame_bits
                                   = StatBitsPtr->emulate_bits = StatBitsPtr->last_unit_bits = 0; //rm52k_r2
    StatBitsPtr->bitrate = StatBitsPtr->coded_pic_num = StatBitsPtr->time_s = 0; //rm52k_r2

    currStream = AllocBitstream();

    hd->eos = 0;

    init_conf(argc, argv);

    OpenBitstreamFile(input->infile);

    if (input->ref_pic_order) {   //ref order
        hd->dec_ref_num = 0;
    }

    malloc_slice();
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


    outprint.buffer_num = 0;

    hd->last_output = -1;
    hd->end_SeqTr = -1;
    hd->curr_IDRtr = 0;
    hd->curr_IDRcoi = 0;
    hd->next_IDRtr = 0;
    hd->next_IDRcoi = 0;
    // Allocate Slice data struct
    img->number = 0;
    img->type = I_IMG;

    img->imgtr_next_P = 0;

    img->imgcoi_next_ref = 0;


    img->num_of_references = 0;
    hc->seq_header = 0;

    img->new_sequence_flag   = 1;

    hd->vec_flag = 0;

    hd->FrameNum = 0;

    // B pictures
    hc->Bframe_ctr = 0;
    hc->total_frames = 0;

    // time for total decoding session
    hc->tot_time = 0;

    do {

        while ((decode_one_frame(snr) != EOS) && (!IsEndOfBitstream()));

    } while (!IsEndOfBitstream());
    if (input->alf_enable) {
        ReleaseAlfGlobalBuffer();
    }
    if (StatBitsPtr->time_s == 0) {
        StatBitsPtr->total_bitrate[StatBitsPtr->time_s++] = StatBitsPtr->bitrate;
    }

    hd->eos = 1;


#if REF_OUTPUT
    for (j = 0; j < outprint.buffer_num; j++) {
        int tmp_min, pos = -1;
        tmp_min = 1 << 20;

        for (i = 0; i < outprint.buffer_num; i++) {
            if (outprint.stdoutdata[i].tr < tmp_min && outprint.stdoutdata[i].tr > hd->last_output) {
                pos = i;
                tmp_min = outprint.stdoutdata[i].tr;
            }
        }
        if (pos != -1) {
            hd->last_output = outprint.stdoutdata[pos].tr;
            report_frame(outprint, pos);
            write_frame(hd->p_out, outprint.stdoutdata[pos].tr);

        }
    }
#endif



    report(snr);


    free_mem2Dint(AVS_SCAN8x8);
    free_mem2Dint(AVS_SCAN16x16);
    free_mem2Dint(AVS_SCAN32x32);
    free_mem2Dint(AVS_SCAN4x4);
    free_mem2Dint(AVS_CG_SCAN8x8);
    free_mem2Dint(AVS_CG_SCAN16x16);
    free_mem2Dint(AVS_CG_SCAN32x32);
    FreeBitstream();
    free_slice();

    free_global_buffers();



    CloseBitstreamFile();

#if ROI_M3264
    ClosePosFile();
#endif

    fclose(hd->p_out);

    if (hd->p_ref) {
        fclose(hd->p_ref);
    }
    if (hd->p_out_background) {
        fclose(hd->p_out_background);
    }
    if (hd->p_ref_background) {
        fclose(hd->p_ref_background);
    }

    if (input->MD5Enable & 0x01) {
//    clock_t start = clock();
        long long filelength;

//    start = clock();
        filelength = FileMD5(input->infile, MD5val);
        printf("\n================================================================\n");
        printf("Input AVS Stream Size: %lld bit\n", filelength << 3);
        printf("Input AVS Stream MD5 : %08X%08X%08X%08X\n", MD5val[0], MD5val[1], MD5val[2], MD5val[3]);
//    printf("MD5 Calculation Time : %.3f sec\n", 1.0*(clock()-start)/CLOCKS_PER_SEC);
        printf("----------------------------------------------------------------\n");
//    start = clock();
        filelength = FileMD5(input->outfile, MD5val);
        printf("Decoded YUV Size     : %lld byte\n", filelength);
        printf("Decoded YUV MD5      : %08X%08X%08X%08X\n", MD5val[0], MD5val[1], MD5val[2], MD5val[3]);
//    printf("MD5 Calculation Time : %.3f sec\n",1.0 * (clock() - start) / CLOCKS_PER_SEC);
        printf("================================================================\n");
    }

#if TRACE
    fclose(hc->p_trace);
#endif

    free_mem2Dint(AVS_SCAN4x16);
    free_mem2Dint(AVS_SCAN16x4);
    free_mem2Dint(AVS_SCAN2x8);
    free_mem2Dint(AVS_SCAN8x2);
    free_mem2Dint(AVS_SCAN32x8);
    free_mem2Dint(AVS_SCAN8x32);

    free(input);

    free(snr);
    free(StatBitsPtr);   //1105

    free(img);
    printf("\n=============== Check value bound ==============================\n");
    printf("value_s_bound: %d\n", NUN_VALUE_BOUND);
    printf("max_value_s: %d\n", max_value_s);
    printf("================================================================\n");
    return 0;
}

/*
*************************************************************************
* Function:Initilize some arrays
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void init()  //!< image parameters
{
    int i;

    // initilize quad mat`rix used in snr routine
    for (i = 0; i < (1 << input->sample_bit_depth); i++) {
        img->quad[i] = i * i;
    }

}

/*
*************************************************************************
* Function:Read input from configuration file
* Input:Name of configuration filename
* Output:
* Return:
* Attention:
*************************************************************************
*/

void init_conf(int numpar, char **config_str)
{
    FILE *fd;
    int         i = 0;


    input->check_BBV_flag = 0;

    // read the decoder configuration file

#if OUTPUT_INTERLACE_MERGED_PIC

    if (numpar != 1 && numpar != 2 && numpar != 9  && numpar != 10)
#else
    if (numpar != 1 && numpar != 2 && numpar != 9)
#endif
    {
        snprintf(hc->errortext, ET_SIZE,
                 "Usage: %s <config.dat> <avsfilename> <decfilename> <reffilename> <ref_num> <lfdisable> <yuv_structure> <bbv_check>\n \
      ?-the decoder can be configured by configfile or by command line \n \
      ?-param with only <config.dat> defines decoder with file\n \
      ?-params with allof the params define decoder params with command line\n \
      ", config_str[0]);
        error(hc->errortext, 300);
    }

    if (numpar == 1) {
        fprintf(stdout, "**************************************************************************\n");
        fprintf(stdout, "The decoder config file doesn't exist. Use the default parameters instead!\n");
        fprintf(stdout, "**************************************************************************\n\n");
    } else {
        if ((fd = fopen(config_str[1], "r")) == NULL) {
            snprintf(hc->errortext, ET_SIZE, "Error: Control file %s not found\n", config_str[1]);
            error(hc->errortext, 300);
        }
    }

    strcpy(input->infile, "test.avs");

    strcpy(input->outfile, "test_dec.yuv");
    strcpy(input->reffile, "test_rec.yuv");

#if B_BACKGROUND_Fix
	strcpy(input->backgroundref_File,"background_ref.yuv");
#endif

#if ROI_M3264
    strcpy(input->out_datafile, "out_position_data.txt");
#endif
    input->buf_cycle = 2;
    input->LFParametersFlag = 0;
    input->yuv_structure = 0;
    input->check_BBV_flag = 0;
    input->ref_pic_order  = 0;
    input->output_dec_pic = 1;

    input->MD5Enable = 0;
    memset(MD5val, 0, 16);
    memset(MD5str, 0, 33);

#if OUTPUT_INTERLACE_MERGED_PIC
    input->output_interlace_merged_picture = 0;
#endif

    if (numpar == 2) {
        fscanf(fd, "%s", input->infile);              // AVS compressed input bitsream
        fscanf(fd, "%*[^\n]");


        fscanf(fd, "%s", input->outfile);             // YUV output format
        fscanf(fd, "%*[^\n]");

        fscanf(fd, "%s", input->reffile);             // reference file
        fscanf(fd, "%*[^\n]");

        fscanf(fd, "%d,", &input->yuv_structure);
        fscanf(fd, "%*[^\n]");

        fscanf(fd, "%d,", &input->ref_pic_order);
        fscanf(fd, "%*[^\n]");

        fscanf(fd, "%d,", &input->output_dec_pic);
        fscanf(fd, "%*[^\n]");

        fscanf(fd, "%d,", &input->MD5Enable);
        fscanf(fd, "%*[^\n]");

#if OUTPUT_INTERLACE_MERGED_PIC
        fscanf(fd, "%d,", &input->output_interlace_merged_picture);
        fscanf(fd, "%*[^\n]");
#endif
#if B_BACKGROUND_Fix
		fscanf(fd, "%s", input->backgroundref_File);     // 
		fscanf(fd, "%*[^\n]");
#endif
    }



    if (numpar >= 9) {
        i = 2;
        strcpy(input->infile, config_str[i++]);

        strcpy(input->outfile, config_str[i++]);
        strcpy(input->reffile, config_str[i++]);

        input->yuv_structure   = atoi(config_str[i++]);
        input->ref_pic_order   = atoi(config_str[i++]);
        input->output_dec_pic  = atoi(config_str[i++]);
        input->MD5Enable       = atoi(config_str[i++]);

#if OUTPUT_INTERLACE_MERGED_PIC
        if (numpar > i) {
            input->output_interlace_merged_picture = atoi(config_str[i++]);
        }
#endif
    }

#if TRACE

    if ((hc->p_trace = fopen(TRACEFILE, "w")) == 0) {        // append new statistic at the end
        snprintf(hc->errortext, ET_SIZE, "Error open file %s!", TRACEFILE);
        error(hc->errortext, 500);
    }

#endif



    if (input->output_dec_pic) {   //output_dec_pic

        if ((hd->p_out = fopen(input->outfile, "wb")) == 0) {
            snprintf(hc->errortext, ET_SIZE, "Error open file %s ", input->outfile);
            error(hc->errortext, 500);
        }
    }

    fprintf(stdout, "--------------------------------------------------------------------------\n");
    fprintf(stdout, " Decoder config file                    : %s \n", config_str[1]);
    fprintf(stdout, "--------------------------------------------------------------------------\n");
    fprintf(stdout, " Input AVS bitstream                    : %s \n", input->infile);

    fprintf(stdout, " Output decoded YUV 4:2:0               : %s \n", input->outfile);

    fprintf(stdout, " Output status file                     : %s \n", LOGFILE);


    if ((hd->p_ref = fopen(input->reffile, "rb")) == 0) {
        fprintf(stdout, " Input reference file                   : %s does not exist \n", input->reffile);
        fprintf(stdout, "                                          SNR values are not available\n");
        hd->RefPicExist = 0;  // 20071224
    } else {
        fprintf(stdout, " Input reference file                   : %s \n", input->reffile);
        hd->RefPicExist = 1;  // 20071224
    }

    fprintf(stdout, "--------------------------------------------------------------------------\n");

    if (input->MD5Enable & 0x02) {
        fprintf(stdout, " Frame   TR    QP   SnrY    SnrU    SnrV   Time(ms)   FRM/FLD  Bits  EmulateBits\tYUV_MD5\n");
    } else {
        fprintf(stdout, " Frame   TR    QP   SnrY    SnrU    SnrV   Time(ms)   FRM/FLD  Bits  EmulateBits\n");
    }
}

/*
*************************************************************************
* Function:Reports the gathered information to appropriate outputs
* Input:
struct inp_par *input,
ImageParameters *img,
SNRParameters *stat
* Output:
* Return:
* Attention:
*************************************************************************
*/

void report(SNRParameters *snr)
{
    char string[OUTSTRING_SIZE];
    FILE *p_log;

#ifndef WIN32
    time_t  now;
    struct tm *l_time;
#else
    char timebuf[128];
#endif

    if (img->sequence_end_flag) {
        fprintf(stdout, "Sequence End\n\n");
    }
    {
        int i;
        float framerate[8] = {24000 / 1001, 24, 25, 30000 / 1001, 30, 50, 60000 / 1001, 60};

        if ((int)(StatBitsPtr->coded_pic_num - (StatBitsPtr->time_s + 1) *framerate[hd->frame_rate_code - 1] + 0.5) == 0) {
            StatBitsPtr->total_bitrate[StatBitsPtr->time_s++] = StatBitsPtr->bitrate;
            StatBitsPtr->bitrate = 0;
        }

        StatBitsPtr->total_bitrate[StatBitsPtr->time_s - 1] += 32;
        printf("Second(s)\tBitrate(bit/s)\n");

        for (i = 0; i < StatBitsPtr->time_s; i++) {
            printf(" %3d\t\t %d\n", i, StatBitsPtr->total_bitrate[i]);
        }

        if (StatBitsPtr->time_s == 0) {
            printf(" %3d\t\t %d\n", 0, StatBitsPtr->total_bitrate[0]);
        }
    }

    fprintf(stdout, "-------------------- Average SNR all frames ------------------------------\n");
    fprintf(stdout, " SNR Y(dB)           : %5.2f\n", snr->snr_ya);
    fprintf(stdout, " SNR U(dB)           : %5.2f\n", snr->snr_ua);
    fprintf(stdout, " SNR V(dB)           : %5.2f\n", snr->snr_va);
    fprintf(stdout, " Total decoding time : %.3f sec \n", hc->tot_time * 0.001);
    fprintf(stdout, "--------------------------------------------------------------------------\n");
    fprintf(stdout, " Output file bit depth                  : %d \n", input->output_bit_depth);
    fprintf(stdout, "--------------------------------------------------------------------------\n");


    fprintf(stdout, " Exit RD %s decoder, ver %s ", RD, VERSION);
    fprintf(stdout, "\n");

    snprintf(string, OUTSTRING_SIZE, "%s", LOGFILE);

    if ((p_log = fopen(string, "r")) == 0) {             // check if file exist
        if ((p_log = fopen(string, "a")) == 0) {
            snprintf(hc->errortext, ET_SIZE, "Error open file %s for appending", string);
            error(hc->errortext, 500);
        } else {                                          // Create header to new file
#ifdef M3624MDLOG
            fprintf(p_log,
                    " ----------------------------------------------------------------------------------------------------------------------------\n");
            fprintf(p_log,
                    "| Decoder statistics. This file is made first time, later runs are appended                                                 |\n");
            fprintf(p_log,
                    " --------------------------------------------------------------------------------------------------------------------------- \n");
            fprintf(p_log,
                    "|Date |Time | AVS Stream                                 |#Img|  Format |SNRY 1|SNRU 1|SNRV 1|SNRY N|SNRU N|SNRV N| Decode T|\n");
            fprintf(p_log,
                    " ----------------------------------------------------------------------------------------------------------------------------\n");
#endif
        }
    } else {
        fclose(p_log);
        p_log = fopen(string, "a");                 // File exist,just open for appending
    }

#ifdef WIN32
    _strdate(timebuf);
    fprintf(p_log, "|%1.5s|", timebuf);

    _strtime(timebuf);
    fprintf(p_log, "%1.5s|", timebuf);
#else
    now = time((time_t *) NULL);      // Get the system time and put it into 'now' as 'calender time'
    time(&now);
    l_time = localtime(&now);
    strftime(string, sizeof string, "%d-%b-%Y", l_time);
    fprintf(p_log, "| %1.5s |", string);

    strftime(string, sizeof string, "%H:%M:%S", l_time);
    fprintf(p_log, "| %1.5s |", string);
#endif

#ifdef M3624MDLOG
    {
        char *ch;
        int i;
        for (i = (int)strlen(input->infile); i >= 0 && input->infile[i] != '\\'; ch = &input->infile[i--]);
        if (strlen(ch) > 44) {
            ch += strlen(ch) - 44;
        }
        fprintf(p_log, "%-44s|", ch);
    }
    fprintf(p_log, "%4d|", img->number);
    fprintf(p_log, "%4dx%-4d|", img->width, img->height);
    fprintf(p_log, "%6.3f|", snr->snr_y1);
    fprintf(p_log, "%6.3f|", snr->snr_u1);
    fprintf(p_log, "%6.3f|", snr->snr_v1);
    fprintf(p_log, "%6.3f|", snr->snr_ya);
    fprintf(p_log, "%6.3f|", snr->snr_ua);
    fprintf(p_log, "%6.3f|", snr->snr_va);
    fprintf(p_log, "%9.2f|\n", hc->tot_time * 0.001);
#endif
    fclose(p_log);

#ifdef REPORT
    {
        char *ch;
        FILE *logtable = fopen("DecRD.log", "a+");
        int i;
        for (i = strlen(input->reffile); i >= 0 && input->reffile[i] != '\\'; ch = &input->reffile[i--]);
        fprintf(logtable, "%dx%d\t", img->width, img->height);
        fprintf(logtable, "%s\t", ch);
        fprintf(logtable, "%d\t", img->qp);
        fprintf(logtable, "%.3f\t", hc->tot_time * 0.001);
        fprintf(logtable, "\n");
        fclose(logtable);
    }
#endif

    snprintf(string, OUTSTRING_SIZE, "%s", DATADECFILE);
    p_log = fopen(string, "a");

    if (hc->Bframe_ctr != 0) {   // B picture used
        fprintf(p_log, "%3d %2d %2d %2.2f %2.2f %2.2f %5d "
                "%2.2f %2.2f %2.2f %5d "
                "%2.2f %2.2f %2.2f %5d %.3f\n",
                img->number, 0, img->qp,
                snr->snr_y1,
                snr->snr_u1,
                snr->snr_v1,
                0,
                0.0,
                0.0,
                0.0,
                0,
                snr->snr_ya,
                snr->snr_ua,
                snr->snr_va,
                0,
                (double) 0.001 * hc->tot_time / (img->number + hc->Bframe_ctr - 1));
    } else {
        fprintf(p_log, "%3d %2d %2d %2.2f %2.2f %2.2f %5d "
                "%2.2f %2.2f %2.2f %5d "
                "%2.2f %2.2f %2.2f %5d %.3f\n",
                img->number, 0, img->qp,
                snr->snr_y1,
                snr->snr_u1,
                snr->snr_v1,
                0,
                0.0,
                0.0,
                0.0,
                0,
                snr->snr_ya,
                snr->snr_ua,
                snr->snr_va,
                0,
                (double) 0.001 * hc->tot_time / img->number);
    }

    fclose(p_log);
}

/*
*************************************************************************
* Function:Allocates a Bitstream
* Input:
* Output:allocated Bitstream point
* Return:
* Attention:
*************************************************************************
*/

Bitstream *AllocBitstream()
{
    Bitstream *bitstream;

    bitstream = (Bitstream *) calloc(1, sizeof(Bitstream));

    if (bitstream == NULL) {
        snprintf(hc->errortext, ET_SIZE, "AllocBitstream: Memory allocation for Bitstream failed");
        error(hc->errortext, 100);
    }

    bitstream->streamBuffer = (unsigned char *) calloc(MAX_CODED_FRAME_SIZE, sizeof(unsigned char));

    if (bitstream->streamBuffer == NULL) {
        snprintf(hc->errortext, ET_SIZE, "AllocBitstream: Memory allocation for streamBuffer failed");
        error(hc->errortext, 100);
    }

    return bitstream;
}


/*
*************************************************************************
* Function:Frees a partition structure (array).
* Input:Partition to be freed, size of partition Array (Number of Partitions)
* Output:
* Return:
* Attention:n must be the same as for the corresponding call of AllocPartition
*************************************************************************
*/

void FreeBitstream()
{
    assert(currStream != NULL);
    assert(currStream->streamBuffer != NULL);

    free(currStream->streamBuffer);
    free(currStream);
}

/*
*************************************************************************
* Function:Dynamic memory allocation of frame size related global buffers
buffers are defined in global.h, allocated memory must be freed in
void free_global_buffers()
* Input:Input Parameters struct inp_par *input, Image Parameters ImageParameters *img
* Output:Number of allocated bytes
* Return:
* Attention:
*************************************************************************
*/


int init_global_buffers()
{
    int i, j;
    int refnum;

    int memory_size = 0;
    int img_height = (hd->vertical_size + img->auto_crop_bottom);

    img->buf_cycle = input->buf_cycle + 1;

    img->buf_cycle *= 2;

    if (hd->background_picture_enable) {
#if B_BACKGROUND_Fix
		if ((hd->p_ref_background = fopen(input->backgroundref_File, "rb")) == 0) {
			snprintf(hc->errortext, ET_SIZE, "Error open file backgroundref_File  %s", input->backgroundref_File);
			hd->BgRefPicExist = 0;
		}
#else
        if ((hd->p_ref_background = fopen("background_ref.yuv", "rb")) == 0) {
            snprintf(hc->errortext, ET_SIZE, "Error open file background_ref.yuv ");
            hd->BgRefPicExist = 0;
        }
#endif	
		else {
            hd->BgRefPicExist = 1;
        }
        if (input->output_dec_pic && (hd->p_out_background = fopen("background_dec.yuv", "wb")) == 0) {
            snprintf(hc->errortext, ET_SIZE, "Error open file background_out.yuv ");
            error(hc->errortext, 500);
        }
    }

    if (hd->background_picture_enable) {
        for (i = 0; i < 3; i++) {
            if (i == 0) {
                get_mem2D(&hd->background_frame[i], img->height, img->width);
            } else {
                get_mem2D(&hd->background_frame[i], img->height_cr, img->width_cr);
            }
        }
    }


    // allocate memory for imgYPrev
    memory_size += get_mem2D(&hc->imgYPrev, img_height, img->width);
    memory_size += get_mem3D(&hc->imgUVPrev, 2, img_height / (input->chroma_format == 1 ? 2 : 1), img->width_cr);

    // allocate memory for reference frames of each block: refFrArr
    memory_size += get_mem2Dint(&hc->refFrArr, img_height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);
    memory_size += get_mem2Dint(&hc->p_snd_refFrArr, img->height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);

    // allocate memory for reference frame in find_snr
    memory_size += get_mem2D(&hd->imgYRef, img_height, img->width);
    memory_size += get_mem3D(&hd->imgUVRef, 2, img_height / (input->chroma_format == 1 ? 2 : 1), img->width_cr);

    // allocate memory in structure img
    if (((img->mb_data) = (codingUnit *) calloc((img->width / MIN_CU_SIZE) * (img_height / MIN_CU_SIZE),
                          sizeof(codingUnit))) == NULL) {
        no_mem_exit("init_global_buffers: img->mb_data");
    }

    if (((img->intra_block) = (int **) calloc((j = (img->width / MIN_CU_SIZE) * (img_height / MIN_CU_SIZE)),
                              sizeof(int *))) == NULL) {
        no_mem_exit("init_global_buffers: img->intra_block");
    }

    for (i = 0; i < j; i++) {
        if ((img->intra_block[i] = (int *) calloc(4, sizeof(int))) == NULL) {
            no_mem_exit("init_global_buffers: img->intra_block");
        }
    }

// Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    Init_QMatrix();
#endif

    memory_size += get_mem3Dint(& (img->tmp_mv), img_height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE , 4);

    memory_size += get_mem3Dint(& (img->p_snd_tmp_mv), img_height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE , 4);

    memory_size += get_mem2Dint(& (img->ipredmode), img_height / MIN_BLOCK_SIZE + 2, img->width / MIN_BLOCK_SIZE + 2);

    get_mem2Dint(& (img->predBlock), (1 << input->g_uiMaxSizeInBit), (1 << input->g_uiMaxSizeInBit));
    get_mem2Dint(&(img->predBlockTmp), (1 << input->g_uiMaxSizeInBit), (1 << input->g_uiMaxSizeInBit));
    get_mem2Dint(&img->resiY, (1 << (input->g_uiMaxSizeInBit + 1)), (1 << (input->g_uiMaxSizeInBit + 1)));
    if ((img->quad = (int *) calloc((long long int)(1 << input->sample_bit_depth), sizeof(int))) == NULL) {
        no_mem_exit("init_img: img->quad");
    }
    init();

    memory_size += get_mem2Dint(& (img->fw_refFrArr), img_height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);
    memory_size += get_mem2Dint(& (img->bw_refFrArr), img_height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE);

    memory_size += get_mem3Dint(& (img->fw_mv), img_height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE, 3);
    memory_size += get_mem3Dint(& (img->bw_mv), img_height / MIN_BLOCK_SIZE, img->width / MIN_BLOCK_SIZE, 3);

    // Prediction mode is set to -1 outside the frame, indicating that no prediction can be made from this part
    for (i = 0; i < img->width / (MIN_BLOCK_SIZE) + 2; i++) {
        for (j = 0; j < img_height / (MIN_BLOCK_SIZE) + 2; j++) {
            img->ipredmode[j][i] = -1;
        }
    }

    //by oliver 0512

    img->buf_cycle = input->buf_cycle + 1;

    // allocate frame buffer
    for (i = 0; i < 3; i++) {
        if (i == 0) {
            get_mem2D(&hc->backgroundReferenceFrame[i], img_height, img->width);
        } else {
            get_mem2D(&hc->backgroundReferenceFrame[i], img_height / (input->chroma_format == 1 ? 2 : 1), img->width_cr);
        }
    }
    hc->background_ref = hc->backgroundReferenceFrame;


    for (refnum = 0; refnum < REF_MAXBUFFER; refnum++) {
        fref[refnum] = (avs2_frame_t *)malloc(sizeof(avs2_frame_t));  ////fref[i] memory allocation
        fref[refnum]->imgcoi_ref = -257;
        fref[refnum]->is_output = -1;
        fref[refnum]->refered_by_others = -1;
        fref[refnum]->imgtr_fwRefDistance = -256;
        init_frame_t(fref[refnum]);
    }


    // get_mem3DSAOstatdate( &(img->saostatData), (( img->width >> input->g_uiMaxSizeInBit) + (img->width % ( 1<< input->g_uiMaxSizeInBit) ? 1 : 0)) * (( img->height >> input->g_uiMaxSizeInBit) + (img->height % ( 1<< input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS, NUM_SAO_NEW_TYPES);

    get_mem2DSAOParameter(&(img->saoBlkParams),
                          ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                      img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS);

    get_mem2DSAOParameter(&img->rec_saoBlkParams,
                          ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                      img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS);


    if ((img->cur_saorate = (double *) calloc(NUM_SAO_COMPONENTS, sizeof(double))) == NULL) {
        no_mem_exit("init_img: img->cur_saorate");
    }
    get_mem2D(&hc->imgY_sao, img->height, img->width);
    get_mem2D(&(hc->imgUV_sao[0]), img->height_cr, img->width_cr);
    get_mem2D(&(hc->imgUV_sao[1]), img->height_cr, img->width_cr);
    return (memory_size);
}

/*
*************************************************************************
* Function:Free allocated memory of frame size related global buffers
buffers are defined in global.h, allocated memory is allocated in
int init_global_buffers()
* Input:Input Parameters struct inp_par *input, Image Parameters ImageParameters *img
* Output:
* Return:
* Attention:
*************************************************************************
*/

void free_global_buffers()
{
    int  i, j;
    int img_height = (hd->vertical_size + img->auto_crop_bottom);

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    free_QMatrix();
#endif

    if (hd->background_picture_enable) {
        for (i = 0; i < 3; i++) {
            free_mem2D(hd->background_frame[i]);
        }
    }

    free_mem2D(hc->imgYPrev);
    free_mem3D(hc->imgUVPrev, 2);

    free_mem2Dint(hc->refFrArr);  //1105
    free_mem2Dint(hc->p_snd_refFrArr);



    free_mem2D(hd->imgYRef);
    free_mem3D(hd->imgUVRef, 2);
    // free mem, allocated for structure img
    if (img->mb_data       != NULL) {
        free(img->mb_data);
    }

    j = (img->width / MIN_CU_SIZE) * (img->height / MIN_CU_SIZE);

    for (i = 0; i < j; i++) {
        free(img->intra_block[i]);
    }

    free(img->intra_block);

    free_mem3Dint(img->tmp_mv, img_height / MIN_BLOCK_SIZE);
    free_mem3Dint(img->p_snd_tmp_mv, img_height / MIN_BLOCK_SIZE);

    free_mem2Dint(img->ipredmode);
    free_mem2Dint(img->resiY);
    free_mem2Dint(img->predBlock);
    free_mem2Dint(img->predBlockTmp);
    free(img->quad);

    free_mem2Dint(img->fw_refFrArr);
    free_mem2Dint(img->bw_refFrArr);

    free_mem3Dint(img->fw_mv, img_height / MIN_BLOCK_SIZE);
    free_mem3Dint(img->bw_mv, img_height / MIN_BLOCK_SIZE);
    free_mem2D(hc->imgY_sao);
    free_mem2D(hc->imgUV_sao[0]);
    free_mem2D(hc->imgUV_sao[1]);

    // free_mem3DSAOstatdate(img->saostatData,(( img->width >> input->g_uiMaxSizeInBit) + (img->width % ( 1<< input->g_uiMaxSizeInBit) ? 1 : 0)) * (( img->height >> input->g_uiMaxSizeInBit) + (img->height % ( 1<< input->g_uiMaxSizeInBit) ? 1 : 0)), NUM_SAO_COMPONENTS);

    free_mem2DSAOParameter(img->saoBlkParams,
                           ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                       img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)));

    free_mem2DSAOParameter(img->rec_saoBlkParams,
                           ((img->width >> input->g_uiMaxSizeInBit) + (img->width % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)) * ((
                                       img->height >> input->g_uiMaxSizeInBit) + (img->height % (1 << input->g_uiMaxSizeInBit) ? 1 : 0)));


    free(img->cur_saorate);
    for (i = 0; i < REF_MAXBUFFER; i++) {
        free_frame_t(fref[i]);
    }

    for (j = 0; j < 3; j++) {
        free_mem2D(hc->backgroundReferenceFrame[j]);
    }

}



/*!
************************************************************************
* \brief
*    Allocates a stand-alone partition structure.  Structure should
*    be freed by FreePartition();
*    data structures
*
* \par Input:
*    n: number of partitions in the array
* \par return
*    pointer to DataPartition Structure, zero-initialized
*  added by lzhang
************************************************************************
*/

DataPartition *AllocPartition(int n)
{
    DataPartition *partArr;

    partArr = (DataPartition *) calloc(n, sizeof(DataPartition));

    if (partArr == NULL) {
        snprintf(hc->errortext, ET_SIZE, "AllocPartition: Memory allocation for Data Partition failed");
        error(hc->errortext, 100);
    }

//   for ( i = 0; i < n; i++ ) // loop over all data partitions
//   {
//     dataPart = & ( partArr[i] );
//
//     dataPart->bitstream = ( Bitstream * ) calloc ( 1, sizeof ( Bitstream ) );
//
//     if ( dataPart->bitstream == NULL )
//     {
//       snprintf ( errortext, ET_SIZE, "AllocPartition: Memory allocation for Bitstream failed" );
//       error ( errortext, 100 );
//     }
//   }       //1105

    return partArr;
}

/*!
************************************************************************
* \brief
*    Allocates the slice structure along with its dependent
*    data structures
*
* \par Input:
*    Input Parameters struct inp_par *input,  ImageParameters *img
* \author
* lzhang
************************************************************************
*/
void malloc_slice()
{
    Slice *currSlice;

    img->currentSlice = (Slice *) calloc(1, sizeof(Slice));

    if ((currSlice = img->currentSlice) == NULL) {
        snprintf(hc->errortext, ET_SIZE, "Memory allocation for Slice datastruct in NAL-mode %d failed", input->FileFormat);
        error(hc->errortext, 100);
    }

    // create all context models
    currSlice->syn_ctx = create_contexts_SyntaxInfo();

    currSlice->max_part_nr = 1;//1105
    currSlice->partArr = AllocPartition(currSlice->max_part_nr);
}
/*!
************************************************************************
* \brief
*    Memory frees of the Slice structure and of its dependent
*    data structures
*
* \par Input:
*    Input Parameters struct inp_par *input,  ImageParameters *img
* \author
* lzhang
************************************************************************
*/
void free_slice()
{
    Slice *currSlice = img->currentSlice;

    FreePartition(currSlice->partArr, 1);

    // delete all context models
    delete_contexts_SyntaxInfo(currSlice->syn_ctx);

    free(img->currentSlice);
    img->currentSlice = NULL;

    currSlice = NULL;
}
/*!
************************************************************************
* \brief
*    Frees a partition structure (array).
*
* \par Input:
*    Partition to be freed, size of partition Array (Number of Partitions)
*
* \par return
*    None
*
* \note
*    n must be the same as for the corresponding call of AllocPartition
* \author : lzhang
************************************************************************
*/
void FreePartition(DataPartition *dp, int n)
{
    assert(dp != NULL);
    //assert ( dp->bitstream != NULL );               //1105
    //assert ( dp->bitstream->streamBuffer != NULL );

    free(dp);
}
