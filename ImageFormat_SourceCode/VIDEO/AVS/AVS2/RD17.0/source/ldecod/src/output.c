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
* File name:  output.c
* Function: Output an image and Trance support
*
*************************************************************************************
*/


#include "../../lcommon/inc/contributors.h"

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include <stdlib.h>
#include <string.h>

static void reduce_yuv_dim(byte *imgYuv_buffer, byte **imgY, byte ***imgUV,
                           int img_width, int img_height,
                           int img_width_cr, int img_height_cr)
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

static void write_yuv_shift(FILE *p_out, byte *img, unsigned char *buf, int length, int shift1)
{
    int j;

    for (j = 0; j < length; j++) {
        buf[j] = (unsigned char)Clip1((img[j] + (1 << (shift1 - 1))) >> shift1);
    }

    fwrite(buf, sizeof(unsigned char), length, p_out);
}

static void write_yuv_byte(FILE *p_out, byte *img, unsigned char *buf, int length)
{
    fwrite(img, sizeof(byte), length, p_out);
}

static void write_yuv_uchar(FILE *p_out, byte *img, unsigned char *buf, int length)
{
    int j;

    for (j = 0; j < length; j++) {
        buf[j] = (unsigned char)img[j];
    }

    fwrite(buf, sizeof(unsigned char), length, p_out);
}

static void write_yuv_frame(FILE *p_out, byte *img, int img_length)
{
    int nSampleSize;
    int shift1;
    unsigned char *buf;

    if (input->output_bit_depth == 8) {
        nSampleSize = 1;
    } else if (input->output_bit_depth > 8) {
        nSampleSize = 2;
    }
    shift1 = input->sample_bit_depth - input->output_bit_depth;
    buf = malloc(img_length * nSampleSize);

    if (!shift1 && input->output_bit_depth == 8) { // 8bit input -> 8bit encode
        write_yuv_uchar(p_out, img, buf, img_length);
    } else if (!shift1 && input->output_bit_depth > 8) { // 10/12bit input -> 10/12bit encode
        write_yuv_byte(p_out, img, buf, img_length);
    } else if (shift1 && input->output_bit_depth == 8) { // 8bit input -> 10/12bit encode
        write_yuv_shift(p_out, img, buf, img_length, shift1);
    }
    free(buf);
    fflush(p_out);
}

/*
*************************************************************************
* Function:Write decoded GB frame to output file
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void write_GB_frame(FILE *p_out)          //!< filestream to output file
{
    int img_width = (img->width - img->auto_crop_right);
    int img_height = (img->height - img->auto_crop_bottom);
    int img_width_cr = (img_width / 2);
    int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));
    byte **imgY_out;
    byte **imgUV_out[2];
    int img_length = img_height * img_width + img_width_cr * img_height_cr * 2;
    byte *imgYuv_buffer = malloc(img_length * sizeof(byte));

    imgY_out     = hd->background_frame[0];
    imgUV_out[0] = hd->background_frame[1];
    imgUV_out[1] = hd->background_frame[2];

    reduce_yuv_dim(imgYuv_buffer, imgY_out, imgUV_out, img_width, img_height, img_width_cr, img_height_cr);
    write_yuv_frame(p_out, imgYuv_buffer, img_length);
    free(imgYuv_buffer);
}
/*
*************************************************************************
* Function:Write decoded frame to output file
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
void write_frame(FILE *p_out , int pos)
{
    int j;
    int img_width = (img->width - img->auto_crop_right);
    int img_height = (img->height - img->auto_crop_bottom);
    int img_width_cr = (img_width / 2);
    int img_height_cr = (img_height / (input->chroma_format == 1 ? 2 : 1));
    int img_length = img_height * img_width + img_width_cr * img_height_cr * 2;
    byte *imgYuv_buffer = malloc(img_length * sizeof(byte));
    byte **imgY_out;
    byte ***imgUV_out;

    for (j = 0; j < REF_MAXBUFFER; j++) {
        if (fref[j]->imgtr_fwRefDistance == pos) {
            imgY_out = fref[j]->ref[0];
            imgUV_out = &fref[j]->ref[1];

            fref[j]->is_output = -1;
            if (fref[j]->refered_by_others == 0 || fref[j]->imgcoi_ref == -257) {
                fref[j]->imgtr_fwRefDistance = -256;
                fref[j]->imgcoi_ref = -257;
#if M3480_TEMPORAL_SCALABLE
                fref[j]->temporal_id = -1;
#endif
            }
            break;
        }
    }

    reduce_yuv_dim(imgYuv_buffer, imgY_out, imgUV_out, img_width, img_height, img_width_cr, img_height_cr);
    write_yuv_frame(p_out, imgYuv_buffer, img_length);
    free(imgYuv_buffer);
}

#if TRACE
static int bitcounter = 0;

/*
*************************************************************************
* Function:Tracing bitpatterns for symbols
A code word has the following format: 0 Xn...0 X2 0 X1 0 X0 1
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void tracebits(
    const char *trace_str,  //!< tracing information, char array describing the symbol
    int len,                //!< length of syntax element in bits
    int info,               //!< infoword of syntax element
    int value1)
{

    int i, chars;

    if (len >= 34) {
        snprintf(hc->errortext, ET_SIZE, "Length argument to put too long for trace to work");
        error(hc->errortext, 600);
    }

    putc('@', hc->p_trace);
    chars = fprintf(hc->p_trace, "%i", bitcounter);

    while (chars++ < 6) {
        putc(' ', hc->p_trace);
    }

    chars += fprintf(hc->p_trace, "%s", trace_str);

    while (chars++ < 55) {
        putc(' ', hc->p_trace);
    }

    // Align bitpattern
    if (len < 15) {
        for (i = 0 ; i < 15 - len ; i++) {
            fputc(' ', hc->p_trace);
        }
    }

    // Print bitpattern
    for (i = 0 ; i < len / 2 ; i++) {
        fputc('0', hc->p_trace);
    }

    // put 1
    fprintf(hc->p_trace, "1");

    // Print bitpattern
    for (i = 0 ; i < len / 2 ; i++) {
        if (0x01 & (info >> ((len / 2 - i) - 1))) {
            fputc('1', hc->p_trace);
        } else {
            fputc('0', hc->p_trace);
        }
    }

    fprintf(hc->p_trace, "  (%3d)\n", value1);
    bitcounter += len;

    fflush(hc->p_trace);

}

/*
*************************************************************************
* Function:Tracing bitpatterns
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/


void tracebits2(
    const char *trace_str,  //!< tracing information, char array describing the symbol
    int len,                //!< length of syntax element in bits
    int info)
{

    int i, chars;

    if (len >= 45) {
        snprintf(hc->errortext, ET_SIZE, "Length argument to put too long for trace to work");
        error(hc->errortext, 600);
    }

    putc('@', hc->p_trace);
    chars = fprintf(hc->p_trace, "%i", bitcounter);

    while (chars++ < 6) {
        putc(' ', hc->p_trace);
    }

    chars += fprintf(hc->p_trace, "%s", trace_str);

    while (chars++ < 55) {
        putc(' ', hc->p_trace);
    }

    // Align bitpattern
    if (len < 15)
        for (i = 0 ; i < 15 - len ; i++) {
            fputc(' ', hc->p_trace);
        }

    bitcounter += len;

    while (len >= 32) {
        for (i = 0 ; i < 8 ; i++) {
            fputc('0', hc->p_trace);
        }

        len -= 8;

    }

    // Print bitpattern
    for (i = 0 ; i < len ; i++) {
        if (0x01 & (info >> (len - i - 1))) {
            fputc('1', hc->p_trace);
        } else {
            fputc('0', hc->p_trace);
        }
    }

    fprintf(hc->p_trace, "  (%3d)\n", info);

    fflush(hc->p_trace);

}

#endif

