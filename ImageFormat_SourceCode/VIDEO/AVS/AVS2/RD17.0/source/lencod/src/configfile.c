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
#define INCLUDED_BY_CONFIGFILE_C

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <memory.h>

#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "configfile.h"
#if TDRDO
#include "tdrdo.h"
#endif

#define MAX_ITEMS_TO_PARSE  10000
#define _S_IREAD      0000400         /* read permission, owner */
#define _S_IWRITE     0000200         /* write permission, owner */

static char *GetConfigFileContent(char *Filename);
static void ParseContent(char *buf, int bufsize);
static int ParameterNameToMapIndex(char *s);
static void PatchInp();

static int ParseRefContent(char **buf);



Mapping Map[] = {
    {"MaxSizeInBit",             &configinput.g_uiMaxSizeInBit,        0},
    {"ProfileID",                &configinput.profile_id,              0},
    {"LevelID",                  &configinput.level_id,                0},
    {"IntraPeriod",              &configinput.intra_period,            0},
    {"VECPeriod",                &configinput.vec_period,              0},
    {"SeqHeaderPeriod",          &configinput.seqheader_period,        0},
    {"FramesToBeEncoded",        &configinput.no_frames,               0},
    {"QPIFrame",                 &configinput.qpI,                     0},
    {"QPPFrame",                 &configinput.qpP,                     0},
    {"FrameSkip",                &configinput.jumpd_all,               0},
    {"UseHadamard",              &configinput.hadamard,                0},
    {"FME",                      &configinput.usefme,                  0},
    {"SearchRange",              &configinput.search_range,            0},
    {"NumberReferenceFrames",    &configinput.no_multpred,             0},
    {"SourceWidth",              &configinput.img_width,               0},
    {"SourceHeight",             &configinput.img_height,              0},
    {"TDMode",                   &configinput.TD_mode,                 0},
    {"ViewPackingMode",          &configinput.view_packing_mode,       0},
    {"ViewReverse",              &configinput.view_reverse,            0},

    { "InputFile",                &configinput.infile,                 1},
    { "ReconFile",                &configinput.ReconFile,              1},
#if ROI_M3264
    {"InputROIDataFile",         &configinput.infile_data,             1},
    {"ROICoding",                &configinput.ROI_Coding,              0},
#endif
    {"FrameSkipNums",            &configinput.infile_frameskip,        0},

    {"OutputFile",               &configinput.outfile,                 1},
    {"TraceFile",                &configinput.TraceFile,               1},
#if M3480_TEMPORAL_SCALABLE
    {"TemporalScalableFlag",     &configinput.TemporalScalableFlag,    0},
#endif
    {"FFRAMEEnable",             &configinput.fframe_enabled,          0},
    {"DHPEnable",                &configinput.dhp_enabled,             0},
    {"MHPSKIPEnable",            &configinput.b_mhpskip_enabled,       0},
    {"DMHEnableEncoder",         &configinput.dmh_enabled_encoder,     0},

    {"WSMEnable",                &configinput.wsm_enabled,             0},
    {"NumberBFrames",            &configinput.successive_Bframe_all,   0},
    {"HierarchicalCoding",       &configinput.Hierarchical_B,          0},
    {"QPBFrame",                 &configinput.qpB,                     0},
    {"InterSearch16x16",         &configinput.InterSearch16x16,        0},
    {"InterSearch16x8",          &configinput.InterSearch16x8 ,        0},
    {"InterSearch8x16",          &configinput.InterSearch8x16,         0},
    {"InterSearch8x8",           &configinput.InterSearch8x8 ,         0},
    {"InterSearchAMP",           &configinput.InterSearchAMP ,         0},
#if PicExtensionData
    {"PicExtensionData",         &configinput.PicExtentData ,          0},
#endif
    {"RDOptimization",           &configinput.rdopt,                   0},
    {"InterlaceCodingOption",    &configinput.InterlaceCodingOption,   0},
    {"repeat_first_field",       &configinput.repeat_first_field,      0},
    {"top_field_first",          &configinput.top_field_first,         0},
#if INTERLACE_CODING
    {"OutputMergedPicture",      &configinput.output_merged_picture,   0},
#endif
#if AVS2_HDR_HLS
    {"HDRMetaDataExtension",     &configinput.hdr_metadata,            0},
#endif
    {"LoopFilterDisable",        &configinput.loop_filter_disable,     0},
    {"LoopFilterParameter",      &configinput.loop_filter_parameter_flag,        0},
    {"LoopFilterAlphaOffset",    &configinput.alpha_c_offset,          0},
    {"LoopFilterBetaOffset",     &configinput.beta_offset,             0},
    {"SAOEnable",                &configinput.sao_enable,              0},
    {"ALFEnable",                &configinput.alf_enable,              0},
    {"ALF_LowLatencyEncodingEnable", &configinput.alf_LowLatencyEncodingEnable,  0},

    {"CrossSliceLoopFilter",     &configinput.crossSliceLoopFilter,    0},

    {"Progressive_sequence",     &configinput.progressive_sequence,    0},
    {"Progressive_frame",        &configinput.progressive_frame,       0},
    {"NumberOfLCUsInSlice",      &configinput.slice_row_nr,            0},
    {"SliceParameter",           &configinput.slice_parameter,         0},
    {"FrameRate",                &configinput.frame_rate_code,         0},
    {"ChromaFormat",             &configinput.chroma_format,           0},
    {"YUVStructure",             &configinput.yuv_structure,           0},

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    {"WQEnable",                 &configinput.WQEnable,               0},
    {"SeqWQM",                   &configinput.SeqWQM,                 0},
    {"SeqWQFile",                &configinput.SeqWQFile,              1},

    {"PicWQEnable",              &configinput.PicWQEnable,            0},
    {"WQParam",                  &configinput.WQParam,                0},
    {"WQModel",                  &configinput.WQModel,                0},
    {"WeightParamDetailed",      &configinput.WeightParamDetailed,    1},
    {"WeightParamUnDetailed",    &configinput.WeightParamUnDetailed,  1},
    //{"MBAdaptQuant",            &configinput.MBAdaptQuant,  0},
    //#if CHROMA_DELTA_QP
    {"ChromaDeltaQPDisable",     &configinput.chroma_quant_param_disable, 0},
    {"ChromaDeltaU",             &configinput.chroma_quant_param_delta_u, 0},
    {"ChromaDeltaV",             &configinput.chroma_quant_param_delta_v, 0},
    //#endif
    {"PicWQDataIndex",           &configinput.PicWQDataIndex,          0},
    {"PicWQFile",                &configinput.PicWQFile,               1},
#endif

    {"RDOQEnable",               &configinput.use_rdoq,                0},
    {"LambdaFactor",             &configinput.lambda_factor_rdoq,      0},
    {"LambdaFactorP",            &configinput.lambda_factor_rdoq_p,    0},
    {"LambdaFactorB",            &configinput.lambda_factor_rdoq_b,    0},

    {"PMVREnable",               &configinput.b_pmvr_enabled,          0},
    {"NSQT",                     &configinput.useNSQT,                 0},
#if MB_DQP
    {"DeltaQP",                  &configinput.useDQP,                  0},
#endif
    {"SDIP",                     &configinput.useSDIP,                 0},
#if REFINED_QP
    {"RefineQP",                 &configinput.use_refineQP,            0},
#endif
    {"OutPutEncPic",             &configinput.output_enc_pic,          0},
    {"BackgroundEnable",         &configinput.bg_enable,               0},
    {"BGFileName",               &configinput.bg_file_name,            1},
#if B_BACKGROUND_Fix
	{ "Backgroundref_File",       &configinput.Backgroundref_File,     1},
#endif
    {"BGInputNumber",            &configinput.bg_input_number,         0},
    {"BackgroundPeriod",         &configinput.bg_period,               0},
    {"ModelNumber",              &configinput.bg_model_number,         0},
    {"BackgroundQP",             &configinput.bg_qp,                   0},
    {"ModelMethod",              &configinput.bg_model_method,         0},
#if AVS2_S2_FASTMODEDECISION
    {"BGFastMode",               &configinput.bg_fast_mode,            0},
#endif

//#if HDR_CHROMA_DELTA_QP
	{"ChromaHDRDeltaQPDisable",  &configinput.chroma_hdr_chroma_delta_disable, 0},
	{"ChromaQpScale",            &configinput.chromaQpScale, 0},
	{"ChromaQpOffset",           &configinput.chromaQpOffset, 0},
	{"CbQpScale",                &configinput.chromaCbQpScale, 0},
	{"CrQpScale",                &configinput.chromaCrQpScale, 0},
//#endif

    {"SECTEnable",               &configinput.b_secT_enabled,          0},
    {"SampleBitDepth",           &configinput.sample_bit_depth,        0},
    {"InputSampleBitDepth",      &configinput.input_sample_bit_depth,  0},
#ifdef TDRDO
    {"TDRDOEnable",              &configinput.TDEnable,                0},
#endif
#ifdef AQPO
    {"AQPOEnable",               &configinput.AQPEnable,               0},
#endif
#ifdef RATECONTROL
    {"RateControl",              &configinput.EncControl,              0},
    {"TargetBitRate",            &configinput.TargetBitRate,           0},
    {"RCInitialQP",              &configinput.ECInitialQP,             0},
#endif

    {"MD5Enable",                &configinput.MD5Enable,               0},

    {NULL,                       NULL,                                 -1}
};

/*
*************************************************************************
* Function:Parse the command line parameters and read the config files.
* Input: ac
number of command line parameters
av
command line parameters
* Output:
* Return:
* Attention:
*************************************************************************
*/

void Configure(int ac, char *av[])
{
    char *content;
    int CLcount, ContentLen, NumberParams;

    memset(&configinput, 0, sizeof(InputParameters));

    // Process default config file
    // Parse the command line

    CLcount = 1;

    he->gop_size = -1;

    while (CLcount < ac) {
        if (0 == strncmp(av[CLcount], "-f", 2)) {      // A file parameter?
            content = GetConfigFileContent(av[CLcount + 1]);
            printf("Parsing Configfile %s", av[CLcount + 1]);
            ParseContent(content, (int)strlen(content));
            printf("\n");
            free(content);
            CLcount += 2;
        } else {
            if (0 == strncmp(av[CLcount], "-p", 2)) {      // A config change?
                // Collect all data until next parameter (starting with -<x> (x is any character)),
                // put it into content, and parse content.
                CLcount++;
                ContentLen = 0;
                NumberParams = CLcount;

                // determine the necessary size for content
                while (NumberParams < ac && av[NumberParams][0] != '-') {
                    ContentLen += (int)strlen(av[NumberParams++]);     // Space for all the strings
                }

                ContentLen += 1000;                     // Additional 1000 bytes for spaces and \0s

                if ((content = malloc(ContentLen)) == NULL) {
                    no_mem_exit("Configure: content");
                }

                content[0] = '\0';

                // concatenate all parameters itendified before
                while (CLcount < NumberParams) {
                    char *source = &av[CLcount][0];
                    char *destin = &content[strlen(content) ];

                    while (*source != '\0') {
                        if (*source == '=') {   // The Parser expects whitespace before and after '='
                            *destin++ = ' ';
                            *destin++ = '=';
                            *destin++ = ' '; // Hence make sure we add it
                        } else {
                            *destin++ = *source;
                        }

                        source++;
                    }

                    *destin++ = ' ';      // add a space to support multiple config items
                    *destin = '\0';
                    CLcount++;
                }

                printf("Parsing command line string '%s'", content);
                ParseContent(content, (int)strlen(content));
                free(content);
                printf("\n");
            } else {
                snprintf(hc->errortext, ET_SIZE, "Error in command line, ac %d, around string '%s', missing -f or -p parameters?",
                         CLcount, av[CLcount]);
                error(hc->errortext, 300);
            }
        }
    }


    input->b_dmh_enabled = 1;


    PatchInp();  //rm52k

    input->successive_Bframe = input->successive_Bframe_all;
    printf("\n");
}

/*
*************************************************************************
* Function: Alocate memory buf, opens file Filename in f, reads contents into
buf and returns buf
* Input:name of config file
* Output:
* Return:
* Attention:
*************************************************************************
*/
char *GetConfigFileContent(char *Filename)
{
    unsigned FileSize;
    FILE *f;
    char *buf;

    if (NULL == (f = fopen(Filename, "r"))) {
        snprintf(hc->errortext, ET_SIZE, "Cannot open configuration file %s.\n", Filename);
        error(hc->errortext, 300);
    }

    if (0 != fseek(f, 0, SEEK_END)) {
        snprintf(hc->errortext, ET_SIZE, "Cannot fseek in configuration file %s.\n", Filename);
        error(hc->errortext, 300);
    }

    FileSize = ftell(f);

    if (FileSize < 0 || FileSize > 60000) {
        snprintf(hc->errortext, ET_SIZE, "Unreasonable Filesize %d reported by ftell for configuration file %s.\n", FileSize,
                 Filename);
        error(hc->errortext, 300);
    }

    if (0 != fseek(f, 0, SEEK_SET)) {
        snprintf(hc->errortext, ET_SIZE, "Cannot fseek in configuration file %s.\n", Filename);
        error(hc->errortext, 300);
    }

    if ((buf = malloc(FileSize + 1)) == NULL) {
        no_mem_exit("GetConfigFileContent: buf");
    }

    // Note that ftell() gives us the file size as the file system sees it.  The actual file size,
    // as reported by fread() below will be often smaller due to CR/LF to CR conversion and/or
    // control characters after the dos EOF marker in the file.

    FileSize = (unsigned int)fread(buf, 1, FileSize, f);
    buf[FileSize] = '\0';

    fclose(f);

    return buf;
}

void print_ref_man(int v)
{
    int i, j;
    ref_man *p;
    if (v) {
        printf("\n DOI POI type qpof rfby nref reflist      nrem remlist");
    } else {
        printf("\n POI DOI type qpof rfby nref reflist      nrem remlist");
    }

    for (j = 0; j < he->gop_size_all; j++) {
        p = he->cfg_ref_all + j;
        printf("\n %2d %3d %4d %4d %4d", j + 1, p->poc, p->type, p->qp_offset, p->referd_by_others);
        //if(p->predict > 0)
        //  printf("%2d",p->deltaRPS);
        //else
        printf("  ");
        printf(" %4d ", p->num_of_ref);
        for (i = 0; i < p->num_of_ref; i++) {
            printf(" %2d", p->ref_pic[i]);
        }
        for (i = 0; i < MAXREF - p->num_of_ref; i++) {
            printf("   ");
        }
        printf(" %4d ", p->num_to_remove);
        for (i = 0; i < p->num_to_remove; i++) {
            printf(" %2d", p->remove_pic[i]);
        }
        for (i = 0; i < MAXREF - p->num_to_remove; i++) {
            printf("   ");
        }
        if (configinput.TemporalScalableFlag == 1) {
            printf(" %2d", p->temporal_id);
        }
    }
    printf("\n");
}
void TranslateRPS()
{
    ref_man tmp_cfg_ref[MAXGOP];
    ref_man *p;
    int i, j;
    int poi, doi;
    for (i = 0; i < he->gop_size_all; i++) {
        tmp_cfg_ref[i] = he->cfg_ref_all[i];
    }
    //按照DOI的顺序重新排列RPS
    for (i = 0; i < he->gop_size_all; i++) {
        j = tmp_cfg_ref[i].poc - 1;//重排前.poc里放的实际上是DOI（从1开始），而POI则是重排前的顺序索引，即i
        he->cfg_ref_all[j] = tmp_cfg_ref[i];
        he->cfg_ref_all[j].poc = i + 1; //poi也是从1开始
    }
    //下面把用POI表示的参考帧和删除帧翻译成用deltaDOI表示
    for (i = 0; i < he->gop_size_all; i++) {
        p = he->cfg_ref_all + i;//i+1是当前图像的doi
        for (j = 0; j < p->num_of_ref; j++) {
            int n;
            poi = p->ref_pic[j];//取第j个参考图像的poi，从1开始
            n = 0;
            while (poi < 1) { //把poi调整到1到gop_size_all之间
                poi += he->gop_size_all;
                n += he->gop_size_all;
            }
            doi = tmp_cfg_ref[poi - 1].poc;//doi从1开始
            p->ref_pic[j] = i + 1 - doi + n;
        }
        for (j = 0; j < p->num_to_remove; j++) {
            int n;
            poi = p->remove_pic[j];//取第j个参考图像的poi，从1开始
            n = 0;
            while (poi < 1) { //把poi调整到1到gop_size_all之间
                poi += he->gop_size_all;
                n += he->gop_size_all;
            }
            doi = tmp_cfg_ref[poi - 1].poc;//doi从1开始
            p->remove_pic[j] = i + 1 - doi + n;
        }
    }
#ifdef ZHAOHAIWU20150420
    if (he->gop_size_all == 24) { //GOP24
        if (he->cfg_ref_all[0].poc == 8)
            for (j = 8; j < 24; j++) { //GOP24结构是GOP8+GOP16
                he->cfg_ref_all[j].poc -= 8;
            }
        else if (he->cfg_ref_all[0].poc == 16)
            for (j = 16; j < 24; j++) { //GOP24结构为GOP16+GOP8
                he->cfg_ref_all[j].poc -= 16;
            }
    }
#else
    if (he->cfg_ref_all[0].poc < he->gop_size_all) { //there are sub gops
        int sub_gop_size, sub_gop_start, sub_gop_num = 1;
        sub_gop_size = he->cfg_ref_all[0].poc;
        sub_gop_start = sub_gop_size;
        while (sub_gop_start < he->gop_size_all) {
            for (j = sub_gop_start; j < he->gop_size_all; j++) {
                he->cfg_ref_all[j].poc -= sub_gop_size;//减去上一个subgop的图像数
            }
            sub_gop_size = he->cfg_ref_all[sub_gop_start].poc;//取下一个subgop的图像数
            sub_gop_start += sub_gop_size;//计算再下一个subgop的起点
            sub_gop_num++;//subgop计数，可以用来做合法性检查。
            //顺便多说一句，sub_gop_size可以用来检查与配置参数NumberBFrames的一致性
        }
    }
#endif
}
int ParseRefContent(char **buf)
{
    ref_man *tmp;
    int flag = 0;
    int i = 1;
    int j = 0;
    char *token;
    char **p = buf;
    char *tmp_str = ":";
    char str[3];
    char headstr[10] = {'F', 'r', 'a', 'm', 'e', '\0', '\0', '\0', '\0', '\0'};

// Fix by Sunil for RD5.0 test in Linux (2013.11.06)
    sprintf(str, "%d", i);
    strcat(headstr, str);
    strcat(headstr, tmp_str);

    memset(he->cfg_ref_all, -1, sizeof(struct reference_management)*MAXGOP);

    while (0 == strcmp(headstr, *p++)) {
        tmp = he->cfg_ref_all + i - 1;
        token = *p++;
        tmp->poc = atoi(token);
        token = *p++;
        tmp->qp_offset = atoi(token);
        token = *p++;
        tmp->num_of_ref = atoi(token);
        token = *p++;
        tmp->referd_by_others = atoi(token);
        for (j = 0; j < tmp->num_of_ref; j++) {
            token =  *p++;
            tmp->ref_pic[j] = atoi(token);
        }

        //check the reference configuration
        for (j = 0; j < tmp->num_of_ref - 1; j++) {
            if (tmp->ref_pic[j] < tmp->ref_pic[j + 1]) {
                printf("wrong reference configuration");
                exit(-1);
            }
        }

        token = *p++;
        /*tmp->predict = atoi(token);
        if (0 != tmp->predict)
        {
            token = *p++;
            tmp->deltaRPS = atoi(token);
        }
        token =  *p++;*/
        tmp->num_to_remove = atoi(token);
        for (j = 0; j < tmp->num_to_remove; j++) {
            token =  *p++;
            tmp->remove_pic[j] = atoi(token);
        }

#if M3480_TEMPORAL_SCALABLE
        if (configinput.TemporalScalableFlag == 1) {
            token =  *p++;
            tmp->temporal_id = atoi(token);
        }
#endif

        i++;
        headstr[5] = headstr[6] = headstr[7] = headstr[8] = headstr[9] = '\0';
// Fix by Sunil for RD5.0 test in Linux (2013.11.06)
        sprintf(str, "%d", i);
        strcat(headstr, str);
        strcat(headstr, tmp_str);
    }

    he->gop_size_all = i - 1;
#if TDRDO
    // This is a copy of QP offset from cfg files.
    for (j = 0; j < he->gop_size_all; j++) {
        QpOffset[j] = he->cfg_ref_all[j].qp_offset;
    }
    GroupSize = he->gop_size_all;
#endif
    //print_ref_man(0);
    TranslateRPS();
    //print_ref_man(1);
    return (int)(p - buf - 1);
}
/*
*************************************************************************
* Function: Parses the character array buf and writes global variable input, which is defined in
configfile.h.  This hack will continue to be necessary to facilitate the addition of
new parameters through the Map[] mechanism (Need compiler-generated addresses in map[]).
* Input:  buf
buffer to be parsed
bufsize
buffer size of buffer
* Output:
* Return:
* Attention:
*************************************************************************
*/

void ParseContent(char *buf, int bufsize)
{
    char *items[MAX_ITEMS_TO_PARSE];
    int MapIdx;
    int item     = 0;
    int InString = 0;
    int InItem   = 0;
    char *p      = buf;
    char *bufend = &buf[bufsize];
    int IntContent;
    int i;

    char headstr[10] = {'F', 'r', 'a', 'm', 'e', '1', ':', '\0'};

    // Stage one: Generate an argc/argv-type list in items[], without comments and whitespace.
    // This is context insensitive and could be done most easily with lex(1).

    while (p < bufend) {
        switch (*p) {
        case 13:
            p++;
            break;
        case '#':                 // Found comment
            *p = '\0';              // Replace '#' with '\0' in case of comment immediately following integer or string

            while (*p != '\n' && p < bufend) {   // Skip till EOL or EOF, whichever comes first
                p++;
            }

            InString = 0;
            InItem = 0;
            break;
        case '\n':
            InItem = 0;
            InString = 0;
            *p++ = '\0';
            break;
        case ' ':
        case '\t':              // Skip whitespace, leave state unchanged

            if (InString) {
                p++;
            } else {
                // Terminate non-strings once whitespace is found
                *p++ = '\0';
                InItem = 0;
            }

            break;
        case '"':               // Begin/End of String
            *p++ = '\0';

            if (!InString) {
                items[item++] = p;
                InItem = ~InItem;
            } else {
                InItem = 0;
            }

            InString = ~InString; // Toggle
            break;
        default:

            if (!InItem) {
                items[item++] = p;
                InItem = ~InItem;
            }

            p++;
        }
    }

    item--;

    for (i = 0; i < item; i += 3) {
        if (0 == strcmp(items[i], headstr)) {
            i += ParseRefContent(&items[i]);
        }

        if (0 > (MapIdx = ParameterNameToMapIndex(items[i]))) {
            snprintf(hc->errortext, ET_SIZE, " Parsing error in config file: Parameter Name '%s' not recognized.", items[i]);
            error(hc->errortext, 300);
        }

        if (strcmp("=", items[i + 1])) {
            snprintf(hc->errortext, ET_SIZE, " Parsing error in config file: '=' expected as the second token in each line.");
            error(hc->errortext, 300);
        }

        // Now interprete the Value, context sensitive...
        switch (Map[MapIdx].Type) {
        case 0:           // Numerical

            if (1 != sscanf(items[i + 2], "%d", &IntContent)) {
                snprintf(hc->errortext, ET_SIZE, " Parsing error: Expected numerical value for Parameter of %s, found '%s'.", items[i],
                         items[i + 2]);
                error(hc->errortext, 300);
            }

            * (int *)(Map[MapIdx].Place) = IntContent;
            printf(".");
            break;
        case 1:
            strcpy((char *) Map[MapIdx].Place, items [i + 2]);
            printf(".");
            break;
        default:
            assert("Unknown value type in the map definition of configfile.h");
        }
    }

    memcpy(input, &configinput, sizeof(InputParameters));
#if M3480_TEMPORAL_SCALABLE
    if (input->TemporalScalableFlag == 1) {
        he->temporal_id_exist_flag = 1;
    } else {
        he->temporal_id_exist_flag = 0;
    }
#endif
}

/*
*************************************************************************
* Function:Return the index number from Map[] for a given parameter name.
* Input:parameter name string
* Output:
* Return: the index number if the string is a valid parameter name,         \n
-1 for error
* Attention:
*************************************************************************
*/

static int ParameterNameToMapIndex(char *s)
{
    int i = 0;

    while (Map[i].TokenName != NULL) {
        if (0 == strcmp(Map[i].TokenName, s)) {
            return i;
        } else {
            i++;
        }
    }

    return -1;
};

/*
*************************************************************************
* Function:Checks the input parameters for consistency.
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

static void PatchInp()
{
    unsigned int ui_MaxSize = 8;

#if INTERLACE_CODING
    input->org_img_height = input->img_height;
    input->org_img_width = input->img_width;
    input->org_no_frames = input->no_frames;
    if (input->InterlaceCodingOption == 3) {
        img->is_field_sequence = 1;
        input->img_height = input->img_height / 2;
        input->no_frames = input->no_frames * 2;
        input->intra_period = input->intra_period * 2;
    } else {
        img->is_field_sequence = 0;
    }
#endif

    // consistency check of QPs
#if MB_DQP
    if (input->useDQP) {
        input->fixed_picture_qp = 0;/*lgp*/
    } else {
        input->fixed_picture_qp = 1;
    }
#else
    input->fixed_picture_qp = 1;
#endif

    if (input->profile_id == 0x12 && input->intra_period != 1) {
        snprintf(hc->errortext, ET_SIZE, "Baseline picture file only supports intra picture coding!");
        error(hc->errortext, 400);
    }

    if (input->profile_id == 0x20 && (input->sample_bit_depth > 8 || input->input_sample_bit_depth > 8)) {
        snprintf(hc->errortext, ET_SIZE, "Baseline file only supports 8-bit coding!");
        error(hc->errortext, 400);
    }

    if (input->qpI > MAX_QP + (input->sample_bit_depth - 8) * 8 || input->qpI < MIN_QP) {
        snprintf(hc->errortext, ET_SIZE, "Error input parameter quant_I,check configuration file");
        error(hc->errortext, 400);
    }

    if (input->qpP > MAX_QP + (input->sample_bit_depth - 8) * 8 || input->qpP < MIN_QP) {
        snprintf(hc->errortext, ET_SIZE, "Error input parameter quant_P,check configuration file");
        error(hc->errortext, 400);
    }

    if (input->qpB > MAX_QP + (input->sample_bit_depth - 8) * 8 || input->qpB < MIN_QP) {
        snprintf(hc->errortext, ET_SIZE, "Error input parameter quant_B,check configuration file");
        error(hc->errortext, 400);
    }

    if (!input->useNSQT && input->useSDIP) {
        snprintf(hc->errortext, ET_SIZE, "Error SDIP shouldn't be tunned on when NSQT is off!");
        error(hc->errortext, 400);
    }


    if (input->g_uiMaxSizeInBit > 6 || input->g_uiMaxSizeInBit < 3) {
        snprintf(hc->errortext, ET_SIZE, "Error input parameter MaxSizeInBit,check configuration file");
        error(hc->errortext, 400);
    }

    // consistency check no_multpred
    if (input->no_multpred < 1) {
        input->no_multpred = 1;
    }
#if FIX_MAX_REF
    if (input->no_multpred > MAXREF) {
        fprintf(stdout, "\n At most %d reference frame is supported.\n", input->no_multpred);
        input->no_multpred = MAXREF;
    }
#endif

#if !INTERLACE_CODING
    //20080721
    if (input->progressive_sequence == 1 && input->progressive_frame == 0) {
        snprintf(hc->errortext, ET_SIZE, "\nprogressive_frame should be set to 1 in the case of progressive sequence input!");
        error(hc->errortext, 400);
    }

    if (input->progressive_frame == 1 && input->InterlaceCodingOption != 0) {
        snprintf(hc->errortext, ET_SIZE, "\nProgressive frame cann't use interlace coding!");
        error(hc->errortext, 400);
    }
#endif
#if INTERLACE_CODING_FIX
    if (input->InterlaceCodingOption == 1) {
        snprintf(hc->errortext, ET_SIZE, "\nfield coding in a frame is forbidden in AVS2-baseline!");
        error(hc->errortext, 400);
    }
    if (input->InterlaceCodingOption == 2) {
        snprintf(hc->errortext, ET_SIZE, "\npicture adaptive frame/field coding (PAFF) is forbidden in AVS2-baseline!");
        error(hc->errortext, 400);
    }
    if (input->InterlaceCodingOption == 3 && input->progressive_sequence == 1) {
        snprintf(hc->errortext, ET_SIZE, "\nfield picture coding  is forbidden when progressive_sequence is '1'");
        error(hc->errortext, 400);
    }
    if (input->progressive_sequence == 1 && input->progressive_frame == 0) {
        snprintf(hc->errortext, ET_SIZE, "\nprogressive_frame should be set to 1 when progressive_sequence is '1'!");
        error(hc->errortext, 400);
    }
#endif

    //20080721
    //qyu 0817 padding

    if (input->img_width % ui_MaxSize != 0) {
        img->auto_crop_right = ui_MaxSize - (input->img_width % ui_MaxSize);
    } else {
        img->auto_crop_right = 0;
    }
#if INTERLACE_CODING
    if (!input->progressive_sequence && !img->is_field_sequence)     //only used for frame picture's field coding
#else
    if (!input->progressive_sequence)   //20080721
#endif
    {
        if (input->img_height % (ui_MaxSize << 1) != 0) {
            img->auto_crop_bottom = (ui_MaxSize << 1) - (input->img_height % (ui_MaxSize << 1));
        } else {
            img->auto_crop_bottom = 0;
        }
    } else {
        if (input->img_height % ui_MaxSize != 0) {
            img->auto_crop_bottom = ui_MaxSize - (input->img_height % ui_MaxSize);
        } else {
            img->auto_crop_bottom = 0;
        }
    }
    if (input->slice_row_nr != 0 &&
        input->slice_row_nr > (int)((input->img_height / ui_MaxSize) * (input->img_width / ui_MaxSize))) {
        snprintf(hc->errortext, ET_SIZE, "\nNumber of LCUs in slice cannot exceed total LCU in picture!");
        error(hc->errortext, 400);
    }

    // check range of filter offsets
    if (input->alpha_c_offset > 8 || input->alpha_c_offset < -8) {   //20080721
        snprintf(hc->errortext, ET_SIZE, "Error input parameter LFAlphaC0Offset, check configuration file");
        error(hc->errortext, 400);
    }

    if (input->beta_offset > 8 || input->beta_offset < -8) {   //20080721
        snprintf(hc->errortext, ET_SIZE, "Error input parameter LFBetaOffset, check configuration file");
        error(hc->errortext, 400);
    }

    // Set block sizes

    // B picture consistency check
    if (input->successive_Bframe > input->jumpd) {
        snprintf(hc->errortext, ET_SIZE, "Number of B-frames %d can not exceed the number of frames skipped",
                 input->successive_Bframe);
        error(hc->errortext, 400);
    }
    if (input->successive_Bframe_all == 0) {
        input->low_delay = 1;
    } else {
        input->low_delay = 0;
    }

    // Open Files
    if (strlen(input->infile) > 0 && (he->p_in = fopen(input->infile, "rb")) == NULL) {
        snprintf(hc->errortext, ET_SIZE, "Input file %s does not exist", input->infile);
        error(hc->errortext, 400);//yuquanhe@hisilicon.com
    }

    if (input->output_enc_pic)   //output_enc_pic
        if (strlen(input->ReconFile) > 0 && (he->p_dec = fopen(input->ReconFile, "wb")) == NULL) {
            snprintf(hc->errortext, ET_SIZE, "Error open file %s", input->ReconFile);
            error(hc->errortext, 400);//yuquanhe@hisilicon.com
        }

    if (input->output_enc_pic && input->bg_enable)  //output_enc_pic
#if B_BACKGROUND_Fix
		if (strlen(input->ReconFile) > 0 && (he->p_dec_background = fopen(input->Backgroundref_File, "wb")) == NULL) {

			snprintf(hc->errortext, ET_SIZE, "Error open file Backgroundref_File %s", input->Backgroundref_File);
			error(hc->errortext, 400);//yuquanhe@hisilicon.com
#else
        if (strlen(input->ReconFile) > 0 && (he->p_dec_background = fopen("background_ref.yuv", "wb")) == NULL) {

            snprintf(hc->errortext, ET_SIZE, "Error open file background_ref.yuv");
            error(hc->errortext, 400);//yuquanhe@hisilicon.com

       
#endif
			 }
#if TRACE
    if (strlen(input->TraceFile) > 0 && (hc->p_trace = fopen(input->TraceFile, "w")) == NULL) {
        snprintf(hc->errortext, ET_SIZE, "Error open file %s", input->TraceFile);
    }
#endif
    // frame/field consistency check

    // input intra period and Seqheader check Add cjw, 20070327
    if (input->seqheader_period != 0 && input->intra_period == 0) {
        if (input->intra_period == 0) {
            snprintf(hc->errortext, ET_SIZE, "\nintra_period  should not equal %d when seqheader_period equal %d",
                     input->intra_period, input->seqheader_period);
        }

        error(hc->errortext, 400);
    }

    // input intra period and Seqheader check Add cjw, 20070327
    if (input->seqheader_period == 0 && input->vec_period != 0) {
        snprintf(hc->errortext, ET_SIZE, "\nvec_period  should not equal %d when seqheader_period equal %d",
                 input->intra_period,
                 input->seqheader_period);
        error(hc->errortext, 400);
    }

    if (input->profile_id == BASELINE_PROFILE && (input->sample_bit_depth > 8 || input->input_sample_bit_depth > 8)) {
        snprintf(hc->errortext, ET_SIZE, "\nThe value of sample_bit_depth or input_sample_bit_depth is invalid!");
        error(hc->errortext, 400);
    }

    // Added by LiShao, Tsinghua, 20070327
    //ProfileCheck();
    //LevelCheck();
}

/*
******************************************************************************
*  Function: Determine the MVD's value (1/4 pixel) is legal or not.
*  Input:
*  Output:
*  Return: 0: out of the legal mv range; 1: in the legal mv range
*  Attention:
*  Author: xiaozhen zheng, 20071009
******************************************************************************
*/
void DecideMvRange()
{
    if (input->profile_id == BASELINE10_PROFILE || input->profile_id == BASELINE_PROFILE) {
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
