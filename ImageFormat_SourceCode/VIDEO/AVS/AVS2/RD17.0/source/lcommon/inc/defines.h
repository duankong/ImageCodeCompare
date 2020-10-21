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
#ifndef _DEFINES_H_
#define _DEFINES_H_

#define RD      "17.0"
#define VERSION "17.0"

#define RESERVED_PROFILE_ID      0x24
#define BASELINE_PICTURE_PROFILE 18
#define BASELINE_PROFILE         32  //0x20
#define BASELINE10_PROFILE       34  //0x22
#define TRACE                    0 //!< 0:Trace off 1:Trace on

#include <stdlib.h>
#include <stdio.h>

/* Type definitions and file operation for Windows/Linux
 * All file operations for windows are replaced with native (FILE *) operations
 * Falei LUO (falei.luo@vipl.ict.ac.cn)
 * */
#ifdef WIN32
#undef fseek
#define fseek _fseeki64

#define int16 __int16
#define int64 __int64
#else  //! for Linux
#define _FILE_OFFSET_BITS 64       // for 64 bit fseeko
#define fseek fseeko

#define int16 int16_t
#define int64 int64_t
#endif


/***************************************************************************************
 * AVS2 macros start
 ***************************************************************************************/

#define INTERLACE_CODING                   1
#if INTERLACE_CODING  //M3531: MV scaling compensation
//Luma component
#define HALF_PIXEL_COMPENSATION            1 //common functions definition
#define HALF_PIXEL_COMPENSATION_PMV        1 //spacial MV prediction
#define HALF_PIXEL_COMPENSATION_DIRECT     1 //B direct mode
#define HALF_PIXEL_COMPENSATION_M1         1 //MV derivation method 1, weighted P_skip mode
#define HALF_PIXEL_COMPENSATION_M1_FUCTION 1 //M1 related with mv-scaling function
#define HALF_PIXEL_COMPENSATION_MVD        1 //MV scaling from FW->BW
//Chroma components
#define HALF_PIXEL_CHROMA                  1 //chroma MV is scaled with luma MV for 4:2:0 format
#define HALF_PIXEL_PSKIP                   1 //half pixel compensation for p skip/direct

#define INTERLACE_CODING_FIX               1 //HLS fix
#define OUTPUT_INTERLACE_MERGED_PIC        1

#endif
/*
*************************************************************************************
AVS2 10bit/12bit profile
*************************************************************************************
*/

#define DBFIX_10bit              1

/*
*************************************************************************************
AVS2 HIGH LEVEL SYNTAX
*************************************************************************************
*/
#define AVS2_HDR_HLS             1

#define AVS2_HDR_Tec                       0 //AVS2 HDR technology //yuquanhe@hisilicon.com
#if AVS2_HDR_Tec
#define HDR_CHROMA_DELTA_QP                0 //M3905
#endif
/*
*************************************************************************************
AVS2 S2
*************************************************************************************
*/
#define AVS2_S2_FASTMODEDECISION 1
#define RD1510_FIX_BG            1      // 20160714, flluo@pku.edu.cn


//////////////////// prediction techniques /////////////////////////////
#define LAM_2Level_TU            0.8


#define DIRECTION                4
#define DS_FORWARD               4
#define DS_BACKWARD              2
#define DS_SYM                   3
#define DS_BID                   1

#define MH_PSKIP_NUM             4
#define NUM_OFFSET               0
#define BID_P_FST                1
#define BID_P_SND                2
#define FW_P_FST                 3
#define FW_P_SND                 4
#define WPM_NUM                  3
#define MAX_MVP_CAND_NUM         2     // M3330 changes it to 2, the original value is 3

#define DMH_MODE_NUM             5     // Number of DMH mode
#define TH_ME                    0     // Threshold of ME

#define MV_SCALE                 1

//////////////////// reference picture management /////////////////////////////
#define FIX_MAX_REF              1     // Falei LUO, flluo@pku.edu.cn
#if FIX_MAX_REF
#define MAXREF                   7     // maximum number of reference frame for each frame
#define MAXGOP                   32
#endif
#define REF_MAXBUFFER            7

///////////////////Adaptive Loop Filter//////////////////////////
#define NUM_ALF_COEFF_CTX        1
#define NUM_ALF_LCU_CTX          4

#define LAMBDA_SCALE_LUMA       (1.0)
#define LAMBDA_SCALE_CHROMA     (1.0)



//////////////////// entropy coding /////////////////////////////
#define NUN_VALUE_BOUND          254  // M3090: Make sure rs1 will not overflow for 8-bit unsign char
#define Encoder_BYPASS_Final     1    // M3484
#define Decoder_Bypass_Annex     0    // M3484 
#define Decoder_Final_Annex      0    // M3540


//////////////////// coefficient coding /////////////////////////////
#define CG_SIZE                  16    // M3035 size of an coefficient group, 4x4

#define SWAP(x,y)                {(y)=(y)^(x);(x)=(y)^(x);(y)=(x)^(y);}

//////////////////// bug fix /////////////////////////////
#define ALFSliceFix                        1
#define WRITENBIT_FIX                      1
#define FIX_PROFILE_LEVEL_DPB_RPS_1        1
#define FIX_PROFILE_LEVEL_DPB_RPS_2        1
#define FIX_RPS_PICTURE_REMOVE             1   // flluo@pku.edu.cn
#define Mv_Clip                            1   //yuquanhe@hisilicon.com
#define REMOVE_UNUSED                      1   //yuquanhe@hisilicon.com
#define SAO_Height_Fix                     1   //yuquanhe@hisilicon.com
#define B_BACKGROUND_Fix                   1   //yuquanhe@hisilicon.com
#define Check_Bitstream                    1   //yuquanhe@hisilicon.com
#define Wq_param_Clip                      1   //yuquanhe@hisilicon.com
#define RD1501_FIX_BG                      1   //luofalei flluo@pku.edu.cn , wlq15@mails.tsinghua.edu.cn , Longfei.Wang@mediatek.com
#define Mv_Rang                            1   //yuquanhe@hisilicon.com ; he-yuan.lin@mstarsemi.com
#define RD160_FIX_BG                       1    // Longfei.Wang@mediatek.com ;fred.chiu@mediatek.com  jie1222.chen@samsung.com
#define RD1601_FIX_BG                      1    //Y_K_Tu@novatek.com.tw, he-yuan.lin@mstarsemi.com,victor.huang@montage-tech.com M4041
#define SEQ_CHANGE_CHECKER                 1    //he-yuan.lin@mstarsemi.com
#define Mv_check_bug                       1    //wlq15@mails.tsinghua.edu.cn
#define SAO_ASSERTION_FIX                  1    //fred.chiu@mediatek.com
#define FIELD_HORI_MV_NO_SCALE_FIX         1    //fred.chiu@mediatek.com
//////////////////// encoder optimization //////////////////////////////////////////////
#define TH                                 2

#define M3624MDLOG                             // reserved

#define TDRDO                               1  // M3528
//#define FIX_TDRDO_BG                        1  // flluo@pku.edu.cn, 20160318//
#define RATECONTROL                         1  // M3580 M3627 M3689
#define AQPO                                1  // M3623
#define AQPOM3694                           1
#define AQPOM3762                           1

//#define REPORT
//////////////////// Quantization   ///////////////////////////////////////
#define FREQUENCY_WEIGHTING_QUANTIZATION    1 // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#define CHROMA_DELTA_QP                     1
#define AWQ_WEIGHTING                       1
#define AWQ_LARGE_BLOCK_ENABLE              1
#define COUNT_BIT_OVERHEAD                  0
#define AWQ_LARGE_BLOCK_EXT_MAPPING         1
#endif

#define QuantClip                           1
#define QuantMatrixClipFix                  1  // 20160418, fllu@pku.edu.cn

#define WQ_MATRIX_FCD                       1
#if !WQ_MATRIX_FCD
#define WQ_FLATBASE_INBIT  7
#else
#define WQ_FLATBASE_INBIT  6
#endif


#define REFINED_QP                          1


//////////////////// delta QP /////////////////////////////
#define MB_DQP                    1      // M3122: the minimum dQP unit is Macro block
#define LEFT_PREDICTION           1      // M3122: 1 represents left prediction and 0 represents previous prediction


////////////////////////SAO//////////////////////////////////////////////////////////////
#define NUM_BO_OFFSET             32
#define MAX_NUM_SAO_CLASSES       32
#define NUM_SAO_BO_CLASSES_LOG2   5
#define NUM_SAO_BO_CLASSES_IN_BIT 5
#define MAX_DOUBLE                1.7e+308
#define NUM_SAO_EO_TYPES_LOG2     2
#define NUM_SAO_BO_CLASSES        (1<<NUM_SAO_BO_CLASSES_LOG2)
#define SAO_RATE_THR              0.75
#define SAO_RATE_CHROMA_THR       1
#define SAO_SHIFT_PIX_NUM         4

#define SAO_PARA_CROSS_SLICE      1
#define SAO_MULSLICE_FTR_FIX      1

///////////////////// Transform /////////////////////
#define SEC_TR_SIZE               4
#define SEC_TR_MIN_BITSIZE        3   // apply secT to greater than or equal to 8x8 block,

#define BUGFIXED_COMBINED_ST_BD   1

///////////////////// Scalable /////////////////////
#define M3480_TEMPORAL_SCALABLE   1
#define TEMPORAL_MAXLEVEL         8
#define TEMPORAL_MAXLEVEL_BIT     3




/*
*************************************************************************************
* AVS2 macros end
*
*************************************************************************************
*/

#define CHROMA                    1
#define LUMA_8x8                  2
#define NUM_BLOCK_TYPES           8

#define clamp(a,b,c) ( (a)<(b) ? (b) : ((a)>(c)?(c):(a)) )    //!< clamp a to the range of [b;c]

#define LOG2_MAX_FRAME_NUM_MINUS4    4                // POC200301 moved from defines.h
#define MAX_CODED_FRAME_SIZE         15000000         //!< bytes for one frame

// ---------------------------------------------------------------------------------
// FLAGS and DEFINES for new chroma intra prediction, Dzung Hoang
// Threshold values to zero out quantized transform coefficients.
// Recommend that _CHROMA_COEFF_COST_ be low to improve chroma quality
#define _LUMA_COEFF_COST_         4 //!< threshold for luma coeffs

#define IMG_PAD_SIZE              64   //!< Number of pixels padded around the reference frame (>=4)

#define OUTSTRING_SIZE            255


#define absm(A) ((A)<(0) ? (-(A)):(A)) //!< abs macro, faster than procedure
#define MAX_VALUE                999999   //!< used for start value for some variables

#define Clip1(a)            ((a)>255?255:((a)<0?0:(a)))
#define Clip3(min,max,val)  (((val)<(min))?(min):(((val)>(max))?(max):(val)))

// ---------------------------------------------------------------------------------

// block size of block transformed by AVS
#define PSKIPDIRECT               0
#define P2NX2N                    1
#define P2NXN                     2
#define PNX2N                     3
#define PHOR_UP                   4
#define PHOR_DOWN                 5
#define PVER_LEFT                 6
#define PVER_RIGHT                7
#define PNXN                      8
#define I8MB                      9
#define I16MB                     10
#define IBLOCK                    11
#define InNxNMB                   12
#define INxnNMB                   13
#define MAXMODE                   14   // add yuqh 20130824
#define  LAMBDA_ACCURACY_BITS     16
#define  LAMBDA_FACTOR(lambda)        ((int)((double)(1<<LAMBDA_ACCURACY_BITS)*lambda+0.5))
#define  WEIGHTED_COST(factor,bits)   (((factor)*(bits))>>LAMBDA_ACCURACY_BITS)
#define  MV_COST(f,s,cx,cy,px,py)     (WEIGHTED_COST(f,mvbits[((cx)<<(s))-px]+mvbits[((cy)<<(s))-py]))
#define  REF_COST(f,ref)              (WEIGHTED_COST(f,refbits[(ref)]))

#define  BWD_IDX(ref)                 (((ref)<2)? 1-(ref): (ref))
#define  REF_COST_FWD(f,ref)          (WEIGHTED_COST(f,((img->num_ref_pic_active_fwd_minus1==0)? 0:refbits[(ref)])))
#define  REF_COST_BWD(f,ref)          (WEIGHTED_COST(f,((img->num_ref_pic_active_bwd_minus1==0)? 0:BWD_IDX(refbits[ref]))))

#define IS_INTRA(MB)                  ((MB)->cuType==I8MB||(MB)->cuType==I16MB||(MB)->cuType==InNxNMB ||(MB)->cuType==INxnNMB)
#define IS_INTER(MB)                  ((MB)->cuType!=I8MB && (MB)->cuType!=I16MB&&(MB)->cuType!=InNxNMB &&(MB)->cuType!=INxnNMB)
#define IS_INTERMV(MB)                ((MB)->cuType!=I8MB && (MB)->cuType!=I16MB &&(MB)->cuType!=InNxNMB &&(MB)->cuType!=INxnNMB&& (MB)->cuType!=0)


#define IS_DIRECT(MB)                ((MB)->cuType==PSKIPDIRECT     && (img ->   type==B_IMG))
#define IS_P_SKIP(MB)                ((MB)->cuType==PSKIPDIRECT     && (((img ->   type==F_IMG))||((img ->   type==P_IMG))))
#define IS_P8x8(MB)                  ((MB)->cuType==PNXN)

// Quantization parameter range
#define MIN_QP                       0
#define MAX_QP                       63
#define SHIFT_QP                     11

// Picture types
#define INTRA_IMG                    0   //!< I frame
#define INTER_IMG                    1   //!< P frame
#define B_IMG                        2   //!< B frame
#define I_IMG                        0   //!< I frame
#define P_IMG                        1   //!< P frame
#define F_IMG                        4  //!< F frame

#define BACKGROUND_IMG               3

#define BP_IMG                       5


// Direct Mode types
#define MIN_CU_SIZE                  8
#define MIN_BLOCK_SIZE               4
#define MIN_CU_SIZE_IN_BIT           3
#define MIN_BLOCK_SIZE_IN_BIT        2
#define BLOCK_MULTIPLE              (MIN_CU_SIZE/(MIN_BLOCK_SIZE))
#define MAX_CU_SIZE                  64
#define MAX_CU_SIZE_IN_BIT           6
#define B4X4_IN_BIT                  2
#define B8X8_IN_BIT                  3
#define B16X16_IN_BIT                4
#define B32X32_IN_BIT                5
#define B64X64_IN_BIT                6
#define NUM_INTRA_PMODE              33        //!< # luma intra prediction modes
#define NUM_MODE_FULL_RD             9         // number of luma modes for full RD search
#define NUM_INTRA_PMODE_CHROMA       5         //!< #chroma intra prediction modes

// luma intra prediction modes

#define DC_PRED                      0
#define PLANE_PRED                   1
#define BI_PRED                      2
#define VERT_PRED                    12
#define HOR_PRED                     24


// chroma intra prediction modes
#define DM_PRED_C                    0
#define DC_PRED_C                    1
#define HOR_PRED_C                   2
#define VERT_PRED_C                  3
#define BI_PRED_C                    4

#define EOS                          1         //!< End Of Sequence
#define SOP                          2                       //!< Start Of Picture

#define DECODING_OK                  0
#define SEARCH_SYNC                  1
#define DECODE_MB                    1

#ifndef max
#define max(a, b)                   ((a) > (b) ? (a) : (b))  //!< Macro returning max value
#define min(a, b)                   ((a) < (b) ? (a) : (b))  //!< Macro returning min value
#endif


#define XY_MIN_PMV                   1
#if XY_MIN_PMV
#define MVPRED_xy_MIN                0
#else
#define MVPRED_MEDIAN                0
#endif
#define MVPRED_L                     1
#define MVPRED_U                     2
#define MVPRED_UR                    3

#define DUAL                         4
#define FORWARD                      0
#define BACKWARD                     1
#define SYM                          2
#define BID                          3
#define INTRA                        -1

#define BUF_CYCLE                    5

#define ROI_M3264                    1      // ROI Information Encoding

#define PicExtensionData             1


#define REF_OUTPUT                   1  //M3337


/* MV scaling 14 bit */
#define MULTI                        16384
#define HALF_MULTI                   8192
#define OFFSET                       14
/* end of MV scaling */

#define MV_DECIMATION_FACTOR         4  // store the middle pixel's mv in a motion information unit

/* BUGFIX_AVAILABILITY_INTRA */
#define NEIGHBOR_INTRA_LEFT                 0
#define NEIGHBOR_INTRA_UP                   1
#define NEIGHBOR_INTRA_UP_RIGHT             2
#define NEIGHBOR_INTRA_UP_LEFT              3
#define NEIGHBOR_INTRA_LEFT_DOWN            4
/* end of BUGFIX_AVAILABILITY_INTRA */


#endif // #if _DEFINES_H_
