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
* File name: commonStructures.h
* Function:  common structures definitions for for AVS encoder and decoder.
*
*************************************************************************************
*/

#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdio.h>                              //!< for FILE
#include "defines.h"

#ifdef WIN32
#define  snprintf _snprintf
# define  open     _open
# define  close    _close
# define  read     _read
# define  write    _write
# define  lseek    _lseeki64
# define  fsync    _commit
# define  tell     _telli64
#endif

typedef unsigned short byte;    //!< byte type definition
#define pel_t byte

//! Boolean Type
typedef enum {
    FALSE,
    TRUE
} Boolean;

typedef enum {
    FRAME_CODING,
    FIELD_CODING,
} /*CodingType*/;

typedef enum {
    FIELD,
    FRAME
};

typedef enum {
    UVLC,
    AEC
};

typedef enum {
    SE_HEADER,
    SE_PTYPE,
    SE_MBTYPE,
    SE_REFFRAME,
    SE_INTRAPREDMODE,
    SE_MVD,
    SE_DMH,
    SE_WPM1,
    SE_WPM2,
    SE_CBP_INTRA,
    SE_LUM_DC_INTRA,
    SE_CHR_DC_INTRA,
    SE_LUM_AC_INTRA,
    SE_CHR_AC_INTRA,
    SE_CBP_INTER,
    SE_CBP01,
    SE_LUM_DC_INTER,
    SE_CHR_DC_INTER,
    SE_LUM_AC_INTER,
    SE_CHR_AC_INTER,
    SE_DELTA_QUANT_INTER,
    SE_DELTA_QUANT_INTRA,
    SE_BFRAME,
    SE_EOS,
    SE_MAX_ELEMENTS  //!< number of maximum syntax elements, this MUST be the last one!
} /*SE_type*/;

typedef enum {
    BITS_HEADER,
    BITS_TOTAL_MB,
    BITS_MB_MODE,
    BITS_INTER_MB,
    BITS_CBP_MB,
    BITS_CBP01_MB,
    BITS_COEFF_Y_MB,
    BITS_COEFF_UV_MB,
    BITS_DELTA_QUANT_MB,
    BITS_SAO_MB,
    MAX_BITCOUNTER_MB
} BitCountType;

typedef enum {
    INTRA_PRED_VER = 0,
    INTRA_PRED_HOR,
    INTRA_PRED_DC_DIAG
};

typedef enum SAOComponentIdx {
    SAO_Y = 0,
    SAO_Cb,
    SAO_Cr,
    NUM_SAO_COMPONENTS
};

typedef enum SAOMode { //mode
    SAO_MODE_OFF = 0,
    SAO_MODE_MERGE,
    SAO_MODE_NEW,
    NUM_SAO_MODES
};

typedef enum SAOModeMergeTypes {
    SAO_MERGE_LEFT = 0,
    SAO_MERGE_ABOVE,
    NUM_SAO_MERGE_TYPES
};


typedef enum SAOModeNewTypes { //NEW: types
    SAO_TYPE_EO_0,
    SAO_TYPE_EO_90,
    SAO_TYPE_EO_135,
    SAO_TYPE_EO_45,
    SAO_TYPE_BO ,
    NUM_SAO_NEW_TYPES
};
enum SAOEOClasses { // EO Groups, the assignments depended on how you implement the edgeType calculation
    SAO_CLASS_EO_FULL_VALLEY = 0,
    SAO_CLASS_EO_HALF_VALLEY = 1,
    SAO_CLASS_EO_PLAIN       = 2,
    SAO_CLASS_EO_HALF_PEAK   = 3,
    SAO_CLASS_EO_FULL_PEAK   = 4,
    SAO_CLASS_BO             = 5,
    NUM_SAO_EO_CLASSES = SAO_CLASS_BO,
    NUM_SAO_OFFSET
};

struct SAOstatdata {
    long int diff[MAX_NUM_SAO_CLASSES];
    long int  count[MAX_NUM_SAO_CLASSES];
};

typedef struct {
    int extension_id;
    int copyright_flag;
    int copyright_id;
    int original_or_copy;
    int reserved;
    int copyright_number;
} CopyRight;

typedef struct {
    int reserved;
    int camera_id;
    int height_of_image_device;
    int focal_length;
    int f_number;
    int vertical_angle_of_view;
    int camera_position_x;
    int camera_position_y;
    int camera_position_z;
    int camera_direction_x;
    int camera_direction_y;
    int camera_direction_z;
    int image_plane_vertical_x;
    int image_plane_vertical_y;
    int image_plane_vertical_z;
} CameraParamters;

//! SNRParameters
typedef struct {
    double snr_y;               //!< current Y SNR
    double snr_u;               //!< current U SNR
    double snr_v;               //!< current V SNR
    double snr_y1;              //!< SNR Y(dB) first frame
    double snr_u1;              //!< SNR U(dB) first frame
    double snr_v1;              //!< SNR V(dB) first frame
    double snr_ya;              //!< Average SNR Y(dB) remaining frames
    double snr_ua;              //!< Average SNR U(dB) remaining frames
    double snr_va;              //!< Average SNR V(dB) remaining frames
#if INTERLACE_CODING
    double i_snr_ya;               //!< current Y SNR
    double i_snr_ua;               //!< current U SNR
    double i_snr_va;               //!< current V SNR
#endif
} SNRParameters;

// signal to noise ratio parameters
/*struct snr_par
{
  double snr_y;                                //<! current Y SNR
  double snr_u;                                //<! current U SNR
  double snr_v;                                //<! current V SNR
  double snr_y1;                               //<! SNR Y(dB) first frame
  double snr_u1;                               //<! SNR U(dB) first frame
  double snr_v1;                               //<! SNR V(dB) first frame
  double snr_ya;                               //<! Average SNR Y(dB) remaining frames
  double snr_ua;                               //<! Average SNR U(dB) remaining frames
  double snr_va;                               //<! Average SNR V(dB) remaining frames
};*/

//! codingUnit
typedef struct codingUnit {
    unsigned int        ui_MbBitSize;
    int                 uiBitSize;            //size of MB
    int                 currSEnr;             //!< number of current syntax element
    int                 slice_nr;
    int                 delta_quant;          //!< for rate control
    int                 delta_qp;
    int                 qp ;
    int                 bitcounter[MAX_BITCOUNTER_MB];
    struct codingUnit
        *mb_available[3][3];        /*!< pointer to neighboring MBs in a 3x3 window of current MB, which is located at [1][1] \n
                          NULL pointer identifies neighboring MBs which are unavailable */
    // some storage of codingUnit syntax elements for global access
    int                 cuType;
    int                 weighted_skipmode;

    int                 md_directskip_mode;

    int                 trans_size;
    int
    mvd[2][BLOCK_MULTIPLE][BLOCK_MULTIPLE][3];          //!< indices correspond to [forw,backw][block_y][block_x][x,y, dmh]

    int                 intra_pred_modes[BLOCK_MULTIPLE * BLOCK_MULTIPLE];
    int                 real_intra_pred_modes[BLOCK_MULTIPLE * BLOCK_MULTIPLE];
    int                 l_ipred_mode;
    int                 cbp, cbp_blk;
    unsigned long cbp_bits;

    int                 b8mode[4];
    int                 b8pdir[4];
    int                 c_ipred_mode;      //!< chroma intra prediction mode


    struct codingUnit   *mb_available_up;   //!< pointer to neighboring MB (AEC)
    struct codingUnit   *mb_available_left; //!< pointer to neighboring MB (AEC)
    int                 mbAddrA, mbAddrB, mbAddrC, mbAddrD;

    int                 slice_set_index;       //!<added by mz, 2008.04
    int                 slice_header_flag;     //added by mz, 2008.04
    int                 sliceqp;         //added by mz, 2008.04
#if MB_DQP
    int                 previouse_qp;
    int                 left_cu_qp;
#endif
    int                 block_available_up;
    int                 block_available_left;

} codingUnit;

//rm52k_r2


//! all input parameters
typedef struct {
    int no_frames;                //!< number of frames to be encoded
    int qpI;                      //!< QP of first frame
    int qpP;                      //!< QP of remaining frames
    int jumpd;                    //!< number of frames to skip in input sequence (e.g 2 takes frame 0,3,6,9...)
    int jumpd_all;
    int jumpd_sub[4];
    int hadamard;                 /*!< 0: 'normal' SAD in 1/3 pixel search.  1: use 4x4 Haphazard transform and '
                  Sum of absolute transform difference' in 1/3 pixel search                   */
    int usefme;                   //!< Fast Motion Estimat. 0=disable, 1=UMHexagonS
    int search_range;             /*!< search range - integer pel search and 16x16 blocks.  The search window is
                  generally around the predicted vector. Max vector is 2xmcrange.  For 8x8
                  and 4x4 block sizes the search range is 1/2 of that for 16x16 blocks.       */
    int no_multpred;              /*!< 1: prediction from the last frame only. 2: prediction from the last or
                  second last frame etc.  Maximum 5 frames                                    */
    int img_width;                //!< GH: if CUSTOM image format is chosen, use this size
    int img_height;               //!< GH: width and height must be a multiple of 16 pels
    int yuv_format;               //!< GH: YUV format (0=4:0:0, 1=4:2:0, 2=4:2:2, 3=4:4:4,currently only 4:2:0 is supported)

#if INTERLACE_CODING
    int org_img_width;            //!< The width for the input parameter
    int org_img_height;           //!< The height for the input parameter
    int org_no_frames;            //!< The number of the coding frames for the input parameter
    int output_merged_picture;
#endif


    int infile_frameskip;


    char infile[1000];            //!< YUV 4:2:0 input format
    char ReconFile[1000];         //!< Reconstructed Pictures
#if B_BACKGROUND_Fix
	char Backgroundref_File[1000]; 
#endif
#if ROI_M3264
    char infile_data[1000];       //!< input ROI data...
    int ROI_Coding;
#endif
    char outfile[1000];           //!< AVS compressed output bitstream
    char TraceFile[1000];         //!< Trace Outputs
    int intra_period;

    int fframe_enabled;

    int dhp_enabled;

    int b_mhpskip_enabled;

    int  b_dmh_enabled;
    int  dmh_enabled_encoder;

    int  b_secT_enabled;

    int wsm_enabled;

    // B pictures
    int successive_Bframe;        //!< number of B frames that will be used

    int successive_Bframe_all;
    int successive_Bframe_sub[4];

    int Hierarchical_B;
    int qpB;                      //!< QP of B frames

    int InterSearch16x16;
    int InterSearch16x8;
    int InterSearch8x16;
    int InterSearch8x8;

    int InterSearchAMP;
#if PicExtensionData
    int PicExtentData;
#endif

    int rdopt;

    int InterlaceCodingOption;

    //AVS
    int aspect_ratio_information;
    int frame_rate_code;//xfwang  2004.7.28
    //int bit_rate;
    int bit_rate_lower;
    int bit_rate_upper;

    int vec_period;
    int seqheader_period; // Random Access period though sequence header

    int video_format;
    int color_description;
    int color_primaries;
    int transfer_characteristics;
    int matrix_coefficients;
    int hour;
    int minute;
    int second;
    int frame_offset;
    int profile_id;
    unsigned int g_uiMaxSizeInBit;
    int level_id;
    int progressive_sequence;
    int repeat_first_field;
    int top_field_first;
    int low_delay;
    int chroma_format;
    int yuv_structure;
    int sample_precision;
    int video_range;
    int progressive_frame;
    int fixed_picture_qp;
    int time_code_flag;
    int display_horizontal_size;
    int display_vertical_size;

#if AVS2_HDR_HLS
    int hdr_metadata;
    int display_primaries_x0;
    int display_primaries_y0;
    int display_primaries_x1;
    int display_primaries_y1;
    int display_primaries_x2;
    int display_primaries_y2;
    int white_point_x;
    int white_point_y;
    int max_display_mastering_luminance;
    int min_display_mastering_luminance;
    int maximum_content_light_level;
    int maximum_frame_average_light_level;
#endif

    int TD_mode;
    int view_packing_mode;
    int view_reverse;

    //  int slice_enable;
    int slice_parameter;
    int slice_row_nr;
    int output_enc_pic;  //output_enc_pic
    int loop_filter_disable;
    int loop_filter_parameter_flag;
    int alpha_c_offset;
    int beta_offset;
#if M3480_TEMPORAL_SCALABLE
    int TemporalScalableFlag;
#endif
    int sao_enable;
    int alf_enable;
    int alf_LowLatencyEncodingEnable;

    int crossSliceLoopFilter;

    int channel_type;
    int frame_rate;
    int stuff_height;

    int  use_rdoq;
    int  lambda_factor_rdoq;
    int  lambda_factor_rdoq_p;
    int  lambda_factor_rdoq_b;
    int  b_pmvr_enabled;

    int  useNSQT;
#if MB_DQP
    int useDQP;
#endif
    int  useSDIP;

#if REFINED_QP
    int use_refineQP;
#endif

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
#define WQMODEL_PARAM_SIZE 64       //M2148 2007-09
#define FILE_NAME_SIZE 100          //M2331 2008-04
#endif

#if FREQUENCY_WEIGHTING_QUANTIZATION

    int  WQEnable;
    int  SeqWQM;
    char SeqWQFile[256];

    int PicWQEnable;
    int WQParam;
    int WQModel;
    char WeightParamDetailed[WQMODEL_PARAM_SIZE];         //M2148 2007-09
    char WeightParamUnDetailed[WQMODEL_PARAM_SIZE];           //M2148 2007-09
    int MBAdaptQuant;                     //M2331 2008-04
    //#if CHROMA_DELTA_QP
    int chroma_quant_param_disable;
    int chroma_quant_param_delta_u;
    int chroma_quant_param_delta_v;
    //#endif
    int PicWQDataIndex;
    char PicWQFile[256];
#endif


    char bbv_mode;
    int  bbv_buffer_size;

    int  bg_enable;

    char bg_file_name[1000];
    int  bg_input_number;
    int  bg_period;
    int  bg_model_number;
    int  bg_qp;
    int  bg_model_method;
    int  always_no_bgmodel;

#if AVS2_S2_FASTMODEDECISION
    int   bg_fast_mode;
#endif

//#if  HDR_CHROMA_DELTA_QP
	int chroma_hdr_chroma_delta_disable;
	int    chromaCbQpScale;
	int    chromaCrQpScale;
	int    chromaQpScale;
	int    chromaQpOffset;
//#endif


    int   input_sample_bit_depth; // inputfile bit depth
    int   sample_bit_depth;  // sample bit depth

#ifdef TDRDO
    int TDEnable;
#endif

#ifdef AQPO
    int AQPEnable;
#endif

#ifdef RATECONTROL
    int EncControl;
    int TargetBitRate;
    int ECInitialQP;
#endif


    int MD5Enable;

} InputParameters;


// image parameters
typedef struct syntaxelement SyntaxElement;
typedef struct slice Slice;
typedef struct SAOstatdata SAOStatData;
typedef struct alfdatapart AlfDataPart;
typedef struct {
    int modeIdc; //NEW, MERGE, OFF
    int typeIdc; //NEW: EO_0, EO_90, EO_135, EO_45, BO. MERGE: left, above
    int startBand; //BO: starting band index
    int startBand2;
    int deltaband;
    int offset[MAX_NUM_SAO_CLASSES];
} SAOBlkParam;
typedef struct {
    int alf_flag;
    int num_coeff;
    int filters_per_group;
    int componentID;
    int *filterPattern;
    int **coeffmulti;
} ALFParam;

enum ALFComponentID {
    ALF_Y = 0,
    ALF_Cb,
    ALF_Cr,
    NUM_ALF_COMPONENT
};
typedef struct {
    int usedflag;
    int cur_number;
    int max_number;
    ALFParam alf_par[NUM_ALF_COMPONENT];
} ALF_APS;


/* ------------------------------------------------------
 * frame data
 */
typedef struct {
    int imgcoi_ref;
    byte **referenceFrame[3];
    int **refbuf;
    int ** *mvbuf;
    double saorate[NUM_SAO_COMPONENTS];
    byte ** *ref;

    int imgtr_fwRefDistance;
    int refered_by_others;
    int is_output;
#if M3480_TEMPORAL_SCALABLE
    int temporal_id;          //temporal level setted in configure file
#endif
    byte **oneForthRefY;
#if FIX_MAX_REF
    int ref_poc[MAXREF];
#else
    int ref_poc[4];
#endif
} avs2_frame_t;


typedef struct {
    codingUnit    *mb_data;
    int number;                                 //<! frame number
    int numIPFrames;

    int type;
    int typeb;
    int typeb_before;

    int qp;                                     //<! quant for the current frame
    int current_mb_nr;              // bitstream order
    int current_slice_nr;
    int tr;                                     //<! temporal reference, 8 bit,

    int width;                   //!< Number of pels
    int width_cr;                //!< Number of pels chroma
    int height;                  //!< Number of lines
    int height_cr;               //!< Number of lines  chroma
    int PicWidthInMbs;
    int PicSizeInMbs;
    int block8_x, block8_y;
    int   subblock_x;
    int   subblock_y;

    int num_of_references;

    int auto_crop_right;                        //<! Bug Fix: correct picture size for outputted reconstructed pictures
    int auto_crop_bottom;
    int buf_cycle;
    int picture_structure;
    Slice       *currentSlice;                  //<! pointer to current Slice data struct

    int **predBlock;             //!< current best prediction mode
    int **predBlockTmp;
    int **resiY;              //!< the diff pixel values between orginal image and prediction
    int *quad;               //!< Array containing square values,used for snr computation

    //////////////////////////////////////////////////////////////////////////
    /////////////////////location of current MB////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    int mb_y;                    //!< current MB vertical
    int mb_x;                    //!< current MB horizontal
    int pix_y;                   //!< current pixel vertical
    int pix_x;                   //!< current pixel horizontal
    int pix_c_y;                 //!< current pixel chroma vertical
    int pix_c_x;                 //!< current pixel chroma horizontal

    int imgtr_next_P;

    int imgcoi_next_ref;

    int **ipredmode;             //!< GH ipredmode[90][74];prediction mode for inter frames */ /* fix from ver 4.1
    int **rec_ipredmode;


////////////////decoder//////////////////////////
    int max_mb_nr;
    int **intra_block;

    int block_y;
    int block_x;

    int ** *tmp_mv;                                 //<! [92][72][3]  // delete by xfwang 2004.7.29
    int ** *p_snd_tmp_mv;
    int resiUV[2][MAX_CU_SIZE][MAX_CU_SIZE];                       //<! final 4x4 block. Extended to 16x16 for AVS

    int **fw_refFrArr;                          //<! [72][88];
    int **bw_refFrArr;                          //<! [72][88];

    int random_access_decodable_flag;

    int seq_header_indicate;
    int B_discard_flag;

    // B pictures
    int ** *fw_mv;                              //<! [92][72][3];
    int ** *bw_mv;                              //<! [92][72][3];

    unsigned int pic_distance;

    unsigned int coding_order;

    unsigned int PrevPicDistanceLsb;
    signed int CurrPicDistanceMsb;

    int PicHeightInMbs;

    int types;

    int new_sequence_flag;
    int sequence_end_flag;            //<! rm52k_r2

    int current_slice_set_index;          //<! added by mz, 2008.04
    int current_slice_header_flag;        //<! added by mz, 2008.04
    int slice_set_qp[64];             //<! added by mz, 2008.04


    int inter_amp_enable;

//////////////////////////encoder//////////////////////////

    // int nb_references;     //!< replaced by "num_of_references"

    int framerate;

    int ** *predBlockY;        //!< all 9 prediction modes
    int ** **predBlockUV;     //!< new chroma 8x8 intra prediction modes

    int **Coeff_all;//qyu 0821

    SyntaxElement   *MB_SyntaxElements; //!< by oliver 0612

    // B pictures

    int b_frame_to_code;
    int num_ref_pic_active_fwd_minus1;
    int num_ref_pic_active_bwd_minus1;
    int mv_range_flag;

    unsigned int frame_num;   //frame_num for this frame
    int slice_offset;
    //the following are sent in the slice header
    int NoResidueDirect;
    int coded_mb_nr;
    int progressive_frame;
    int tc_reserve_bit;
    int mb_no_currSliceLastMB; // the last MB no in current slice.      Yulj 2004.07.15
    int Seqheader_flag;     // Added by cjw, 20070327
    int EncodeEnd_flag;         // Carmen, 2007/12/19

    unsigned short bbv_delay;

    int ** ** *allSymMv;     //!< replaces local allSymMv
    int ** ** *allFwMv;     //!< replaces local allFwMv
    int ** ** *allBwMv;    //!< replaces local allBwMv
    int ** ** *forwardpred_mv;
    int ** ** *forwardpred_allFwMv;
    int ** ** *allBidFwMv;
    int ** ** *allBidBwMv;

    int ** ** *allIntegerBFwMv;
    int ** ** *allIntegerBBwMv;
    int ** ** *allIntegerPFwMv;

    int ** ** *mv;               //!< motion vectors for all block types and all reference frames
    int ** ** *predBFwMv;     //!< for MVDFW
    int ** ** *predBBwMv;     //!< for MVDBW
    int ** ** *predSymMv  ; //!< for MVD Bid

    int ** ** *predBidFwMv;
    int ** ** *predBidBwMv;

    int ** ** *predDualFstMv;  //first motion vector predictor of dual hypothesis prediction mode
    int ** ** *predDualSndMv;  //second motion vector predictor of dual hypothesis prediction mode

    int ** ** *allDualFstMv;  //first motion vector of dual hypothesis prediction mode
    int ** ** *allDualSndMv;  //second motion vector of dual hypothesis prediction mode

    int tmp_fwBSkipMv[DIRECTION + 1][2];
    int tmp_bwBSkipMv[DIRECTION + 1][2];

    int tmp_pref_fst[MH_PSKIP_NUM + NUM_OFFSET + 1];
    int tmp_pref_snd[MH_PSKIP_NUM + NUM_OFFSET + 1];
    int tmp_fstPSkipMv[MH_PSKIP_NUM + NUM_OFFSET + 1][3];
    int tmp_sndPSkipMv[MH_PSKIP_NUM + NUM_OFFSET + 1][3];
    int ** *currMv;
    int ** *currFwMv;
    int ** *currBwMv;
    int  **currRef;
    int  **currFwRef;
    int  **currBwRef;
    int  **currPSndRef;
    int ** *currPSndMv;

    int  **ipredmode_curr;
    int  **Coeff_all_to_write;
    byte **recon_currY , * *recon_currU, * *recon_currV;
    ////////////////SAO parameter//////////////////
    SAOStatData ** *saostatData; //[SMB][comp][types]
    SAOBlkParam  **saoBlkParams; //[SMB][comp]
    SAOBlkParam  **rec_saoBlkParams;//[SMB][comp]
    double        *cur_saorate;
    int            slice_sao_on[NUM_SAO_COMPONENTS];
    int            pic_alf_on[NUM_ALF_COMPONENT];
    int        ** *Coeff_all_to_write_ALF;
    AlfDataPart   *dp_ALF;

#if INTERLACE_CODING
    int is_field_sequence;
    int is_top_field;
#endif


} ImageParameters;



//! struct for context management
typedef struct {
    unsigned char MPS;   //1 bit
    unsigned int  LG_PMPS; //10 bits
    unsigned char cycno;  //2 bits
} BiContextType;
typedef BiContextType *BiContextTypePtr;

/***********************************************************************
* D a t a    t y p e s   f o r  A E C
************************************************************************/



typedef struct pix_pos {
    int available;   //ABCD
    int mb_addr;    //MB position
    int x;
    int y;
    int pos_x;     //4x4 x-pos
    int pos_y;
} PixelPos;



typedef struct {
    int type;
    int typeb;

    int   framenum;
    int   tr;
    int   qp;
    double snr_y;
    double snr_u;
    double snr_v;
    int   tmp_time;
    int   picture_structure;
    int   curr_frame_bits;
    int   emulate_bits;

    unsigned int DecMD5Value[4];
#if RD1501_FIX_BG
	int background_picture_output_flag;//Longfei.Wang@mediatek.com
#endif
#if	RD160_FIX_BG
	int picture_reorder_delay;
#endif
    char str_reference_list[128];  // reference list information
} STDOUT_DATA;

/**********************************************************************
* C O N T E X T S   F O R   T M L   S Y N T A X   E L E M E N T S
**********************************************************************
*/
#define NUM_CuType_CTX              11+10
#define NUM_B8_TYPE_CTX              9
#define NUM_MVD_CTX                 15
#define NUM_PMV_IDX_CTX             10
#define NUM_REF_NO_CTX               6
#define NUM_DELTA_QP_CTX             4
#define NUM_INTER_DIR_CTX           18
#define NUM_INTER_DIR_DHP_CTX           3
#define NUM_B8_TYPE_DHP_CTX             1
#define NUM_AMP_CTX                  2
#define NUM_C_INTRA_MODE_CTX         4
#define NUM_CBP_CTX                  4
#define NUM_BCBP_CTX                 4
#define NUM_MAP_CTX                 17
#define NUM_LAST_CTX                17

#define NUM_INTRA_MODE_CTX           7

#define NUM_ABS_CTX                  5
#define NUM_TU_CTX                   3
#define NUM_SPLIT_CTX                8  // CU depth

#define NUM_BRP_CTX                  8


#define NUM_LAST_CG_CTX_LUMA        12
#define NUM_LAST_CG_CTX_CHROMA       6
#define NUM_SIGCG_CTX_LUMA           2
#define NUM_SIGCG_CTX_CHROMA         1
#define NUM_LAST_POS_CTX_LUMA   56
#define NUM_LAST_POS_CTX_CHROMA 16
#define NUM_LAST_CG_CTX             (NUM_LAST_CG_CTX_LUMA+NUM_LAST_CG_CTX_CHROMA)
#define NUM_SIGCG_CTX               (NUM_SIGCG_CTX_LUMA+NUM_SIGCG_CTX_CHROMA)
#define NUM_LAST_POS_CTX        (NUM_LAST_POS_CTX_LUMA+NUM_LAST_POS_CTX_CHROMA)
#define NUM_SAO_MERGE_FLAG_CTX                   3
#define NUM_SAO_MODE_CTX                         1
#define NUM_SAO_OFFSET_CTX                       2
#define NUM_INTER_DIR_MIN_CTX         2


typedef struct {
    BiContextType cuType_contexts       [NUM_CuType_CTX];
    BiContextType pdir_contexts         [NUM_INTER_DIR_CTX];
    BiContextType amp_contexts          [NUM_AMP_CTX];
    BiContextType b8_type_contexts      [NUM_B8_TYPE_CTX];
    BiContextType pdir_dhp_contexts         [NUM_INTER_DIR_DHP_CTX];
    BiContextType b8_type_dhp_contexts      [NUM_B8_TYPE_DHP_CTX];
    BiContextType b_dir_skip_contexts   [DIRECTION];
    BiContextType p_skip_mode_contexts  [MH_PSKIP_NUM];

    BiContextType wpm_contexts[WPM_NUM];

    BiContextType mvd_contexts          [3][NUM_MVD_CTX];
    BiContextType pmv_idx_contexts      [2][NUM_PMV_IDX_CTX];
    BiContextType ref_no_contexts       [NUM_REF_NO_CTX];
    BiContextType delta_qp_contexts     [NUM_DELTA_QP_CTX];
    BiContextType l_intra_mode_contexts [NUM_INTRA_MODE_CTX];
    BiContextType c_intra_mode_contexts [NUM_C_INTRA_MODE_CTX];
    BiContextType cbp_contexts          [3][NUM_CBP_CTX];
    BiContextType map_contexts          [NUM_BLOCK_TYPES][NUM_MAP_CTX];
    BiContextType last_contexts         [NUM_BLOCK_TYPES][NUM_LAST_CTX];
    BiContextType split_contexts        [NUM_SPLIT_CTX];
    BiContextType tu_contexts           [NUM_TU_CTX];
    BiContextType lastCG_contexts       [NUM_LAST_CG_CTX];
    BiContextType sigCG_contexts        [NUM_SIGCG_CTX];
    BiContextType lastPos_contexts[NUM_LAST_POS_CTX];
    BiContextType saomergeflag_context     [NUM_SAO_MERGE_FLAG_CTX];
    BiContextType saomode_context          [NUM_SAO_MODE_CTX];
    BiContextType saooffset_context          [NUM_SAO_OFFSET_CTX];
    BiContextType m_cALFLCU_Enable_SCModel    [3][NUM_ALF_LCU_CTX];
    BiContextType brp_contexts          [NUM_BRP_CTX];

    BiContextType pdirMin_contexts      [NUM_INTER_DIR_MIN_CTX];


    BiContextType padding_ctxs[3];  // TODO: FIXME: should be no less than 3, or the decoder would crash.
} SyntaxInfoContexts;




#endif // #ifndef _TYPES_H_

