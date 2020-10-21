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

#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include <stdio.h>

#include "../../lcommon/inc/defines.h"
#include "../../lcommon/inc/commonStructures.h"
#include "../../lcommon/inc/commonVariables.h"

void bg_allocModel(byte *frame, int w, int h);
void bg_build(byte *imageData);
void bg_insert(byte *frame);
void bg_releaseModel();

void write_GB_frame(FILE *p_dec);
void write_GB_org_frame(FILE *p_dec);


#if !FIX_MAX_REF
#define MAXREF 4
#define MAXGOP 32
#endif

typedef struct reference_management {
    int type;
    int poc;
    int qp_offset;
    int num_of_ref;
    int referd_by_others;
    int ref_pic[MAXREF];
    int predict;
    int deltaRPS;
    int num_to_remove;
    int remove_pic[MAXREF];
#if M3480_TEMPORAL_SCALABLE
    int temporal_id;
#endif
} ref_man;

typedef struct {
    int num_skipmode;
    int dmh_mode;
    int dir;
} MCParam;
//! Bitstream
typedef struct {
    int                 byte_pos;           //!< current position in bitstream;
    int                 bits_to_go;         //!< current bitcounter
    unsigned char                byte_buf;           //!< current buffer for last written byte
    unsigned char                *streamBuffer;      //!< actual buffer for written bytes
    int                 write_flag;         //!< Bitstream contains data and needs to be written
} Bitstream;
Bitstream *currBitStream;
Bitstream *currBitStream_ALF;

//!< statistics
typedef struct {
    int   quant0;                 //!< quant for the first frame
    int   quant1;                 //!< average quant for the remaining frames
    double bitr;                   //!< bit rate for current frame, used only for output til terminal
    double bitr0;                  //!< stored bit rate for the first frame
    double bitrate;                //!< average bit rate for the sequence except first frame
    long long int   bit_ctr;                //!< counter for bit usage
    long long int   bit_ctr_0;              //!< stored bit use for the first frame
    long long int   bit_ctr_n;              //!< bit usage for the current frame
    long long int   bit_slice;              //!< number of bits in current slice
    long long int   bit_ctr_dup_gb;
    int   bit_use_mode_inter[2][MAXMODE]; //!< statistics of bit usage
    int   bit_ctr_emulationprevention; //!< stored bits needed to prevent start code emulation
    int   mode_use_intra[25];     //!< codingUnit mode usage for Intra frames
    int   mode_use_inter[2][MAXMODE];

    int   mb_use_mode[2];/*lgp*/

    // B pictures
    int   *mode_use_Bframe;
    int   *bit_use_mode_Bframe;
    long long int   bit_ctr_P;
    long long int   bit_ctr_B;
    double bitrate_P;
    double bitrate_B;

#define NUM_PIC_TYPE 5
    int   bit_use_stuffingBits[NUM_PIC_TYPE];
    int   bit_use_cuType[NUM_PIC_TYPE];
    int   bit_use_header[NUM_PIC_TYPE];
    int   tmp_bit_use_cbp[NUM_PIC_TYPE];
    int   bit_use_coeffY[NUM_PIC_TYPE];
    int   bit_use_coeffC[NUM_PIC_TYPE];
    int   bit_use_delta_quant[NUM_PIC_TYPE];

} StatParameters;
extern StatParameters *stat;


/* ------------------------------------------------------
* enc data
*/
typedef struct {
#if TDRDO
    byte    *imgY_pre_buffer;
    int      globenumber;
#endif
    int      best_pmvidx_ref[10];

    byte **background_frame[3];
    byte **background_frame_quarter;

    int background_reference_enable;
    int background_output_flag;
    int last_background_frame_number;
    int intra_temp;
    int g_bg_number;
    int last_input_frame;
    int train_start;
    int background_number;
    int total_bg_number;
    int background_output_flag_before;


    int duplicated_gb_flag; //flags the duplicated GB before S in Random Access. only appears in encoder in order to avoid influences on background modeling.
    int gb_is_ready; // the QP for G is QPI, for GB is QPI-9; this flag is to distinguish the duplicated GB coming from G or GB.

    int background_pred_flag;


#if AVS2_S2_FASTMODEDECISION
    int bg_flag;
    int sum_diff_8x8, sum_diff_16x16, sum_diff_32x32, sum_diff_64x64;
#endif

    int subGopID;
    int subGopNum;
#if M3480_TEMPORAL_SCALABLE
    int temporal_id_exist_flag;
#endif

    int picture_reorder_delay;
    int gop_size_all;
    int gop_size;
#if M3480_TEMPORAL_SCALABLE
    int cur_layer;
#endif
    ref_man cfg_ref[MAXGOP];
    ref_man cfg_ref_all[MAXGOP];
    ref_man curr_RPS;

    int trtmp;
    int last_output;
    int curr_IDRcoi;
    int curr_IDRtr;
    int next_IDRtr;
    int next_IDRcoi;
    int use_RPSflag;
    int flag_gop;

    double lambda_mode;

    int bypass_all_dmh;

    byte   *imgY_org_buffer;           //!< Reference luma image
    byte   *imgY_GB_org_buffer;         // for duplicated GB
    // global picture format dependend buffers, mem allocation in image.c
    byte  **imgY_org;           //!< Reference luma image
    byte ** *imgUV_org;         //!< Reference croma image


    int   **img4Y_tmp;              //!< for quarter pel interpolation
#if FIX_MAX_REF
    byte  **integerRefUV[MAXREF][2];     //!< pix chroma

    pel_t  *Refbuf11[MAXREF];             //!< 1/1th pel (full pel) reference frame buffer
#else
    byte  **integerRefUV[4][2];     //!< pix chroma

    pel_t  *Refbuf11[4];             //!< 1/1th pel (full pel) reference frame buffer
#endif

    int ** ** *all_mincost;  //store the MV and SAD information needed;
    int ** ** *all_bwmincost; //store for backward prediction


    int    intras;            //!< Counts the intra updates in each frame.

    int    slice_header[3];
    double global_lambda;
    int    g_block8x8;
    int    g_intra_mode;

    int *ACLevel_RDOQ;
    int *ACRun_RDOQ;

#if MB_DQP
    int   lastQP;   // for previous prediction
    FILE *testQP;
    int   cof_flag;

    int zeroDqpCount;
    int DqpCount;
#endif

    //qhg for demulation
    Bitstream *tmpStream;
    int        current_slice_bytepos;
    int        current_slice_bytepos_alf;   // store the start pos for de-emulation when alf is on
    //qhg 20060327 for de-emulation

    int  AEC_writting;

    // Adaptive frequency weighting quantization
#if FREQUENCY_WEIGHTING_QUANTIZATION
    int  mb_wq_mode;                     //M2331 2008-04
#if COUNT_BIT_OVERHEAD
    int  g_count_overhead_bit;
#endif
#endif

    int alllevel[1024];
    int allrun[1024];

    // files
    //FILE *p_dec,*p_dec_u,*p_dec_v;   //!< internal decoded image for debugging
    FILE *p_stat;                    //!< status file for the last encoding session
    FILE *p_dec; //!< internal decoded image for debugging
    FILE *p_in;                      //!< YUV
    FILE *p_dec_background; //background ref frame
    FILE *p_org_background; //org background frame


} Video_Enc_data;
extern Video_Enc_data *he;

//! struct to characterize the state of the arithmetic coding engine
typedef struct {
    unsigned int  Elow;
    unsigned int  E_s1;
    unsigned int  E_t1;
    unsigned char  Ebuffer;
    unsigned int  Ebits_to_go;
    unsigned int  Ebits_to_follow;
    unsigned char          *Ecodestrm;
    int           *Ecodestrm_len;
    //  int           *Ecodestrm_laststartcode;
    // storage in case of recode MB
    unsigned int  ElowS, ErangeS;
    unsigned int  EbufferS;
    unsigned int  Ebits_to_goS;
    unsigned int  Ebits_to_followS;
    unsigned char          *EcodestrmS;
    int           *Ecodestrm_lenS;
    int           C, CS;
    int           E, ES;
    int           B, BS;
} EncodingEnvironment;
typedef EncodingEnvironment *EncodingEnvironmentPtr;
#define MAXSLICEPERPICTURE 256

extern CopyRight *cp;

//float singlefactor;
extern InputParameters *input;

int  sign(int a, int b);
void init_img();
void report();
void information_init();

int   LumaResidualCoding8x8_TRANS(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *,
                                  int *, int, int, int, int, int, int **);
#if Mv_Rang
void LumaResidualCoding8x8_PRED(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cbp,
                                int *cbp_blk, int block8x8, int mode, int fw_mode, int bw_mode, int fw_refframe, int bw_refframe, MCParam *pMCParam,
                                int **curr_blk);
#else
void LumaResidualCoding8x8_PRED(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *cbp,
                                int *cbp_blk, int block8x8, int fw_mode, int bw_mode, int fw_refframe, int bw_refframe, MCParam *pMCParam,
                                int **curr_blk);
#endif
pel_t *FastLineX(int, pel_t *, int, int);
pel_t *UMVLineX(int, pel_t *, int, int);

#if Mv_Rang
    void LumaResidualCoding(codingUnit *, unsigned int, unsigned int, MCParam *, int);
#else
    void LumaResidualCoding(codingUnit *, unsigned int, unsigned int, MCParam *);
#endif
void ChromaResidualCoding(codingUnit *, unsigned int, unsigned int, int *, MCParam *);
void IntraChromaPrediction8x8(codingUnit *, unsigned int, unsigned int, int *, int *, int *);

int  writeMBHeader(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int rdopt);

void  PartitionMotionSearch(int , int , double , int , int);
void  PartitionMotionSearch_sym(int, int, double, int, int);
void  PartitionMotionSearch_dual(int, int, double, int, int);

#if Mv_Rang
int   LumaResidualCoding8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *, int *,
                            int, int, int, int, int, int, MCParam *pMCParam);
#else
int   LumaResidualCoding8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int *, int *,
                            int, int, int, int, int, MCParam *pMCParam);
#endif

int   writeLumaCoeff8x8(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                        unsigned int uiPixInSMB_y, int block8x8, int intra8x8mode, int uiPosition);
int   writeMotionVector8x8(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int, int, int,
                           int, int, int, int, int);
int writeMotionVector8x8_sym(int  i0,  int  j0,  int  i1, int  j1, int  refframe, int  dmv_flag, int  fwd_flag,
                             int  mv_mode, int  pdir, codingUnit *currMB, int  uiPositionInPic);

void  FindSkipModeMotionVector(unsigned int uiBitSize, unsigned int uiPositionInPic);

void FindSkipModeMotionVectorInCCD(unsigned int uiBitSize, unsigned int uiPositionInPic);
// dynamic mem allocation
int  init_global_buffers();
void free_global_buffers();
void no_mem_exit(char *where);

int  get_mem_mv(int ** ** **);
void free_mem_mv(int ** ** *);
void free_img();

int  writeBlockCoeff(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                     unsigned int uiPixInSMB_y, int block8x8, int uiPositionInPic);  //qyu 0821 transform size
void intrapred_luma_AVS(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int img_x,
                        int img_y);
int  storeMotionInfo(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int pos,
                     int dmh_mode);
double encode_one_SMB(unsigned int uiBitSize, unsigned int uiPositionInPic, double split_rd_cost);

void  start_codingUnit();
void  set_MB_parameters(int mb);            //! sets up img-> according to input-> and currSlice->
void  terminate_codingUnit(Boolean *end_of_picture, int lucIndex);

void  Init_Curr_codingUnit(codingUnit *tmpMB, unsigned int smbSize, unsigned int positionint);    //,int  **ipredmodes);
void  write_one_SMB(unsigned int uiBitSize, unsigned int uiPositionInPic, int eos_bit);

void  proceed2nextcodingUnit();

void  error(char *text, int code);
int   start_sequence();
int   terminate_sequence();
int   start_slice();
void demulateFunction();
void BitStreamCopy(Bitstream *BitStream_dst, Bitstream *BitStream_src);

//============= rate-distortion optimization ===================
void  clear_rdopt();
void  init_rdopt();

void  SetImgType();

extern int   *refbits;
extern int ** *motion_cost;
extern int ** *motion_cost_sym;

extern int ** *motion_cost_dual;

void  Get_direct(unsigned int uiBitSize, unsigned int uiPositionInPic);
void  ForwardPred(int *fw_mcost, int *best_fw_ref, int *best_bw_ref, int max_ref, int adjust_ref, int mode, int block,
                  double lambda_motion, int write_ref, int lambda_motion_factor, int mb_nr, int bit_size);

void BiPred(int *best_bw_ref, int *bw_mcost, int *sym_mcost, int *sym_best_fw_ref, int *sym_best_bw_ref,
            int *bid_mcost, int *bid_best_fw_ref, int *bid_best_bw_ref, int mode, int block, double lambda_motion, int max_ref,
            int adjust_ref , int mb_nr, int bit_size);
void DualPred(int *dual_mcost, int *dual_best_fst_ref, int *dual_best_snd_ref, int max_ref, int adjust_ref, int mode,
              int block, double lambda_motion, int write_ref, int lambda_motion_factor, int mb_nr, int bit_size);

void  FreeBitstream_ALF();
void  AllocateBitstream_ALF();
void  FreeBitstream();
void  AllocateBitstream();
void write_frame(FILE *p_out, int pos);//yuquanhe@hisilicon.com
//! Syntax Element
struct syntaxelement {
    int                 type;           //!< type of syntax element for data part.
    int                 value1;         //!< numerical value of syntax element
    int                 value2;         //!< for blocked symbols, e.g. run/level
    int                 len;            //!< length of code
    int                 inf;            //!< info part of UVLC code
    unsigned int        bitpattern;     //!< UVLC bitpattern
    int                 context;        //!< AEC context
    int                 k;              //!< AEC context for coeff_count,uv
    int                 golomb_grad;    //needed if type is a golomb element (AVS)
    int                 golomb_maxlevels; // if this is zero, do not use the golomb coding. (AVS)

#if TRACE
#define               TRACESTRING_SIZE 100            //!< size of trace string
    char                tracestring[TRACESTRING_SIZE];  //!< trace string
#endif

    //!< for mapping of syntaxElement to UVLC
    void (*mapping)(int value1, int value2, int *len_ptr, int *info_ptr);

    //!< edit start for mapping of syntaxElement to UVLC
    void (*writing)(codingUnit *currMB, struct syntaxelement *, EncodingEnvironmentPtr,
                    int uiPosition);  //added by lzhang

    //!< edit end for mapping of syntaxElement to UVLC

};

//! DataPartition
typedef struct datapartition {

    Bitstream           *bitstream;
    EncodingEnvironment ee_AEC;
    int (*writeSyntaxElement)(codingUnit *currMB, SyntaxElement *, struct datapartition *, int uiPositionInPic);

} DataPartition;
//! Slice
struct slice {
    int                 qp;

    int                 start_mb_nr;
    //  int                 num_mb;       //!< number of MBs in the slice
    DataPartition       *partArr;     //!< array of partitions
    SyntaxInfoContexts  *syn_ctx;     //!< pointer to struct of context models for use in AEC
};
struct alfdatapart {
    Bitstream           *bitstream;
    EncodingEnvironment ee_AEC;
    SyntaxInfoContexts  *syn_ctx;     //!< pointer to struct of context models for use in AEC
};
typedef struct {
    int   no_slices;
    int   bits_per_picture;
    double distortion_y;
    double distortion_u;
    double distortion_v;
    //!EDIT START <added by lzhang AEC
    Slice *slices[MAXSLICEPERPICTURE];       //added by lzhang
    //!EDIT end <added by lzhang AEC
} Picture;

Picture *frame_pic;

Picture *malloc_picture();
#if TRACE
void  trace2out(SyntaxElement *se);
#endif

typedef struct {
    Bitstream            *bitstream;
    int                   currSEnr;
    int                   bitcounter[MAX_BITCOUNTER_MB];
    SyntaxInfoContexts   *syn_ctx;
    EncodingEnvironment  *encenv;
    int         no_part;
} CSobj;
typedef CSobj *CSptr;


// Added for (re-) structuring the RM09 soft
void free_picture(Picture *pic);
int  encode_one_slice(Picture *pic);    //! returns the number of MBs in the slice
void free_slice_list(Picture *currPic);

void AllocatetmpBitstream();
void FreetmpBitstream();
void Demulate(Bitstream *currStream, int current_slice_bytepos);
void SetModesAndRefframeForBlocks(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int mode,
                                  MCParam *pMCParam);
void SetModesAndRefframe(codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPositionInPic, int b8,
                         int *fw_mode, int *bw_mode, int *fw_ref, int *bw_ref);
int  writeLumaCoeff8x8_AEC(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                           unsigned int uiPixInSMB_y,  int intra8x8mode, int uiPositionInPic) ;
int  writeChromaCoeff8x8_AEC(int **Quant_Coeff, codingUnit *currMB, unsigned int uiBitSize, unsigned int uiPixInSMB_x,
                             unsigned int uiPixInSMB_y, int b8 , int uiPositionInPic);

int  terminate_slice();

void free_slice();

#define MAX_REGION_NUM  100

typedef struct {
    int pos[4];
    int slice_set_index;
    int sliceqp;
} SliceSetRegion;

SliceSetRegion *sliceregion;



#endif


