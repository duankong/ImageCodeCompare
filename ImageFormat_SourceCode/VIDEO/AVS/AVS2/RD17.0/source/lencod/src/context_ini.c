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


#define CONTEXT_INI_C

#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include "../../lcommon/inc/defines.h"
#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "AEC.h"


#define BIARI_CTX_INIT1_LOG(jj,ctx)\
  {\
    for (j=0; j<jj; j++)\
    {\
      biari_init_context_logac(&(ctx[j]));\
    }\
  }

#define BIARI_CTX_INIT2_LOG(ii,jj,ctx)\
  {\
    for (i=0; i<ii; i++)\
      for (j=0; j<jj; j++)\
      {\
        biari_init_context_logac(&(ctx[i][j]));\
      }\
  }

void init_contexts()
{
    SyntaxInfoContexts  *syn = img->currentSlice->syn_ctx;
    int i, j;

    BIARI_CTX_INIT1_LOG(NUM_CuType_CTX,                syn->cuType_contexts);
    BIARI_CTX_INIT1_LOG(NUM_INTER_DIR_CTX,             syn->pdir_contexts);
    BIARI_CTX_INIT1_LOG(NUM_AMP_CTX,                   syn->amp_contexts);
    BIARI_CTX_INIT1_LOG(NUM_B8_TYPE_CTX,               syn->b8_type_contexts);
    BIARI_CTX_INIT1_LOG(NUM_INTER_DIR_DHP_CTX,             syn->pdir_dhp_contexts);
    BIARI_CTX_INIT1_LOG(NUM_B8_TYPE_DHP_CTX,               syn->b8_type_dhp_contexts);
    BIARI_CTX_INIT1_LOG(DIRECTION,                     syn->b_dir_skip_contexts);
    BIARI_CTX_INIT1_LOG(MH_PSKIP_NUM,                  syn->p_skip_mode_contexts);
    BIARI_CTX_INIT1_LOG(WPM_NUM,                         syn->wpm_contexts);
    BIARI_CTX_INIT2_LOG(3,               NUM_MVD_CTX,  syn->mvd_contexts);
    BIARI_CTX_INIT2_LOG(2,               NUM_PMV_IDX_CTX,  syn->pmv_idx_contexts);
    BIARI_CTX_INIT1_LOG(NUM_REF_NO_CTX,                syn->ref_no_contexts);
    BIARI_CTX_INIT1_LOG(NUM_DELTA_QP_CTX,              syn->delta_qp_contexts);
    BIARI_CTX_INIT1_LOG(NUM_INTRA_MODE_CTX,            syn->l_intra_mode_contexts);
    BIARI_CTX_INIT1_LOG(NUM_C_INTRA_MODE_CTX,          syn->c_intra_mode_contexts);
    BIARI_CTX_INIT2_LOG(3,               NUM_CBP_CTX,  syn->cbp_contexts);
    BIARI_CTX_INIT2_LOG(NUM_BLOCK_TYPES, NUM_MAP_CTX,  syn->map_contexts);
    BIARI_CTX_INIT2_LOG(NUM_BLOCK_TYPES, NUM_LAST_CTX, syn->last_contexts);
    BIARI_CTX_INIT1_LOG(NUM_SPLIT_CTX,                 syn->split_contexts);
    BIARI_CTX_INIT1_LOG(NUM_TU_CTX,                    syn->tu_contexts);
    BIARI_CTX_INIT1_LOG(NUM_SIGCG_CTX,                 syn->sigCG_contexts);
    BIARI_CTX_INIT1_LOG(NUM_LAST_CG_CTX,               syn->lastCG_contexts);
    BIARI_CTX_INIT1_LOG(NUM_LAST_POS_CTX,          syn->lastPos_contexts);
    BIARI_CTX_INIT1_LOG(NUM_SAO_MERGE_FLAG_CTX,               syn->saomergeflag_context);
    BIARI_CTX_INIT1_LOG(NUM_SAO_MODE_CTX,               syn->saomode_context);
    BIARI_CTX_INIT1_LOG(NUM_SAO_OFFSET_CTX,               syn->saooffset_context);
    BIARI_CTX_INIT2_LOG(NUM_ALF_COMPONENT, NUM_ALF_LCU_CTX,   syn->m_cALFLCU_Enable_SCModel);
    BIARI_CTX_INIT1_LOG(NUM_BRP_CTX,                  syn->brp_contexts);

    BIARI_CTX_INIT1_LOG(NUM_INTER_DIR_MIN_CTX,         syn->pdirMin_contexts);
}

