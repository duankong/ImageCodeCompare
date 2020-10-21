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
* File name: rdopt_coding_state.c
* Function:
Storing/restoring coding state for
Rate-Distortion optimized mode decision
*
*************************************************************************************
*/


#include <stdlib.h>
#include <math.h>
#include <memory.h>

//!EDIT START <added by lzhang AEC
#include "AEC.h"            // added by lzhang
//!EDIT end <added by lzhang AEC


#include "rdopt_coding_state.h"

/*
*************************************************************************
* Function:delete structure for storing coding state
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void delete_coding_state(CSptr cs)
{
    if (cs != NULL) {
        if (cs->bitstream != NULL) {
            free(cs->bitstream);
        }

        //=== coding state structure ===
        if (cs->encenv !=  NULL) {
            free(cs->encenv);
        }

        delete_contexts_SyntaxInfo(cs->syn_ctx);

        free(cs);
        cs = NULL;
    }

}

/*
*************************************************************************
* Function:create structure for storing coding state
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

CSptr create_coding_state()
{
    CSptr cs;

    //=== coding state structure ===
    if ((cs = (CSptr) calloc(1, sizeof(CSobj))) == NULL) {
        no_mem_exit("init_coding_state: cs");
    }

    //=== important variables of data partition array ===

    if ((cs->bitstream = (Bitstream *) calloc(1, sizeof(Bitstream))) == NULL) {
        no_mem_exit("init_coding_state: cs->bitstream");
    }

    //=== important variables of data partition array ===
    cs->no_part = 1;

    if ((cs->encenv = (EncodingEnvironment *) calloc(cs->no_part, sizeof(EncodingEnvironment))) == NULL) {
        no_mem_exit("init_coding_state: cs->encenv");
    }

    cs->syn_ctx = create_contexts_SyntaxInfo();

    return cs;
}

/*
*************************************************************************
* Function:store coding state (for rd-optimized mode decision)
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void store_coding_state(CSptr cs)
{
    int         i;
    Bitstream            *bs_src, *bs_dest;
    EncodingEnvironment  *ee_src, *ee_dest;
    SyntaxInfoContexts   *syn_src;  //  = img->currentSlice->syn_ctx;
    SyntaxInfoContexts   *syn_dest; // = cs->syn_ctx;

    syn_src  = img->currentSlice->syn_ctx;
    syn_dest = cs->syn_ctx;

    for (i = 0; i < 1; i++) {
        bs_src  =  currBitStream;
        bs_dest = cs->bitstream;

        ee_src  = & (img->currentSlice->partArr[i].ee_AEC);
        ee_dest = & (cs->encenv   [i]);

        memcpy(ee_dest, ee_src, sizeof(EncodingEnvironment));

        memcpy(bs_dest, bs_src, sizeof(Bitstream));
    }

    //=== contexts for binary arithmetic coding ===
    memcpy(syn_dest, syn_src, sizeof(SyntaxInfoContexts));
}

/*
*************************************************************************
* Function:restore coding state (for rd-optimized mode decision)
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/

void reset_coding_state(CSptr cs)
{
    int  i;

    EncodingEnvironment  *ee_src, *ee_dest;
    Bitstream            *bs_src, *bs_dest;

    SyntaxInfoContexts   *syn_dest;// = img->currentSlice->syn_ctx;
    SyntaxInfoContexts   *syn_src  ;//= cs->syn_ctx;

    syn_dest = img->currentSlice->syn_ctx;
    syn_src  = cs->syn_ctx;

    //=== important variables of data partition array ===
    //only one partition for IDR img
    //  for (i = 0; i <(img->currentPicture->idr_flag? 1:cs->no_part); i++)
    for (i = 0; i < 1; i++) {
        ee_dest = & (img->currentSlice->partArr[i].ee_AEC);
        ee_src  = & (cs->encenv   [i]);

        bs_dest = currBitStream;

        bs_src  = & (cs->bitstream[i]);

        //--- parameters of encoding environments ---
        memcpy(ee_dest, ee_src, sizeof(EncodingEnvironment));

        memcpy(bs_dest, bs_src, sizeof(Bitstream));
    }


    //=== contexts for binary arithmetic coding ===
    memcpy(syn_dest, syn_src, sizeof(SyntaxInfoContexts));
}

