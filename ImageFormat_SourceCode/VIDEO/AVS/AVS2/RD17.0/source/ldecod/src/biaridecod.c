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

#include <stdlib.h>
#include "math.h"
#include "global.h"
#include "../../lcommon/inc/commonVariables.h"
#include "../../lcommon/inc/memalloc.h"
#include <assert.h>



//AC ENGINE PARAMETERS
unsigned int t1, value_t;
unsigned char s1, value_s;
unsigned char cFlag;

unsigned char value_s_bound = NUN_VALUE_BOUND;
unsigned char is_value_bound;
unsigned int max_value_s = 0;
unsigned char is_value_domain;//  is value in R domain 1 is R domain 0 is LG domain
extern int symbolCount;

int binCount = 0;

#define Dbuffer         (dep->Dbuffer)
#define Dbits_to_go     (dep->Dbits_to_go)
#define Dcodestrm       (dep->Dcodestrm)
#define Dcodestrm_len   (dep->Dcodestrm_len)

#define B_BITS  10

#define LG_PMPS_SHIFTNO 2

#define HALF      (1 << (B_BITS-1))
#define QUARTER   (1 << (B_BITS-2))


/************************************************************************
* M a c r o s
************************************************************************
*/

// yin hai cang
#define get_byte(){                                         \
    Dbuffer = Dcodestrm[(*Dcodestrm_len)++];\
    Dbits_to_go = 7;                        \
  }
// yin hai cang


/************************************************************************
************************************************************************
init / exit decoder
************************************************************************
************************************************************************/


/*!
************************************************************************
* \brief
*    Allocates memory for the DecodingEnvironment struct
* \return DecodingContextPtr
*    allocates memory
************************************************************************
*/

/*!
************************************************************************
* \brief
*    Initializes the DecodingEnvironment for the arithmetic coder
************************************************************************
*/


void arideco_start_decoding(DecodingEnvironmentPtr dep, unsigned char *cpixcode, int firstbyte, int *cpixcode_len,
                            int slice_type)
{
    Dcodestrm = cpixcode;
    Dcodestrm_len = cpixcode_len;
    *Dcodestrm_len = firstbyte;

    s1 = 0;
    t1 = QUARTER - 1; //0xff
    value_s = 0;

    value_t = 0;

    {
        int i;
        Dbits_to_go = 0;

        for (i = 0; i < B_BITS - 1 ; i++) {
            if (--Dbits_to_go < 0) {
                get_byte();
            }

            value_t = (value_t << 1)  | ((Dbuffer >> Dbits_to_go) & 0x01);
        }
    }
    is_value_domain = 1;
    cFlag = 1;
}



/*!
************************************************************************
* \brief
*    arideco_bits_read
************************************************************************
*/
int arideco_bits_read(DecodingEnvironmentPtr dep)
{
    return 8 * ((*Dcodestrm_len) - 1) + (8 - Dbits_to_go);
}

void update_ctx(DecodingEnvironmentPtr dep, BiContextTypePtr bi_ct, int is_LPS)
{
    register unsigned char cwr, cycno = bi_ct->cycno;
    register unsigned int lg_pmps = bi_ct->LG_PMPS;


    cwr = (cycno <= 1) ? 3 : (cycno == 2) ? 4 : 5;     //FAST ADAPTION PARAMETER
    //update other parameters
    if (is_LPS) {
        cycno = (cycno <= 2) ? (cycno + 1) : 3;
    } else if (cycno == 0) {
        cycno = 1;
    }

    bi_ct->cycno = cycno;

    //update probability estimation
    if (is_LPS) {
        switch (cwr) {
        case 3:
            lg_pmps = lg_pmps + 197;
            break;
        case 4:
            lg_pmps = lg_pmps + 95;
            break;
        default:
            lg_pmps = lg_pmps + 46;
        }

        if (lg_pmps >= (256 << LG_PMPS_SHIFTNO)) {
            lg_pmps = (512 << LG_PMPS_SHIFTNO) - 1 - lg_pmps;
            bi_ct->MPS = !(bi_ct->MPS);
        }
    } else {
        lg_pmps = lg_pmps - (unsigned int)(lg_pmps >> cwr) - (unsigned int)(lg_pmps >> (cwr + 2));
    }

    bi_ct->LG_PMPS = lg_pmps;
}


unsigned int biari_decode_symbol(DecodingEnvironmentPtr dep, BiContextTypePtr bi_ct)
{
    register unsigned char bit ;
    register unsigned char s_flag, is_LPS = 0;
    register unsigned int lg_pmps = bi_ct->LG_PMPS >> LG_PMPS_SHIFTNO;
    register unsigned int t_rlps;
    register unsigned int t2;
    register unsigned char s2;


    bit = bi_ct->MPS;

    if (is_value_domain == 1 || (s1 == value_s_bound &&
                                 is_value_bound == 1)) {  //value_t is in R domain  s1=0 or s1 == value_s_bound
        s1 = 0;
        value_s = 0;

        while (value_t < QUARTER && value_s < value_s_bound) {
            int j;
            if (--Dbits_to_go < 0) {
                get_byte();
            }
            j = (Dbuffer >> Dbits_to_go) & 0x01;
            // Shift in next bit and add to value

            value_t = (value_t << 1) | j;
            value_s++;
        }
        if (value_t < QUARTER) {
            is_value_bound = 1;
        } else {
            is_value_bound = 0;
        }

        value_t = value_t & 0xff;
    }

    if (t1 >=  lg_pmps) {
        s2 = s1;
        t2 = t1 -  lg_pmps ; //8bits
        s_flag = 0;
    } else {
        s2 = s1 + 1;
        t2 = 256 + t1 - lg_pmps ; //8bits
        s_flag = 1;
    }

    if (value_s > value_s_bound) {
        printf("value_s:%d\n", value_s);
        exit(1);
    }
    if (value_s > max_value_s) {
        max_value_s = value_s;
    }

    if ((s2 > value_s || (s2 == value_s && value_t >= t2)) && is_value_bound == 0) {     //LPS
        is_LPS = 1;
        bit = !bit; //LPS
        is_value_domain = 1;
        t_rlps = (s_flag == 0) ? (lg_pmps)
                 : (t1 +  lg_pmps);

        if (s2 == value_s) {
            value_t = (value_t - t2);
        } else {
            if (--Dbits_to_go < 0) {
                get_byte();
            }

            // Shift in next bit and add to value
            value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);
            value_t = 256 + value_t - t2;
        }

        //restore range
        while (t_rlps < QUARTER) {
            t_rlps = t_rlps << 1;

            if (--Dbits_to_go < 0) {
                get_byte();
            }

            // Shift in next bit and add to value
            value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);
        }

        s1 = 0;
        t1 = t_rlps & 0xff;

    } else { //MPS
        s1 = s2;
        t1 = t2;
        is_value_domain = 0;
    }
    if (cFlag) {
        update_ctx(dep, bi_ct, is_LPS);
    }

    return (bit);
}

unsigned int biari_decode_symbolW(DecodingEnvironmentPtr dep, BiContextTypePtr bi_ct1 , BiContextTypePtr bi_ct2)
{
    register unsigned char bit1, bit2;
    register unsigned char pred_MPS, bit;
    register unsigned int  lg_pmps;
    register unsigned char cwr1, cycno1 = bi_ct1->cycno;
    register unsigned char cwr2, cycno2 = bi_ct2->cycno;
    register unsigned int lg_pmps1 = bi_ct1->LG_PMPS, lg_pmps2 = bi_ct2->LG_PMPS;
    register unsigned int t_rlps;
    register unsigned char s_flag;
    register unsigned int t2;
    register unsigned char s2, is_LPS;

    bit1 = bi_ct1->MPS;
    bit2 = bi_ct2->MPS;


    cwr1 = (cycno1 <= 1) ? 3 : (cycno1 == 2) ? 4 : 5;
    cwr2 = (cycno2 <= 1) ? 3 : (cycno2 == 2) ? 4 : 5;

    if (bit1 == bit2) {
        pred_MPS = bit1;
        lg_pmps = (lg_pmps1 + lg_pmps2) / 2;
    } else {
        if (lg_pmps1 < lg_pmps2) {
            pred_MPS = bit1;
            lg_pmps = (256 << LG_PMPS_SHIFTNO) - 1 - ((lg_pmps2 - lg_pmps1) >> 1);
        } else {
            pred_MPS = bit2;
            lg_pmps = (256 << LG_PMPS_SHIFTNO) - 1 - ((lg_pmps1 - lg_pmps2) >> 1);
        }
    }

    if (t1 >= (lg_pmps >> LG_PMPS_SHIFTNO)) {
        s2 = s1;
        t2 = t1 - (lg_pmps >> LG_PMPS_SHIFTNO);
        s_flag = 0;
    } else {
        s2 = s1 + 1;
        t2 = 256 + t1 - (lg_pmps >> LG_PMPS_SHIFTNO);
        s_flag = 1;
    }

    bit = pred_MPS;

    if (value_s > value_s_bound) {
        printf("value_s:%d\n", value_s);
        exit(1);
    }
    if (value_s > max_value_s) {
        max_value_s = value_s;
    }

    if ((s2 > value_s || (s2 == value_s && value_t >= t2)) && is_value_bound == 0) {     //LPS
        is_LPS = 1;
        bit = !bit; //LPS
        t_rlps = (s_flag == 0) ? (lg_pmps >> LG_PMPS_SHIFTNO) : (t1 + (lg_pmps >> LG_PMPS_SHIFTNO));

        if (s2 == value_s) {
            value_t = (value_t - t2);
        } else {
            if (--Dbits_to_go < 0) {
                get_byte();
            }

            // Shift in next bit and add to value
            value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);
            value_t = 256 + value_t - t2;
        }

        //restore range
        while (t_rlps < QUARTER) {
            t_rlps = t_rlps << 1;

            if (--Dbits_to_go < 0) {
                get_byte();
            }

            // Shift in next bit and add to value
            value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);
        }

        s1 = 0;
        t1 = t_rlps & 0xff;

        //restore value
        value_s = 0;
    }//--LPS
    else { //MPS
        s1 = s2;
        t1 = t2;
    }
    if (is_LPS == 1 || (s1 == value_s_bound && is_value_bound == 1 && is_LPS == 0)) {
        s1 = 0;
        value_s = 0;
        while (value_t < QUARTER && value_s < value_s_bound) {
            int j;
            if (--Dbits_to_go < 0) {
                get_byte();
            }
            j = (Dbuffer >> Dbits_to_go) & 0x01;
            // Shift in next bit and add to value

            value_t = (value_t << 1) | j;
            value_s++;
        }
        if (value_t < QUARTER) {
            is_value_bound = 1;
        } else {
            is_value_bound = 0;
        }
        value_t = value_t & 0xff;
    }

    if (bit != bit1) {
        cycno1 = (cycno1 <= 2) ? (cycno1 + 1) : 3;     //LPS occurs
    } else {
        if (cycno1 == 0) {
            cycno1 = 1;
        }
    }

    if (bit != bit2) {
        cycno2 = (cycno2 <= 2) ? (cycno2 + 1) : 3;     //LPS occurs
    } else {
        if (cycno2 == 0) {
            cycno2 = 1;
        }
    }

    bi_ct1->cycno = cycno1;
    bi_ct2->cycno = cycno2;

    //update probability estimation
    {
        //bi_ct1
        if (bit == bit1) {
            lg_pmps1 = lg_pmps1 - (unsigned int)(lg_pmps1 >> cwr1) - (unsigned int)(lg_pmps1 >> (cwr1 + 2));
        } else {
            switch (cwr1) {
            case 3:
                lg_pmps1 = lg_pmps1 + 197;
                break;
            case 4:
                lg_pmps1 = lg_pmps1 + 95;
                break;
            default:
                lg_pmps1 = lg_pmps1 + 46;
            }

            if (lg_pmps1 >= (256 << LG_PMPS_SHIFTNO)) {
                lg_pmps1 = (512 << LG_PMPS_SHIFTNO) - 1 - lg_pmps1;
                bi_ct1->MPS = !(bi_ct1->MPS);
            }
        }

        bi_ct1->LG_PMPS = lg_pmps1;

        //bi_ct2
        if (bit == bit2) {
            lg_pmps2 = lg_pmps2 - (unsigned int)(lg_pmps2 >> cwr2) - (unsigned int)(lg_pmps2 >> (cwr2 + 2));
        } else {
            switch (cwr2) {
            case 3:
                lg_pmps2 = lg_pmps2 + 197;
                break;
            case 4:
                lg_pmps2 = lg_pmps2 + 95;
                break;
            default:
                lg_pmps2 = lg_pmps2 + 46;
            }

            if (lg_pmps2 >= (256 << LG_PMPS_SHIFTNO)) {
                lg_pmps2 = (512 << LG_PMPS_SHIFTNO) - 1 - lg_pmps2;
                bi_ct2->MPS = !(bi_ct2->MPS);
            }
        }

        bi_ct2->LG_PMPS = lg_pmps2;
    }

    return (bit);
}

/*!
************************************************************************
* \brief
*    biari_decode_symbol_eq_prob():
* \return
*    the decoded symbol
************************************************************************
*/
unsigned int biari_decode_symbol_eq_prob(DecodingEnvironmentPtr dep)
{
    unsigned char bit;
    BiContextType octx;
    BiContextTypePtr ctx = &octx;
#if Decoder_Bypass_Annex
    register unsigned int t2;
    register unsigned char s2;
    bit = 0;
    if (is_value_domain || (s1 == value_s_bound && is_value_bound == 1)) { //s2==1 t2==t1 s2=value_s+1 or s2==value_s
        s1 = 0; // value_t is in R domain
        is_value_domain = 1;
        if (--Dbits_to_go < 0) {
            get_byte();
        }
        value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);      ////R
        if (value_t >= (256 + t1)) { //LPS
            bit = !bit; //LPS
            value_t -= (256 + t1); //R
        } else {
            bit = 0;
        }
    } else {
        s2 = s1 + 1;
        t2 =  t1 ; //8bits
        if (s2 > value_s || (s2 == value_s && value_t >= t2) && is_value_bound == 0) {    //LPS
            bit = !bit; //LPS

            if (s2 == value_s) {
                value_t = (value_t - t2);
            } else {
                if (--Dbits_to_go < 0) {
                    get_byte();
                }
                value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);      ////R
                value_t = 256 + value_t - t2;
            }

            is_value_domain = 1;
            t1 = t1;
        } else {
            s1 = s2;
            t1 = t2;
            is_value_domain = 0;
        }
    }
#else
    ctx->LG_PMPS = (QUARTER << LG_PMPS_SHIFTNO);
    ctx->MPS = 0;
    cFlag = 0;
    bit = biari_decode_symbol(dep, ctx);
#endif
    cFlag = 1;
    return (bit);
}

unsigned int biari_decode_final(DecodingEnvironmentPtr dep)
{
    unsigned char bit;
    BiContextType octx;
    BiContextTypePtr ctx = &octx;
#if Decoder_Final_Annex
    register unsigned char s_flag, is_LPS = 0;
    register unsigned int t_rlps, rang;
    register unsigned int t2;
    register unsigned char s2;
    bit = 0;
    if (is_value_domain == 1 || (s1 == value_s_bound &&
                                 is_value_bound == 1)) {  //value_t is in R domain  s1=0 or s1 == value_s_bound
        s1 = 0;
        value_s = 0;

        while (value_t < QUARTER && value_s < value_s_bound) {
            int j;
            if (--Dbits_to_go < 0) {
                get_byte();
            }
            j = (Dbuffer >> Dbits_to_go) & 0x01;
            // Shift in next bit and add to value

            value_t = (value_t << 1) | j;
            value_s++;
        }
        if (value_t < QUARTER) {
            is_value_bound = 1;
        } else {
            is_value_bound = 0;
        }

        value_t = value_t & 0xff;
    }
    if (t1) {
        s2 = s1;
        t2 = t1 - 1;
    } else {
        s2 = s1 + 1;
        t2 = 255;
    }
    if (s2 > value_s || (s2 == value_s && value_t >= t2)  && is_value_bound == 0) {    //LPS
        is_LPS = 1;
        bit = !bit; //LPS

        if (s2 == value_s) {
            value_t = (value_t - t2);
        } else {
            if (--Dbits_to_go < 0) {
                get_byte();
            }
            value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);      ////R
            value_t = 256 + value_t - t2;
        }

        // valueT = (valueT << 8 ) | read_bits(8)
        t_rlps = 1;
        while (t_rlps < QUARTER) {   //for 8 times
            t_rlps = t_rlps << 1;

            if (--Dbits_to_go < 0) {
                get_byte();
            }

            // Shift in next bit and add to value
            value_t = (value_t << 1) | ((Dbuffer >> Dbits_to_go) & 0x01);
        }

        s1 = 0;
        t1 = 0;
        is_value_domain = 1;

    } else {
        s1 = s2;
        t1 = t2;
        is_value_domain = 0;
    }
#else
    ctx->LG_PMPS = 1 << LG_PMPS_SHIFTNO;
    ctx->MPS = 0;
    cFlag = 0;
    bit = biari_decode_symbol(dep, ctx);
#endif
    cFlag = 1;
    return (bit);
}

void biari_init_context_logac(BiContextTypePtr ctx)
{
    ctx->LG_PMPS = (QUARTER << LG_PMPS_SHIFTNO) - 1;   //10 bits precision
    ctx->MPS   = 0;
    ctx->cycno  =  0;
}























