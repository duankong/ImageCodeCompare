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

//#include <math.h>
//#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "inter-prediction.h"
#include "block_info.h"
/////////////////////////////////////////////////////////////////////////////
/// local function declaration
/////////////////////////////////////////////////////////////////////////////

#if !(HALF_PIXEL_COMPENSATION || HALF_PIXEL_CHROMA)
static int calculate_distance(int blkref, int fw_bw);
#endif





/////////////////////////////////////////////////////////////////////////////
///function definition
/////////////////////////////////////////////////////////////////////////////
/*
******************************************************************************
*  Function: calculated field or frame distance between current field(frame)
*            and the reference field(frame).
*     Input:
*    Output:
*    Return:
* Attention:
*    Author: Yulj 2004.07.14
******************************************************************************
*/
int calculate_distance(int blkref, int fw_bw)    //fw_bw>=: forward prediction.
{
    int distance = 1;

    //if ( img->picture_structure == 1 ) // frame
    {
        if ((img->type == F_IMG) || (img->type == INTER_IMG)) {
#if FIX_MAX_REF
            distance = hc->picture_distance * 2 - fref[blkref]->imgtr_fwRefDistance * 2;
#else
            if (blkref == 0) {
                distance = hc->picture_distance * 2 - fref[0]->imgtr_fwRefDistance * 2;  // Tsinghua 200701
            } else if (blkref == 1) {
                distance = hc->picture_distance * 2 - fref[1]->imgtr_fwRefDistance * 2;  // Tsinghua 200701
            } else if (blkref == 2) {
                distance = hc->picture_distance * 2 - fref[2]->imgtr_fwRefDistance * 2;
            } else if (blkref == 3) {
                distance = hc->picture_distance * 2 - fref[3]->imgtr_fwRefDistance * 2;
            } else {
                assert(0);    //only two reference pictures for P frame
            }
#endif
        } else { //B_IMG
            if (fw_bw >= 0) {   //forward
                distance = hc->picture_distance * 2 - fref[0]->imgtr_fwRefDistance * 2;  // Tsinghua 200701
            } else {
                distance = img->imgtr_next_P * 2 - hc->picture_distance * 2;  // Tsinghua 200701
            }
        }
    }

    distance = (distance + 512) % 512;   // Added by Xiaozhen ZHENG, 20070413, HiSilicon
    return distance;
}
/*Lou 1016 End*/
/*Lou 1016 Start*/
//The unit of time distance is calculated by field time
/*
*************************************************************************
* Function:
* Input:
* Output:
* Return:
* Attention:
*************************************************************************
*/
#if Mv_Clip
int scale_motion_vector(int motion_vector, int currblkref, int neighbourblkref,
	int ref,int delta2)  //qyu 0820 modified , int currsmbtype, int neighboursmbtype, int block_y_pos, int curr_block_y, int ref, int direct_mv)
#else
int scale_motion_vector(int motion_vector, int currblkref, int neighbourblkref,
                        int ref)  //qyu 0820 modified , int currsmbtype, int neighboursmbtype, int block_y_pos, int curr_block_y, int ref, int direct_mv)
#endif
{
    int sign = (motion_vector > 0 ? 1 : -1);
    int mult_distance;
    int devide_distance;

    motion_vector = abs(motion_vector);

#if REMOVE_UNUSED
	if (motion_vector == 0) {
		return delta2;
	}
#else
    if (motion_vector == 0) {
        return 0;
    }
#endif

    mult_distance = calculate_distance(currblkref, ref);
    devide_distance = calculate_distance(neighbourblkref, ref);


#if Mv_Rang
	motion_vector = Clip3(-32768, 32767, (sign * (((long long int)(motion_vector) * mult_distance * (MULTI / devide_distance) + HALF_MULTI) >> OFFSET))+delta2);
#else
	motion_vector = sign * ((motion_vector * mult_distance * (MULTI / devide_distance) + HALF_MULTI) >> OFFSET);
    motion_vector = Clip3(-32768, 32767, motion_vector);
#endif
    return motion_vector;
}
/*Lou 1016 End*/

void scalingMV(int *cur_mv_x, int *cur_mv_y, int curT, int ref_mv_x, int ref_mv_y, int refT, int factor_sign)
{
#if	Mv_Rang
    *cur_mv_x =  Clip3(-32768, 32767,((long long int)(curT) * ref_mv_x * (MULTI / refT) + HALF_MULTI) >> OFFSET);
    *cur_mv_y = Clip3(-32768, 32767,((long long int)(curT) * ref_mv_y * (MULTI / refT) + HALF_MULTI) >> OFFSET);
#else
	*cur_mv_x = (curT * ref_mv_x * (MULTI / refT) + HALF_MULTI) >> OFFSET;
	*cur_mv_y = (curT * ref_mv_y * (MULTI / refT) + HALF_MULTI) >> OFFSET;

    *cur_mv_x = Clip3(-32768, 32767, (*cur_mv_x));
    *cur_mv_y = Clip3(-32768, 32767, (*cur_mv_y));
#endif
}

#if MV_SCALE
int scale_mv(int mv, int dist_dst, int dist_src)
{
	#if	Mv_Rang
	mv = Clip3(-32768, 32767,(((long long int)(mv) * dist_dst * (MULTI / dist_src) + HALF_MULTI) >> OFFSET));
	#else
    mv = (int)((mv * dist_dst * (MULTI / dist_src) + HALF_MULTI) >> OFFSET);
	#endif
 return mv;
}

int scale_mv_y1(int mvy, int dist_dst, int dist_src)
{
    int oriPOC = 2 * hc->picture_distance;
    int oriRefPOC = oriPOC - dist_src;
    int scaledPOC = 2 * hc->picture_distance;
    int scaledRefPOC = scaledPOC - dist_dst;
    int delta, delta2;

    getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if	RD1601_FIX_BG
	return (int)Clip3(-32768, 32767,(((long long int)(mvy + delta) * dist_dst * (MULTI / dist_src) + HALF_MULTI) >> OFFSET) - delta2);
#else
    return (int)(scale_mv(mvy + delta, dist_dst, dist_src) - delta2);
#endif
}

int scale_mv_y2(int mvy, int dist_dst, int dist_src)
{
    int oriPOC = 2 * hc->picture_distance;
    int oriRefPOC = oriPOC - dist_src;
    int scaledPOC = 2 * hc->picture_distance;
    int scaledRefPOC = scaledPOC - dist_dst;
    int delta, delta2;

    getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if	RD1601_FIX_BG
	return (int)Clip3(-32768, 32767,-((((long long int)(mvy + delta) * dist_dst * (MULTI / dist_src) + HALF_MULTI) >> OFFSET) + delta2))* (-1);
#else
    return (int)(scale_mv(mvy + delta, dist_dst, dist_src) + delta2);
#endif
}

int scale_motion_vector_y1(int mvy, int currblkref, int neighbourblkref, int ref)
{
    int dist_dst = calculate_distance(currblkref, ref);
    int dist_src = calculate_distance(neighbourblkref, ref);
    int oriPOC = 2 * hc->picture_distance;
    int oriRefPOC = oriPOC - dist_src;
    int scaledPOC = 2 * hc->picture_distance;
    int scaledRefPOC = scaledPOC - dist_dst;
    int delta, delta2;

    getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if Mv_Clip
	return (int)(scale_motion_vector(mvy + delta, currblkref, neighbourblkref, ref, -delta2));
#else
    return (int)(scale_motion_vector(mvy + delta, currblkref, neighbourblkref, ref) - delta2);
#endif

}

int scale_motion_vector_y2(int mvy, int currblkref, int neighbourblkref, int ref)
{
    int dist_dst = calculate_distance(currblkref, ref);
    int dist_src = calculate_distance(neighbourblkref, ref);
    int oriPOC = 2 * hc->picture_distance;
    int oriRefPOC = oriPOC - dist_src;
    int scaledPOC = 2 * hc->picture_distance;
    int scaledRefPOC = scaledPOC - dist_dst;
    int delta, delta2;

    getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
#if Mv_Clip
	return (int)(scale_motion_vector(mvy + delta, currblkref, neighbourblkref, ref, delta2));
#else
    return (int)(scale_motion_vector(mvy + delta, currblkref, neighbourblkref, ref) + delta2);
#endif

}

// MV scaling for Direct Mode
int scale_mv_direct(int mv, int dist_dst, int dist_src)
{
  #if Mv_Rang
	 return ((long long int)(MULTI / dist_dst) * (1 + dist_src * mv) - 1) >> OFFSET;
#else
    return (int)((MULTI / dist_dst) * (1 + dist_src * mv) - 1) >> OFFSET;
#endif
}

void scale_mv_direct_x(int mv_x, int dist2, int dist4, int dist5, int *Fwmv_x, int *Bwmv_x)
{
    if (mv_x < 0) {
        *Fwmv_x = -scale_mv_direct(mv_x, dist2, -dist4);
        *Bwmv_x = scale_mv_direct(mv_x, dist2, -dist5);
    } else {
        *Fwmv_x = scale_mv_direct(mv_x, dist2, dist4);
        *Bwmv_x = -scale_mv_direct(mv_x, dist2, dist5);
    }
}

void scale_mv_direct_y(int mv_y, int dist1, int dist2, int dist3, int dist4, int dist5, int *Fwmv_y, int *Bwmv_y)
{
    if (mv_y < 0) {
        *Fwmv_y = -scale_mv_direct(mv_y, dist2, -dist4);
        *Bwmv_y = scale_mv_direct(mv_y, dist2, -dist5);
    } else {
        *Fwmv_y = scale_mv_direct(mv_y, dist2, dist4);
        *Bwmv_y = -scale_mv_direct(mv_y, dist2, dist5);
    }
#if HALF_PIXEL_COMPENSATION_DIRECT
    if (img->is_field_sequence) {
        int delta, delta2, delta_d, delta2_d;
        int oriPOC = dist1;
        int oriRefPOC = dist1 - dist2;
        int scaledPOC = dist3;
        int scaledRefPOC = dist3 - dist4;
        getDeltas(&delta, &delta2, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);

        scaledRefPOC = dist3 - dist5;
        getDeltas(&delta_d, &delta2_d, oriPOC, oriRefPOC, scaledPOC, scaledRefPOC);
        assert(delta_d == delta);

        if (mv_y + delta < 0) {
            *Fwmv_y = -scale_mv_direct(mv_y + delta, dist2, -dist4) - delta2;
            *Bwmv_y = scale_mv_direct(mv_y + delta_d, dist2, -dist5) - delta2_d;
        } else {
            *Fwmv_y = scale_mv_direct(mv_y + delta, dist2, dist4) - delta2;
            *Bwmv_y = -scale_mv_direct(mv_y + delta_d, dist2, dist5) - delta2_d;
        }
    }
#endif
}

int derive_dv(int neigh_mv)
{
    return neigh_mv;
}
#endif


/*
 ******************************************************************************
 *  Function: get reference list string.
 *     Input:
 *    Output: string in "[%d %d %d %d]" format
 *    Return:
 * Attention:
 *    Author: Falei LUO, 2016.01.30
 ******************************************************************************
 */
void get_reference_list_info(char *str)
{
    char str_tmp[16];
    int i;
    int poc = hc->f_rec->imgtr_fwRefDistance;

    if (img->num_of_references > 0) {
        strcpy(str, "[");
        for (i = 0; i < img->num_of_references; i++) {
#if RD1510_FIX_BG
            if (img->type == B_IMG) {
                sprintf(str_tmp, "%4d    ", hc->f_rec->ref_poc[img->num_of_references - 1 - i]);
            } else {
                sprintf(str_tmp, "%4d    ", hc->f_rec->ref_poc[i]);
            }
#else
            sprintf(str_tmp, "%4d     ", fref[i]->imgtr_fwRefDistance);
#endif

            str_tmp[5] = '\0';
            strcat(str, str_tmp);
        }
        strcat(str, "]");
    } else {
        str[0] = '\0';
    }
}

