#include "adaptiveQP.h"
//#include "global.h"
#include "float.h"
#include "stdlib.h"

#if ADAPTIVE_QUANTIZATION
/** Initialize member variables
 * \param iWidth Picture width
 * \param iHeight Picture height
 * \param uiMaxAQDepth Maximum depth of unit block for assigning QP adaptive to local image characteristics
 * \return void
 */
void adaptiveQPInit( int iWidth, int iHeight, unsigned int uiMaxAQDepth)
{ 
  unsigned int uiAQPartWidth;
  unsigned int uiAQPartHeight;		
  unsigned int uiNumAQPartInWidth;
  unsigned int uiNumAQPartInHeight;
  unsigned int d;

  aQPpic->m_uiMaxAQDepth = uiMaxAQDepth;
  if ( uiMaxAQDepth > 0 )
  {
    aQPpic->m_acAQLayer = (PicAdaptiveQLayer *)calloc(uiMaxAQDepth, sizeof(PicAdaptiveQLayer))  ; 
    for (d = 0; d < uiMaxAQDepth; d++)
    {
      uiAQPartWidth = MAX_CU_SIZE>>d;
      uiAQPartHeight = MAX_CU_SIZE>>d;		
      uiNumAQPartInWidth = (iWidth + uiAQPartWidth-1) / uiAQPartWidth;
      uiNumAQPartInHeight = (iHeight + uiAQPartHeight-1) / uiAQPartHeight;
      aQPpic->m_acAQLayer[d].m_acTEncAQU = (AdaptiveQUnit *) calloc(uiNumAQPartInWidth * uiNumAQPartInHeight,sizeof(AdaptiveQUnit));
    }
  }
}

/** Analyze source picture and compute local image characteristics used for QP adaptation
 * \param iWidth Picture width
 * \param iHeight Picture height
 * \return Void
 */
void xPreanalyze(int iWidth, int iHeight)
{
	unsigned int iStride = iWidth;

	unsigned int d;
	byte* pLineY;
	PicAdaptiveQLayer* pcAQLayer;

	unsigned int uiAQPartWidth;
	unsigned int uiAQPartHeight;
	unsigned int uiNumAQPartInWidth;
	unsigned int uiNumAQPartInHeight;
	AdaptiveQUnit* pcAQU;

	double dSumAct;
	unsigned int y, x;
	unsigned int uiCurrAQPartHeight, uiCurrAQPartWidth;

	byte* pBlkY;
	unsigned long uiSum[4], uiSumSq[4];
	int i;
	unsigned int uiNumPixInAQPart, by, bx;
	double dMinVar, dAverage, dVariance, dAvgAct;

	unsigned int aquy, aqux;

	for (d = 0; d < aQPpic->m_uiMaxAQDepth; d++ )
	{
		pLineY = imgY_org_buffer;
		pcAQLayer = &(aQPpic->m_acAQLayer[d]);
		uiAQPartWidth = MAX_CU_SIZE>>d;
		uiAQPartHeight = MAX_CU_SIZE>>d;
		uiNumAQPartInWidth = (iWidth + uiAQPartWidth-1) / uiAQPartWidth;
		uiNumAQPartInHeight = (iHeight + uiAQPartHeight-1) / uiAQPartHeight;
		pcAQU = pcAQLayer->m_acTEncAQU;

		dSumAct = 0.0;
		for (y = 0; y < iHeight; y += uiAQPartHeight )
		{
			uiCurrAQPartHeight = min(uiAQPartHeight, iHeight-y);
			for (x = 0; x < iWidth; x += uiAQPartWidth, pcAQU++ )
			{
				uiCurrAQPartWidth = min(uiAQPartWidth, iWidth-x);
				pBlkY = &pLineY[x];
				for(i = 0;i < 4;i++)
				{
					uiSum[i] = 0;
					uiSumSq[i] = 0;
				}

				uiNumPixInAQPart = 0;
				by = 0;
				for ( ; by < uiCurrAQPartHeight>>1; by++ )
				{
					bx = 0;
					for ( ; bx < uiCurrAQPartWidth>>1; bx++, uiNumPixInAQPart++ )
					{
						uiSum  [0] += pBlkY[bx];
						uiSumSq[0] += pBlkY[bx] * pBlkY[bx];
					}
					for ( ; bx < uiCurrAQPartWidth; bx++, uiNumPixInAQPart++ )
					{
						uiSum  [1] += pBlkY[bx];
						uiSumSq[1] += pBlkY[bx] * pBlkY[bx];
					}
					pBlkY += iStride;
				}
				for ( ; by < uiCurrAQPartHeight; by++ )
				{
					bx = 0;
					for ( ; bx < uiCurrAQPartWidth>>1; bx++, uiNumPixInAQPart++ )
					{
						uiSum  [2] += pBlkY[bx];
						uiSumSq[2] += pBlkY[bx] * pBlkY[bx];
					}
					for ( ; bx < uiCurrAQPartWidth; bx++, uiNumPixInAQPart++ )
					{
						uiSum  [3] += pBlkY[bx];
						uiSumSq[3] += pBlkY[bx] * pBlkY[bx];
					}
					pBlkY += iStride;
				}

				dMinVar = DBL_MAX;
				for (i=0; i<4; i++)
				{
					dAverage = (double)(uiSum[i]) / uiNumPixInAQPart;
					dVariance = (double)(uiSumSq[i]) / uiNumPixInAQPart - dAverage * dAverage;
					dMinVar = min(dMinVar, dVariance);
				}
				pcAQU->dActivity = 1.0 + dMinVar;
				dSumAct += pcAQU->dActivity;
			}
			pLineY += iStride * uiCurrAQPartHeight;
		}

		dAvgAct = dSumAct / (uiNumAQPartInWidth * uiNumAQPartInHeight);
		pcAQLayer->m_dAvgActivity= dAvgAct ;
		pcAQU = pcAQLayer->m_acTEncAQU;
		for (aquy = 0; aquy < uiNumAQPartInHeight; aquy++ )
		{
			for ( aqux = 0; aqux < uiNumAQPartInWidth; aqux++, pcAQU++ )
			{
				pcAQU->dNormAct = (2.0*pcAQU->dActivity + dAvgAct) / (pcAQU->dActivity + 2.0*dAvgAct);
			}
		}
	}
}
#endif