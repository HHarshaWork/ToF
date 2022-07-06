#include <iostream>
#include <fstream>
#include <stdlib.h>
//#define OPENCV_DEBUG

#ifdef OPENCV_DEBUG
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
#endif

#include "../include/image_filtering.h"

using namespace std;

/* Arrange every column as max, med, min
*  max0 max1 max2 -> min of first row
*  med0 med1 med2 -> med of second row -> med of these 3 numbers 
*  min0 min1 min2 -> max of theird row 
*/

#define FMIN(a,b) (((a) < (b)) ? (a) : (b))
#define FMAX(a,b) (((a) > (b)) ? (a) : (b))

void GammaCorrect(uint16_t* pData, int nNumPixels)
{
	for (int i = 0; i < nNumPixels; i++)
	{
		float nRead = (float)pData[i];
		float nOutVal = (float)(256.0f * log(nRead)) / log(2048.0f);
		pData[i] = (uint16_t)nOutVal;
	}
}

void MedianFilter(uint16_t *pImg, uint16_t *pOutImg, int32_t cols, int32_t rows, int32_t kernelsz)
{
	if (kernelsz == KERNEL_SIZE_3x3)
	{
		for (int32_t y = 1; y < rows-1; y++)
		{
			/* Find Maximum, Minimum and Median of Column 1 */
			int32_t nTemp1 = FMIN(pImg[y * cols + 0], pImg[(y + 1) * cols + 0]);
			int32_t nTemp2 = FMAX(pImg[y * cols + 0], pImg[(y + 1) * cols + 0]);

			int32_t nMin = FMIN(pImg[(y - 1) * cols + 0], nTemp1);    /* Minimum */
			nTemp1 = FMAX(pImg[(y - 1) * cols + 0], nTemp1);

			int32_t nMed = FMIN(nTemp1, nTemp2); /* Median */
			int32_t nMax = FMAX(nTemp1, nTemp2); /* Maximum */

			int32_t nMed1 = nMed;
			int32_t nMed2 = nMed;
			int32_t nMax1 = nMax;
			int32_t nMax2 = nMax;
			int32_t nMin1 = nMin;
			int32_t nMin2 = nMin;

			for (int32_t x = 1; x < cols; x++)
			{
				/* Find Maximum, Minimum and Median of Column x */
				nTemp1 = FMIN(pImg[y * cols + x], pImg[(y + 1) * cols + x]);
				nTemp2 = FMAX(pImg[y * cols + x], pImg[(y + 1) * cols + x]);

				nMin = FMIN(pImg[(y - 1) * cols + x], nTemp1);
				nTemp1 = FMAX(pImg[(y - 1) * cols + x], nTemp1);

				nMed = FMIN(nTemp1, nTemp2);
				nMax = FMAX(nTemp1, nTemp2);

				/* nMinMax = Minimum of the maximums */
				int32_t nMinMax = FMIN(nMax1, nMax2);
				nMinMax = FMIN(nMax, nMinMax);

				/* nMaxMin = Maximum of the minimums */
				int32_t nMaxMin = FMAX(nMin1, nMin2);
				nMaxMin = FMAX(nMin, nMaxMin);

				/* nMedMed = Median of Median */
				int32_t nMedMed = FMIN(nMed, nMed1);
				nMed1 = FMAX(nMed, nMed1);
				nMedMed = FMAX(nMed2, nMedMed);
				nMedMed = FMIN(nMed1, nMedMed);

				nMed1 = nMed2;
				nMed2 = nMed;

				nMin1 = nMin2;
				nMin2 = nMin;

				nMax1 = nMax2;
				nMax2 = nMax;

				/* nMedian = Median(nMinMax,nMaxMin,nMedMed) */
				int32_t nMedian = FMIN(nMedMed, nMinMax);
				nMinMax = FMAX(nMedMed, nMinMax);
				nMedian = FMAX(nMedian, nMaxMin);
				nMedian = FMIN(nMedian, nMinMax);

				pOutImg[y * cols + (x - 1)] = nMedian;
			}

			int32_t nMinMax = FMIN(nMax2, nMax1);
			int32_t nMaxMin = FMAX(nMin2, nMin1);
			int32_t nMedMed = nMed2;

			/* nMedian = Median(nMinMax,nMaxMin,nMedMed) */
			int32_t nMedian = FMIN(nMedMed, nMinMax);
			nMinMax = FMAX(nMedMed, nMinMax);
			nMedian = FMAX(nMedian, nMaxMin);
			nMedian = FMIN(nMedian, nMinMax);

			pOutImg[y * cols + (cols - 1)] = nMedian;
		}
	}
	
	if (kernelsz == KERNEL_SIZE_5x5)
	{
		
	}
}

void Dilation(uint16_t* pImg, uint16_t* pOutImg, int32_t cols, int32_t rows, int32_t kernelsz, uint16_t* mask)
{
	for (int y = kernelsz / 2; y < rows - kernelsz / 2; y++)
	{
		for (int x = kernelsz / 2; x < cols - kernelsz / 2; x++)
		{		
			uint16_t* p = &pImg[y * cols + x];
			int max = -1;

			for (int i = 0; i < kernelsz; i++)
			{
				for (int j = 0; j < kernelsz; j++)
				{
					if (mask[i * kernelsz + j])
					{
						if (p[(-kernelsz / 2 + i) * cols + (-kernelsz / 2 + j)] > max)
						{
							max = p[(-kernelsz / 2 + i) * cols + (-kernelsz / 2 + j)];
						}
					}
				} /* j */
			} /* i */
			pOutImg[y * cols + x] = max;
		} /* x */
	} /* y */
}

void Erosion(uint16_t* pImg, uint16_t* pOutImg, int32_t cols, int32_t rows, int32_t kernelsz, uint16_t* mask)
{
	for (int y = kernelsz / 2; y < rows - kernelsz / 2; y++)
	{
		for (int x = kernelsz / 2; x < cols - kernelsz / 2; x++)
		{
			uint16_t* p = &pImg[y * cols + x];
			int min = 0x7FFF;

			for (int i = 0; i < kernelsz; i++)
			{
				for (int j = 0; j < kernelsz; j++)
				{
					if (mask[i * kernelsz + j])
					{
						if (p[(-kernelsz / 2 + i) * cols + (-kernelsz / 2 + j)] < min)
						{
							min = p[(-kernelsz / 2 + i) * cols + (-kernelsz / 2 + j)];
						}
					}
				} /* j */
			} /* i */
			pOutImg[y * cols + x] = min;
		} /* x */
	} /* y */
}

void regionGrow_int16(uint16_t* img, uint8_t* outimg, int16_t rows, int16_t cols, 
	roi roiparams, int16_t seed_x, int16_t seed_y, int32_t threshold)
{
	int32_t stack_head_index = 0;
	int32_t stack_count = 0;
	int32_t neighbours[8*2] = {-1,-1, -1,0, -1,1, 0,-1, 0,1, 1,-1, 1,0, 1,1 };
	memset(outimg, 0, rows * cols * sizeof(uint16_t));
	
	uint32_t* stack = new uint32_t[rows * cols];
#ifdef OPENCV_DEBUG
	Mat debugImg = Mat::zeros(Size(cols, rows), CV_8UC1);
	uint8_t* p = debugImg.data;
#endif
	//stack push
	stack[stack_head_index] = (seed_x << 16) | seed_y;
	stack_count++;
	stack_head_index++;

	while (stack_count)
	{
		// stack pop
		int x_centre = (stack[stack_head_index-1] & 0xFFFF0000) >> 16;
		int y_centre = (stack[stack_head_index-1] & 0xFFFF);
		stack_head_index--;
		stack_count--;

		int val_centre = img[y_centre * cols + x_centre];

		for (int i = 0; i < 8; i++)
		{
			int y = y_centre + neighbours[2 * i];
			int x = x_centre + neighbours[2 * i + 1];
			if (y > roiparams.miny && y < roiparams.maxy && x > roiparams.minx && x < roiparams.maxx)
			{
				if (abs(val_centre - img[y * cols + x]) <= threshold && outimg[y * cols + x] == 0)
				{
					stack[stack_head_index] = (x << 16) | y;
					stack_count++;
					stack_head_index++;
#ifdef OPENCV_DEBUG
					p[y * cols + x] = 255;
#endif
					outimg[y * cols + x] = 255;
				}
			}
		}
	}
	delete[] stack;
}