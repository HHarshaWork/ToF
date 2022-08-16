#include <fstream>
#include <iostream>
#include <assert.h>

#ifndef __IMAGE_FILTERING_H__
#define __IMAGE_FILTERING_H__

#define KERNEL_SIZE_3x3 0
#define KERNEL_SIZE_5x5 1

typedef struct roitag
{
	int32_t minx;
	int32_t miny;
	int32_t maxx;
	int32_t maxy;
}roi;

extern
void MedianFilter(uint16_t* pImg, uint16_t* pOutImg, int32_t cols, int32_t rows, int32_t kernelsz);

extern
void Dilation(uint16_t* pImg, uint16_t* pOutImg, int32_t cols, int32_t rows, int32_t kernelsz, uint16_t* mask);

extern
void Erosion(uint16_t* pImg, uint16_t* pOutImg, int32_t cols, int32_t rows, int32_t kernelsz, uint16_t* mask);

extern 
void regionGrow_int16(uint16_t* img, uint8_t* outimg, int16_t rows, int16_t cols,roi roiparams, int16_t seed_x, int16_t seed_y, int32_t threshold);

extern
void GammaCorrect(uint16_t* pData, int nNumPixels);
#endif
