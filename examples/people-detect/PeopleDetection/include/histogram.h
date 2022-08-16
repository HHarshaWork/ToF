#include <fstream>
#include <iostream>
#include <assert.h>

#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__
typedef struct roi_tag
{
	int32_t x;
	int32_t y;
	int32_t rows;
	int32_t cols;
	int32_t stride_x;
}roi;

class histogram_1d
{
private:
	int32_t* histogram = 0;
	int16_t m_start = 0;
	int16_t m_end = 0x7FFF;
	int16_t m_range = 1;
	int16_t nbins = -1;

	void compute_histogram(uint16_t* pBuf, int32_t size);

public:
	histogram_1d(void)
	{

	}

	histogram_1d(uint16_t* pBuf, int32_t size, int16_t start = 0, int16_t end = 32767, int16_t range = 1);
	
	~histogram_1d()
	{
		delete histogram;
	}
	void set_histogram_param(int32_t start, int16_t end, int16_t range);
	int32_t* get_histogram(uint16_t* pBuf, int16_t size, int32_t& nbins);
	void get_max_bin(int16_t& index, int32_t& val);
	void get_min_bin(int16_t& index, int32_t& val);
};
#endif