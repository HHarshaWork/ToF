#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "../include/histogram.h"

void histogram_1d::compute_histogram(uint16_t *pBuf, int32_t size)
{
	int32_t nbins = (int32_t)((m_end - m_start) / (float)m_range) + 1;
	if (histogram != 0)	delete[] histogram;
	int32_t* histogram = new int32_t[nbins];
	memset(histogram, 0, nbins * sizeof(int32_t));

	for (int i = 0; i < size; i++)
	{
		if (pBuf[i] >= m_start && pBuf[i] <= m_end)
		{
			int32_t val = pBuf[i] / m_range;
			pBuf[val]++;
		}
	}
}

histogram_1d::histogram_1d(uint16_t* pBuf, int32_t size, int16_t start, int16_t end, int16_t range)
{
	m_start = start;
	m_end = end;
	m_range = range;

	compute_histogram(pBuf, size);
	return;
}

void histogram_1d::set_histogram_param(int32_t start, int16_t end, int16_t range)
{
	m_start = start;
	m_end = end;
	m_range = range;
}

int32_t * histogram_1d::get_histogram(uint16_t* pBuf, int16_t size, int32_t& nbins)
{
	compute_histogram(pBuf, size);
	return histogram;
}

void histogram_1d::get_max_bin(int16_t& index, int32_t& val)
{
	int32_t max = 0;
	index = -1;
	for (int i = 0; i < nbins; i++)
	{
		if (histogram[i] > max)
		{
			max = histogram[i];
			index = i;
		}
	}
	val = max;
}

void histogram_1d::get_min_bin(int16_t& index, int32_t& val)
{
	int32_t min = 0x7FFFFFFF;
	index = -1;
	for (int i = 0; i < nbins; i++)
	{
		if (histogram[i] < min && histogram[i] > 0)
		{
			min = histogram[i];
			index = i;
		}
	}
	val = min;
}
