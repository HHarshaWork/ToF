#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "../include/histogram.h"
#include "../include/region_statistics.h"

void RegionStatistics::set_roi(const roi roi_region)
{
	m_roi_region = roi_region;
}

void RegionStatistics::set_data(uint16_t* data_ptr)
{
	m_data_ptr = data_ptr + m_roi_region.y * m_roi_region.stride_x + m_roi_region.x;
}

void RegionStatistics::get_stats(int32_t* mean, int32_t* max, int32_t* min)
{
	if (m_data_ptr == 0) return;

	int32_t sum = 0, min_val = 0x7FFF, max_val = 0;
	uint16_t* ptr = &m_data_ptr[m_roi_region.y * m_roi_region.stride_x + m_roi_region.x];
	int32_t val;
	int32_t cnt = 0;
	for (int y = 0; y < m_roi_region.rows; y++)
	{
		for (int x = 0; x < m_roi_region.cols; x++)
		{
			val = *ptr++;
			sum += val;
			if (val > max_val && val > 0) max_val = val;
			if (val < min_val && val > 0) min_val = val;
			cnt++;
		}
		ptr += (m_roi_region.stride_x - m_roi_region.cols);
	}
	*mean = (int32_t)(sum / (float)cnt);
	*max = max_val;
	*min = min_val;
	return;
}

int16_t* RegionStatistics::get_histogram(int32_t& max, int16_t& max_index, int32_t& bins)
{
	m_hist_1d.set_histogram_param(100, 400, 5);
	if (m_data_ptr == 0) return 0;
	uint16_t* data = new uint16_t[m_num_pixels_in_region];
	uint16_t* ptr = data;
	int32_t offset = m_roi_region.y * m_roi_region.stride_x + m_roi_region.x;
	for (int y = 0; y < m_roi_region.rows; y++)
	{
		for (int x = 0; x < m_roi_region.cols; x++)
		{
			*ptr++ = m_data_ptr[offset + y * m_roi_region.stride_x + m_roi_region.x];
		}
	}
	m_hist_1d.get_histogram(data, m_num_pixels_in_region, bins);
	m_hist_1d.get_max_bin(max_index, max);
	delete[] data;
}

uint16_t* RegionStatistics::get_thresholded_region(int32_t& threshold_min, int32_t& threshold_max)
{
	if (m_out_ptr != 0) delete[] m_out_ptr;
	m_out_ptr = new uint16_t[m_num_pixels_in_region];
}
