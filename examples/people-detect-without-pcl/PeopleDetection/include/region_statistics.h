#include <fstream>
#include <iostream>
#include <assert.h>

#include "histogram.h"

#ifndef __REGION_STATISTICS_H__
#define __REGION_STATISTICS_H__

class RegionStatistics
{
private:
	histogram_1d m_hist_1d;
	roi m_roi_region;
	uint16_t* m_data_ptr = 0;
	uint16_t* m_out_ptr = 0;
	int32_t mean = 0;
	int32_t max = 0;
	int32_t min = 0;
	int32_t m_num_pixels_in_region = 0;
public:
	RegionStatistics()
	{
		m_roi_region.cols = 0;
		m_roi_region.rows = 0;
		m_roi_region.x = 0;
		m_roi_region.y = 0;
		m_roi_region.stride_x = 0;
		m_num_pixels_in_region = 0;
	}
	RegionStatistics(const roi roi_region)
	{
		m_roi_region = roi_region;
#if 0
		m_roi_region.cols = roi_region.cols;
		m_roi_region.rows = roi_region.rows;
		m_roi_region.x = roi_region.x;
		m_roi_region.y = roi_region.y;
		m_roi_region.stride_x = roi_region.stride_x;
#endif
		m_num_pixels_in_region = m_roi_region.cols * m_roi_region.rows;
	}
	RegionStatistics(const roi roi_region, uint16_t *data_ptr)
	{
#if 0
		m_roi_region.cols = roi_region.cols;
		m_roi_region.rows = roi_region.rows;
		m_roi_region.x = roi_region.x;
		m_roi_region.y = roi_region.y;
		m_roi_region.stride_x = roi_region.stride_x;
#endif
		m_roi_region = roi_region;
		m_num_pixels_in_region = m_roi_region.cols * m_roi_region.rows;
		m_data_ptr = data_ptr;
	}
	~RegionStatistics()
	{
		if (m_out_ptr != 0) delete[] m_out_ptr;
	}

	void set_roi(const roi roi_region);
	roi get_roi()
	{
		return m_roi_region;
	}
	void set_data(uint16_t* data_ptr);
	uint16_t* get_data()
	{
		return m_data_ptr;
	}
	void get_stats(int32_t *mean = 0, int32_t *max = 0, int32_t *min = 0);
	int16_t*  get_histogram(int32_t &max, int16_t &max_index, int32_t &bins);
	uint16_t* get_thresholded_region(int32_t& threshold_min, int32_t& threshold_max);
};

#endif
