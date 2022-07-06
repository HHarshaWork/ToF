#include <fstream>
#include <sstream>
#include <iostream>


#include "../include/histogram.h"
#include "../include/region_statistics.h"
#include "../include/detect_people_head.h"

void detect_and_track_head(uint16_t* curr_depth_image, int32_t rows, int32_t cols)
{

}

void scale_image_16bitgray_to_8bitrgb(unsigned short* pSrc, unsigned char* pDst, int size)
{
	int max = 0, min = 65535;
	int index;
	for (int i = 0; i < size; i++)
	{
		if (pSrc[i] > max)
		{
			index = i;
			max = pSrc[i];
		}
		if (pSrc[i] < min) min = pSrc[i];
	}
	int j = 0;
	for (int i = 0; i < size; i++)
	{
		unsigned char a = (pSrc[i] - min) / (float)(max - min) * 255;
		pDst[j++] = a;
		pDst[j++] = a;
		pDst[j++] = a;
	}
}

void DetectPeopleHead::detect_frame_difference_region(RegionStatistics* r)
{
	roi roi_region = r->get_roi();
	uint16_t* ptr_curr = r->get_data();
	uint16_t* ptr_prev = m_prev_depth_img + roi_region.y * roi_region.stride_x + roi_region.x;
	for (int y = 0; y < roi_region.rows; y++)
	{
		for (int x = 0; x < roi_region.cols; x++)
		{
			uint16_t prev_val = ptr_prev[y * roi_region.stride_x + x];
			uint16_t curr_val = ptr_prev[y * roi_region.stride_x + x];
			if (abs(prev_val - curr_val) > m_depth_threshold)
			{
#ifdef OPENCV_DEBUG
				uint8_t b,g,r;
				b = debug_display_img.data[(y * roi_region.stride_x + x) * 3];
				debug_display_img.data[(y * roi_region.stride_x + x) * 3] = COLOR(b, 0.7, 255);
#endif
			}
		}
	}
}
void DetectPeopleHead::Init(void)
{

}

void DetectPeopleHead::Configure(void)
{

}

void DetectPeopleHead::Detect(uint16_t* curr_depth_img, uint16_t* curr_ir_img)
{
	/* for each region of interest find difference in depth from previous frame */
#ifdef OPENCV_DEBUG
	scale_image_16bitgray_to_8bitrgb(curr_depth_img, debug_display_img.data, m_rows * m_cols);
#endif
	roi roi_region;
	m_curr_depth_img = curr_depth_img;
	m_curr_ir_img = curr_ir_img;
	if (m_prev_depth_img != 0)
	{
		for (int i = 0; i < m_num_regions; i++)
		{
			roi_region = regions_of_interest[i].get_roi();
			regions_of_interest[i].set_data(curr_depth_img + roi_region.y * roi_region.stride_x + roi_region.x);
			detect_frame_difference_region(&regions_of_interest[i]);
		}
	}
	m_prev_depth_img = m_curr_depth_img;
}

void DetectPeopleHead::SetRegionsOfInterest(std::vector<Region> regions, int32_t num_regions)
{
	m_num_regions = num_regions;
	if (regions_of_interest != 0) delete[] regions_of_interest;
	regions_of_interest = new RegionStatistics[m_num_regions];
	roi roi_region;
	for (int i = 0; i < num_regions; i++)
	{
		
		roi_region.x = regions[i].TopLeft.x;
		roi_region.y = regions[i].TopLeft.y;
		roi_region.rows = regions[i].BottomRight.y - regions[i].TopLeft.y + 1;
		roi_region.cols = regions[i].BottomRight.x - regions[i].TopLeft.x + 1;
		roi_region.stride_x = m_cols;

		regions_of_interest->set_roi(roi_region);
	}
}



