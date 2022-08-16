#include <fstream>
#include <iostream>
#include <assert.h>

#include "histogram.h"
#include "region_statistics.h"

#define OPENCV_DEBUG

#ifdef OPENCV_DEBUG
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
#endif

#ifndef __DETECT_PEOPLE_HEAD_H__
#define __DETECT_PEOPLE_HRAD_H__

#define COLOR(r,s,v) ((r) * (s) + (v) * (1-s))
typedef struct point_tag
{
	int32_t x;
	int32_t y;
}Point2D;

typedef struct regions_tag
{
	Point2D TopLeft;
	Point2D BottomRight;
}Region;

class DetectPeopleHead
{
private:
	uint16_t* m_prev_depth_img = 0;
	uint16_t* m_prev_ir_img = 0;
	uint16_t* m_curr_depth_img = 0;
	uint16_t* m_curr_ir_img = 0;
	int32_t m_num_frames = 0;
	int32_t m_rows = 0;
	int32_t m_cols = 0;
	int32_t m_depth_threshold = 0;
	RegionStatistics* regions_of_interest=0;
	int32_t m_num_regions = 0;

	void detect_frame_difference_region(RegionStatistics* r);

public:
#ifdef OPENCV_DEBUG
	Mat debug_display_img;
#endif

	DetectPeopleHead(int32_t rows, int32_t cols)
	{
		m_rows = rows;
		m_cols = cols;
		m_num_frames = 0;
		m_num_regions = 0;
		m_depth_threshold = 5; /* 5 cm */
#ifdef OPENCV_DEBUG
		debug_display_img.create(rows, cols, CV_8UC3);
#endif
	}
	
	~DetectPeopleHead()
	{
		delete[] regions_of_interest;
	}

	void Init(void);
	void Configure(void);
	void Detect(uint16_t* curr_depth_img, uint16_t* curr_ir_img);
	void SetRegionsOfInterest(std::vector<Region> regions, int32_t num_regions);
#ifdef OPENCV_DEBUG
	Mat* GetDebugImg()
	{
		return &debug_display_img;
	}
#endif
};

#endif
