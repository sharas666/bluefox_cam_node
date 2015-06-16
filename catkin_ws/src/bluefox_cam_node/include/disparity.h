#ifndef __DISPARITY__H__
#define __DISPARITY__H__

#include "utility.h"

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/calib3d/calib3d.hpp>

namespace Disparity
{
	void sgbm(Stereopair const&, cv::Mat&, cv::StereoSGBM);
  void bm(Stereopair const&, cv::Mat& , cv::StereoBM);
  void tm(Stereopair const&, cv::Mat& ,unsigned int);
}

#endif //__DISPARITY__H__