#ifndef __UTILITY__H__
#define __UTILITY__H__

#include <dirent.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Camera stuff
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

//Logging stuff
#include "easylogging++.h"

#include "Camera.h"

struct Stereopair
{
	Stereopair();
	Stereopair(cv::Mat &, cv::Mat &);
	~Stereopair();

	cv::Mat mLeft;
	cv::Mat mRight;
	std::string mTag;
};

struct CameraInit
{
	Camera* left;
	Camera* right;
	bool init;
};

namespace Utility
{
	int getFiles (std::string const& dir, std::vector<std::string> &files);
	bool directoryExist(std::string const& dirPath);
	bool createDirectory(std::string const& dirPath);

	bool initCameras(mvIMPACT::acquire::DeviceManager&,Camera*&,Camera*&);

	bool checkConfig(std::string const&,std::vector<std::string> const&, cv::FileStorage &);

	double checkSharpness(cv::Mat const&);
	bool covariance(cv::Mat const&, cv::Mat const&, cv::Scalar&);
	bool standartDeviation(cv::Mat const&, cv::Scalar&);
	bool normalizedCrossCorrelation(Stereopair const&, cv::Mat&);

	bool calcCoordinate(cv::Mat_<float> &,cv::Mat const&, cv::Mat const&,int,int);

}

#endif //__UTILITY__H__
