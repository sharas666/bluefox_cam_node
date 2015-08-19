#ifndef __STEREOSYSTEM__H__
#define __STEREOSYSTEM__H__

#include <iostream>
#include <string>
#include <vector>
#include <future>

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "Camera.h"
#include "utility.h"

class Stereosystem
{
	public:
		Stereosystem(Camera*, Camera*);
		~Stereosystem();

		double calibrate(std::vector<cv::Mat> const&, std::vector<cv::Mat> const&,double,cv::Size);

		bool loadExtrinisic(std::string const&);
		bool loadIntrinsic(std::string const&);
		bool saveExtrinsic(std::string const&);
		bool saveIntrinsic(std::string const&);

		bool getImagepair(Stereopair&);

		bool undistort_images(Stereopair&) const;

		bool initRectification();
		void rectify(Stereopair&) const;
		bool rectify_images(Stereopair&);
		void resetRectification();

		void getFundamentalMatrix(cv::Mat &) const;
		void getTranslationMatrix(cv::Mat &) const;

		double getBaseline() const;
		cv::Mat getRotationMatrix() const;
		cv::Mat getQMatrix() const;

	private:
		Camera 			*mLeft;
		Camera 			*mRight;
		cv::Mat			mR;
		cv::Mat			mT;
		cv::Mat			mE;
		cv::Mat			mF;

		bool				mIsInit;
		cv::Mat			mMap1[2];
		cv::Mat			mMap2[2];
		cv::Mat 		mR0;
		cv::Mat			mR1;
		cv::Mat 		mP0;
		cv::Mat 		mP1;
		cv::Mat 		mQ;
		cv::Rect 		mValidROI[2];
		cv::Rect    mDisplayROI;

		cv::Mat 		mIntrinsicLeft;
		cv::Mat 		mIntrinsicRight;
		cv::Mat 		mDistCoeffsLeft;
		cv::Mat 		mDistCoeffsRight;

		std::string		mTag;
};

#endif //__STEREOSYSTEM__H__
