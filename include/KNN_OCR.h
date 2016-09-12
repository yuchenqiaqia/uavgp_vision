/******************************************************************************

版权所有 (C), 2016, 中国科学院沈阳自动化研究所，一室，旋翼飞行机器人课题组

******************************************************************************
作	  者   : 肖斌
******************************************************************************/
#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include <cv.h>
#include <highgui.h>
#include <ml.h>
#include <stdio.h>
#include <ctype.h>
#endif
//#include <opencv2/opencv.hpp>
//#include <opencv2/ml/ml.hpp>
using namespace cv;

class basicOCR
{
	public:
		float classify(IplImage* img,int showResult);
        basicOCR (char*);
		void test();	
	private:
		char file_path[255];
		int train_samples;
		int classes;
		CvMat* trainData;
		CvMat* trainClasses;
        Mat trainData_mat;
        Mat trainClasses_mat;
		int size;
		//static const int K=10;
        static const int K = 10;

        //CvKNearest *knn;
        Ptr<ml::KNearest>  knn;

		void getData();
		void train();
};
