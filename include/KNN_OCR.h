/*
 * @file	: KNN.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

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

class KNNResult
{
  public:
    KNNResult()
    {
        classResult = -1;
        precisionRatio = -1;
        accuracyAmount = -1;
    }
    float classResult;
    float precisionRatio;
    int accuracyAmount;
    float min_distance[3];
    Mat mat_distance;
};

class basicOCR
{
	public:
		float classify(IplImage* img,int showResult);
        basicOCR (char*);
        void test();
        KNNResult knn_result;

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
