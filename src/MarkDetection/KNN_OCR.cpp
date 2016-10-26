/*
 * @file	: KNN_OCR.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */
#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include "ml.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#include "preprocessing.h"
#include "KNN_OCR.h"
using namespace cv::ml;

void basicOCR::getData()
{
	IplImage* src_image;
	IplImage prs_image;
	CvMat row,data;
	char file[255];
	int i,j;
	for(i =0; i<classes; i++)
	{
		for( j = 0; j< train_samples; j++)
		{
			//Load file
			if(j<10)
				sprintf(file,"%s%d/%d0%d.pbm",file_path, i, i , j);
			else
				sprintf(file,"%s%d/%d%d.pbm",file_path, i, i , j);
			src_image = cvLoadImage(file,0);
			if(!src_image)
			{
				printf("Error: Cant load image %s\n", file);
				//exit(-1);
			}
			//process file
			prs_image = preprocessing(src_image, size, size);
			
			//Set class label
			cvGetRow(trainClasses, &row, i*train_samples + j);
			cvSet(&row, cvRealScalar(i));
			//Set data 
			cvGetRow(trainData, &row, i*train_samples + j);

			IplImage* img = cvCreateImage( cvSize( size, size ), IPL_DEPTH_32F, 1 );
			//convert 8 bits image to 32 float image
			cvConvertScale(&prs_image, img, 0.0039215, 0);

			cvGetSubRect(img, &data, cvRect(0,0, size,size));
			
			CvMat row_header, *row1;
			//convert data matrix sizexsize to vecor
			row1 = cvReshape( &data, &row_header, 0, 1 );
			cvCopy(row1, &row, NULL);
		}
	}
    printf("get data done!\n");
    return;
}

void basicOCR::train()
{
    //knn = new CvKNearest( trainData, trainClasses, 0, false, K );

    trainData_mat = cvarrToMat(trainData,true);
    trainClasses_mat = cvarrToMat(trainClasses,true);

    Mat temp0(trainData->rows, trainData->cols, trainData->type, trainData->data.fl);
    //temp0.copyTo(trainData_mat);
    Mat temp1(trainClasses->rows, trainClasses->cols, trainClasses->type, trainClasses->data.fl);
    //temp1.copyTo(trainClasses_mat);

    knn->train( trainData_mat, ml::ROW_SAMPLE, trainClasses_mat );
    printf("train data done!\n");
    return;
}


float basicOCR::classify(IplImage* img, int showResult)
{
	IplImage prs_image;
	CvMat data;
	CvMat* nearest=cvCreateMat(1,K,CV_32FC1);
	float result;
	//process file
	prs_image = preprocessing(img, size, size);
	
	//Set data 
    IplImage* img32 = cvCreateImage( cvSize( size, size ), IPL_DEPTH_32F, 1 );
	cvConvertScale(&prs_image, img32, 0.0039215, 0);
	cvGetSubRect(img32, &data, cvRect(0,0, size,size));
	CvMat row_header, *row1;
	row1 = cvReshape( &data, &row_header, 0, 1 );

    //result=knn->find_nearest(row1,K,0,0,nearest,0);
    Mat row1_mat = cvarrToMat(row1);
    Mat nearest_mat = cvarrToMat(nearest);
    Mat dist;
    result=knn->findNearest(row1_mat,K,noArray(),nearest_mat,dist);

    //std::cout<< "min distance = " << dist <<std::endl;
    //std::cout<< "nearest_mat= "<< nearest_mat <<std::endl;
    //std::cout<<"nearest label = ";
    for(int i=0;i<10;++i)
    {
        knn_result.min_distance[i] = dist.at<float>(0,i);  //usually, dist.at<float>(0,0) < 300;
        knn_result.nearest_label[i] = nearest_mat.at<float>(0,i);
        //std::cout<<int(knn_result.nearest_label[i])<<"; ";
    }
    //std::cout<<std::endl;
    knn_result.mat_distance = dist.clone();
    knn_result.mat_nearest = nearest_mat.clone();

    int accuracy=0;
	for(int i=0;i<K;i++)
	{
        if( nearest->data.fl[i] == result )
                    accuracy++;
	}
	float pre=100*((float)accuracy/(float)K);
	if(showResult==1)
	{
		//printf("|\t%.0f\t| \t%.2f%%  \t| \t%d of %d \t| \n",result,pre,accuracy,K);
	}

    knn_result.classResult = result;
    knn_result.precisionRatio = pre;
    knn_result.accuracyAmount = accuracy;

    cvReleaseImage(&img32);
    cvReleaseImageHeader(&img32);
    cvReleaseMat(&nearest);
	return result;
}

void basicOCR::test()
{
	IplImage* src_image;
	IplImage prs_image;
	//CvMat row,data;
	char file[255];
	int i,j;
	int error=0;
	int testCount=0;
	for(i =0; i<classes; i++)
	{
		for( j = 50; j< 50+train_samples; j++)
		{
			
			sprintf(file,"%s%d/%d%d.pbm",file_path, i, i , j);
			src_image = cvLoadImage(file,0);
			if(!src_image)
			{
				printf("Error: Cant load image %s\n", file);
				//exit(-1);
			}
			//process file
			prs_image = preprocessing(src_image, size, size);
			float r=classify(&prs_image,0);
			if((int)r!=i)
				error++;
			
			testCount++;
		}
	}
	float totalerror=100*(float)error/(float)testCount;
}


//initial
basicOCR::basicOCR(char* baseDir)
{
    knn = KNearest::create();
    knn->setAlgorithmType(KNearest::BRUTE_FORCE);

    sprintf(file_path , "%s/OCR/",baseDir);
    //sample number
    train_samples = 100; //50
    classes= 10;
	size=40;

	trainData = cvCreateMat(train_samples*classes, size*size, CV_32FC1);
	trainClasses = cvCreateMat(train_samples*classes, 1, CV_32FC1);

    printf("get data ...\n");
	getData();
	
	train();
}


