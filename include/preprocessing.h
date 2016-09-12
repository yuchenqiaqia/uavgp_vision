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
#include <stdio.h>
#include <ctype.h>
#endif
//#include </usr/local/include/opencv2/opencv.hpp>

IplImage preprocessing(IplImage* imgSrc,int new_width, int new_height);
