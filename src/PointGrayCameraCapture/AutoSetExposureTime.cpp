/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "DetectRectToGetImageLightness.h"


////获取键盘字符线程
char pressedKey[20] = {-1};
void* getKey(void*)
{
    while(1)
    {
        pressedKey[0] = 'a';
        if( 'e' != pressedKey[0] )
        {
           printf("\nPress the key 'e' to start auto set exposure time: \n");
           int c = scanf("%s",pressedKey);
           printf("pressedKey=%s\n", pressedKey);
        }
        else
        {
            printf("Start setting exposure time...\n");
            break;
        }
    break;
    }
}

void OpenGetKeyThread()
{
    pthread_t getKeyBoard;
    int ret=pthread_create(&getKeyBoard,NULL,getKey,NULL);
    if(ret!=0)
    {
        //获取键盘字符线程创建失败
        printf ("Create get KeyBoard thread error!..\n");
        exit (1);
    }
}

double HSV_h_value;
double HSV_s_value;
double HSV_v_value;
double HSVCenterROI( Mat& srct )
{
    Mat hsv;
    cvtColor(srct,hsv,CV_BGR2HSV_FULL);
    Mat srctShow;
    srct.copyTo(srctShow);
    int radius = srctShow.rows/8;
    rectangle( srctShow,Point2i(srctShow.cols/2-radius,srctShow.rows/2-radius),Point2i(srctShow.cols/2+radius, srctShow.rows/2+radius),Scalar(0,255,0),2 );

    Rect rect=Rect(hsv.cols/2-radius, hsv.rows/2-radius,radius*2,radius*2);
    Mat ROI = hsv(rect);
    double HSV_h=0;
    double HSV_s=0;
    double HSV_v=0;

     for(unsigned int j=0;j < ROI.rows;j++)
     {
        uchar* data=ROI.ptr<uchar>(j);
        for(unsigned int p=0;p<ROI.cols*3;p=p+3)
        {
            HSV_h=HSV_h+data[p];
            HSV_s=HSV_s+data[p+1];
            HSV_v=HSV_v+data[p+2];
         }
      }

      HSV_h_value=HSV_h/(ROI.rows*ROI.cols);
      HSV_s_value=HSV_s/(ROI.rows*ROI.cols);
      HSV_v_value=HSV_v/(ROI.rows*ROI.cols);

      char text[100];
      sprintf(text,"[%4.1f,%4.1f,%4.1f]", HSV_h_value, HSV_s_value, HSV_v_value );
      Point2i text_center=Point2i( srctShow.cols/2-radius*1.5, srctShow.rows/2+radius*1.5 );
      putText(srctShow, text, text_center, CV_FONT_HERSHEY_PLAIN, 2.5, Scalar(0,0,255),3);
      resize(srctShow, srctShow, Size(640,480));
      imshow("CameraCapture",srctShow);
      int c = waitKey(1);
      if (113 == c)
          exit(1);
      return HSV_v_value;
}



double currentHSV_v = 0;
double CalculateExposureTime( Mat& currentImg, const double currentExposureTime,  double brightnessSet, const double rectAreaBrightness )
{
    //currentHSV_v = HSVCenterROI( currentImg );
    currentHSV_v = rectAreaBrightness;
    printf("HSV_v=%f\n", currentHSV_v);

    double exposureTimeSet ;
    double timeUnit = 0.05;     //单位：ms, 0.1
    double brightnessErr = currentHSV_v - brightnessSet;
    if(fabs(brightnessErr) >= 20)
    {
        if (currentExposureTime > 10)
        {
            //timeUnit = fabs(brightnessErr/100 * 5);
            timeUnit = timeUnit * 20.0;
        }
        else if (currentExposureTime >= 5 && currentExposureTime <= 10)
        {
            timeUnit = timeUnit * 10;
        }
        else if (currentExposureTime < 5 && currentExposureTime >= 1.5)
        {
            timeUnit = timeUnit * 5;
        }
        else if (currentExposureTime < 1.5 && currentExposureTime >= 0.5)
        {
            timeUnit = timeUnit * 1;
        }
        else if (currentExposureTime < 0.5 && currentExposureTime >= 0.1)
        {
            timeUnit = timeUnit * 0.5;
        }
        else
        {
            timeUnit = timeUnit * 0.1;
        }
    }
    else
    {
        if (currentExposureTime > 10)
        {
            timeUnit = timeUnit * 10.0;
        }
        else if (currentExposureTime >= 5 && currentExposureTime <= 10)
        {
            timeUnit = timeUnit * 2.0;
        }
        else if (currentExposureTime < 5 && currentExposureTime >= 1.5)
        {
            timeUnit = timeUnit * 1.0;
        }
        else if (currentExposureTime < 1.5 && currentExposureTime >= 0.5)
        {
            timeUnit = timeUnit * 0.5;
        }
        else if (currentExposureTime < 0.5 && currentExposureTime >= 0.1)
        {
            timeUnit = timeUnit * 0.25;
        }
        else
        {
            timeUnit = timeUnit * 0.05;
        }
    }

    if (brightnessErr >= -0.5 && brightnessErr <= 0.5)
    {
        exposureTimeSet = currentExposureTime;
    }
    else if(brightnessErr < -0.5)
    {
        exposureTimeSet = currentExposureTime + timeUnit;
    }
    else
    {
        exposureTimeSet = currentExposureTime - timeUnit;
    }

    return exposureTimeSet;
}
