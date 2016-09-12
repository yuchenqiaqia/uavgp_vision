#include "../include/CameraApi.h" //相机SDK的API头文件

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <unistd.h>

using namespace cv;
using namespace std;

//计算要设置的曝光时间
double AutoSetExposureTime( Mat currentImg, const double currentExposureTime,  const double hsv_h, double* currentHSV_v);

static bool autoSetExposureTimeFlag = true;
double exposureTimeSet;
double  currentExposureTime;
double set_hsv_v = 200;

extern char pressedKey[20];
extern bool cameraThreadExitFlag;
extern bool mainThreadExitFlag;
extern bool copyIsRunningFlag;
extern bool rawImgHasCopiedOut;
extern Mat g_rawImage;
unsigned char * g_pRgbBuffer;     //处理后数据缓存区

////迈德威视sdk变量
int                                      iCameraCounts = 1;
int                                      iStatus=-1;
tSdkCameraDevInfo        tCameraEnumList;
int                                      hCamera;
tSdkCameraCapbility      tCapability;      //设备描述信息
tSdkFrameHead               sFrameInfo;
BYTE*                                pbyBuffer;
int                                      iDisplayFrames = 10000;
IplImage *                         iplImage = NULL;
int                                      channel=3;
tSdkFrameHead   m_sFrInfo;		//用于保存当前图像帧的帧头信息
CameraSdkStatus status;

void* MvCameraCaptureThread(void*)
{
    CameraSdkInit(1);
    //cout<<"CameraSdkInit..."<<endl;

    //枚举设备，并建立设备列表
    CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    //cout<<"CameraEnumerateDevice..."<<endl;

    //没有连接设备
    if(iCameraCounts==0)
    {
        exit(-1);
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
    //cout<<"CameraInit..."<<endl;
    //初始化失败
    if(iStatus!=CAMERA_STATUS_SUCCESS)
    {
            exit(-1);
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    //cout<<"CameraGetCapability..."<<endl;
    CameraGetCapability(hCamera,&tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    ////"通知SDK内部建该相机的属性页面";
    //char cameraName[50];
    //sprintf(cameraName,"mindvision camera");
    //CameraCreateSettingPage(hCamera,NULL,cameraName,NULL,NULL,0);

    //让SDK进入工作模式，开始接收来自相机发送的图像数据
    //cout<<"CameraPlay..."<<endl;
    CameraPlay(hCamera);

    //TRUE显示相机配置界面。FALSE则隐藏。
    //CameraShowSettingPage(hCamera,TRUE);

    //设置相机的曝光模式,  TRUE：自动曝光，FALSE：手动曝光
    CameraSetAeState(hCamera,  FALSE);

    BOOL aeState;
    CameraGetAeState(hCamera, &aeState);
    if ( FALSE == aeState )
    {
        //设置曝光时间，单位：微秒
        //CameraSetExposureTime(hCamera,15000);
        CameraGetExposureTime(hCamera, &currentExposureTime);
    }

    // 一般情况，0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
    CameraSetTriggerMode(hCamera, 0);

    if(tCapability.sIspCapacity.bMonoSensor)
    {
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

    unsigned int imgId = 0;
    while( 1 )
    {
        CameraSoftTrigger(hCamera);
        //usleep(10000);

        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,500) == CAMERA_STATUS_SUCCESS)
		{
            status = CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

            //分辨率改变了，则刷新背景
            if (m_sFrInfo.iWidth != sFrameInfo.iWidth || m_sFrInfo.iHeight != sFrameInfo.iHeight)
            {
                m_sFrInfo.iWidth = sFrameInfo.iWidth;
                m_sFrInfo.iHeight = sFrameInfo.iHeight;
            }

            if (iplImage)
                cvReleaseImageHeader(&iplImage);

            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
            cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
            //cvShowImage("OpenCV Demo",iplImage);

            Mat src(iplImage);
            if ( false == autoSetExposureTimeFlag )
            {
                copyIsRunningFlag = true;
                src.copyTo(g_rawImage);
                copyIsRunningFlag = false;
                rawImgHasCopiedOut = false;
            }
            //在成功调用CameraGetImageBuffer后，调用CameraReleaseImageBuffer来释放获得的buffer。
			CameraReleaseImageBuffer(hCamera,pbyBuffer);
            //memcpy(&m_sFrInfo,&sFrameInfo,sizeof(tSdkFrameHead));

            resize(src,src,Size(640,480));
            imshow("src",src);
            int c = waitKey(1);
            if (c>0)
                break;

            //printf("set_hsv_v=%f\n", set_hsv_v);
            double hsv_v = 0;
            //计算要设置的曝光时间
            exposureTimeSet = AutoSetExposureTime(src, currentExposureTime, set_hsv_v, &hsv_v);

            if( (true == autoSetExposureTimeFlag && 'e' == pressedKey[0])  && imgId>10 )
            {
                //设置曝光时间，单位：微秒
                CameraSetExposureTime(hCamera, exposureTimeSet);
                CameraGetExposureTime(hCamera, &currentExposureTime);
                //printf("currentExposureTime=%f\n", currentExposureTime);
                if ( fabs(set_hsv_v - hsv_v) < 0.8)
                    autoSetExposureTimeFlag = false;
            }
            imgId++;   
        }
        usleep(2000);       
    }
    CameraUnInit(hCamera);
    //反初始化后再free
    free(g_pRgbBuffer);
}

////MV sdk 方式打开(迈德威视)相机
int OpenCameraWithMvSDK(void)
{

    //创建一个新的线程
    pthread_t thread_openCam;
    int ret;
    ret=pthread_create(&thread_openCam,NULL,MvCameraCaptureThread,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create ros image publish thread error!..\n");
        return -1;
    }

    return 1;
}
