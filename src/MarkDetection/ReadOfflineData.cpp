/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"

extern bool copyIsRunningFlag;
extern bool rawImgHasCopiedOut;
extern Mat g_rawImage;
static VideoCapture capture;
static char baseDir[200] = "/home/sia/vision_input/20160722_1540_14";
//static char baseDir[200] = "/home/xiaobin/vision_input/20160722_1546_43";
//static char baseDir[200] = "/home/xiaobin/vision_input/20160722_1551_03";


void* OfflineDataCapture(void*)
{
    int n = 300;
    unsigned int captureNo = 0;
    unsigned int readVideoNo = 0;
    Mat captureImg;

    while ( 1 )
    {
        if( 1 == ( (captureNo+1)%n) )
        {
            char read_video_name[200];
            sprintf(read_video_name,"%s/raw/raw-%06d.avi",baseDir,readVideoNo);
            capture.release();
            capture.open( read_video_name );

            if(! capture.isOpened() )
            {
                cout<<"Open offline video failed..."<<endl;
                exit(0);
            }
            readVideoNo++;
        }

        while( false == rawImgHasCopiedOut)
            usleep(2000);
        capture>>captureImg;
        copyIsRunningFlag = true;
        captureImg.copyTo(g_rawImage);
        copyIsRunningFlag = false;
        rawImgHasCopiedOut = false;
        printf("CaptureNo = %d\n", captureNo);
        captureNo++;
    }
}

//read offline data
int ReadOfflineData(  )
{
    pthread_t captureThread;
    int ret=pthread_create(&captureThread,NULL,OfflineDataCapture,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create OfflineData Capture thread error!..\n");
        exit (1);
    }
    return 1;
}

