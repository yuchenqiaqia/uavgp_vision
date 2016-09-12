#include "declare.h"


extern bool copyIsRunningFlag;
extern bool rawImgHasCopiedOut;
extern Mat g_rawImage;
VideoCapture capture;

////Opencv相机采集线程
void* CameraCapture(void*)
{
    int rate = 15;
    ros::NodeHandle rawImgPubNode;
    image_transport::ImageTransport it(rawImgPubNode);
    image_transport::Publisher pub;
    pub = it.advertise("camera/raw_image",  1 );
    ros::Rate loop_rate( rate );

    while ( rawImgPubNode.ok() )
    {
        Mat captureImg;
        capture>>captureImg;
        copyIsRunningFlag = true;
        captureImg.copyTo(g_rawImage);
        copyIsRunningFlag = false;
        rawImgHasCopiedOut = false;
        usleep(1000);


        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", captureImg).toImageMsg();
        //说明要发布的消息对象
         pub.publish(msg);
         loop_rate.sleep();

    }
}

////创建Opencv相机采集线程
void CreateCameraCaptureThread()
{
    pthread_t captureThread;
    int ret=pthread_create(&captureThread,NULL,CameraCapture,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create camera capture thread error!..\n");
        exit (1);
    }
    return;
}

////Opencv方式打开UVC摄像头
int OpenCameraByOpencv(int cameraNo)
{
    //opencv方式打开相机
    capture.open(cameraNo);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    double fps = capture.get(CV_CAP_PROP_FPS);

    if(false == capture.isOpened())
    {
        cout<<"Open camera failed..."<<endl;
        return -1;
    }
    CreateCameraCaptureThread();
    return 1;
}

////打开相机
void OpenCamera(int openCameraMode, int cameraNo)
{
    //创建一个新线程用于捕获键盘输入
    //OpenGetKeyThread();

    //if (MVMODE == openCameraMode)
    //{
    //    ////open camera with SDK
    //    if( -1 == OpenCameraWithMvSDK( ) )
    //        exit(-1);
    //}

    if (POINTGRAYCAMERA == openCameraMode)
    {
        ////open camera with pg sdk
        //if( -1 == OpenPointGrayCamera( ) )
        //     exit(-1);
        return;
     }

    else if (OPENCV_VIDEOCAPTURE == openCameraMode)
    {
        ////open camera with opencv
        if( -1 == OpenCameraByOpencv( cameraNo ) )
             exit(-1);
     }
    else
    {
        //read offline data
        if( -1 == ReadOfflineData(  ) )
             exit(-1);
    }
    return;
}
