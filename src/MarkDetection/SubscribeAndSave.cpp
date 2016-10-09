/*
 * @file	: .cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"
#include "StereoMatching.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>  //IMU
#include <geometry_msgs/Vector3Stamped.h>       //velocity
#define FRAME_AMOUNT_OF_VIDEO  (18*60*15)
#define SUBSCRIBE_RATE  30

static char baseDir[1000] = {0};
static VideoWriter resultVideo;
static VideoWriter rawVideo;
static unsigned int rawVideoNo = 0;
static unsigned int resultVideoNo = 0;
static int rawImageNo = 0;
static int resultImageNo = 0;

Mat pointgray_raw_image;
////回调函数：订阅PointGray相机图像
static void RawDataSubThreadCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //Mat pointgray_raw_image;
    try
    {
        pointgray_raw_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if( 1 == ( (rawImageNo+1)%FRAME_AMOUNT_OF_VIDEO) )
    {
        char raw_video_name[1000];
        sprintf(raw_video_name,"%s/raw/raw-%06d.avi",baseDir,rawVideoNo);
        rawVideo.open( raw_video_name, CV_FOURCC('M', 'P', '4', '2'), 18.0, Size(pointgray_raw_image.cols, pointgray_raw_image.rows),true );  // CV_FOURCC('M', 'P', '4', '2') ;  CV_FOURCC('D', 'I', 'V', 'X')
        rawVideoNo++;
    }
    rawVideo<<pointgray_raw_image;

    printf("sub_raw=%d\n", rawImageNo);
    rawImageNo++;
}

//子线程：订阅相机图像
static void* RawDataSubThread(void*)
{   
    ros::NodeHandle raw_data_sub_node;
    image_transport::ImageTransport raw_it(raw_data_sub_node);
    image_transport::Subscriber raw_sub = raw_it.subscribe("vision/camera_image", 1, RawDataSubThreadCallback);
    printf("wait for images ...\n");
    //ros::spin();
    int rate = SUBSCRIBE_RATE;
    ros::Rate raw_sub_thread_loop_rate( rate );
    while( raw_data_sub_node.ok() )
    {
        ros::spinOnce();
        raw_sub_thread_loop_rate.sleep();
    }
}


////回调函数：订阅结果图像
static void ResultDataSubThreadCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat subscribed_image;
    try
    {
        subscribed_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if( 1 == ( (resultImageNo+1)%FRAME_AMOUNT_OF_VIDEO) )
    {
        char result_video_name[1000];
        sprintf(result_video_name,"%s/result/result-%06d.avi",baseDir,resultVideoNo);
        resultVideo.open( result_video_name, CV_FOURCC('M', 'P', '4', '2'), 10.0, Size(subscribed_image.cols, subscribed_image.rows),true );
        resultVideoNo++;
    }
    resultVideo<<subscribed_image;
    //imshow("result_data_subscriber", subscribed_image);
    //int c = waitKey(1);
    printf("sub_result=%d\n", resultImageNo);
    resultImageNo++;
}

//子线程：订阅结果
static void* ResultDataSubThread(void*)
{
    ros::NodeHandle result_data_sub_node;
    image_transport::ImageTransport reult_it(result_data_sub_node);
    image_transport::Subscriber reult_sub = reult_it.subscribe("vision/result_image", 1, ResultDataSubThreadCallback);
    printf("wait for images ...\n");
    ros::spin();
    /*
    int rate = SUBSCRIBE_RATE;
    ros::Rate result_sub_thread_loop_rate( rate );
    while( result_data_sub_node.ok() )
    {
        ros::spinOnce();
        //result_sub_thread_loop_rate.sleep();
    }
    */
}



//创建ros消息订阅的各个线程
static void CreateSubAndSaveThread(void)
{
    //2. 为保存处理结果创建一个新的线程
    pthread_t thread_result_data_sub;
    int ret=pthread_create(&thread_result_data_sub,NULL,ResultDataSubThread,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create save result thread error!..\n");
        exit (-1);
    }

/*
    //1. 为保存原始图像创建一个新的线程
    pthread_t thread_raw_data_sub;
    int ret;
    ret=pthread_create(&thread_raw_data_sub,NULL,RawDataSubThread,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create save raw data thread error!..\n");
        exit (-1);
    }
    //3. 接受guidance消息创建线程
    pthread_t thread_guidance_sub;
    ret=pthread_create(&thread_guidance_sub,NULL,GuidanceDataSubThread,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create guidance subscribe thread error!..\n");
        exit (-1);
    }
*/

    return;
}



//创建文件夹及保存数据子线程
int CreatSaveDir (char* dir , bool saveImgFlag)
{
    if (NULL == dir)
        return -1;

    memcpy(baseDir,dir,sizeof(baseDir));
    if( -1 == CreateDir(baseDir))
        return (-1);
    memcpy(dir, baseDir, sizeof(baseDir) );

    if ( false == saveImgFlag )
        return 1;

    //CreateSubAndSaveThread( );
    return 1;
}
