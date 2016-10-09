/*
 * @file	: Publish.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"
#define FRAME_AMOUNT_OF_VIDEO  (10*60*10)

vector<VisionResult>  g_visionResult;
extern Attitude3D attitude3d;
int k;
extern Mat g_rectResultImg;
extern bool g_rectResultImgUpdated;
static char baseDir[1000];
static VideoWriter resultVideo;

void SaveResultImage( Mat& input_image )
{
    static int resultImageNo = 0;
    static int resultVideoNo = 0;

    if( 1 == ( (resultImageNo+1)%FRAME_AMOUNT_OF_VIDEO) )
    {
        char result_video_name[1000];
        sprintf(result_video_name,"%s/result/result-%06d.avi",baseDir,resultVideoNo);
        resultVideo.open( result_video_name, CV_FOURCC('M', 'P', '4', '2'), 10.0, Size(input_image.cols, input_image.rows),true );
        resultVideoNo++;
    }
    resultVideo<<input_image;
    //printf("sub_result=%d\n", resultImageNo);
    resultImageNo++;
}


//1. ros图像消息发布线程，小分辨率图像
void* RosImagePublishThread_1(void*)
{
    while( false == g_rectResultImgUpdated )
    {
        usleep(50000);
    }

    ros::NodeHandle smallImgPubNode;
    image_transport::ImageTransport it_0(smallImgPubNode);
    image_transport::Publisher pub2ground;
    pub2ground = it_0.advertise("vision/result_small_image",  1 );

    //ros::NodeHandle imgPubNode;
    //image_transport::ImageTransport it_1(imgPubNode);
    //image_transport::Publisher pub2save;
    //pub2save = it_1.advertise("vision/result_image",  1 );

    int rate = 10;
    ros::Rate loop_rate( rate );
    Mat img;
    Mat little_img;
    unsigned int imgNo = 0;

    while( smallImgPubNode.ok() )
    {
        //while( false == g_rectResultImgUpdated )
        //{
        //    usleep(10000);
        //}
        //if (true == g_rectResultImgUpdated)
        //{
            g_rectResultImg.copyTo(img);
            //g_rectResultImgUpdated = false;
            resize(img, little_img, Size(240,180));
        //}

        //sensor_msgs::ImagePtr msg_save = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        //pub2save.publish(msg_save);
        //ros::spinOnce();

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", little_img).toImageMsg();
        pub2ground.publish(msg);
        loop_rate.sleep();

        //printf("img-%d publish done!\n", imgNo);
        imgNo++;
    }
}


//2. ros图像消息发布线程, 标准640*480图像
void* RosImagePublishThread_2(void*)
{
    while( false == g_rectResultImgUpdated )
    {
        usleep(50000);
    }

    ros::NodeHandle imgPubNode;
    image_transport::ImageTransport it_1(imgPubNode);
    image_transport::Publisher pub2save;
    pub2save = it_1.advertise("vision/result_image",  1 );

    int rate = 10;
    ros::Rate loop_rate( rate );
    Mat img;
    Mat little_img;
    unsigned int imgNo = 0;

    while( false == g_rectResultImgUpdated )
    {
        usleep(500000);
    }

    while( imgPubNode.ok() )
    {
        //while( false == g_rectResultImgUpdated )
        //{
        //    usleep(10000);
        //}
        if (true == g_rectResultImgUpdated)
        {
            g_rectResultImg.copyTo(img);
            g_rectResultImgUpdated = false;
        }
        SaveResultImage( img );

        FILE* fp;
        char txtName[1000];
        sprintf(txtName, "%s/txt/all_result.txt", baseDir);
        fp = fopen(txtName,"a+");
        if (NULL == fp)
        {
             printf("Open txt failed ...\n");
             exit(-2);
         }
        fprintf(fp,"%d %d\n", imgNo, int(g_visionResult.size()));
        for(int i=0;i<(int)g_visionResult.size();++i)
        {
            fprintf(fp,"%d %f %f %f %f %f %f %f %f\n", g_visionResult[i].digitNo, g_visionResult[i].imagePos2D.x, g_visionResult[i].imagePos2D.y,
                                g_visionResult[i].cameraPos3D.x, g_visionResult[i].cameraPos3D.y, g_visionResult[i].cameraPos3D.z,
                                g_visionResult[i].negPos3D.x, -g_visionResult[i].negPos3D.y, -g_visionResult[i].negPos3D.z
                                );
        }
        fclose(fp);

        //sensor_msgs::ImagePtr msg_save = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        //pub2save.publish(msg_save);
        loop_rate.sleep();
        imgNo++;
    }
}


//创建ros消息发布的各个线程
void CreateRosPublishThread(const char* dir)
{
    memcpy(baseDir, dir, sizeof(baseDir));
    //1. 为ros图像消息发布创建一个新的线程
    pthread_t thread_imgPub_1;
    int ret;
    ret=pthread_create(&thread_imgPub_1,NULL,RosImagePublishThread_1,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create ros image publish thread 1 error!..\n");
        exit (1);
    }
    pthread_t thread_imgPub_2;
    ret=pthread_create(&thread_imgPub_2,NULL,RosImagePublishThread_2,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create ros image publish thread 2 error!..\n");
        exit (1);
    }

    return;
}
