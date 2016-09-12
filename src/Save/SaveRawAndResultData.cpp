#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;

bool threadEnd0 = false;
bool threadEnd1 = false;

int rawImageNo = 0;
void RawDataSubThreadCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
        Mat subscribed_image;
        subscribed_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        resize(subscribed_image,subscribed_image,Size(800,600));

        char text[50];
        sprintf(text,"raw image");
        putText(subscribed_image,text,Point(0,25),CV_FONT_HERSHEY_COMPLEX_SMALL,1.5,Scalar(0,0,255),2);
        //imshow("raw_data_subscriber", subscribed_image);
        //int c = waitKey(1);
        ////按键‘q’或‘Q’退出
        //if ( 113 == c )
        //    exit(1);
        printf("rawSubNo=%d\n", rawImageNo);
        rawImageNo++;

        threadEnd0 = false;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int resultImageNo = 0;
void ResultDataSubThreadCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
        Mat subscribed_image;
        subscribed_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        char text[50];
        sprintf(text,"result image");
        putText(subscribed_image,text,Point(0,25),CV_FONT_HERSHEY_COMPLEX_SMALL,1.5,Scalar(0,0,255),2);
        //imshow("result_data_subscriber", subscribed_image);
        //int c = waitKey(1);
        ////按键‘q’或‘Q’退出
        //if ( 113 == c )
        //    exit(1);
        printf("resultSubNo=%d\n", resultImageNo);
        resultImageNo++;

        threadEnd1 = false;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void* RawDataSubThread(void*)
{
    ros::NodeHandle raw_data_sub_node;
    image_transport::ImageTransport raw_it(raw_data_sub_node);
    image_transport::Subscriber raw_sub = raw_it.subscribe("vision/camera_image", 1, RawDataSubThreadCallback);
    printf("wait for images ...\n");
    //ros::spin();
    int rate = 300;
    ros::Rate raw_sub_thread_loop_rate( rate );
    while(1)
    {
        ros::spinOnce();
        raw_sub_thread_loop_rate.sleep();
    }
}

void* ResultDtaSubThread(void*)
{
    ros::NodeHandle result_data_sub_node;
    image_transport::ImageTransport reult_it(result_data_sub_node);
    image_transport::Subscriber reult_sub = reult_it.subscribe("vision/result_image", 1, ResultDataSubThreadCallback);
    printf("wait for images ...\n");
    //ros::spin();
    int rate = 300;
    ros::Rate result_sub_thread_loop_rate( rate );
    while(1)
    {
        ros::spinOnce();
        result_sub_thread_loop_rate.sleep();
    }
}



//创建ros消息发布的各个线程
static void CreateSubAndSaveThread(void)
{
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

    //2. 为保存处理结果创建一个新的线程
    pthread_t thread_result_data_sub;
    ret=pthread_create(&thread_result_data_sub,NULL,ResultDtaSubThread,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create save result thread error!..\n");
        exit (-1);
    }
    return;
}

//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  CreateSubAndSaveThread( );
  usleep(2000000);

  while(1)
  {
      usleep(500000);
      if(true == threadEnd0 || true == threadEnd1)
          exit(1);

       threadEnd0 = true;
       threadEnd1 = true;
  }
}

