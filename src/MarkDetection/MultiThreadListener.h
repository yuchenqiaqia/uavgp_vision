/******************************************************************************

版权所有 (C), 2016, 中国科学院沈阳自动化研究所，一室，旋翼飞行机器人课题组

******************************************************************************
版 本 号   : v1.0
作	  者   : 肖斌
******************************************************************************/
#include "KNN_OCR.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <time.h>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <boost/thread.hpp>

using namespace std;
using namespace cv;

class MultiThreadListener
{
public:
    MultiThreadListener()
    {
            sub1 = mainVisionProcessNode.subscribe("chatter1", 1, &multiThreadListener::chatterCallback1,this);
            sub2 = mainVisionProcessNode.subscribe("chatter2", 1, &multiThreadListener::chatterCallback2,this);
    }
    void chatterCallback1(const std_msgs::String::ConstPtr& msg);
    void chatterCallback2(const std_msgs::String::ConstPtr& msg);

private:
    ros::NodeHandle mainVisionProcessNode;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;
};

#endif // MULTITHREADLISTENER_H
