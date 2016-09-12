#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
        Mat subscribed_image;
        subscribed_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        resize(subscribed_image,subscribed_image,Size(800,600));

        char text[50];
        sprintf(text,"Subscribed Image");
        putText(subscribed_image,text,Point(0,25),CV_FONT_HERSHEY_COMPLEX_SMALL,1.5,Scalar(0,0,255),2);
        imshow("image_subscriber", subscribed_image);
        waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("image_subscriber");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("vision/image", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("camera/raw_image", 1, imageCallback);
  ros::spin();
}

