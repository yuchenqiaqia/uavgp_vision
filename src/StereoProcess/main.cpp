#include "declare.h"
#include "StereoMatching.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>  //IMU
#include <geometry_msgs/Vector3Stamped.h>       //velocity
#define SUBSCRIBE_RATE  80

int MyImageSegmentation( Mat& input_xyz_img, Mat& input_gray_img );
void MystereoShow( StereoMatchingType& stereo_cam );
int PoinCloudProcess(Mat& depth_img, Mat& xyz_img, Mat& gray_img);
int PoinCloudSave(Mat& depth_img, Mat& xyz_img, Mat& gray_img);
int PointCloudShow ( );

StereoMatchingType stereo_cam_1;
Mat guidance_gray_image_left;
Mat guidance_gray_image_right;
Mat guidance_depth_image;
Mat depth8;
Mat pseudo_color_img;
Mat xyz_img;

//显示原始图像、深度灰度图、深度伪彩图
int ShowImages(Mat& gray_image_left, Mat& gray_image_right, Mat& depth8, Mat& pseudo_color_img)
{
    if ( !gray_image_left.data || !gray_image_right.data)
          return 0;

    static unsigned int showImgNo = 0;
    Mat gray_image_show;
    cvtColor(gray_image_left, gray_image_show, CV_GRAY2BGR);

    //显示中心点的三维坐标
    Point3f dis;
    dis.x = xyz_img.at<Vec3f>(xyz_img.rows/2, xyz_img.cols/2)[0];
    dis.y = xyz_img.at<Vec3f>(xyz_img.rows/2, xyz_img.cols/2)[1];
    dis.z = GetROI_AverageVal( xyz_img, Point(xyz_img.cols/2, xyz_img.rows/2), 2, 1);
    char str[50];
    sprintf(str,"%.2f,%.2f,%.2f", dis.x, dis.y, dis.z);
    Point center = Point(gray_image_show.cols/2, gray_image_show.rows/2);
    circle(gray_image_show, center, 5, Scalar(0,0,255), -1, 8);
    putText(gray_image_show, str, center, CV_FONT_HERSHEY_PLAIN, 1.3, Scalar(0,0,255), 2);

    //resize(guidance_gray_image_left, guidance_gray_image_left, Size(640,480));
    imshow("gray_image", gray_image_show);
    //resize(pseudo_color_img, pseudo_color_img, Size(640,480));
    imshow("pseudo_color", pseudo_color_img);
    //resize(depth8,depth8,Size(640,480));
    imshow("depth8", depth8);
    waitKey(1);

    /*
            char imgName[200];
            sprintf(imgName, "%s/guidance/depth_1_%06d.png", baseDir, showImgNo);
            //imwrite(imgName, guidance_depth_image);
            sprintf(imgName, "%s/guidance/gray_left_1_%06d.png", baseDir, showImgNo);
            imwrite(imgName, gray_image_left);
            sprintf(imgName, "%s/guidance/gray_right_1_%06d.png", baseDir, showImgNo);
            imwrite(imgName, gray_image_right);
    */
    showImgNo++;
    return 1;
}

////////回调函数：订阅深度图像, 并转为三维点云图像
static void GuidanceDepthImageSubCallback( const sensor_msgs::ImageConstPtr& msg )
{
    //ros图像转为opencv图像
    try
    {
        guidance_depth_image = cv_bridge::toCvCopy(msg, "mono16")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
    static unsigned int depthImgNo = 0;

    // 16位short型深度图像 转换为 8位灰度图像
    guidance_depth_image.convertTo(depth8, CV_8UC1);
    //cvtColor(depth8,depth8,CV_GRAY2BGR);

    // 16位short型深度图像 转换为 32位浮点型深度图像
    Mat realDisImg = Mat::zeros(guidance_depth_image.rows, guidance_depth_image.cols, CV_32F);
    for (int i=0;i<realDisImg.rows;++i)
    {
        ushort* data= guidance_depth_image.ptr<ushort>(i);
        for (int j=0;j<realDisImg.cols;++j)
        {
            realDisImg.at<float>(i,j) = data[j]/128.0;
        }
    }

    // 深度图像转换为三维点云图像
    DepthTo3D(realDisImg, xyz_img);
    //伪彩图
    ConvertToPseudoColor( xyz_img, pseudo_color_img );
    //PCL process
    //PoinCloudProcess(depth8, xyz_img, guidance_gray_image_left);

    stereo_cam_1.StereoMatching();
    if (!stereo_cam_1.mat_xyz.data)
        return;
    double distance = GetROI_AverageVal( stereo_cam_1.mat_xyz, Point(stereo_cam_1.mat_xyz.cols/2,stereo_cam_1.mat_xyz.rows/2), 2, 2);
    char str[50];
    sprintf(str, "%.3f",distance);
    Point center = Point(stereo_cam_1.img_pseudo_color.cols/2,stereo_cam_1.img_pseudo_color.rows/2);
    putText(stereo_cam_1.img_pseudo_color, str, center, CV_FONT_HERSHEY_PLAIN, 2.0, Scalar(0,0,255), 2, 8);
    circle(stereo_cam_1.img_pseudo_color, center, 5, Scalar(0,0,255), -1, 8);
    MystereoShow( stereo_cam_1 );

    MyImageSegmentation( stereo_cam_1.mat_xyz, stereo_cam_1.img1_raw );

    ShowImages(guidance_gray_image_left, guidance_gray_image_right, depth8, pseudo_color_img);
    //PCL process
    //PoinCloudProcess(stereo_cam_1.disp8_show, stereo_cam_1.mat_xyz, stereo_cam_1.img1_raw);
    depthImgNo++;
    return;
}

static void GuidanceGrayImageLeftSubCallback( const sensor_msgs::ImageConstPtr& msg )
{
    try
    {
        guidance_gray_image_left = cv_bridge::toCvCopy(msg, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
    guidance_gray_image_left.copyTo(stereo_cam_1.img1_raw);
    stereo_cam_1.left_img_has_copied = false;
    return;
}

static void GuidanceGrayImageRightSubCallback( const sensor_msgs::ImageConstPtr& msg )
{
    try
    {
        guidance_gray_image_right = cv_bridge::toCvCopy(msg, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
    guidance_gray_image_right.copyTo(stereo_cam_1.img2_raw);
    stereo_cam_1.right_img_has_copied = false;
    return;
}

GuidanceDistance ultrasonic;
GuidanceDistance obstacle_distance;
Point3f velocity;
//Guidance输出的超声波距离信息
static void GuidanceUltrasonicSubCallback( const sensor_msgs::LaserScan& msg)
{
    ultrasonic.vbus_1_distance = fabs(msg.ranges[1]);
    ultrasonic.vbus_1_reliability = msg.intensities[1];
    ultrasonic.vbus_2_distance = fabs(msg.ranges[2]);
    ultrasonic.vbus_2_reliability = msg.intensities[2];
    ultrasonic.vbus_3_distance = fabs(msg.ranges[3]);
    ultrasonic.vbus_3_reliability = msg.intensities[3];
    ultrasonic.vbus_4_distance = fabs(msg.ranges[4]);
    ultrasonic.vbus_4_reliability = msg.intensities[4];
    ultrasonic.vbus_5_distance = fabs(msg.ranges[0]);
    ultrasonic.vbus_5_reliability = msg.intensities[0];

    printf( "ultrasonic_1 = %.3f; reliability = %.1f\n", ultrasonic.vbus_1_distance, ultrasonic.vbus_1_reliability );
    printf( "obsta_dis_1 = %.3f\n", obstacle_distance.vbus_1_distance );
    //printf( "vx = %6.3f; vy = %6.3f; vz = %6.3f; \n\n", velocity.x, velocity.y, velocity.z );

    return;
}

//Guidance输出的障碍物距离信息
static void GuidanceObstacleDistanceSubCallback( const sensor_msgs::LaserScan& msg)
{
    obstacle_distance.vbus_1_distance = fabs(msg.ranges[1]);
    if (obstacle_distance.vbus_1_distance > 50)
        obstacle_distance.vbus_1_distance = 0;
    obstacle_distance.vbus_2_distance = fabs(msg.ranges[2]);
    if (obstacle_distance.vbus_2_distance > 50)
        obstacle_distance.vbus_2_distance = 0;
    obstacle_distance.vbus_3_distance = fabs(msg.ranges[3]);
    if (obstacle_distance.vbus_3_distance > 50)
        obstacle_distance.vbus_3_distance = 0;
    obstacle_distance.vbus_4_distance = fabs(msg.ranges[4]);
    if (obstacle_distance.vbus_4_distance > 50)
        obstacle_distance.vbus_4_distance = 0;
    obstacle_distance.vbus_5_distance = fabs(msg.ranges[0]);
    if (obstacle_distance.vbus_5_distance > 50)
        obstacle_distance.vbus_5_distance = 0;

    return;
}

//Guidance输出的速度信息
static void GuidanceVelocitySubCallback( const geometry_msgs::Vector3Stamped& msg)
{
    velocity.x = msg.vector.x;
    velocity.y = msg.vector.y;
    velocity.z = msg.vector.z;

    return;
}

//子线程：订阅Guidance深度图像、灰度图像、超声波,GuidanceDataSubThread
int main(int argc, char **argv)
{
    ros::init(argc, argv, "guidance_data_process");
    ros::NodeHandle guidance_data_sub_node;
    image_transport::ImageTransport guidance_it(guidance_data_sub_node);
    image_transport::Subscriber guidance_depth_img_sub = guidance_it.subscribe("/guidance/depth_image", 1, GuidanceDepthImageSubCallback);
    image_transport::Subscriber guidance_gray_img_left_sub = guidance_it.subscribe("/guidance/left_image", 1, GuidanceGrayImageLeftSubCallback);
    image_transport::Subscriber guidance_gray_img_right_sub = guidance_it.subscribe("/guidance/right_image", 1, GuidanceGrayImageRightSubCallback);
    ros::Subscriber  ultrasonic_sub = guidance_data_sub_node.subscribe("/guidance/ultrasonic", 1, GuidanceUltrasonicSubCallback);
    ros::Subscriber  obstacle_distance_sub = guidance_data_sub_node.subscribe("/guidance/obstacle_distance", 1, GuidanceObstacleDistanceSubCallback);
    ros::Subscriber  velocity_sub = guidance_data_sub_node.subscribe("/guidance/velocity", 1, GuidanceVelocitySubCallback);
    printf("wait for guidance images ...\n");

    ros::spin();
/*
    int rate = SUBSCRIBE_RATE;
    ros::Rate guidance_sub_thread_loop_rate( rate );
    while( guidance_data_sub_node.ok() )
    {
        ros::spinOnce();
        guidance_sub_thread_loop_rate.sleep();
    }
*/
}


