/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */
#include "declare.h"
#include <opencv2/opencv.hpp>
#include "project_path_config.h"
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#define DISPLAYSCREEN 0
#define PRINTBOARD 1
#define FRAME_AMOUNT_OF_VIDEO  (10*60*10)
using namespace std;
using namespace cv;

class RawDataSaveType
{
public:
    RawDataSaveType();
    int CreatSaveDir(char* baseDir);
    void InitRawImgSubscriber();
    char baseDir[500];

private:
    void AttitudeSubCallBack(const geometry_msgs::TransformStamped::ConstPtr& att_msg);
    void SaveDataCallBack( const sensor_msgs::ImageConstPtr& msg );
    void camera_switch_cb(const std_msgs::Int32::ConstPtr& msg);
    void SaveResultImage( Mat& input_image );

    int targetType;
    Attitude3D attitude3d;
    VideoWriter resultVideo;
    ros::Subscriber attSub;
    std_msgs::Int32 camera_switch_data;
};

RawDataSaveType::RawDataSaveType()
{
    sprintf(baseDir,"%s", OCR_DIR_PATH);
    targetType = DISPLAYSCREEN;
}


void RawDataSaveType::AttitudeSubCallBack(const geometry_msgs::TransformStamped::ConstPtr& att_msg)
{
    attitude3d.roll = float(att_msg->transform.rotation.x);
    attitude3d.pitch = float(att_msg->transform.rotation.y);
    attitude3d.yaw = float(att_msg->transform.rotation.z);
    //printf("roll=%.2f; pit=%.2f; yaw=%.2f;\n",attitude3d.roll*180/3.14,attitude3d.pitch*180/3.14,attitude3d.yaw*180/3.14);
    return;
}

void RawDataSaveType::SaveDataCallBack( const sensor_msgs::ImageConstPtr& msg )
{
    Mat rawSaveImage;
    rawSaveImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (!rawSaveImage.data)
        return;
    RawDataSaveType::SaveResultImage( rawSaveImage );
    return;
}

/* subscribe camera_switch_data from state_machine offb_simulation_test node. */
void RawDataSaveType::camera_switch_cb(const std_msgs::Int32::ConstPtr& msg)
{
    camera_switch_data = *msg;
    if( 1 == camera_switch_data.data)
        targetType = DISPLAYSCREEN;
    if(2 == camera_switch_data.data)
        targetType = PRINTBOARD;
    //ROS_INFO("get camera_switch_data = %d",camera_switch_data.data);
}


void RawDataSaveType::InitRawImgSubscriber( )
{
    ros::NodeHandle imageSaveNode;
    image_transport::Subscriber rawImgSub;
    image_transport::ImageTransport imageSaveNode_it(imageSaveNode);//&RawDataSaveType::
    rawImgSub = imageSaveNode_it.subscribe("vision/camera_image", 1, &RawDataSaveType::SaveDataCallBack,this);
    attSub = imageSaveNode.subscribe("imu/attitude", 1, &RawDataSaveType::AttitudeSubCallBack,this);

    /* get camera_switch from state_machine(offb_simulation_test node). */
    camera_switch_data.data = 0;
    ros::Subscriber camera_switch_sub = imageSaveNode.subscribe("camera_switch", 1, &RawDataSaveType::camera_switch_cb,this);

    ros::spin();
}


void RawDataSaveType::SaveResultImage( Mat& input_image )
{
    static int resultImageNo = 0;
    static int resultVideoNo = 0;

    if( 1 == ( (resultImageNo+1)%FRAME_AMOUNT_OF_VIDEO) )
    {
        char result_video_name[500];
        sprintf(result_video_name,"%s/raw-%06d.avi",baseDir,resultVideoNo);
        resultVideo.open( result_video_name, CV_FOURCC('D', 'I', 'V', 'X'), 10.0, Size(input_image.cols, input_image.rows),true );
        resultVideoNo++;
    }
    resultVideo<<input_image;

    FILE* fp;
    char txtName[200];
    sprintf(txtName, "%s/attitude.txt", baseDir);
    fp = fopen(txtName,"a+");
    if (NULL == fp)
    {
          printf("Open txt failed ...\n");
          exit(-2);
    }
    fprintf(fp,"%d %f %f %f\n", resultImageNo, attitude3d.roll, attitude3d.pitch, attitude3d.yaw);
    fclose(fp);

    sprintf(txtName, "%s/switch_mode.txt", baseDir);
    fp = fopen(txtName,"a+");
    if (NULL == fp)
    {
          printf("Open txt failed ...\n");
          exit(-2);
    }
    fprintf(fp,"%d %d\n", resultImageNo, targetType);
    fclose(fp);

    //printf("sub_result=%d\n", resultImageNo);
    resultImageNo++;
    return;
}

int RawDataSaveType::CreatSaveDir(char* baseDir)
{
    int status = 0;
    char dir_name[500];

    sprintf(dir_name,"%s/output",baseDir);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        //cout<<"'output' folder already exists,no need to create..."<<endl;
    }
    else
    {
        cout<<"'output' folder has been created."<<endl;
    }
    memcpy(baseDir,dir_name,sizeof(dir_name));

    sprintf(dir_name,"%s/Save",baseDir);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        //cout<<"'Save' folder already exists,no need to create..."<<endl;
    }
    else
    {
        cout<<"'Save' folder has been created."<<endl;
    }
    memcpy(baseDir,dir_name,sizeof(dir_name));

    time_t  now;
    struct tm* timenow;
    time(&now);
    timenow = localtime(&now);
    sprintf(dir_name,"%s/%04d%02d%02d_%02d%02d_%02d",baseDir,timenow->tm_year+1900,timenow->tm_mon+1,timenow->tm_mday,timenow->tm_hour,timenow->tm_min,timenow->tm_sec);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        cout<<dir_name<<" forder already exists."<<endl;
    }
    else
    {
        cout<<dir_name<<" folder has been created."<<endl;
    }
    memcpy(baseDir,dir_name,sizeof(dir_name));

    return 1;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_raw_data_save");
    RawDataSaveType raw_data_save;
    raw_data_save.CreatSaveDir(raw_data_save.baseDir);
    raw_data_save.InitRawImgSubscriber();
    return 0;
}
