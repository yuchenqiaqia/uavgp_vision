/*
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */
#include <iostream>
//#include <string>
//#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/opencv.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include<pcl/visualization/pcl_visualizer.h>
//#include<pcl/common/common.h>
//#include<sensor_msgs/PointCloud2.h>

using namespace std;
using namespace cv;
using namespace pcl;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
}
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    
}

//PointCloud<PointXYZRGB> cloud;
static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
static pcl::PointCloud<pcl::PointXYZRGB>& cloud = *cloud_ptr;

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

int PoinCloudProcess(Mat& depth_img, Mat& xyz_img, Mat& gray_img)
{
    static pcl::visualization::CloudViewer viewer ("Cloud Viewer");

    //create point cloud
    cloud.width = depth_img.cols;
    cloud.height = depth_img.rows;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for(int i=0; i<depth_img.rows; i++)
    {
        for(int j=0; j<depth_img.cols; j++)
        {
                cloud[i*cloud.width+j].x = xyz_img.at<Vec3f>(i,j)[0];
                cloud[i*cloud.width+j].y = xyz_img.at<Vec3f>(i,j)[1];
                cloud[i*cloud.width+j].z = xyz_img.at<Vec3f>(i,j)[2];
                cloud[i*cloud.width+j].b = (uint8_t)gray_img.at<uchar>(i, j);
                cloud[i*cloud.width+j].g = (uint8_t)gray_img.at<uchar>(i, j);
                cloud[i*cloud.width+j].r = (uint8_t)gray_img.at<uchar>(i, j);
        }
    }

    //pcl::io::savePCDFileBinaryCompressed("/home/sia/test_pcdc.pcd",cloud);
    static bool imgNo = 0;

    //pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;// 创建滤波器
    //outrem.setInputCloud( cloud_ptr );                    //设置输入点云
    //outrem.setRadiusSearch( 0.8 );                        //设置在0.8半径的范围内找邻近点
    //outrem.setMinNeighborsInRadius( 5 );      //设置查询点的邻近点集数小于2的删除
    //outrem.filter (*cloud_filtered);                    //执行条件滤波，存储结果到cloud_filtered

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;// 创建滤波器对象
    //sor.setInputCloud( cloud_ptr );                      //设置呆滤波的点云
    //sor.setMeanK(50);                                   //设置在进行统计时考虑查询点邻近点数
    //sor.setStddevMulThresh(1.0);                //设置判断是否为离群点的阈值
    //sor.filter(*cloud_filtered);                       //执行滤波处理保存内点到cloud_filtered
    //viewer.showCloud ( cloud_filtered );

    viewer.showCloud (cloud_ptr);
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    viewer.runOnVisualizationThread (viewerPsycho);

    imgNo++;
    return 1;
}


int PoinCloudSave(Mat& depth_img, Mat& xyz_img, Mat& gray_img)
{
        PointCloud<pcl::PointXYZRGB> cloud;

        //create point cloud
        cloud.width = depth_img.cols;
        cloud.height = depth_img.rows;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        for(int i=0; i<depth_img.rows; i++)
       {
            for(int j=0; j<depth_img.cols; j++)
            {
                cloud[i*cloud.width+j].x = xyz_img.at<Vec3f>(i,j)[0];
                cloud[i*cloud.width+j].y = xyz_img.at<Vec3f>(i,j)[1];
                cloud[i*cloud.width+j].z = xyz_img.at<Vec3f>(i,j)[2];
                cloud[i*cloud.width+j].b = (uint8_t)gray_img.at<uchar>(i, j);
                cloud[i*cloud.width+j].g = (uint8_t)gray_img.at<uchar>(i, j);
                cloud[i*cloud.width+j].r = (uint8_t)gray_img.at<uchar>(i, j);
            }
        }

    static int pcdNo = 0;
    char fileName[100];
    sprintf(fileName, "/home/sia/dji_guidance/pcd/test_pcd_%06d.pcd", pcdNo);
    pcl::io::savePCDFileBinaryCompressed(fileName,cloud);

    pcdNo++;
    return 1;
}


int user_data;

int PointCloudShow ( )
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_a;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    cv::Mat image = cv::imread("/home/sia/SKY.BMP");

    int rowNumber = image.rows;
    int colNumber = image.cols;

    cloud_a.width  = rowNumber;
    cloud_a.height = colNumber;
    cloud_a.points.resize(cloud_a.width*cloud_a.height);

    cv::Mat_<cv::Vec3b>::iterator it    = image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator itend = image.end<cv::Vec3b>();

    for(unsigned int i=0; i<cloud_a.points.size(); ++i)
    {
        cloud_a.points[i].x = 1024*rand()/(RAND_MAX+1.0f);
        cloud_a.points[i].y = 1024*rand()/(RAND_MAX+1.0f);
        cloud_a.points[i].z = 1024*rand()/(RAND_MAX+1.0f);

        cloud_a.points[i].r = (int) (*it)[2];
        cloud_a.points[i].g = (int) (*it)[1];
        cloud_a.points[i].b = (int) (*it)[0];

        ++it;
    }

    //*cloud = cloud_a;

    char filename[100];
    sprintf(filename, "/home/sia/dji_guidance/pcd/test_pcd_001000.pcd");
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename,*cloud)==-1)//*打开点云文件
   {
       PCL_ERROR("Couldn't read file test_pcd.pcd\n");
       return(-1);
   }

    //static pcl::visualization::CloudViewer viewer ("Cloud Viewer");
    //viewer.showCloud(cloud);
    //viewer.runOnVisualizationThreadOnce (viewerOneOff);
    //viewer.runOnVisualizationThread (viewerPsycho);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;// 创建滤波器对象
    sor.setInputCloud( cloud );                      //设置呆滤波的点云
    sor.setMeanK(2000);                                   //设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh(0.5);                //设置判断是否为离群点的阈值
    sor.filter(*cloud_filtered);                         //执行滤波处理保存内点到cloud_filtered

    //pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;// 创建滤波器
    //outrem.setInputCloud( cloud );                    //设置输入点云
    //outrem.setRadiusSearch( 20 );                        //设置在0.8半径的范围内找邻近点
    //outrem.setMinNeighborsInRadius( 100 );      //设置查询点的邻近点集数小于2的删除
    //outrem.filter (*cloud_filtered);                    //执行条件滤波，存储结果到cloud_filtered

    pcl::visualization::PCLVisualizer viewer2("PCLVisualizer");
    viewer2.initCameraParameters();

    int v1(0);
    viewer2.createViewPort(0.0, 0.0, 0.5, 1, v1);
    viewer2.setBackgroundColor(1.0, 0.5, 1.0,v1);
    viewer2.addText("Cloud before voxelgrid filtering", 10, 10,"v1 test", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_color(cloud);
    viewer2.addPointCloud<pcl::PointXYZRGB>(cloud, cloud_color, "cloud", v1);

    int v2(0);
    viewer2.createViewPort(0.5, 0.0, 1.0, 1, v2);
    viewer2.setBackgroundColor(1.0, 0.5, 1.0,v2);
    viewer2.addText("Cloud after voxelgrid filtering", 10, 10, "v2 test", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_afterfilter_color(cloud_filtered);
    viewer2.addPointCloud<pcl::PointXYZRGB>(cloud_filtered, cloud_afterfilter_color, "cloud_filtered", v2);

    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");


    while (!viewer2.wasStopped ())
    {
        viewer2.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
