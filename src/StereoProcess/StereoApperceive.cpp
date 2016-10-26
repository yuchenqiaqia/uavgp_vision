/*
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */
#include "StereoMatching.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
using namespace std;
using namespace cv;


//相机内参
static double fx=247.357576, fy=247.390025, cx=153.295063, cy=116.893925;
//guidance输出的深度图像转为三维点云图像
void DepthTo3D(Mat& depth_img, Mat& xyz_img)
{
    xyz_img = Mat::zeros(depth_img.rows, depth_img.cols, CV_32FC3);
    for (int i=0;i<depth_img.rows;++i)
    {
        float* dis_data= depth_img.ptr<float>(i);
        for (int j=0;j<depth_img.cols;++j)
        {
            xyz_img.at<Vec3f>(i,j)[0] = dis_data[j]*(j - cx)/fx;
            xyz_img.at<Vec3f>(i,j)[1] = dis_data[j]*(i - cy)/fy;
            xyz_img.at<Vec3f>(i,j)[2] = dis_data[j];
        }
    }

    return;
}

//深度图转为深度伪彩图
double distanceZ_min = 0.52;
double distanceZ_max = 8.0;
double dif_value = distanceZ_max - distanceZ_min;
void ConvertToPseudoColor( Mat& mat_xyz, Mat& img_pseudo_color )
{
    Mat pseudo_color = Mat::zeros(mat_xyz.rows, mat_xyz.cols, CV_8UC3);

    for(int r = 0; r < mat_xyz.rows; r++)
    {
        uchar* data_pseudo=pseudo_color.ptr<uchar>(r);
        for(int c = 0; c < mat_xyz.cols*3; c=c+3)
        {
                Vec3f point_xyz = mat_xyz.at<Vec3f>(r, c/3);
                float X = point_xyz[0];
                float Y = point_xyz[1];
                float Z = point_xyz[2];
                float distanceZ = Z;
                if( distanceZ >= distanceZ_min && distanceZ <= distanceZ_max)
                {
                    int pseudo_raw_value=int( ( (distanceZ-distanceZ_min)/dif_value)*510 );
                    if( pseudo_raw_value <= 255 && pseudo_raw_value > 0 )
                    {
                        data_pseudo[c] = 0;
                        data_pseudo[c+1] = pseudo_raw_value;
                        data_pseudo[c+2] = 255-pseudo_raw_value;
                    }
                    else if(pseudo_raw_value > 255 && pseudo_raw_value < 510)
                    {
                        data_pseudo[c+1] = 510-pseudo_raw_value;
                        data_pseudo[c] = 255-data_pseudo[c+1];
                        data_pseudo[c+2] = 0;
                    }
                    else
                    {
                        data_pseudo[c] = 180;	//180
                        data_pseudo[c+1] =180;
                        data_pseudo[c+2] =180;
                    }
                }
                else	if( distanceZ<distanceZ_min && distanceZ > 0.4)
                {
                    data_pseudo[c] = 0;
                    data_pseudo[c+1] = 0;
                    data_pseudo[c+2] = 255;
                }
                else	if( distanceZ > distanceZ_max && distanceZ < 50)
                {
                    data_pseudo[c] = 255;
                    data_pseudo[c+1] = 0;
                    data_pseudo[c+2] = 0;
                }
                else
                {
                    data_pseudo[c]=180;
                    data_pseudo[c+1]=180;
                    data_pseudo[c+2]=180;
                }
        }
    }
    pseudo_color.copyTo(img_pseudo_color);
    return;
}


void MystereoShow( StereoMatchingType& stereo_cam )
{
    if (stereo_cam.img_pseudo_color.data)
    {
            imshow("left raw", stereo_cam.img1_raw);
            imshow("right raw", stereo_cam.img2_raw);
            imshow("my pseudo color img", stereo_cam.img_pseudo_color);
            imshow("my disp8", stereo_cam.disp8);
            int c = waitKey(1);
            if (c>0)
            {
                char filename[100];
                static int imgNo = 0;
                sprintf(filename, "/home/sia/dji_guidance/image/left-%d.jpg", imgNo);
                imwrite(filename, stereo_cam.img1_raw);
                sprintf(filename, "/home/sia/dji_guidance/image/right-%d.jpg", imgNo);
                imwrite(filename, stereo_cam.img2_raw);
                imgNo++;
            }
    }
    return;
}
