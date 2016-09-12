#include <opencv2/opencv.hpp>
using namespace cv;

//获取图像roi区域的平均值
double GetROI_AverageVal( Mat src, Point point, int channel, int radius)
{
    Mat src_roi;
    int rect_x1=0;
    int rect_y1=0;
    int rect_x2=0;
    int rect_y2=0;
    rect_x1 = point.x-radius;
    rect_y1 = point.y-radius;
    rect_x2 = point.x+radius;
    rect_y2 = point.y+radius;

    //防止越界
    if(rect_x1<0)
    {
        rect_x1 = 0;
    }
    if(rect_y1<0)
    {
        rect_y1 = 0;
    }
    if(rect_x2 > src.cols-1)
    {
        rect_x2 = src.cols-1;
    }
    if(rect_y2 > src.rows-1)
    {
        rect_y2 = src.rows-1;
    }

    //选取roi区域
    src_roi = src(Rect(rect_x1,rect_y1, rect_x2 - rect_x1,rect_y2 - rect_y1));

    //统计roi区域像素值的平均值
    double sum = 0.0;
    int count = 0;
    for(int i=0; i<src_roi.rows; i++)
    {
        for(int j=0;j<src_roi.cols;j++)
        {
            double temp;
            if( CV_8UC1 == src.type() )
            {
                //8位单通道uchar图像
                temp = double( src_roi.at<uchar>(i,j) );
            }
            else if( CV_16SC1 == src.type() )
            {
                //8位单通道ushort图像
                temp = double( src_roi.at<ushort>(i,j) );
            }
            else if( CV_8UC3 == src.type() )
            {
                //8位3通道uchar型图像
                Vec3b temp3b = src_roi.at<Vec3b>(i,j);
                temp = double(temp3b[channel]);
            }
            else if( CV_32FC3 == src.type() )
            {
                //32位3通道float型图像
                Vec3f temp3f = src_roi.at<Vec3f>(i,j);
                temp = double(temp3f[channel]);
            }
            if( fabs(temp) > 0.0001)
            {
                //只统计不为零像素点
                sum += temp;
                count++;
            }
        }
    }
    if( count>0 )
        return sum/count;
    else
        return 0;
}
