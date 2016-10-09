/*
 * @file	: .cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"

//创建数据存储路径
int CreateDir(char* baseDir)
{
    int status = 0;
    char dir_name[1000];

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

    time_t  now;
    struct tm* timenow;
    time(&now);
    timenow = localtime(&now);
    sprintf(dir_name,"%s/%04d%02d%02d_%02d%02d_%02d",baseDir,timenow->tm_year+1900,timenow->tm_mon+1,timenow->tm_mday,timenow->tm_hour,timenow->tm_min,timenow->tm_sec);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        cout<<dir_name<<" forder created failed."<<endl;
        return -1;
    }
    else
    {
        cout<<dir_name<<" folder has been created."<<endl;
    }
    memcpy(baseDir,dir_name,sizeof(dir_name));

    sprintf(dir_name,"%s/raw",baseDir);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        cout<<"raw forder created failed."<<endl;
        return -1;
    }
    else
    {
        cout<<"'raw' folder has been created."<<endl;
    }

    sprintf(dir_name,"%s/result",baseDir);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        cout<<"result forder created failed."<<endl;
        return -1;
    }
    else
    {
        cout<<"'result' folder has been created."<<endl;
    }

    sprintf(dir_name,"%s/txt",baseDir);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        cout<<"txt forder created failed."<<endl;
        return -1;
    }
    else
    {
        cout<<"'txt' folder has been created."<<endl;
    }

    sprintf(dir_name,"%s/digit_image",baseDir);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        cout<<"digit_image forder created failed."<<endl;
        return -1;
    }
    else
    {
        cout<<"'digit_image' folder has been created."<<endl;
    }

    sprintf(dir_name,"%s/guidance",baseDir);
    status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == status)
    {
        cout<<"guidance forder created failed."<<endl;
        return -1;
    }
    else
    {
        cout<<"'guidance' folder has been created."<<endl;
    }
    return 1;
}


extern double time0,time1,time2;
//显示时间、帧率
void ShowTime(Mat& img, int k, float shrink)
{
    //每n帧更新一次时间显示
    int n = 2;
    static int imgFps = 0;
    if(0 == (k%n))
    {
        time2=((double)getTickCount()-time0)/getTickFrequency();
        imgFps = int( n/(time2-time1) );
    }
    char frameN[50];
    sprintf(frameN,"F:%d,fps:%d", k, imgFps);  //将帧数，帧率输入到frameN中
    Point2i k_center;
    k_center=Point2i(2,img.rows-3);
    putText( img, frameN, k_center,CV_FONT_HERSHEY_PLAIN,2.5*img.cols/1384,Scalar(0,0,255),4.5*img.cols/1384);

    if(0 == (k%n))
    {
        time1=time2;
    }

    char tim[50];
    sprintf(tim,"Tim:%d%1d:%1d%1d:%1d", int (time1/60/10),int (int(time1/60)%10),             //十分位，个分位
                                          int ((int(time1)%60)/10),int ((int(time1)%100)%10),  //十秒位，个秒位
                                          int (int(time1*10)%10) );                            //秒小数位
    Point2i tim_center;
    tim_center=Point2i( img.cols-int(250*img.cols/1384),int(img.rows-3) );
    putText(img, tim, tim_center,CV_FONT_HERSHEY_PLAIN,2.5*img.cols/1384,Scalar(0,0,255),4.5*img.cols/1384);//显示时间
}


//图像像素平均值
double GetAllPixelAverageValue(Mat& img)
{
    double totalValue = 0;
    double nCount = 0;
    for (int i=0;i<img.rows;++i)
    {
        uchar* data= img.ptr<uchar>(i);
        for (int j=0;j<img.cols*img.channels();++j)
        {
            totalValue += (double)data[j];
            nCount += 1;
        }
    }
    return totalValue/nCount;
}

//判断是否是有效图像
int FindValidContours(Mat& src)
{
    vector< vector<Point> >contours;
    vector<Vec4i>hierarchy;
    //查找轮廓
    findContours( src, contours, hierarchy ,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
    if (0 == contours.size())
    {
        return 0;
    }
    float area = float(src.cols * src.rows);
    for (int i=0;i<(int)contours.size();++i)
    {
        float areaTemp = fabs( (float)contourArea(contours[i]) );
        if (areaTemp < area)
        {
            area = areaTemp;
        }
    }
    //float minAreaThres = (float)src.cols/20 * (float)src.rows/30;
    //if (area < minAreaThres)
    //{
    //	return 0;
    //}
    return 1;
}





//视觉检测结果保存为txt
void SaveResultToTxt(char* baseDir,  float shrink, vector<VisionResult>& result )
{
    static int num[10] = {0};
    FILE* fp;
    char txtName[1000];
    for(int i=0;i<(int)result.size();++i)
    {
        sprintf(txtName, "%s/txt/result_%d.txt", baseDir, result[i].digitNo);
        fp = fopen(txtName,"a+");
        if (NULL == fp)
        {
            printf("Open txt failed ...\n");
            exit(-2);
        }
        fprintf(fp,"%d %d %d %d %f %f %f \n", num[result[i].digitNo], result[i].frameNo, int(result[i].imagePos2D.x*shrink/2), int(result[i].imagePos2D.y*shrink/2),  result[i].cameraPos3D.x,  result[i].cameraPos3D.y, result[i].cameraPos3D.z );
        fclose(fp);
        num[result[i].digitNo] += 1;
    }
    return;
}
