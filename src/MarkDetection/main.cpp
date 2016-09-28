/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "define.h"
#include "project_path_config.h"

int  openCameraMode = POINTGRAYCAMERA ;  //OPENCV_VIDEOCAPTURE,  POINTGRAYCAMERA, OFFLINEDATA
//保存视频
bool imageSaveEnable = false;
bool digitBinaryImgSaveEnable = true;
//程序读写文件依赖的基本父路径
char baseDir[200] = OCR_DIR_PATH;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_result_publisher");
    time0=static_cast<double>(getTickCount());
    //初始化分类器, 分类器初始化要放在文件夹创建之前
    basicOCR myKNNocr(baseDir);
    KNNocr = &myKNNocr;
    CreatSaveDir(baseDir, imageSaveEnable);
    CreateRosPublishThread(baseDir);
    InitRawImgSubscriber( );
    return 0;
}

ros::Publisher vision_digit_position_publisher;
ros::Subscriber attSub;
//初始化订阅相机的节点
void InitRawImgSubscriber( )
{
    ros::NodeHandle imageProcessNode;
    image_transport::Subscriber rawImgSub;
    image_transport::ImageTransport imageProcessNode_it(imageProcessNode);
    rawImgSub = imageProcessNode_it.subscribe("vision/camera_image", 1, MainImageProcessing);
    vision_digit_position_publisher = imageProcessNode.advertise<sensor_msgs::LaserScan>("vision/digit_nws_position", 1);
    attSub = imageProcessNode.subscribe("imu/attitude", 1, AttitudeSubCallBack);

    ros::spin();
}

void AttitudeSubCallBack(const geometry_msgs::TransformStamped::ConstPtr& att_msg)
{
    attitude3d.roll = float(att_msg->transform.rotation.x);
    attitude3d.pitch = float(att_msg->transform.rotation.y);
    attitude3d.yaw = float(att_msg->transform.rotation.z);
    //printf("roll=%.2f; pit=%.2f; yaw=%.2f;\n",attitude3d.roll*180/3.14,attitude3d.pitch*180/3.14,attitude3d.yaw*180/3.14);
    return;
}

int target = PRINTBOARD; //PRINTBOARD; DISPLAYSCREEN
void MainImageProcessing( const sensor_msgs::ImageConstPtr& msg )
{
    Mat g_rawSaveImage;

    g_rawSaveImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (!g_rawSaveImage.data)
        return;
    Mat rawCameraImg;
    g_rawSaveImage.copyTo( rawCameraImg );
    if (rawCameraImg.cols < 1384)
        resize(rawCameraImg, rawCameraImg, Size(1384,1032));

    if (PRINTBOARD == target)
        PrintBoardProcess(rawCameraImg);
    else if (DISPLAYSCREEN == target)
        DisplayScreenProcess(rawCameraImg);
    //printf("imgNo=%d\n", imageProcessedNo);
    imageProcessedNo++;
    return;
}

void DisplayScreenProcess(Mat& rawCameraImg)
{
    Mat hsv_img;
    cvtColor(rawCameraImg,hsv_img,CV_BGR2HSV_FULL);
    vector<Mat> channels;
    split(hsv_img, channels);
    Mat color_img;
    Mat light_img;
    color_img = channels.at(0);
    light_img = channels.at(2);
    for(int i=0;i<rawCameraImg.rows;++i)
    {
        for(int j=0;j<rawCameraImg.cols;++j)
        {
            int color = color_img.at<uchar>(i,j);
            if(color>30 && color<220)
            {
                rawCameraImg.at<Vec3b>(i,j)[0] = 0;
                rawCameraImg.at<Vec3b>(i,j)[1] = 0;
                rawCameraImg.at<Vec3b>(i,j)[2] = 0;
                light_img.at<uchar>(i,j) = 0;
            }
        }
    }
    Mat show_img;
    resize(rawCameraImg,show_img,Size(640,480),0,0,INTER_AREA);
    resize(light_img,light_img,Size(640,480),0,0,INTER_AREA);
    imshow("color filter", show_img);
    imshow("light_img", light_img);
    waitKey(1);
    return;
}

void PrintBoardProcess(Mat& rawCameraImg)
{
    Mat srcImg;
    //resize image
    ResizeImageByDistance(rawCameraImg, srcImg, lastFrameResult);
    Mat lightness_img_8UC3;
    GetLightnessImage( srcImg, lightness_img_8UC3, lastValidResult);

    if (1 == srcImg.channels())
        cvtColor(srcImg,srcImg,CV_GRAY2BGR);
    Mat rectResultImg = srcImg.clone();
    Mat alg2_rectResultImg = srcImg.clone();

    //a rect detection algorithm based on statistics errors
    RectDetectByStatisticsError( lightness_img_8UC3, alg2_rectResultImg, lastValidResult, incompleteRectResult);

    //rect detector
    RectangleDetect( lightness_img_8UC3, rectResultImg, rectCategory, imageProcessedNo );
    //透视变换
    PerspectiveTransformation(lightness_img_8UC3, rectCandidateImg, rectCategory);
    //Jugde rect kind
    GetRectKinds( rectCategory );
    //位置估计
    EstimatePosition(rectResultImg, rectCategory);
    //数字识别
    DigitDetector(rectResultImg, KNNocr, rectCategory, digitBinaryImgSaveEnable);

    //switch result by the algorithm based on statistics errors
    if (0 == visionResult.size() && incompleteRectResult.size()>0)
    {
        swap(visionResult,incompleteRectResult);
        rectResultImg = alg2_rectResultImg;
    }

    //视觉检测结果保存为txt
    SaveResultToTxt( baseDir, shrink, visionResult );
    //show time and fps
    ShowTime(rectResultImg, imageProcessedNo, shrink);
    //show result image
    resize(rectResultImg, rectResultImg, Size(640,480), 0, 0, INTER_AREA);

    rectResultImg.copyTo(g_rectResultImg);
    g_rectResultImgUpdated = true;

    //Coordinate transformation
    CameraCoordinate2NegCoordinate(visionResult, attitude3d);
    //publish digit result
    DigitResultPublish( visionResult );

    for(int k=0;k<(int)visionResult.size();++k)
    {
        ROS_INFO("\nimgNo=%d;\ndigit=%d;\nx=%.2f;y=%.2f;z=%.2f; \nroll=%.2f;pit=%.2f;yaw=%.2f; \nE=%.2f;N=%.2f;U=%.2f;\n",
                imageProcessedNo,
                visionResult[k].digitNo,
                visionResult[k].cameraPos3D.x, visionResult[k].cameraPos3D.y,visionResult[k].cameraPos3D.z,
                attitude3d.roll*180/3.14,attitude3d.pitch*180/3.14,attitude3d.yaw*180/3.14,
                visionResult[k].negPos3D.y, visionResult[k].negPos3D.x, -visionResult[k].negPos3D.z
                );
    }

    imshow("ImageProcessing",rectResultImg);

    if (false == g_visionResult.empty())
        vector<VisionResult>().swap(g_visionResult);
    for(int k=0;k<visionResult.size();++k)
        g_visionResult.push_back(visionResult[k]);

    //clear memory
    if (false == rectCandidateImg.empty())
        vector<Mat>().swap(rectCandidateImg);
    if (false == rectPossible.empty())
        vector<RectMark>().swap(rectPossible);
    if (false == rectCategory.empty())
        vector< vector<RectMark> >().swap(rectCategory);
    if (false == visionResult.empty())
        vector<VisionResult>().swap(visionResult);
    if (false == incompleteRectResult.empty())
        vector<VisionResult>().swap(incompleteRectResult);
    g_PrecisionRatio = -1;
    g_AccuracyAmount = -1;

    int waitValue = 1;
    int c = waitKey(waitValue);
    if ('p' == c)
        waitValue = 0;
    //press ‘q’ to exit
    if ( 113 == c )  //113 == c || 131153 == c
        exit(0);
    return;
}


void GetLightnessImage( Mat& input_bgr_img, Mat& output_lightness_img, vector< vector<VisionResult> >& lastValidResult)
{
    static unsigned int imgNo = 0;
    Mat rawHSVImg;
    cvtColor(input_bgr_img,rawHSVImg, CV_BGR2HSV_FULL);
    vector<Mat> channels;
    split(rawHSVImg, channels);
    Mat value_image = channels.at(2);
    output_lightness_img = value_image;

    float dist = 0;
    float sum = 0;
    int num = 0;
    if (lastValidResult.size() < 1)
        dist = 10;
    else
    {
        for(int i=0;i<lastValidResult[0].size();++i)
        {
            sum += lastValidResult[0][i].cameraPos3D.z;
            num++;
        }
        dist = sum/num;
    }
    //if (dist < 1.5 )//&& (0 == imgNo%2)
    //    equalizeHist(value_image,output_lightness_img);
    cvtColor(output_lightness_img,output_lightness_img,CV_GRAY2BGR);
    Mat equalize_img;
    resize(output_lightness_img,equalize_img,Size(640,480));
    imshow("hsv_v", equalize_img);
    imgNo++;
}

//矩形（四边形）检测
void RectangleDetect( Mat& lightness_img, Mat& resultImg, vector< vector<RectMark> >& rectCategory, int frameNo)
{
    Mat srcGray;
    cvtColor(lightness_img,srcGray,CV_BGR2GRAY);
    Mat imgBinary;
    int min_size = 100; //100
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(srcGray, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV
    //morphologyEx(imgBinary, imgBinary, MORPH_OPEN, Mat());
    //printf("adaptiveThreshold done!\n");

    double s = 5*shrink;   //* (640)
    int size = ( 1 == int(s)%2 ) ? int(s) : int(s)+1;
    if (size < 3)
        size = 3;
    Mat element;
    if (lastFrameResult.size()>0 && lastFrameResult[0].cameraPos3D.z<1)
    {
        element=getStructuringElement(MORPH_ELLIPSE, Size( size,size ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    }
    else
    {
        element=getStructuringElement(MORPH_RECT, Size( size,size ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    }

    morphologyEx(imgBinary, imgBinary, MORPH_CLOSE ,element);

    Mat imgBinaryShow;
    resize(imgBinary, imgBinaryShow, Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    imshow("adaptiveThresholdImg",imgBinaryShow);
    //printf("imshow imgBinaryShow done!\n");

    vector<Vec4i> hierarchy;
    vector< vector<Point> > all_contours;
    vector< vector<Point> > contours;
    //查找轮廓
    //findContours( imgBinary, all_contours, hierarchy ,RETR_LIST, CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
    findContours( imgBinary, all_contours, RETR_LIST, CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
    //printf("Contours number before filter: %d\n", int(all_contours.size()) );

    //过滤掉太小的轮廓
    for (int i = 0; i < (int)all_contours.size(); ++i)
    {
        if ((int)all_contours[i].size() > srcGray.cols/40*4 && (int)all_contours[i].size()<srcGray.rows*4)
        {
            float area = fabs( (float)contourArea(all_contours[i]) );
            if (area > pow((float)srcGray.rows/40,2))
                contours.push_back(all_contours[i]);
        }
    }
    //cout<<"contours num=" << contours.size() <<endl;

    //四边形筛选
    vector<Point> approxCurve;
    int id = 0;
    for (int i=0; i<(int)contours.size(); ++i)
    {
        //拟合精度
        double fitting_accuracy = 0.015;    //0.005
        //近似多边形逼近
        approxPolyDP(contours[i], approxCurve, double(contours[i].size())*fitting_accuracy, true);	//double(contours[i].size())*0.05
        //非4边形不感兴趣
        if (approxCurve.size() != 4)
            continue;
        //非凸4边形不感兴趣
        if (!isContourConvex(approxCurve))
            continue;
        //四个顶点排序，顺时针：0，1，2，3，左上角为0；
        for(int m=0;m<(int)approxCurve.size();++m)
        {
            for (int n=m+1;n<(int)approxCurve.size();++n)
            {
                if (approxCurve[m].y > approxCurve[n].y)
                    std::swap(approxCurve[m], approxCurve[n]);
            }
        }
        if (approxCurve[0].x > approxCurve[1].x)
            std::swap(approxCurve[0], approxCurve[1]);
        if (approxCurve[3].x > approxCurve[2].x)
            std::swap(approxCurve[2], approxCurve[3]);

        // 找四边形的最小边、最大边
        float minDist = float(1384 * shrink);
        float maxDist = 0.0f;
        float sideLength[4] = {0};
        for (int n=0; n<(int)approxCurve.size(); ++n)
        {
            sideLength[n] = sqrt( pow(float(approxCurve[n].x - approxCurve[(n+1)%4].x),2)+pow(float(approxCurve[n].y - approxCurve[(n+1)%4].y),2) );
            if (sideLength[n] < minDist)
            {
                minDist = sideLength[n];
            }
            if (sideLength[n] > maxDist)
            {
                maxDist = sideLength[n];
            }
        }
        //竖长方形，0、2号边长应小于1、3号边长
        float minLength = sideLength[0];
        if (minLength > sideLength[2])
        {
            minLength = sideLength[2];
        }
        if (minLength > sideLength[1] || minLength > sideLength[3] )
        {
            continue;
        }
        //各边夹角不可太小
        double angle0 = GetTwoSideAngle(approxCurve[3],approxCurve[0], approxCurve[1]);
        double angle1 = GetTwoSideAngle(approxCurve[0],approxCurve[1], approxCurve[2]);
        double angle2 = GetTwoSideAngle(approxCurve[1],approxCurve[2], approxCurve[3]);
        double angle3 = GetTwoSideAngle(approxCurve[2],approxCurve[3], approxCurve[0]);
        double minAngleThres = 90 - 30;
        double maxAngleThres = 90 + 30;
        //相邻两角不可同时大于90度
        if( (angle0>100 && angle1>100) || (angle1>100 && angle2>100) || (angle2>100 && angle3>100) || (angle3>100 && angle0>100) )
        {
            continue;
        }
        if ( (angle0<minAngleThres || angle0>maxAngleThres) || (angle1<minAngleThres || angle1>maxAngleThres)
                || (angle2<minAngleThres || angle2>maxAngleThres) || (angle3<minAngleThres || angle3>maxAngleThres) )
        {
            continue;
        }
        ////不能是竖向平行四边形
        //double angle0_0 = GetTwoSideAngle(approxCurve[1],approxCurve[0],Point2f(approxCurve[1].x,approxCurve[0].y) );
        //double angle1_0 = GetTwoSideAngle(approxCurve[0],approxCurve[1],Point2f(approxCurve[1].x,approxCurve[0].y) );
        //if (angle0<80 && angle1>100 && angle0_0>20 && angle1_0>20)
        //{
        //	continue;
        //}
        //四边形最短边不可太小
        float m_minSideLengthAllowed = float(srcGray.rows/40);
        //最长边与最短边之比不可过小
        float m_maxSideLengthRatio = maxDist/minDist;
        //过滤并存储有效的四边形信息
        if (minDist > m_minSideLengthAllowed && m_maxSideLengthRatio < maxSideLengthRatioAllowed)
        {
            RectMark markTemp;
            for (int n=0; n<4; ++n)
            {
                markTemp.m_points.push_back( Point2f( (float)approxCurve[n].x, (float)approxCurve[n].y ) );
            }
            markTemp.area = fabs( (float)contourArea(contours[i]) );
            markTemp.minSideLength = minDist;
            markTemp.maxSideLength = maxDist;
            markTemp.validFlag = true;
            markTemp.indexId = id;
            markTemp.frameNo = frameNo;
            id++;
            rectPossible.push_back(markTemp);
        }
     }

    //剔除重合的四边形
    RectErase( rectPossible );
    //四边形分类
    RectClassify(rectPossible, rectCategory);
    //按面积排序
    RectSortByArea( rectCategory );
    //不同类的四边形按坐标排序
    RectSortByPositionX( rectCategory );
    //画出各四边形
    DrawAllRect(resultImg, rectCategory);

    return;
}

//求四边形内侧夹角
double GetTwoSideAngle(Point2f p1,Point2f p2, Point2f p3)
{
    //vector1
    double xV1 = p2.x-p1.x;
    double yV1 =p2.y - p1.y;
    //vector2
    double xV2 = p3.x - p2.x;
    double yV2 = p3.y - p2.y;
    if ((0==xV1 && 0 ==yV1) || (0 == xV2 && 0 == yV2))
        return 0;
    else
        return ( acos((xV1*xV2 + yV1*yV2) / sqrt((xV1*xV1 + yV1*yV1)*(xV2*xV2 + yV2*yV2))) * 180 / 3.1415926 );
}


//剔除重合的四边形
void RectErase( vector<RectMark>& rectPossible )
{
    float rectErr[4] = {0};
    float maxErr = 5;
    for (int i=0; i<(int)rectPossible.size(); ++i)
    {
        for (int j=i+1; j<(int)rectPossible.size(); ++j)
        {
            for (int k=0; k<4; ++k)
            {
                //计算两个矩形对应的第k个顶点的像素距离
                rectErr[k] = sqrt(pow((rectPossible[i].m_points[k].x - rectPossible[j].m_points[k].x),2)
                                + pow((rectPossible[i].m_points[k].y - rectPossible[j].m_points[k].y),2));
                if (rectErr[k] > maxErr)
                    break;
            }
            if (rectErr[0] <= maxErr  &&  rectErr[1] <= maxErr  &&  rectErr[2] <= maxErr  &&  rectErr[3] <= maxErr)
            {
                //取重复矩形四个顶点的平均值作为保留顶点
                for (int k=0; k<4; k++)
                {
                    rectPossible[j].m_points[k].x = (rectPossible[i].m_points[k].x + rectPossible[j].m_points[k].x)/2;
                    rectPossible[j].m_points[k].y = (rectPossible[i].m_points[k].y + rectPossible[j].m_points[k].y)/2;
                }
                //剔除重复矩形的顶点
                rectPossible.erase(rectPossible.begin() + j);
                j--;
                break;
            }
        }
    }
    return;
}

//四边形分类
void RectClassify( vector<RectMark>& rectPossible, vector< vector<RectMark> >& rectCategory)
{
    for (int i=0;i<(int)rectPossible.size();++i)
    {
        //rectArrayTemp can't defined before "for(int i=0;...){"!
        vector<RectMark> rectArrayTemp;
        rectArrayTemp.push_back(rectPossible[i]);

        Point2f middlePoint_i = Point2f((rectPossible[i].m_points[0].x+rectPossible[i].m_points[2].x)/2,
                                        (rectPossible[i].m_points[0].y+rectPossible[i].m_points[2].y)/2);
        for (int j=i+1;j<(int)rectPossible.size();++j)
        {
            Point2f middlePoint_j = Point2f((rectPossible[j].m_points[0].x+rectPossible[j].m_points[2].x)/2,
                                            (rectPossible[j].m_points[0].y+rectPossible[j].m_points[2].y)/2);
            //两个四边形对角线中点的距离
            float twoPointDistance = sqrt(pow(middlePoint_i.x-middlePoint_j.x,2)+pow(middlePoint_i.y-middlePoint_j.y,2));
            float minSide = 0;
            if (rectPossible[i].minSideLength < rectPossible[j].minSideLength)
                minSide = rectPossible[i].minSideLength;
            else
                minSide = rectPossible[j].minSideLength;

            //判断两四边形是否属于为同一物理标志
            if (twoPointDistance < (minSide * rectClassifyThres))
            {
                rectArrayTemp.push_back(rectPossible[j]);
                rectPossible.erase(rectPossible.begin() + j);
                j--;
            }
        }
        //同一类的放在一起
        rectCategory.push_back(rectArrayTemp);
    }
    return;
}

//同一类内的四边形按面积排序
void RectSortByArea( vector< vector<RectMark> >& rectCategory )
{
    for (int k=0;k<(int)rectCategory.size();++k)
    {
        //同一类内排序
        for (int i=0;i<(int)rectCategory[k].size();++i)
        {
            for (int j=i+1;j<(int)rectCategory[k].size();++j)
            {
                if (rectCategory[k][j].area < rectCategory[k][i].area)
                {
                    swap(rectCategory[k][i],rectCategory[k][j]);
                }
            }
        }
    }
    return;
}

//不同类的四边形按坐标排序
void RectSortByPositionX( vector< vector<RectMark> >& rectCategory )
{
        //同一类内排序
        for (int i=0;i<(int)rectCategory.size();++i)
        {
            for (int j=i+1;j<(int)rectCategory.size();++j)
            {
                if (rectCategory[j][0].m_points[0].x < rectCategory[i][0].m_points[0].x)
                {
                    swap(rectCategory[j],rectCategory[i]);
                }
            }
        }
    return;
}

//画出各四边形
void DrawAllRect(Mat& resultImg, vector< vector<RectMark> >& rectCategory)
{
    //image = Mat::zeros(480, 640, CV_8UC3);
    for (int k=0;k<(int)rectCategory.size();++k)
    {
        for (int i=0; i<(int)rectCategory[k].size(); ++i)
        {
            int b = (unsigned)theRNG() & 255;
            int g = (unsigned)theRNG() & 255;
            int r = (unsigned)theRNG() & 255;
            line(resultImg, rectCategory[k][i].m_points[0], rectCategory[k][i].m_points[1], Scalar(255,255,0), 3, 8);
            line(resultImg, rectCategory[k][i].m_points[1], rectCategory[k][i].m_points[2], Scalar(255,255,0), 3, 8);
            line(resultImg, rectCategory[k][i].m_points[2], rectCategory[k][i].m_points[3], Scalar(255,255,0), 3, 8);
            line(resultImg, rectCategory[k][i].m_points[3], rectCategory[k][i].m_points[0], Scalar(255,255,0), 3, 8);
            if (0 == i)
            {
                //只在最内侧四边形上标出0、1、2、3
                for (int j=0;j<4;j++)
                {
                    circle(resultImg,rectCategory[k][i].m_points[j],4,Scalar(0,255,0),-1);
                    char strNumber[200];
                    sprintf( strNumber,"%d",j);
                    putText(resultImg,strNumber,rectCategory[k][i].m_points[j],CV_FONT_HERSHEY_COMPLEX_SMALL,1.0,Scalar(0,255,0),1);
                }
            }
        }
    }
    //cout<<"MarkNo="<<rectCategory.size()<<endl;
    //imshow("矩形检测",resultImg);
    return;
}


//透视变换
void PerspectiveTransformation(Mat& srcImg, vector<Mat>& rectCandidateImg, vector< vector<RectMark> >& rectCategory)
{
    char windowName[50];
    vector<Point2f> imagePoints2d(4);
    for (int i=0; i<(int)rectCategory.size(); i++)
    {
        int minLoopCount = ((int)rectCategory[i].size()>1) ? (1) : ((int)rectCategory[i].size());
        for (int j=0;j<minLoopCount;++j)
        {
            //目标矩形对应图像坐标0~3
            imagePoints2d[0]=Point2d(rectCategory[i][j].m_points[0].x,rectCategory[i][j].m_points[0].y);
            imagePoints2d[1]=Point2d(rectCategory[i][j].m_points[1].x,rectCategory[i][j].m_points[1].y);
            imagePoints2d[2]=Point2d(rectCategory[i][j].m_points[2].x,rectCategory[i][j].m_points[2].y);
            imagePoints2d[3]=Point2d(rectCategory[i][j].m_points[3].x,rectCategory[i][j].m_points[3].y);

            // 标准Rect在2d空间为100*100的矩形
            Size m_RectSize = Size(100, int(100*1.25));
            // 矩形 4个角点的正交投影标准值
            vector<Point2f> m_RectCorners2d;
            //vector<Point3f> m_RectCorners3d;
            m_RectCorners2d.push_back(Point2f(0, 0));
            m_RectCorners2d.push_back(Point2f(float(m_RectSize.width-1), 0));
            m_RectCorners2d.push_back(Point2f(float(m_RectSize.width-1), float(m_RectSize.height-1)));
            m_RectCorners2d.push_back(Point2f(0, float(m_RectSize.height-1) ) );

            // 投影变换,恢复2维标准视图
            Mat canonicalImg;
            // 得到当前marker的透视变换矩阵M
            Mat M = getPerspectiveTransform(rectCategory[i][j].m_points, m_RectCorners2d);
            // 将当前的marker变换为正交投影
            warpPerspective(srcImg, canonicalImg, M, m_RectSize);
            //存储矩形区域图像
            canonicalImg.copyTo(rectCategory[i][j].perspectiveImg);
            //分窗口显示各标准矩形区域
            sprintf(windowName,"perspectiveImg-[%d][%d]",i,j);
            //imshow(windowName, rectCategory[i][j].perspectiveImg);
        }
    }
    return;
}


//单目位置估计
void EstimatePosition(Mat& srcColor, vector< vector<RectMark> >& rectCategory)
{
    vector<Point2f> imagePoints2d(4);
    vector<Point3f> objectPoints3d(4);
    const static double kind_0_width = 0.3;
    const static double kind_0_height = 0.4;
    const static double kind_1_width = 0.4;
    const static double kind_1_height = 0.5;
    const static double kind_2_width = 0.6;
    const static double kind_2_height = 0.7;
    double realRectHalfWidth;
    double realRectHalfHeight;

    for (int i=0; i<(int)rectCategory.size(); ++i)
    {
        if (-1 == rectCategory[i][0].rectKind)
        {
            continue;
        }
        else if (0 == rectCategory[i][0].rectKind)
        {
            realRectHalfWidth  = kind_0_width/2;
            realRectHalfHeight = kind_0_height/2;
        }
        else if (1 == rectCategory[i][0].rectKind)
        {
            realRectHalfWidth  = kind_1_width/2;
            realRectHalfHeight = kind_1_height/2;
        }
        else if (2 == rectCategory[i][0].rectKind)
        {
            realRectHalfWidth  = kind_2_width/2;
            realRectHalfHeight = kind_2_height/2;
        }

        //目标矩形上四个点世界坐标
        objectPoints3d[0]=Point3d(-realRectHalfWidth, -realRectHalfHeight, 0);
        objectPoints3d[1]=Point3d(realRectHalfWidth,  -realRectHalfHeight, 0);
        objectPoints3d[2]=Point3d(realRectHalfWidth,   realRectHalfHeight, 0);
        objectPoints3d[3]=Point3d(-realRectHalfWidth,  realRectHalfHeight, 0);

        //目标矩形对应图像坐标0~3
        imagePoints2d[0]=Point2d(rectCategory[i][0].m_points[0].x,rectCategory[i][0].m_points[0].y);
        imagePoints2d[1]=Point2d(rectCategory[i][0].m_points[1].x,rectCategory[i][0].m_points[1].y);
        imagePoints2d[2]=Point2d(rectCategory[i][0].m_points[2].x,rectCategory[i][0].m_points[2].y);
        imagePoints2d[3]=Point2d(rectCategory[i][0].m_points[3].x,rectCategory[i][0].m_points[3].y);

        //Point Gray Flea3-14s3c
        double t_fx=893.25550*shrink, t_fy=894.63039*shrink, t_cx=686.67029*shrink, t_cy=518.99170*shrink;
        Mat t_distcoef=(Mat_<double>(1,5) << -0.05173, 0.07077, -0.00047, 0.00061,0);
        Mat t_cameraMatrix=(Mat_<double>(3,3) << t_fx,0,t_cx,0,t_fy,t_cy,0,0,1);

        //计算旋转矩阵、平移矩阵
        Mat rvec,tvec;
        solvePnP(objectPoints3d,imagePoints2d,t_cameraMatrix,t_distcoef,rvec,tvec);

        double tvec_x,tvec_y,tvec_z;
        tvec_x=tvec.at<double>(0,0);
        tvec_y=tvec.at<double>(1,0);
        tvec_z=tvec.at<double>(2,0);
        rectCategory[i][0].position = Point3d( tvec_x,tvec_y,tvec_z );
        //剔除过远或过近的
        if(tvec_z>10.5 || tvec_z<0.3)
        {
            rectCategory.erase(rectCategory.begin()+i);
            i--;
            continue;
        }
        char transf[50];
        //sprintf_s(transf,"T%u:[%0.3fm,%0.3fm,%0.3fm]",i,tvec_y,tvec_x,tvec_z);
        sprintf(transf,"%0.3fm",tvec_z);
        Point T_showCenter;
        T_showCenter=Point2d(rectCategory[i][0].m_points[1].x,(rectCategory[i][0].m_points[1].y + rectCategory[i][0].m_points[2].y)/2);
        putText(srcColor, transf, T_showCenter,CV_FONT_HERSHEY_PLAIN,2.2*shrink,Scalar(0,0,255),int(4.5*shrink));
    }
    //在图像中显示平移向量（x,y,z）
    Mat imgColor;
    //resize(srcColor,imgColor,Size(640,480));
    //imshow("rect distance",imgColor);
    return;
}


//数字识别
void DigitDetector(Mat& ResultImg, basicOCR* ocr, vector< vector<RectMark> >& rectCategory, bool saveDigitBinaryImg)
{
    for (int i=0;i<(int)rectCategory.size();++i)
    {
        if ( -1 == rectCategory[i][0].rectKind )
        {
            continue;
        }
        Mat possibleDigitBinaryImg = rectCategory[i][0].possibleDigitBinaryImg.clone();
        if (!possibleDigitBinaryImg.data)
            continue;

        //数字识别
        IplImage ipl_img(possibleDigitBinaryImg);
        float classResult = ocr->classify(&ipl_img,1);
        float precisionRatio = g_PrecisionRatio;
        //printf("classify done!\n");

        char digit[100];
        //存储用于识别的数字图像
        if (true == saveDigitBinaryImg)
        {
            sprintf(digit, "%s/digit_image/digit_%d_%06d__%d.pbm", baseDir, int(classResult), rectCategory[i][0].frameNo, int(precisionRatio));
            imwrite(digit,  rectCategory[i][0].possibleDigitBinaryImg );
        }

        //识别再过滤, 预测率小于80%都算误识别
        if ((rectCategory[i][0].position.z <=7.5 && (int)precisionRatio < 90) || (rectCategory[i][0].position.z <= 9.0 && rectCategory[i][0].position.z > 7.5 && (int)precisionRatio < 100) || rectCategory[i][0].position.z > 9.0)
            continue;

        //printf("Digit=%d;Precision=%0.1f%%\n",(int)classResult,precisionRatio);
        rectCategory[i][0].digitNo = (int)classResult;

        //最终检测结果存入vector
        VisionResult result;
        result.frameNo = rectCategory[i][0].frameNo;
        result.digitNo = (int)classResult;
        result.imagePos2D = Point2f( (rectCategory[i][0].m_points[0].x+rectCategory[i][0].m_points[2].x)/2, (rectCategory[i][0].m_points[0].y+rectCategory[i][0].m_points[2].y)/2 );
        result.cameraPos3D = rectCategory[i][0].position;
        visionResult.push_back(result);

        //将用于识别的digit图像进行分窗口显示
        sprintf(digit,"%d",int(classResult));
        imshow(digit, rectCategory[i][0].possibleDigitBinaryImg);

        int x = int((rectCategory[i][0].m_points[0].x + rectCategory[i][0].m_points[2].x)/2) - 10;
        int y = int((rectCategory[i][0].m_points[0].y + rectCategory[i][0].m_points[2].y)/2) + 10;
        if(x <= 0)
            x = ResultImg.cols/64;
        if (x >= ResultImg.cols)
            x = ResultImg.cols - ResultImg.cols/64;
        if(y <= 0)
            y = ResultImg.rows/64;
        if (y >= ResultImg.rows)
            y = ResultImg.rows - ResultImg.rows/64;
        Point showCenter = Point(x,y);
        //double textScale = double( float(rectCategory[i][0].maxSideLength/ResultImg.rows)*8 + 1.0 );
        putText(ResultImg, digit, showCenter,CV_FONT_HERSHEY_DUPLEX,2.2*shrink,Scalar(0,0,255), int(4.5*shrink));
        //putText(ResultImg, digit, showCenter,CV_FONT_HERSHEY_DUPLEX,3.0,Scalar(0,255,0),3);
        //imshow("Classify",possibleDigitBinaryImg);

        sprintf(digit,"beforeClassify-%d",int(classResult));
        imshow(digit, rectCategory[i][0].possibleRectBinaryImg);

        //if ( precisionRatio<70 )
        //    waitKey(0);
    }

    //清空上一帧的检测结果
    if (false == lastFrameResult.empty())
        vector<VisionResult>().swap(lastFrameResult);
    for(int i=0; i<(int)visionResult.size(); ++i)
    {
        lastFrameResult.push_back(visionResult[i]);
    }

    if (visionResult.size() >= 1)
    {
        vector< vector<VisionResult> >().swap(lastValidResult);
        lastValidResult.push_back(visionResult);
    }
    return;
}

void ResizeImageByDistance( Mat& inputImg, Mat& outputImg, vector<VisionResult>& lastFrameResult)
{
    static double lastFrameResultDistance = 0;

    if ( lastFrameResult.size() < 1 )
    {
        shrink = SHRINK_HIGHEST_VALUE;
        resize(inputImg, outputImg, Size(inputImg.cols*shrink, inputImg.rows*shrink), 0, 0, CV_INTER_AREA);
        return;
    }

    double totalDis = 0;
    for(int i=0; i<lastFrameResult.size(); ++i)
    {
        totalDis += lastFrameResult[i].cameraPos3D.z;
    }
    lastFrameResultDistance = totalDis/lastFrameResult.size();

    if (lastFrameResultDistance <= 1.0)
        shrink = SHRINK_LOWEST_VALUE;
    else if (lastFrameResultDistance <= 1.5 && lastFrameResultDistance > 1.0)
        shrink = SHRINK_LOWEST_VALUE;
    else if (lastFrameResultDistance <= 2 && lastFrameResultDistance > 1.5)
        shrink = SHRINK_LOWEST_VALUE + 0.1; //0.57
    else if (lastFrameResultDistance <= 2.5 && lastFrameResultDistance > 2)
        shrink = SHRINK_LOWEST_VALUE + 0.2; //0.67
    else if (lastFrameResultDistance <= 3 && lastFrameResultDistance > 2.5)
        shrink = SHRINK_LOWEST_VALUE + 0.3; //0.77
    else if (lastFrameResultDistance <= 3.5 && lastFrameResultDistance > 3)
        shrink = SHRINK_LOWEST_VALUE + 0.4; //0.87
    else if (lastFrameResultDistance <= 4 && lastFrameResultDistance > 3.5)
        shrink = SHRINK_LOWEST_VALUE + 0.5; //0.97
    else if (lastFrameResultDistance <= 4.5 && lastFrameResultDistance > 4)
        shrink = SHRINK_HIGHEST_VALUE;
    else
        shrink = SHRINK_HIGHEST_VALUE;

    if (fabs(shrink - 1.0) < 0.001)
    {
        outputImg = inputImg.clone();
        return;
    }

    if(shrink < 0.99)
    {
        resize(inputImg, outputImg, Size(inputImg.cols*shrink, inputImg.rows*shrink), 0, 0, INTER_AREA);
    }
    else
    {
        resize(inputImg, outputImg, Size(inputImg.cols*shrink, inputImg.rows*shrink), 0, 0, INTER_LINEAR);
    }
    return;
}


void DigitResultPublish(vector<VisionResult>& visionResult )
{
    // publish digits position
    sensor_msgs::LaserScan digits_position;

    if (visionResult.size() < 1)
    {
        digits_position.ranges.resize(4);
        digits_position.header.frame_id = "digits_position";
        digits_position.header.stamp    = ros::Time::now();
        int i = 0;
        digits_position.ranges[i*4] = float(-1);
        digits_position.ranges[i*4 + 1] = float(-1);
        digits_position.ranges[i*4 + 2] = float(-1);
        digits_position.ranges[i*4 + 3] = float(-1);
    }
    else
    {
        digits_position.ranges.resize(visionResult.size()*4);
        digits_position.header.frame_id = "digits_position";
        digits_position.header.stamp    = ros::Time::now();
        for ( int i = 0; i < (int)visionResult.size(); ++i )
        {
            digits_position.ranges[i*4] = float(visionResult[i].digitNo);
            digits_position.ranges[i*4 + 1] = float(visionResult[i].negPos3D.y);
            digits_position.ranges[i*4 + 2] = float(visionResult[i].negPos3D.x);
            digits_position.ranges[i*4 + 3] = float(-visionResult[i].negPos3D.z);
        }
    }
    vision_digit_position_publisher.publish(digits_position);

    return;
}
