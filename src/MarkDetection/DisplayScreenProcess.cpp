/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/30
 */

#include "DisplayScreenProcess.h"
static void ShowTime(Mat& img, int k, float shrink);

static float GetAverageValue(Mat& input_img, bool non_zero_pixel, float thres = 0)
{
    float sum = 0;
    int count_num = 0;
    for(int i=0;i<(int)input_img.rows;++i)
    {
        for(int j=0;j<(int)input_img.cols;++j)
        {
            int value = input_img.at<uchar>(i,j);
            if (true == non_zero_pixel)
            {
                if (value > thres)
                {
                        sum += input_img.at<uchar>(i,j);
                        count_num++;
                }
            }
            else
            {
                sum += input_img.at<uchar>(i,j);
                count_num++;
            }
        }
    }
    if (0 == count_num)
        return 0;
    else
        return int(sum/count_num);
}

static void StrongContrast(Mat& gray_img, float contrast_ratio)
{
    for(int i=0;i<gray_img.rows;++i)
    {
        for(int j=0;j<gray_img.cols;++j)
        {
            gray_img.at<uchar>(i,j) = saturate_cast<uchar>(float(gray_img.at<uchar>(i,j))*contrast_ratio * float(gray_img.at<uchar>(i,j)));
        }
    }
}

//judge rect1 is inside rect2 or not
bool Rect1IsInside(Rect rect1, Rect rect2)
{
    return (rect1 == (rect1&rect2));
}

DisplayScreenProcessType::DisplayScreenProcessType( )
{
    imgNo = 0;
    shrink = 0.8;
    rect_filter_two_side_ratio_max = 0.9;   ////0.9
    rect_filter_two_side_ratio_min = 0.1;   ////0.2
    min_bounding_rect_height_ratio = 0.06;  ////0.1
    min_precision_ratio_thres = 80.0;    ////80
    min_knn_distance_thres = 260;
    return;
}

int DisplayScreenProcessType::DisplayScreenProcess(Mat& input_img, basicOCR* KNNocr, const char* baseDir)
{
    rawCameraImg = input_img.clone();
    resize(rawCameraImg,rawCameraImg,Size(rawCameraImg.cols*shrink,rawCameraImg.rows*shrink),0,0,INTER_AREA);

    Mat median_blur_light_img;
    DisplayScreenProcessType::ColorFilter(rawCameraImg, median_blur_light_img);

    Mat imgBinary;
    DisplayScreenProcessType::ThresholdProcess(median_blur_light_img, imgBinary);
    Mat binary_img = imgBinary.clone();

    vector< vector<Point> > all_contours;
    findContours( imgBinary, all_contours, RETR_LIST, CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL

    vector< vector<Point> > contours;
    DisplayScreenProcessType::ContoursPreFilter(all_contours, contours);

    vector<RoiAreaInfo> roiAreaInfos;
    DisplayScreenProcessType::GetDigitRoi(contours, median_blur_light_img, roiAreaInfos);
    DisplayScreenProcessType::DigitClassify(roiAreaInfos, rawCameraImg, KNNocr, baseDir);
    DisplayScreenProcessType::DigitSort(rawCameraImg, roiAreaInfos);

    resize(rawCameraImg,show_img,Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    ShowTime(show_img, imgNo, 0.5);
    imshow("Display screen", show_img);
    waitKey(1);
    imgNo++;

    if (roiAreaInfos.size() >= 1)
        return roiAreaInfos[0].digitNo;
    else
        return -1;
}

void DisplayScreenProcessType::DigitSort(Mat& input_img, vector<RoiAreaInfo>& roiAreaInfos)
{
    for(int i=0;i<(int)roiAreaInfos.size();++i)
    {
        for(int j=i+1;j<(int)roiAreaInfos.size();++j)
        {
            int x_i = roiAreaInfos[i].minBoundingRect.x + roiAreaInfos[i].minBoundingRect.width/2;
            int y_i = roiAreaInfos[i].minBoundingRect.y + roiAreaInfos[i].minBoundingRect.height/2;
            int x_j = roiAreaInfos[j].minBoundingRect.x + roiAreaInfos[j].minBoundingRect.width/2;
            int y_j = roiAreaInfos[j].minBoundingRect.y + roiAreaInfos[j].minBoundingRect.height/2;
            float dis_c_i = sqrt(pow(x_i - input_img.cols/2,2) + pow(y_i - input_img.rows/2,2));
            float dis_c_j = sqrt(pow(x_j - input_img.cols/2,2) + pow(y_j - input_img.rows/2,2));
            if (dis_c_i > dis_c_j)
            {
                swap(roiAreaInfos[i],roiAreaInfos[j]);
            }
        }
    }
    return;
}

void DisplayScreenProcessType::DigitClassify(vector<RoiAreaInfo>& roiAreaInfos, Mat& rawCameraImg, basicOCR* KNNocr, const char* baseDir)
{
    int num = 0;
    for(int i=0;i<(int)roiAreaInfos.size();++i)
    {
        IplImage ipl_img(roiAreaInfos[i].roi_imgs);
        float classResult = KNNocr->classify(&ipl_img,1);
        float precisionRatio = KNNocr->knn_result.precisionRatio;
        float min_distance[3];
        for(int j=0;j<3;++j)
           min_distance[j] = KNNocr->knn_result.min_distance[j];

        char digit[500];
        sprintf(digit, "%s/digit_image/digit_%d_accuracy_%d_dist_%d_%d_No_%06d.pbm", baseDir, int(classResult), int(precisionRatio), int(min_distance[0]), int(min_distance[1]), imgNo);
        imwrite(digit,  roiAreaInfos[i].roi_imgs );

        if (min_distance[0] >= min_knn_distance_thres || min_distance[0] < 1)
        {
            roiAreaInfos.erase(roiAreaInfos.begin() + i);
            i--;
            continue;
        }
        if (precisionRatio <= min_precision_ratio_thres)
        {
            roiAreaInfos.erase(roiAreaInfos.begin() + i);
            i--;
            continue;
        }
        rectangle( rawCameraImg, roiAreaInfos[i].minBoundingRect, Scalar(0,255,255), 5, 8);
        printf("digit=%d; accuracy=%d%%; dist=%d,%d,%d\n\n",(int)classResult,(int)precisionRatio, int(min_distance[0]), int(min_distance[1]), int(min_distance[2]));
        sprintf(digit,"%d",(int)classResult);
        Point showCenter = Point(roiAreaInfos[i].minBoundingRect.x + roiAreaInfos[i].minBoundingRect.width + 60, roiAreaInfos[i].minBoundingRect.y + roiAreaInfos[i].minBoundingRect.height*0.5);
        putText(rawCameraImg, digit, showCenter,CV_FONT_HERSHEY_DUPLEX,5.2*shrink,Scalar(0,0,255), int(5.5*shrink));
        char window_name[50];
        sprintf(window_name,"digit_%d", num);
        imshow(window_name,roiAreaInfos[i].roi_imgs);
        roiAreaInfos[i].digitNo = (int)classResult;
        num++;
    }
    return;
}


void DisplayScreenProcessType::ColorFilter(Mat& rawCameraImg, Mat& median_blur_light_img)
{
    // color filter
    Mat hsv_img;
    cvtColor(rawCameraImg,hsv_img,CV_BGR2HSV_FULL);
    vector<Mat> channels;
    split(hsv_img, channels);
    Mat color_img;
    Mat saturation_img;
    Mat light_img;
    color_img = channels.at(0);
    saturation_img = channels.at(1);
    light_img = channels.at(2);
    for(int i=0;i<rawCameraImg.rows;++i)
    {
        for(int j=0;j<rawCameraImg.cols;++j)
        {
            int color = color_img.at<uchar>(i,j);
            int saturation = saturation_img.at<uchar>(i,j);
            if(saturation < 120)
                light_img.at<uchar>(i,j) = 0;
            if( color>60 && color<220 ) //red: 0~15, 221~255;
                light_img.at<uchar>(i,j) = 0;
        }
    }
    medianBlur(light_img,median_blur_light_img,5);
    Mat resized_light_img;
    resize(light_img,resized_light_img,Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    imshow("light_img",resized_light_img);
    Mat resized_median_blur_light_img;
    resize(median_blur_light_img,resized_median_blur_light_img,Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    imshow("median blur light img",resized_median_blur_light_img);
    return;
}


void DisplayScreenProcessType::ThresholdProcess(Mat& median_blur_light_img, Mat& imgBinary)
{
    float contrast_ratio = 0.005;
    //StrongContrast(median_blur_light_img, contrast_ratio);
    Rect roi = Rect(0+median_blur_light_img.cols*0.2, 0+median_blur_light_img.rows*0.1, median_blur_light_img.cols*0.6, median_blur_light_img.rows*0.89);
    Mat img = median_blur_light_img(roi);
    float cal_thres = 80;
    float value = GetAverageValue( img,true,80 );

    contrast_ratio = 0.025 * 20/value;
    StrongContrast(median_blur_light_img, contrast_ratio);
    if (value <= cal_thres*2)
        equalizeHist(median_blur_light_img,median_blur_light_img);

    Mat resized_median_blur_light_img;
    resize(median_blur_light_img,resized_median_blur_light_img,Size(1384*0.5,1032*0.5));
    imshow("Strong Contrast median_blur_light_img",resized_median_blur_light_img);
    int min_size = 70; //100, 80
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(median_blur_light_img, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV
    double s = 11*shrink;
    int size = ( 1 == int(s)%2 ) ? int(s) : int(s)+1;
    if (size < 3)
        size = 3;
    Mat element;
    element=getStructuringElement(MORPH_ELLIPSE, Size( size,size ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    morphologyEx(imgBinary, imgBinary, MORPH_CLOSE ,element);
    Mat imgBinaryShow;
    resize(imgBinary, imgBinaryShow, Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    imshow("adaptiveThresholdImg",imgBinaryShow);
    return;
}


void DisplayScreenProcessType::ContoursPreFilter(vector< vector<Point> >& all_contours, vector< vector<Point> >& contours)
{
    for (int i = 0; i < (int)all_contours.size(); ++i)
    {
        if ((int)all_contours[i].size() > rawCameraImg.rows*0.2 && (int)all_contours[i].size()<rawCameraImg.rows*2.5)
        {
            float area = fabs( (float)contourArea(all_contours[i]) );
            //if (area > pow((float)imgBinary.rows*0.2,2))
                contours.push_back(all_contours[i]);
        }
    }

    for(int i=0;i<(int)contours.size();++i)
    {
        RotatedRect boxTemp = minAreaRect( Mat(contours[i]) );
        Point2f vertex[4];
        boxTemp.points(vertex);

        //vertex sort，clock wise, top left corner is 0；
        for(int m=0;m<4;++m)
        {
            for (int n=m+1;n<4;++n)
            {
                if (vertex[m].y > vertex[n].y)
                    std::swap(vertex[m], vertex[n]);
            }
        }
        if (vertex[0].x > vertex[1].x)
            std::swap(vertex[0], vertex[1]);
        if (vertex[3].x > vertex[2].x)
            std::swap(vertex[2], vertex[3]);

        float side0 = sqrt( pow(vertex[1].x - vertex[0].x, 2) + pow(vertex[1].y - vertex[0].y, 2));
        float side1 = sqrt( pow(vertex[2].x - vertex[1].x, 2) + pow(vertex[2].y - vertex[1].y, 2));
        float area = side0 * side1;
        //printf("side0/side1=%0.3f\n",side0/side1);
        if (area < pow(rawCameraImg.rows*0.05,2) || side0>rawCameraImg.cols*0.5 || side0/side1 > rect_filter_two_side_ratio_max || side0/side1 < rect_filter_two_side_ratio_min)
        {
            contours.erase(contours.begin() + i);
            i--;
            continue;
        }
        for(int j=0; j<4;++j)
        {
            line(rawCameraImg, vertex[j], vertex[(j+1)%4], Scalar(0,255,0), 1, 8);
            char strNumber[200];
            sprintf( strNumber,"%d",j);
            //putText(rawCameraImg,strNumber,vertex[j],CV_FONT_HERSHEY_COMPLEX_SMALL,1.5,Scalar(0,0,255),2);
        }
    }
    return;
}



void DisplayScreenProcessType::GetDigitRoi(vector< vector<Point> >& contours, Mat& input_img, vector<RoiAreaInfo>& roiAreaInfos)
{
    for(int i=0;i<(int)contours.size();++i)
    {
        Rect minBoundingRect = boundingRect( Mat(contours[i]) );
        rectangle( rawCameraImg, minBoundingRect, Scalar(0,255,255), 1, 8);
        if ((minBoundingRect.height < rawCameraImg.cols*min_bounding_rect_height_ratio) || ((minBoundingRect.width*1.0/minBoundingRect.height) > rect_filter_two_side_ratio_max) || ((minBoundingRect.width*1.0/minBoundingRect.height) < rect_filter_two_side_ratio_min))
            continue;

        int height = minBoundingRect.height*1.1;
        int y = int(minBoundingRect.y + minBoundingRect.height*0.5 - height*0.5);
        int width = int(minBoundingRect.height * 2.7/4 * 1.2);
        int x = int(minBoundingRect.x + minBoundingRect.width*0.5 - width*0.5);
        if (x<=0)
            x = 0;
        if (y<=0)
            y = 0;
        if ((x + width) >= (rawCameraImg.cols-1))
            width = rawCameraImg.cols - x;
        if ((y + height) >= (rawCameraImg.rows-1))
            height = rawCameraImg.rows - y;
        Rect roi = Rect(x, y, width, height);
        Mat roi_img;
        input_img(roi).copyTo(roi_img);

        resize(roi_img,roi_img,Size(128,128),0,0,INTER_AREA);
        medianBlur(roi_img,roi_img,5);
        imshow("before GaussianBlur", roi_img);
        GaussianBlur(roi_img,roi_img,Size(5,5),0,0);

        //equalizeHist(roi_img,roi_img);
        Rect roi_img_roi = Rect(0+roi_img.cols*0.1, 0+roi_img.rows*0.1, roi_img.cols*0.89, roi_img.rows*0.89);
        Mat img = roi_img(roi_img_roi);
        float value = GetAverageValue( img,true );
        float contrast_ratio = 0.025 * 20/value;
        StrongContrast(roi_img, contrast_ratio);
        imshow("GaussianBlur_light_img_roi", roi_img);

        float average_value = GetAverageValue( roi_img,false );
        printf("average_value=%d\n",int(average_value));

        if (average_value < 50)
        {
            Mat element=getStructuringElement(MORPH_ELLIPSE, Size( 13,13 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
            int morphology_option = MORPH_CLOSE;
            morphologyEx(roi_img, roi_img, morphology_option ,element);
        }
        else
        {
            Mat element=getStructuringElement(MORPH_ELLIPSE, Size( 5,5 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
            erode(roi_img,roi_img,element);
        }

        double min_filter_thres = 255*pow(average_value,2)/(255*255)*3;
        threshold(roi_img, roi_img, min_filter_thres, 255, THRESH_BINARY_INV);   //120
        imshow("wait classify roi", roi_img);
        RoiAreaInfo roiAreaInfo;
        roiAreaInfo.minBoundingRect = minBoundingRect;
        roiAreaInfo.roi_imgs = roi_img;
        roiAreaInfos.push_back(roiAreaInfo);
    }
    //sort by area
    for(int i=0;i<int(roiAreaInfos.size());++i)
    {
        for(int j=i+1;j<(int)roiAreaInfos.size();++j)
        {
            if (roiAreaInfos[i].minBoundingRect.area() < roiAreaInfos[j].minBoundingRect.area())
                swap(roiAreaInfos[i],roiAreaInfos[j]);
        }
    }
    //erase inside rects
    for(int i=0;i<int(roiAreaInfos.size());++i)
    {
        for(int j=i+1;j<(int)roiAreaInfos.size();++j)
        {
            bool is_inside = Rect1IsInside(roiAreaInfos[j].minBoundingRect, roiAreaInfos[i].minBoundingRect);
            if (true == is_inside)
            {
                roiAreaInfos.erase(roiAreaInfos.begin()+j);
                j--;
                continue;
            }
        }
    }

    return;
}




extern double time0,time1,time2;
//显示时间、帧率
static void ShowTime(Mat& img, int k, float shrink)
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
