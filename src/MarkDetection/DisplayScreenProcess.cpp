/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/30
 */

#include "DisplayScreenProcess.h"

//judge rect1 is inside rect2 or not
bool Rect1IsInside(Rect rect1, Rect rect2)
{
    return (rect1 == (rect1&rect2));
}

DisplayScreenProcessType::DisplayScreenProcessType( )
{
    imgNo = 0;
    shrink = 1.0;
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
            //if(saturation < 100)
            //    light_img.at<uchar>(i,j) = 0;
            if(saturation < 120 || (color>30 && color<220)) //red: 0~15, 221~255;
                light_img.at<uchar>(i,j) = 0;
        }
    }
    medianBlur(light_img,median_blur_light_img,7);
    Mat resized_light_img;
    resize(light_img,resized_light_img,Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    imshow("light_img",resized_light_img);
    Mat resized_median_blur_light_img;
    resize(median_blur_light_img,resized_median_blur_light_img,Size(1384*0.3,1032*0.3),0,0,INTER_AREA);
    imshow("median blur light img",resized_median_blur_light_img);
    return;
}


void DisplayScreenProcessType::ThresholdProcess(Mat& median_blur_light_img, Mat& imgBinary)
{
    equalizeHist(median_blur_light_img,median_blur_light_img);
    Mat resized_median_blur_light_img;
    resize(median_blur_light_img,resized_median_blur_light_img,Size(1384*0.5,1032*0.5));
    imshow("equalizeHist_median_blur_light_img",resized_median_blur_light_img);
    int min_size = 80; //100, 80
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(median_blur_light_img, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV
    double s = 5*shrink;
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
        GaussianBlur(roi_img,roi_img,Size(5,5),0,0);
        //equalizeHist(roi_img,roi_img);
        StrongContrast(roi_img);
        imshow("median_blur_light_img_roi", roi_img);
        Mat element=getStructuringElement(MORPH_ELLIPSE, Size( 13,13 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
        morphologyEx(roi_img, roi_img, MORPH_CLOSE ,element);
        threshold(roi_img, roi_img, 100, 255, THRESH_BINARY_INV);
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

void StrongContrast(Mat& gray_img)
{
    float ref_value = 120;
    for(int i=0;i<gray_img.rows;++i)
    {
        for(int j=0;j<gray_img.cols;++j)
        {
            gray_img.at<uchar>(i,j) = saturate_cast<uchar>(float(gray_img.at<uchar>(i,j))/ref_value * float(gray_img.at<uchar>(i,j)));
        }
    }
}
