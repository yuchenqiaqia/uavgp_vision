#include "DisplayScreenProcess.h"

DisplayScreenProcessType::DisplayScreenProcessType( )
{
    imgNo = 0;
    shrink = 1.0;
    rect_filter_two_side_ratio_max = 0.9;
    rect_filter_two_side_ratio_min = 0.25;
    return;
}

void DisplayScreenProcessType::DisplayScreenProcess(Mat& input_img, basicOCR* KNNocr, const char* baseDir)
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

    RoiAreaInfo roiAreaInfos;
    DisplayScreenProcessType::GetDigitRoi(contours, binary_img, roiAreaInfos);
    DisplayScreenProcessType::DigitClassify(roiAreaInfos, rawCameraImg, KNNocr, baseDir);

    resize(rawCameraImg,show_img,Size(640,480),0,0,INTER_AREA);
    imshow("show_img", show_img);
    waitKey(1);

    imgNo++;
    return;
}


void DisplayScreenProcessType::DigitClassify(RoiAreaInfo& roiAreaInfos, Mat& rawCameraImg, basicOCR* KNNocr, const char* baseDir)
{
    for(int i=0;i<(int)roiAreaInfos.roi_imgs.size();++i)
    {
        IplImage ipl_img(roiAreaInfos.roi_imgs[i]);
        float classResult = KNNocr->classify(&ipl_img,1);
        float precisionRatio = KNNocr->knn_result.precisionRatio;
        printf("digit=%d\nprecisionRatio=%0.2f\n",(int)classResult,precisionRatio*0.01);

        char digit[100];
        sprintf(digit, "%s/digit_image/digit_%d_%06d__%d.pbm", baseDir, int(classResult), imgNo, int(precisionRatio));
        imwrite(digit,  roiAreaInfos.roi_imgs[i] );

        if (precisionRatio >= 90)
        {
            sprintf(digit,"%d",(int)classResult);
            Point showCenter = Point(roiAreaInfos.minBoundingRects[i].x + roiAreaInfos.minBoundingRects[i].width + 60, roiAreaInfos.minBoundingRects[i].y + roiAreaInfos.minBoundingRects[i].height*0.5);
            putText(rawCameraImg, digit, showCenter,CV_FONT_HERSHEY_DUPLEX,5.2*shrink,Scalar(0,0,255), int(5.5*shrink));
            rectangle( rawCameraImg, roiAreaInfos.minBoundingRects[i], Scalar(255,0,255), 4, 8);
        }
        imshow("digit",roiAreaInfos.roi_imgs[i]);
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
            if(saturation < 120 || (color>30 && color<220))
                light_img.at<uchar>(i,j) = 0;
        }
    }
    medianBlur(light_img,median_blur_light_img,7);
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
    int min_size = 70; //100, 80
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(median_blur_light_img, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV
    double s = 5*shrink;   //* (640)
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
        if (area < pow(rawCameraImg.rows*0.08,2) || side0>rawCameraImg.cols*0.5 || side0/side1 > rect_filter_two_side_ratio_max || side0/side1 < rect_filter_two_side_ratio_min)
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



void DisplayScreenProcessType::GetDigitRoi(vector< vector<Point> >& contours, Mat& binary_img, RoiAreaInfo& roiAreaInfos)
{
    for(int i=0;i<(int)contours.size();++i)
    {
        Rect minBoundingRect = boundingRect( Mat(contours[i]) );
        rectangle( rawCameraImg, minBoundingRect, Scalar(0,255,255), 1, 8);
        if ((minBoundingRect.height < rawCameraImg.cols*0.15) || ((minBoundingRect.width*1.0/minBoundingRect.height) > rect_filter_two_side_ratio_max) || ((minBoundingRect.width*1.0/minBoundingRect.height) < rect_filter_two_side_ratio_min))
            continue;

        int height = minBoundingRect.height*1.2;
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
        binary_roi_img = binary_img(roi).clone();
        resize(binary_roi_img,binary_roi_img,Size(128,128));
        Mat element=getStructuringElement(MORPH_ELLIPSE, Size( 11,11 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
        morphologyEx(binary_roi_img, binary_roi_img, MORPH_CLOSE ,element);
        threshold(binary_roi_img, binary_roi_img, 125, 255, THRESH_BINARY_INV);

        roiAreaInfos.minBoundingRects.push_back(minBoundingRect);
        roiAreaInfos.roi_imgs.push_back(binary_roi_img);
    }
    return;
}
