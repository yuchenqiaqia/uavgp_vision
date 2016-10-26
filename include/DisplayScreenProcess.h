/*
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "KNN_OCR.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class RoiAreaInfo
{
public:
    RoiAreaInfo()
    {
        digitNo = -1;
        acccuracy = -1;
    }
    Rect minBoundingRect;
    Mat  roi_img;
    int digitNo;
    float acccuracy;
    float distance[3];
};

class DisplayScreenProcessType
{
public:
    DisplayScreenProcessType();
    int DisplayScreenProcess(Mat& input_img, basicOCR* KNNocr, const char* baseDir, int color_filter_value);

    int imgNo;
    float shrink;
    Mat binary_roi_img;
    Mat show_img;

private:
    void GetPossibleRois(Mat& color_filtered_img, vector<Rect>& preprocess_rois);
    void DigitSort(Mat& input_img, vector<RoiAreaInfo>& roiAreaInfos);
    void ContoursPreFilter(vector< vector<Point> >& all_contours, vector< vector<Point> >& contours);
    void GetDigitRoi(vector< vector<Point> >& contours, Mat& binary_img, vector<RoiAreaInfo>& roiAreaInfos);
    void ColorFilter(Mat& rawCameraImg, Mat& color_filtered_img);
    void ThresholdProcess(Mat& color_filtered_img, vector<Rect>& preprocess_rois, Mat& median_blur_light_img, Mat& imgBinary);
    void DigitClassify(vector<RoiAreaInfo>& roiAreaInfos, Mat& rawCameraImg, basicOCR* KNNocr, const char* baseDir);

    Mat rawCameraImg;
    float rect_filter_two_side_ratio_max;
    float rect_filter_two_side_ratio_min;
    float min_bounding_rect_height_ratio;
    float min_precision_ratio_thres;
    float min_knn_distance_thres;
    int red_filter_value;
};

