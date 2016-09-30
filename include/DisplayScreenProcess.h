#include "KNN_OCR.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class RoiAreaInfo
{
public:
    vector<Rect> minBoundingRects;
    vector<Mat>  roi_imgs;
};

class DisplayScreenProcessType
{
public:
    DisplayScreenProcessType();
    void DisplayScreenProcess(Mat& input_img, basicOCR* KNNocr, const char* baseDir);

    int imgNo;
    float shrink;
    Mat rawCameraImg;
    Mat binary_roi_img;
    Mat show_img;
    float rect_filter_two_side_ratio_max;
    float rect_filter_two_side_ratio_min;
    float min_bounding_rect_height_ratio;
    float min_precision_ratio_thres;
    float min_knn_distance_thres;

private:
    void ContoursPreFilter(vector< vector<Point> >& all_contours, vector< vector<Point> >& contours);
    void GetDigitRoi(vector< vector<Point> >& contours, Mat& binary_img, RoiAreaInfo& roiAreaInfos);
    void ColorFilter(Mat& rawCameraImg, Mat& median_blur_light_img);
    void ThresholdProcess(Mat& median_blur_light_img, Mat& imgBinary);
    void DigitClassify(RoiAreaInfo& roiAreaInfos, Mat& rawCameraImg, basicOCR* KNNocr, const char* baseDir);
};


