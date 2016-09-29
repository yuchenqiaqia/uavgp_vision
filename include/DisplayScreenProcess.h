#include "KNN_OCR.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class DisplayScreenProcessType
{
public:
    DisplayScreenProcessType( );
    void DisplayScreenProcess(Mat& input_img, basicOCR* KNNocr, const char* baseDir);

    int imgNo;
    float shrink;
    Mat rawCameraImg;
    Mat binary_roi_img;
    Mat show_img;
    float rect_filter_two_side_ratio_max;
    float rect_filter_two_side_ratio_min;
};
