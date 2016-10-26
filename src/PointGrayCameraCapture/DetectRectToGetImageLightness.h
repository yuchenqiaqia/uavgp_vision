/*
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */
#include "declare.h"

using namespace std;
using namespace cv;

class DetectRectToGetImageLightness
{
public:
    DetectRectToGetImageLightness();
    void RectangleDetect( );
    double GetTwoSideAngle(Point2f p1,Point2f p2, Point2f p3);
    void RectErase( );
    void RectClassify( );
    void RectSortByArea( );
    void RectSortByPositionX( );
    void DrawAllRect( );
    void EstimatePosition( );
    void GetTheLargestRect();
    void PerspectiveTransformation( );
    double HSVCenterROI( );
    double GetRectAreaLightness(Mat& inputImg);

    double shrink;
    int frameNo;
    float maxDistance;
    float minDistance;
    Mat srcImg;
    Mat resultImg;
    vector< vector<RectMark> > rectCategory;
    vector<RectMark> rectPossible;

    double rect_area_lightness;
};

