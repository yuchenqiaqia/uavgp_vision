/******************************************************************************

版权所有:  2016, 中国科学院沈阳自动化研究所，一室，旋翼飞行机器人课题组

******************************************************************************
作	  者   : 肖斌
******************************************************************************/
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class StereoMatchingType
{
public:
    StereoMatchingType();
    void StereoMatching( );
    void StereoMatchingConfigration();
    void ConvertXYZToPseudoColor( );

    bool left_img_has_copied;
    bool right_img_has_copied;
    Size img_size;
    Rect roi1, roi2;

    Mat img1_raw;
    Mat img2_raw;
    Mat mat_xyz;
    Mat img_pseudo_color;
    Mat disp8;
    Mat disp8_show;

    double distanceZ_min;
    double distanceZ_max;
    double dif_value;

private:
    double fx_l ;			     //left相机内参
    double fy_l;
    double cx_l;
    double cy_l;

    Mat M_CamLeft;
    Mat D_CamLeft;

    double fx_r;                  //right相机内参
    double fy_r;
    double cx_r;
    double cy_r;
    Mat M_CamRight;
    Mat D_CamRight;

    Mat stereo_RV;
    Mat stereo_RM;
    Mat stereo_T;

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg;
    int BM_SadWindowSize;
    int SGBM_SadWindowSize;
    int numberOfDisparities ;
    int SADWindowSize;

    Ptr<StereoBM> bm;
    Ptr<StereoSGBM> sgbm;
    Mat map11, map12, map21, map22;
    Mat Q;
};

