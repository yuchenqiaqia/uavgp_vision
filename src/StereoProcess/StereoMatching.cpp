#include "StereoMatching.h"


StereoMatchingType::StereoMatchingType()
{
    left_img_has_copied = true;
    right_img_has_copied = true;

    bm = StereoBM::create(16,9);
    sgbm = StereoSGBM::create(0,16,3);

     //left相机内参
    fx_l=238.22462;
    fy_l=237.37011;
    cx_l=158.84752;
    cy_l=120.37943 ;
    M_CamLeft=(Mat_<double>(3,3) << fx_l,0,cx_l,0,fy_l,cy_l,0,0,1);
    D_CamLeft=(Mat_<double>(5,1) << 0.01035, -0.00777, 0.00180, -0.00009,0.000);

     //right相机内参
    fx_r=239.68003;
    fy_r=238.72529;
    cx_r=158.77224;
    cy_r=119.32757 ;
    M_CamRight=(Mat_<double>(3,3) << fx_r,0,cx_r,0,fy_r,cy_r,0,0,1);
    D_CamRight=(Mat_<double>(5,1) << 0.02186, -0.01878, -0.00017, 0.00113,0.000);

    //stereo_RV=(Mat_<double>(3,1) << -0.00489, 0.00366,  0.00041 );
    stereo_RV=(Mat_<double>(3,1) << -0.00489, -0.0009,  0.00041 );      //该参数更准
    stereo_T=(Mat_<double>(3,1) << -150.12139, -0.12009, 1.97403 );

    alg = STEREO_BM;
    BM_SadWindowSize = 13;  //17
    SGBM_SadWindowSize = 7;
    numberOfDisparities =  0;// 21,0
    img_size = Size(320,240);

    distanceZ_min = 0.52;
    distanceZ_max = 8.0;
    dif_value = distanceZ_max - distanceZ_min;

    StereoMatchingType::StereoMatchingConfigration( );
}

void StereoMatchingType::StereoMatchingConfigration( )
{
    float scale = 1.f;

    Rodrigues( stereo_RV, stereo_RM );			//旋转向量到旋转矩阵
    Mat M1, D1, M2, D2;
    M1=M_CamLeft;
    D1=D_CamLeft;
    M2=M_CamRight;
    D2=D_CamRight;

    M1 *= scale;
    M2 *= scale;

    Mat R, T, R1, P1, R2, P2;
    R=stereo_RM;
    T=stereo_T;
    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

    return;
}


void StereoMatchingType::StereoMatching(  )
{
    if(true == left_img_has_copied || true == right_img_has_copied)
    {
        return;
    }

        Mat img1r, img2r;
        remap(img1_raw, img1r, map11, map12, INTER_LINEAR);
        remap(img2_raw, img2r, map21, map22, INTER_LINEAR);
        left_img_has_copied = true;
        right_img_has_copied = true;

        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

        bm->setROI1(roi1);
        bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(BM_SadWindowSize > 0 ? BM_SadWindowSize : 9);
        bm->setMinDisparity(0);
        bm->setNumDisparities(numberOfDisparities);
        bm->setTextureThreshold(10);
        bm->setUniquenessRatio(20);//18
        bm->setSpeckleWindowSize(100);//100
        bm->setSpeckleRange(32);//32
        bm->setDisp12MaxDiff(1);

        //bm.state->roi1 = roi1;
        //bm.state->roi2 = roi2;
        //bm.state->preFilterCap = 31;  //原参数
        //bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
        //bm.state->minDisparity = 0;
        //bm.state->numberOfDisparities = numberOfDisparities;
        //bm.state->textureThreshold = 10;
        //bm.state->uniquenessRatio = 15;
        //bm.state->speckleWindowSize = 100;
        //bm.state->speckleRange = 32;
        //bm.state->disp12MaxDiff = 1;

        sgbm->setPreFilterCap(63);
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        sgbm->setBlockSize(sgbmWinSize);

        int cn = img1r.channels();


        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        if(alg==STEREO_HH)
            sgbm->setMode(StereoSGBM::MODE_HH);
        else if(alg==STEREO_SGBM)
            sgbm->setMode(StereoSGBM::MODE_SGBM);
        else if(alg==STEREO_3WAY)
            sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

        Mat img1p, img2p, dispp;
        copyMakeBorder(img1r, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);  //拓展搜索匹配
        copyMakeBorder(img2r, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

        Mat disp;
        if( alg == STEREO_BM )
            //bm(img1r, img2r, disp,CV_16S);
            //bm(img1p, img2p, dispp,CV_16S);
            //bm->compute(img1p, img2p, dispp);
            bm->compute(img1r, img2r, disp);
        else if(  alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
            //sgbm(img1r, img2r, disp);
            //sgbm(img1p, img2p, dispp);
            sgbm->compute(img1p, img2p, dispp);

        //拓展搜索匹配
        //disp = dispp.colRange(numberOfDisparities, img1p.cols);
        if( alg != STEREO_VAR )
            disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        else
            disp.convertTo(disp8, CV_8U);

        disp8.copyTo(disp8_show);
        //disp8_show=disp8_show(roi1);

        reprojectImageTo3D(disp, mat_xyz, Q, true);
        StereoMatchingType::ConvertXYZToPseudoColor( );

        return;
}

//深度图转为深度伪彩图
void StereoMatchingType::ConvertXYZToPseudoColor( )
{
    Mat pseudo_color = Mat::zeros(mat_xyz.rows, mat_xyz.cols, CV_8UC3);

    for(int r = 0; r < mat_xyz.rows; r++)
    {
        uchar* data_pseudo=pseudo_color.ptr<uchar>(r);
        for(int c = 0; c < mat_xyz.cols*3; c=c+3)
        {
                mat_xyz.at<Vec3f>(r, c/3)[0] = mat_xyz.at<Vec3f>(r, c/3)[0] * 0.016;
                mat_xyz.at<Vec3f>(r, c/3)[1] = mat_xyz.at<Vec3f>(r, c/3)[1] * 0.016;
                mat_xyz.at<Vec3f>(r, c/3)[2] = mat_xyz.at<Vec3f>(r, c/3)[2] * 0.016;
                if (mat_xyz.at<Vec3f>(r, c/3)[2] > 30 || mat_xyz.at<Vec3f>(r, c/3)[2] < 0.6)
                    mat_xyz.at<Vec3f>(r, c/3)[2] = 0;

                Vec3f point_xyz = mat_xyz.at<Vec3f>(r, c/3);
                float X = point_xyz[0];
                float Y = point_xyz[1];
                float Z = point_xyz[2];
                float distanceZ = Z;
                if( distanceZ >= distanceZ_min && distanceZ <= distanceZ_max)
                {
                    int pseudo_raw_value=int( ( (distanceZ-distanceZ_min)/dif_value)*510 );
                    if( pseudo_raw_value <= 255 && pseudo_raw_value > 0 )
                    {
                        data_pseudo[c] = 0;
                        data_pseudo[c+1] = pseudo_raw_value;
                        data_pseudo[c+2] = 255-pseudo_raw_value;
                    }
                    else if(pseudo_raw_value > 255 && pseudo_raw_value < 510)
                    {
                        data_pseudo[c+1] = 510-pseudo_raw_value;
                        data_pseudo[c] = 255-data_pseudo[c+1];
                        data_pseudo[c+2] = 0;
                    }
                    else
                    {
                        data_pseudo[c] = 180;	//180
                        data_pseudo[c+1] =180;
                        data_pseudo[c+2] =180;
                    }
                }
                else	if( distanceZ<distanceZ_min && distanceZ > 0.4)
                {
                    data_pseudo[c] = 0;
                    data_pseudo[c+1] = 0;
                    data_pseudo[c+2] = 255;
                }
                else	if( distanceZ > distanceZ_max && distanceZ < 80)
                {
                    data_pseudo[c] = 255;
                    data_pseudo[c+1] = 0;
                    data_pseudo[c+2] = 0;
                }
                else
                {
                    data_pseudo[c]=180;
                    data_pseudo[c+1]=180;
                    data_pseudo[c+2]=180;
                }
        }
    }
    pseudo_color.copyTo(img_pseudo_color);
    //img_pseudo_color = img_pseudo_color(roi1);
    return;
}
