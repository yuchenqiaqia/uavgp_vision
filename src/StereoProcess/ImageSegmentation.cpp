#include "declare.h"
int FindObstacleArea( Mat& img_input );

//得到均匀的种子点
int GetSeeds(Mat input_img, vector<Point3f>& seeds)
{
    //printf("GetSeeds: rows=%d\n", input_img.rows);

    vector<Point3f> depth_pixels;
    for(unsigned int j=0;j < input_img.rows;++j)
    {
       for(unsigned int p=0;p<input_img.cols;++p)
       {
           //提取种子点的距离阈值
           if ( input_img.at<Vec3f>(j,p)[2] > 0.75 && input_img.at<Vec3f>(j,p)[2] < 10)
           {
               Point3f temp = Point3f(p, j, input_img.at<Vec3f>(j,p)[2]);
               depth_pixels.push_back(temp);
               //printf("Point3f = (%f,%f,%f)\n", temp.x,temp.y, temp.z);
           }
        }
     }

    if ( depth_pixels.size() <= 200)
    {
        return 0;
    }

    int seeds_num = 30;
    float add_diff = (float)depth_pixels.size()/seeds_num;
    for (float i=0; i<(float)depth_pixels.size(); i+=add_diff)
    {
        seeds.push_back(depth_pixels[ (int)i ]);
    }
    return 1;
}

//对三维图像进行分割
int MyImageSegmentation( Mat& input_xyz_img, Mat& input_gray_img )
{
    if (!input_xyz_img.data)
        return 0;

    static int imgNo = 0;
    Mat gray_img = input_gray_img.clone();
    cvtColor(gray_img, gray_img, CV_GRAY2BGR);

    Mat img = input_xyz_img.clone();
    Mat element=getStructuringElement(MORPH_ELLIPSE, Size(5,5) ); //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    erode(img, img, element);
    element=getStructuringElement(MORPH_ELLIPSE, Size(7,7) );
    dilate(img, img, element);

    vector<Point3f> seeds;
    int have_seeds = GetSeeds( img, seeds);
    if ( 0 == have_seeds )
    {
        return 0;
    }

    float max = 1050;
    float min = 50;
    srand(time(NULL));
    vector< vector<Point> > areas;
    for (int i=0; i<(int)seeds.size(); ++i)
    {
        int x = seeds[i].x;
        int y = seeds[i].y;
        if (img.at<Vec3f>(y,x)[2] < 50)
        {
            float b =  float( (double)rand() / ((double)RAND_MAX + 1) * (max - min) + min );
            float g =  float( (double)rand() / ((double)RAND_MAX + 1) * (max - min) + min );
            float r =  float( (double)rand() / ((double)RAND_MAX + 1) * (max - min) + min );
            Rect ccomp;
            floodFill( img, Point(x,y), Scalar(b,g,r), &ccomp, Scalar(0.5,0.5,0.5), Scalar(0.5,0.5,0.5) );
            vector<Point> one_area_points;
            for(int m=0; m<img.rows; ++m)
            {
                for(int n=0; n<img.cols; ++n)
                {
                    if ( ((int)img.at<Vec3f>(m,n)[0] == (int)b) && ((int)img.at<Vec3f>(m,n)[1] == (int)g) && ((int)img.at<Vec3f>(m,n)[2] == (int)r) )
                    {
                        one_area_points.push_back(Point(n,m));
                    }
                }
            }
            areas.push_back(one_area_points);
        }
    }

    Mat img_show;
    img.convertTo(img_show, CV_8UC3);

    for(int k=0; k<areas.size(); ++k)
    {
        if (areas[k].size() < 1000)
            continue;
        RotatedRect boxTemp = minAreaRect( Mat(areas[k]) );
        if (boxTemp.size.area() < 900)
            continue;
        Point2f vertex[4];
        boxTemp.points(vertex);
        //画出各个最小面积的包围矩形
        for(int j=0; j<4;++j)
        {
            line(gray_img, vertex[j], vertex[(j+1)%4], Scalar(0,0,255), 2, 8);
        }
        Point center = Point(boxTemp.center.x,boxTemp.center.y);
        double distance = GetROI_AverageVal(input_xyz_img, center, 2, boxTemp.size.width/4);
        char dis[50];
        sprintf(dis, "%.2f", distance);
        putText(gray_img, dis, center,  CV_FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255), 2);
    }

    char imgName[1000];
    sprintf(imgName, "/home/sia/dji_guidance/1/gray_%06d.png", imgNo);
    imwrite(imgName, gray_img);
    sprintf(imgName, "/home/sia/dji_guidance/1/color_%06d.png", imgNo);
    imwrite(imgName, img_show);

    imshow("floodFill", img_show);
    imshow("area show", gray_img);
    waitKey(1);
    imgNo++;
    return 1;
}


int FindObstacleArea( Mat& img_input )
{
    Mat show_img = img_input.clone();
    Mat srcGray;
    cvtColor(img_input,srcGray,CV_BGR2GRAY);
    Mat imgBinary;
    int min_size = 12;
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(srcGray, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV
    imshow("obstacle binary img", imgBinary);

    //Mat gaussianImg;
    //GaussianBlur(img_input, gaussianImg, Size(5,5),2,2);
    //Canny(gaussianImg,gaussianImg,150,40);
    //imshow("canny",gaussianImg);
    //imgBinary = gaussianImg;

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( imgBinary, contours, hierarchy ,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL

    for(int i=0; i<contours.size(); ++i)
    {
        RotatedRect boxTemp = minAreaRect( Mat(contours[i]) );
        if (boxTemp.size.area() < 1500)
            continue;
        Point2f vertex[4];
        boxTemp.points(vertex);
        //画出各个最小面积的包围矩形
        for(int j=0; j<4;++j)
        {
            line(show_img, vertex[j], vertex[(j+1)%4], Scalar(0,255,0), 2, 8);
        }
    }
    imshow("min box", show_img);
    return 1;
}
