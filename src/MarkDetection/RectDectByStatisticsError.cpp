/*
 * @file	: RectDetectByStatisticError.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"

class LinearEquation
{
public:
    LinearEquation()
    {
        line_length = 0;
        statistics_points_num = 0;
        inline_points_num = 0;
        is_side_of_rect = false;
    }
    double A;
    double B;
    double C;
    Point vertex0;
    Point vertex1;
    double line_length;

    int statistics_points_num;
    int inline_points_num;
    bool is_side_of_rect;
};

class ContourVertexesInfo
{
public:
    ContourVertexesInfo()
    {
        for(int i=0; i<4; ++i)
            min_dis_between_rect_vertex_and_contour[i] = 1344;
    }
    RotatedRect min_area_rect_box;
    Point2f min_area_rect_vertex[4];
    Point2f contour_vertex[4];
    double min_dis_between_rect_vertex_and_contour[4];
};

class ContoursInfo
{
public:
    vector< vector<Point> > contours;
    vector< ContourVertexesInfo > rotatedRectsInfo;
    vector< vector<LinearEquation> > linearEquationsInfo;
    vector< int > isRects;
};

class RectInfo
{
public:
    Point2f deteced_vertex[4];
    Point real_vertex[4];
    float width;
    Point2f imagePos2D;
    Point3f cameraPos3D;
    Point3d negPos3D;
};

void GetPossibleRectVertexes(vector< vector<Point> >& contours, ContoursInfo& contoursInfo, Mat& input_img)
{
    Mat show_img = input_img;

    for(int k=0;k<contours.size();++k)
    {
        ContourVertexesInfo contour_vertexes;
        vector<Point> contour;
        contour = contours[k];

        RotatedRect boxTemp = minAreaRect( Mat(contour) );
        Point2f vertex[4];
        boxTemp.points(vertex);

        //四个顶点排序，顺时针：0，1，2，3，左上角为0；
        for(int m=0;m<4;m++)
        {
            for (int n=m+1;n<4;n++)
            {
                if (vertex[m].y > vertex[n].y)
                    std::swap(vertex[m], vertex[n]);
            }
        }
        if (vertex[0].x > vertex[1].x)
            std::swap(vertex[0], vertex[1]);
        if (vertex[3].x > vertex[2].x)
            std::swap(vertex[2], vertex[3]);

        // 找四边形的最小边、最大边
        float minDist = float(1384);
        float maxDist = 0.0f;
        float sideLength[4] = {0};
        for (int n=0; n<4; ++n)
        {
            sideLength[n] = sqrt( pow(float(vertex[n].x - vertex[(n+1)%4].x),2)+pow(float(vertex[n].y - vertex[(n+1)%4].y),2) );
            if (sideLength[n] < minDist)
            {
                minDist = sideLength[n];
            }
            if (sideLength[n] > maxDist)
            {
                maxDist = sideLength[n];
            }
        }
        if (maxDist/minDist > 4)
            continue;
        if (boxTemp.size.width*boxTemp.size.height < pow(show_img.rows*0.2,2) || boxTemp.size.width*boxTemp.size.height > pow(show_img.rows*0.9,2))
            continue;

        contour_vertexes.min_area_rect_box = boxTemp;
        for (int i=0; i<4; ++i)
        {
            contour_vertexes.min_area_rect_vertex[i] = vertex[i];
        }

        for (int i=0; i<contour.size(); ++i)
        {
            double dis[4];
            for(int j=0; j<4; ++j)
            {
                dis[j] = sqrt(pow(contour[i].x - vertex[j].x, 2) + pow(contour[i].y - vertex[j].y, 2));
                if (dis[j] < contour_vertexes.min_dis_between_rect_vertex_and_contour[j])
                {
                    contour_vertexes.contour_vertex[j] = contour[i];
                    contour_vertexes.min_dis_between_rect_vertex_and_contour[j] = dis[j];
                }
            }
        }

        int point_num = 0;
        double thres = 0.2;
        float side_length = (boxTemp.size.width<=boxTemp.size.height ? boxTemp.size.width : boxTemp.size.height);
        for(int j=0;j<4;++j)
        {
            if (contour_vertexes.min_dis_between_rect_vertex_and_contour[j] < side_length*thres)
                point_num++;
        }

        if ( point_num < 3 )
            continue;

        contour_vertexes.contour_vertex[2] = vertex[2];
        contour_vertexes.contour_vertex[3] = vertex[3];

        //各边夹角不可太小
        double angle0 = GetTwoSideAngle(contour_vertexes.contour_vertex[3],contour_vertexes.contour_vertex[0], contour_vertexes.contour_vertex[1]);
        double angle1 = GetTwoSideAngle(contour_vertexes.contour_vertex[0],contour_vertexes.contour_vertex[1], contour_vertexes.contour_vertex[2]);
        double angle2 = GetTwoSideAngle(contour_vertexes.contour_vertex[1],contour_vertexes.contour_vertex[2], contour_vertexes.contour_vertex[3]);
        double angle3 = GetTwoSideAngle(contour_vertexes.contour_vertex[2],contour_vertexes.contour_vertex[3], contour_vertexes.contour_vertex[0]);
        double minAngleThres = 90 - 30;
        double maxAngleThres = 90 + 30;
        //相邻两角不可同时大于90度
        if( (angle0>100 && angle1>100) || (angle1>100 && angle2>100) || (angle2>100 && angle3>100) || (angle3>100 && angle0>100) )
            continue;
        if ( (angle0<minAngleThres || angle0>maxAngleThres) || (angle1<minAngleThres || angle1>maxAngleThres)
                || (angle2<minAngleThres || angle2>maxAngleThres) || (angle3<minAngleThres || angle3>maxAngleThres) )
            continue;

        //画出最小面积的包围矩形
        for(int j=0; j<4;++j)
        {
            line(show_img, contour_vertexes.contour_vertex[j], contour_vertexes.contour_vertex[(j+1)%4], Scalar(255,255,0), 3, 8);
            circle(show_img,contour_vertexes.contour_vertex[j],4,Scalar(0,255,0),-1);
            char strNumber[200];
            sprintf( strNumber,"%d",j);
            putText(show_img,strNumber,contour_vertexes.contour_vertex[j],CV_FONT_HERSHEY_COMPLEX_SMALL,1.0,Scalar(0,255,0),1);
        }
        //save
        contoursInfo.contours.push_back(contour);
        contoursInfo.rotatedRectsInfo.push_back(contour_vertexes);
    }

    return;
}

int GetLinearEquationByTwoPoint(Point& input_point0, Point& input_point1, LinearEquation& output_linear_equation)
{
    double A = 0;
    double B = 0;
    double C = 0;

    if (input_point1.x == input_point0.x)
    {
        A = 1;
        B = 0;
        C = -input_point0.x;
        output_linear_equation.A = A;
        output_linear_equation.B = B;
        output_linear_equation.C = C;
        output_linear_equation.vertex0 = input_point0;
        output_linear_equation.vertex1 = input_point1;
        output_linear_equation.line_length = sqrt(pow(input_point1.x - input_point0.x, 2) + pow(input_point1.y - input_point0.y, 2));
        return 0;
    }

    double k = double(input_point1.y - input_point0.y)/(input_point1.x - input_point0.x);
    A = k;
    B = -1;
    C = input_point0.y - k*input_point0.x;
    output_linear_equation.A = A;
    output_linear_equation.B = B;
    output_linear_equation.C = C;
    output_linear_equation.vertex0 = input_point0;
    output_linear_equation.vertex1 = input_point1;
    output_linear_equation.line_length = sqrt(pow(input_point1.x - input_point0.x, 2) + pow(input_point1.y - input_point0.y, 2));

    return 1;
}


void ErrorStatisticsBetweenRectAndContour(ContoursInfo& contoursInfo, Mat& input_img)
{
    for(int k=0;k<(int)contoursInfo.rotatedRectsInfo.size();++k)
    {
        vector< vector<Point3d> >  points_error(4);
        vector<LinearEquation>  linear_equation(4);
        vector<Point> vertex(4);
        for (int i=0; i<4; ++i)
            vertex[i] = Point(contoursInfo.rotatedRectsInfo[k].contour_vertex[i].x,contoursInfo.rotatedRectsInfo[k].contour_vertex[i].y);
        for (int i=0; i<4; ++i)
        {
            GetLinearEquationByTwoPoint(vertex[i%4], vertex[(i+1)%4], linear_equation[i]);
        }
        contoursInfo.linearEquationsInfo.push_back( linear_equation );

        vector<Point> contour;
        contour = contoursInfo.contours[k];
        double thres = 0.1;  //必须小于0.125
        for (int i=0; i<(int)contour.size(); ++i)
        {
            double d0 = fabs(linear_equation[0].A*contour[i].x + linear_equation[0].B*contour[i].y + linear_equation[0].C)/(sqrt(pow(linear_equation[0].A,2) + pow(linear_equation[0].B,2)));
            double d1 = fabs(linear_equation[1].A*contour[i].x + linear_equation[1].B*contour[i].y + linear_equation[1].C)/(sqrt(pow(linear_equation[1].A,2) + pow(linear_equation[1].B,2)));
            double d2 = fabs(linear_equation[2].A*contour[i].x + linear_equation[2].B*contour[i].y + linear_equation[2].C)/(sqrt(pow(linear_equation[2].A,2) + pow(linear_equation[2].B,2)));
            double d3 = fabs(linear_equation[3].A*contour[i].x + linear_equation[3].B*contour[i].y + linear_equation[3].C)/(sqrt(pow(linear_equation[3].A,2) + pow(linear_equation[3].B,2)));
            if ( d0 < linear_equation[0].line_length*thres && (d0<d1 && d0<d2 && d0<d3) )
            {
                points_error[0].push_back(Point3d(contour[i].x, contour[i].y, d0));
                linear_equation[0].statistics_points_num++;
            }
            else if ( d1 < linear_equation[1].line_length*thres && (d1<d0 && d1<d2 && d1<d3) )
            {
                points_error[1].push_back(Point3d(contour[i].x, contour[i].y, d1));
                linear_equation[1].statistics_points_num++;
            }
            else if ( d2 < linear_equation[2].line_length*thres && (d2<d0 && d2<d1 && d2<d3) )
            {
                points_error[2].push_back(Point3d(contour[i].x, contour[i].y, d2));
                linear_equation[2].statistics_points_num++;
            }
            else if ( d3 < linear_equation[3].line_length*thres && (d3<d0 && d3<d1 && d3<d2) )
            {
                points_error[3].push_back(Point3d(contour[i].x, contour[i].y, d3));
                linear_equation[3].statistics_points_num++;
            }
        }

        for (int m=0; m<4; ++m)
        {
            for(int n=0; n<points_error[m].size(); ++n)
            {
                if (points_error[m][n].z < linear_equation[m].line_length*thres*0.5)
                {
                    linear_equation[m].inline_points_num++;
                }
            }
        }

        for(int m=0; m<4; ++m)
        {
            float line_length_thres;
            if (0 == m)
                line_length_thres = 0.85;
            else
                line_length_thres = 0.5;
            if ( linear_equation[m].inline_points_num>linear_equation[m].line_length*0.5 && linear_equation[m].inline_points_num > linear_equation[m].statistics_points_num*0.8)
            {
                linear_equation[m].is_side_of_rect = true;
            }
        }

        if (true == linear_equation[0].is_side_of_rect && true == linear_equation[1].is_side_of_rect && true == linear_equation[3].is_side_of_rect)
        {
            contoursInfo.isRects.push_back(1);
            ContourVertexesInfo contour_vertexes;
            contour_vertexes = contoursInfo.rotatedRectsInfo[k];
            //for(int i=0;i<4;++i)
                //line(input_img, contour_vertexes.contour_vertex[i], contour_vertexes.contour_vertex[(i+1)%4], Scalar(0,0,255), 3, 8);
        }
        else
        {
            contoursInfo.isRects.push_back(0);
        }
    }

    int rects_num = 0;
    for (int i=0;i<(contoursInfo.isRects.size());++i)
    {
        if ( 1 == contoursInfo.isRects[i] )
            rects_num++;
    }
    //printf("rects number = %d\n", rects_num);
    return;
}

void GetTheTargetRect(ContoursInfo& contoursInfo, vector<RectInfo>& rectsInfo, Mat& input_img)
{
    for(int i=0;i<contoursInfo.isRects.size();++i)
    {
        if (1 == contoursInfo.isRects[i])
        {
            Point2f middle_point = Point2f( (contoursInfo.rotatedRectsInfo[i].contour_vertex[0].x + contoursInfo.rotatedRectsInfo[i].contour_vertex[2].x)/2, (contoursInfo.rotatedRectsInfo[i].contour_vertex[0].y + contoursInfo.rotatedRectsInfo[i].contour_vertex[2].y)/2);
            if (middle_point.x>=input_img.cols*0.1 && middle_point.x<=input_img.cols*0.9 && middle_point.y>=input_img.rows*0.1)
            {
                RectInfo rectInfo;
                for(int j=0;j<4;++j)
                {
                    rectInfo.deteced_vertex[j] = contoursInfo.rotatedRectsInfo[i].contour_vertex[j];
                    if (j<2)
                        rectInfo.real_vertex[j] = rectInfo.deteced_vertex[j];
                    else if (2 == j)
                    {
                        float y = rectInfo.deteced_vertex[1].y + contoursInfo.linearEquationsInfo[i][0].line_length*1.333;
                        double A = contoursInfo.linearEquationsInfo[i][1].A;
                        double B = contoursInfo.linearEquationsInfo[i][1].B;
                        double C = contoursInfo.linearEquationsInfo[i][1].C;
                        rectInfo.real_vertex[j] = Point(-(B*y + C)/A, y);
                    }
                    else if (3 == j)
                    {
                        float y = rectInfo.deteced_vertex[0].y + contoursInfo.linearEquationsInfo[i][0].line_length*1.333;
                        double A = contoursInfo.linearEquationsInfo[i][3].A;
                        double B = contoursInfo.linearEquationsInfo[i][3].B;
                        double C = contoursInfo.linearEquationsInfo[i][3].C;
                        rectInfo.real_vertex[j] = Point(-(B*y + C)/A, y);
                    }
                }
                rectInfo.width = contoursInfo.linearEquationsInfo[i][0].line_length;
                Point point_mid_02 = Point((rectInfo.real_vertex[0].x+rectInfo.real_vertex[2].x)/2, (rectInfo.real_vertex[0].y+rectInfo.real_vertex[2].y)/2);
                Point point_mid_13 = Point((rectInfo.real_vertex[1].x+rectInfo.real_vertex[3].x)/2, (rectInfo.real_vertex[1].y+rectInfo.real_vertex[3].y)/2);
                rectInfo.imagePos2D = Point((point_mid_02.x+point_mid_13.x)/2, (point_mid_02.y+point_mid_13.y)/2);
                for(int j=0;j<4;++j)
                    line(input_img, rectInfo.real_vertex[j], rectInfo.real_vertex[(j+1)%4], Scalar(255,0,0), 3, 8);
                rectsInfo.push_back(rectInfo);
            }
        }
    }

    for (int i=0;i<(int)rectsInfo.size();++i)
    {
        Point ref_point = Point(input_img.cols*0.5, input_img.rows*0.6);
        float pixel_dis_i = sqrt(pow(ref_point.x - rectsInfo[i].imagePos2D.x, 2) + pow(ref_point.y - rectsInfo[i].imagePos2D.y, 2));
        for(int j=i+1;j<(int)rectsInfo.size();++j)
        {
            float pixel_dis_j = sqrt(pow(ref_point.x - rectsInfo[j].imagePos2D.x, 2) + pow(ref_point.y - rectsInfo[j].imagePos2D.y, 2));
            //if (rectsInfo[i].width > rectsInfo[j].width)
            if (pixel_dis_i > pixel_dis_j)
                swap(rectsInfo[i],rectsInfo[j]);
        }
    }
    if (rectsInfo.size() > 0)
        circle(input_img,rectsInfo[0].imagePos2D,6,Scalar(0,0,255),-1,8,0);
    return;
}

void EstimateTargetPosition(vector<RectInfo>& rectsInfo, Mat& inputImg)
{
    float shrink = 0.5;
    vector<Point2f> imagePoints2d(4);
    vector<Point3f> objectPoints3d(4);
    const static double kind_0_width = 0.3;
    const static double kind_0_height = 0.4;
    const static double kind_1_width = 0.4;
    const static double kind_1_height = 0.5;
    const static double kind_2_width = 0.6;
    const static double kind_2_height = 0.7;
    double realRectHalfWidth;
    double realRectHalfHeight;
    realRectHalfWidth  = kind_0_width/2;
    realRectHalfHeight = kind_0_height/2;
    //目标矩形上四个点世界坐标
    objectPoints3d[0]=Point3d(-realRectHalfWidth, -realRectHalfHeight, 0);
    objectPoints3d[1]=Point3d(realRectHalfWidth,  -realRectHalfHeight, 0);
    objectPoints3d[2]=Point3d(realRectHalfWidth,   realRectHalfHeight, 0);
    objectPoints3d[3]=Point3d(-realRectHalfWidth,  realRectHalfHeight, 0);

    for (int i=0; i<(int)rectsInfo.size(); ++i)
    {
        //目标矩形对应图像坐标0~3
        imagePoints2d[0]=Point2d((double)rectsInfo[i].real_vertex[0].x,(double)rectsInfo[i].real_vertex[0].y);
        imagePoints2d[1]=Point2d((double)rectsInfo[i].real_vertex[1].x,(double)rectsInfo[i].real_vertex[1].y);
        imagePoints2d[2]=Point2d((double)rectsInfo[i].real_vertex[2].x,(double)rectsInfo[i].real_vertex[2].y);
        imagePoints2d[3]=Point2d((double)rectsInfo[i].real_vertex[3].x,(double)rectsInfo[i].real_vertex[3].y);

        //Point Gray Flea3-14s3c
        double t_fx=893.25550*shrink, t_fy=894.63039*shrink, t_cx=686.67029*shrink, t_cy=518.99170*shrink;
        Mat t_distcoef=(Mat_<double>(1,5) << -0.05173, 0.07077, -0.00047, 0.00061,0);
        Mat t_cameraMatrix=(Mat_<double>(3,3) << t_fx,0,t_cx,0,t_fy,t_cy,0,0,1);

        //计算旋转矩阵、平移矩阵
        Mat rvec,tvec;
        solvePnP(objectPoints3d,imagePoints2d,t_cameraMatrix,t_distcoef,rvec,tvec);

        double tvec_x,tvec_y,tvec_z;
        tvec_x=tvec.at<double>(0,0);
        tvec_y=tvec.at<double>(1,0);
        tvec_z=tvec.at<double>(2,0);
        rectsInfo[i].cameraPos3D = Point3d( tvec_x,tvec_y,tvec_z );
        //剔除过远或过近的
        if(tvec_z>1 || tvec_z<0.2)
        {
            rectsInfo.erase(rectsInfo.begin()+i);
            i--;
            continue;
        }

        char transf[50];
        sprintf(transf,"T:[%0.3fm,%0.3fm,%0.3fm]",tvec_x,tvec_y,tvec_z);
        Point T_showCenter;
        T_showCenter=Point2d(3,20);
        if (0 == i)
            putText(inputImg, transf, T_showCenter,CV_FONT_HERSHEY_PLAIN,1.4,Scalar(255,255,0),2);
    }
    return;
}

int RectDetectByStatisticsError(Mat& input_img, vector< vector<VisionResult> >& lastValidResult, vector<VisionResult>& incompleteRectResult)
{
    if (lastValidResult.size() < 1)
        return 0;
    float dist = 0;
    float sum = 0;
    int num = 0;
    for(int i=0;i<lastValidResult[0].size();++i)
    {
        sum += lastValidResult[0][i].cameraPos3D.z;
        num++;
    }
    dist = sum/num;
    if (dist > 1.0)
        return 0;

    resize(input_img, input_img, Size(1384*0.5,1032*0.5), 0, 0, INTER_LINEAR);
    Mat show_rect = input_img;
    Mat show_img = input_img.clone();

    Mat gaussianImg,cannyImg;
    GaussianBlur(show_img, gaussianImg, Size(7,7),0,0);
    //imshow("gauss",gaussianImg);
    Canny(gaussianImg,cannyImg,200,80);
    //imshow("canny",cannyImg);
/*
    Mat srcGray;
    cvtColor(show_img,srcGray,CV_BGR2GRAY);
    Mat imgBinary;
    int min_size = 100; //100
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(srcGray, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV

    Mat element;
    element=getStructuringElement(MORPH_ELLIPSE, Size( 5,5 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    morphologyEx(imgBinary, imgBinary, MORPH_CLOSE ,element);
    //imshow("StatisticsErrorBinary", imgBinary);
*/
    vector< vector<Point> > all_contours;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( cannyImg, all_contours, hierarchy , RETR_LIST, CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
    //printf("Contours number before filter: %d\n", int(all_contours.size()) );

    ////filter too small contours
    for (int i = 0; i < (int)all_contours.size(); ++i)
    {
        if ((int)all_contours[i].size() > input_img.rows*0.1*3 )
        {
           contours.push_back(all_contours[i]);
        }
    }
    //cout<<"contours num=" << contours.size() <<endl;

    ContoursInfo contoursInfo;
    GetPossibleRectVertexes(contours, contoursInfo, show_img);

    //drawContours(show_img, contours, -1, Scalar(0,0,255), 1 );
    //imshow("all_contours", show_img);

    ErrorStatisticsBetweenRectAndContour(contoursInfo, show_rect);
    vector<RectInfo> rectsInfo;
    GetTheTargetRect(contoursInfo, rectsInfo, show_rect);
    EstimateTargetPosition(rectsInfo, show_rect);

    if (rectsInfo.size()>0)
    {
       VisionResult  incomplete_rect;
       incomplete_rect.digitNo = 11;
       incomplete_rect.imagePos2D.x = rectsInfo[0].imagePos2D.x;
       incomplete_rect.imagePos2D.y = rectsInfo[0].imagePos2D.y;
       incomplete_rect.cameraPos3D.x = rectsInfo[0].cameraPos3D.x;
       incomplete_rect.cameraPos3D.y = rectsInfo[0].cameraPos3D.y;
       incomplete_rect.cameraPos3D.z = rectsInfo[0].cameraPos3D.z;
       incompleteRectResult.push_back(incomplete_rect);
    }
    //imshow("rects",show_rect);
    return 1;
}
