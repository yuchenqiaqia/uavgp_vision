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
    vector<VisionResult> resultRects;
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
        if (boxTemp.size.width*boxTemp.size.height < pow(show_img.rows*0.1,2) || boxTemp.size.width*boxTemp.size.height > pow(show_img.rows*0.9,2))
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

        vector<Point> contour;
        contour = contoursInfo.contours[k];
        double thres = 0.1;  //必须小于0.125
        for (int i=0; i<(int)contour.size(); ++i)
        {
            double d0 = fabs(linear_equation[0].A*contour[i].x + linear_equation[0].B*contour[i].y + linear_equation[0].C)/(sqrt(pow(linear_equation[0].A,2) + pow(linear_equation[0].B,2)));
            double d1 = fabs(linear_equation[1].A*contour[i].x + linear_equation[1].B*contour[i].y + linear_equation[1].C)/(sqrt(pow(linear_equation[1].A,2) + pow(linear_equation[1].B,2)));
            double d2 = fabs(linear_equation[2].A*contour[i].x + linear_equation[2].B*contour[i].y + linear_equation[2].C)/(sqrt(pow(linear_equation[2].A,2) + pow(linear_equation[2].B,2)));
            double d3 = fabs(linear_equation[3].A*contour[i].x + linear_equation[3].B*contour[i].y + linear_equation[3].C)/(sqrt(pow(linear_equation[3].A,2) + pow(linear_equation[3].B,2)));
            if ( d0 < linear_equation[0].line_length*thres && contour[i].x>linear_equation[0].vertex0.x && contour[i].x<linear_equation[0].vertex1.x && (d0<d1 && d0<d2 && d0<d3) )
            {
                points_error[0].push_back(Point3d(contour[i].x, contour[i].y, d0));
                linear_equation[0].statistics_points_num++;
            }
            else if ( d1 < linear_equation[1].line_length*thres && contour[i].y>linear_equation[1].vertex0.y && contour[i].y<linear_equation[1].vertex1.y && (d1<d0 && d1<d2 && d1<d3) )
            {
                points_error[1].push_back(Point3d(contour[i].x, contour[i].y, d1));
                linear_equation[1].statistics_points_num++;
            }
            else if ( d2 < linear_equation[2].line_length*thres && contour[i].x>linear_equation[2].vertex1.x && contour[i].x<linear_equation[2].vertex0.x && (d2<d0 && d2<d1 && d2<d3) )
            {
                points_error[2].push_back(Point3d(contour[i].x, contour[i].y, d2));
                linear_equation[2].statistics_points_num++;
            }
            else if ( d3 < linear_equation[3].line_length*thres && contour[i].y>linear_equation[3].vertex1.y && contour[i].y<linear_equation[3].vertex0.y && (d3<d0 && d3<d1 && d3<d2) )
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
            for(int i=0;i<4;++i)
                line(input_img, contour_vertexes.contour_vertex[i], contour_vertexes.contour_vertex[(i+1)%4], Scalar(0,0,255), 3, 8);
        }
        else
        {
            contoursInfo.isRects.push_back(0);
        }
    }
    return;
}


int RectDectByStatisticsError(Mat& input_img)
{
    Mat show_img = input_img.clone();
    resize(show_img, show_img, Size(640,480));
    Mat resizedImg = show_img.clone();

    Mat gaussianImg,cannyImg;
    GaussianBlur(resizedImg, gaussianImg, Size(7,7),0,0);
    //imshow("gauss",gaussianImg);
    Canny(gaussianImg,cannyImg,160,80);
    imshow("canny",cannyImg);

    Mat srcGray;
    cvtColor(show_img,srcGray,CV_BGR2GRAY);
    Mat imgBinary;
    int min_size = 100; //100
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(srcGray, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV

    Mat element;
    element=getStructuringElement(MORPH_ELLIPSE, Size( 5,5 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    morphologyEx(imgBinary, imgBinary, MORPH_CLOSE ,element);
    imshow("StatisticsErrorBinary", imgBinary);

    vector< vector<Point> > all_contours;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //查找轮廓
    findContours( cannyImg, all_contours, hierarchy , RETR_LIST, CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
    //printf("Contours number before filter: %d\n", int(all_contours.size()) );

    //过滤掉太小的轮廓
    for (int i = 0; i < (int)all_contours.size(); ++i)
    {
        if ((int)all_contours[i].size() > srcGray.rows*0.1*3 )
        {
           contours.push_back(all_contours[i]);
        }
    }
    cout<<"contours num=" << contours.size() <<endl;

    ContoursInfo contoursInfo;
    GetPossibleRectVertexes(contours, contoursInfo, show_img);

    drawContours(show_img, contours, -1, Scalar(0,0,255), 1 );
    imshow("all_contours", show_img);
    //return 0;

    ErrorStatisticsBetweenRectAndContour(contoursInfo, show_img);
    return 1;
}

























int b = (unsigned)theRNG() & 255;
int g = (unsigned)theRNG() & 255;
int r = (unsigned)theRNG() & 255;
int HullQuadRangleDetect(vector<Point>& contour, vector<Point>& output_approxCurve, Mat& input_img)
{
    Mat show_img = input_img.clone();

    //拟合凸包（可以去除边缘遮挡干扰）
    vector<Point> hull( contour.size() );
    convexHull( Mat(contour), hull, false );

    //近似多边形逼近
    vector<Point> approxCurve;
    approxPolyDP(hull, approxCurve, double(contour.size())*0.01, true);	//double(contours[i].size())*0.05
    //非4边形不感兴趣
    if (approxCurve.size() != 4)
        return 0;
    //非凸4边形不感兴趣
    if (!isContourConvex(approxCurve))
        return 0;
    //四个顶点排序，顺时针：0，1，2，3，左上角为0；
    for(int m=0;m<(int)approxCurve.size();m++)
    {
        for (int n=m+1;n<(int)approxCurve.size();n++)
        {
            if (approxCurve[m].y > approxCurve[n].y)
                std::swap(approxCurve[m], approxCurve[n]);
        }
    }
    if (approxCurve[0].x > approxCurve[1].x)
        std::swap(approxCurve[0], approxCurve[1]);
    if (approxCurve[3].x > approxCurve[2].x)
        std::swap(approxCurve[2], approxCurve[3]);

    //画出四边形
    line(show_img, approxCurve[0], approxCurve[1], Scalar(255,0,0), 3, 8);
    line(show_img, approxCurve[1], approxCurve[2], Scalar(255,0,0), 3, 8);
    line(show_img, approxCurve[2], approxCurve[3], Scalar(255,0,0), 3, 8);
    line(show_img, approxCurve[3], approxCurve[0], Scalar(255,0,0), 3, 8);
    //画出四边形的四个顶点
    for (int j=0;j<4;j++)
    {
        circle(show_img, approxCurve[j], 8, Scalar(255,0,0),-1);
        char strNumber[8];
        sprintf( strNumber,"%d",j);
        putText(show_img,strNumber,approxCurve[j],CV_FONT_HERSHEY_COMPLEX_SMALL, 2.0, Scalar(0,255,0), 2);
    }

    resize(show_img, show_img, Size(640,480));
    imshow("hull rect", show_img);
    swap(output_approxCurve, approxCurve);
    return 1;
}
