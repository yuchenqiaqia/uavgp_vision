#include "define.h"

//保存视频
bool saveEnble = false;

float g_CutScaleWidth   = 1.5f;
float g_CutScaleLength  = 1.5f;
float rectClassifyThres = 1.0f;

int main(int argc, char **argv)
{  
    //ros init
    ros::init(argc, argv, "image_publisher");

    //为ros消息发布创建一个新的线程
    pthread_t id;
    int ret;
    ret=pthread_create(&id,NULL,RosThread,NULL);
    if(ret!=0)
    {
        //线程创建失败
        printf ("Create ros_thread error!\n");
        exit (1);
    }

	//初始化分类器
	basicOCR myKNNocr;

    //opencv方式打开相机
    capture.open(0);
    if(false == capture.isOpened())
    {
        cout<<"打开相机失败"<<endl;
        return 0;
    }

	if (true == saveEnble)
	{
		//创建视频
		outputVideo.open("../output/DigitDetect.avi", CV_FOURCC('M','P','4','2'), 20.0, Size(640, 480),true );
	}

    Mat srcImg;
	int k = 0;

//    while( imgPubNode.ok() )
    while( 1 )
	{
		//读图片
		char srcImg_filename[200];	
        sprintf( srcImg_filename,"..\\output\\digitDetect-%d.jpg",k);
		//srct = imread( srcImg_filename );

        //在相机中抓取一帧图像
        capture>>srcImg;

		//防止线缆松动
        if(!srcImg.data)
		{	
            cout<<"读相机失败"<<endl;
            waitKey(50);
			continue;	
		}

        //去除边缘的3个像素
        srcImg = srcImg(Rect(3,3,srcImg.cols-6,srcImg.rows-6));
        resize(srcImg,srcImg,Size(640 *2/shrink,480 *2/shrink));
		if (true == saveEnble)
		{
			//存储原始图像
			char fileName[50];
            sprintf(fileName,"../output/digitDetect-%d.jpg",k);
            imwrite(fileName,srcImg);
		}
		cout<<"imageNo="<<k<<endl;
		k++;
		//imshow("原图像",srct);
		waitKey(1);
		Mat srcGray;
        cvtColor(srcImg,srcGray,CV_BGR2GRAY);
		Mat srcCanny;
		GaussianBlur(srcGray,srcCanny,Size(7,7),0,0);
		Canny(srcCanny,srcCanny,120,30,3);
        imshow("Canny",srcCanny);

        Mat rectResultImg = srcImg.clone();
		//检测四边形
		RectangleDetect( srcCanny, rectResultImg );
		//透视变换
        PerspectiveTransformation(srcImg, rectCandidateImg, rectCategory);
		//对透视变换后的图像检测黑框
		BlackFrameDetect(rectCategory);
		//位置估计
		EstimatePosition(rectResultImg, rectCategory);
		//数字识别
		DigitDetector(rectResultImg, myKNNocr, rectCategory);
		//显示最终结果图像
		imshow("FinalResultImg",rectResultImg);
		if (true == saveEnble)
        {
            outputVideo<<rectResultImg;
        }

        //result image copy to globle image
        rectResultImg.copyTo(g_rectResultImg);

        //清理内存
		rectPossible.clear();
		rectCategory.clear();
		rectCandidateImg.clear();
		g_PrecisionRatio = -1;
		g_AccuracyAmount = -1;

        int c=waitKey(1);
		if (c > 0)
		{
			break;
		}
	}
	return 0;
}


//矩形（四边形）检测
void RectangleDetect(Mat& srcCanny, Mat& resultImg)
{
	Mat srcGray;
	cvtColor(resultImg,srcGray,CV_BGR2GRAY);
	Mat imgBinary;
	int min_size = 100;
	int thresh_size = (min_size/4)*2 + 1;  
	adaptiveThreshold(srcGray, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV 
	//threshold(img_gray, img_bin, 125, 255, THRESH_BINARY_INV|THRESH_OTSU);  
	morphologyEx(imgBinary, imgBinary, MORPH_OPEN, Mat());
	//srcGray = imgBinary;
	//imshow("imgBinary",imgBinary);

	vector< Point > hull;	//凸包点
	vector< vector<Point> >all_contours;
	vector< vector<Point> >contours;
	vector<Vec4i>hierarchy;
	//查找轮廓
	findContours( srcCanny, all_contours, hierarchy ,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
	//cout<<"轮廓总个数=" << all_contours.size() <<endl; 

	//过滤掉太小的轮廓
	for (int i = 0; i < (int)all_contours.size(); ++i)  
	{  
		if ((int)all_contours[i].size() > srcCanny.cols/20*4 && (int)all_contours[i].size()<srcCanny.rows*4)  
		{  
			float area = fabs( (float)contourArea(all_contours[i]) );
			if (area > pow((float)srcGray.rows/10,2))
				contours.push_back(all_contours[i]);  
		}  
	} 
	//cout<<"有效轮廓个数=" << contours.size() <<endl; 

	//四边形筛选
	vector<Point> approxCurve;
	int id = 0;
	for (size_t i=0; i<contours.size(); i++)
	{
		//近似多边形逼近，另外可以试试拟合凸包（可以去除边缘遮挡干扰）
		approxPolyDP(contours[i], approxCurve, double(contours[i].size())*0.005, true);	//double(contours[i].size())*0.05
		//非4边形不感兴趣
		if (approxCurve.size() != 4)
			continue;
		//非凸4边形不感兴趣
		if (!isContourConvex(approxCurve))
			continue;
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

		// 找四边形的最小边、最大边
		float minDist = 1280.0f;
		float maxDist = 0.0f;
		float sideLength[4] = {0};
		for (int n=0; n<(int)approxCurve.size(); n++)
		{
			sideLength[n] = sqrt( pow(float(approxCurve[n].x - approxCurve[(n+1)%4].x),2)+pow(float(approxCurve[n].y - approxCurve[(n+1)%4].y),2) );			
			if (sideLength[n] < minDist)
			{
				minDist = sideLength[n];
			}
			if (sideLength[n] > maxDist)
			{
				maxDist = sideLength[n];
			}
		}
		//竖长方形，0、2号边长应小于1、3号边长
		float minLength = sideLength[0];
		if (minLength > sideLength[2])
		{
			minLength = sideLength[2];
		}
		if (minLength > sideLength[1] || minLength > sideLength[3] )
		{
			continue;
		}
		//各边夹角不可太小
		double angle0 = GetTwoSideAngle(approxCurve[3],approxCurve[0], approxCurve[1]);
		double angle1 = GetTwoSideAngle(approxCurve[0],approxCurve[1], approxCurve[2]);
		double angle2 = GetTwoSideAngle(approxCurve[1],approxCurve[2], approxCurve[3]);
		double angle3 = GetTwoSideAngle(approxCurve[2],approxCurve[3], approxCurve[0]);
		double minAngleThres = 90 - 15;
		double maxAngleThres = 90 + 15;
		//相邻两角不可同时大于90度
		if( (angle0>95 && angle1>95) || (angle1>95 && angle2>95) || (angle2>95 && angle3>95) || (angle3>95 && angle0>95) )
		{
			continue;
		}
		if ( (angle0<minAngleThres || angle0>maxAngleThres) || (angle1<minAngleThres || angle1>maxAngleThres) 
				|| (angle2<minAngleThres || angle2>maxAngleThres) || (angle3<minAngleThres || angle3>maxAngleThres) )
		{
			continue;
		}
		////不能是竖向平行四边形
		//double angle0_0 = GetTwoSideAngle(approxCurve[1],approxCurve[0],Point2f(approxCurve[1].x,approxCurve[0].y) );
		//double angle1_0 = GetTwoSideAngle(approxCurve[0],approxCurve[1],Point2f(approxCurve[1].x,approxCurve[0].y) );
		//if (angle0<80 && angle1>100 && angle0_0>20 && angle1_0>20)
		//{
		//	continue;
		//}
		//四边形最短边不可太小
		float m_minSideLengthAllowed = float(srcCanny.rows/25);
		//最长边与最短边之比不可过小
		float m_maxSideLengthRatio = maxDist/minDist;
		//过滤并存储有效的四边形信息
		if (minDist > m_minSideLengthAllowed && m_maxSideLengthRatio < maxSideLengthRatioAllowed)
		{
			RectMark markTemp;
			for (int n=0; n<4; n++)
			{
				markTemp.m_points.push_back( Point2f( (float)approxCurve[n].x, (float)approxCurve[n].y ) );
			}
			markTemp.area = fabs( (float)contourArea(contours[i]) );
			markTemp.minSideLength = minDist;
			markTemp.maxSideLength = maxDist;
			markTemp.validFlag = true;
			markTemp.indexId = id;
			id++;
			rectPossible.push_back(markTemp);
		}
	 }

	//剔除重合的四边形
	RectErase( rectPossible );
	//四边形分类
	RectClassify(rectPossible, rectCategory);
	//按面积排序
	RectSortByArea( rectCategory );
	//画出各四边形
	DrawAllRect(resultImg, rectCategory);

	return;
}

//求四边形内侧夹角
double GetTwoSideAngle(Point2f p1,Point2f p2, Point2f p3)
{
	//vector1
	double xV1 = p2.x-p1.x;
	double yV1 =p2.y - p1.y; 
	//vector2
	double xV2 = p3.x - p2.x;
	double yV2 = p3.y - p2.y;
	if ((0==xV1 && 0 ==yV1) || (0 == xV2 && 0 == yV2))
		return 0;
	else
		return ( acos((xV1*xV2 + yV1*yV2) / sqrt((xV1*xV1 + yV1*yV1)*(xV2*xV2 + yV2*yV2))) * 180 / 3.1415926 );
}


//剔除重合的四边形
void RectErase( vector<RectMark>& rectPossible )
{
	float rectErr[4] = {0};
	float maxErr = 10;
	for (int i=0; i<(int)rectPossible.size(); ++i)
	{
		for (int j=i+1; j<(int)rectPossible.size(); ++j)
		{
			for (int k=0; k<4; k++)
			{
				//计算两个矩形对应的第k个顶点的像素距离
				rectErr[k] = sqrt(pow((rectPossible[i].m_points[k].x - rectPossible[j].m_points[k].x),2) 
								+ pow((rectPossible[i].m_points[k].y - rectPossible[j].m_points[k].y),2));
				if (rectErr[k] > maxErr)
					break;
			}
			if (rectErr[0] <= maxErr  &&  rectErr[1] <= maxErr  &&  rectErr[2] <= maxErr  &&  rectErr[3] <= maxErr)
			{
				//取重复矩形四个顶点的平均值作为保留顶点
				for (int k=0; k<4; k++)
				{
					rectPossible[j].m_points[k].x = (rectPossible[i].m_points[k].x + rectPossible[j].m_points[k].x)/2;
					rectPossible[j].m_points[k].y = (rectPossible[i].m_points[k].y + rectPossible[j].m_points[k].y)/2;
				}
				//剔除重复矩形的顶点
				rectPossible.erase(rectPossible.begin() + i);
				i--;
				break;
			}
		}
	}
	return;
}

//四边形分类
void RectClassify( vector<RectMark>& rectPossible, vector< vector<RectMark> >& rectCategory)
{
	vector<RectMark> rectArrayTemp;
	for (int i=0;i<(int)rectPossible.size();++i)
	{
		rectArrayTemp.push_back(rectPossible[i]);

		Point2f middlePoint_i = Point2f((rectPossible[i].m_points[0].x+rectPossible[i].m_points[2].x)/2, 
										(rectPossible[i].m_points[0].y+rectPossible[i].m_points[2].y)/2);
		for (int j=i+1;j<(int)rectPossible.size();++j)
		{
			Point2f middlePoint_j = Point2f((rectPossible[j].m_points[0].x+rectPossible[j].m_points[2].x)/2, 
											(rectPossible[j].m_points[0].y+rectPossible[j].m_points[2].y)/2);
			//两个四边形对角线中点的距离
			float twoPointDistance = sqrt(pow(middlePoint_i.x-middlePoint_j.x,2)+pow(middlePoint_i.y-middlePoint_j.y,2));
			float minSide = 0;
			if (rectPossible[i].minSideLength < rectPossible[j].minSideLength)
				minSide = rectPossible[i].minSideLength;
			else
				minSide = rectPossible[j].minSideLength;

			//判断两四边形是否属于为同一物理标志
			if (twoPointDistance < (minSide * rectClassifyThres))
			{
				rectArrayTemp.push_back(rectPossible[j]);
				rectPossible.erase(rectPossible.begin() + j);
				j--;
			}
		}
		//同一类的放在一起
		rectCategory.push_back(rectArrayTemp);
		rectArrayTemp.clear();
	}
	return;
}

//同一类内的四边形按面积排序
void RectSortByArea( vector< vector<RectMark> >& rectCategory )
{
	for (int k=0;k<(int)rectCategory.size();++k)
	{
		//同一类内排序
		for (int i=0;i<(int)rectCategory[k].size();++i)
		{
			for (int j=i+1;j<(int)rectCategory[k].size();++j)
			{
				if (rectCategory[k][j].area < rectCategory[k][i].area)
				{
					swap(rectCategory[k][i],rectCategory[k][j]);
				}
			}
		}
	}
	return;
}


//画出各四边形
void DrawAllRect(Mat& resultImg, vector< vector<RectMark> >& rectCategory)
{
	//image = Mat::zeros(480, 640, CV_8UC3);
	for (int k=0;k<(int)rectCategory.size();++k)
	{
		for (size_t i=0; i<(int)rectCategory[k].size(); i++)
		{
			int b = (unsigned)theRNG() & 255;
			int g = (unsigned)theRNG() & 255;
			int r = (unsigned)theRNG() & 255;			
			line(resultImg, rectCategory[k][i].m_points[0], rectCategory[k][i].m_points[1], Scalar(255,255,0), 3, 8);
			line(resultImg, rectCategory[k][i].m_points[1], rectCategory[k][i].m_points[2], Scalar(255,255,0), 3, 8);
			line(resultImg, rectCategory[k][i].m_points[2], rectCategory[k][i].m_points[3], Scalar(255,255,0), 3, 8);
			line(resultImg, rectCategory[k][i].m_points[3], rectCategory[k][i].m_points[0], Scalar(255,255,0), 3, 8);
			if (0 == i)
			{
				//只在最内侧四边形上标出0、1、2、3
				for (int j=0;j<4;j++)
				{
					circle(resultImg,rectCategory[k][i].m_points[j],4,Scalar(0,255,0),-1);
					char strNumber[200];	
                    sprintf( strNumber,"%d",j);
					putText(resultImg,strNumber,rectCategory[k][i].m_points[j],CV_FONT_HERSHEY_COMPLEX_SMALL,1.0,Scalar(0,255,0),1);
				}
			}
		}
	}
	//cout<<"MarkNo="<<rectCategory.size()<<endl;
	//imshow("矩形检测",resultImg);
	return;
}


//透视变换
void PerspectiveTransformation(Mat& srcImg, vector<Mat>& rectCandidateImg, vector< vector<RectMark> >& rectCategory)
{
	char windowName[50];
	vector<Point2f> imagePoints2d(4);
	for (int i=0; i<(int)rectCategory.size(); i++)
	{
		int minLoopCount = ((int)rectCategory[i].size()>2) ? (2) : ((int)rectCategory[i].size());
		for (int j=0;j<minLoopCount;++j)
		{
			//目标矩形对应图像坐标0~3
			imagePoints2d[0]=Point2d(rectCategory[i][j].m_points[0].x,rectCategory[i][j].m_points[0].y); 
			imagePoints2d[1]=Point2d(rectCategory[i][j].m_points[1].x,rectCategory[i][j].m_points[1].y);
			imagePoints2d[2]=Point2d(rectCategory[i][j].m_points[2].x,rectCategory[i][j].m_points[2].y);
			imagePoints2d[3]=Point2d(rectCategory[i][j].m_points[3].x,rectCategory[i][j].m_points[3].y);

			// 标准Rect在2d空间为100*100的矩形
			Size m_RectSize = Size(300, int(300*1.25));
			// 矩形 4个角点的正交投影标准值
			vector<Point2f> m_RectCorners2d;	
			//vector<Point3f> m_RectCorners3d;	
			m_RectCorners2d.push_back(Point2f(0, 0));
			m_RectCorners2d.push_back(Point2f(float(m_RectSize.width-1), 0));
			m_RectCorners2d.push_back(Point2f(float(m_RectSize.width-1), float(m_RectSize.height-1)));
			m_RectCorners2d.push_back(Point2f(0, float(m_RectSize.height-1) ) );

			// 投影变换,恢复2维标准视图
			Mat canonicalImg;		
			// 得到当前marker的透视变换矩阵M
			Mat M = getPerspectiveTransform(rectCategory[i][j].m_points, m_RectCorners2d);
			// 将当前的marker变换为正交投影
			warpPerspective(srcImg, canonicalImg, M, m_RectSize);
			//存储矩形区域图像
			canonicalImg.copyTo(rectCategory[i][j].perspectiveImg);
			//rectCandidateImg.push_back(canonicalImg);
			//分窗口显示各标准矩形区域
            sprintf(windowName,"标准四边形-%d",j);
			//imshow(windowName, rectCategory[i][j].perspectiveImg);
		}
	}
	return;
}


//黑框检测
void BlackFrameDetect(vector< vector<RectMark> >& rectCategory)
{
	for (int i=0;i<(int)rectCategory.size();++i)
	{
		int minLoopCount = (rectCategory[i].size()>2) ? (2) : (rectCategory[i].size());
		for (int j=0;j<minLoopCount;++j)
		{
			Mat perspectiveImg;
			Mat img =rectCategory[i][j].perspectiveImg;
			if (!img.data)
			{
				cout<<"rectCategory[i][j].perspectiveImg = NULL"<<endl;
//				system("pause");
			}
			if (3 == img.channels())
				cvtColor(img,perspectiveImg,CV_BGR2GRAY);
			else
				img.copyTo(perspectiveImg);

			int averagePixelVal1 = GetBoxFramePixelAverageValue(perspectiveImg);
			int imgCols = perspectiveImg.cols;
			int imgRows = perspectiveImg.rows;
			int cellWidth = imgCols/8;
			int cellheight = imgRows/10;
			Rect roiInside = Rect(cellWidth,cellheight,imgCols-cellWidth*2,imgRows-cellheight*2);
			int averagePixelVal2 = GetBoxFramePixelAverageValue(perspectiveImg(roiInside));
			int diffValue = averagePixelVal2 - averagePixelVal1;
			if (diffValue < averagePixelVal2*0.2)
				continue;
			double thresHoldVal = double((averagePixelVal1 + averagePixelVal2)/g_ThresHold_Val1_Val2_Ratio);

			//二值化
			Mat perspectiveImgBinary;
			threshold(perspectiveImg,perspectiveImgBinary,thresHoldVal,255,THRESH_BINARY);	// THRESH_BINARY | THRESH_OTSU
			imshow("Binary_raw",perspectiveImgBinary);

			//开运算,先腐蚀再膨胀
			Mat element=getStructuringElement(MORPH_ELLIPSE,Size(5,5));
			morphologyEx(perspectiveImgBinary,perspectiveImgBinary,MORPH_OPEN ,element);
			//闭运算,消除小黑点
			element=getStructuringElement(MORPH_ELLIPSE,Size(3,3));
			morphologyEx(perspectiveImgBinary,perspectiveImgBinary,MORPH_CLOSE ,element);
			element=getStructuringElement(MORPH_ELLIPSE,Size(11,11));
			erode(perspectiveImgBinary,perspectiveImgBinary,element);
			//imshow("perspectiveImgBinary",perspectiveImgBinary);

			int binaryImgAveragePixelVal = GetBoxFramePixelAverageValue(perspectiveImgBinary);
			if (binaryImgAveragePixelVal > 50)
				continue;
			//检测到黑框
			rectCategory[i][j].blackFrameDetectedFlag = true;
			//存储可能的数字roi区域
			int widthCut  = int((perspectiveImg.cols/2.0/8.0) * g_CutScaleWidth);
			int heightCut = int((perspectiveImg.rows/2.0/8.0) * g_CutScaleLength);
			Rect roiPossibleDigit = Rect(roiInside.x+widthCut,roiInside.y+heightCut, roiInside.width-widthCut*2,roiInside.height-heightCut*2);
			perspectiveImgBinary(roiPossibleDigit).copyTo(rectCategory[i][j].possibleDigitBinaryImg);
			//imshow("yesBlackFrame",rectCategory[i][j].possibleDigitBinaryImg);
			//有黑框的四边形移至队头
			swap(rectCategory[i][0],rectCategory[i][j]);	
		}
	}
	return;
}

//计算包围黑框像素平均值
int GetBoxFramePixelAverageValue(const Mat& img)
{
	double totalValue = 0;
	int count = 0;
	for (int i=0;i<img.rows;++i)
	{
		if ( i>(img.rows/10) && i<(img.rows - img.rows/10) )
			continue;
        const uchar* data= img.ptr<uchar>(i);
		for (int j=0;j<img.cols*img.channels();++j)
		{
			if ( j>(img.cols/8) && i<(img.cols - img.cols/8) )
				continue;
			totalValue += data[j];
			count++;
		}
	}
    return int(totalValue/count);
}


//单目位置估计
void EstimatePosition(Mat& srcColor, vector< vector<RectMark> >& rectCategory)
{
	vector<Point2f> imagePoints2d(4);
	vector<Point3f> objectPoints3d(4);

	for (int i=0; i<(int)rectCategory.size(); i++)	//size_t
	{
		double outSideRectHalfWidth  = 0.4/2;
		double outSideRectHalfLength = 0.5/2;
		double inSideRectHalfWidth   = 0.3/2;
		double inSideRectHalfLength  = 0.4/2;

		double myOutSideRectHalfWidth  = 0.201/2;
		double myOutSideRectHalfLength = 0.260/2;
		double myInSideRectHalfWidth   = 0.149/2;
		double myInSideRectHalfLength  = 0.200/2;

		//室内试验暂用小标志物代替实际标志物
		outSideRectHalfWidth  = myOutSideRectHalfWidth;
		outSideRectHalfLength = myOutSideRectHalfLength;
		inSideRectHalfWidth   = myInSideRectHalfWidth;
		inSideRectHalfLength  = myInSideRectHalfLength;
		//按是否检测到黑框为实际坐标赋值
		if (false == rectCategory[i][0].blackFrameDetectedFlag)
		{
			//没有检测到黑框，目标内侧矩形上四个点世界坐标
			objectPoints3d[0]=Point3d(-inSideRectHalfWidth, -inSideRectHalfLength, 0);  
			objectPoints3d[1]=Point3d(inSideRectHalfWidth,  -inSideRectHalfLength, 0);
			objectPoints3d[2]=Point3d(inSideRectHalfWidth,   inSideRectHalfLength, 0);
			objectPoints3d[3]=Point3d(-inSideRectHalfWidth,  inSideRectHalfLength, 0);
		}
		else
		{
			//检测到黑框，目标外侧矩形上四个点世界坐标
			objectPoints3d[0]=Point3d(-outSideRectHalfWidth, -outSideRectHalfLength, 0);  
			objectPoints3d[1]=Point3d(outSideRectHalfWidth,  -outSideRectHalfLength, 0);
			objectPoints3d[2]=Point3d(outSideRectHalfWidth,   outSideRectHalfLength, 0);
			objectPoints3d[3]=Point3d(-outSideRectHalfWidth,  outSideRectHalfLength, 0);
		}

		//目标矩形对应图像坐标0~3
		imagePoints2d[0]=Point2d(rectCategory[i][0].m_points[0].x,rectCategory[i][0].m_points[0].y); 
		imagePoints2d[1]=Point2d(rectCategory[i][0].m_points[1].x,rectCategory[i][0].m_points[1].y);
		imagePoints2d[2]=Point2d(rectCategory[i][0].m_points[2].x,rectCategory[i][0].m_points[2].y);
		imagePoints2d[3]=Point2d(rectCategory[i][0].m_points[3].x,rectCategory[i][0].m_points[3].y);

		Mat rvec,tvec;
		//计算旋转矩阵、平移矩阵
		solvePnP(objectPoints3d,imagePoints2d,t_cameraMatrix,t_distcoef,rvec,tvec);

		//在图像中显示平移向量（x,y,z）
		Mat imgColor;
		//cvtColor(srcGray,imgColor,CV_GRAY2BGR);
		imgColor = srcColor;
		double tvec_x,tvec_y,tvec_z;
		tvec_x=tvec.at<double>(0,0),tvec_y=tvec.at<double>(1,0), tvec_z=tvec.at<double>(2,0);
		//剔除过远或过近的
		if(tvec_z>10 || tvec_z<0.1)
		{
			rectCategory.erase(rectCategory.begin()+i);
			i--;
		}
		char transf[50];
		//sprintf_s(transf,"T%u:[%0.3fm,%0.3fm,%0.3fm]",i,tvec_y,tvec_x,tvec_z);
        sprintf(transf,"%0.3fm",tvec_z);
		Point T_showCenter;
		T_showCenter=Point2d(rectCategory[i][0].m_points[1].x,(rectCategory[i][0].m_points[1].y + rectCategory[i][0].m_points[2].y)/2);
		putText(imgColor, transf, T_showCenter,CV_FONT_HERSHEY_PLAIN,1.0,Scalar(0,0,255),2);
		//imshow("矩形检测",imgColor);
	}
	return;
}


//数字识别
void DigitDetector(Mat& ResultImg, basicOCR& ocr, vector< vector<RectMark> >& rectCategory)
{
	for (int i=0;i<(int)rectCategory.size();++i)
	{
		bool blackFrameDetectFlag = rectCategory[i][0].blackFrameDetectedFlag;
		if (false == blackFrameDetectFlag)
		{
			//黑框外边缘没有检测到
			Mat perspectiveImg;
			if (3 == rectCategory[i][0].perspectiveImg.channels())
			{
				cvtColor(rectCategory[i][0].perspectiveImg,perspectiveImg,CV_BGR2GRAY);
			}
			else
			{
				perspectiveImg = rectCategory[i][0].perspectiveImg.clone();
			}
			int widthCut  = int((perspectiveImg.cols/2.0/8.0) * g_CutScaleWidth);
			int heightCut = int((perspectiveImg.rows/2.0/8.0) * g_CutScaleLength);
			Rect roiPossibleDigit = Rect(widthCut,heightCut, perspectiveImg.cols-widthCut*2,perspectiveImg.rows-heightCut*2);
			//裁剪掉边缘
			perspectiveImg = perspectiveImg(roiPossibleDigit);
			//按整个内区域取阈值
			double averageAllPixelsVal = (double)GetAllPixelAverageValue(perspectiveImg);
			double thresHoldValAllPixels = double(averageAllPixelsVal*2.0/g_ThresHold_Val1_Val2_Ratio);
			//按内区域外边一圈取阈值
			int averagePixelVal = GetBoxFramePixelAverageValue(perspectiveImg);
			int thresHoldVal = int(averagePixelVal*2.0/3.5);//2/4

			//二值化
			Mat perspectiveImgBinary;
			threshold(perspectiveImg,perspectiveImgBinary,thresHoldValAllPixels,255,THRESH_BINARY);	// THRESH_BINARY | THRESH_OTSU
			imshow("Binary_raw",perspectiveImgBinary);
			//形态学处理
			//开运算,先腐蚀再膨胀，消除小白点
			Mat element=getStructuringElement(MORPH_ELLIPSE,Size(5,5));
			morphologyEx(perspectiveImgBinary,perspectiveImgBinary,MORPH_OPEN ,element);
			//闭运算,消除小黑点
			element=getStructuringElement(MORPH_ELLIPSE,Size(3,3));
			morphologyEx(perspectiveImgBinary,perspectiveImgBinary,MORPH_CLOSE ,element);
			element=getStructuringElement(MORPH_ELLIPSE,Size(11,11));
			erode(perspectiveImgBinary,perspectiveImgBinary,element);
			//存入vector
			perspectiveImgBinary.copyTo(rectCategory[i][0].possibleDigitBinaryImg);
			//imshow("DigitBinaryImg",rectCategory[i][0].possibleDigitBinaryImg);
		}
		
		//进一步判断是否是真正的数字区域,增强程序鲁棒性		
		Mat possibleDigitBinaryImg = rectCategory[i][0].possibleDigitBinaryImg.clone();
		resize(possibleDigitBinaryImg,possibleDigitBinaryImg,Size(128,128));
		imshow("waitClassifyImg",possibleDigitBinaryImg);
		//预判断
		double pixelAverageValue = GetAllPixelAverageValue(possibleDigitBinaryImg);
		if (pixelAverageValue > 253 || pixelAverageValue < 2)
		{
			rectCategory[i][0].validFlag = false;
			continue;
		}
		//数字识别
		IplImage ipl_img(possibleDigitBinaryImg); 
		ocrImg = &ipl_img;
		float classResult = ocr.classify(ocrImg,1);
		//打印识别结果
		float precisionRatio = g_PrecisionRatio;
		int accuracyAmount   = g_AccuracyAmount;
		printf("Result=%d   Precision=%0.1f%%\n\n",(int)classResult,precisionRatio);

		//识别结果在图像中显示
		char digit[50];
        sprintf(digit,"%d",int(classResult));
		int x = int((rectCategory[i][0].m_points[0].x + rectCategory[i][0].m_points[2].x)/2) - int(rectCategory[i][0].minSideLength/10);
		int y = int((rectCategory[i][0].m_points[0].y + rectCategory[i][0].m_points[2].y)/2) + int(rectCategory[i][0].minSideLength/10);
		if(x <= 0)
			x = ResultImg.cols/64;
		if (x >= ResultImg.cols)
			x = ResultImg.cols - ResultImg.cols/64;
		if(y <= 0)
			y = ResultImg.rows/64;
		if (y >= ResultImg.rows)
			y = ResultImg.rows - ResultImg.rows/64;	
		Point showCenter = Point(x,y);
		//double textScale = double( float(rectCategory[i][0].maxSideLength/ResultImg.rows)*8 + 1.0 );
		putText(ResultImg, digit, showCenter,CV_FONT_HERSHEY_DUPLEX,1.0,Scalar(0,255,0),2);
		//putText(ResultImg, digit, showCenter,CV_FONT_HERSHEY_DUPLEX,3.0,Scalar(0,255,0),3);
		imshow("Classify",possibleDigitBinaryImg);
	}
}

//图像像素平均值
double GetAllPixelAverageValue(Mat& img)
{
	double totalValue = 0;
	double nCount = 0;
	for (int i=0;i<img.rows;++i)
	{
		uchar* data= img.ptr<uchar>(i);
		for (int j=0;j<img.cols*img.channels();++j)
		{
			totalValue += (double)data[j];
			nCount += 1;
		}
	}
	return totalValue/nCount;
}


//判断是否是有效图像
int FindValidContours(Mat& src)
{
	vector< vector<Point> >contours;
	vector<Vec4i>hierarchy;
	//查找轮廓
	findContours( src, contours, hierarchy ,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
	if (0 == contours.size())
	{
		return 0;
	}
	float area = float(src.cols * src.rows);
	for (int i=0;i<(int)contours.size();++i)
	{
		float areaTemp = fabs( (float)contourArea(contours[i]) );
		if (areaTemp < area)
		{
			area = areaTemp;
		}
	}
	//float minAreaThres = (float)src.cols/20 * (float)src.rows/30;
	//if (area < minAreaThres)
	//{
	//	return 0;
	//}
	return 1;
}
