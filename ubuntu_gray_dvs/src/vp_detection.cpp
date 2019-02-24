#include "vp_detection.h"

vp_detection::vp_detection() 
{

}

/*函数功能：求两条直线交点*/ 
/*输入：两条Vec4i类型直线*/ 
/*返回：Point2f类型的点*/ 
Point2f vp_detection::getCrossPoint(Vec4i LineA, Vec4i LineB,double &rate) 
{ 
	double ka, kb;
	ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); //求出LineA斜率
	kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); //求出LineB斜率 
	rate = 1.0 - abs(ka)/(abs(ka)+abs(kb));
	Point crossPoint;
	crossPoint.x = (ka*LineA[0] - LineA[1] - kb*LineB[0] + LineB[1]) / (ka - kb); 
	crossPoint.y = (ka*kb*(LineA[0] - LineB[0]) + ka*LineB[1] - kb*LineA[1]) / (ka - kb); 
	return crossPoint;
 }

bool vp_detection::Houghlane(cv::Mat canny,cv::Mat &image_mask,vector<Vec4i> &Line_filter, bool left_lane,int nums,int length)	
{
	Line_filter.clear();
	// Hough
	vector<Vec4i> lines;
	vector<Vec4i> Line_store;
	Vec4i line_temp;

	int houghThreshold = 100;//70	
	cv::HoughLinesP(canny, lines, 1, 1*CV_PI/180, houghThreshold, nums,length);//20,20

	while(lines.size() > 100)
	{
		lines.clear();
		houghThreshold += 5;
		cv::HoughLinesP(canny, lines, 1,1*CV_PI/180, houghThreshold, nums,length);//20,20
	}
//	printf("lines size : %d\n",lines.size());

	//定义一个偏移
	int marginx = image_mask.cols/8;
	int marginy = (image_mask.rows-440)/4;
	if(left_lane) //左边车道线
	{
		if(lines.size())
		{
			for(int i=0; i<lines.size(); i++)
			{		
				Point pt1, pt2;
				pt1.x = lines[i][0];
				pt1.y = lines[i][1]+440;
				pt2.x = lines[i][2];
				pt2.y = lines[i][3]+440;
				line_temp[0] = pt1.x;
				line_temp[1] = pt1.y;
				line_temp[2] = pt2.x;
				line_temp[3] = pt2.y;

				int diff_x = pt2.x - pt1.x;
				int diff_y = pt2.y - pt1.y;
				
				//利用斜率滤出部分
				double angle = atan2(double(diff_y),diff_x) * 180 /CV_PI;
				if (abs(angle) <= 15)//25
					continue;
				if (abs(angle) >= 75)//65
					continue;
				
				//利用位置滤波
				if((pt1.x+pt2.x)<image_mask.cols && angle>0)
					continue;
				if((pt1.x+pt2.x)<image_mask.cols/2-marginx)
					continue;
				if((pt1.x+pt2.x)>image_mask.cols/2+marginx)
					continue;

				if((pt1.y+pt2.y)>image_mask.rows+440+marginy)
					continue;
				if((pt1.y+pt2.y)<image_mask.rows+440-marginy)
					continue;
				//printf("angle = %f\n",angle);
				Line_store.push_back(line_temp);
				line(image_mask, pt1, pt2,cv::Scalar(255, 0, 0),2,CV_AA);
			}
//				printf("Line_store size : %d\n",Line_store.size());
		}
		else
		{
			return false;
		}	
	}
	else //右边车道线
	{
		if(lines.size())
		{
			for(int i=0; i<lines.size(); i++)
			{		
				Point pt1, pt2;
				pt1.x = lines[i][0]+image_mask.cols/2;
				pt1.y = lines[i][1]+440;
				pt2.x = lines[i][2]+image_mask.cols/2;
				pt2.y = lines[i][3]+440;
				line_temp[0] = pt1.x;
				line_temp[1] = pt1.y;
				line_temp[2] = pt2.x;
				line_temp[3] = pt2.y;

				int diff_x = pt2.x - pt1.x;
				int diff_y = pt2.y - pt1.y;
				
				//利用斜率滤出部分
				double angle = atan2(double(diff_y),diff_x) * 180 /CV_PI;
				if (abs(angle) <= 15)//25
					continue;
				if (abs(angle) >= 75)//65
					continue;
				
				//利用位置滤波
				if((pt1.x+pt2.x)>=image_mask.cols && angle<0)
					continue;
				if((pt1.x+pt2.x)>=image_mask.cols && angle>50)
					continue;
				if((pt1.x+pt2.x)<image_mask.cols/2*3-marginx)
					continue;
				if((pt1.x+pt2.x)>image_mask.cols/2*3+marginx)
					continue;

				if((pt1.y+pt2.y)>image_mask.rows+440+marginy)
					continue;
				if((pt1.y+pt2.y)<image_mask.rows+440-marginy)
					continue;
			
				//printf("angle = %f\n",angle);
				Line_store.push_back(line_temp);
				line(image_mask, pt1, pt2,cv::Scalar(255, 0, 0),2,CV_AA);
			}
//				printf("Line_store size : %d\n",Line_store.size());
			}
			else
			{
				return false;
			}	
	}

	for(int j = 0;j<Line_store.size();j++)
	{
		Line_filter.push_back(Line_store[j]);
	}

	//如果候选直线大于２条
	int count=0;
	if(Line_store.size()>=2)
	{
		//统计斜率最像的一组
		for(int j = 0;j<Line_store.size();j++)
		{
			for(int i = 0;i<Line_store.size();i++)
			{
				int a0 =  Line_store[j][0] - Line_store[j][2];
				int a1 =  Line_store[j][1] - Line_store[j][3];
				int b0 =  Line_store[i][0] - Line_store[i][2];
				int b1 =  Line_store[i][1] - Line_store[i][3];
				double angle1 = atan2(static_cast<double>(a1),static_cast<double>(a0)) * 180 /CV_PI;
				double angle2 = atan2(static_cast<double>(b1),static_cast<double>(b0)) * 180 /CV_PI;
				double ans = abs(angle2-angle1);
				//		printf("angle1-angle1=%f\n",ans);
				// 如果大量斜率一致且数目超过阈值
				if(ans <= 3)
				{
					count++;
					if(count>Line_store.size()/2)
					{
						Line_filter.push_back(Line_store[j]);
						break;
					}
						
				}
					
			}
			count = 0;
		}		
	}
	else
	{
		return false;
	}

	if(Line_filter.size())
	{
		return true;
	}


	return false;
}

//延长线
void vp_detection::line_longer(Point s, Point e,Point &ss,Point &ee)
{	
	double k=(s.y-e.y)/(s.x-e.x+0.000001);//不加0.000001 会变成曲线，斜率可能为0，即e.x-s.x可能为0
	double h=640,w=768;		
	ss.y=h;
	ss.x=(ss.y-s.y)/k+s.x;	
	ee.y=h/3*2;
	ee.x=(ee.y-e.y)/k+e.x;

}

bool vp_detection::vanish_point_detect(Mat &dst,cv::Mat &image_masks,Point &vp,double &weight)
{
	//分为左右两部分容易算交点
	Mat imgGRAY_L;
	Mat imgGRAY_R;
	imgGRAY_L = dst(Rect(0, 0, dst.cols/2, 200));
	imgGRAY_R = dst(Rect(dst.cols/2, 0, dst.cols/2, 200));
	vector<Vec4i> Line_filters_L;
	vector<Vec4i> Line_filters_R;
	int h1 = Houghlane(imgGRAY_L,image_masks,Line_filters_L,true,25,200); //左边车道线检测
	int h2 = Houghlane(imgGRAY_R,image_masks,Line_filters_R,false,25,200); //右边车道线检测

	Point p1, p2;
	Point p11, p22;
	if(Line_filters_L.size())
	{
		p1.x = Line_filters_L[0][0];
		p1.y = Line_filters_L[0][1];
		p2.x = Line_filters_L[0][2];
		p2.y = Line_filters_L[0][3];
		line_longer(p1,p2,p11,p22);
		line(image_masks, p11, p22,cv::Scalar(0,0,255),5,CV_AA);
	}

	if(Line_filters_R.size())
	{
		p1.x = Line_filters_R[0][0];
		p1.y = Line_filters_R[0][1];
		p2.x = Line_filters_R[0][2];
		p2.y = Line_filters_R[0][3];
		line_longer(p1,p2,p11,p22);
		line(image_masks, p11, p22,cv::Scalar(0,0,255),5,CV_AA);
	}
	//灭点检测
	if(h1&&h2)
	{
//		double weight;//根据斜率不同使得ROI更准
		vp = getCrossPoint(Line_filters_L[0],Line_filters_R[0],weight); 
		circle(image_masks, vp, 3, CV_RGB(0, 255, 0), 5);
	}

//    imshow("imgL",imgGRAY_L);
//    imshow("imgR",imgGRAY_R);
//	imshow("image_mask", image_masks);
	if(h1&&h2)
		return true;
	return false;
}
