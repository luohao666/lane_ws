#include <opencv2/opencv.hpp>
#include <ctime> //time
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

Mat threshold_process(Mat src, double thresh_min, double thresh_max, bool scale) {
	Mat abs_src, dst;

	dst = Mat::zeros(src.rows, src.cols, CV_8U);
	if (scale) {
		convertScaleAbs(src, abs_src);
		for (int i = 0; i < abs_src.rows; i++) {
			for (int j = 0; j < abs_src.cols; j++) {
				int v = abs_src.at<uchar>(i, j);
				if (v >= thresh_min && v <= thresh_max)
					dst.at<uchar>(i, j) = 255;
			}
		}
	}
	else {
		src.convertTo(src, CV_8U);
		for (int i = 0; i < src.rows; i++) {
			for (int j = 0; j < src.cols; j++) {
				int v = src.at<uchar>(i, j);
				if (v >= thresh_min && v <= thresh_max)
					dst.at<uchar>(i, j) = 255;
			}
		}
	}

	return dst;
}

Mat perspective_img(Mat img)
{
    float delta = 75;
    float delta2 = 75;

    Mat res;
	//openv.avi	
    Point2f src[4];
    Point2f dst[4];
    Mat M;
	src[0] = cv::Point2f(180+delta, 0);//180,0
	src[1] = cv::Point2f(580-delta-delta2, 0);//580,0
	src[2] = cv::Point2f(740, 180);//730,200
	src[3] = cv::Point2f(20, 180);//30,200
	// to points
	dst[0] = cv::Point2f(0, 0); //30,0
	dst[1] = cv::Point2f(768, 0); //768,0
	dst[2] = cv::Point2f(768,640); //768,200
	dst[3] = cv::Point2f(0, 640); //30,200

    M = getPerspectiveTransform(src,dst);
	warpPerspective(img, res, M, Size(768,640));
    
    line(img,src[0],src[1] , Scalar(255), 2);
	line(img, src[1],src[2],Scalar(255), 2);
	line(img, src[2],src[3],Scalar(255), 2);
	line(img, src[3],src[0],Scalar(255), 2);

    return res;
}

Mat perspective_img_roi(Mat img,int x1,int x2)
{
    Mat res;
	//openv.avi	
    Point2f src[4];
    Point2f dst[4];
    Mat M;
	src[0] = cv::Point2f(x1, 0);//180,0
	src[1] = cv::Point2f(x2, 0);//580,0
	src[2] = cv::Point2f(740, 170);//730,200
	src[3] = cv::Point2f(20, 170);//30,200
	// to points
	dst[0] = cv::Point2f(0, 0); //30,0
	dst[1] = cv::Point2f(768, 0); //768,0
	dst[2] = cv::Point2f(768,640); //768,200
	dst[3] = cv::Point2f(0, 640); //30,200

    M = getPerspectiveTransform(src,dst);
	warpPerspective(img, res, M, Size(768,640));
    
    line(img,src[0],src[1] , Scalar(255), 2);
	line(img, src[1],src[2],Scalar(255), 2);
	line(img, src[2],src[3],Scalar(255), 2);
	line(img, src[3],src[0],Scalar(255), 2);

    return res;
}

/*函数功能：求两条直线交点*/ 
/*输入：两条Vec4i类型直线*/ 
/*返回：Point2f类型的点*/ 
Point2f getCrossPoint(Vec4i LineA, Vec4i LineB,double &rate) 
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

bool Houghlane(cv::Mat canny,cv::Mat &image_mask,vector<Vec4i> &Line_filter, bool left_lane,int nums,int length)	
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
void line_longer(Point s, Point e,Point &ss,Point &ee)
{	
	double k=(s.y-e.y)/(s.x-e.x+0.000001);//不加0.000001 会变成曲线，斜率可能为0，即e.x-s.x可能为0
	double h=640,w=768;		
	ss.y=h;
	ss.x=(ss.y-s.y)/k+s.x;	
	ee.y=h/3*2;
	ee.x=(ee.y-e.y)/k+e.x;

}

bool vanish_point_detect(Mat &dst,cv::Mat &image_masks,Point &vp,double &weight)
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

int main(int argc,char** argv)
{
	cout<< "lane gray"<<endl;
    Mat img;
    Mat evt;

	//lane detection
    Mat blur;
    Mat roi;
    Mat roi_evt;
    Mat sobel_x;
    Mat sobel_y;
    Mat dst;
    Mat ipm;

	//vp detection
	Mat mask;
	Mat roi_canny;
	int count = 0;
	vector<Point> vp_last;
	vector<double> w_last;
	int p1x = 0;
	int p2x = 0;

    int i=0;
    for(;;)
    {
    clock_t time_s1 = clock(); // 计时   
    i++;
    char ss[50];
    char ss2[50];

	sprintf(ss,"../dvs_dataset/90ms/FullPic/Pic%d.bmp",i); 
    sprintf(ss2,"../dvs_dataset/90ms/EventPic/Pic%d.bmp",i);    
   
    img = imread(ss,-1);
    evt = imread(ss2,-1);

    //ROI select
    roi = img(Rect(0,440,img.cols,img.rows-440));
    roi_evt = evt(Rect(0,440,img.cols,img.rows-440));


    //Rect四个形参分别是：x坐标，y坐标，长，高；注意(x,y)指的是矩形的左上角点
    //GaussianBlur(roi, blur, Size(3, 3), 0, 0, BORDER_DEFAULT);
    medianBlur(roi,blur,3);
    Sobel(blur, sobel_x, CV_64F, 1, 0, 3);
    Sobel(blur, sobel_y, CV_64F, 0, 1, 3);
    convertScaleAbs(sobel_x, sobel_x);              //  转化为8位灰度级显示
    convertScaleAbs(sobel_y, sobel_y);              //  转化为8位灰度级显示
    Mat sobel_xx = threshold_process(sobel_x,40,255,1);
    Mat sobel_yy = threshold_process(sobel_y,50,255,1);
    medianBlur(sobel_xx,sobel_xx,3);
    medianBlur(sobel_yy,sobel_yy,3);
    bitwise_and(sobel_xx,sobel_yy,dst);
    cout <<"preprocessing time is " << 1000* (clock() - time_s1)/(double)CLOCKS_PER_SEC << "ms"<< endl;
	
    //vanish point wasting time too much!
	Point vp;
	double w=1;
	int r = 50;
	if(count<3)
	{
		clock_t time_s3 = clock(); // 计时 
		mask = Mat(img.size(), CV_8UC3, Scalar(0));
		mask.setTo(cv::Scalar(0, 0, 0));  
		Canny(roi, roi_canny, 50,100, 3);//180 120
		bool vp_flag = vanish_point_detect(roi_canny,mask,vp,w);
		cout <<"vp time is " << 1000* (clock() - time_s3)/(double)CLOCKS_PER_SEC << "ms"<< endl;
		if(vp_flag)
		{
			vp_last.push_back(vp);
			w_last.push_back(w);
			count++;
		}
		//单独调试vanish_point_detect用
//		imshow("roi_canny",roi_canny);
// 		imshow("mask",mask);
//		waitKey(10); 
		continue;
	}
	printf("vp_last size is:%d\n",vp_last.size());
	printf("count is:%d\n",count);

	if(count == 3)
	{
		if(abs(vp_last[0].x -vp_last[1].x) < r && abs(vp_last[0].x -vp_last[2].x) < r && abs(vp_last[1].x -vp_last[2].x) < r
		&& abs(vp_last[0].y -vp_last[1].y) < r &&abs(vp_last[0].y -vp_last[2].y) < r && abs(vp_last[1].y -vp_last[2].y) < r)
		{
			vp = vp_last[0];
			w = w_last[0];
			count++;
			//利用vp画出感兴趣区域
			p1x = vp.x-200*w;
			p2x = vp.x+200*(1-w);
		//	p1x = vp.x-100;
		//	p2x = vp.x+100;

		}
		else
		{
			count = 2;
			vector<Point>::iterator k = vp_last.begin();
			vp_last.erase(k);//删除第一个元素
		}
	}

    //
    //ipm wasting time too much!
	if(count > 3)
	{
		clock_t time_s2 = clock(); // 计时   
		ipm = perspective_img_roi(dst,p1x,p2x);
		cout <<"perspective time is " << 1000* (clock() - time_s2)/(double)CLOCKS_PER_SEC << "ms"<< endl;
		imshow("ipm",ipm);
	}


    imshow("src",img);
    imshow("roi",roi);
    imshow("blur",blur);
    imshow("x",sobel_x);
    imshow("y",sobel_y);
    imshow("xx",sobel_xx);
    imshow("yy",sobel_yy);
    imshow("and",dst);
    imshow("evt",roi_evt);
  	imshow("mask",mask);

    waitKey(30);    
    }
   
    return 0;
}
