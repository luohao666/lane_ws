#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <opencv2/videoio.hpp>
#include <highgui.h>
#include <algorithm>
#include <iterator>

#define MAX_NUM_LINES 200
using namespace std;
using namespace cv;

// 如果要提高精度需要排序
//定义一个结构体
typedef struct NODE 
{
	double x;
	int y;
}node;

//从小到大排序
bool comp(node & a, node & b) 
{
	return a.x < b.x;//若前一个元素小于后一个元素，则返回真，否则返回假，即可自定义排序方式
}

//排序模板，返回索引
template < typename T> vector< size_t> sort_indexes(const vector< T> & v)
{ 
	// initialize original index locations 
	vector< size_t> idx(v.size()); 
	for (size_t i = 0; i != idx.size(); ++i) 
	idx[i] = i; 
	// sort indexes based on comparing values in v 
	sort(idx.begin(), idx.end(), [& v](size_t i1, size_t i2)
	{
		return v[i1] < v[i2];
	});
	return idx;
}

//霍夫变换，输入图像，输出掩摸，直线寄存器
bool Houghlane(cv::Mat imgGRAY,cv::Mat &image_mask3,vector<Vec4i> &Line_store,int nums,int length,int v1)	//对line_store进行滤波，判断是否平行
{
	Line_store.clear();

	vector<Vec4i> lines;
	Vec4i line_temp;
	//先存储斜率，然后排序去除差别最大的两根直线
	vector<node> left_angle;
	vector<node> right_angle;

//	v1=25;
	int v2=90-v1;
	int houghThreshold = 100;//70	
	cv::HoughLinesP(imgGRAY, lines, 1, 1*CV_PI/180, houghThreshold, nums,length);//20,20

//	printf("lines size : %d\n",lines.size());

	while(lines.size() > MAX_NUM_LINES)
	{
		lines.clear();
		houghThreshold += 5;
		cv::HoughLinesP(imgGRAY, lines, 1,1*CV_PI/180, houghThreshold, nums,length);//20,20
	}

	if(lines.size()<1)
	{
		return false;
	}

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

		//没有滤波
//		line(image_mask, pt1, pt2,cv::Scalar(255, 0, 0),1,CV_AA);

		//1.先做一个角度预滤波
		double angle = atan2(double(diff_y),diff_x) * 180 /CV_PI;
		if (abs(angle) <= v1)
			continue;
		if (abs(angle) >= v2)
			continue;
//		printf("angle = %f\n",angle);
	
		//第一次滤波
//		line(image_mask2, pt1, pt2,cv::Scalar(255, 0, 0),1,CV_AA);

		//２．根据线的位置进行滤波区分左右车道
		//左边车道线
		if((pt1.x+pt2.x)<image_mask3.cols)
		{
			//对于曲线只有灭点附近的斜率比较接近临界值
			if((pt1.x+pt2.x) >= 2*image_mask3.cols/4 && (pt1.x+pt2.x) < 2*image_mask3.cols/2 
			&& (pt1.y+pt2.y) >= 2*(440) && (pt1.y+pt2.y) < 2*(540)
				) 
			{
				//利用斜率微调
				if (angle >= -v1)//65
					continue;
				if (angle <= -v2)//65
					continue;
			}
			else
			{
				//利用斜率微调
				if (angle >= -v1)//65
					continue;
				if (angle <= -v2)//65
					continue;
			}

			//除掉位置相差太远的			
			if((pt1.x+pt2.x) >= 0 && (pt1.x+pt2.x) < 2*image_mask3.cols/4  && (pt1.y+pt2.y) >= 2*(440) && (pt1.y+pt2.y) < 2*(540-20))
				continue;
			if((pt1.x+pt2.x) >= 2*image_mask3.cols/3 && (pt1.x+pt2.x) < 2*image_mask3.cols/2 && (pt1.y+pt2.y) >= 2*(540+20) && (pt1.y+pt2.y) < 2*(640))
				continue;
			
			//左边车道线一个顶点太右边
			if(pt1.x >= image_mask3.cols/2+50)
				continue;
			if(pt2.x >= image_mask3.cols/2+50)
				continue;
			node n;
			n.x = angle; //斜率
			n.y = i; //索引
			left_angle.push_back(n);//存储
		}
		else
		{
			//除掉位置相差太远的
			//对于曲线只有灭点附近的斜率比较接近临界值
			if((pt1.x+pt2.x) >= 2*image_mask3.cols/2 && (pt1.x+pt2.x) < 2*image_mask3.cols/4*3 
			&& (pt1.y+pt2.y) >= 2*(440) && (pt1.y+pt2.y) < 2*(540)
				) 
			{
				//利用斜率微调
				if (angle <v1)//65
					continue;
				if (angle >= v2)//70
					continue;
			//除掉位置相差太远的
			}
			else
			{
				//利用斜率微调
				if (angle <= v1)//65
					continue;
				if (angle >= v2-10)//70
					continue;
			}
			//除掉位置相差太远的			
			if((pt1.x+pt2.x) >= 2*image_mask3.cols/4*3 && (pt1.x+pt2.x) < 2*image_mask3.cols  && (pt1.y+pt2.y) >= 2*(440) && (pt1.y+pt2.y) < 2*(540-20))
				continue;
			if((pt1.x+pt2.x) >= 2*image_mask3.cols/2 && (pt1.x+pt2.x) < 2*image_mask3.cols/3*2 && (pt1.y+pt2.y) >= 2*(540+20) && (pt1.y+pt2.y) < 2*(640))
				continue;

			//右边车道线一个顶点太左边
			if(pt1.x <= image_mask3.cols/2-50)
				continue;
			if(pt2.x <= image_mask3.cols/2-50)
				continue;

			//右边车道线顶点太右边
			if(min(pt1.x,pt2.x) >= 500-60)
				continue;


			node n;
			n.x = angle; //斜率
			n.y = i;//索引
			right_angle.push_back(n);//存储

		}

//		Line_store.push_back(line_temp);
//		line(image_mask3, pt1, pt2,cv::Scalar(255),5,CV_AA);
		
//		printf("%d,%d,%d,%d\n",pt1.x,pt1.y,pt2.x,pt2.y);
//		printf("%d,%d\n",diff_x,diff_y);
	}
	//最好做一个斜率的排序，去掉异常值
	if(left_angle.size()>=3)
	{
		int bias = 1;
		if(left_angle.size()>=7)
		{
			bias = 2;
			if(left_angle.size()>=9)
			{
				bias = 3;
				if(left_angle.size()>=11)
				{
					bias = 4;
					if(left_angle.size()>=13)
					{
						bias = left_angle.size()/2-2;
					}
				}
			}
		}
		sort(left_angle.begin(), left_angle.end(),comp);
		vector<node>::iterator pos;
		for(pos = left_angle.begin()+bias+1; pos != left_angle.end()-bias; ++pos)
		{
			Line_store.push_back(lines[pos->y]);
		}
	}
	if(right_angle.size()>=3)
	{
		int bias = 1;
		if(right_angle.size()>=7)
		{
			bias = 2;
			if(right_angle.size()>=9)
			{
				bias = 3;
				if(right_angle.size()>=11)
				{
					bias = 4;
					if(right_angle.size()>=13)
					{
						bias = right_angle.size()/2-2;
					}
				}
			}
		}
		sort(right_angle.begin(), right_angle.end(),comp);
		vector<node>::iterator pos;
		for(pos = right_angle.begin()+bias; pos != right_angle.end()-bias; ++pos)
		{
			Line_store.push_back(lines[pos->y]);
		}
		printf("right line size :%d\n",Line_store.size());
	}
	if(Line_store.size())
	{
		Point pt1, pt2;
		for(int i=0;i<Line_store.size();i++)
		{
			pt1.x = Line_store[i][0];
			pt1.y = Line_store[i][1]+440;
			pt2.x = Line_store[i][2];
			pt2.y = Line_store[i][3]+440;
			line(image_mask3, pt1, pt2,cv::Scalar(255),5,CV_AA);
		}
		return true;
	}
	return false;
}

void coutour_filter(cv::Mat imgBlur,cv::Mat &contour_f)
{
	
		clock_t time_s2 = clock();
		//轮廓滤波
		cv::threshold(imgBlur, contour_f, 100, 255, cv::THRESH_BINARY);
		std::vector< std::vector< cv::Point> > contours;
		cv::findContours(
			contour_f,
			contours,
			cv::noArray(),
			cv::RETR_LIST,
			cv::CHAIN_APPROX_SIMPLE
		); //CV_RETR_EXTERNAL,RETR_LIST
		contour_f = cv::Scalar::all(0);
		cv::drawContours(contour_f, contours, -1, cv::Scalar::all(255));

 		cv::Mat contour_f2(contour_f.size(), CV_8U, Scalar(0));
	 /*
		//计算轮廓的面积  
		cout << "【筛选前总共轮廓个数为】：" << (int)contours.size() << endl;
		for (int i = 0; i < (int)contours.size(); i++)
		{
			double g_dConArea = contourArea(contours[i], true);
			cout << "【用轮廓面积计算函数计算出来的第" << i << "个轮廓的面积为：】" << g_dConArea << endl;
		}
	*/
		//筛选剔除掉面积小于1000的轮廓
		vector <vector<Point>>::iterator itc_rect = contours.begin();
		for (; itc_rect != contours.end();)
		{
			double g_dConArea = contourArea(*itc_rect);
			if (g_dConArea < 10 || g_dConArea > 500 || (*itc_rect).size()> 50)
			{
				itc_rect = contours.erase(itc_rect);
			}
			else
			{
				++itc_rect;
			}
		}
		/*
		cout << "【筛选后总共轮廓个数为：" << (int)contours.size() << endl;
		for (int i = 0; i < (int)contours.size(); i++)
		{
		double g_dConArea = contourArea(contours[i], true);
		cout << "【用轮廓面积计算函数计算出来的第" << i << "个轮廓的面积为：】" << g_dConArea << endl;
		}
		*/
		contour_f2 = cv::Scalar::all(0);
		drawContours(contour_f2, contours, -1, cv::Scalar::all(255),CV_FILLED);

		cout << "contour time is :" << 1000*(clock()-time_s2)/(double)CLOCKS_PER_SEC << "ms" << endl;

}

//最小二乘拟合器
/*
check "http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/"
*/
bool polynomialfit(int obs, int degree, vector<double> dx, vector<double> dy, double *store) {
	int i, j, k;
	vector<double> X;                           //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	X.assign(2*degree+1, 0.);
	for (i = 0; i<2 * degree + 1; i++)
	{
		X[i] = 0;
		for (j = 0; j<obs; j++)
			X[i] = X[i] + pow(dx[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	}

	vector<double> row;
	row.assign(degree + 2, 0.);
	vector< vector<double> > B;                   //B is the Normal matrix(augmented) that will store the equations
	B.assign(degree + 1, row);
	for (i = 0; i <= degree; i++)
		for (j = 0; j <= degree; j++)
			B[i][j] = X[i + j];                 //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	vector<double> Y;                           //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	Y.assign(degree + 1, 0.);
	for (i = 0; i<degree + 1; i++){
		Y[i] = 0;
		for (j = 0; j<obs; j++)
			Y[i] = Y[i] + pow(dx[j], i)*dy[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	}
	for (i = 0; i <= degree; i++)
		B[i][degree + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	degree = degree + 1;                        //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	for (i = 0; i<degree; i++)                  //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
		for (k = i + 1; k<degree; k++)
			if (B[i][i]<B[k][i])
				for (j = 0; j <= degree; j++)
				{
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}

	for (i = 0; i<degree - 1; i++)              //loop to perform the gauss elimination
		for (k = i + 1; k<degree; k++)
		{
			double t = B[k][i] / B[i][i];
			for (j = 0; j <= degree; j++)
				B[k][j] = B[k][j] - t*B[i][j];   //make the elements below the pivot elements equal to zero or elimnate the variables
		}
	for (i = degree - 1; i >= 0; i--)            //back-substitution
	{                                            //x is an array whose values correspond to the values of x,y,z..
		store[i] = B[i][degree];                 //make the variable to be calculated equal to the rhs of the last equation
		for (j = 0; j<degree; j++)
			if (j != i)                          //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
				store[i] =store[i] - B[i][j] * store[j];
		store[i] = store[i] / B[i][i];           //now finally divide the rhs by the coefficient of the variable to be calculated
	}   

	return true; /* we do not "analyse" the result (cov matrix mainly)
				 to know if the fit is "good" */
}

int main(int argc, char** argv) {

	//视频
 	VideoCapture cap;
	//	string string1 = "../test_video/30_evt.avi";
	//	cap.open(string1);
	cap.open(argv[1]);
	if(!cap.isOpened())
	    return 0;

	cv::Mat frame;
	cv::Mat img;
	cv::Mat imgGRAY;
	cv::Mat imgBlur;
	cv::Mat readyNotBlur;
	cv::Mat readyDelete;
	cv::Mat imgBlurAll; //如果

	cv::Mat contour_f; //轮廓特征看看
	cv::Mat roi;
	cv::Mat histeq; //如果
	//定义一个全局变量用来保存上一次检测到的车道线的结果
	vector<double> _left_lane_x, _left_lane_y; 
	vector<double> _right_lane_x, _right_lane_y;

	double al =0;
	double bl= 0;
	double cl= 0;
	double ar =0;
	double br= 0;
	double cr= 0;

	//定义一个全局的图像变量用来保存上一次的结果
	cv::Mat last_out;
	for(;;)
	{
//		clock_t time_stt = clock();
		cap >> frame;
		cv::cvtColor(frame,img,CV_BGR2GRAY);
/*
		medianBlur(img,histeq,3);
		equalizeHist(histeq,histeq);
*/
	//	frame.copyTo(img);
	//	imgGRAY = img(Rect(0, 440, img.cols, img.rows-440));

		cv::Mat image_mask = Mat(img.size(), CV_8UC3, Scalar(0));
		image_mask.setTo(cv::Scalar(0, 0, 0)); 

		cv::Mat image_mask2 = Mat(img.size(), CV_8UC3, Scalar(0));
		image_mask2.setTo(cv::Scalar(0, 0, 0)); 

		cv::Mat image_mask3 = Mat(img.size(), CV_8UC1, Scalar(0));
		image_mask3.setTo(cv::Scalar(0)); 

		cv::Mat image_mask4 = Mat(img.size(), CV_8UC3, Scalar(0));
		image_mask4.setTo(cv::Scalar(0, 0, 0)); 

		cv::Mat outputImg;

		//掩摸
		//为了保留细节，不能全局进行高斯滤波，只能部分滤波
		//选择感兴趣区域
		//为了尽可能的减小误差和计算量，还是要进行ROI选择
		int shift1 = 120;
		int offset = 0;
		int shift2 = 50;
		Point PointArray[6];
		Mat mask_roi = Mat::zeros(img.rows,img.cols,CV_8UC1);    
		PointArray[0] = Point(img.cols/2-shift1,440);
		PointArray[1] = Point(img.cols/2+shift1-offset,440);//
		PointArray[2] = Point(img.cols,img.rows-shift2);
		PointArray[3] = Point(img.cols,img.rows);
		PointArray[4] = Point(0,img.rows);
		PointArray[5] = Point(0,img.rows-shift2);
		fillConvexPoly(mask_roi,PointArray,6,Scalar(255));
		img.copyTo(roi,mask_roi);//带掩模mask的复制
		
		int shift3 = 100;
		int offset2 = 50;
		Point PointArray2[4];
		Mat mask_roi2 = Mat::zeros(img.rows,img.cols,CV_8UC1);    
		PointArray2[0] = Point(img.cols/2-shift3,540);
		PointArray2[1] = Point(img.cols/2+shift3,540);//
		PointArray2[2] = Point(img.cols/2+shift3+offset2,img.rows);
		PointArray2[3] = Point(img.cols/2-shift3-offset2,img.rows);
		fillConvexPoly(mask_roi2,PointArray2,4,Scalar(255));
		image_mask3.copyTo(roi,mask_roi2);//带掩模mask的复制

		imgGRAY = roi(Rect(0, 440, roi.cols, roi.rows-440));
/*
		//除去明显不是车道线的区域,中心线下方的区域
		//选择感兴趣区域
		int a = 100;
		Rect r2(imgGRAY.cols/2-a, imgGRAY.rows-a, 2*a, a);
		readyDelete = image_mask3(r2);
		readyDelete.copyTo(imgGRAY(r2));
*/
		//由于计算量大，进行全局滤波
		medianBlur(imgGRAY,imgBlur,3);

		coutour_filter(imgBlur,contour_f);

		//判断一下是否强曝光
		float nonZerosnum = countNonZero(imgBlur);
		float totalNum = imgBlur.size().width * imgBlur.size().height;
		float rate = nonZerosnum/totalNum;
		if(rate > 0.25 || rate < 0.01)
		{
			printf("rate is %f\n",rate);
			//continue;
			cv::Mat last_out2;
			addWeighted(frame, 1, last_out, 2, -1, last_out2);
			cv::imshow ( "out", last_out2 );
		}
		else
		{	
			//hough
			clock_t time_s1 = clock();
			vector<Vec4i> Line_store; //contour_f
			bool h = Houghlane(imgBlur,image_mask3,Line_store,25,100,25);//imgGRAY,50,150imgBlur
			cout << "hough time is :" << 1000*(clock()-time_s1)/(double)CLOCKS_PER_SEC << "ms" << endl;

			img.copyTo(outputImg,image_mask3);//img 
			//	printf("Line_store size : %d\n",Line_store.size());

			cv::Mat result;
	
		outputImg.copyTo(result); //霍夫变换
//			contour_f2.copyTo(result); 
			//是否中值滤波
		//	readyBlur.copyTo(result(r1));	

			//直线拟合
clock_t time_s3 = clock();
			cv::Mat _process_img;
			result.copyTo(_process_img);
			cvtColor(_process_img, _process_img, CV_GRAY2RGB);

			int _nwindows = 18;
			int _window_height = int((img.rows-440) / _nwindows);
			int _minpix = 3;
			//定义一个可以滑窗的标志，即第一次检测到了车道线部分
			int left_sliding = 0;
			int right_sliding = 0;

			//车道线坐标
	//		vector<double> _left_lane_x, _left_lane_y; 
	//		vector<double> _right_lane_x, _right_lane_y;
			_left_lane_x.clear();
			_left_lane_y.clear();
			_right_lane_x.clear();
			_right_lane_y.clear();

			//手工确定基点
			int cur_x_left = img.cols/8;
			int cur_x_right = img.cols/8*7;

			for (int i = 0; i < _nwindows; i++) 
			{
				// Identify window boundaries in x and y(and right and left)
				int win_y_low = img.rows - i*_window_height;
				int win_y_high = img.rows - (i + 1)*_window_height;

				//第一个窗口需要更大的搜索空间
				int win_x_left_low = 0;
				int win_x_left_high = 383;
				int win_x_right_low = 384;
				int win_x_right_high = 768;
		
				int margin_offset = 50; //25
				if(i>0&&left_sliding)
				{
					win_x_left_low = cur_x_left - margin_offset;
					win_x_left_high = cur_x_left + margin_offset;
				}
				if(i>0&&right_sliding)
				{
					win_x_right_low = cur_x_right - margin_offset;
					win_x_right_high = cur_x_right + margin_offset;
				}

				int left_non_zero = 0;
				int right_non_zero = 0;
				vector<Point> left_inds, right_inds;
				// Find and collect non-zero points
				for (int k = win_y_high; k < win_y_low; k++) 
				{
					for (int a = win_x_left_low, b = win_x_right_low; a < win_x_left_high, b < win_x_right_high; a++, b++) 
					{
						if (result.at<uchar>(Point(a, k)) != 0) 
						{
							left_inds.push_back(Point(a, k));
							left_non_zero++;
						}
						if (result.at<uchar>(Point(b, k)) != 0) {
							right_inds.push_back(Point(b, k));
							right_non_zero++;
						}
					}
				}

				Point zero(0, 0);
				Point sum;
				if (left_non_zero > _minpix) {
					//取均值
					left_sliding = 1;
					sum = accumulate(left_inds.begin(), left_inds.end(), zero);
					cur_x_left = sum.x / left_inds.size();
					if(win_x_left_low < cur_x_left && cur_x_left < win_x_left_high)
					{
						_left_lane_x.push_back(cur_x_left);
						_left_lane_y.push_back((win_y_high+win_y_low)/2);
					}
					else
					{
						cur_x_left = (win_x_left_low+win_x_left_high)/2;
						_left_lane_x.push_back(cur_x_left);
						_left_lane_y.push_back((win_y_high+win_y_low)/2);
					}
					cv::circle( _process_img,Point(cur_x_left,(win_y_high+win_y_low)/2),8,Scalar( 0, 255, 255 ),-1,2 );
					cur_x_left = cur_x_left + img.cols/128*3;
				}
				else
				{
					cur_x_left = cur_x_left + img.cols/128*3;
				}
				if (right_non_zero > _minpix) {
					//取均值
					right_sliding = 1;
					sum = accumulate(right_inds.begin(), right_inds.end(), zero);
					cur_x_right = sum.x / right_inds.size();
					if(win_x_right_low < cur_x_right && cur_x_right < win_x_right_high)
					{
						_right_lane_x.push_back(cur_x_right);
						_right_lane_y.push_back((win_y_high+win_y_low)/2);
					}
					else
					{
						cur_x_right = (win_x_right_low+win_x_right_high)/2;
						_right_lane_x.push_back(cur_x_right);
						_right_lane_y.push_back((win_y_high+win_y_low)/2);
					}
					cv::circle( _process_img,Point(cur_x_right,(win_y_high+win_y_low)/2),8,Scalar( 0, 255, 255 ),-1,2 ); 
					cur_x_right = cur_x_right - img.cols/128*3; 

				}
				else
				{
					cur_x_right = cur_x_right - img.cols/128*3;
				}
				// Draw the windows on the visualization image
				// comment out to speed up
		//		printf("left_x_ave = %d\n",cur_x_left);
		//		printf("right_x_ave = %d\n",cur_x_right);
				cv::rectangle(_process_img, Point(win_x_left_low, win_y_low), Point(win_x_left_high, win_y_high), 255, 2);
				cv::rectangle(_process_img, Point(win_x_right_low, win_y_low), Point(win_x_right_high, win_y_high), 255, 2);	
			}

			//拟合就行啦
			int _degree=1; //拟合阶数
			double *store_left; //用来存储拟合的系数
			double *store_right;
			store_left = new double[_degree+1];
			store_right = new double[_degree+1];
			//1.拟合直线	
			// 为了避免x方向上要确定边界，因此将y当做自变量，x当做因变量进行拟合
			polynomialfit(_left_lane_x.size(), _degree, _left_lane_y, _left_lane_x, store_left);
			// 为了避免x方向上要确定边界，因此将y当做自变量，x当做因变量进行拟合
			polynomialfit(_right_lane_x.size(), _degree, _right_lane_y, _right_lane_x, store_right);

			//判断是否拟合出了直线呀
			if(!(store_left[0]>0) && !(store_left[1]<0))
			{
				if(al)
				{
					store_left[0]=al;
					store_left[1]=bl;
					store_left[2]=cl;
				}
			}
			else
			{
				al =store_left[0];
				bl= store_left[1];
				cl= store_left[2];
			}
			if(!(store_right[0]<0) && !(store_right[1]>0))
			{
				if(ar)
				{
					store_right[0]=ar;
					store_right[1]=br;
					store_right[2]=cr;
				}
			}
			else
			{
				ar=store_right[0];
				br=store_right[1];
				cr=store_right[2];
			}
			cout << "fitting time is :" << 1000*(clock()-time_s3)/(double)CLOCKS_PER_SEC << "ms" << endl;
			//2.画出整个图像范围里的曲线
			std::vector<cv::Point> points_fitted_left; 
			std::vector<cv::Point> points_fitted_right; 
			for (int y = 440; y < img.rows; y++) 
			{ 
				double x_left = store_left[0]+store_left[1]*y+store_left[2]*y*y;
				points_fitted_left.push_back(cv::Point(x_left, y));
				double x_right = store_right[0]+store_right[1]*y+store_right[2]*y*y;
				points_fitted_right.push_back(cv::Point(x_right, y));
			}
			
			cv::polylines(image_mask4, points_fitted_left, false, cv::Scalar(0, 255, 255), 4, 2, 0); 
			cv::polylines(image_mask4, points_fitted_right, false, cv::Scalar(0, 255, 255), 4, 2, 0);
			//融合
			Mat out;
			addWeighted(frame, 1, image_mask4, 2, -1, out);
			if(_left_lane_x.size()>=5 && _right_lane_x.size()>=5)
				image_mask4.copyTo(last_out);
			delete[] store_left;
			delete[] store_right;
			
			cv::imshow ( "img", img );
			cv::imshow ( "imgGRAY", imgGRAY );
			cv::imshow ( "imgBlur", imgBlur );
			cv::imshow ( "image_mask3", image_mask3 );
//			cv::imshow ( "outputImg", outputImg );
			cv::imshow ( "result", result );
			cv::imshow ( "_process_img", _process_img );
//			cv::imshow ( "image_mask4", image_mask4 );
			cv::imshow ( "out", out );
			cv::imshow ( "contourf", contour_f );
//			cv::imshow ( "contourf2", contour_f2 );
			cv::imshow ( "roi", roi );
//			cv::imshow ( "last_out", last_out );
//			cv::imshow ( "histeq", histeq );
//			cv::imshow ( "mask_roi2", mask_roi2 );

		}
		
		cv::waitKey (10);
	}

	return 0;
}




