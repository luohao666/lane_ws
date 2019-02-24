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
	src[2] = cv::Point2f(740, 200);//730,200
	src[3] = cv::Point2f(20, 200);//30,200
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

int main(int argc,char** argv)
{
	cout<< "lane gray"<<endl;
    clock_t time_s1 = clock(); // 计时   
    Mat img;
    Mat blur;
    Mat roi;
    Mat sobel_x;
    Mat sobel_y;
    Mat dst;
    Mat ipm;
    img = imread(argv[1],-1);
    //ROI select
    roi = img(Rect(0,440,img.cols,img.rows-440));
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
    clock_t time_s2 = clock(); // 计时   
    ipm = perspective_img(dst);
    cout <<"perspective time is " << 1000* (clock() - time_s2)/(double)CLOCKS_PER_SEC << "ms"<< endl;

	

    imshow("src",img);
    imshow("roi",roi);
    imshow("blur",blur);
    imshow("x",sobel_x);
    imshow("y",sobel_y);
    imshow("xx",sobel_xx);
    imshow("yy",sobel_yy);
    imshow("and",dst);
    imshow("ipm",ipm);


//    imshow("Keypoints", img_keypoints);

    waitKey(0);    
    return 0;
}