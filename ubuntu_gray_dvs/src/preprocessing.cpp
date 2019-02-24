#include "preprocessing.h"

Preprecessing::Preprecessing() 
{

}

Mat Preprecessing::threshold_process(Mat src, double thresh_min, double thresh_max, bool scale) {
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

Mat Preprecessing::gray_processing(Mat roi)
{
    Mat dst;
    Mat blur;
    Mat roi_evt;
    Mat sobel_x;
    Mat sobel_y;
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

    imshow("blur",blur);
    imshow("x",sobel_x);
    imshow("y",sobel_y);
    imshow("xx",sobel_xx);
    imshow("yy",sobel_yy);

    return dst;
}