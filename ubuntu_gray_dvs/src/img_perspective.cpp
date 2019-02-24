#include "img_perspective.h"

Perspective::Perspective()
{

}

Mat Perspective::perspective_img(Mat img)
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

Mat Perspective::perspective_img_roi(Mat img,int x1,int x2)
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

Mat Perspective::inverse_perspective_img_roi(Mat img,int x1,int x2)
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

    M = getPerspectiveTransform(dst,src);
	warpPerspective(img, res, M, Size(768,640));
    
    line(img,src[0],src[1] , Scalar(255), 2);
	line(img, src[1],src[2],Scalar(255), 2);
	line(img, src[2],src[3],Scalar(255), 2);
	line(img, src[3],src[0],Scalar(255), 2);

    return res;
}