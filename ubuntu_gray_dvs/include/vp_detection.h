#include <opencv2/opencv.hpp>
#include <ctime> //time
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std ;
using namespace cv ;

class vp_detection
{
    public:
        vp_detection(); 
        bool vanish_point_detect(Mat &dst,cv::Mat &image_masks,Point &vp,double &weight);

    private:
        Point2f getCrossPoint(Vec4i LineA, Vec4i LineB,double &rate); 
        bool Houghlane(cv::Mat canny,cv::Mat &image_mask,vector<Vec4i> &Line_filter, bool left_lane,int nums,int length);
        void line_longer(Point s, Point e,Point &ss,Point &ee);

};