#include <opencv2/opencv.hpp>
#include <ctime> //time
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std ;
using namespace cv ;

class lane_detection
{
    public:
        //要保证对象在全局
        //定义一个全局变量用来保存上一次检测到的车道线的结果
        vector<double> _left_lane_x, _left_lane_y; 
        vector<double> _right_lane_x, _right_lane_y;
        //生成ransac的总集
        vector<Point2d> _left_pt_set;
        vector<Point2d> _right_pt_set;
        
        lane_detection();
        Mat sliding_windows(Mat result,bool impOrNot);
    private:

};