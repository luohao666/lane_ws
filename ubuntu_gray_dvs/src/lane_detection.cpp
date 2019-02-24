#include "lane_detection.h"

lane_detection::lane_detection()
{

}

Mat lane_detection::sliding_windows(Mat result,bool impOrNot)
{
    cv::Mat _process_img;
    result.copyTo(_process_img);
    cvtColor(_process_img, _process_img, CV_GRAY2RGB);

    int _nwindows = 18;
    int _window_height = 0;
    if(impOrNot)
    {
        _window_height = int((result.rows) / _nwindows);
    }
    else
    {
        _window_height = int((result.rows-440) / _nwindows);
    }
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
    _left_pt_set.clear();
    _right_pt_set.clear();

    //手工确定基点
    int cur_x_left = result.cols/8;
    int cur_x_right = result.cols/8*7;

    for (int i = 0; i < _nwindows; i++) 
    {
        // Identify window boundaries in x and y(and right and left)
        int win_y_low = result.rows - i*_window_height;
        int win_y_high = result.rows - (i + 1)*_window_height;

        //第一个窗口需要更大的搜索空间
        int win_x_left_low = 0;
        int win_x_left_high = 383;
        int win_x_right_low = 384;
        int win_x_right_high = 768;

        int margin_offset = 50; //25,50
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
                _left_pt_set.push_back(Point2d((win_y_high+win_y_low)/2,cur_x_left));
            }
            else
            {
                cur_x_left = (win_x_left_low+win_x_left_high)/2;
                _left_lane_x.push_back(cur_x_left);
                _left_lane_y.push_back((win_y_high+win_y_low)/2);
                _left_pt_set.push_back(Point2d((win_y_high+win_y_low)/2,cur_x_left));
            }
            cv::circle( _process_img,Point(cur_x_left,(win_y_high+win_y_low)/2),8,Scalar( 0, 255, 255 ),-1,2 );
            cur_x_left = cur_x_left + result.cols/128*2;
        }
        else
        {
            cur_x_left = cur_x_left + result.cols/128*2;
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
                _right_pt_set.push_back(Point2d((win_y_high+win_y_low)/2,cur_x_right));
            }
            else
            {
                cur_x_right = (win_x_right_low+win_x_right_high)/2;
                _right_lane_x.push_back(cur_x_right);
                _right_lane_y.push_back((win_y_high+win_y_low)/2);
                _right_pt_set.push_back(Point2d((win_y_high+win_y_low)/2,cur_x_right));

            }
            cv::circle( _process_img,Point(cur_x_right,(win_y_high+win_y_low)/2),8,Scalar( 0, 255, 255 ),-1,2 ); 
            cur_x_right = cur_x_right - result.cols/128*2; 

        }
        else
        {
            cur_x_right = cur_x_right - result.cols/128*2;
        }
        // Draw the windows on the visualization image
        // comment out to speed up
//		printf("left_x_ave = %d\n",cur_x_left);
//		printf("right_x_ave = %d\n",cur_x_right);
//       cv::circle( _process_img,Point(cur_x_left,(win_y_high+win_y_low)/2),8,Scalar( 0, 255, 255 ),-1,2 );
//       cv::circle( _process_img,Point(cur_x_right,(win_y_high+win_y_low)/2),8,Scalar( 0, 255, 255 ),-1,2 ); 
        cv::rectangle(_process_img, Point(win_x_left_low, win_y_low), Point(win_x_left_high, win_y_high), 255, 2);
        cv::rectangle(_process_img, Point(win_x_right_low, win_y_low), Point(win_x_right_high, win_y_high), 255, 2);	
    }
    return _process_img;
}