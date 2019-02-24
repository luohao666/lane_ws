#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <time.h>
#include <math.h>

using namespace std ;
using namespace cv ;

class ransac_fit
{
    public:
    double generate_random();
    bool get_sample(int set_size,vector<int> &sub_set,int n);
    void ransac(vector<Point2d> set,cv::Mat &best_model,int n_max,int n_sample,double thre,vector<bool> &inliner_flag);
    bool ransac2(vector<Point2d> set,cv::Mat &best_model,int n_max,int n_sample,double thre,vector<bool> &inliner_flag);

    void generation_data_line(Mat &img,vector<Point2d> &ptSet);
    void generation_data_curve(Mat &img,vector<Point2d> &ptSet);


    private:
    double uniform_random();
};
