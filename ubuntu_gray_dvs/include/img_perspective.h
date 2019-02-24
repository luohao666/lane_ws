#include <opencv2/opencv.hpp>
#include <ctime> //time
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std ;
using namespace cv ;

class Perspective
{
    public:
        Perspective();
        Mat perspective_img(Mat img);
        Mat perspective_img_roi(Mat img,int x1,int x2);
        Mat inverse_perspective_img_roi(Mat img,int x1,int x2);

    private:

};