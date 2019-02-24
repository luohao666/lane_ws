#include <opencv2/opencv.hpp>
#include <ctime> //time
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std ;
using namespace cv ;

class Preprecessing
{
    public:
        Preprecessing();
        Mat gray_processing(Mat roi);

    private:
        Mat threshold_process(Mat src, double thresh_min, double thresh_max, bool scale);
};