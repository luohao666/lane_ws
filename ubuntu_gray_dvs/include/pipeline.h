#include <opencv2/opencv.hpp>
#include <ctime> //time
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "vp_detection.h"
#include "preprocessing.h"
#include "img_perspective.h"
#include "lane_detection.h"
#include "ransac_fit.h"

using namespace std ;
using namespace cv ;

class Pipeline
{
    public:
        Pipeline();
        void image_sequence(bool vp_flag,bool ipm_flag);

    private:
        //存储好的拟合系数
        double al =0;
        double bl= 0;
        double cl= 0;
        double ar =0;
        double br= 0;
        double cr= 0;
};