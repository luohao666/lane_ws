#include "pipeline.h"


Pipeline::Pipeline()
{

}

void Pipeline::image_sequence(bool vp_flag,bool ipm_flag)
{
    cout<< "lane gray"<<endl;
    Mat img;
    Mat evt;

	//preprocessing
	Preprecessing preo;
    Mat roi;
    Mat roi_evt;
    Mat dst; 

    //vp detection
    vp_detection vpo;
    Mat mask;
    Mat roi_canny;
    int count = 0;
    vector<Point> vp_last;
    vector<double> w_last;
    int p1x = 0;
    int p2x = 0;
    
    // ipm
    Perspective ipmo;
    Mat ipm;
    
    // lane_detection
    lane_detection laneo;

    int i=0;
    for(;;)
    {
    clock_t time_s = clock(); // 计时   
    i++;
    char ss[50];
    char ss2[50];

	sprintf(ss,"../dvs_dataset/50ms/FullPic/Pic%d.bmp",i); 
    sprintf(ss2,"../dvs_dataset/50ms/EventPic/Pic%d.bmp",i);    
   
    img = imread(ss,-1);
    evt = imread(ss2,-1);
    
    // display
    Mat image_mask = Mat(img.size(), CV_8UC3, Scalar(0));
    image_mask.setTo(cv::Scalar(0, 0, 0)); 
    Mat image_mask2 = Mat(img.size(), CV_8UC3, Scalar(0));
    image_mask2.setTo(cv::Scalar(0, 0, 0)); 

    //ROI select
    Rect r1(0,440,img.cols,img.rows-440);
    roi = img(r1);
    roi_evt = evt(r1);

	clock_t time_s1 = clock(); // 计时   
   	dst=preo.gray_processing(roi);
    Mat lane_im = Mat(img.size(), CV_8UC1, Scalar(0));
    Mat lane = Mat(img.size(), CV_8UC1, Scalar(0));
    // 要转化成img的大小
    Mat temp = lane_im(Rect(0,440,768,200));
    dst.copyTo(temp);//img 
    // delete ROI
    int shift3 = 100;
    int offset2 = 50;
    Point PointArray2[4];
    Mat mask_roi2 = Mat::zeros(img.rows,img.cols,CV_8UC1);    
    PointArray2[0] = Point(img.cols/2-shift3,540);
    PointArray2[1] = Point(img.cols/2+shift3,540);//
    PointArray2[2] = Point(img.cols/2+shift3+offset2,img.rows);
    PointArray2[3] = Point(img.cols/2-shift3-offset2,img.rows);
    fillConvexPoly(mask_roi2,PointArray2,4,Scalar(255));
    lane.copyTo(lane_im,mask_roi2);//带掩模mask的复制
    cout <<"preprocessing time is " << 1000* (clock() - time_s1)/(double)CLOCKS_PER_SEC << "ms"<< endl;
	
    clock_t time_s4 = clock(); // 计时 
    if(vp_flag)
    {
        //vanish point wasting time too much!
        Point vp;
        double w=1;
        int r = 50;
        if(count<3)
        {
            mask = Mat(img.size(), CV_8UC3, Scalar(0));
            mask.setTo(cv::Scalar(0, 0, 0));  
            Canny(roi, roi_canny, 50,100, 3);//180 120
            bool vp_flag = vpo.vanish_point_detect(roi_canny,mask,vp,w);

            if(vp_flag)
            {
                vp_last.push_back(vp);
                w_last.push_back(w);
                count++;
            }
            //单独调试vanish_point_detect用
    //		imshow("roi_canny",roi_canny);
    // 		imshow("mask",mask);
    //		waitKey(10); 
            continue;
        }
    //	printf("vp_last size is:%d\n",vp_last.size());
    //	printf("count is:%d\n",count);

        if(count == 3)
        {
            if(abs(vp_last[0].x -vp_last[1].x) < r && abs(vp_last[0].x -vp_last[2].x) < r && abs(vp_last[1].x -vp_last[2].x) < r
            && abs(vp_last[0].y -vp_last[1].y) < r &&abs(vp_last[0].y -vp_last[2].y) < r && abs(vp_last[1].y -vp_last[2].y) < r)
            {
                vp = vp_last[0];
                w = w_last[0];
                count++;
                //利用vp画出感兴趣区域
                p1x = vp.x-200*w;
                p2x = vp.x+200*(1-w);

            }
            else
            {
                count = 2;
                vector<Point>::iterator k = vp_last.begin();
                vp_last.erase(k);//删除第一个元素
            }
        }
        imshow("mask",mask);
    }
    
    //ipm wasting time too much!
	if(count > 3)
	{
        if(ipm_flag)
        {
            clock_t time_s2 = clock(); // 计时   
            ipm = ipmo.perspective_img_roi(dst,p1x,p2x);
            cout <<"perspective time is " << 1000* (clock() - time_s2)/(double)CLOCKS_PER_SEC << "ms"<< endl;
            imshow("ipm",ipm);
        }
        else
        {
//            count = 2;
            if(vp_flag)
            {
                //ROI
                int shift2 = 50;
                Point PointArray[6];
                Mat mask_roi = Mat::zeros(img.rows,img.cols,CV_8UC1);    
                PointArray[0] = Point(p1x,440);
                PointArray[1] = Point(p2x,440);//
                PointArray[2] = Point(img.cols,img.rows-shift2);
                PointArray[3] = Point(img.cols,img.rows);
                PointArray[4] = Point(0,img.rows);
                PointArray[5] = Point(0,img.rows-shift2);
                fillConvexPoly(mask_roi,PointArray,6,Scalar(255));
                lane_im.copyTo(lane,mask_roi);//带掩模mask的复制
            }
            else
            {

            }

        }
	}
    cout <<"vp time is " << 1000* (clock() - time_s4)/(double)CLOCKS_PER_SEC << "ms"<< endl;

    // sliding windows
    clock_t time_s2 = clock();
    if(ipm_flag)
    {
        lane = laneo.sliding_windows(ipm,ipm_flag);
    }
    else
    {
        lane = laneo.sliding_windows(lane,ipm_flag);
    }	
    cout << "sliding windows time is :" << 1000*(clock()-time_s2)/(double)CLOCKS_PER_SEC << "ms" << endl;

//    printf("pt size is :%d",laneo._left_pt_set.size());

    // ransac line fitting
    //拟合就行啦
    /*不做投影变换拟合二次曲线效果并不好*/

    //1.产生数据_left_pt_set,_right_pt_set
    //2.ransac
    srand((unsigned int)time(NULL)); //设置随机数种子
    ransac_fit ransac;
    cv::Mat left_best_model(3,1,CV_64FC1);//曲线模型
    cv::Mat right_best_model(3,1,CV_64FC1);//曲线模型
    int n_max = 50;//最大迭代次数
    int n_sample = 10;//采样数目
    double thre = 10;//内点阈值
    vector<bool> left_inliner_flag;//内点标志
    vector<bool> right_inliner_flag;//内点标志
    clock_t time_s3 = clock();	
    bool left_flag = ransac.ransac2(laneo._left_pt_set,left_best_model,n_max,n_sample,thre,left_inliner_flag);
    bool right_flag =ransac.ransac2(laneo._right_pt_set,right_best_model,n_max,n_sample,thre,right_inliner_flag);
			
    //判断是否拟合出了直线呀
    if((left_best_model.at<double>(2,0)>0)&& (left_best_model.at<double>(1,0)<0))
    {
        al= left_best_model.at<double>(2,0);
        bl= left_best_model.at<double>(1,0);
        cl= left_best_model.at<double>(0,0);
    }
    else
    {
        if(al>0)
        {
            left_best_model.at<double>(2,0)=al;
            left_best_model.at<double>(1,0)=bl;
            left_best_model.at<double>(0,0)=cl;
        }
    }
    if((right_best_model.at<double>(2,0)<0)&& (right_best_model.at<double>(1,0)>0))
    {
        ar=right_best_model.at<double>(2,0);
        br=right_best_model.at<double>(1,0);
        cr=right_best_model.at<double>(0,0);
    }
    else
    {
        if(ar<0)
        {
            right_best_model.at<double>(2,0)=ar;
            right_best_model.at<double>(1,0)=br;
            right_best_model.at<double>(0,0)=cr;
        }
    }
    cout << "ransac time is :" << 1000*(clock()-time_s3)/(double)CLOCKS_PER_SEC << "ms" << endl;

    //2.画出整个图像范围里的曲线
    std::vector<cv::Point> points_fitted_left; 
    std::vector<cv::Point> points_fitted_right; 
    for (double y = 440; y < img.rows; y=y+2) 
    { 
        double x_left = left_best_model.at<double>(2,0)+left_best_model.at<double>(1,0)*y+left_best_model.at<double>(0,0)*y*y;
        points_fitted_left.push_back(cv::Point(x_left, y));
        double x_right = right_best_model.at<double>(2,0)+right_best_model.at<double>(1,0)*y+right_best_model.at<double>(0,0)*y*y;
        points_fitted_right.push_back(cv::Point(x_right, y));
    }
			
    polylines(image_mask, points_fitted_left, false, cv::Scalar(0, 255, 255), 4, 2, 0); 
    polylines(image_mask, points_fitted_right, false, cv::Scalar(0, 255, 255), 4, 2, 0);
    //融合
    Mat out;
    if(ipm_flag)
    {
        //反变换
        image_mask=ipmo.inverse_perspective_img_roi(image_mask,p1x,p2x); 
        //除去明显不是车道线的区域,中心线下方的区域
        //选择感兴趣区域

        Rect r2(0, 0, img.cols,img.rows-440);
        Mat mask_roi = image_mask(r2);
        mask_roi.copyTo(image_mask2(r1));

        cvtColor(img,img,CV_GRAY2BGR);
        addWeighted(img, 1, image_mask2, 2, -1, out);   
    }
    else
    {
        cvtColor(img,img,CV_GRAY2BGR);
        addWeighted(img, 1, image_mask, 2, -1, out);   
    }
    

    imshow("src",img);
    imshow("roi",roi);
    imshow("evt",roi_evt);
    imshow("and",dst);
    imshow("lane_im",lane_im);
    imshow("lane",lane);
    imshow("image_mask",image_mask);
    imshow("out",out);
    cout <<"all time is " << 1000* (clock() - time_s)/(double)CLOCKS_PER_SEC << "ms"<< endl;

    waitKey(1);    
    }
}