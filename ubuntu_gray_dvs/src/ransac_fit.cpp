#include<ransac_fit.h>

double ransac_fit::uniform_random()
{
    //生成[0,1]之间符合均匀分布的数
    return (double)rand() / (double)RAND_MAX;
}

//这样生成的高斯分布随机数序列的期望为0.0，方差为1.0
//若指定期望为E，方差为V，则只需增加
//X = X * V + E;
double ransac_fit::generate_random()
{
     /* This Gaussian routine is stolen from Numerical Recipes and is their copyright. */ 
     static int next_gaussian = 0; 
     static double saved_gaussian_value; 
     double fac, rsq, v1, v2; 
     if (next_gaussian == 0) 
     { 
        do 
        { 
            v1 = 2 * uniform_random() - 1; 
            v2 = 2 * uniform_random() - 1; 
            rsq = v1*v1 + v2*v2; 
        }
        while (rsq >= 1.0 || rsq == 0.0); 
        fac = sqrt(-2 * log(rsq) / rsq); 
        saved_gaussian_value = v1*fac; 
        next_gaussian = 1; 
        return v2*fac; 
        } 
    else 
    {
        next_gaussian = 0;
        return saved_gaussian_value;
    }
}

//随机采样n个samples
bool ransac_fit::get_sample(int set_size,vector<int> &sub_set,int n)
{
    vector<int> temp;
    if(set_size>n)
    {
        temp.push_back(int(uniform_random()*(set_size-1)));

        int j=1;
        while(j<n)
        {
            int t = int(uniform_random()*(set_size-1));
            if(find(temp.begin(), temp.end(), t)==temp.end())//指针指向了末尾
            {
                temp.push_back(t);
                j++;
            }
        }
        for(int j=0;j<n;j++)
        {
//            cout<<temp[j]<<endl;
            sub_set.push_back(temp[j]);
        }
    }
    else
    {
        return false;
    }
    return true;
}


void ransac_fit::ransac(vector<Point2d> set,cv::Mat &best_model,int n_max,int n_sample,double thre,vector<bool> &inliner_flag)
{
    srand(time(NULL));
    vector<bool> inlierstemp;
   
    int max_cnt = 0;
    int model_size = best_model.rows;

    printf("model_size=%d\n",model_size);
    printf("set_size=%d\n",set.size());


    inliner_flag = vector<bool>(set.size(), false);
    for(int i=0;i<n_max;i++)
    {
        //1.随机采样
        vector<int> sub_set;
        get_sample(set.size(),sub_set,n_sample);
        //2.模型估计
        //构造AB矩阵，所有数据
        cv::Mat A(set.size(), model_size, CV_64FC1) ;
        cv::Mat B(set.size(),1, CV_64FC1) ;
        for(int j=0;j<set.size();j++)
		{
            for(int k=0;k<model_size;k++)
            {
                A.at<double>(j,k) = pow(set[j].x, model_size-k-1) ;
            //    printf("pow(set[j].x, model_size-k)=%f\n",pow(set[j].x, model_size-k));
            }
            B.at<double>(j,0) = set[j].y ;
            //printf("set[j].y=%f\n",set[j].y);
		}
        //构造AABB矩阵，采样数据
        cv::Mat AA(n_sample,model_size,CV_64FC1) ;
		cv::Mat BB(n_sample,1, CV_64FC1) ;
		for(int j=0;j<n_sample;j++)
		{
            for(int k=0;k<model_size;k++)
            {
                AA.at<double>(j,k) = pow(set[sub_set[j]].x, model_size-k-1) ;
            }
            BB.at<double>(j,0) = set[sub_set[j]].y ;
		}
		cv::Mat AA_pinv(model_size,n_sample,CV_64FC1) ;
		invert(AA, AA_pinv, cv::DECOMP_SVD);//用svd求为你
		cv::Mat X = AA_pinv * BB ;

        cout <<  X.at<double>(0,0)  << endl;
	    cout <<  X.at<double>(1,0)  << endl;

        //3.模型评估
        cv::Mat residual(set.size(),1,CV_64FC1) ;
		residual = cv::abs(B-A*X) ;//比较计算得到的y和实际y的绝对值距离，距离过大则为外点

        int inliners = 0;
        for( int j=0;j<set.size();j++)
		{
			double data = residual.at<double>(j,0) ;
			
			if( data < thre ) 
			{
				inliners++ ;
     //           inlierstemp[i] = true;
			}
		}
    //    printf("77\n");
        if( inliners > max_cnt ) 
		{
			best_model = X ;
			max_cnt = inliners ;
        //    inliner_flag = inlierstemp;
		}

    //    cout << max_cnt << endl;
    }
}

bool ransac_fit::ransac2(vector<Point2d> set,cv::Mat &best_model,int n_max,int n_sample,double thre,vector<bool> &inliner_flag)
{
    srand(time(NULL));
    vector<bool> inlierstemp;
   
    int max_cnt = 0;
    int model_size = best_model.rows;

//    printf("model_size=%d\n",model_size);
//    printf("set_size=%d\n",set.size());

    bool flag = true;
    inliner_flag = vector<bool>(set.size(), false);
    for(int i=0;i<n_max;i++)
    {
        //1.随机采样
        vector<int> sub_set;
        flag = get_sample(set.size(),sub_set,n_sample);
        if(!flag)
            return false;
        //2.模型估计
        //构造AB矩阵，所有数据
        cv::Mat A(set.size(), model_size, CV_64FC1) ;
        cv::Mat B(set.size(),1, CV_64FC1) ;
        for(int j=0;j<set.size();j++)
		{
            for(int k=0;k<model_size;k++)
            {
                A.at<double>(j,k) = pow(set[j].x, model_size-k-1) ;
            //    printf("pow(set[j].x, model_size-k)=%f\n",pow(set[j].x, model_size-k));
            }
            B.at<double>(j,0) = set[j].y ;
            //printf("set[j].y=%f\n",set[j].y);
		}
        //构造AABB矩阵，采样数据
        cv::Mat AA(n_sample,model_size,CV_64FC1) ;
		cv::Mat BB(n_sample,1, CV_64FC1) ;
		for(int j=0;j<n_sample;j++)
		{
            for(int k=0;k<model_size;k++)
            {
                AA.at<double>(j,k) = pow(set[sub_set[j]].x, model_size-k-1) ;
            }
            BB.at<double>(j,0) = set[sub_set[j]].y ;
		}
		cv::Mat AA_pinv(model_size,n_sample,CV_64FC1) ;
		invert(AA, AA_pinv, cv::DECOMP_SVD);//用svd求为你
		cv::Mat X = AA_pinv * BB ;

//        cout <<  X.at<double>(0,0)  << endl;
//	    cout <<  X.at<double>(1,0)  << endl;

        //3.模型评估
        cv::Mat residual(set.size(),1,CV_64FC1) ;
		residual = cv::abs(B-A*X) ;//比较计算得到的y和实际y的绝对值距离，距离过大则为外点

        int inliners = 0;
        for( int j=0;j<set.size();j++)
		{
			double data = residual.at<double>(j,0) ;
			
			if( data < thre ) 
			{
				inliners++ ;
     //           inlierstemp[i] = true;
			}
		}
    //    printf("77\n");
        if( inliners > max_cnt ) 
		{
			best_model = X ;
			max_cnt = inliners ;
        //    inliner_flag = inlierstemp;
		}

    //    cout << max_cnt << endl;
    }
    return true;
}

void ransac_fit::generation_data_line(Mat &img,vector<Point2d> &ptSet)
{
    //demo 
    int width = 640; 
    int height = 320; 
    //直线参数 
    double a = 1, b = 2, c = -640; 
    //随机获取直线上20个点 
    int ninliers = 0; 
   
    srand((unsigned int)time(NULL)); //设置随机数种子 
    while (true) 
    { 
        double x = uniform_random()*(width - 1); 
        double y = -(a*x + c) / b; 
        //加0.5高斯噪声 
        x += generate_random()*0.5; 
        y += generate_random()*0.5; 
        if (x >= 640 && y >= 320) continue;
        Point2d pt(x, y); 
        ptSet.push_back(pt); 
        ninliers++; 
        if (ninliers == 20) break; 
    } 
    int nOutliers = 0; //随机获取10个野值点 
    while (true) 
    { 
        double x = uniform_random() * (width - 1); 
        double y = uniform_random() * (height - 1);
        if (fabs(a*x + b*y + c) < 10) //野值点到直线距离不小于10个像素 
            continue; 
        Point2d pt(x, y); 
        ptSet.push_back(pt); 
        nOutliers++; 
        if (nOutliers == 10) 
            break; 
    }
    //绘制点集中所有点 
    for (unsigned int i = 0; i < ptSet.size(); i++) 
        circle(img, ptSet[i], 3, Scalar(255, 0, 0), 3, 8);
    imshow("line fitting", img);
}

void ransac_fit::generation_data_curve(Mat &img,vector<Point2d> &ptSet)
{
    //demo 
    int width = 640; 
    int height = 320; 
    //曲线参数
    double a = 0.005, b = 0.5, c = 0; 
    //随机获取直线上20个点 
    int ninliers = 0; 
   
    srand((unsigned int)time(NULL)); //设置随机数种子 
    while (true) 
    { 
        double x = uniform_random()*(width - 1); 
        double y = a*x*x + b*x + c; 
        //加0.5高斯噪声 
        x += generate_random()*0.5; 
        y += generate_random()*0.5; 
        if (x >= 640 && y >= 320) continue;
        Point2d pt(x, y); 
        ptSet.push_back(pt); 
        ninliers++; 
        if (ninliers == 30) break; 
    } 
    int nOutliers = 0; //随机获取10个野值点 
    while (true) 
    { 
        double x = uniform_random() * (width*0.5 - 1); 
        double y = uniform_random() * (height - 1);
        if (fabs(a*x*x + b*x + c - y) < 5) //野值点到直线距离不小于10个像素 
            continue; 
        Point2d pt(x, y); 
        ptSet.push_back(pt); 
        nOutliers++; 
        if (nOutliers == 10) 
            break; 
    }
    //绘制点集中所有点 
    for (unsigned int i = 0; i < ptSet.size(); i++) 
        circle(img, ptSet[i], 3, Scalar(255, 0, 0), 3, 8);
    imshow("line fitting", img);
}
