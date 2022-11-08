#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/time_synchronizer.h>


using namespace std;
using namespace cv;
using namespace message_filters;

ros::Subscriber left_sub;
ros::Subscriber right_sub;
ros::Publisher depth_pub;

Mat left_img;
Mat right_img;

/* 相机参数 */
double fx = 387.355;
double fy = 387.355;
double cx =  315.917;
double cy = 243.442;
double b = 0.095;
void img2depth();


/* 相机左右图像同步回调 */
void photoCallback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
{

    /* 解码相机消息:将ros话题图像转换为opencv图像 */
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_ptr_left = cv_bridge::toCvCopy(left,left->encoding);
    left_img = cv_ptr_left->image;
    
    cv_bridge::CvImagePtr cv_ptr_right;
    cv_ptr_right = cv_bridge::toCvCopy(right,right->encoding);
    right_img = cv_ptr_right->image;

    /* sgbm 算法 */
    img2depth();

}

/* sgbm生成深度图 */

void img2depth()
{
    /* 创建sgbm 对象*/
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8*9*9, 32*9*9, 1,63, 10, 100, 32);
    cv::Mat  disparity_sgbm, disparity;
    sgbm->compute(left_img,right_img,disparity_sgbm);
    /* 得到视差 */
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0/16.0f);
    for(int v = 0; v < left_img.rows; ++v)
    {
        for(int u = 0; u < left_img.cols;++u)
        {
            double depth = fx*b/(disparity.at<float>(v,u));
            disparity.at<float>(v,u) = depth;
        }
    }
    sensor_msgs::ImageConstPtr img_msg = cv_bridge::CvImage(std_msgs::Header(),"sgbmdepth",disparity).toImageMsg();

    depth_pub.publish(img_msg);
}


int main(int argc, char *argv[])
{
    cout << "hello!" << endl;
    ros::init(argc,argv,"depth_sgbm");
    ros::NodeHandle nh("~");

    //同步订阅左右图
    message_filters::Subscriber<sensor_msgs::Image>  left_sub(nh,"/left",1000);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh,"/right",1000);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),left_sub ,right_sub);
    sync.registerCallback(boost::bind(&photoCallback,_1,_2));

    //发布深度图
    depth_pub = nh.advertise<sensor_msgs::Image>("/sgbm/depth",10);
    ros::spin();

    return 0;
}



