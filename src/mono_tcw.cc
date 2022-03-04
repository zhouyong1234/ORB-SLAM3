// rosrun image_transport republish compressed in:=/camera/image raw out:=/usb_cam/image_raw

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>


#include"/home/touchair/vio_ws/src/orb_slam3/include/System.h"

#include"std_msgs/String.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;

class ImageGrabber
{
public:

    ros::NodeHandle nh;  //定义句柄初始化
	ros::Publisher  pub1,pub_tcw;  //定义发布者

    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM),nh("~")
    {
        pub1=nh.advertise<std_msgs::String>("fang",10);
         // 编写发布逻辑并发布数据
    // 要求以10hz频率发布数据，并且文本后添加编号
        pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("CameraPose", 10); 
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    string ORBvoc_path = "/home/lin/code/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    string config_path = "/home/lin/code/ORB_SLAM3/Examples/ROS/ORB_SLAM3/Asus.yaml";

    if(argc != 1)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(ORBvoc_path,config_path,ORB_SLAM3::System::MONOCULAR,true);
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);
   // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat img;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // 跟踪线程 图片与时间戳
    cv_ptr->image.copyTo(img);
    
    cv::resize(img,img,cv::Size(960,480));

    cv::Mat Tcw;
    Tcw = mpSLAM->TrackMonocular(img,cv_ptr->header.stamp.toSec());
    if(!Tcw.empty())
    {
        cv::Mat Twc =Tcw.inv();
					cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3);  
					cv::Mat tWC=  Twc.rowRange(0,3).col(3);

					Eigen::Matrix<double,3,3> eigMat ;
					eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
									RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
									RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
					Eigen::Quaterniond q(eigMat);
 
				 geometry_msgs::PoseStamped tcw_msg; 					
                 tcw_msg.pose.position.x=tWC.at<float>(0);
                 tcw_msg.pose.position.y=tWC.at<float>(1);			 
                 tcw_msg.pose.position.z=tWC.at<float>(2);
				 
				tcw_msg.pose.orientation.x=q.x();
				tcw_msg.pose.orientation.y=q.y();
				tcw_msg.pose.orientation.z=q.z();
				tcw_msg.pose.orientation.w=q.w();

        // tcw_msg.header= header;
        // std_msgs::Header header ;
        pub_tcw.publish(tcw_msg);

    }
    else
	{
	  cout<<"Twc is empty ..."<<endl;
	}
}


