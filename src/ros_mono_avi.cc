#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../orb_slam3/include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    string ORBvoc_path = "/home/touchair/vio_ws/src/orb_slam3/Vocabulary/ORBvoc.txt";
    string config_path = "/home/touchair/vio_ws/src/orb_slam3/Examples/Monocular/my_PC.yaml";

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
    // ros::Subscriber sub = nodeHandler.subscribe("/cv/ipcam/image_raw", 1, &ImageGrabber::GrabImage,&igb);
   ros::Subscriber sub = nodeHandler.subscribe("/cv/ipcam/image_raw", 1, &ImageGrabber::GrabImage,&igb);

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
    cv::resize(img,img,cv::Size(640,480));
  //  cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(960,480));
 //   mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    mpSLAM->TrackMonocular(img,cv_ptr->header.stamp.toSec());
    
}


