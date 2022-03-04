// rosrun image_transport republish compressed in:=/camera/image raw out:=/usb_cam/image_raw

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 1 || argc > 2)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }


  if(argc==1)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  string ORBvoc_path = "/home/lin/code/ORB_SLAM3/Vocabulary/ORBvoc.txt";
  string config_path = "/home/lin/code/ORB_SLAM3/Examples/ROS/ORB_SLAM3/src/cam_inertial.yaml";
  
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(ORBvoc_path,config_path,ORB_SLAM3::System::IMU_MONOCULAR,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual); // TODO
  
  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/android/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img0 = n.subscribe("/usb_cam/image_raw", 100, &ImageGrabber::GrabImage,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());
      img0Buf.pop();
      this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      // {
      //   // Load imu measurements from buffer
      //   vImuMeas.clear();
      //   while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
      //   {
      //     double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
      //     cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
      //     cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
      //     vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
      //     mpImuGb->imuBuf.pop();
      //   }
      // }

      if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                static bool time_flag = true;
                static auto TIME_OFFSET = 0.0;
                if(time_flag){
                    TIME_OFFSET = mpImuGb->imuBuf.front()->header.stamp.toSec()-tIm;
                    time_flag = false;
                }
                cout<<"imu_time: "<<setprecision(11)<<mpImuGb->imuBuf.front()->header.stamp.toSec()-TIME_OFFSET<<endl;
                cout<<"image_time: "<<setprecision(11)<<tIm<<endl;
                cout<<"---------------"<<endl;
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()-TIME_OFFSET<=tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec() - TIME_OFFSET;
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    cout<<"imudata: "<< t<<" "<<acc.x<<" "<<acc.y<<" "<<acc.z<<" "<<gyr.x<<" "<<gyr.y<<" "<<gyr.z<<endl;
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
 
                    mpImuGb->imuBuf.pop();
                }
            }
      


      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);


      cv::resize(im,im,cv::Size(640,480));
      
      if(cv::waitKey(10)=='q')
        break;
      mpSLAM->TrackMonocular(im,tIm,vImuMeas);

    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}


