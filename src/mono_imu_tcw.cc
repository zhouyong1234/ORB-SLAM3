/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

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

#include"/home/touchair/vio_ws/src/orb_slam3/include/System.h"
#include"../include/ImuTypes.h"


#include <geometry_msgs/PoseStamped.h>

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

   ros::NodeHandle nh;  //定义句柄初始化
	ros::Publisher  pub1,pub_tcw;  //定义发布者
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe),nh("~")
    {
        pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("CameraPose", 10); 
    }

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
      // if(!mpImuGb->imuBuf.empty())
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
      // 时间对齐
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
      // if(cv::waitKey(10)=='q')
      //   break;
       
    cv::Mat Tcw;
    Tcw =  mpSLAM->TrackMonocular(im,tIm,vImuMeas); 
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

        
        pub_tcw.publish(tcw_msg);
    }
    else
	  {
	  cout<<"Twc is empty ..."<<endl;
	   }
   
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


