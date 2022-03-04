// if ( cv::waitKey(15) =='q')
// string argv2 = "Examples/Monocular/TUM1.yaml";
// cv::VideoCapture cam("/home/lin/output/jc/2.MP4");
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;



int main(int argc, char **argv)
{
   
    string argv1 = "Vocabulary/ORBvoc.txt";
    string argv2 = "Examples/Monocular/TUM1.yaml";
   
     cv::Mat im;
     int tframe =0;  //时间戳，double类型，这里用计数器就行从0开始

    ORB_SLAM3::System SLAM(argv1,argv2,ORB_SLAM3::System::MONOCULAR,true);
   
    while (1)
    {
      cv::Mat im = cv::imread("/home/lin/ff/1.png");
      //  cv::resize(im,im,cv::Size(960,540));
        cv::resize(im,im,cv::Size(640,480));
        SLAM.TrackMonocular(im,tframe);
        if ( cv::waitKey(40) =='q'|| im.empty())
            break;
       
        tframe++;
    }
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("avi_KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
   
    cv::destroyAllWindows();   //释放全部窗口
    return 0;
}


