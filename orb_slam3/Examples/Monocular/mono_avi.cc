// if ( cv::waitKey(15) =='q')
// string argv2 = "Examples/Monocular/TUM1.yaml";
// cv::VideoCapture cam("/home/lin/output/jc/2.MP4");


// string argv2 = "Examples/Monocular/TUM_512.yaml";
//  cv::VideoCapture cam("/home/lin/output/jc/6.mp4");
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;



int main(int argc, char **argv)
{
   
    string argv1 = "/home/touchair/vio_ws/src/orb_slam3/Vocabulary/ORBvoc.txt";
    // string argv2 = "Examples/Monocular/TUM_avi.yaml";
    // string argv2 = "Examples/Monocular/TUM1.yaml";
    string argv2 = "/home/touchair/vio_ws/src/orb_slam3/Examples/Monocular/my_PC.yaml";
    cout<<"open videos"<<endl;

     //cv::VideoCapture cam(0);
    // cv::VideoCapture cam("/home/touchair/VID_20220225_151648.mp4");
    cv::VideoCapture cam("/home/touchair/VID_20220216_141813.mp4");
    
    cv::Mat im;
    int tframe =0;  //时间戳，double类型，这里用计数器就行从0开始

    ORB_SLAM3::System SLAM(argv1,argv2,ORB_SLAM3::System::MONOCULAR,true);
    if(!cam.isOpened())
        {
            cerr << endl << "image || camera Failed to load  !"<<endl;
            return 1;
            }
    while (cam.isOpened())
    {
        cam>>im;           //读取当前帧
        // cv::resize(im,im,cv::Size(960,480));
        cv::resize(im,im,cv::Size(640,360));
        im = im(cv::Rect(0,0,640,360)); // 裁剪图像
        SLAM.TrackMonocular(im,tframe);
        if (im.empty())
            break;
        
        tframe++;
    }
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("avi_KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    cam.release();     //释放摄像头资源
    cv::destroyAllWindows();   //释放全部窗口
    return 0;
}


