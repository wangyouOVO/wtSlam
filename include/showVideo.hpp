#ifndef _SHOWVIDEO_H_
#define _SHOWVIDEO_H_
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <string>
#include <iostream>
#endif

namespace wtSlam
{
    class ShowVideoUtil
    {

        std::string fileName;

    public:
        ShowVideoUtil() {}
        ShowVideoUtil(std::string fileName) : fileName(fileName) {}
        void showVideo()
        {
            cv::VideoCapture capture;
            capture.open(fileName);
            cv::Mat frame;
            if (!capture.isOpened())
            {
                std::cout << "Open the Video failed!";
            }
            while (1)
            {
                if (!capture.read(frame))
                {
                    std::cout << "no video frame" << std::endl;
                    break;
                }
                cv::Mat grayFrame;
                cv::cvtColor(frame,grayFrame,cv::COLOR_BGR2GRAY);
                cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
                std::vector<cv::KeyPoint> keyPoints;
                detector->detect(grayFrame,keyPoints);
                int radius = 10; 
                cv::Scalar RGB(0,255,0);
                for(auto item:keyPoints){
                    std::cout << "["<<item.pt.x <<","<<item.pt.y <<"]"<<std::endl;
                    cv::circle(frame,item.pt,radius,cv::Scalar(255,0,0),2);
                }


                cv::imshow("video",frame);
                if(cv::waitKey(30)==27){
                    break;
                };
            }
        }
    };

}