/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>
#include<opencv2/core/core.hpp>
#include <k4a/k4a.hpp>
#include <System.h>
#include <boost/format.hpp>

using namespace std;
using namespace cv;
class RGBDdetactor
{
    k4a::device device;
    k4a_device_configuration_t config;
    k4a::image rgbImage;
    k4a::image depthImage;
    k4a::image transformed_depthImage;
    k4a::capture capture;
    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    std::chrono::_V2::system_clock::time_point start;

public:
    int verifyKinect()
    {
        const uint32_t device_count = k4a::device::get_installed_count();
        if (0 == device_count)
        {
            std::cout << "Error: no K4A devices found. " << std::endl;
            return -1;
        }
        else
        {
            std::cout << "Found " << device_count << " connected devices. " << std::endl;
            if (1 != device_count) // 超过1个设备，也输出错误信息。
            {
                std::cout << "Error: more than one K4A devices found. " << std::endl;
                return -1;
            }
            else // 该示例代码仅限对1个设备操作
            {
                std::cout << "Done: found 1 K4A device. " << std::endl;
                return 0;
            }
        }
    }
    void startInitKinect()
    {
        device = k4a::device::open(K4A_DEVICE_DEFAULT);
        start = std::chrono::system_clock::now();
        std::cout << "Done: open device. " << std::endl;

        /*
            检索并保存 Azure Kinect 图像数据
        */
        // 配置并启动设备
        config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
        config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        //确保深度和彩色图像在捕获中均可用
        config.synchronized_images_only = true;
        device.start_cameras(&config);
        std::cout << "Done: start camera." << std::endl;
    }
    int autoExposure()
    {
        int iAuto = 0;      //用来稳定，类似自动曝光
        int iAutoError = 0; // 统计自动曝光的失败次数
        while (true)
        {
            if (device.get_capture(&capture))
            {
                std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;

                // 跳过前 n 个（成功的数据采集）循环，用来稳定
                if (iAuto != 30)
                {
                    iAuto++;
                    continue;
                }
                else
                {
                    std::cout << "Done: auto-exposure" << std::endl;
                    break; // 跳出该循环，完成相机的稳定过程
                }
            }
            else
            {
                std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
                if (iAutoError != 30)
                {
                    iAutoError++;
                    continue;
                }
                else
                {
                    std::cout << "Error: failed to give auto-exposure. " << std::endl;
                    return -1;
                }
            }
        }
        std::cout << "-----------------------------------" << std::endl;
        std::cout << "----- Have Started Kinect DK. -----" << std::endl;
        std::cout << "-----------------------------------" << std::endl;
        return 0;
    }
    void updateImage()
    {
        if (device.get_capture(&capture))
        {
            // rgb
            // * BGRA32数据的每个像素为四个字节。 前三个字节代表蓝色，绿色，
            // * 和红色数据。 第四个字节是Alpha通道，在Azure Kinect API中未使用。
            rgbImage = capture.get_color_image();
#if DEBUG_std_cout == 1
            std::cout << "[rgb] "
                      << "\n"
                      << "format: " << rgbImage.get_format() << "\n"
                      << "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n"
                      << "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n"
                      << "height*width: " << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels()
                      << std::endl;
#endif

            // depth
            // * DEPTH16数据的每个像素是两个字节的小端无符号深度数据。
            // * 数据的单位距离相机的原点以毫米为单位。
            depthImage = capture.get_depth_image();
#if DEBUG_std_cout == 1
            std::cout << "[depth] "
                      << "\n"
                      << "format: " << depthImage.get_format() << "\n"
                      << "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n"
                      << "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n"
                      << "height*width: " << depthImage.get_height_pixels() << ", " << depthImage.get_width_pixels()
                      << std::endl;
#endif

            //深度图和RGB图配准
            k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution); //获取相机标定参数

            k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);

            transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);

            cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
                                             (void *)rgbImage.get_buffer());
            cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);

            cv_depth = cv::Mat(transformed_depthImage.get_height_pixels(), transformed_depthImage.get_width_pixels(), CV_16U,
                               (void *)transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));

            normalize(cv_depth, cv_depth_8U, 0, 256 * 256, NORM_MINMAX);
            cv_depth_8U.convertTo(cv_depth, CV_8U, 1);
        }
    }
    Mat RGBImageGet(){
        return cv_rgbImage_no_alpha;
    }
    Mat DImageGet(){
        return cv_depth_8U;
    }
    double stampGet(){
        auto now = chrono::system_clock::now();
        std::chrono::milliseconds timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        return double(timestamp.count())/1000.0;
    }
    RGBDdetactor(){};
    ~RGBDdetactor()
    {
        cv_rgbImage_with_alpha.release();
        cv_rgbImage_no_alpha.release();
        cv_depth.release();
        cv_depth_8U.release();

        rgbImage.reset();
        depthImage.reset();
        capture.reset();
        device.close();
    };
};

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings " << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    RGBDdetactor rgbdetactor;
    if(rgbdetactor.verifyKinect() == -1){
        return 0;
    }
    rgbdetactor.startInitKinect();
    rgbdetactor.autoExposure();
    

    cv::Mat imRGB, imD;
    double timeStamp;
    for(int i = 0;i<150;i++){
        rgbdetactor.updateImage();
        imRGB = rgbdetactor.RGBImageGet();  
        imD = rgbdetactor.DImageGet();
        timeStamp = rgbdetactor.stampGet();
        Mat imRGBout;
        cv::resize(imRGB, imRGBout, Size(640, 480));
        Mat imDout;
        cv::resize(imD, imDout, Size(640, 480));
        SLAM.TrackRGBD( imRGBout, imDout, timeStamp  );
    }
    rgbdetactor.~RGBDdetactor();
    SLAM.Shutdown();
    return 0;
}

