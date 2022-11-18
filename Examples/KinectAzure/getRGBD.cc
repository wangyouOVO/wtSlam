// C++
#include <iostream>
#include <chrono>
#include <string>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// Kinect DK
#include <k4a/k4a.hpp>

using namespace cv;
using namespace std;

// 宏
// 方便控制是否 std::cout 信息
#define DEBUG_std_cout 1

int main(int argc, char *argv[])
{
    /*
        找到并打开 Azure Kinect 设备
    */
    // 发现已连接的设备数

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
        }
    }
    // 打开（默认）设备
    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
    std::cout << "Done: open device. " << std::endl;

    /*
        检索并保存 Azure Kinect 图像数据
    */
    // 配置并启动设备
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    //确保深度和彩色图像在捕获中均可用
    config.synchronized_images_only = true;
    device.start_cameras(&config);
    std::cout << "Done: start camera." << std::endl;

    // 稳定化
    k4a::capture capture;
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
    // 从设备获取捕获
    k4a::image rgbImage;
    k4a::image depthImage;
    k4a::image irImage;
    k4a::image transformed_depthImage;

    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    cv::Mat cv_irImage;
    cv::Mat cv_irImage_8U;

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

        // ir
        // * IR16数据的每个像素是两个字节的小端无符号深度数据。 数据的值代表亮度。
        irImage = capture.get_ir_image();
#if DEBUG_std_cout == 1
        std::cout << "[ir] "
                  << "\n"
                  << "format: " << irImage.get_format() << "\n"
                  << "device_timestamp: " << irImage.get_device_timestamp().count() << "\n"
                  << "system_timestamp: " << irImage.get_system_timestamp().count() << "\n"
                  << "height*width: " << irImage.get_height_pixels() << ", " << irImage.get_width_pixels()
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

        cv_irImage = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U,
                             (void *)irImage.get_buffer(), static_cast<size_t>(irImage.get_stride_bytes()));
        normalize(cv_irImage, cv_irImage_8U, 0, 256 * 256, NORM_MINMAX);
        cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1);

        // show image
        cv::imshow("color", cv_rgbImage_no_alpha);
        cv::waitKey(2000);
        cv::imshow("depth", cv_depth_8U);
        cv::waitKey(2000);
        cv::imshow("ir", cv_irImage_8U);
        cv::waitKey(2000);
        // 保存图片

        std::string filename_rgb = "rgb.png";

        std::string filename_d = "depth.png";

        std::string filename_ir = "ir.png";
        // imwrite("./rgb/" + filename_rgb, cv_rgbImage_no_alpha);
        // imwrite("./depth/" + filename_d, cv_depth_8U);
        // imwrite("./ir/" + filename_ir, cv_irImage_8U);
        imwrite(filename_rgb, cv_rgbImage_no_alpha);
        imwrite(filename_d, cv_depth_8U);
        imwrite(filename_ir, cv_irImage_8U);

        std::cout << "Acquiring!" << endl;

        cv_rgbImage_with_alpha.release();
        cv_rgbImage_no_alpha.release();
        cv_depth.release();
        cv_depth_8U.release();
        cv_irImage.release();
        cv_irImage_8U.release();

        capture.reset();
    }
    else
    {
        std::cout << "false: K4A_WAIT_RESULT_TIMEOUT." << std::endl;
    }

    cv::destroyAllWindows();

    // 释放，关闭设备
    rgbImage.reset();
    depthImage.reset();
    irImage.reset();
    capture.reset();
    device.close();

    return 1;
}

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
    RGBDdetactor();
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