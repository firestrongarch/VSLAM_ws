#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include "ov2slam.hpp"
#include "slam_params.hpp"

void LoadImages(const std::string &strPath, std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps);

class SensorsGrabber {

public:
    SensorsGrabber(SlamManager *slam): pslam_(slam) {
        std::cout << "\nSensors Grabber is created...\n";
    }

    void subLeftImage(const sensor_msgs::msg::Image &image) {
        std::lock_guard<std::mutex> lock(img_mutex);
        img0_buf.push(image);
    }

    void subRightImage(const sensor_msgs::msg::Image &image) {
        std::lock_guard<std::mutex> lock(img_mutex);
        img1_buf.push(image);
    }

    cv::Mat getGrayImageFromMsg(const sensor_msgs::msg::Image &img_msg)
    {
        // Get and prepare images
        cv_bridge::CvImageConstPtr ptr;
        try {    
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC3);
        } 
        catch(cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("cv_bridge_logger"), "\n\n\ncv_bridge exeception: %s\n\n\n", e.what());
        }

        return ptr->image;
    }

    // extract images with same timestamp from two topics
    // (mostly derived from Vins-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
    void sync_process()
    {
        std::cout << "\nStarting the measurements reader thread!\n";
        
        while( !pslam_->bexit_required_ )
        {
            if( pslam_->pslamstate_->stereo_ )
            {
                cv::Mat image0, image1;

                std::lock_guard<std::mutex> lock(img_mutex);

                if (!img0_buf.empty() && !img1_buf.empty())
                {
                    double time0 = rclcpp::Time( img0_buf.front().header.stamp ).seconds();
                    double time1 = rclcpp::Time( img1_buf.front().header.stamp ).seconds();

                    // sync tolerance
                    if(time0 < time1 - 0.015)
                    {
                        img0_buf.pop();
                        std::cout << "\n Throw img0 -- Sync error : " << (time0 - time1) << "\n";
                    }
                    else if(time0 > time1 + 0.015)
                    {
                        img1_buf.pop();
                        std::cout << "\n Throw img1 -- Sync error : " << (time0 - time1) << "\n";
                    }
                    else
                    {
                        image0 = getGrayImageFromMsg(img0_buf.front());
                        image1 = getGrayImageFromMsg(img1_buf.front());
                        cv::cvtColor(image0, image0, cv::COLOR_BGR2GRAY);
                        cv::cvtColor(image1, image1, cv::COLOR_BGR2GRAY);
                        img0_buf.pop();
                        img1_buf.pop();

                        if( !image0.empty() && !image1.empty() ) {
                            pslam_->addNewStereoImages(time0, image0, image1);
                        }
                    }
                }
            } 
            else if( pslam_->pslamstate_->mono_ ) 
            {
                cv::Mat image0;

                std::lock_guard<std::mutex> lock(img_mutex);

                if ( !img0_buf.empty() )
                {
                    double time = rclcpp::Time( img0_buf.front().header.stamp ).seconds();
                    image0 = getGrayImageFromMsg(img0_buf.front());
                    img0_buf.pop();

                    if( !image0.empty()) {
                        pslam_->addNewMonoImage(time, image0);
                    }
                }
            }

            std::chrono::milliseconds dura(1);
            std::this_thread::sleep_for(dura);
        }

        std::cout << "\n Bag reader SyncProcess thread is terminating!\n";
    }

    std::queue<sensor_msgs::msg::Image> img0_buf;
    std::queue<sensor_msgs::msg::Image> img1_buf;
    std::mutex img_mutex;
    
    SlamManager *pslam_;
};

int main(int argc, char** argv)
{
    // Init the node
    rclcpp::init(argc, argv);

    if(argc < 2)
    {
       std::cout << "\nUsage: rosrun ov2slam ov2slam_node parameters_files/params.yaml\n";
       return 1;
    }

    std::cout << "\nLaunching OV²SLAM...\n\n";

    auto node = rclcpp::Node::make_shared("ov2slam_node");

    // Load the parameters
    std::string parameters_file = argv[1];

    std::cout << "\nLoading parameters file : " << parameters_file << "...\n";

    const cv::FileStorage fsSettings(parameters_file.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
       std::cout << "Failed to open settings file...";
       return 1;
    } else {
        std::cout << "\nParameters file loaded...\n";
    }

    std::shared_ptr<SlamParams> pparams;
    pparams.reset( new SlamParams(fsSettings) );

    // Create the ROS Visualizer
    std::shared_ptr<RosVisualizer> prosviz;
    prosviz.reset( new RosVisualizer(node) );

    // Setting up the SLAM Manager
    SlamManager slam(pparams, prosviz);

    // Start the SLAM thread
    std::thread slamthread(&SlamManager::run, &slam);

    // Create the Bag file reader & callback functions
    SensorsGrabber sb(&slam);

    // Create callbacks according to the topics set in the parameters file
    auto subleft = node->create_subscription<sensor_msgs::msg::Image>(fsSettings["Camera.topic_left"], 2, [&sb](const sensor_msgs::msg::Image &image){return sb.subLeftImage(image);});
    auto subright = node->create_subscription<sensor_msgs::msg::Image>(fsSettings["Camera.topic_right"], 2, [&sb](const sensor_msgs::msg::Image &image){return sb.subRightImage(image);});

    // Start a thread for providing new measurements to the SLAM
    std::thread sync_thread(&SensorsGrabber::sync_process, &sb);

    std::cout<<"\nStarting datasets...\n";
    std::vector<std::string> vstrImageLeft;
    std::vector<std::string> vstrImageRight;
    std::vector<double> vTimeStamp;
    LoadImages("/data/datasets/lab-module-csc", vstrImageLeft, vstrImageRight, vTimeStamp);
    const int nImages = vstrImageLeft.size();
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    std::cout<<"nImages: "<<nImages<<"\n";
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        // step 4.1 读取原始图像
        imLeft = cv::imread(vstrImageLeft[ni]);
        imRight = cv::imread(vstrImageRight[ni]);
        long tframe = vTimeStamp[ni];

        // auto time = node->now();
        auto imLeft_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imLeft).toImageMsg();
        auto imRight_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imRight).toImageMsg();
        imLeft_msg->header.stamp.nanosec = tframe % 1000000000;
        imRight_msg->header.stamp.nanosec = tframe % 1000000000;
        imLeft_msg->header.stamp.sec = tframe / 1000000000;
        imRight_msg->header.stamp.sec = tframe / 1000000000;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the images to the SLAM system
        // step 4.5 开始追踪
        // cv::Mat left(IMG_H, IMG_W, CV_8UC3),right(IMG_H, IMG_W, CV_8UC3);
        // net.extract(imLeftRect, left);
        // net.extract(imRightRect, right);

        sb.subLeftImage(*imLeft_msg);
        sb.subRightImage(*imRight_msg);
        // slam.addNewStereoImages(tframe, imLeftRect,imRightRect);

        // step 4.6 追踪完成，停止计时，计算追踪时间
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        static std::vector<float> vTimesTrack(nImages);
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        // // step 4.7 等待一段时间以符合下一帧图像的时间戳
        // using namespace std::chrono_literals;
        // // std::this_thread::sleep_for(30ms);

        // double T=0;
        // if(ni<nImages-1)
        //     T = vTimeStamp[ni+1]-tframe;
        // else if(ni>0)
        //     T = tframe-vTimeStamp[ni-1];

        // if(ttrack<T)
        //     std::this_thread::sleep_for((T-ttrack)*1ns);
    }

    // Create callbacks according to the topics set in the parameters file

    // ROS Spin
    rclcpp::spin(node);

    // Request Slam Manager thread to exit
    slam.bexit_required_ = true;

    // Waiting end of SLAM Manager
    while( slam.bis_on_ ) {
        std::chrono::seconds dura(1);
        std::this_thread::sleep_for(dura);
    }

    return 0;
}

void LoadImages(const std::string &strPath, std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps)
{
    std::ifstream data;
    data.open(strPath + "/cam0/data_1.csv");
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);

    std::string line;
    std::string name;
    std::getline(data, line);
    while(std::getline(data, line))
    {
        std::istringstream ss(line);
        if(std::getline(ss, name, ',')){
            vstrImageLeft.push_back(strPath + "/cam0/data/" + name + ".png");
            vstrImageRight.push_back(strPath + "/cam1/data/" + name + ".png");

            std::stringstream ss(name);
            long t;
            ss >> t;
            vTimeStamps.push_back(t);
        }
    }
}