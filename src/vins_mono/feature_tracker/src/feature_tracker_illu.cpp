#include <iostream>
#include <locale>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "feature_tracker.h"

// c++ 14 thread
#include <thread>
#include <fstream>
#include <queue>

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::SharedPtr> img_buf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match,pub_raw;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;


void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
        last_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
        return;
    }
    // detect unstable camera stream
    if (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - last_image_time > 1.0 || img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) < last_image_time)
    {
        RCUTILS_LOG_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
    // frequency control
    if (round(1.0 * pub_count / (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
#ifdef LET_NET
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
#else
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
#endif
    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCUTILS_LOG_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9));
        else
        {
#ifdef LET_NET
            trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
#else
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
#endif
        }


#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        // std::cout<<"pub points\n";
        pub_count++;
        sensor_msgs::msg::PointCloud feature_points;
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points.header = img_msg->header;
        feature_points.header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;

            // std::cout<<ids.size()<<"- ids.size()\n";
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                // std::cout<<trackerData[i].track_cnt[j]<<"\n";
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    // std::cout<<p.x<<"- x\n";

                    feature_points.points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points.channels.push_back(id_of_point);
        feature_points.channels.push_back(u_of_point);
        feature_points.channels.push_back(v_of_point);
        feature_points.channels.push_back(velocity_x_of_point);
        feature_points.channels.push_back(velocity_y_of_point);
        RCUTILS_LOG_DEBUG("publish %f, at %f", feature_points.header.stamp.sec+feature_points.header.stamp.nanosec * (1e-9), rclcpp::Clock().now().nanoseconds()*(1e-9));
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else{
            pub_img->publish(feature_points);
            // std::cout<<"x:\n";
            // std::cout<<"x:"<<feature_points->points[0].x<<"\n";
        }
            

        if (SHOW_TRACK)
        {
#ifndef LET_NET
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
#endif
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
#ifndef LET_NET
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);
#endif

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match->publish(*(ptr->toImageMsg()));
        }
    }
    // RCUTILS_LOG_INFO("whole feature tracker processing costs: %fms", t_r.toc());
}

void readImage(rclcpp::Node::SharedPtr n) {
    const std::string path = "/home/fu/datasets/lab-module-csc/";
    const std::string timestamp_path = path + "cam0/data.csv";
    const std::string imu_path = path + "imu0/data.csv";
    // read timestamp
    std::ifstream timestamp_file(timestamp_path);
    std::string line;
    std::vector<long double> timestamp;
    std::vector<std::string> img_name;
    std::getline(timestamp_file, line); // skip first line
    while (std::getline(timestamp_file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::getline(iss, token, ',');
        // string to long double
        timestamp.push_back(std::stod(token));
        std::getline(iss, token, ',');
        img_name.push_back(token);
    }
    long double start_time = timestamp[0];
    for (auto &t : timestamp) {
        t -= start_time;
    }

    // read imu
    std::ifstream imu_file(imu_path);
    std::vector<sensor_msgs::msg::Imu> imu_data;
    std::vector<long double> imu_timestamp;
    std::getline(imu_file, line); // skip first line
    while (std::getline(imu_file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::getline(iss, token, ',');
        // string to long double
        long double t = std::stod(token);
        std::getline(iss, token, ',');
        double wx = std::stod(token);
        std::getline(iss, token, ',');
        double wy = std::stod(token);
        std::getline(iss, token, ',');
        double wz = std::stod(token);
        std::getline(iss, token, ',');
        double ax = std::stod(token);
        std::getline(iss, token, ',');
        double ay = std::stod(token);
        std::getline(iss, token, ',');
        double az = std::stod(token);
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = n->now();
        imu_msg.angular_velocity.x = wx;
        imu_msg.angular_velocity.y = wy;
        imu_msg.angular_velocity.z = wz;
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        imu_data.push_back(imu_msg);
        imu_timestamp.push_back(t);
    }
    for (auto &t : imu_timestamp) {
        t -= start_time;
    }
    // read image and publish
    // publish first image
    auto start = n->now();
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = start;
    img_msg.header.frame_id = "world";
    img_msg.height = ROW;
    img_msg.width = COL;
#ifdef LET_NET
    img_msg.encoding = "bgr8";
#else
    img_msg.encoding = "mono8";
#endif
    img_msg.is_bigendian = false;
    img_msg.step = COL;
#ifdef LET_NET
    img_msg.data.resize(ROW * COL * 3);
    cv::Mat img = cv::imread(path +"cam0/data/"+ img_name[0]);
    cv::resize(img, img, cv::Size(COL, ROW));
    sensor_msgs::msg::Image::SharedPtr img_ptr = cv_bridge::CvImage(img_msg.header, "bgr8", img).toImageMsg();
#else
    img_msg.data.resize(ROW * COL);
    cv::Mat img = cv::imread(path +"cam0/data/"+ img_name[0], 0);
    cv::resize(img, img, cv::Size(COL, ROW));
    memcpy(&img_msg.data[0], img.data, ROW * COL);
    sensor_msgs::msg::Image::SharedPtr img_ptr = boost::make_shared<sensor_msgs::msg::Image>(img_msg);
#endif

    pub_raw->publish(*img_ptr);
    img_callback(img_ptr);

    decltype(imu_timestamp.size()) pub_index = 1;
    decltype(imu_timestamp.size()) imu_index = 0;
    while (true) {
        auto now = n->now();
        auto d = now - start;
        // d to ns
        long double d_ns = d.seconds() * 1e9 + d.nanoseconds();
        if (d_ns < imu_timestamp[imu_index]) {
            continue;
        }else {
            imu_data[imu_index].header.stamp = start + rclcpp::Duration::from_nanoseconds((int64)imu_timestamp[imu_index]);
            pub_imu->publish(imu_data[imu_index]);
            imu_index++;
            if (imu_index == imu_timestamp.size()) {
                break;
            }
        }
        if (d_ns < timestamp[pub_index]) {
            continue;
        }else {
            //
            // ros::Duration img_time;
            img_msg.header.stamp = start + rclcpp::Duration::from_nanoseconds((int64)timestamp[pub_index]);
#ifdef LET_NET
            img = cv::imread(path +"cam0/data/"+ img_name[pub_index], cv::IMREAD_COLOR);
            cv::resize(img, img, cv::Size(COL, ROW));
            img_ptr = cv_bridge::CvImage(img_msg.header, "bgr8", img).toImageMsg();
#else
            img = cv::imread(path +"cam0/data/"+ img_name[pub_index], 0);
            cv::resize(img, img, cv::Size(COL, ROW));
            memcpy(&img_msg.data[0], img.data, ROW * COL);
            img_ptr = boost::make_shared<sensor_msgs::Image>(img_msg);
#endif
            img_callback(img_ptr);
            sensor_msgs::msg::Image img_msg_gray;
            img_msg_gray.header.stamp = start + rclcpp::Duration::from_nanoseconds((int64)timestamp[pub_index]);
            img_msg_gray.header.frame_id = "world";
            img_msg_gray.height = ROW;
            img_msg_gray.width = COL;
            img_msg_gray.encoding = "mono8";
            cv::Mat img_gray = cv::imread(path +"cam0/data/"+ img_name[pub_index], 0);
            cv::resize(img_gray, img_gray, cv::Size(COL, ROW));
            sensor_msgs::msg::Image::SharedPtr img_ptr_gray = cv_bridge::CvImage(img_msg.header, "mono8", img_gray).toImageMsg();
            pub_raw->publish(*img_ptr_gray);
            pub_index++;
            if (pub_index == timestamp.size()) {
                break;
            }
            // std::cout<< "pub_index: " << pub_index << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("feature_tracker");

    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                RCUTILS_LOG_INFO("load mask fail");
            }
            else
                RCUTILS_LOG_INFO("load mask success");
        }
    }

    // auto sub_img = n->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img_callback);
    
    pub_img = n->create_publisher<sensor_msgs::msg::PointCloud>("feature", 1000);
    pub_raw = n->create_publisher<sensor_msgs::msg::Image>(IMAGE_TOPIC, 2000);
    pub_match = n->create_publisher<sensor_msgs::msg::Image>("feature_img",1000);
    pub_restart = n->create_publisher<std_msgs::msg::Bool>("restart",1000);
    pub_imu = n->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, 2000);

    RCUTILS_LOG_INFO("readimage thread launch");
    // new thread for read image
    std::cout<<"n->now().seconds()\n";
    std::cout<<n->now().seconds()<<"\n";
    std::thread read_img_thread(readImage,n);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    rclcpp::spin(n);
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
