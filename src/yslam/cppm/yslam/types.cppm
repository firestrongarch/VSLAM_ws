module;
#include <opencv2/core/mat.hpp>

export module types;

import cv;
import std;
import lockfree;

export namespace Yslam {

class Frame;
class MapPoint : public cv::Point3d {
public:
    MapPoint() = delete;
    MapPoint(const cv::Point3d& p, const int& id)
        : cv::Point3d(p)
        , ID(id)
    {
    }

    const int ID;
    inline static int factory_id = 0;
    bool is_outlier = false;
    std::list<std::weak_ptr<Frame>> obs; // Observing keyframes
};

class KeyPoint : public cv::KeyPoint {
public:
    using Ptr = std::shared_ptr<KeyPoint>;
    KeyPoint() = default;
    KeyPoint(const cv::KeyPoint& kp)
        : cv::KeyPoint(kp)
    {
    }

    std::weak_ptr<MapPoint> map_point;
    cv::Mat des;
    cv::Point2f match;

    bool is_outlier = false;
};

struct Frame {
    const int ID;
    inline static int factory_id_ = 0;

    double timestamp; // Timestamp in microseconds
    cv::Mat img0; // Left image
    cv::Mat img1; // Right image

    std::vector<KeyPoint> kps; // Extracted KeyPoints

    inline static cv::Mat K = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928,
        0, 718.856, 185.2157,
        0, 0, 1); // Camera intrinsic matrix
    inline static std::shared_ptr<Frame> last;

    cv::Mat T_wc = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat T_ww = cv::Mat::eye(4, 4, CV_64F);
    inline static cv::Mat T_01 = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0.537166,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1); // camera 1 to camera 0 pose

    cv::Point2d Pixel2Camera(const cv::Point2d& p2d)
    {
        return cv::Point2d(
            (p2d.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p2d.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
    }

    cv::Point2d Camera2Pixel(const cv::Point3d& p3d)
    {
        // cv::Mat p3d_mat = (cv::Mat_<double>(3, 1) << p3d.x/p3d.z, p3d.y/p3d.z, 1);
        // cv::Mat p3d_pixel = K * p3d_mat;
        // return cv::Point2d(
        //     p3d_pixel.at<double>(0),
        //     p3d_pixel.at<double>(1)
        // );
        return cv::Point2d(
            K.at<double>(0, 0) * p3d.x / p3d.z + K.at<double>(0, 2),
            K.at<double>(1, 1) * p3d.y / p3d.z + K.at<double>(1, 2));
    }

    cv::Point2d World2Pixel(const cv::Point3d& p3d)
    {
        cv::Mat p3d_mat = (cv::Mat_<double>(4, 1) << p3d.x, p3d.y, p3d.z, 1);
        cv::Mat P = T_wc.inv()(cv::Range(0, 3), cv::Range::all());
        cv::Mat p2d_mat = P * p3d_mat;
        cv::Mat p3d_cam = p2d_mat / p2d_mat.at<double>(2);
        cv::Mat p2d_pixel = K * p3d_cam;
        return cv::Point2d(p2d_pixel.at<double>(0) / p2d_pixel.at<double>(2), p2d_pixel.at<double>(1) / p2d_pixel.at<double>(2));
    }

    cv::Point3d Pixel2World(const cv::Point2d& p2d)
    {
        cv::Mat p2d_mat = (cv::Mat_<double>(3, 1) << p2d.x, p2d.y, 1);
        cv::Mat P = T_wc(cv::Range(0, 3), cv::Range::all());
        cv::Mat p3d_mat = P * K * p2d_mat;
        return cv::Point3d(p3d_mat.at<double>(0) / p3d_mat.at<double>(3), p3d_mat.at<double>(1) / p3d_mat.at<double>(3), p3d_mat.at<double>(2) / p3d_mat.at<double>(3));
    }
};

struct Pose {
    Pose() = default;
    Pose(const cv::Quatd& q, const cv::Point3d& t)
        : q(q)
        , t(t)
    {
    }

    cv::Quatd q; // rotation
    cv::Point3d t; // translation
};

class KeyFrame : public Frame {
public:
    KeyFrame() = delete;
    KeyFrame(const Frame& frame)
        : Frame(frame)
        , KF_ID(factory_kf_id++)
    {
        pose.q = cv::Quatd::createFromRotMat(frame.T_wc(cv::Range(0, 3), cv::Range(0, 3)));
        pose.t = cv::Point3d(
            frame.T_wc.at<double>(0, 3),
            frame.T_wc.at<double>(1, 3),
            frame.T_wc.at<double>(2, 3));
    }

    Pose pose; // Pose of the keyframe

    const int KF_ID;
    inline static int factory_kf_id = 0;
};

class Map {
public:
    using KeyFrames = std::unordered_map<unsigned long, std::shared_ptr<KeyFrame>>;
    using MapPoints = std::unordered_map<unsigned long, std::shared_ptr<MapPoint>>;

    Map(const Map&) = delete;

    void InsertKeyFrame(std::shared_ptr<Frame> frame)
    {
        ref_kf_ = frame;
        auto key_frame = std::make_shared<KeyFrame>(*frame);
        kf_queue_.push(key_frame);
        all_key_frames_.insert({ key_frame->KF_ID, key_frame });
    }
    void InsertMapPoint(std::shared_ptr<MapPoint> map_point)
    {
        if (all_map_points_.find(map_point->ID) == all_map_points_.end()) {
            all_map_points_.insert(make_pair(map_point->ID, map_point));
        }
    }
    void InsertPoseVO(const cv::Mat& pose)
    {
        poses_vo_.push_back(pose);
    }
    std::vector<cv::Mat> GetAllPosesVO()
    {
        return poses_vo_;
    }

    void RemoveOutliers()
    {
        for (auto& [id, map_point] : all_map_points_) {
            if (map_point->is_outlier) {
                all_map_points_.erase(id);
            }
        }
    }

    // MapPoints GetAllMapPoints();
    // KeyFrames GetAllKeyFrames();

    static auto& GetInstance()
    {
        static Map instance;
        return instance;
    }
    // std::shared_ptr<Frame> current_keyframe_ { nullptr };

    boost::lockfree::spsc_queue<std::shared_ptr<Frame>> frame_queue_ { 5000 };
    boost::lockfree::spsc_queue<std::shared_ptr<KeyFrame>> kf_queue_ { 5000 };

    std::shared_ptr<Frame> ref_kf_ { nullptr };

    KeyFrames all_key_frames_;
    MapPoints all_map_points_;
    MapPoints active_map_point_;

    KeyFrames active_key_frames_;

private:
    Map() = default;

    std::vector<cv::Mat> poses_vo_;
    unsigned int num_active_key_frames_;
};

} // namespace Yslam