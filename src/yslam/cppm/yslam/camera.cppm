export module camera;
import cv;
import types;
import std;
import convert;

export namespace Yslam {

class Camera {
public:
    struct TrackParams {
        const cv::Mat& img0;
        const cv::Mat& img1;
        std::vector<KeyPoint>& kps0;
        std::vector<KeyPoint>& kps1;
    };
    virtual void Init(std::shared_ptr<Frame> frame) = 0;
    virtual void Extract3d(std::shared_ptr<Frame> frame) = 0;
    virtual void Track(TrackParams params) = 0;
};

class Stereo : public Camera {
public:
    void Init(std::shared_ptr<Frame> frame) override
    {
        auto& kps = frame->kps;
        auto& img0 = frame->img0;
        auto& last_frame = frame->last;

        // auto ex = cv::ORB::create();
        // std::vector<cv::KeyPoint> kps_ex;
        // ex->detect(img0, kps_ex);
        // for (const auto& kp : kps_ex) {
        //     KeyPoint keypoint(kp);
        //     kps.emplace_back(keypoint);
        // }
        Extract3d(frame);
        last_frame = frame;
        // Initialize stereo camera with the given frame
    }

    void Extract3d(std::shared_ptr<Frame> frame) override
    {
        auto& kps = frame->kps;
        auto& img0 = frame->img0;
        auto& img1 = frame->img1;
        // 屏蔽已有特征点的区域
        cv::Mat mask(img0.size(), cv::MAT_8UC1, cv::Scalar::all(255));
        for (const auto& kp : kps) {
            cv::circle(mask, kp.pt, 10, cv::Scalar(0), cv::FILLED);
        }

        std::vector<cv::KeyPoint> kps1;
        auto ex = cv::ORB::create(1000);
        ex->detect(img0, kps1, mask);

        std::vector<KeyPoint> new_kps0, new_kps1;
        for (const auto& kp : kps1) {
            KeyPoint keypoint(kp);
            new_kps0.emplace_back(keypoint);
        }
        // 使用LK光流法在右图中寻找对应点
        Track({ img0, img1, new_kps0, new_kps1 });
        std::vector<cv::Point2f> pts1_cam, pts2_cam;
        for (int i = 0; i < new_kps0.size(); i++) {
            pts1_cam.push_back(frame->Pixel2Camera(new_kps0[i].pt));
            pts2_cam.push_back(frame->Pixel2Camera(new_kps1[i].pt));
        }
        // 构建投影矩阵：基于当前帧位姿
        // 第一个相机的投影矩阵：从frame->pose提取前3行
        // 第一个相机的投影矩阵：3x4单位矩阵
        static cv::Mat P1 = cv::Mat::eye(3, 4, cv::MAT_32F);

        // 第二个相机的投影矩阵：右目相对左目的变换
        static cv::Mat P2 = frame->T_01.inv()(cv::Range(0, 3), cv::Range::all());
        // 三角化
        cv::Mat points4d;
        cv::triangulatePoints(P1, P2, pts1_cam, pts2_cam, points4d);
        // 转换成非齐次坐标
        for (int i = 0; i < points4d.cols; i++) {
            double w = points4d.at<float>(3, i);
            cv::Point3d p3d(
                points4d.at<float>(0, i) / w,
                points4d.at<float>(1, i) / w,
                points4d.at<float>(2, i) / w);
            // // 检查深度是否在合理范围内 (0.1米到100米)
            if (p3d.z <= 0.1 || p3d.z > 100) {
                continue;
            }
            // 只有当3D点有效时，才保存对应的2D点
            // cv::Mat p3d_mat = (cv::Mat_<double>(4, 1) << p3d.x, p3d.y, p3d.z, 1);
            cv::Vec4d p3d_vec(p3d.x, p3d.y, p3d.z, 1.0);
            cv::Mat p3d_mat = cv::Mat(p3d_vec).reshape(1, 4); // 转换为4x1矩阵

            cv::Mat p3d_world_mat = cv::MatMul(frame->T_wc, p3d_mat);
            cv::Point3d p3d_world(p3d_world_mat.at<double>(0), p3d_world_mat.at<double>(1), p3d_world_mat.at<double>(2));

            auto map_point = std::make_shared<MapPoint>(p3d_world, MapPoint::factory_id++);
            new_kps0[i].map_point = map_point;
            kps.emplace_back(new_kps0[i]);
            Map::GetInstance().InsertMapPoint(map_point);

            // std::println("3D Point: ({}, {}, {})", p3d_world.x, p3d_world.y, p3d_world.z);
        }
    }

    void Track(TrackParams params) override
    {
        auto& [img0, img1, kps0, kps1] = params;

        std::vector<cv::Point2f> pts0, pts1;
        for (const auto& kp : kps0) {
            pts0.emplace_back(kp.pt);
        }

        std::vector<unsigned char> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(
            img0,
            img1,
            pts0,
            pts1,
            status,
            err,
            cv::Size(21, 21),
            7);

        // Filter points based on status
        for (int i = 0; i < status.size(); ++i) {
            if (status[i]) {
                KeyPoint kp0, kp1;
                kp1.pt = pts1[i];
                kp1.map_point = kps0[i].map_point; // 保持3D点关联
                kps1.emplace_back(kp1);
            }
        }
    }
};

}; // namespace Yslam