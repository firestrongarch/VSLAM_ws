module;

#include <opencv2/core/mat.hpp>

export module optimizer;
import types;
import cv;
import std;
export namespace Yslam {

class Optimizer {
public:
    void PerspectiveNPoint(std::shared_ptr<Frame> frame)
    {
        frame->T_wc = frame->T_ww * Frame::last->T_wc;
        cv::Mat T_cw = frame->T_wc.inv();
        cv::Mat R = T_cw(cv::Range(0, 3), cv::Range(0, 3));

        for (int i = 0; i < 4; i++) {
            cv::Mat tvec = T_cw(cv::Range(0, 3), cv::Range(3, 4));
            cv::Mat rvec;
            cv::Rodrigues(R, rvec); // 将旋转矩阵转换为旋转向量
            std::vector<int> inliers;
            std::vector<cv::Point3d> points3d;
            std::vector<cv::Point2f> points2f;
            for (const auto& kp : frame->kps) {
                if (!kp.map_point.lock()) {
                    throw std::runtime_error("Map point not found");
                }
                cv::Point3d p3d = *kp.map_point.lock();
                points3d.push_back(p3d);
                points2f.push_back(kp.pt);
            }

            // std::println("PnP iteration {}: {} points", i, points3d.size());
            cv::solvePnPRansac(
                points3d,
                points2f,
                frame->K,
                cv::Mat(),
                rvec,
                tvec,
                true,
                100, // max iterations
                8.0, // reprojection error
                0.99, // confidence
                inliers);
            // 根据inliers筛选特征点
            std::vector<KeyPoint> filtered_kps;
            for (auto idx : inliers) {
                filtered_kps.push_back(frame->kps[idx]);
            }
            frame->kps = filtered_kps;

            cv::Rodrigues(rvec, R);
            cv::Mat T_cw_new = cv::Mat::eye(4, 4, cv::MAT_64F);
            R.copyTo(T_cw_new(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(T_cw_new(cv::Rect(3, 0, 1, 3)));
            frame->T_wc = T_cw_new.inv();
        }
        Map::GetInstance().InsertPoseVO(frame->T_wc);

        frame->T_ww = frame->T_wc * Frame::last->T_wc.inv();
    }
};

}