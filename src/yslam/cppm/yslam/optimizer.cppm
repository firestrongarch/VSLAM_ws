export module optimizer;
import types;
import cv;
import std;
import ceres;
export namespace Yslam {

// 使用四元数的Ceres重投影误差因子
struct ReprojectionError {
    ReprojectionError(cv::Point3d observed_point, cv::Point2f observed_pixel, cv::Mat K)
        : observed_point_(observed_point)
        , observed_pixel_(observed_pixel)
        , K_(K)
    {
    }

    template <typename T>
    bool operator()(const T* const camera_pose,
        const T* const point,
        T* residuals) const
    {
        // 相机姿态参数: [quaternion(4), translation(3)]
        // quaternion: [w, x, y, z]

        // 使用四元数旋转点
        T p[3];
        ceres::QuaternionRotatePoint(camera_pose, point, p);

        // 平移
        p[0] += camera_pose[4];
        p[1] += camera_pose[5];
        p[2] += camera_pose[6];

        // 投影到图像平面
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        // 应用相机内参
        T predicted_x = T(K_.at<double>(0, 0)) * xp + T(K_.at<double>(0, 2));
        T predicted_y = T(K_.at<double>(1, 1)) * yp + T(K_.at<double>(1, 2));

        // 计算重投影误差
        residuals[0] = predicted_x - T(observed_pixel_.x);
        residuals[1] = predicted_y - T(observed_pixel_.y);

        return true;
    }

    static ceres::CostFunction* Create(const cv::Point3d& observed_point,
        const cv::Point2f& observed_pixel,
        const cv::Mat& K)
    {
        // 参数维度: 2维残差, 7维相机姿态(4维四元数+3维平移), 3维点坐标
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 3>(
            new ReprojectionError(observed_point, observed_pixel, K)));
    }

    cv::Point3d observed_point_;
    cv::Point2f observed_pixel_;
    cv::Mat K_;
};

class Optimizer {
public:
    void PerspectiveNPoint(std::shared_ptr<Frame> frame)
    {
        // frame->T_wc = frame->T_ww * Frame::last->T_wc;
        frame->T_wc = cv::MatMul(frame->T_ww, Frame::last->T_wc);
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
                10, // max iterations
                7.815, // reprojection error
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

        frame->T_ww = cv::MatMul(frame->T_wc, Frame::last->T_wc.inv());
    }

    // 使用四元数的Bundle Adjustment方法
    void BundleAdjustment(std::vector<std::shared_ptr<Frame>>& keyframes)
    {
        ceres::Problem problem;

        // 获取地图中所有的地图点
        auto& map_instance = Map::GetInstance();
        auto all_map_points_map = map_instance.GetAllMapPoints();

        // 转换为向量以便索引访问，使用MapPoint的ID作为索引
        std::vector<std::shared_ptr<MapPoint>> map_points;
        int max_id = 0;

        // 先找到最大的ID以确定vector大小
        for (const auto& pair : all_map_points_map) {
            if (pair.second->ID > max_id) {
                max_id = pair.second->ID;
            }
        }

        // 调整vector大小
        map_points.resize(max_id + 1);

        // 填充vector，使用MapPoint的ID作为索引
        for (const auto& pair : all_map_points_map) {
            map_points[pair.second->ID] = pair.second;
        }

        // 为每个关键帧创建位姿参数块 (四元数+平移)
        std::vector<std::vector<double>> camera_poses(keyframes.size(), std::vector<double>(7, 0.0));
        for (std::size_t i = 0; i < keyframes.size(); ++i) {
            auto& frame = keyframes[i];
            // 将T_cw转换为四元数表示
            cv::Mat T_cw = frame->T_wc.inv();
            cv::Mat R = T_cw(cv::Range(0, 3), cv::Range(0, 3));
            cv::Mat t = T_cw(cv::Range(0, 3), cv::Range(3, 4));

            // 将旋转矩阵转换为四元数
            cv::Quatd quat = cv::Quatd::createFromRotMat(R);

            // 填充参数 [w, x, y, z, tx, ty, tz]
            camera_poses[i][0] = quat.w;
            camera_poses[i][1] = quat.x;
            camera_poses[i][2] = quat.y;
            camera_poses[i][3] = quat.z;

            camera_poses[i][4] = t.at<double>(0, 0);
            camera_poses[i][5] = t.at<double>(1, 0);
            camera_poses[i][6] = t.at<double>(2, 0);
        }

        // 为每个地图点创建参数块
        std::vector<std::vector<double>> point_positions(map_points.size(), std::vector<double>(3, 0.0));
        for (std::size_t i = 0; i < map_points.size(); ++i) {
            if (map_points[i]) { // 检查指针是否有效
                auto& point = map_points[i];
                point_positions[i][0] = point->x;
                point_positions[i][1] = point->y;
                point_positions[i][2] = point->z;
            }
        }

        // 添加重投影误差项
        for (std::size_t i = 0; i < keyframes.size(); ++i) {
            auto& frame = keyframes[i];
            for (const auto& kp : frame->kps) {
                auto map_point_ptr = kp.map_point.lock();
                if (!map_point_ptr)
                    continue;

                // 使用MapPoint的ID作为索引
                int point_id = map_point_ptr->ID;

                // 检查ID是否在有效范围内并且指针有效
                if (point_id >= 0 && point_id < static_cast<int>(map_points.size()) && map_points[point_id]) {
                    // 创建重投影误差成本函数
                    ceres::CostFunction* cost_function = ReprojectionError::Create(*map_point_ptr, kp.pt, frame->K);

                    // 添加残差块
                    problem.AddResidualBlock(cost_function,
                        new ceres::HuberLoss(1.0), // 使用Huber损失函数提高鲁棒性
                        camera_poses[i].data(),
                        point_positions[point_id].data());
                }
            }
        }

        // 固定第一个关键帧的位姿（作为参考坐标系）
        if (!camera_poses.empty()) {
            problem.SetParameterBlockConstant(camera_poses[0].data());
            problem.SetManifold(camera_poses[0].data(), new ceres::QuaternionManifold);
        }

        // 为所有其他关键帧添加四元数流形
        for (std::size_t i = 1; i < camera_poses.size(); ++i) {
            problem.SetManifold(camera_poses[i].data(), new ceres::QuaternionManifold);
        }

        // 配置求解器选项
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 50;

        // 求解优化问题
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // 更新优化后的位姿和地图点位置
        for (std::size_t i = 0; i < keyframes.size(); ++i) {
            auto& frame = keyframes[i];
            // 将优化后的四元数转换回变换矩阵
            cv::Quatd quat(
                camera_poses[i][0], // w
                camera_poses[i][1], // x
                camera_poses[i][2], // y
                camera_poses[i][3] // z
            );

            // 归一化四元数
            quat = quat.normalize();

            cv::Mat R = cv::Mat(quat.toRotMat3x3());
            cv::Mat t(3, 1, cv::MAT_64F);
            t.at<double>(0, 0) = camera_poses[i][4];
            t.at<double>(1, 0) = camera_poses[i][5];
            t.at<double>(2, 0) = camera_poses[i][6];

            cv::Mat T_cw = cv::Mat::eye(4, 4, cv::MAT_64F);
            R.copyTo(T_cw(cv::Rect(0, 0, 3, 3)));
            t.copyTo(T_cw(cv::Rect(3, 0, 1, 3)));

            frame->T_wc = T_cw.inv();
        }

        for (std::size_t i = 0; i < map_points.size(); ++i) {
            if (map_points[i]) { // 检查指针是否有效
                auto& point = map_points[i];
                point->x = point_positions[i][0];
                point->y = point_positions[i][1];
                point->z = point_positions[i][2];
            }
        }
    }
};

}