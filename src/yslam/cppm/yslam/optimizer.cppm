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
            // for (const auto& kp : frame->kps) {
            //     auto mp = kp.map_point.lock();
            //     // if (!mp || mp->is_outlier) {
            //     //     continue;
            //     // }
            //     cv::Point3d p3d = *kp.map_point.lock();
            //     points3d.push_back(p3d);
            //     points2f.push_back(kp.pt);
            // }
            std::erase_if(frame->kps, [&](auto& kp) {
                auto mp = kp.map_point.lock();
                if (!mp || mp->is_outlier) {
                    return true;
                }
                cv::Point3d p3d = *kp.map_point.lock();
                points3d.push_back(p3d);
                points2f.push_back(kp.pt);
                return false;
            });

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
                8, // reprojection error
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
    void BundleAdjustment(std::list<std::shared_ptr<KeyFrame>>& kfs)
    {
        // 为每个关键帧创建位姿参数块 (四元数+平移)
        std::unordered_map<int, std::vector<double>> pose_blocks;
        std::unordered_map<int, std::vector<double>> mp_blocks;
        for (auto& kf : kfs) {
            // 将T_cw转换为四元数表示
            cv::Mat T_cw = kf->T_wc.inv();
            cv::Mat R = T_cw(cv::Range(0, 3), cv::Range(0, 3));
            cv::Mat t = T_cw(cv::Range(0, 3), cv::Range(3, 4));

            // 将旋转矩阵转换为四元数
            cv::Quatd quat = cv::Quatd::createFromRotMat(R);

            quat = quat.normalize();
            // 填充参数 [w, x, y, z, tx, ty, tz]
            pose_blocks[kf->ID] = { quat.w, quat.x, quat.y, quat.z,
                t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0) };

            // 为每个地图点创建参数块
            for (const auto& kp : kf->kps) {
                auto map_point_ptr = kp.map_point.lock();
                if (map_point_ptr) {
                    if (map_point_ptr->is_outlier)
                        continue;
                    mp_blocks.insert_or_assign(map_point_ptr->ID, std::vector<double> { map_point_ptr->x, map_point_ptr->y, map_point_ptr->z });
                    // 将地图点的位置添加到参数块
                    // mp_blocks[map_point_ptr->ID] = { map_point_ptr->x, map_point_ptr->y, map_point_ptr->z };
                }
            }
        }

        std::unordered_map<int, ceres::CostFunction*> cost_functions;
        ceres::Problem problem;
        // 添加重投影误差项
        for (auto& kf : kfs) {
            for (const auto& kp : kf->kps) {
                auto mp = kp.map_point.lock();
                if (!mp || mp->is_outlier)
                    continue;

                auto cost_fun = ReprojectionError::Create(*mp, kp.pt, kf->K);
                // cost_fun.
                problem.AddResidualBlock(
                    cost_fun,
                    new ceres::HuberLoss(std::sqrt(7.815)),
                    pose_blocks[kf->ID].data(),
                    mp_blocks[mp->ID].data());
            }
        }

        // 设置第一个关键帧固定
        if (!pose_blocks.empty()) {
            auto first_kf_id = kfs.front()->ID;
            problem.SetParameterBlockConstant(pose_blocks[first_kf_id].data());
            // 对于固定帧，可以完全固定整个参数块，或者使用局部参数化
        }

        // 为其余关键帧设置四元数流形
        for (auto& kf : kfs) {
            if (kf->ID == kfs.front()->ID)
                continue;
            // 使用EigenQuaternionManifold配合SubsetManifold
            problem.SetManifold(pose_blocks[kf->ID].data(),
                new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>());
        }

        // 配置并运行求解器
        ceres::Solver::Options options;
        // options.linear_solver_type = ceres::DENSE_SCHUR;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.preconditioner_type = ceres::PreconditionerType::SCHUR_JACOBI;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 40;
        options.function_tolerance = 1e-6;
        options.gradient_tolerance = 1e-10;
        options.parameter_tolerance = 1e-8;
        options.num_threads = 4; // 根据CPU核心数量调整
        // options.num_linear_solver_threads = 4;

        ceres::Solver::Summary summary;
        // std::printf("Bundle Adjustment: \n");
        ceres::Solve(options, &problem, &summary);
        // std::printf("Bundle Adjustment Finished \n");

        // 更新优化后的位姿和地图点位置
        for (auto& kf : kfs) {
            std::vector<double>& pose_data = pose_blocks[kf->ID];

            cv::Quatd quat(pose_data[0], pose_data[1], pose_data[2], pose_data[3]);
            quat = quat.normalize();

            cv::Mat R = cv::Mat(quat.toRotMat3x3());
            cv::Mat t = cv::Mat::zeros(3, 1, cv::MAT_64F);
            t.at<double>(0, 0) = pose_data[4];
            t.at<double>(1, 0) = pose_data[5];
            t.at<double>(2, 0) = pose_data[6];

            cv::Mat T_cw = cv::Mat::eye(4, 4, cv::MAT_64F);
            R.copyTo(T_cw(cv::Rect(0, 0, 3, 3)));
            t.copyTo(T_cw(cv::Rect(3, 0, 1, 3)));

            kf->T_wc = T_cw.inv();
        }
        // std::printf("Updated poses...\n");

        for (auto& kf : kfs) {
            for (const auto& kp : kf->kps) {
                auto mp = kp.map_point.lock();
                if (mp && !mp->is_outlier) {
                    const std::vector<double>& pos = mp_blocks[mp->ID];
                    mp->x = pos[0];
                    mp->y = pos[1];
                    mp->z = pos[2];
                }
            }
        }
        // std::printf("Updated map points...\n");

        // ===== 新增：计算每个地图点的重投影误差，并根据阈值标记为外点 =====
        {
            const double reproj_threshold_pixels = 5.891; // 可调整阈值（像素）
            for (auto& kf : kfs) {
                // 使用更新后的位姿和相机内参
                cv::Mat T_cw = kf->T_wc.inv();
                cv::Mat R = T_cw(cv::Range(0, 3), cv::Range(0, 3));
                cv::Mat t = T_cw(cv::Range(0, 3), cv::Range(3, 4));

                double fx = kf->K.at<double>(0, 0);
                double fy = kf->K.at<double>(1, 1);
                double cx = kf->K.at<double>(0, 2);
                double cy = kf->K.at<double>(1, 2);

                for (const auto& kp : kf->kps) {
                    auto mp = kp.map_point.lock();
                    if (!mp || mp->is_outlier)
                        continue;

                    // 使用最新地图点坐标
                    cv::Mat pw = cv::Mat::zeros(3, 1, cv::MAT_64F);
                    pw.at<double>(0, 0) = mp->x;
                    pw.at<double>(1, 0) = mp->y;
                    pw.at<double>(2, 0) = mp->z;
                    cv::Mat pc;
                    cv::add(cv::MatMul(R, pw), t, pc);
                    double X = pc.at<double>(0, 0);
                    double Y = pc.at<double>(1, 0);
                    double Z = pc.at<double>(2, 0);

                    double reproj_err = 1e6;
                    if (Z > 1e-6) {
                        double u = fx * (X / Z) + cx;
                        double v = fy * (Y / Z) + cy;
                        double dx = u - kp.pt.x;
                        double dy = v - kp.pt.y;
                        reproj_err = std::sqrt(dx * dx + dy * dy);
                    } else {
                        // 点在相机后方或深度无效，视为大误差
                        reproj_err = reproj_threshold_pixels * 10.0;
                    }

                    // 输出每个地图点重投影误差
                    // std::printf("KF %d MP %d reproj_err = %.3f px\n", kf->ID, mp->ID, reproj_err);

                    // 根据阈值标记为外点
                    if (reproj_err > reproj_threshold_pixels) {
                        mp->is_outlier = true;
                    } else {
                        mp->is_outlier = false;
                    }
                }
            }
            // std::printf("Marked outliers by reprojection error (threshold = %.2f px)\n", reproj_threshold_pixels);
        }
        // ===== 新增结束 =====
    }
};

// Yslam
}