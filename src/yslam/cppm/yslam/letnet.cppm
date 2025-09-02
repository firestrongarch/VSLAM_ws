export module letnet;
export import cv;
export import std;
import ncnn;
export namespace Yslam {

class LetNet : public cv::Feature2D {
public:
    static cv::Ptr<LetNet> create(const std::string& model_path)
    {
        cv::Ptr<LetNet> instance(new LetNet());
        std::filesystem::path p(model_path);
        instance->net.load_param((p / "model.param").string().c_str());
        instance->net.load_model((p / "model.bin").string().c_str());
        return instance;
    }
    /**
     * @brief 从给定的得分图中提取特征点，使用非极大值抑制策略，并避免与已有关键点过于接近。
     *
     * 该函数将图像划分为多个网格单元(cell)，在每个未被占用的单元中寻找局部最大值作为候选特征点。
     * 若主检测不足，则使用次优检测进行补充，以尽量填满所有网格单元。
     *
     * @param[in] score         输入的得分图（单通道浮点型矩阵），用于检测特征点。
     * @param[in] ncellsize     每个网格单元的边长（像素）。
     * @param[in] cur_kps       当前已有的关键点集合，用于标记已被占用的网格单元。
     * @param[out] out_kps      输出的关键点集合（注意：此参数未在函数中使用，可能为预留参数）。
     *
     * @note 该函数没有返回值，但会修改传入的 out_kps（尽管当前实现中未体现）。
     */
    auto extractFeature(
        const cv::Mat& score,
        int ncellsize,
        const std::vector<cv::KeyPoint>& cur_kps,
        std::vector<cv::KeyPoint>& out_kps)
    {
        if (score.empty()) {
            return;
        }

        // 获取图像尺寸
        auto ncols = score.cols;
        auto nrows = score.rows;

        // 计算半格子大小，用于绘制遮蔽区域
        auto nhalfcell = ncellsize / 4;

        // 计算网格行列数和总网格数
        auto nhcells = nrows / ncellsize;
        auto nwcells = ncols / ncellsize;
        auto nbcells = nhcells * nwcells;

        // 存储检测到的特征点
        std::vector<cv::Point2f> detected_pts;
        detected_pts.reserve(nbcells);

        // 标记已被占用的网格单元
        std::vector<std::vector<bool>> occupied_cells(
            nhcells + 1,
            std::vector<bool>(nwcells + 1, false));

        // 创建遮蔽掩码，防止重复检测
        cv::Mat mask = cv::Mat::ones(score.rows, score.cols, cv::MAT_8UC1);

        // 遍历已有关键点，标记其所在网格为已占用，并在掩码中画圆遮蔽
        for (const auto& px : cur_kps) {
            occupied_cells[px.pt.y / ncellsize][px.pt.x / ncellsize] = true;
            cv::circle(mask, px.pt, nhalfcell, cv::Scalar(0.), -1);
        }

        auto nboccup = 0;

        // 主检测点和次级检测点存储容器
        std::vector<std::vector<cv::Point2f>> detected_cells_pts(nbcells);
        std::vector<std::vector<cv::Point2f>> secondary_detections_pts(nbcells);

        auto range = cv::Range(0, nbcells);

        // 遍历每一个网格单元
        for (int i = 0; i < nbcells; i++) {
            auto r = std::floor(i / nwcells);
            auto c = i % nwcells;

            // 如果该网格已被占用，跳过
            if (occupied_cells[r][c]) {
                nboccup++;
                continue;
            }

            auto x = c * ncellsize;
            auto y = r * ncellsize;

            cv::Rect hroi(x, y, ncellsize, ncellsize);

            // 确保ROI在图像范围内
            if (x + ncellsize < ncols - 1 && y + ncellsize < nrows - 1) {
                double dminval, dmaxval;
                cv::Point minpx, maxpx;

                // 第一次查找局部最大值
                cv::minMaxLoc(score(hroi).mul(mask(hroi)), &dminval, &dmaxval, &minpx, &maxpx);
                maxpx.x += x;
                maxpx.y += y;

                // 如果最大值大于阈值，则记录为主检测点
                if (dmaxval >= 0.2) {
                    detected_cells_pts.at(i).push_back(maxpx);
                    cv::circle(mask, maxpx, nhalfcell, cv::Scalar(0.), -1);
                }

                // 第二次再次查找局部最大值（用于次优检测）
                cv::minMaxLoc(score(hroi).mul(mask(hroi)), &dminval, &dmaxval, &minpx, &maxpx);
                maxpx.x += x;
                maxpx.y += y;

                // 如果最大值大于阈值，则记录为次级检测点
                if (dmaxval >= 0.2) {
                    secondary_detections_pts.at(i).push_back(maxpx);
                    cv::circle(mask, maxpx, nhalfcell, cv::Scalar(0.), -1);
                }
            }
        }

        // 将主检测点加入最终结果
        for (const auto& vpx : detected_cells_pts) {
            if (!vpx.empty()) {
                detected_pts.insert(detected_pts.end(), vpx.begin(), vpx.end());
            }
        }

        auto nbkps = detected_pts.size();

        // 如果检测点数量不足，用次级检测点进行补充
        if (nbkps + nboccup < nbcells) {
            auto nbsec = nbcells - nbkps - nboccup;
            auto k = 0;
            for (const auto& vseckp : secondary_detections_pts) {
                if (!vseckp.empty()) {
                    detected_pts.push_back(vseckp.back());
                    k++;
                    if (k == nbsec) {
                        break;
                    }
                }
            }
        }

        // 将检测到的特征点转换为cv::KeyPoint格式
        for (const auto& pt : detected_pts) {
            out_kps.emplace_back(pt, ncellsize / 2.f, -1);
        }
    }

    virtual void detectAndCompute(cv::InputArray image, cv::InputArray mask,
        std::vector<cv::KeyPoint>& keypoints,
        cv::OutputArray descriptors,
        bool useProvidedKeypoints = false) override
    {
        cv::Mat src = image.getMat();
        cv::Mat mask_mat = mask.getMat();

        cv::Mat score(src.rows, src.cols, cv::MAT_8UC1);
        cv::Mat desc(src.rows, src.cols, cv::MAT_8UC3);

        ncnn::Mat in;
        ncnn::Mat out1, out2;
        in = ncnn::Mat::from_pixels(src.data, ncnn::Mat::PIXEL_GRAY, src.cols, src.rows);
        in.substract_mean_normalize(mean_vals, norm_vals);

        ncnn::Extractor ex = net.create_extractor();
        ex.input("input", in);
        ex.extract("score", out1);
        ex.extract("descriptor", out2);

        out1.substract_mean_normalize(mean_vals_inv, norm_vals_inv);
        out2.substract_mean_normalize(mean_vals_inv, norm_vals_inv);

        out1.to_pixels(score.data, ncnn::Mat::PIXEL_GRAY);
        out2.to_pixels(desc.data, ncnn::Mat::PIXEL_BGR);

        desc.copyTo(descriptors);

        extractFeature(score, 15, keypoints, keypoints);
    }

private:
    const float mean_vals[3] = { 0, 0, 0 };
    const float norm_vals[3] = { 1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0 };
    const float mean_vals_inv[3] = { 0, 0, 0 };
    const float norm_vals_inv[3] = { 255.f, 255.f, 255.f };
    ncnn::Net net;
    // Add LetNet specific members here
};
}