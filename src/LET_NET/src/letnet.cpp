#include "letnet/letnet.h"

LetNet::LetNet(const std::string& path)
{
    net.load_param((path + "/model.param").c_str());
    net.load_model((path + "/model.bin").c_str());
}

bool LetNet::extract(const cv::Mat& src, cv::Mat& desc, cv::Mat& score)
{
    ncnn::Mat in;
    ncnn::Mat out1, out2;

    in = ncnn::Mat::from_pixels(src.data, ncnn::Mat::PIXEL_BGR, src.cols, src.rows);
    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = net.create_extractor();
    ex.input("input", in);
    ex.extract("score", out1);
    ex.extract("descriptor", out2);
    out1.substract_mean_normalize(mean_vals_inv, norm_vals_inv);
    out2.substract_mean_normalize(mean_vals_inv, norm_vals_inv);

    out1.to_pixels(score.data, ncnn::Mat::PIXEL_GRAY);
    out2.to_pixels(desc.data, ncnn::Mat::PIXEL_BGR);
    return true;
}

std::vector<cv::Point2f> LetNet::extractFeature(
    const cv::Mat& score,
    int ncellsize,
    const std::vector<cv::Point2f>& vcurkps)
{
    if (score.empty()) {
        return std::vector<cv::Point2f>();
    }

    size_t ncols = score.cols;
    size_t nrows = score.rows;

    size_t nhalfcell = ncellsize / 4;

    size_t nhcells = nrows / ncellsize;
    size_t nwcells = ncols / ncellsize;
    size_t nbcells = nhcells * nwcells;

    std::vector<cv::Point2f> vdetectedpx;
    vdetectedpx.reserve(nbcells);

    std::vector<std::vector<bool>> voccupcells(
        nhcells + 1,
        std::vector<bool>(nwcells + 1, false));

    cv::Mat mask = cv::Mat::ones(score.rows, score.cols, CV_8UC1);

    for (const auto& px : vcurkps) {
        voccupcells[px.y / ncellsize][px.x / ncellsize] = true;
        cv::circle(mask, px, nhalfcell, cv::Scalar(0.), -1);
    }

    size_t nboccup = 0;

    std::vector<std::vector<cv::Point2f>> vvdetectedpx(nbcells);
    std::vector<std::vector<cv::Point2f>> vvsecdetectionspx(nbcells);

    auto cvrange = cv::Range(0, nbcells);

    parallel_for_(cvrange, [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; i++) {

            size_t r = floor(i / nwcells);
            size_t c = i % nwcells;

            if (voccupcells[r][c]) {
                nboccup++;
                continue;
            }

            size_t x = c * ncellsize;
            size_t y = r * ncellsize;

            cv::Rect hroi(x, y, ncellsize, ncellsize);

            if (x + ncellsize < ncols - 1 && y + ncellsize < nrows - 1) {

                double dminval, dmaxval;
                cv::Point minpx, maxpx;

                cv::minMaxLoc(score(hroi).mul(mask(hroi)), &dminval, &dmaxval, &minpx, &maxpx);
                maxpx.x += x;
                maxpx.y += y;

                if (dmaxval >= 0.2) {
                    vvdetectedpx.at(i).push_back(maxpx);
                    cv::circle(mask, maxpx, nhalfcell, cv::Scalar(0.), -1);
                }

                cv::minMaxLoc(score(hroi).mul(mask(hroi)), &dminval, &dmaxval, &minpx, &maxpx);
                maxpx.x += x;
                maxpx.y += y;

                if (dmaxval >= 0.2) {
                    vvsecdetectionspx.at(i).push_back(maxpx);
                    cv::circle(mask, maxpx, nhalfcell, cv::Scalar(0.), -1);
                }
            }
        }
    });

    for (const auto& vpx : vvdetectedpx) {
        if (!vpx.empty()) {
            vdetectedpx.insert(vdetectedpx.end(), vpx.begin(), vpx.end());
        }
    }

    size_t nbkps = vdetectedpx.size();

    if (nbkps + nboccup < nbcells) {
        size_t nbsec = nbcells - nbkps - nboccup;
        size_t k = 0;
        for (const auto& vseckp : vvsecdetectionspx) {
            if (!vseckp.empty()) {
                vdetectedpx.push_back(vseckp.back());
                k++;
                if (k == nbsec) {
                    break;
                }
            }
        }
    }

    return vdetectedpx;
}