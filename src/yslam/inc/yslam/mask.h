#pragma once
#include "yslam/frame.h"

namespace Yslam {
class Mask {
public:
    virtual void mask(Frame& frame) = 0;
    virtual ~Mask() = default;
};

class LetNetMask : public Mask {
public:
    void mask(Frame& frame) override
    {
        size_t ncols = frame.img0.cols;
        size_t nrows = frame.img0.rows;

        auto ncellsize = frame.cell_num;

        size_t nhalfcell = ncellsize / 4;

        size_t nhcells = nrows / ncellsize;
        size_t nwcells = ncols / ncellsize;
        size_t nbcells = nhcells * nwcells;

        std::vector<cv::Point2f> vdetectedpx;
        vdetectedpx.reserve(nbcells);

        std::vector<std::vector<bool>> voccupcells(
            nhcells + 1,
            std::vector<bool>(nwcells + 1, false));

        cv::Mat mask = cv::Mat::ones(nrows, ncols, CV_8UC1);

        for (const auto& px : frame.pts0) {
            voccupcells[px.y / ncellsize][px.x / ncellsize] = true;
            cv::circle(mask, px, nhalfcell, cv::Scalar(0.), -1);
        }

        frame.mask = mask;
    }
};

}