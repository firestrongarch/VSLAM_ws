#include "letnet/letnet.h"

LetNet::LetNet(const std::string& path){
    net.load_param((path + "/model.param").c_str());
    net.load_model((path + "/model.bin").c_str());
}

bool LetNet::extract(const cv::Mat& src, cv::Mat& out){
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

    // out1.to_pixels(score_.data, ncnn::Mat::PIXEL_GRAY);
    out2.to_pixels(out.data, ncnn::Mat::PIXEL_BGR);
    return true;
}