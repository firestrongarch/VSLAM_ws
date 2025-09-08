export module camera;
import cv;
import types;
import std;
import convert;

export namespace Yslam {

class Camera {
public:
    virtual void Init(std::shared_ptr<Frame> frame) = 0;
    virtual void Extract3d(std::shared_ptr<Frame> frame) = 0;
    virtual void Track(std::shared_ptr<Frame> frame) = 0;
};

class Stereo : public Camera {
public:
    void Init(std::shared_ptr<Frame> frame) override
    {
        auto& kps = frame->kps;
        auto& img0 = frame->img0;
        auto& last_frame = frame->last;

        auto ex = cv::ORB::create();
        std::vector<cv::KeyPoint> kps_ex;
        ex->detect(img0, kps_ex);
        for (const auto& kp : kps_ex) {
            KeyPoint keypoint(kp);
            kps.emplace_back(keypoint);
        }
        last_frame = frame;
        // Initialize stereo camera with the given frame
    }

    void Extract3d(std::shared_ptr<Frame> frame) override
    {
        // Extract 3D information from the stereo camera
    }

    void Track(std::shared_ptr<Frame> frame) override
    {
        auto& kps0 = frame->last->kps;
        auto& kps1 = frame->kps;
        auto& img0 = frame->last->img0;
        auto& img1 = frame->img0;

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
                KeyPoint kp;
                kp.pt = pts1[i];
                kps1.emplace_back(kp);
            } else {
                kps0[i].is_outlier = true;
                // Point lost, do nothing
            }
        }
    }
};

}; // namespace Yslam