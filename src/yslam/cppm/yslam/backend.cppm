export module backend;
export import types;
export import cv;
export import ceres;
export import std;
import optimizer;

export namespace Yslam {
class Backend {
    Backend() = default;
    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;

public:
    void Run();
    // void RemoveOutliers();
    static Backend& GetInstance()
    {
        static Backend instance;
        return instance;
    }
};

void Backend::Run()
{
    std::list<std::shared_ptr<KeyFrame>> kf_list;
    while (1) {
        std::shared_ptr<KeyFrame> kf;

        Map::GetInstance().kf_queue_.pop(kf);
        if (kf) {
            kf_list.push_back(kf);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        static Optimizer opt;
        opt.BundleAdjustment(kf_list);

        if (kf_list.size() > 5) {
            kf_list.pop_front();
            // kf_list.pop_front();
        }

        Map::GetInstance().RemoveOutliers();
    }
}
}