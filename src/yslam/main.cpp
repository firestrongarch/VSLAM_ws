import std;
import odom;
int main(int argc, char* argv[])
{
    // std::println("Hello, from slam!");
    // std::print("Hello, from slam!\n");
    std::cout << "Hello, from slam!\n";

    Yslam::Odom odom;
    odom.run("E:/CPP/VSLAM_ws/src/yslam/config/kitti00-02.toml"); // Pass the config file as an argument
    odom.stop();

    return 0;
}
