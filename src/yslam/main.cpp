#include "odom.h"
#include <iostream>
#include <print>

int main(int argc, char* argv[])
{
    std::printf("Hello, from slam!\n");
    std::print("Hello, from slam!\n");
    std::cout << "Hello, from slam!\n";

    Yslam::Odom odom;
    odom.run(argv[1]); // Pass the config file as an argument
    odom.stop();

    return 0;
}
