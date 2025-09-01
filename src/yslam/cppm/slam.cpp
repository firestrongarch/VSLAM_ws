// main.cpp
import std;

import cv;

/**
 * @brief 主函数入口
 *
 * 主程序入口，调用add函数并输出结果
 *
 * @return 0 程序正常结束
 */
int main()
{
    auto orb = cv::ORB::create();
    auto mat = cv::imread("C:/Users/00465078/Pictures/arch-linux-xw0szpochzcu0gml.jpg");
    cv::imshow("arch", mat);
    // std::cout << add(1, 2) << std::endl;
    return 0;
}