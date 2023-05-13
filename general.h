
#pragma once

// STD Library
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

//常用的包
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef struct
{
    int pitch_angle; //偏航角
    int yaw_angle;   //俯仰角
    int roll_angle;  //翻滚角
    int dis;         //到兑换框距离
    int x;
    int y;
    int z;
    bool isDetected;    //是否检测到目标
} VisionData;
