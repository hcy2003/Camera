#include <iostream>
#include <fstream>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>

#include "serial_port.hpp"
#include "general.h"
#include "HKCamera.h"

//#define HK;
#define SERIAL;

using namespace std;
using namespace cv;

int currentCamera = 2;

bool serialWatcher(Serial_Port &serial);
bool serialWatcher(Serial_Port &serial)
{
    // 串口通讯相关程序
    printf("\n*******Begin setting Com*********\n ");
    char const *device0 = UART_NAME;
    // 串口无权限时自动开启所有权限            while (100)
    {
        cout << "lll"
             << " ";
        waitKey(1000);
    }
    if (access(UART_NAME, W_OK) != 0) // 成功执行时，返回0。失败返回-1
    {
        char commed1[100], dest[30];
        strcpy(commed1, "echo ");
        strcpy(dest, " | sudo -S chmod 777 ");
        strcat(commed1, PASSWD);
        strcat(commed1, dest);
        strcat(commed1, UART_NAME);
        std::cout << commed1 << std::endl;
        system(commed1);
    }

    sleep(1);
    // 检测文件夹是否存在或串口需要初始化
    if (access(UART_NAME, F_OK) == -1 || serial.is_serial_opened == false)
    {
        serial.is_serial_opened = false;
        //* 初始化串口
        serial.init_serial(device0);
    }
    return true;
}

void getMode()
{
	const char *str_order;

	Serial_Port serial_port;
    serialWatcher(serial_port);
	while (true)
	{   
	#ifdef SERIAL
		if (serial_port.uart_recv())
		{
			currentCamera = serial_port.mode;
			cout << currentCamera << endl;
		}else if(currentCamera == 50){
			break;
		}
	#endif
	}
}

void getFrame1()
{
	const char* video_device_1 = "/dev/video0";

	// 打开相机1并设置参数
	int fd_1 = open(video_device_1, O_RDWR);
	struct v4l2_format fmt_1;
	fmt_1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt_1.fmt.pix.width = 640;
	fmt_1.fmt.pix.height = 480;
	fmt_1.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ioctl(fd_1, VIDIOC_S_FMT, &fmt_1);

	// 分配内存缓冲区
	struct v4l2_requestbuffers req;
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	// 分配相机1的缓冲区
	ioctl(fd_1, VIDIOC_REQBUFS, &req);
	vector<uchar*> buffers_1(req.count);
	vector<int> buffer_lengths_1(req.count);
	for (int i = 0; i < req.count; ++i)
	{
		struct v4l2_buffer buf;
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		ioctl(fd_1, VIDIOC_QUERYBUF, &buf);
		buffers_1[i] = (uchar*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_1, buf.m.offset);
		buffer_lengths_1[i] = buf.length;
		ioctl(fd_1, VIDIOC_QBUF, &buf);
	}
	// 开始捕捉视频数据
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(fd_1, VIDIOC_STREAMON, &type);

	struct v4l2_buffer buf;

	cv::namedWindow("Camera1", WINDOW_NORMAL);
	cv::setWindowProperty("Camera1", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

	while (true)
	{
		if (currentCamera == 0) {
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			ioctl(fd_1, VIDIOC_DQBUF, &buf);
			
			Mat image1(fmt_1.fmt.pix.height, fmt_1.fmt.pix.width, CV_8UC2, buffers_1[buf.index], fmt_1.fmt.pix.bytesperline);
			cvtColor(image1, image1, COLOR_YUV2BGR_YUYV);

			imshow("Camera1", image1);
			ioctl(fd_1, VIDIOC_QBUF, &buf);
        }else if(currentCamera == 50){
			break;
		}else if(currentCamera == 1 || currentCamera == 2){
			//destroyWindow("Camera1");
		}
	}
	ioctl(fd_1, VIDIOC_STREAMOFF, &type);

	// 解除内存映射
	for (int i = 0; i < req.count; ++i)
	{
		munmap(buffers_1[i], buffer_lengths_1[i]); 
	}

	// 关闭视频设备
	close(fd_1);
}

void getFrame2()
{
	const char* video_device_2 = "/dev/video2";

	// 打开相机2并设置参数
	int fd_2 = open(video_device_2, O_RDWR);
	struct v4l2_format fmt_2;
	fmt_2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt_2.fmt.pix.width = 640;
	fmt_2.fmt.pix.height = 480;
	fmt_2.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ioctl(fd_2, VIDIOC_S_FMT, &fmt_2);

	// 分配内存缓冲区
	struct v4l2_requestbuffers req;
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	// 分配相机2的缓冲区
	ioctl(fd_2, VIDIOC_REQBUFS, &req);
	vector<uchar*> buffers_2(req.count);
	vector<int> buffer_lengths_2(req.count);
	for (int i = 0; i < req.count; ++i)
	{
		struct v4l2_buffer buf;
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		ioctl(fd_2, VIDIOC_QUERYBUF, &buf);
		buffers_2[i] = (uchar*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_2, buf.m.offset);
		buffer_lengths_2[i] = buf.length;
		ioctl(fd_2, VIDIOC_QBUF, &buf);
	}
	// 开始捕捉视频数据
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ioctl(fd_2, VIDIOC_STREAMON, &type);

	// 创建窗口并显示视频数据
	cv::namedWindow("Camera2", WINDOW_NORMAL);
	cv::setWindowProperty("Camera2", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

 	// 从队列中取出内存缓冲区
	struct v4l2_buffer buf;
	while(true){
		if(currentCamera == 2)
		{
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			ioctl(fd_2, VIDIOC_DQBUF, &buf);
			
			Mat image2(fmt_2.fmt.pix.height, fmt_2.fmt.pix.width, CV_8UC2, buffers_2[buf.index], fmt_2.fmt.pix.bytesperline);
			cvtColor(image2, image2, COLOR_YUV2BGR_YUYV);
			
			imshow("Camera2", image2);
			ioctl(fd_2, VIDIOC_QBUF, &buf);
		}else if(currentCamera == 50){
			break;
		}else if(currentCamera == 1 || currentCamera == 0){
			//destroyWindow("Camera2");
		}
	}
	ioctl(fd_2, VIDIOC_STREAMOFF, &type);

	// 解除内存映射
	for (int i = 0; i < req.count; ++i)
	{
		munmap(buffers_2[i], buffer_lengths_2[i]);
	}

	// 关闭视频设备
	close(fd_2);

}

std::vector<std::string> list_v4l2_devices()
{
    std::vector<std::string> devices;
    std::ifstream infile("/proc/devices");
    std::string line;
    while (std::getline(infile, line))
    {
        if (line.find("video") != std::string::npos)
        {
            int major;
            if (std::sscanf(line.c_str(), " %d %*s", &major) == 1)
            {
                std::string device_name = "/dev/video" + std::to_string(major);
                devices.push_back(device_name);
            }
        }
    }
    return devices;
}

int main()
{
#ifdef HK
	HaiKang HaikangCamera;
	HaikangCamera.SetFloatValue("ExposureTime", 4000);
	HaikangCamera.SetFloatValue("Gain",8.0062);
	HaikangCamera.SetBoolValue("GammaEnable", 1);
	HaikangCamera.SetFloatValue("Gamma", 1);
#endif
	Serial_Port serial_port;
    serialWatcher(serial_port);
 
	std::thread t1(getMode);
	std::thread t2(getFrame1);
	std::thread t3(getFrame2);

	while (true)
	{
	#ifdef HK
		if(currentCamera == 1){
			cv::namedWindow("Camera3", WINDOW_NORMAL);
	    	cv::setWindowProperty("Camera3", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
			Mat image3 = HaikangCamera.GetOneFrameForOpencv();
			imshow("Camera3", image3);
		}
	#endif
		// 从对应相机缓存中读取数据
		
		// 检查是否按下了ESC键
		int c = waitKey(1);
		if (c == 49 ){
			currentCamera = 0;
		}else if(c == 50){
			currentCamera = 2;
		}
	#ifdef HK
		else if(c == 51){
			currentCamera = 1;
		}
	#endif
		if (c == 27){
			currentCamera = 50;
			break;
		}
	}
    t1.join();
	t2.join();
	t3.join();

	return 0;
}