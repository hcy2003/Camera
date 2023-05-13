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

#define HK
#define SERIAL

using namespace std;
using namespace cv;

int currentCamera = 0;
bool cam1 = true;
bool cam2 = true;
bool cam3 = true;


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

void getMode(Serial_Port serial_port)
{
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
	const char *str_order;

	Serial_Port serial_port;
	serialWatcher(serial_port);
	// 打开视频设备  one video0 one video2
	const char* video_device_1 = "/dev/video0";
	const char* video_device_2 = "/dev/video2";
	const char* video_device_3 = "/dev/video4";
#ifdef HK
	HaiKang HaikangCamera;
	if(HaikangCamera.isOpen){
		HaikangCamera.SetFloatValue("ExposureTime", 3000);
		HaikangCamera.SetFloatValue("Gain",8.0062);
		HaikangCamera.SetBoolValue("GammaEnable", 1);
		HaikangCamera.SetFloatValue("Gamma", 1);
	}
#endif

	// 打开相机1并设置参数
	int fd_1 = open(video_device_1, O_RDWR);
	//while(fd_1==-1)	{cout<<"the first camera open error"<<endl; waitKey(1000);}
	cout << endl << "fd_1: " << fd_1 << endl; 
	if(fd_1 == -1){
		cam1 = false;
		currentCamera = 1;
	}
	cout << "cam1: " << cam1 << endl;
	struct v4l2_format fmt_1;
	if(cam1){
		fmt_1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt_1.fmt.pix.width = 640;
		fmt_1.fmt.pix.height = 480;
		fmt_1.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		ioctl(fd_1, VIDIOC_S_FMT, &fmt_1);
	}

	// 打开相机2并设置参数
	int fd_2 = open(video_device_2, O_RDWR);
	//while(fd_2==-1) {cout<<"second camera open error"<<endl; waitKey(1000);}
	cout << "fd_2: " << fd_2 << endl; 
	if(fd_2 == -1){
		cam2 = false;
		if(!cam1)
			currentCamera = 2;
	}	
	cout << "cam2: " << cam2 << endl;
	struct v4l2_format fmt_2;
	if(cam2){
	
		fmt_2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt_2.fmt.pix.width = 640;
		fmt_2.fmt.pix.height = 480;
		fmt_2.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		ioctl(fd_2, VIDIOC_S_FMT, &fmt_2);
	}

	// 打开相机3并设置参数
	int fd_3 = open(video_device_3, O_RDWR);
	cout << "fd_3: " << fd_3 << endl; 
	if(fd_3 == -1){
		cam3 = false;
		if(!cam1 && !cam2)
			currentCamera = 3;
	}	
	cout << "cam3: " << cam3 << endl;
	struct v4l2_format fmt_3;
	if(cam3){
	
		fmt_3.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt_3.fmt.pix.width = 640;
		fmt_3.fmt.pix.height = 480;
		fmt_3.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		ioctl(fd_3, VIDIOC_S_FMT, &fmt_3);
	}
	struct v4l2_requestbuffers req;
	req.count = 4 ;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	// 分配相机1的缓冲区
	vector<uchar*> buffers_1(req.count);
	vector<int> buffer_lengths_1(req.count);
	if(cam1){
		ioctl(fd_1, VIDIOC_REQBUFS, &req);
		
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
			ioctl(fd_1, VIDIOC_STREAMON, &type);
		}
	}

	// 分配相机2的缓冲区
	vector<uchar*> buffers_2(req.count);
	vector<int> buffer_lengths_2(req.count);

	if(cam2){
		ioctl(fd_2, VIDIOC_REQBUFS, &req);
		
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
			ioctl(fd_2, VIDIOC_STREAMON, &type);
		}
	}

	// 分配相机3的缓冲区
	vector<uchar*> buffers_3(req.count);
	vector<int> buffer_lengths_3(req.count);

	if(cam3){
		ioctl(fd_3, VIDIOC_REQBUFS, &req);
		
		for (int i = 0; i < req.count; ++i)
		{
			struct v4l2_buffer buf;
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = i;
			ioctl(fd_3, VIDIOC_QUERYBUF, &buf);
			buffers_3[i] = (uchar*)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_3, buf.m.offset);
			buffer_lengths_3[i] = buf.length;
			ioctl(fd_3, VIDIOC_QBUF, &buf);
			ioctl(fd_3, VIDIOC_STREAMON, &type);
		}
	}
	// 创建窗口并显示视频数据
	cv::namedWindow("Camera", WINDOW_NORMAL);
	cv::setWindowProperty("Camera", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

	// 从队列中取出内存缓冲区
	struct v4l2_buffer buf;

	std::thread t1(getMode,serial_port);
	while (true)
	{
		int c;
		//cout << currentCamera << endl;
		if (currentCamera == 0 && cam1) {
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			if(ioctl(fd_1, VIDIOC_DQBUF, &buf) == -1)
				cam1 = false;
			Mat image1(fmt_1.fmt.pix.height, fmt_1.fmt.pix.width, CV_8UC2, buffers_1[buf.index], fmt_1.fmt.pix.bytesperline);
			cvtColor(image1, image1, COLOR_YUV2BGR_YUYV);
			//cout<<"***";
			imshow("Camera", image1);
			ioctl(fd_1, VIDIOC_QBUF, &buf);
        }else if((currentCamera == 1 && cam2)){
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			if(ioctl(fd_2, VIDIOC_DQBUF, &buf) == -1)
				cam2 = false;
			Mat image2(fmt_2.fmt.pix.height, fmt_2.fmt.pix.width, CV_8UC2, buffers_2[buf.index], fmt_2.fmt.pix.bytesperline);
			cvtColor(image2, image2, COLOR_YUV2BGR_YUYV);
			//cout<<"===";
			// transpose(image2, image2);
			// flip(image2,image2,0);
			imshow("Camera", image2);
			ioctl(fd_2, VIDIOC_QBUF, &buf);
        }else if(currentCamera == 2 && cam3){
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			if(ioctl(fd_3, VIDIOC_DQBUF, &buf) == -1)
				cam3 = false;
			Mat image3(fmt_3.fmt.pix.height, fmt_3.fmt.pix.width, CV_8UC2, buffers_3[buf.index], fmt_3.fmt.pix.bytesperline);
			cvtColor(image3, image3, COLOR_YUV2BGR_YUYV);
			//cout<<"***";
			imshow("Camera", image3);
			ioctl(fd_3, VIDIOC_QBUF, &buf);
		}
		#ifdef HK
		else if(currentCamera == 3 && HaikangCamera.isOpen){
			Mat image4 = HaikangCamera.GetOneFrameForOpencv();
			imshow("Camera", image4);
		}
		#endif
		// 从对应相机缓存中读取数据
		// 检查是否按下了ESC键
		c = waitKey(1);
		//cout << c << endl;
		if (c == 49 && cam1){
			cout << c << endl;
			currentCamera = 0;
		}else if(c == 50 && cam2){
			currentCamera = 1;
			cout << c << endl;
		}
		#ifdef HK
		else if(c == 52 && HaikangCamera.isOpen){
			cout << c << endl;
			currentCamera = 3;
		}
		#endif
		else if(c == 51 && cam3){
			cout << c << endl;
			currentCamera = 2;
		}
		if (c == 27){
			currentCamera = 50;
			cout << c << endl;
			break;
		}
	}
    t1.join();
	// 停止捕捉视频数据
	if(cam1)
		ioctl(fd_1, VIDIOC_STREAMOFF, &type);
	if(cam2)
		ioctl(fd_2, VIDIOC_STREAMOFF, &type);
	if(cam3)
		ioctl(fd_3, VIDIOC_STREAMOFF, &type);
	
	// 解除内存映射
	for (int i = 0; i < req.count; ++i)
	{
		if(cam1)
			munmap(buffers_1[i], buffer_lengths_1[i]);
		if(cam2)
			munmap(buffers_2[i], buffer_lengths_2[i]);
		if(cam3)
			munmap(buffers_3[i], buffer_lengths_3[i]);
	}

	// 关闭视频设备
	if(cam1)
		close(fd_1);
	if(cam2)
		close(fd_2);
	if(cam3)
		close(fd_3);
	return 0;
}

