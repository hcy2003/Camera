/*
 * @Date: 2022-10-23 15:29:49
 * @LastEditTime: 2022-10-23 16:45:23
 * @FilePath: \RM-Vision2022\camera\HikVision\MyCamera.cpp
 */
#include "HKCamera.h"
#include <string.h>
#include <iostream>

HaiKang::HaiKang()
{
	m_hDevHandle = NULL;
	MV_CC_DEVICE_INFO_LIST device_list;
	int ok = MY_FAIL;
 	int p = 0;
	while (ok == MY_FAIL)
	{
		p++;
		if (m_hDevHandle != NULL)
		{
			// std::cout << m_hDevHandle << std::endl;
			printf("相机已被使用，请重启相机\n");
		}
		else
		{
			EnumDevices(&device_list); 		   //枚举设备列表
			ok = Open(device_list.pDeviceInfo[0]); //打开设备
		}
		cv::waitKey(500);
		if(p == 10){
			isOpen = false;
			break;
		}
	}
	//******************************************************************

	// MV_CC_SetGrabStrategy(m_hDevHandle, MV_GrabStrategy_LatestImagesOnly);
	// MV_CC_FeatureSave(m_hDevHandle, "FeatureFile.ini");//保存相机属性
	//  Get value of Enum nodes. Such as, 'TriggerMode' etc.
	if(isOpen){
		MVCC_ENUMVALUE stEnumVal;
		memset(&stEnumVal, 0, sizeof(MVCC_ENUMVALUE));

		float ex;
		GetFloatValue("ExposureTime", &ex);
		std::cout << ex << std::endl;

		int nRet = MV_CC_GetEnumValue(m_hDevHandle, "PixelFormat", &stEnumVal);
		if (MV_OK != nRet)
		{
			printf("Get PixelFormat Mode fail! nRet [0x%x]\n", nRet);
		}
		// nRet = MV_CC_SetEnumValue(m_hDevHandle, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
		// if (MV_OK != nRet)
		//{
		//     printf("Set PixelFormat fail! nRet [0x%x]\n", nRet);
		// }

		// SetFloatValue("ExposureTime", ExposureTime);//设置曝光，打开之后也可以设置
		PixelFormat = MvGvspPixelType(stEnumVal.nCurValue);
		if (PixelFormat == 35127317)
		{
			printf("PixelFormat Mode is: [ BGR8_Packed ]\n");
			PixelFormat = 1;
		}
		else if (PixelFormat == 17301513)
			printf("PixelFormat Mode is: [ BayerRG8 ]\n");
		else
			printf("Current PixelFormat [%d]\n", PixelFormat);
		//******************************************************************
		StartGrabbing(); //相机开始取流，要设制需在此之前

		//获取缓存区各种尺寸
		unsigned int stParamX;
		GetIntValue("PayloadSize", &stParamX); //获取图像缓冲区大小
		if (PixelFormat == 1)
			g_nPayloadSizeX = stParamX; // BGR数据模式缓存区已经是3通道的了
		else
			g_nPayloadSizeX = 1920 * 1080 * 3;										 //其他模式单通道转成BGR需扩大缓存区      //用于查找图像时输入缓存大小的变量
		pDataX = (unsigned char *)malloc(sizeof(unsigned char) * (g_nPayloadSizeX)); //图像数据接收指针
	}
}

HaiKang::HaiKang(int Mod)
{
	m_hDevHandle = NULL;
	MV_CC_DEVICE_INFO_LIST device_list;
	int ok = MY_FAIL;
	int q = 0;
	while (ok == MY_FAIL)
	{
		q ++;
		if (m_hDevHandle != NULL)
		{
			printf("相机已被打开，请重启相机\n");
		}
		else
		{
			EnumDevices(&device_list);			   //枚举设备列表
			ok = Open(device_list.pDeviceInfo[0]); //打开设备
		}
		cv::waitKey(500);
		if (q == 10)
			break;
	}
	//******************************************************************
	if (Mod == 0)
	{
	}

	//******************************************************************
	StartGrabbing(); //相机开始取流，要设制需在此之前

	//获取缓存区各种尺寸
	unsigned int stParamX;
	GetIntValue("PayloadSize", &stParamX);										 //获取图像缓冲区大小
	g_nPayloadSizeX = stParamX;													 //用于查找图像时输入缓存大小的变量
	pDataX = (unsigned char *)malloc(sizeof(unsigned char) * (g_nPayloadSizeX)); //图像数据接收指针
}

cv::Mat HaiKang::GetOneFrameForOpencv()
{
	int nRet = 1;
	if (PixelFormat == 1)
	{
		nRet = MV_CC_GetImageBuffer(m_hDevHandle, &pstFrame, 1000);
		if (pstFrame.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
		{
			Matimage = cv::Mat(pstFrame.stFrameInfo.nHeight, pstFrame.stFrameInfo.nWidth, CV_8UC3, pstFrame.pBufAddr);
			MV_CC_FreeImageBuffer(m_hDevHandle, &pstFrame);
			return Matimage;
		}
	}
	else
	{
		nRet = MV_CC_GetImageForBGR(m_hDevHandle, pDataX, g_nPayloadSizeX, &stImageInfoX, 1000);
		Matimage = cv::Mat(stImageInfoX.nHeight, stImageInfoX.nWidth, CV_8UC3, pDataX);
		if (!Matimage.data)
		{
			printf("camera get frame failed!!!");
			cv::waitKey(0);
		}
		return Matimage;
	}
}

HaiKang::~HaiKang()
{
	if (m_hDevHandle)
	{
		StopGrabbing(); //停止抓图
		Close();		//关闭设备，并销毁句柄
		m_hDevHandle = NULL;
	}
}

int HaiKang::EnumDevices(MV_CC_DEVICE_INFO_LIST *pstDevList)
{
	// Enum GIGE Devices
	int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, pstDevList);
	if (MV_OK != nRet)
	{
		printf("无设备与电脑连接\n");
		return MY_FAIL;
	}
	printf("找到：%d 台设备可用\n", pstDevList->nDeviceNum);

	return MY_OK;
}

// 打开设备
int HaiKang::Open(MV_CC_DEVICE_INFO *pstDeviceInfo)
{
	if (NULL == pstDeviceInfo)
	{
		printf("无设备与电脑连接\n");
		return MY_FAIL;
	}

	printf("打开第一台相机\n");
	int nRet = MV_CC_CreateHandle(&m_hDevHandle, pstDeviceInfo);
	if (MV_OK != nRet)
	{

		printf("创建句柄失败状态码 [0x%x]\n", nRet);
		return MY_FAIL;
	}
	printf("创建设备句柄成功\n");

	nRet = MV_CC_OpenDevice(m_hDevHandle);
	if (MV_OK != nRet)
	{
		printf("打开设备失败，现在销毁句柄  状态码 [0x%x]\n", nRet);
		MV_CC_DestroyHandle(&m_hDevHandle);
		return MY_FAIL;
	}
	printf("打开设备成功 \n ");

	return MY_OK;
}

// 关闭设备
int HaiKang::Close()
{
	if (NULL == m_hDevHandle)
	{
		return MY_FAIL;
	}
	MV_CC_CloseDevice(m_hDevHandle);
	printf("关闭相机，释放句柄。\n");
	return MV_CC_DestroyHandle(m_hDevHandle);
}

// 开启抓图
int HaiKang::StartGrabbing()
{
	if (NULL == m_hDevHandle)
	{
		return MY_FAIL;
	}

	return MV_CC_StartGrabbing(m_hDevHandle);
}

// 停止抓图
int HaiKang::StopGrabbing()
{
	if (NULL == m_hDevHandle)
	{
		return MY_FAIL;
	}
	printf("\n已关闭取流，");
	return MV_CC_StopGrabbing(m_hDevHandle);
}

int HaiKang::GetOneFrameTimeout(unsigned char *pData, unsigned int *pnDataLen, unsigned int nDataSize, MV_FRAME_OUT_INFO_EX *pFrameInfo, int nMsec)
{
	if (NULL == m_hDevHandle || NULL == pData || NULL == pnDataLen || NULL == pFrameInfo)
	{
		return MY_FAIL;
	}

	*pnDataLen = 0;

	int nRet = MV_CC_GetOneFrameTimeout(m_hDevHandle, pData, nDataSize, pFrameInfo, nMsec);
	if (MV_OK != nRet)
	{
		return MY_FAIL;
	}

	*pnDataLen = pFrameInfo->nFrameLen;

	return MY_OK;
}

// 设置显示窗口句柄
int HaiKang::Display(void *hWnd)
{
	if (NULL == m_hDevHandle)
	{
		return MY_FAIL;
	}

	return MV_CC_Display(m_hDevHandle, hWnd);
}

int HaiKang::SaveImage(MV_SAVE_IMAGE_PARAM_EX *pstParam)
{
	if (NULL == pstParam)
	{
		return MY_FAIL;
	}

	return MV_CC_SaveImageEx(pstParam);
}

// 注册图像数据回调
int HaiKang::RegisterImageCallBack(void(__stdcall *cbOutput)(unsigned char *pData,
															   MV_FRAME_OUT_INFO_EX *pFrameInfo,
															   void *pUser),
									 void *pUser)
{
	if (NULL == m_hDevHandle)
	{
		return MY_FAIL;
	}

	return MV_CC_RegisterImageCallBackEx(m_hDevHandle, cbOutput, pUser);
}

// 注册消息异常回调
int HaiKang::RegisterExceptionCallBack(void(__stdcall *cbException)(unsigned int nMsgType,
																	  void *pUser),
										 void *pUser)
{
	if (NULL == m_hDevHandle)
	{
		return MY_FAIL;
	}

	return MV_CC_RegisterExceptionCallBack(m_hDevHandle, cbException, pUser);
}

// 获取Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::GetIntValue(IN const char *strKey, OUT unsigned int *pnValue)
{
	if (NULL == m_hDevHandle || NULL == strKey || NULL == pnValue)
	{
		return MY_FAIL;
	}

	MVCC_INTVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	int nRet = MV_CC_GetIntValue(m_hDevHandle, strKey, &stParam);
	if (MV_OK != nRet)
	{
		return MY_FAIL;
	}

	*pnValue = stParam.nCurValue;

	return MY_OK;
}

// 设置Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::SetIntValue(IN const char *strKey, IN unsigned int nValue)
{
	if (NULL == m_hDevHandle || NULL == strKey)
	{
		return MY_FAIL;
	}

	return MV_CC_SetIntValue(m_hDevHandle, strKey, nValue);
}

// 获取Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::GetFloatValue(IN const char *strKey, OUT float *pfValue)
{
	if (NULL == m_hDevHandle || NULL == strKey || NULL == pfValue)
	{
		return MY_FAIL;
	}

	MVCC_FLOATVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_FLOATVALUE));
	int nRet = MV_CC_GetFloatValue(m_hDevHandle, strKey, &stParam);
	if (MV_OK != nRet)
	{
		return MY_FAIL;
	}

	*pfValue = stParam.fCurValue;

	return MY_OK;
}

// 设置Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::SetFloatValue(IN const char *strKey, IN float fValue)
{
	if (NULL == m_hDevHandle || NULL == strKey)
	{
		return MY_FAIL;
	}

	return MV_CC_SetFloatValue(m_hDevHandle, strKey, fValue);
}

// 获取Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::GetEnumValue(IN const char *strKey, OUT unsigned int *pnValue)
{
	if (NULL == m_hDevHandle || NULL == strKey || NULL == pnValue)
	{
		return MY_FAIL;
	}

	MVCC_ENUMVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_ENUMVALUE));
	int nRet = MV_CC_GetEnumValue(m_hDevHandle, strKey, &stParam);
	if (MV_OK != nRet)
	{
		return MY_FAIL;
	}

	*pnValue = stParam.nCurValue;

	return MY_OK;
}

// 设置Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::SetEnumValue(IN const char *strKey, IN unsigned int nValue)
{
	if (NULL == m_hDevHandle || NULL == strKey)
	{
		return MY_FAIL;
	}

	return MV_CC_SetEnumValue(m_hDevHandle, strKey, nValue);
}

// 获取Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::GetBoolValue(IN const char *strKey, OUT bool *pbValue)
{
	if (NULL == m_hDevHandle || NULL == strKey || NULL == pbValue)
	{
		return MY_FAIL;
	}

	int nRet = MV_CC_GetBoolValue(m_hDevHandle, strKey, pbValue);
	if (MV_OK != nRet)
	{
		return MY_FAIL;
	}

	return MY_OK;
}

// 设置Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::SetBoolValue(IN const char *strKey, IN bool bValue)
{
	if (NULL == m_hDevHandle || NULL == strKey)
	{
		return MY_FAIL;
	}

	return MV_CC_SetBoolValue(m_hDevHandle, strKey, bValue);
}

// 获取String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
int HaiKang::GetStringValue(IN const char *strKey, IN OUT char *strValue, IN unsigned int nSize)
{
	if (NULL == m_hDevHandle || NULL == strKey || NULL == strValue)
	{
		return MY_FAIL;
	}

	MVCC_STRINGVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_STRINGVALUE));
	int nRet = MV_CC_GetStringValue(m_hDevHandle, strKey, &stParam);
	if (MV_OK != nRet)
	{
		return MY_FAIL;
	}

	// strcpy_s(strValue, nSize, stParam.chCurValue);

	return MY_OK;
}

// 设置String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
int HaiKang::SetStringValue(IN const char *strKey, IN const char *strValue)
{
	if (NULL == m_hDevHandle || NULL == strKey)
	{
		return MY_FAIL;
	}

	return MV_CC_SetStringValue(m_hDevHandle, strKey, strValue);
}

// 执行一次Command型命令，如 UserSetSave，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
int HaiKang::CommandExecute(IN const char *strKey)
{
	if (NULL == m_hDevHandle || NULL == strKey)
	{
		return MY_FAIL;
	}

	return MV_CC_SetCommandValue(m_hDevHandle, strKey);
}
