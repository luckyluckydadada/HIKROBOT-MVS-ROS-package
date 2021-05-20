#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include "MvCameraControl.h"

#include "hk_camera.h"

HkCam::HkCam()
	: handle(NULL),
	  nRet(MV_OK),
	  pData(NULL),
	  pDataForRGB(NULL),
	  stConvertParam{0}
{
}

HkCam::~HkCam()
{
	shutdown();
}

void HkCam::shutdown()
{
	if (handle != NULL)
	{
		MV_CC_DestroyHandle(handle);
		handle = NULL;
	}
	if (pData)
	{
		free(pData);
		pData = NULL;
	}
	if (pDataForRGB)
	{
		free(pDataForRGB);
		pDataForRGB = NULL;
	}

	printf("exit\n");
	return;
}

void HkCam::start()
{
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
	nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
	if (MV_OK != nRet)
	{
		printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
		shutdown();
		return;
	}
	if (stDeviceList.nDeviceNum > 0)
	{
		for (int i = 0; i < stDeviceList.nDeviceNum; i++)
		{
			printf("[device %d]:\n", i);
			MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
			if (NULL == pDeviceInfo)
			{
				shutdown();
				return;
			}
			PrintDeviceInfo(pDeviceInfo);
		}
	}
	else
	{
		printf("Find No Devices!\n");
		shutdown();
		return;
	}
	printf("Please Intput camera index: ");
	unsigned int nIndex = 0;
	scanf("%d", &nIndex);
	if (nIndex >= stDeviceList.nDeviceNum)
	{
		printf("Intput error!\n");
		shutdown();
		return;
	}
	nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
	if (MV_OK != nRet)
	{
		printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
		shutdown();
		return;
	}
	nRet = MV_CC_OpenDevice(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
		shutdown();
		return;
	}
	if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
	{
		int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
		if (nPacketSize > 0)
		{
			nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
			if (nRet != MV_OK)
			{
				printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
			}
		}
		else
		{
			printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
		}
	}
	nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
	if (MV_OK != nRet)
	{
		printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
		shutdown();
		return;
	}
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
	if (MV_OK != nRet)
	{
		printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
		shutdown();
		return;
	}
	nRet = MV_CC_StartGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
		shutdown();
		return;
	}
}

bool HkCam::grab_rgb_image()
{
	MV_FRAME_OUT_INFO_EX stImageInfo = {0};
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
	if (NULL == pData)
	{
		shutdown();
		return false;
	}
	unsigned int nDataSize = stParam.nCurValue;
	nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
	if (MV_OK != nRet)
	{
		printf("MV_CC_GetOneFrameTimeout fail! nRet [%x]\n", nRet);
		shutdown();
		return false;
	}
	printf("Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n",
		   stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
	pDataForRGB = (unsigned char *)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
	if (NULL == pDataForRGB)
	{
		shutdown();
		return false;
	}
	// 像素格式转换
	// 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
	// 目标像素格式，输出数据缓存，提供的输出缓冲区大小
	stConvertParam.nWidth = stImageInfo.nWidth;
	stConvertParam.nHeight = stImageInfo.nHeight;
	stConvertParam.pSrcData = pData;
	stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
	stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
	stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
	stConvertParam.pDstBuffer = pDataForRGB;
	stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
	nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
	if (MV_OK != nRet)
	{
		printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
		shutdown();
		return false;
	}

	return true;
}

bool HkCam::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

// int main(int argc, char **argv)
// {
// 	HkCam a;
// 	a.start();
// 	a.grab_rgb_image();
	
// }