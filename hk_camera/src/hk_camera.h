#ifndef HK_CAM_H
#define HK_CAM_H

#include "MvCameraControl.h"

class HkCam
{
public:
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	void *handle;
	unsigned char *pData;
	unsigned char *pDataForRGB;
	int nRet;
	MVCC_INTVALUE stParam;
	MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
	
	HkCam();
	~HkCam();
	void start();
	void shutdown();
	bool grab_rgb_image();
private:
	bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
};

#endif
