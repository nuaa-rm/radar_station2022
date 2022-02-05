
#ifndef MVCAMERA_MVCAMERA_H
#define MVCAMERA_MVCAMERA_H

#include "CameraApi.h"  //相机SDK头文件

#include <thread>
#include <mutex>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MVCamera {
public:
  int    Init(int id = 0);
  int    Play();
  int    Read();
  int    GetFrame(Mat& frame,bool is_color);
  int    Stop();
  int    Uninit();
  int    SetExposureTime(bool auto_exp, double exp_time = 10000);
  double GetExposureTime();
  int    SetLargeResolution(bool if_large_resolution);
  Size   GetResolution();
  int    SetGain(double gain);
  double GetGain();
  int    SetWBMode(bool auto_wb = true);
  int    GetWBMode(bool & auto_wb);
  int    SetOnceWB();
  int GetFrame_B(Mat &frame,bool is_color);
  int Set_fps(int fps_mode);


  int                     iCameraCounts;
  int                     iStatus;
  int                     hCamera;
  int                     channel;

  //static tSdkCameraDevInfo       tCameraEnumList[4];
  tSdkCameraCapbility     tCapability;      //设备描述信息
  tSdkFrameHead           sFrameInfo;
  unsigned char           *	pbyBuffer=NULL;
  unsigned char           * g_pRgbBuffer[2];     //处理后数据缓存区
  int                     ready_buffer_id;
  bool                    stopped;
  bool                    updated;
  bool                    started=0;
  mutex                   mutex1;
  IplImage *iplImage = NULL;


  MVCamera();
};


#endif //MVCAMERA_MVCAMERA_H
