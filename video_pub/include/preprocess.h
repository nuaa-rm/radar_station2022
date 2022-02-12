#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <cuda.h>
#include <cuda_runtime.h>
#include<cuda_runtime_api.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "gpu_timer.h"
static void hdlCudaErr(cudaError err, const char* const func, const char* const file, const int line)
{
    if (err != cudaSuccess)
    {
        std::cerr << "CUDA error at: " << file << ":" << line << std::endl;
        std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
        exit(1);
    }
}
#define CUDA_ALLOC_DEV_MEM(devPtr,size) hdlCudaErr(cudaMalloc(devPtr,size), __FUNCTION__, __FILE__, __LINE__)
#define CUDA_MEMCPY_H2D(dst,src,count) hdlCudaErr(cudaMemcpy(dst,src,count,cudaMemcpyHostToDevice), __FUNCTION__, __FILE__, __LINE__)
#define CUDA_MEMCPY_D2H(dst,src,count) hdlCudaErr(cudaMemcpy(dst,src,count,cudaMemcpyDeviceToHost), __FUNCTION__, __FILE__, __LINE__)
#define CUDA_FREE_DEV_MEM(devPtr) hdlCudaErr(cudaFree(devPtr), __FUNCTION__, __FILE__, __LINE__)
#define CUDA_DEV_MEMSET(devPtr,value,count) hdlCudaErr(cudaMemset(devPtr,value,count), __FUNCTION__, __FILE__, __LINE__)

class cuda_proc
{
public:
    cuda_proc(int rows,int cols);
    ~cuda_proc();
    void alloc_mem( int rows, int cols);
    void reAlloc_mem( int newRows,  int newCols);
    void free_mem();
    void mat2Dmem(cv::Mat &inputImg);
    void proc_update(cv::Mat &inputImg);
public:
    int _rows,_cols;
    uchar3 *h_inputBGR,*d_inputBGR;
    uchar3 *h_resizeBGR, *d_resizeBGR;
    unsigned char *d_redChannel, *d_greenChannel, *d_blueChannel;
    unsigned char *h_redChannel, *h_greenChannel, *h_blueChannel;
    unsigned char *h_compImg, *d_compImg;
    unsigned char *h_monoImg, *d_monoImg;
    size_t input_size,resize_size,mono_size;
    cv::Mat resizeImg,compImg,monoImg,red_ch,green_ch,blue_ch;

};


#endif // PREPROCESS_H
