/*************************************************************************
  > File Name: preprocess.cpp
  > Author: cyz
  > Mail:
  > Created Time: Sun 24 Feb 2019 05:08:34 PM
 ************************************************************************/

#include "preprocess.h"
#include "preprocess_kernel.cuh"


cuda_proc::cuda_proc(int rows,int cols):_rows(rows),_cols(cols)
{
    alloc_mem(rows,cols);
}
cuda_proc::~cuda_proc()
{
    free_mem();
}
void cuda_proc::alloc_mem(int rows, int cols)
{
    //input
    input_size=rows*cols*sizeof(uchar3);
    CUDA_ALLOC_DEV_MEM(&d_inputBGR,input_size);
//output
    if(rows==1024||cols==1280)
    {
        //need resize
        rows=512;
        cols=640;
        resize_size=rows*cols*sizeof(uchar3);
        CUDA_ALLOC_DEV_MEM(&d_resizeBGR,resize_size);
        //create mat
        resizeImg.create(rows,cols,CV_8UC3);
    }

    mono_size=rows*cols*sizeof(unsigned char);
    CUDA_ALLOC_DEV_MEM(&d_blueChannel,mono_size);
    CUDA_ALLOC_DEV_MEM(&d_redChannel,mono_size);
    CUDA_ALLOC_DEV_MEM(&d_greenChannel,mono_size);
    CUDA_ALLOC_DEV_MEM(&d_compImg,mono_size);
//    CUDA_ALLOC_DEV_MEM(&d_monoImg,mono_size);
//creat mat
    compImg.create(rows,cols,CV_8UC1);
    monoImg.create(rows,cols,CV_8UC1);
    red_ch.create(rows,cols,CV_8UC1);
    green_ch.create(rows,cols,CV_8UC1);
    blue_ch.create(rows,cols,CV_8UC1);
    //point host ptr  to  output mat
    h_resizeBGR=(uchar3*)resizeImg.ptr<unsigned char>(0);
    h_compImg=(unsigned char*)compImg.ptr<unsigned char>(0);
    h_monoImg=(unsigned char*)monoImg.ptr<unsigned char>(0);
    h_blueChannel=(unsigned  char*)blue_ch.ptr<unsigned char>(0);
    h_greenChannel=(unsigned  char*)green_ch.ptr<unsigned char>(0);
    h_redChannel=(unsigned  char*)red_ch.ptr<unsigned char>(0);

    printf("GPU initalize done  \n");
}
void cuda_proc::reAlloc_mem(int newRows, int newCols)
{
      free_mem();
      _cols=newCols;
      _rows=newRows;
      alloc_mem(newRows,newCols);
}
void cuda_proc::free_mem()
{


     CUDA_FREE_DEV_MEM(d_inputBGR);
     if(_rows==1024||_cols==1280)
         CUDA_FREE_DEV_MEM(d_resizeBGR);
     CUDA_FREE_DEV_MEM(d_blueChannel);
     CUDA_FREE_DEV_MEM(d_redChannel);
     CUDA_FREE_DEV_MEM(d_greenChannel);
     CUDA_FREE_DEV_MEM(d_monoImg);
     CUDA_FREE_DEV_MEM(d_compImg);

     printf(" GPU uninitialized! \n");

}
void cuda_proc::mat2Dmem(cv::Mat &inputImg)
{
    h_inputBGR=(uchar3*)inputImg.ptr<unsigned char>(0);
     CUDA_MEMCPY_H2D(d_inputBGR,h_inputBGR,input_size);
}
///
/// \brief cuda_proc::proc_update
///  resize , split channel and take threthold
/// \param inputImg
///
void cuda_proc::proc_update(cv::Mat &inputImg)
{
    if(inputImg.rows!=_rows||inputImg.cols!=_cols)
    {
        reAlloc_mem(inputImg.rows,inputImg.cols);
        printf("GPU OVER FLOW! \n");
    }

    //update
    mat2Dmem(inputImg);
    // go to .cu file to see kernel functions
    preKernelWrapper(d_inputBGR,d_resizeBGR,_rows,_cols,70,true,
                     d_redChannel,d_greenChannel,d_blueChannel,
                     d_monoImg,d_compImg);
    //output

    CUDA_MEMCPY_D2H(h_resizeBGR,d_resizeBGR,resize_size);
    CUDA_MEMCPY_D2H(h_monoImg,d_monoImg,mono_size);
    CUDA_MEMCPY_D2H(h_compImg,d_compImg,mono_size);
    CUDA_MEMCPY_D2H(h_redChannel,d_redChannel,mono_size);
    CUDA_MEMCPY_D2H(h_greenChannel,d_greenChannel,mono_size);
    CUDA_MEMCPY_D2H(h_blueChannel,d_blueChannel,mono_size);
}
