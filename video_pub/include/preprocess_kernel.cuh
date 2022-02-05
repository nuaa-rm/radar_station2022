#ifndef PREPROCESS_KERNEL_CUH
#define PREPROCESS_KERNEL_CUH
#include <cuda.h>
#include <cuda_runtime.h>
#include<cuda_runtime_api.h>
#include <math.h>
void preKernelWrapper(uchar3* d_rawImg, uchar3* d_resizeImg,
                      int rows,int cols, unsigned char threthold, bool is_tgt_red,
                      unsigned char* redChannel,
                      unsigned char* greenChannel,
                      unsigned char* blueChannel,
                      unsigned char* d_monoImg, // compare result
                      unsigned char* d_compImg //bianry after threth
                      );

#endif // PREPROCESS_KERNEL_CUH
