#include "project.h"
#include <cuda_runtime_api.h>
#include <cstring>

#define TASKS_IN_PER_THREAD 10
#define THREADS_PER_SM 1022
#define USE_SM 2

float* data_p;
float* output_p;

__device__ constexpr unsigned int dataLengthPerTask(unsigned int k) {
    return k * TASKS_IN_PER_THREAD;
}

constexpr unsigned int maxMemory() {
    return TASKS_IN_PER_THREAD * THREADS_PER_SM * USE_SM * 4 * sizeof(float);
}

constexpr unsigned int maxTasks() {
    return TASKS_IN_PER_THREAD * THREADS_PER_SM * USE_SM;
}

__global__ void getTheoreticalUV(const float* data, int n, const Mat33& cam_matrix, const Mat34& uni_matrix, int cols, float* output) {
    float data_s[dataLengthPerTask(4)];
    unsigned int index = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int index1 = index * 4;
    for (unsigned int i = 0; i < TASKS_IN_PER_THREAD && index + i < n; i += 1) {
        data_s[i * 4] = data[index1 + i * 4];
        data_s[i * 4 + 1] = data[index1 + i * 4 + 1];
        data_s[i * 4 + 2] = data[index1 + i * 4 + 2];
        data_s[i * 4 + 3] = data[index1 + i * 4 + 3];
    }
    for (unsigned int i = 0; i < TASKS_IN_PER_THREAD && index + i < n; i++) {
        vec4 worldPosition;
        worldPosition << data_s[i * 4], data_s[i * 4 + 1], data_s[i * 4 + 2], data_s[i * 4 + 3];
        vec3 res = cam_matrix * uni_matrix * worldPosition;
        int col = (int)round(res(1) / res(3));
        int row = (int)round(res(2) / res(3));
        int offset = (row * cols * 3) + (col * 3);
        output[offset] = data_s[i * 4];
        output[offset + 1] = data_s[i * 4 + 1];
        output[offset + 2] = data_s[i * 4 + 2];
    }
}

void projectInit() {
    cudaMalloc((void**)&data_p, maxMemory());
    cudaMalloc((void**)&output_p, 1280 * 1024 * 3 * sizeof(float));
}

void deInit() {
    cudaFree(data_p);
    cudaFree(output_p);
}

void projectPoints(const pcp& input, const Mat33& cam_matrix, const Mat34& uni_matrix, int cols, int rows, float* output) {
    if (input->empty()) {
        memset(output, 0, cols * rows * 3 * sizeof(float));
        return;
    }
    cudaMemcpy(data_p, input->points.data(), input->size() * sizeof(float) * 4, cudaMemcpyHostToDevice);
    cudaMemset(output_p, 0, cols * rows * 3 * sizeof(float));
    int threads = (int)input->size() / TASKS_IN_PER_THREAD;
    for (int i = 0; i <= threads / maxTasks(); i++) {
        int tasks_this = maxTasks();
        if (i == threads / maxTasks()) {
            tasks_this = (int)(input->size()) % maxTasks();
        }
        dim3 threadsPerBlock(tasks_this / 2 + 1);
        dim3 numBlocks(USE_SM);
        getTheoreticalUV<<<numBlocks, threadsPerBlock>>>(data_p + i * TASKS_IN_PER_THREAD, tasks_this, cam_matrix, uni_matrix, cols, output_p);
    }
    cudaMemcpy(output, output_p, cols * rows * 3 * sizeof(float), cudaMemcpyDeviceToHost);
}


