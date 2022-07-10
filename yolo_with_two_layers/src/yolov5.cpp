#include <iostream>
#include <chrono>
#include <cmath>
#include "yolo_with_two_layers/cuda_utils.h"
#include "yolo_with_two_layers/logging.h"
#include "yolo_with_two_layers/common.hpp"
#include "yolo_with_two_layers/utils.h"
#include "yolo_with_two_layers/calibrator.h"
#include "yolo_with_two_layers/preprocess.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <radar_msgs/yolo_point.h>
#include <radar_msgs/yolo_points.h>


#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE_NUMBER 12
#define BATCH_SIZE_CAR 1
#define MAX_IMAGE_INPUT_SIZE_THRESH 3000 * 3000 // ensure it exceed the maximum size in the input images !

// stuff we know about the network and the input/output blobs
static const int INPUT_H_NUMBER = Yolo::INPUT_H_NUMBER;
static const int INPUT_W_NUMBER = Yolo::INPUT_W_NUMBER;
static const int INPUT_H_CAR = Yolo::INPUT_H_CAR;
static const int INPUT_W_CAR = Yolo::INPUT_W_CAR;
static const int CLASS_NUM_NUMBER = Yolo::CLASS_NUM_NUMBER;
static const int CLASS_NUM_CAR = Yolo::CLASS_NUM_CAR;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) +1;
// we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char *INPUT_BLOB_NAME = "data";
const char *OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;
ros::Publisher far_rectangles;
ros::Publisher close_rectangles;

//std::vector<radar_msgs::points> car_points;//the 2 points of a rectangle, saved in one member of the car_points
void far_imageCB(const sensor_msgs::ImageConstPtr &msg);//ake car detection and send the rect points
void close_imageCB(const sensor_msgs::ImageConstPtr &msg);//ake car detection and send the rect points
void rect2msg(std::vector<Yolo::Detection>::iterator it, std::vector<radar_msgs::yolo_points>::iterator msg_it, cv::Mat &img);

static float prob_number[BATCH_SIZE_NUMBER * OUTPUT_SIZE];
static float prob_car[BATCH_SIZE_CAR * OUTPUT_SIZE];
IExecutionContext *context_number;
IExecutionContext *context_car;
float *buffers_number[2];
float *buffers_car[2];
ICudaEngine *engine_number;
ICudaEngine *engine_car;
IRuntime *runtime;
cudaStream_t stream_number;
cudaStream_t stream_car;
uint8_t *img_host_number = nullptr;
uint8_t *img_device_number = nullptr;
uint8_t *img_host_car = nullptr;
uint8_t *img_device_car = nullptr;
int inputIndex_number;
int outputIndex_number;
int inputIndex_car;
int outputIndex_car;

std::string btl_color = "blue";
int btl_number = 1;

void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *output, int batchSize) {
    // infer on the batch asynchronously, and DMA output back to host
    //CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);

    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost,
                               stream));
    cudaStreamSynchronize(stream);
}

int main(int argc, char **argv) {
    cudaSetDevice(DEVICE);

    std::string engine_name_car = std::string(PACK_PATH) + "/yolov5s_car.engine";
    std::string engine_name_number = std::string(PACK_PATH) + "/yolov5s_number.engine";

    // deserialize the .engine and run inference
    std::ifstream file_car(engine_name_car, std::ios::binary);
    if (!file_car.good())
    {
        std::cerr << "read " << engine_name_car << " error!" << std::endl;
        return -1;
    }
    std::ifstream file_number(engine_name_number, std::ios::binary);
    if (!file_number.good())
    {
        std::cerr << "read " << engine_name_number << " error!" << std::endl;
        return -1;
    }

    char *trtModelStream = nullptr;
    size_t size_number = 0;
    file_number.seekg(0, file_number.end);
    size_number = file_number.tellg();
    file_number.seekg(0, file_number.beg);
    trtModelStream = new char[size_number];
    assert(trtModelStream);
    file_number.read(trtModelStream, size_number);
    file_number.close();

    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine_number = runtime->deserializeCudaEngine(trtModelStream, size_number);
    assert(engine_number != nullptr);
    context_number = engine_number->createExecutionContext();
    assert(context_number != nullptr);
    delete[] trtModelStream;
    assert(engine_number->getNbBindings() == 2);

    size_t size_car = 0;
    file_car.seekg(0, file_car.end);
    size_car = file_car.tellg();
    file_car.seekg(0, file_car.beg);
    trtModelStream = new char[size_car];
    assert(trtModelStream);
    file_car.read(trtModelStream, size_car);
    file_car.close();

    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine_car = runtime->deserializeCudaEngine(trtModelStream, size_car);
    assert(engine_car != nullptr);
    context_car = engine_car->createExecutionContext();
    assert(context_car != nullptr);
    delete[] trtModelStream;
    assert(engine_car->getNbBindings() == 2);

    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    inputIndex_number = engine_number->getBindingIndex(INPUT_BLOB_NAME);
    outputIndex_number = engine_number->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex_number == 0);
    assert(outputIndex_number == 1);

    inputIndex_car = engine_car->getBindingIndex(INPUT_BLOB_NAME);
    outputIndex_car = engine_car->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex_car == 0);
    assert(outputIndex_car == 1);

    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void **) &buffers_number[inputIndex_number], BATCH_SIZE_NUMBER * 3 * INPUT_H_NUMBER * INPUT_W_NUMBER * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **) &buffers_number[outputIndex_number], BATCH_SIZE_NUMBER * OUTPUT_SIZE * sizeof(float)));

    CUDA_CHECK(cudaMalloc((void **) &buffers_car[inputIndex_car], BATCH_SIZE_CAR * 3 * INPUT_H_CAR * INPUT_W_CAR * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **) &buffers_car[outputIndex_car], BATCH_SIZE_CAR * OUTPUT_SIZE * sizeof(float)));

    // Create stream
    CUDA_CHECK(cudaStreamCreate(&stream_car));
    CUDA_CHECK(cudaStreamCreate(&stream_number));

    // prepare input data cache in pinned memory
    CUDA_CHECK(cudaMallocHost((void **) &img_host_car, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
    CUDA_CHECK(cudaMallocHost((void **) &img_host_number, MAX_IMAGE_INPUT_SIZE_THRESH * 3));

    // prepare input data cache in device memory
    CUDA_CHECK(cudaMalloc((void **) &img_device_number, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
    CUDA_CHECK(cudaMalloc((void **) &img_device_car, MAX_IMAGE_INPUT_SIZE_THRESH * 3));


    ros::init(argc, argv, "yolov5_with_two_layers");
    ros::param::get("/battle_color", btl_color);
    if (btl_color == "red")btl_number = 0;
    else btl_number = 1;
    ros::start();
    ros::NodeHandle n;

//    ros::Subscriber farImageSub = n.subscribe("/sensor_far/image_raw", 1, &imageCB);
    ros::Subscriber farImageSub = n.subscribe("/sensor_far/image_raw", 1, &far_imageCB);
    ros::Subscriber closeImageSub = n.subscribe("/sensor_close/image_raw", 1, &close_imageCB);

    //发布识别到的目标坐标
    far_rectangles = n.advertise<radar_msgs::yolo_points>("far_rectangles", 1);
    close_rectangles = n.advertise<radar_msgs::yolo_points>("close_rectangles", 1);
    //ros::Rate loop_rate(30);
    ros::spin();

    // Release stream and buffers
    cudaStreamDestroy(stream_car);
    CUDA_CHECK(cudaFree(img_device_car));
    CUDA_CHECK(cudaFreeHost(img_host_car));
    CUDA_CHECK(cudaFree(buffers_car[inputIndex_car]));
    CUDA_CHECK(cudaFree(buffers_car[outputIndex_car]));

    cudaStreamDestroy(stream_number);
    CUDA_CHECK(cudaFree(img_device_number));
    CUDA_CHECK(cudaFreeHost(img_host_number));
    CUDA_CHECK(cudaFree(buffers_number[inputIndex_number]));
    CUDA_CHECK(cudaFree(buffers_number[outputIndex_number]));

    // Destroy the engine
    context_car->destroy();
    engine_car->destroy();

    context_number->destroy();
    engine_number->destroy();
    runtime->destroy();

    return 0;
}

void far_imageCB(const sensor_msgs::ImageConstPtr &msg)
{
    auto start = std::chrono::system_clock::now();
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    float *buffer_idx_car = (float *) buffers_car[inputIndex_car];
    if (img.empty())
    {
        printf("img empty!!!!\t");
        assert(!img.empty());
    }

    cv::Mat img_raw;
    img.copyTo(img_raw);
    size_t size_image = img.cols * img.rows * 3;
    size_t size_images_dst = INPUT_H_CAR * INPUT_W_CAR * 3;
    //copy data to pinned memory
    memcpy(img_host_car, img.data, size_image);
    //copy data to device memory
    CUDA_CHECK(cudaMemcpyAsync(img_device_car, img_host_car, size_image, cudaMemcpyHostToDevice, stream_car));
    preprocess_kernel_img(img_device_car, img.cols, img.rows, buffer_idx_car, INPUT_W_CAR, INPUT_H_CAR, stream_car);
    buffer_idx_car += size_images_dst;
    std::vector<Yolo::Detection> res_car;
    // Run inference
    doInference(*context_car, stream_car, (void **) buffers_car, prob_car, BATCH_SIZE_CAR);
    nms(res_car, &prob_car[0], CONF_THRESH, NMS_THRESH);
    //识别出的车辆坐标被保存至res
    int fcount = 0;
    std::vector<cv::Mat>imgs_buffer(BATCH_SIZE_NUMBER);
    for (size_t j = 0; j < res_car.size(); j++)
    {
        fcount ++;
        if(fcount < BATCH_SIZE_NUMBER && j + 1 != res_car.size())
        {
            continue;
        }
        float *buffer_idx_number = (float *) buffers_number[inputIndex_number];
        for(int b = 0; b < fcount; b++)
        {
            cv::Rect r = get_rect_car(img, res_car[j - fcount + 1 + b].bbox);
            cv::Mat roi;
            if(r.x < 0)
            {
                r.x = 0;
            }
            else if(r.y < 0)
            {
                r.y = 0;
            }
            else if((r.x + r.width) > img.cols)
            {
                r.width = img.cols - r.x;
            }
            else if((r.y + r.height) > img.rows)
            {
                r.height = img.rows - r.y;
            }
            img_raw(r).copyTo(roi);
            if(roi.empty())
            {
                continue;
            }
            imgs_buffer[b] = roi;
            size_t size_roi = roi.cols * roi.rows * 3;
            size_t size_roi_dst = INPUT_H_NUMBER * INPUT_W_NUMBER * 3;
            //copy data to pinned memory
            memcpy(img_host_number, roi.data, size_roi);
            //copy data to device memory
            CUDA_CHECK(cudaMemcpyAsync(img_device_number, img_host_number, size_roi, cudaMemcpyHostToDevice, stream_number));
            preprocess_kernel_img(img_device_number, roi.cols, roi.rows, buffer_idx_number, INPUT_W_NUMBER, INPUT_H_NUMBER, stream_number);

            // Run inference
            buffer_idx_number += size_roi_dst;
        }

        doInference(*context_number, stream_number, (void **) buffers_number, prob_number, BATCH_SIZE_NUMBER);
        std::vector<std::vector<Yolo::Detection>>batch_res_number(fcount);
        for(int b = 0; b < fcount; b++)
        {
            auto& res = batch_res_number[b];
            nms(res, &prob_number[b*OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
        }
        for(int b = 0; b < fcount; b++)
        {
            auto& res = batch_res_number[b];
            cv::Rect rr = get_rect_car(img_raw, res_car[j - fcount + 1 + b].bbox);
            for(size_t m = 0; m < res.size(); m++)
            {
                cv::Rect number_in_roi = get_rect_number(imgs_buffer[b], res[m].bbox);
                cv::Rect number_in_img = number_in_roi;
                number_in_img.x += rr.x;
                number_in_img.y += rr.y;
                cv::rectangle(img, number_in_img, cv::Scalar(0x27, 0xC1, 0x36), 1);
                if((int)res[m].class_id <= 4)
                {
                    cv::putText(img, std::string("red") + std::to_string((int)res[m].class_id + 1), cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0xFF, 0xFF, 0xFF), 1);
                }
                else if((int)res[m].class_id == 5)
                {
                    cv::putText(img, "red guard", cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0xFF, 0xFF, 0xFF), 1);
                }
                else if((int)res[m].class_id == 11)
                {
                    cv::putText(img, "blue guard", cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0xFF, 0xFF, 0xFF), 1);
                }
                else
                {
                    cv::putText(img, std::string("blue") + std::to_string((int)res[m].class_id - 5), cv::Point(number_in_img.x, number_in_img.y - 1), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0xFF, 0xFF, 0xFF), 1);
                }
            }
            cv::rectangle(img, rr, cv::Scalar(0x27, 0xC1, 0x36), 2);
            if((int)res_car[j].class_id == 0)
            {
                cv::putText(img, "red", cv::Point(rr.x, rr.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
            else
            {
                cv::putText(img, "blue", cv::Point(rr.x, rr.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
        }
        fcount = 0;
    }

    /*//将识别得到的目标框出并发送ROS消息
    if (res.size() != 0) {
        radar_msgs::yolo_points rect_msg = rect2msg(res, img);
        far_rectangles.publish(rect_msg);
    } else {
        radar_msgs::yolo_points rect_msg;
        rect_msg.text = "none";
        far_rectangles.publish(rect_msg);
    }
    cv::resize(img, img, cv::Size(640, 512));
    cv::imshow("yolo_far", img);
    cv::waitKey(1);*/
    auto end = std::chrono::system_clock::now();
    std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    cv::imshow("yolo_far", img);
    cv::waitKey(1);
}

void close_imageCB(
        const sensor_msgs::ImageConstPtr &msg
) {
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
/*
    int fcount = 1;
    float *buffer_idx = (float *) buffers[inputIndex];
    if (img.empty()) {
        printf("img empty!!!!\t");
        assert(!img.empty());
    }

    cv::Mat img_raw;
    img.copyTo(img_raw);
    size_t size_image = img.cols * img.rows * 3;
    // size_t  size_image_dst = INPUT_H * INPUT_W * 3;
    //copy data to pinned memory
    memcpy(img_host, img.data, size_image);
    //copy data to device memory
    CUDA_CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
    preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx, INPUT_W, INPUT_H, stream);

    // Run inference
    auto start = std::chrono::system_clock::now();
    doInference(*context, stream, (void **) buffers, prob, BATCH_SIZE);
    auto end = std::chrono::system_clock::now();
//    std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//              << "ms" << std::endl;
    std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
    auto &res = batch_res[0];
    //识别出的车辆坐标被保存至res
    nms(res, &prob[0], CONF_THRESH, NMS_THRESH);

    //将识别得到的目标框出并发送ROS消息
    if (res.size() != 0) {
        radar_msgs::yolo_points rect_msg = rect2msg(res, img);
        close_rectangles.publish(rect_msg);
    } else {
        radar_msgs::yolo_points rect_msg;
        rect_msg.text = "none";
        close_rectangles.publish(rect_msg);
    }
    cv::resize(img, img, cv::Size(640, 512));
    cv::imshow("yolo_close", img);
    cv::waitKey(1);
*/
}

