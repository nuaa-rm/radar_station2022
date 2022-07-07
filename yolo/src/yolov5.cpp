#include <iostream>
#include <chrono>
#include "yolo/cuda_utils.h"
#include "yolo/logging.h"
#include "yolo/common.hpp"
#include "yolo/calibrator.h"
#include "yolo/preprocess.h"
#include <ros/ros.h>
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
#define BATCH_SIZE 1
#define MAX_IMAGE_INPUT_SIZE_THRESH 3000 * 3000 // ensure it exceed the maximum size in the input images !
//using namespace std;
//using namespace cv;
// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) +
                               1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char *INPUT_BLOB_NAME = "data";
const char *OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;
ros::Publisher far_rectangles;
ros::Publisher close_rectangles;

//std::vector<radar_msgs::points> car_points;//the 2 points of a rectangle, saved in one member of the car_points
void far_imageCB(const sensor_msgs::ImageConstPtr &msg);//ake car detection and send the rect points
void close_imageCB(const sensor_msgs::ImageConstPtr &msg);//ake car detection and send the rect points
radar_msgs::yolo_points rect2msg(std::vector<Yolo::Detection> yolo_detection, cv::Mat &img);

static float prob[BATCH_SIZE * OUTPUT_SIZE];
IExecutionContext *context;
float *buffers[2];
ICudaEngine *engine;
IRuntime *runtime;
cudaStream_t stream;
uint8_t *img_host = nullptr;
uint8_t *img_device = nullptr;
int inputIndex;
int outputIndex;
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
radar_msgs::yolo_points
rect2msg(std::vector<Yolo::Detection> yolo_detection, cv::Mat &img) {
    radar_msgs::yolo_points msg_it;
    radar_msgs::yolo_point rect_point;
    for (std::vector<Yolo::Detection>::iterator it = yolo_detection.begin(); it != yolo_detection.end(); it++) {
        cv::Rect r = get_rect(img, it->bbox);
        msg_it.id = it->class_id;
        rect_point.color=it->class_id;
        rect_point.x = (int) r.x;
        rect_point.y = (int) r.y;
        rect_point.width = (int) r.width;
        rect_point.height = (int) r.height;
        msg_it.data.push_back(rect_point);
        std::cout << r << std::endl;
        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 3);
        std::string color;
        if(rect_point.color==1)color="blue";
        else color="red";
        cv::putText(img, color, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.5,
                    cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    return msg_it;
}

int main(int argc, char **argv) {
    cudaSetDevice(DEVICE);

    std::string wts_name = "";
    std::string engine_name = std::string(PACK_PATH) + "/yolov5s.engine";
    // deserialize the .engine and run inference
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        return -1;
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();

    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void **) &buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **) &buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));

    // Create stream
    CUDA_CHECK(cudaStreamCreate(&stream));
    // prepare input data cache in pinned memory 
    CUDA_CHECK(cudaMallocHost((void **) &img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3));
    // prepare input data cache in device memory
    CUDA_CHECK(cudaMalloc((void **) &img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3));


    ros::init(argc, argv, "yolov5");
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
    ros::Rate loop_rate(30);
    ros::spin();

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(img_device));
    CUDA_CHECK(cudaFreeHost(img_host));
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();


    return 0;
}

void far_imageCB(
        const sensor_msgs::ImageConstPtr &msg
) {
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    int fcount = 1;
    std::vector<cv::Mat> imgs_buffer(BATCH_SIZE);
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
    std::vector<Yolo::Detection> new_res;
    auto &res = batch_res[0];

    //识别出的车辆坐标被保存至res
    nms(res, &prob[0], CONF_THRESH, NMS_THRESH);
//    for (int i = 0; i < res.size(); i++) {
//        if (((int) res[i].class_id) != btl_number) {
//            new_res.push_back(res[i]);
//        }
//    }
    //将识别得到的目标框出并发送ROS消息
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
    cv::waitKey(1);
}

void close_imageCB(
        const sensor_msgs::ImageConstPtr &msg
) {
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    int fcount = 1;
    std::vector<cv::Mat> imgs_buffer(BATCH_SIZE);
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
    std::vector<Yolo::Detection> new_res;
    auto &res = batch_res[0];
    //识别出的车辆坐标被保存至res
    nms(res, &prob[0], CONF_THRESH, NMS_THRESH);
    //remove our own cars
//    for (int i = 0; i < res.size(); i++) {
//        if (((int) res[i].class_id) != btl_number) {
//            new_res.push_back(res[i]);
//        }
//    }

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
}

