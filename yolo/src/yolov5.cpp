#include <iostream>
#include <chrono>
#include <cmath>
#include "yolo/cuda_utils.h"
#include "yolo/logging.h"
#include "yolo/common.hpp"
#include "yolo/utils.h"
#include "yolo/calibrator.h"
#include "yolo/preprocess.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <radar_msgs/point.h>
#include <radar_msgs/points.h>

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
void
rect2msg(std::vector<Yolo::Detection>::iterator it, std::vector<radar_msgs::points>::iterator msg_it, cv::Mat &img);

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

static int get_width(int x, float gw, int divisor = 8) {
    return int(ceil((x * gw) / divisor)) * divisor;
}

static int get_depth(int x, float gd) {
    if (x == 1) return 1;
    int r = round(x * gd);
    if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0) {
        --r;
    }
    return std::max<int>(r, 1);
}

ICudaEngine *
build_engine(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt, float &gd, float &gw,
             std::string &wts_name) {
    INetworkDefinition *network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
    assert(data);
    std::map<std::string, Weights> weightMap = loadWeights(wts_name);
    /* ------ yolov5 backbone------ */
    auto conv0 = convBlock(network, weightMap, *data, get_width(64, gw), 6, 2, 1, "model.0");
    assert(conv0);
    auto conv1 = convBlock(network, weightMap, *conv0->getOutput(0), get_width(128, gw), 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), get_width(128, gw), get_width(128, gw),
                              get_depth(3, gd), true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), get_width(256, gw), 3, 2, 1, "model.3");
    auto bottleneck_csp4 = C3(network, weightMap, *conv3->getOutput(0), get_width(256, gw), get_width(256, gw),
                              get_depth(6, gd), true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_csp4->getOutput(0), get_width(512, gw), 3, 2, 1, "model.5");
    auto bottleneck_csp6 = C3(network, weightMap, *conv5->getOutput(0), get_width(512, gw), get_width(512, gw),
                              get_depth(9, gd), true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_csp6->getOutput(0), get_width(1024, gw), 3, 2, 1, "model.7");
    auto bottleneck_csp8 = C3(network, weightMap, *conv7->getOutput(0), get_width(1024, gw), get_width(1024, gw),
                              get_depth(3, gd), true, 1, 0.5, "model.8");
    auto spp9 = SPPF(network, weightMap, *bottleneck_csp8->getOutput(0), get_width(1024, gw), get_width(1024, gw), 5,
                     "model.9");
    /* ------ yolov5 head ------ */
    auto conv10 = convBlock(network, weightMap, *spp9->getOutput(0), get_width(512, gw), 1, 1, 1, "model.10");

    auto upsample11 = network->addResize(*conv10->getOutput(0));
    assert(upsample11);
    upsample11->setResizeMode(ResizeMode::kNEAREST);
    upsample11->setOutputDimensions(bottleneck_csp6->getOutput(0)->getDimensions());

    ITensor *inputTensors12[] = {upsample11->getOutput(0), bottleneck_csp6->getOutput(0)};
    auto cat12 = network->addConcatenation(inputTensors12, 2);
    auto bottleneck_csp13 = C3(network, weightMap, *cat12->getOutput(0), get_width(1024, gw), get_width(512, gw),
                               get_depth(3, gd), false, 1, 0.5, "model.13");
    auto conv14 = convBlock(network, weightMap, *bottleneck_csp13->getOutput(0), get_width(256, gw), 1, 1, 1,
                            "model.14");

    auto upsample15 = network->addResize(*conv14->getOutput(0));
    assert(upsample15);
    upsample15->setResizeMode(ResizeMode::kNEAREST);
    upsample15->setOutputDimensions(bottleneck_csp4->getOutput(0)->getDimensions());

    ITensor *inputTensors16[] = {upsample15->getOutput(0), bottleneck_csp4->getOutput(0)};
    auto cat16 = network->addConcatenation(inputTensors16, 2);

    auto bottleneck_csp17 = C3(network, weightMap, *cat16->getOutput(0), get_width(512, gw), get_width(256, gw),
                               get_depth(3, gd), false, 1, 0.5, "model.17");

    /* ------ detect ------ */
    IConvolutionLayer *det0 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                        DimsHW{1, 1}, weightMap["model.24.m.0.weight"],
                                                        weightMap["model.24.m.0.bias"]);
    auto conv18 = convBlock(network, weightMap, *bottleneck_csp17->getOutput(0), get_width(256, gw), 3, 2, 1,
                            "model.18");
    ITensor *inputTensors19[] = {conv18->getOutput(0), conv14->getOutput(0)};
    auto cat19 = network->addConcatenation(inputTensors19, 2);
    auto bottleneck_csp20 = C3(network, weightMap, *cat19->getOutput(0), get_width(512, gw), get_width(512, gw),
                               get_depth(3, gd), false, 1, 0.5, "model.20");
    IConvolutionLayer *det1 = network->addConvolutionNd(*bottleneck_csp20->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                        DimsHW{1, 1}, weightMap["model.24.m.1.weight"],
                                                        weightMap["model.24.m.1.bias"]);
    auto conv21 = convBlock(network, weightMap, *bottleneck_csp20->getOutput(0), get_width(512, gw), 3, 2, 1,
                            "model.21");
    ITensor *inputTensors22[] = {conv21->getOutput(0), conv10->getOutput(0)};
    auto cat22 = network->addConcatenation(inputTensors22, 2);
    auto bottleneck_csp23 = C3(network, weightMap, *cat22->getOutput(0), get_width(1024, gw), get_width(1024, gw),
                               get_depth(3, gd), false, 1, 0.5, "model.23");
    IConvolutionLayer *det2 = network->addConvolutionNd(*bottleneck_csp23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5),
                                                        DimsHW{1, 1}, weightMap["model.24.m.2.weight"],
                                                        weightMap["model.24.m.2.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, "model.24", std::vector<IConvolutionLayer *>{det0, det1, det2});
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));
    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2* calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto &mem: weightMap) {
        free((void *) (mem.second.values));
    }

    return engine;
}

ICudaEngine *
build_engine_p6(unsigned int maxBatchSize, IBuilder *builder, IBuilderConfig *config, DataType dt, float &gd, float &gw,
                std::string &wts_name) {
    INetworkDefinition *network = builder->createNetworkV2(0U);
    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor *data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{3, INPUT_H, INPUT_W});
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights(wts_name);

    /* ------ yolov5 backbone------ */
    auto conv0 = convBlock(network, weightMap, *data, get_width(64, gw), 6, 2, 1, "model.0");
    auto conv1 = convBlock(network, weightMap, *conv0->getOutput(0), get_width(128, gw), 3, 2, 1, "model.1");
    auto c3_2 = C3(network, weightMap, *conv1->getOutput(0), get_width(128, gw), get_width(128, gw), get_depth(3, gd),
                   true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *c3_2->getOutput(0), get_width(256, gw), 3, 2, 1, "model.3");
    auto c3_4 = C3(network, weightMap, *conv3->getOutput(0), get_width(256, gw), get_width(256, gw), get_depth(6, gd),
                   true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *c3_4->getOutput(0), get_width(512, gw), 3, 2, 1, "model.5");
    auto c3_6 = C3(network, weightMap, *conv5->getOutput(0), get_width(512, gw), get_width(512, gw), get_depth(9, gd),
                   true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *c3_6->getOutput(0), get_width(768, gw), 3, 2, 1, "model.7");
    auto c3_8 = C3(network, weightMap, *conv7->getOutput(0), get_width(768, gw), get_width(768, gw), get_depth(3, gd),
                   true, 1, 0.5, "model.8");
    auto conv9 = convBlock(network, weightMap, *c3_8->getOutput(0), get_width(1024, gw), 3, 2, 1, "model.9");
    auto c3_10 = C3(network, weightMap, *conv9->getOutput(0), get_width(1024, gw), get_width(1024, gw),
                    get_depth(3, gd), true, 1, 0.5, "model.10");
    auto sppf11 = SPPF(network, weightMap, *c3_10->getOutput(0), get_width(1024, gw), get_width(1024, gw), 5,
                       "model.11");

    /* ------ yolov5 head ------ */
    auto conv12 = convBlock(network, weightMap, *sppf11->getOutput(0), get_width(768, gw), 1, 1, 1, "model.12");
    auto upsample13 = network->addResize(*conv12->getOutput(0));
    assert(upsample13);
    upsample13->setResizeMode(ResizeMode::kNEAREST);
    upsample13->setOutputDimensions(c3_8->getOutput(0)->getDimensions());
    ITensor *inputTensors14[] = {upsample13->getOutput(0), c3_8->getOutput(0)};
    auto cat14 = network->addConcatenation(inputTensors14, 2);
    auto c3_15 = C3(network, weightMap, *cat14->getOutput(0), get_width(1536, gw), get_width(768, gw), get_depth(3, gd),
                    false, 1, 0.5, "model.15");

    auto conv16 = convBlock(network, weightMap, *c3_15->getOutput(0), get_width(512, gw), 1, 1, 1, "model.16");
    auto upsample17 = network->addResize(*conv16->getOutput(0));
    assert(upsample17);
    upsample17->setResizeMode(ResizeMode::kNEAREST);
    upsample17->setOutputDimensions(c3_6->getOutput(0)->getDimensions());
    ITensor *inputTensors18[] = {upsample17->getOutput(0), c3_6->getOutput(0)};
    auto cat18 = network->addConcatenation(inputTensors18, 2);
    auto c3_19 = C3(network, weightMap, *cat18->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd),
                    false, 1, 0.5, "model.19");

    auto conv20 = convBlock(network, weightMap, *c3_19->getOutput(0), get_width(256, gw), 1, 1, 1, "model.20");
    auto upsample21 = network->addResize(*conv20->getOutput(0));
    assert(upsample21);
    upsample21->setResizeMode(ResizeMode::kNEAREST);
    upsample21->setOutputDimensions(c3_4->getOutput(0)->getDimensions());
    ITensor *inputTensors21[] = {upsample21->getOutput(0), c3_4->getOutput(0)};
    auto cat22 = network->addConcatenation(inputTensors21, 2);
    auto c3_23 = C3(network, weightMap, *cat22->getOutput(0), get_width(512, gw), get_width(256, gw), get_depth(3, gd),
                    false, 1, 0.5, "model.23");

    auto conv24 = convBlock(network, weightMap, *c3_23->getOutput(0), get_width(256, gw), 3, 2, 1, "model.24");
    ITensor *inputTensors25[] = {conv24->getOutput(0), conv20->getOutput(0)};
    auto cat25 = network->addConcatenation(inputTensors25, 2);
    auto c3_26 = C3(network, weightMap, *cat25->getOutput(0), get_width(1024, gw), get_width(512, gw), get_depth(3, gd),
                    false, 1, 0.5, "model.26");

    auto conv27 = convBlock(network, weightMap, *c3_26->getOutput(0), get_width(512, gw), 3, 2, 1, "model.27");
    ITensor *inputTensors28[] = {conv27->getOutput(0), conv16->getOutput(0)};
    auto cat28 = network->addConcatenation(inputTensors28, 2);
    auto c3_29 = C3(network, weightMap, *cat28->getOutput(0), get_width(1536, gw), get_width(768, gw), get_depth(3, gd),
                    false, 1, 0.5, "model.29");

    auto conv30 = convBlock(network, weightMap, *c3_29->getOutput(0), get_width(768, gw), 3, 2, 1, "model.30");
    ITensor *inputTensors31[] = {conv30->getOutput(0), conv12->getOutput(0)};
    auto cat31 = network->addConcatenation(inputTensors31, 2);
    auto c3_32 = C3(network, weightMap, *cat31->getOutput(0), get_width(2048, gw), get_width(1024, gw),
                    get_depth(3, gd), false, 1, 0.5, "model.32");

    /* ------ detect ------ */
    IConvolutionLayer *det0 = network->addConvolutionNd(*c3_23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{1, 1},
                                                        weightMap["model.33.m.0.weight"],
                                                        weightMap["model.33.m.0.bias"]);
    IConvolutionLayer *det1 = network->addConvolutionNd(*c3_26->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{1, 1},
                                                        weightMap["model.33.m.1.weight"],
                                                        weightMap["model.33.m.1.bias"]);
    IConvolutionLayer *det2 = network->addConvolutionNd(*c3_29->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{1, 1},
                                                        weightMap["model.33.m.2.weight"],
                                                        weightMap["model.33.m.2.bias"]);
    IConvolutionLayer *det3 = network->addConvolutionNd(*c3_32->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{1, 1},
                                                        weightMap["model.33.m.3.weight"],
                                                        weightMap["model.33.m.3.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, "model.33", std::vector<IConvolutionLayer *>{det0, det1, det2, det3});
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2* calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto &mem: weightMap) {
        free((void *) (mem.second.values));
    }

    return engine;
}

void APIToModel(unsigned int maxBatchSize, IHostMemory **modelStream, bool &is_p6, float &gd, float &gw,
                std::string &wts_name) {
    // Create builder
    IBuilder *builder = createInferBuilder(gLogger);
    IBuilderConfig *config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6) {
        engine = build_engine_p6(maxBatchSize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    } else {
        engine = build_engine(maxBatchSize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    }
    assert(engine != nullptr);

    // Serialize the engine
    (*modelStream) = engine->serialize();

    // Close everything down
    engine->destroy();
    builder->destroy();
    config->destroy();
}

void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *output, int batchSize) {
    // infer on the batch asynchronously, and DMA output back to host
    //CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);

    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost,
                               stream));
    cudaStreamSynchronize(stream);
}

bool parse_args(int argc, char **argv, std::string &wts, std::string &engine, bool &is_p6, float &gd, float &gw,
                std::string &img_dir) {
    if (argc < 4) return false;
    if (std::string(argv[1]) == "-s" && (argc == 5 || argc == 7)) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        auto net = std::string(argv[4]);
        if (net[0] == 'n') {
            gd = 0.33;
            gw = 0.25;
        } else if (net[0] == 's') {
            gd = 0.33;
            gw = 0.50;
        } else if (net[0] == 'm') {
            gd = 0.67;
            gw = 0.75;
        } else if (net[0] == 'l') {
            gd = 1.0;
            gw = 1.0;
        } else if (net[0] == 'x') {
            gd = 1.33;
            gw = 1.25;
        } else if (net[0] == 'c' && argc == 7) {
            gd = atof(argv[5]);
            gw = atof(argv[6]);
        } else {
            return false;
        }
        if (net.size() == 2 && net[1] == '6') {
            is_p6 = true;
        }
    } else if (std::string(argv[1]) == "-d" && argc == 4) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
    } else {
        return false;
    }
    return true;
}


radar_msgs::points
rect2msg(std::vector<Yolo::Detection>::iterator it, cv::Mat &img) {
    radar_msgs::points msg_it;
    cv::Rect r = get_rect(img, it->bbox);
    msg_it.id = it->class_id;
    if (msg_it.id == 0)msg_it.color = "red";
    else msg_it.color = "blue";
    std::vector<radar_msgs::point> rect_point(2);
    rect_point[0].x = (float) r.x;
    rect_point[0].y = (float) r.y;
    rect_point[1].x = (float) (r.x + r.width);
    rect_point[1].y = (float) (r.y + r.height);
    msg_it.data = rect_point;
    std::cout << r << std::endl;
    //draw rectangles on img
    cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 8);
    cv::putText(img, std::to_string((int) it->class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 2.2,
                cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    return msg_it;
}

int main(int argc, char **argv) {
    cudaSetDevice(DEVICE);

    std::string wts_name = "";
    std::string engine_name = std::string(PACK_PATH) + "/yolov5s.engine";

    // bool is_p6 = false;
    // float gd = 0.0f, gw = 0.0f;

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
    far_rectangles = n.advertise<radar_msgs::points>("far_rectangles", 20);
    close_rectangles = n.advertise<radar_msgs::points>("close_rectangles", 20);
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
    for (int i = 0; i < res.size(); i++) {
        if (((int) res[i].class_id) != btl_number) {
            new_res.push_back(res[i]);
        }
    }
    //将识别得到的目标框出并发送ROS消息
    if (new_res.size() != 0) {
        for (std::vector<Yolo::Detection>::iterator it = new_res.begin(); it != new_res.end(); it++) {
            radar_msgs::points rect_msg = rect2msg(it, img);
            if (new_res.size() == 1) {
                rect_msg.text = "far_first_and_last";
            } else if (it == new_res.begin())
                rect_msg.text = "far_first";
            else if (it == new_res.end() - 1)
                rect_msg.text = "far_last";
            else
                rect_msg.text = "far";
            far_rectangles.publish(rect_msg);
        }
    } else {
        radar_msgs::points rect_msg;
        rect_msg.text = "none";
        far_rectangles.publish(rect_msg);
    }
//    std::cout << res.size() << std::endl;
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
    for (int i = 0; i < res.size(); i++) {
        if (((int) res[i].class_id) != btl_number) {
            new_res.push_back(res[i]);
        }
    }
    //将识别得到的目标框出并发送ROS消息
    if (new_res.size() != 0) {
        for (std::vector<Yolo::Detection>::iterator it = new_res.begin(); it != new_res.end(); it++) {
            radar_msgs::points rect_msg = rect2msg(it, img);
            if (new_res.size() == 1) {
                rect_msg.text = "close_first_and_last";
            } else if (it == new_res.begin())
                rect_msg.text = "close_first";
            else if (it == new_res.end() - 1)
                rect_msg.text = "close_last";
            else
                rect_msg.text = "close";
            close_rectangles.publish(rect_msg);
        }
    } else {
        radar_msgs::points rect_msg;
        rect_msg.text = "none";
        close_rectangles.publish(rect_msg);
    }

//    std::cout << res.size() << std::endl;
    cv::resize(img, img, cv::Size(640, 512));
    cv::imshow("yolo_close", img);
    cv::waitKey(1);
}

