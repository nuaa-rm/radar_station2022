#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <radar_msgs/points.h>
#include <radar_msgs/point.h>
#include <radar_msgs/dist_points.h>

using namespace std;
using namespace cv;
cv::Mat img;

double Ky_right = 0.41;
double C_right = 550.0;
double Ky_left = -0.507;
double C_left = -200.53;//x+Ky_left*y-C_left;
int field_width = 28, field_height = 15;
double imgCols = 1280.0, imgRows = 1024.0;
ros::Publisher worldPointPub;
int red_or_blue = 0;//0 is red, 1 is blue

vector<cv::Point3d> far_objectPoints(4);
vector<cv::Point2d> far_imagePoints(4);
cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat far_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat far_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat far_R = Mat::eye(3, 3, CV_64FC1);
Mat far_T = Mat::zeros(3, 1, CV_64FC1);
int far_calc_flag = 0;

vector<cv::Point3d> close_objectPoints(4);
vector<cv::Point2d> close_imagePoints(4);
cv::Mat close_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat close_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat close_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat close_R = Mat::eye(3, 3, CV_64FC1);
Mat close_T = Mat::zeros(3, 1, CV_64FC1);
int close_calc_flag = 0;
vector<radar_msgs::points> far_points;
vector<radar_msgs::points> close_points;
int X_shift = 0;
int Y_shift = 0;
vector<Point> our_R1 = {Point(0, 395), Point(0, 562), Point(33, 562), Point(33, 395)};
vector<Point> our_R2 = {Point(76, 511), Point(160, 569), Point(247, 569), Point(259, 562), Point(235, 532),
                        Point(172, 530), Point(100, 477)};
vector<Point> our_R3 = {Point(0, 572), Point(0, 705), Point(157, 705), Point(157, 654), Point(31, 572)};
vector<Point> our_dafu = {Point(370, 558), Point(370, 609), Point(416, 609), Point(416, 558)};
vector<Point> our_highway = {Point(415, 464), Point(415, 644), Point(450, 644), Point(450, 464)};
vector<vector<Point>> our_warn_regions;
vector<vector<Point>> enemy_warn_regions;
string our_region_names[5] = {"our_R1", "our_R2", "our_R3", "our_dafu", "our_highway"};
string enemy_region_names[5] = {"enemy_R1", "enemy_R2", "enemy_R3", "enemy_dafu", "enemy_highway"};

void onMouse(int event, int x, int y, int flags, void *userdata);//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
void far_calibration(const radar_msgs::points &msg);//相机标定
void far_distPointCallback(const radar_msgs::dist_points &input);

void close_calibration(const radar_msgs::points &msg);//相机标定
void close_distPointCallback(const radar_msgs::dist_points &input);

void draw_point_on_map(const radar_msgs::points &points, Mat &image);

vector<Point> transfer_region(const vector<Point> &input);

void draw_warn_region(Mat &image, const vector<vector<Point>> &our_regions, const vector<vector<Point>> &enemy_regions);

void warn_on_map(const radar_msgs::points &point, Mat &image);

int main(int argc, char **argv) {
    /*预警区域多边形角点初始化*/
    our_warn_regions.emplace_back(our_R1);
    our_warn_regions.emplace_back(our_R2);
    our_warn_regions.emplace_back(our_R3);
    our_warn_regions.emplace_back(our_dafu);
    our_warn_regions.emplace_back(our_highway);
    for (const vector<Point> &points: our_warn_regions) {
        enemy_warn_regions.emplace_back(transfer_region(points));
    }

    /*solvePnP求解相机外参矩阵*/
    ros::init(argc, argv, "small_map");
    ros::NodeHandle n;

    /*小地图解算偏移量*/
    ros::param::get("/small_map/small_map_shift_X", X_shift);
    ros::param::get("/small_map/small_map_shift_Y", Y_shift);
    /*相机标定 Camera Calibration*/
    ros::param::get("/sensor_far/point1/x", far_objectPoints[0].x);
    ros::param::get("/sensor_far/point1/y", far_objectPoints[0].y);
    ros::param::get("/sensor_far/point1/z", far_objectPoints[0].z);
    ros::param::get("/sensor_far/point2/x", far_objectPoints[1].x);
    ros::param::get("/sensor_far/point2/y", far_objectPoints[1].y);
    ros::param::get("/sensor_far/point2/z", far_objectPoints[1].z);
    ros::param::get("/sensor_far/point3/x", far_objectPoints[2].x);
    ros::param::get("/sensor_far/point3/y", far_objectPoints[2].y);
    ros::param::get("/sensor_far/point3/z", far_objectPoints[2].z);
    ros::param::get("/sensor_far/point4/x", far_objectPoints[3].x);
    ros::param::get("/sensor_far/point4/y", far_objectPoints[3].y);
    ros::param::get("/sensor_far/point4/z", far_objectPoints[3].z);
    ros::param::get("/sensor_close/point1/x", close_objectPoints[0].x);
    ros::param::get("/sensor_close/point1/y", close_objectPoints[0].y);
    ros::param::get("/sensor_close/point1/z", close_objectPoints[0].z);
    ros::param::get("/sensor_close/point2/x", close_objectPoints[1].x);
    ros::param::get("/sensor_close/point2/y", close_objectPoints[1].y);
    ros::param::get("/sensor_close/point2/z", close_objectPoints[1].z);
    ros::param::get("/sensor_close/point3/x", close_objectPoints[2].x);
    ros::param::get("/sensor_close/point3/y", close_objectPoints[2].y);
    ros::param::get("/sensor_close/point3/z", close_objectPoints[2].z);
    ros::param::get("/sensor_close/point4/x", close_objectPoints[3].x);
    ros::param::get("/sensor_close/point4/y", close_objectPoints[3].y);
    ros::param::get("/sensor_close/point4/z", close_objectPoints[3].z);
    string btlcolor;
    ros::param::get("/battle_state/battle_color", btlcolor);
    if (btlcolor == "red")red_or_blue = 0;
    if (btlcolor == "blue") red_or_blue = 1;

    ros::param::get("/sensor_far/camera_matrix/zerozero", far_CamMatrix_.at<double>(0, 0));
    ros::param::get("/sensor_far/camera_matrix/zerotwo", far_CamMatrix_.at<double>(0, 2));
    ros::param::get("/sensor_far/camera_matrix/oneone", far_CamMatrix_.at<double>(1, 1));
    ros::param::get("/sensor_far/camera_matrix/onetwo", far_CamMatrix_.at<double>(1, 2));
    ros::param::get("/sensor_far/camera_matrix/twotwo", far_CamMatrix_.at<double>(2, 2));
    far_CamMatrix_.at<double>(0, 1) = 0;
    far_CamMatrix_.at<double>(1, 0) = 0;
    far_CamMatrix_.at<double>(2, 0) = 0;
    far_CamMatrix_.at<double>(2, 1) = 0;
    cout << far_CamMatrix_ << endl;
    ros::param::get("/sensor_far/distortion_coefficient/zero", far_distCoeffs_.at<double>(0, 0));
    ros::param::get("/sensor_far/distortion_coefficient/one", far_distCoeffs_.at<double>(1, 0));
    ros::param::get("/sensor_far/distortion_coefficient/two", far_distCoeffs_.at<double>(2, 0));
    ros::param::get("/sensor_far/distortion_coefficient/three", far_distCoeffs_.at<double>(3, 0));
    ros::param::get("/sensor_far/distortion_coefficient/four", far_distCoeffs_.at<double>(4, 0));
    cout << far_distCoeffs_ << endl;

    ros::param::get("/sensor_close/camera_matrix/zerozero", close_CamMatrix_.at<double>(0, 0));
    ros::param::get("/sensor_close/camera_matrix/zerotwo", close_CamMatrix_.at<double>(0, 2));
    ros::param::get("/sensor_close/camera_matrix/oneone", close_CamMatrix_.at<double>(1, 1));
    ros::param::get("/sensor_close/camera_matrix/onetwo", close_CamMatrix_.at<double>(1, 2));
    ros::param::get("/sensor_close/camera_matrix/twotwo", close_CamMatrix_.at<double>(2, 2));
    close_CamMatrix_.at<double>(0, 1) = 0;
    close_CamMatrix_.at<double>(1, 0) = 0;
    close_CamMatrix_.at<double>(2, 0) = 0;
    close_CamMatrix_.at<double>(2, 1) = 0;
    cout << close_CamMatrix_ << endl;
    ros::param::get("/sensor_close/distortion_coefficient/zero", close_distCoeffs_.at<double>(0, 0));
    ros::param::get("/sensor_close/distortion_coefficient/one", close_distCoeffs_.at<double>(1, 0));
    ros::param::get("/sensor_close/distortion_coefficient/two", close_distCoeffs_.at<double>(2, 0));
    ros::param::get("/sensor_close/distortion_coefficient/three", close_distCoeffs_.at<double>(3, 0));
    ros::param::get("/sensor_close/distortion_coefficient/four", close_distCoeffs_.at<double>(4, 0));
    cout << close_distCoeffs_ << endl;

    ros::Subscriber far_imageSub = n.subscribe("/sensor_far/calibration", 1, &far_calibration);
    ros::Subscriber far_distPointSub = n.subscribe("/sensor_far/distance_point", 1, &far_distPointCallback);
    ros::Subscriber close_imageSub = n.subscribe("/sensor_close/calibration", 1, &close_calibration);
    ros::Subscriber close_distPointSub = n.subscribe("/sensor_close/distance_point", 1, &close_distPointCallback);
    worldPointPub = n.advertise<radar_msgs::points>("/world_point", 50);
    ros::Rate loop_rate(20);
    Mat small_map;
    if (red_or_blue == 0)small_map = imread("/home/chris/radar_station2022/src/small_map/src/red_minimap.png");
    else small_map = imread("/home/chris/radar_station2022/src/small_map/src/blue_minimap.png");
    resize(small_map, small_map, Size(450, 840));
    Mat small_map_copy;
    small_map.copyTo(small_map_copy);
    namedWindow("small_map");
    setMouseCallback("small_map", onMouse, &small_map_copy);
    while (ros::ok()) {
        ros::spinOnce();
        small_map.copyTo(small_map_copy);
        draw_warn_region(small_map_copy, our_warn_regions, enemy_warn_regions);
        for (int i = 0; i < far_points.size(); i++) {
            warn_on_map(far_points[i], small_map_copy);
            draw_point_on_map(far_points[i], small_map_copy);
            worldPointPub.publish(far_points[i]);
        }
//        for (int i = 0; i < close_points.size(); i++) {
//            double x = close_points[i].data[0].x;
//            double y = close_points[i].data[0].y;
//            x *= 450;
//            y *= 840;
//            if (x + Ky_right * y - C_right > 0 || x + Ky_left * y - C_left < 0) {
//                if (close_points[i].color == "blue") {
//                    circle(small_map, Point((int) x-15,
//                                                 (int) y), 10,
//                           Scalar(255, 0, 0), -1, LINE_8, 0);
//                    if(close_points[i].id!=12&&close_points[i].id!=13)putText(small_map, to_string(close_points[i].id),
//                            Point((int) (450 * close_points[i].data[0].x) - 22,
//                                  (int) (840 * close_points[i].data[0].y) +7), cv::FONT_HERSHEY_SIMPLEX, 0.7,
//                            cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//                } else if (close_points[i].color == "red") {
//                    circle(small_map, Point((int) x-15,
//                                                 (int) y), 10,
//                           Scalar(0, 0, 255), -1, LINE_8, 0);
//                    if(close_points[i].id!=12&&close_points[i].id!=13)putText(small_map, to_string(close_points[i].id),
//                            Point((int) (450 * close_points[i].data[0].x) - 22,
//                                  (int) (840 * close_points[i].data[0].y) +7), cv::FONT_HERSHEY_SIMPLEX, 0.7,
//                            cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//                }
//                worldPointPub.publish(close_points[i]);
//            }
//        }
        imshow("small_map", small_map_copy);
        waitKey(1);
        loop_rate.sleep();
    }
    return 0;
}

void onMouse(int event, int x, int y, int flags, void *userdata)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
    if (event == cv::EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆
    {
        circle(*(Mat *) userdata, Point2i(x, y), 5, Scalar(0x27, 0xc1, 0x36), -1);
        int is_in_poly = pointPolygonTest(our_R3, Point2f(x, y), false);
        cout << is_in_poly << "  ";
        if (is_in_poly > 0)cout << "in" << endl;
        else if (is_in_poly < 0)cout << "out" << endl;
        else if (is_in_poly == 0)cout << "edge" << endl;
        imshow("small_map", *(Mat *) userdata);
        cout << Point(x, y) << endl;
    }
}

void warn_on_map(const radar_msgs::points &point, Mat &image) {
    Scalar light_green = Scalar(0xcc, 0xff, 0xcc);
    for (int i = 0; i < our_warn_regions.size(); i++) {
        if (pointPolygonTest(our_warn_regions[i],
                             Point(450 * point.data[0].x - X_shift, 840 * point.data[0].y - Y_shift),
                             false) > 0) {
            drawContours(image, our_warn_regions, i, light_green, -1);
        }
        if (pointPolygonTest(enemy_warn_regions[i],
                             Point(450 * point.data[0].x - X_shift, 840 * point.data[0].y - Y_shift), false) > 0) {
            drawContours(image, enemy_warn_regions, i, light_green, -1);
        }
    }
}

void draw_point_on_map(const radar_msgs::points &points, Mat &image) {
    Scalar scalar;
    string id;
    if (points.color == "red")scalar = Scalar(0, 0, 255);
    else if (points.color == "blue")scalar = Scalar(255, 0, 0);
    circle(image, Point((int) (450 * points.data[0].x - X_shift),
                        (int) (840 * points.data[0].y - Y_shift)), 10,
           scalar, -1, LINE_8, 0);
    if (points.id != 12 && points.id != 13) {
        if (points.id <= 5)id = to_string(points.id + 1);
        if (points.id == 5)id = "G";
        if (points.id >= 6)id = to_string(points.id - 5);
        if (points.id == 11)id = "G";
        putText(image, id,
                Point((int) (450 * points.data[0].x) - X_shift - 7,
                      (int) (840 * points.data[0].y - Y_shift) + 7), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
}

vector<Point> transfer_region(const vector<Point> &input) {
    vector<Point> enemy_region;
    for (const Point &point: input) {
        enemy_region.emplace_back(450 - point.x, 840 - point.y);
    }
    return enemy_region;
}

void
draw_warn_region(Mat &image, const vector<vector<Point>> &our_regions, const vector<vector<Point>> &enemy_regions) {
    Scalar our_scalar;
    Scalar enemy_scalar;
    if (red_or_blue == 0) {
        our_scalar = Scalar(0, 0, 255);
        enemy_scalar = Scalar(255, 0, 0);
    } else {
        enemy_scalar = Scalar(0, 0, 255);
        our_scalar = Scalar(255, 0, 0);
    }
    for (const vector<Point> &region: our_regions) {
        polylines(image, our_regions, 1, our_scalar, 2);
    }
    for (const vector<Point> &region: enemy_regions) {
        polylines(image, enemy_regions, 1, enemy_scalar, 2);
    }
}

void far_distPointCallback(const radar_msgs::dist_points &input) {
    if (far_calc_flag == 1) {
        Mat invR;
        Mat invM;
        invert(far_CamMatrix_, invM);
        invert(far_R, invR);
        std::vector<radar_msgs::points>().swap(far_points);
        for (int i = 0; i < input.data.size(); i++) {
            if (input.data[i].dist > 0) {
                Mat x8_pixel;
                x8_pixel = (Mat_<double>(3, 1) << (double) input.data[i].x, (double) input.data[i].y, 1);
                x8_pixel *= (1000 * input.data[i].dist);
                Mat calcWorld = invR * (invM * x8_pixel - far_T);//2D-3D变换
                calcWorld /= 1000;
                double x = calcWorld.at<double>(0, 0);
                double y = calcWorld.at<double>(1, 0);
                y = field_width - y;
                x /= field_height;
                y /= field_width;
                radar_msgs::point point;
                radar_msgs::points points;
                point.x = x;
                point.y = y;
                if (input.data[i].id == 5) {
                    point.y = 0.80952;
//                    cout << "far 5:" << input.data[i].dist << endl;
                }
                points.data.push_back(point);
                points.id = input.data[i].id;
                if (input.data[i].color == 0)points.color = string("red");
                else points.color = string("blue");
                far_points.push_back(points);
            }
        }
    }
}

void far_calibration(const radar_msgs::points &msg) {
    for (const auto &abc: msg.data) {
        far_imagePoints[abc.id] = cv::Point2d(abc.x, abc.y);
        far_imagePoints[abc.id].x *= imgCols;
        far_imagePoints[abc.id].y *= imgRows;
        cout << far_imagePoints[abc.id] << endl;
    }
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat abc;
    int suc = cv::solvePnPRansac(far_objectPoints, far_imagePoints, far_CamMatrix_, far_distCoeffs_, far_Rjacob,
                                 far_T,
                                 false, 100, 8.0, 0.99,
                                 abc, cv::SOLVEPNP_AP3P);
    Rodrigues(far_Rjacob, far_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << far_R << endl;
    cout << "平移矩阵" << far_T << endl;
    far_calc_flag = 1;
}

void close_distPointCallback(const radar_msgs::dist_points &input) {
    if (close_calc_flag == 1) {
        Mat invR;
        Mat invM;
        invert(close_CamMatrix_, invM);
        invert(close_R, invR);
        std::vector<radar_msgs::points>().swap(close_points);
        for (int i = 0; i < input.data.size(); i++) {
            if (input.data[i].dist > 0) {
                Mat x8_pixel;
                x8_pixel = (Mat_<double>(3, 1) << (double) input.data[i].x, (double) input.data[i].y, 1);
                x8_pixel *= (1000 * input.data[i].dist);
                Mat calcWorld = invR * (invM * x8_pixel - close_T);//2D-3D变换
                calcWorld /= 1000;
                double x = calcWorld.at<double>(0, 0);
                double y = calcWorld.at<double>(1, 0);
                y = field_width - y;
                x /= field_height;
                y /= field_width;
                radar_msgs::point point;
                radar_msgs::points points;
                point.x = x;
                point.y = y;
                points.data.push_back(point);
                points.id = input.data[i].id;
                if (input.data[i].color == 0)points.color = string("red");
                else points.color = string("blue");
                close_points.push_back(points);
            }
        }
    }
}

void close_calibration(const radar_msgs::points &msg) {
    for (const auto &abc: msg.data) {
        close_imagePoints[abc.id] = cv::Point2d(abc.x, abc.y);
        close_imagePoints[abc.id].x *= imgCols;
        close_imagePoints[abc.id].y *= imgRows;
        cout << close_imagePoints[abc.id] << endl;
    }
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat abc;
    int suc = cv::solvePnPRansac(close_objectPoints, close_imagePoints, close_CamMatrix_, close_distCoeffs_,
                                 close_Rjacob, close_T, false, 100, 8.0, 0.99,
                                 abc, cv::SOLVEPNP_AP3P);
    Rodrigues(close_Rjacob, close_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << close_R << endl;
    cout << "平移矩阵" << close_T << endl;
    close_calc_flag = 1;
}
