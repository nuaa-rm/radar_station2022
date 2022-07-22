#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <radar_msgs/points.h>
#include <radar_msgs/point.h>
#include <radar_msgs/dist_points.h>
#include <bitset>

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
radar_msgs::points far_points;
radar_msgs::points close_points;
radar_msgs::points result_points;
radar_msgs::points relative_coordinates;
radar_msgs::points pub_relative;
Point2f our_guard;
Point2f enemy_guard;
int X_shift = 0;
int Y_shift = 0;
vector<Point> our_R1 = {Point(0, 395), Point(0, 562), Point(33, 562), Point(33, 395)};
vector<Point> our_R2 = {Point(76, 511), Point(160, 569), Point(247, 569), Point(259, 562), Point(235, 532),
                        Point(172, 530), Point(100, 477)};
vector<Point> our_R3 = {Point(0, 572), Point(0, 705), Point(157, 705), Point(157, 654), Point(31, 572)};
vector<Point> our_dafu = {Point(370, 558), Point(370, 609), Point(416, 609), Point(416, 558)};
vector<Point> our_highway = {Point(415, 464), Point(415, 644), Point(450, 644), Point(450, 464)};
vector<Point> our_outpost = {Point(414, 558), Point(414, 445), Point(317, 445), Point(317, 558)};
vector<Point> enemy_highway = {Point(35, 376), Point(35, 196), Point(0, 196), Point(0, 376)};
vector<Point> enemy_dafu = {Point(80, 282), Point(80, 231), Point(34, 231), Point(34, 282)};
vector<Point> enemy_outpost = {Point(36, 282), Point(36, 395), Point(133, 395), Point(133, 282)};
vector<Point> enemy_hero_hide = {Point(417, 333), Point(417, 445), Point(450, 445), Point(450, 333)};
vector<Point> enemy_R3 = {Point(450, 268), Point(450, 135), Point(293, 135), Point(293, 186), Point(419, 268)};
vector<vector<Point>> our_warn_regions;
vector<vector<Point>> enemy_warn_regions;
uint16_t warn_region_state = 0x0000;
string our_region_names[6] = {"our_R1", "our_R2", "our_R3", "our_dafu", "our_highway", "our_outpost"};
string enemy_region_names[5] = {"enemy_highway", "enemy_dafu", "enemy_outpost", "enemy_hero_hide", "enemy_R3"};

void onMouse(int event, int x, int y, int flags, void *userdata);//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
void far_calibration(const radar_msgs::points &msg);//相机标定
void far_distPointCallback(const radar_msgs::dist_points &input);

void close_calibration(const radar_msgs::points &msg);//相机标定
void close_distPointCallback(const radar_msgs::dist_points &input);

void draw_point_on_map(const radar_msgs::point &point, Mat &image);

void remove_duplicate();

void draw_warn_region(Mat &image, const vector<vector<Point>> &our_regions, const vector<vector<Point>> &enemy_regions);

void warn_on_map(const radar_msgs::points &points, Mat &image);

radar_msgs::point calculate_relative_codi(const Point2f &guard, const radar_msgs::point &enemy, uint8_t priority_id);

Point2f calculate_pixel_codi(const radar_msgs::point &point);

int main(int argc, char **argv) {
    our_guard.x = 9200;
    our_guard.y = 5333.44;
    enemy_guard.y = 22666.56;
    /*预警区域多边形角点初始化*/
    our_warn_regions.emplace_back(our_R1);
    our_warn_regions.emplace_back(our_R2);
    our_warn_regions.emplace_back(our_R3);
    our_warn_regions.emplace_back(our_dafu);
    our_warn_regions.emplace_back(our_highway);
    our_warn_regions.emplace_back(our_outpost);
    enemy_warn_regions.emplace_back(enemy_highway);
    enemy_warn_regions.emplace_back(enemy_dafu);
    enemy_warn_regions.emplace_back(enemy_outpost);
    enemy_warn_regions.emplace_back(enemy_hero_hide);
    enemy_warn_regions.emplace_back(enemy_R3);
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
    worldPointPub = n.advertise<radar_msgs::points>("/world_point", 10);
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
        if (close_calc_flag&&far_calc_flag)
        {
            remove_duplicate();
            warn_on_map(result_points,small_map_copy);
            for (auto & i : result_points.data) {
                draw_point_on_map(i, small_map_copy);
            }
//            for (auto & i : close_points.data) {
//                draw_point_on_map(i, small_map_copy);
//            }
//            for (auto & i : far_points.data) {
//                draw_point_on_map(i, small_map_copy);
//            }
        }
        if (!pub_relative.data.empty()) {
            worldPointPub.publish(pub_relative);
        }
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

double Point2PointDist(const radar_msgs::point &a, const Point2f &b) {
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

double calculate_dist(const radar_msgs::point &a, const radar_msgs::point &b) {
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

bool is_enemy_car(uint8_t id) {
    if (red_or_blue == 0) {
        if ((id >= 6 && id <= 11) || id == 13)
            return true;
        else return false;
    } else {
        if (id <= 5 || id == 12)
            return true;
        else return false;
    }
}

void remove_duplicate() {
    vector<radar_msgs::point>().swap(result_points.data);
    radar_msgs::points red_no_id_cars;
    radar_msgs::points blue_no_id_cars;
    radar_msgs::points left_may_overlap_points;
    radar_msgs::points right_may_overlap_points;
    vector<Point2f> left_region = {Point(0, 0), Point(0, 840), Point(200, 840), Point(200, 0)};
    vector<Point2f> right_region = {Point(255, 0), Point(255, 840), Point(450, 840), Point(450, 0)};
    for (auto &i: far_points.data) {
        int test = pointPolygonTest(left_region, calculate_pixel_codi(i), false);
        int right_test = pointPolygonTest(right_region, calculate_pixel_codi(i), false);
        if (test > 0) {
            result_points.data.emplace_back(i);
        } else if (test == 0 && i.x != 200) {
            result_points.data.emplace_back(i);
        } else if(-pointPolygonTest(right_region, calculate_pixel_codi(i),false)){
            left_may_overlap_points.data.emplace_back(i);
        }
    }
    for (auto &i: close_points.data) {
        int test = pointPolygonTest(right_region, calculate_pixel_codi(i), false);
        if (test > 0) {
            result_points.data.emplace_back(i);
        } else if (test == 0 && i.x != 255) {
            result_points.data.emplace_back(i);
        } else if(-pointPolygonTest(left_region, calculate_pixel_codi(i), false)){
            right_may_overlap_points.data.emplace_back(i);
        }
    }
    for (auto it_left = left_may_overlap_points.data.begin(); it_left < left_may_overlap_points.data.end();) {
        for (auto it_right = right_may_overlap_points.data.begin(); it_right < right_may_overlap_points.data.end();) {
            if (it_left->id == it_right->id && it_left->id == 12 && calculate_dist(*it_left, *it_right) < 5) {
                radar_msgs::point center;
                center.id = 12;
                center.x = (it_left->x + it_right->x) / 2;
                center.y = (it_left->y + it_right->y) / 2;
                result_points.data.emplace_back(center);
                left_may_overlap_points.data.erase(it_left);
                right_may_overlap_points.data.erase(it_right);
                continue;
            }
            if (it_left->id == it_right->id && it_left->id == 13 && calculate_dist(*it_left, *it_right) < 5) {
                radar_msgs::point center;
                center.id = 13;
                center.x = (it_left->x + it_right->x) / 2;
                center.y = (it_left->y + it_right->y) / 2;
                result_points.data.emplace_back(center);
                left_may_overlap_points.data.erase(it_left);
                right_may_overlap_points.data.erase(it_right);
                continue;
            }
            if (it_left->id == it_right->id && it_left->id < 12) {
                radar_msgs::point center;
                center.id = it_left->id;
                center.x = (it_left->x + it_right->x) / 2;
                center.y = (it_left->y + it_right->y) / 2;
                result_points.data.emplace_back(center);
                left_may_overlap_points.data.erase(it_left);
                right_may_overlap_points.data.erase(it_right);
            }
            it_right++;
        }
        it_left++;
    }
    for (auto &i: left_may_overlap_points.data) {
        result_points.data.emplace_back(i);
    }
    for (auto &i: right_may_overlap_points.data) {
        result_points.data.emplace_back(i);
    }
}

void warn_on_map(const radar_msgs::points &points, Mat &image) {
    vector<radar_msgs::point>().swap(pub_relative.data);
    Scalar light_green = Scalar(0xcc, 0xff, 0xcc);
    far_points.id = 0;
    for (int i = 0; i < our_warn_regions.size(); i++) {
        for (radar_msgs::point car: points.data) {
            if (is_enemy_car(car.id) && pointPolygonTest(our_warn_regions[i],
                                                         calculate_pixel_codi(car),
                                                         false) > 0) {
                warn_region_state |= (0x01 << (i + 5));
                relative_coordinates.data.push_back(calculate_relative_codi(our_guard, car, 0));
                drawContours(image, our_warn_regions, i, light_green, -1);
            }
        }
    }
    double nearest = 50.0;
    double dist;
    uint8_t position = 0;
    for (int i = 0; i < relative_coordinates.data.size(); i++) {
        dist = Point2PointDist(relative_coordinates.data[i], our_guard);
        if (dist < nearest) {
            nearest = dist;
            position = (uint8_t) i;
        }
    }
    if (!relative_coordinates.data.empty()) {
        pub_relative.data.emplace_back(relative_coordinates.data[position]);
    }
    for (int i = 0; i < enemy_warn_regions.size(); i++) {
        for (radar_msgs::point car: points.data) {
            if (is_enemy_car(car.id) && pointPolygonTest(enemy_warn_regions[i],
                                                         calculate_pixel_codi(car),
                                                         false) > 0) {
                warn_region_state |= (0x01 << (i));
                drawContours(image, enemy_warn_regions, i, light_green, -1);
                if (red_or_blue == 0) {
                    if (i == 4 && car.id == 6) {
                        pub_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 1));//敌方英雄到达敌方公路区，可能会打前哨站
                    } else if (i == 2 && car.id != 7) {
                        pub_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 2));//敌方车辆到达敌方前哨站(工程除外)
                    }
                } else {
                    if (i == 4 && car.id == 0) {
                        pub_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 1));//敌方英雄到达敌方公路区，可能会打前哨站
                    } else if (i == 2 && car.id != 1) {
                        pub_relative.data.emplace_back(
                                calculate_relative_codi(our_guard, car, 2));//敌方车辆到达敌方前哨站(工程除外)
                    }
                }
            }
        }
    }
}

radar_msgs::point calculate_relative_codi(const Point2f &guard, const radar_msgs::point &enemy, uint8_t priority_id) {
    radar_msgs::point re_codi;
    re_codi.x = enemy.x * 15000 - guard.x;
    re_codi.y = enemy.y * 28000 - guard.y;
    re_codi.id = priority_id;
    return re_codi;
}

Point2f calculate_pixel_codi(const radar_msgs::point &point) {
    Point2f res;
    res.x = point.x * 450 - (float) X_shift;
    res.y = (1 - point.y) * 840 - (float) Y_shift;
    return res;
}

Point2f calculate_pixel_text_codi(const radar_msgs::point &point) {
    Point2f res;
    res.x = point.x * 450 - (float) X_shift - 7;
    res.y = (1 - point.y) * 840 - (float) Y_shift + 7;
    return res;
}

void draw_point_on_map(const radar_msgs::point &point, Mat &image) {
    Scalar scalar;
    string id;
    if (point.id <= 5 || point.id == 12)scalar = Scalar(0, 0, 255);
    else scalar = Scalar(255, 0, 0);
    circle(image, calculate_pixel_codi(point), 10,
           scalar, -1, LINE_8, 0);
    if (point.id != 12 && point.id != 13) {
        if (point.id <= 5)id = to_string(point.id + 1);
        if (point.id == 5)id = "G";
        if (point.id >= 6)id = to_string(point.id - 5);
        if (point.id == 11)id = "G";
        putText(image, id,
                calculate_pixel_text_codi(point), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
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
        std::vector<radar_msgs::point>().swap(far_points.data);
        for (int i = 0; i < input.data.size(); i++) {
            if (input.data[i].dist > 0) {
                Mat x8_pixel;
                x8_pixel = (Mat_<double>(3, 1) << (double) input.data[i].x, (double) input.data[i].y, 1);
                x8_pixel *= (1000 * input.data[i].dist);
                Mat calcWorld = invR * (invM * x8_pixel - far_T);//2D-3D变换
                calcWorld /= 1000;
                double x = calcWorld.at<double>(0, 0);
                double y = calcWorld.at<double>(1, 0);
                x /= field_height;
                y /= field_width;
                radar_msgs::point point;
                point.x = x;
                point.y = y;
                if (red_or_blue == 0) {
                    if (input.data[i].id == 5) {
                        point.y = 0.19048;
                        our_guard.x = x * 15000;
                    }
                    if (input.data[i].id == 11) {
                        point.y = 0.80952;
                        enemy_guard.x = x * 15000;
                    }
                } else {
                    if (input.data[i].id == 11) {
                        point.y = 0.19048;
                        our_guard.x = x * 15000;
                    }
                    if (input.data[i].id == 5) {
                        point.y = 0.80952;
                        enemy_guard.x = x * 15000;
                    }
                }
                point.id = input.data[i].id;
                far_points.data.push_back(point);
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
        std::vector<radar_msgs::point>().swap(close_points.data);
        for (int i = 0; i < input.data.size(); i++) {
            if (input.data[i].dist > 0) {
                Mat x8_pixel;
                x8_pixel = (Mat_<double>(3, 1) << (double) input.data[i].x, (double) input.data[i].y, 1);
                x8_pixel *= (1000 * input.data[i].dist);
                Mat calcWorld = invR * (invM * x8_pixel - close_T);//2D-3D变换
                calcWorld /= 1000;
                double x = calcWorld.at<double>(0, 0);
                double y = calcWorld.at<double>(1, 0);
                x /= field_height;
                y /= field_width;
                radar_msgs::point point;
                point.x = x;
                point.y = y;
                if (red_or_blue == 0) {
                    if (input.data[i].id == 5) {
                        point.y = 0.19048;
                        our_guard.x = x * 15000;
                    }
                    if (input.data[i].id == 11) {
                        point.y = 0.80952;
                        enemy_guard.x = x * 15000;
                    }
                } else {
                    if (input.data[i].id == 11) {
                        point.y = 0.19048;
                        our_guard.x = x * 15000;
                    }
                    if (input.data[i].id == 5) {
                        point.y = 0.80952;
                        enemy_guard.x = x * 15000;
                    }
                }
                point.id = input.data[i].id;
                close_points.data.push_back(point);
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
    cout<<"close obj points:"<<close_objectPoints<<endl;
    int suc = cv::solvePnPRansac(close_objectPoints, close_imagePoints, close_CamMatrix_, close_distCoeffs_,
                                 close_Rjacob, close_T, false, 100, 8.0, 0.99,
                                 abc, cv::SOLVEPNP_AP3P);
    Rodrigues(close_Rjacob, close_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << close_R << endl;
    cout << "平移矩阵" << close_T << endl;
    close_calc_flag = 1;
}

//            Mat Points(far_points.data.size(),1,CV_32FC2);
//            for(int i=0;i<far_points.data.size();i++){
//                Points.at<Point2f>(i,0)=calculate_pixel_codi(far_points.data[i]);;
//            }
//            Mat labels;
//            Mat centers;
//            Scalar colorLut[4]={
//                    Scalar(0,0,255),
//                    Scalar(0,255,0),
//                    Scalar(255,0,0),
//                    Scalar(0,0,0)
//            };
//            kmeans(Points,3,labels,TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,10,0.1),3,KMEANS_PP_CENTERS);
//            Mat mean_img(840,450,CV_32FC3,Scalar(255,255,255));
//            for(int i=0;i<far_points.data.size();i++){
//                int index=labels.at<int>(i);
//                Point point=Points.at<Point2f>(i);
//                circle(mean_img,point,10,colorLut[index],-1,4);
//            }
//            imshow("kmeans",mean_img);
//            waitKey(1);