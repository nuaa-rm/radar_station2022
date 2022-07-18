/**
 * @file /include/displayer_qt5/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef displayer_qt5_QNODE_HPP_
#define displayer_qt5_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>

#include <QListWidgetItem>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace displayer_qt5 {

/*****************************************************************************
** Class
*****************************************************************************/
enum LogLevel {
         Debug,
         Info,
         Warn,
         Error,
         Fatal
 };
struct log_information{
         LogLevel level;
         QString qstring;
};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
    void imgShowCallback(const sensor_msgs::ImageConstPtr& msg);//camera callback function
    void imgSensorFarCallback(const sensor_msgs::ImageConstPtr& msg);
    void imgSensorCloseCallback(const sensor_msgs::ImageConstPtr& msg);
    void pubCelibrateResult();
    QImage image;
    QImage imageSensorFar;
    QImage imageSensorClose;
    QImage imageCalibrateMainWindow;
    QImage imageCalibrateSecondWindow;
    QListWidgetItem *listWidgetItem;
    log_information *logInformation;
	void log( const LogLevel &level, const std::string &msg);
    void loadParams();
    QString sensorFarImgRaw;
    QString sensorCloseImgRaw;
    QString cameraCelibrating;
    QPoint mouseLoaction;
    bool if_is_celibrating;
    QString realsenseImgRaw;
    QPoint sensor_far_points[4];
    QPoint sensor_close_points[4];
    QString calibrationTopicSensorFar;
    QString calibrationTopicSensorClose;

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void loggingCamera();//发出设置相机图片信号
    void loggingCameraCalibrateMainWindow();
    void loggingCameraCalibrateSecondWindow();


private:
	int init_argc;
	char** init_argv;
    image_transport::Subscriber image_sub;
    image_transport::Subscriber image_sub_sensor_far;
    image_transport::Subscriber image_sub_sensor_close;
    ros::Publisher calibration_pub_sensor_far;
    ros::Publisher calibration_pub_sensor_close;
    cv::Mat img;
    cv::Mat imgSensorFar;
    cv::Mat imgSensorClose;
    int calibrateRate;
    int calibrateMainWindowWidth;
    int calibrateMainWindowHeight;
    int calibrateSecondWindowWidth;
    int calibrateSecondWindowHeight;
};

}  // namespace displayer_qt5

#endif /* displayer_qt5_QNODE_HPP_ */
